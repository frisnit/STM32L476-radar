#include "fft.h"
#include "stdlib.h"

#define TEST_LENGTH_SAMPLES 2048

arm_cfft_instance_f32* fftTables[] = {
//		&arm_cfft_sR_f32_len4096,
//		&arm_cfft_sR_f32_len2048,
		&arm_cfft_sR_f32_len1024,
	&arm_cfft_sR_f32_len512,
	&arm_cfft_sR_f32_len256,
	&arm_cfft_sR_f32_len128,
	&arm_cfft_sR_f32_len64,
	&arm_cfft_sR_f32_len32
};


//uint32_t fftSize = FFT_SIZE;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

#define TEST_FREQUENCY	500.0f // HZ

// fmax = SAMPLE_RATE/2 = 5kHz
// 1024 complex samples
// 512 frequency bins
// = 5kHz/512 Hz/bin = 9.7Hz
// test signal at bin 2500/9.7
// = 256

// for our radar:

// detection range 0-1500Hz (0-100mph)
// sample rate at least 3kHz

// 3.67Hz resolution = 0.11m/s @ 10GHz = 0.4km/h = 0.25mph
// so bins must be 3Hz
// FFT must have at least 500 real bins
// FFT size must be 500*2 = 1000 points, make it 1024
// each bin will be 1500/512 Hz = 2.93Hz ~= 0.08m/s
// so sample length must be 1024 points, at 3kHz gives a measurement time of ~300ms

// 6.67Hz resolution = 0.20m/s @ 10GHz = 0.72km/h = 0.45mph
// so bins must be 6Hz
// FFT must have at least 250 real bins
// FFT size must be 250*2 = 500 points, make it 512
// each bin will be 1500/256 Hz = 5.86Hz ~= 0.1758m/s
// so sample length must be 512 points, at 3kHz gives a measurement time of ~150ms



// create some test data, a few sine waves and some noise
void createData(float32_t *buffer)
{
	static float32_t accumulator=0.0f;
	static float32_t frequency = TEST_FREQUENCY;
	uint16_t i;

	float32_t radiansPerSample=(frequency*2.0*M_PI)/(float32_t)SAMPLE_RATE;

	for(i=0;i<TEST_LENGTH_SAMPLES;i+=2)
	{
		buffer[i]=sin(accumulator);// RE
//		buffer[i]+=sin(accumulator*2.0f)/2.0f;// RE
//		buffer[i]+=sin(accumulator*0.5f)*0.8f;// RE
//		buffer[i]+=i&0x100?0.25f:0.00f;// RE

		buffer[i]+=((float32_t)rand()/(float32_t)RAND_MAX);//*5.0f;

		buffer[i+1]=0.0f;// IM

		accumulator+=radiansPerSample;
	}

	frequency+=10.0f;

	if(frequency>1200.0f)
		frequency=100.0f;

}

// convert the integer data buffer from the ADC into complex floating point buffer for FFT processing
void makeComplexBuffer(uint16_t *buffer, float32_t *output, uint16_t fftLength)
{
	uint16_t i;

	for(i=0;i<fftLength*2;i+=2)
	{
		output[i]	= (float32_t)(buffer[i/2]&0x0fff)/4095.0f;
		output[i+1]	= 0.0f;
	}
}

uint16_t getFftLength(uint16_t index)
{
	return fftTables[index]->fftLen;
}

void doFFT(uint16_t *inputBuffer, float32_t *hertz, float32_t *average, float32_t output[],uint32_t *maxIndex, uint16_t fftLengthIndex)
{
	float32_t maxValue;
	uint16_t fftLength = getFftLength(fftLengthIndex);
	float32_t complexBuffer[FFT_MAX_SIZE*2];

	makeComplexBuffer(inputBuffer, complexBuffer, fftLength);
	// createData(complexBuffer);

  /* Process the data through the CFFT/CIFFT module */
	  arm_cfft_f32(fftTables[fftLengthIndex], complexBuffer, ifftFlag, doBitReverse);

  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(complexBuffer, output, fftLength);

  // ignore the DC value
  output[0]=0.0f;


  uint16_t n, top;
  top = 10;///(50*SAMPLE_RATE)/fftLength;


  // squash everything under 100Hz
  for(n=0;n<top;n++)
  {
	  output[n]=0.0f;
  }

  /* Calculates maxValue and returns corresponding BIN value */
  arm_max_f32(output, fftLength/2, &maxValue, maxIndex);

  // calculate frequency value of peak bin
  float32_t nyquistFrequency = SAMPLE_RATE/2;
  float32_t hertzPerBin = nyquistFrequency/((float)fftLength/2);

  *hertz = hertzPerBin*(float32_t)*maxIndex;

  arm_mean_f32(output, fftLength, average);
}


