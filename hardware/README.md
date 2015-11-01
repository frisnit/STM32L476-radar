# STM32L476-radar hardware

Schematic of the radar hardware. It's just a couple of opamp gain stages with the output coupled to the PA0 ADC input of the STM32L476 via a level converter. The 5V supply for the radar and amplifier is passed through a simple filter to clean it up. It isn't the most optimal design by any means but is sufficient for a demonstration

The amplifier is based on an LM358 single supply dual opamp. The Doppler radar module is one of many examples available on eBay etc. This one was identified as a HB100.

The following IOs of the STM32L476 Discovery board are used:

```
OUTPUTS
3.3V ------------ radar 3.3V supply for level converter
5V_U ------------ radar 5V supply (from ST-LINK USB port) for radar module and amplifier via filter
GND  ------------ radar ground via filter
USB OTG port  --- serial debug data out (waveform and spectrum)
UART ------------ output for serial display board (not detailed here and not used in the original demo)
HEADPHONE ------- live output of sampled signal from radar

INPUT
PA0	 ------------ ADC input from amplifier (shared with the joystick centre button)
```

