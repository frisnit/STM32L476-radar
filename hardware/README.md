# STM32L476-radar hardware

The schematic of the radar hardware. It's just a couple of op amp gain stages with the output coupled to the PA0 ADC input of the STM32L476 via a level converter. The 5V supply for the radar and amplifier is passed through a simple filter to clean it up.

The amplifier is based on an LM358 single supply dual opamp. The Doppler radar module is one of many examples available on eBay etc. This one was identified as a HB100

The following IOs of the STM32L476 Discovery board are used:

```
OUTPUTS
3.3V --- radar 3.3V supply (level converter)
5V_I --- radar 5V supply (radar module and amplifier) via filter
GND  --- radar ground via filter
USB  --- serial debug data (waveform and spectrum)
UART --- output for serial display board (not detailed here and not used in the original demo)

INPUT
PA0	 --- ADC input (shared with the joystick centre button)
```



