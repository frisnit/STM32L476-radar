# STM32L476-radar hardware

Schematic of the radar hardware. It's just a couple of op amp gain stages with the output coupled to the PA0 ADC input of the STM32L476 via a level converter. The 5V supply for the radar and amplifier is passed through a simple filter to clean it up.

The following pins of the STM32L476 Discovery board are used:

```
3.3V --- radar 3.3V supply (level converter)
5V_I --- radar 5V supply (radar module and amplifier) via filter
GND  --- radar ground via filter
PA0	 --- ADC input (shared with the joystick centre button)
```



