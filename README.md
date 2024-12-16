# A6 - Analog to Digital Converter

## Reference Materials
- STM32L4xxxx Reference Manual (RM) – ADC
- STM32L4x6xx Datasheet (DS) – Pinouts
- NUCLEO-L4x6xG Users Manual (UM) – Pin Diagram (L476RG, L4A6ZG, L496ZG)

## Connecting to the ADC
- Connect an adjustable DC voltage source to an analog input pin on the STM32L4. 
- BEWARE OF VOLTAGES ABOVE 5 V! 
- Always be sure to double check the voltage before connecting or powering any analog voltage connected to the STM32L4. When connecting the external source, make sure to connect the ground of the source to the STM32L4 ground. Possible sources include power supplies, function generators, or wavegen outputs from the oscilloscope. Any source that you can reliably adjust a DC voltage from 0 to 3.3V will work. To be careful, never set the voltage above 3.3V. Also be sure to turn on the voltage source and set the voltage output before connecting it to the STM32L4. Many pins have been fried due to turning on a voltage source after connecting it to the STM32L4 only to realize that the output had previously been set to a value outside the acceptable range.

## Instructions
- Write some functions to utilize the ADC
   - ADC_init()
    - Run the ADC with a clock of at least 24 MHz.
    - Single conversion, not repeated, initiated with start conversion bit
    - Use sample and hold timer with a sample time of 2.5 clocks
    - 12-bit conversion using 3.3 V reference
    - Configure analog input pin
  - ADC ISR
    - Save the digital conversion to a global variable
    - Set a global flag
- Write a program that initializes the ADC and then runs in an infinite loop checking for the global flag set by the ADC ISR. When the flag is set, it should save the converted value into an array, reset the flag,  and start the ADC to perform another sample and conversion. After collecting 20 samples from the ADC, process the array to calculate the minimum, maximum, and average values. (Protip: Use 16-bit variables for saving the samples and 32-bit variables for the calculations to avoid overflow errors)
- Connect the selected analog input pin to 1.0 V from a  DC power supply (V+) and run your program. Adjust the voltage input from 0 to 3 V to verify the values from the ADC change accordingly.
### Calibrate your ADC
-  your ADC to get a voltage value from the digital conversion that matches the function generator or power supply. This value should be accurate (+/- 10 mV) for voltages 0.00 – 3.00 V. Approaches on how to derive a calibration equation can be found in the Technical Note on Calibrating ADC / Sensor. (Protip: Avoid the use of floats by adjusting the calibration to result in units other than volts)
- Add this calibration calculation to the array processing, converting the resulting minimum, maximum, and average values into voltages. (Protip: It is faster to only covert min, max, and average instead of converting all 20 values) 
### Print to the Terminal
- Print the calibrated voltages (Min, Max, Avg) to a terminal via USART. Remember that the USART can only transmit a single character at a time, so you will have to break down your calculated values into individual digits and transmit them sequentially. Printing to the terminal cannot make use of any stdio.h, stdlib.h, or string.h C library functions for converting numbers to strings. (No itoa(), sprintf() or similar allowed. Sorry. Not sorry.) These values should be printed in units of volts (not mV or uV)  with 2 decimal precision.
- Using a 1.5 V input, adjust the ADC sample time to 47.5, and 640.5 clock cycles, recording the variation in the ADC values for each. Record each of these voltage values (Min, Max, Avg) into a table for submission.

## Deliverables
- Demonstrate your working program to the instructor or lab assistant
- Create a single pdf document (not a full lab report) containing the following:
    - Table of ADC sample time clocks and corresponding min, max, and average voltages.
- Properly formatted C source code
