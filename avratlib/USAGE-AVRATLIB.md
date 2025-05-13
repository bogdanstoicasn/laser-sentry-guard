## Makefile

Use **make + needed-util** to put the required files in the lib builder.
Example:
    
    1. make pwm adc => adds the pwm and adc to the lib

    2. make gpio    => adds the gpio to the lib

    3. make all     => adds everything to the lib

After each make use: **make lib** to create the ".a" file.

Example on how to run with our main file:

```bash
make all
make lib
make main
make flash
```
## Attention

The base **Makefile** is set to use the **atmega328p** as the target, the **F_CPU** is set to **16000000UL**, the port is set to **/dev/ttyACM0** and the programmer is set to **arduino**.

The above settings can be changed in the **Makefile** or when directly calling the **make** command.
Example:

```bash
make main F_CPU=8000000UL TARGET=atmega328p PORT=/dev/ttyUSB0 PROGRAMMER=usbasp
```

## Lib usage

Check the **.h** files for the functions name and usage.

## TODO

Modify adc driver and add add more pwm options.
