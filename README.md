# ECG-kl25z
kl25z ECG monitor, displays real-time ECG graph on a TFT touch screen. This software is part of my biometric identification system.

KL25z microcontroller is programmed using C++.  The software consists of display control, GUI design, interrupt routines, packets creation, enrollment process, login process, timers and serial port communication.

##Initialization:
During initialization, all the necessary pins are configured accordingly. SPI is initialized and the 4 analog pins for the touch resistors are initialized.
Next the display library is started and configured to use the MOSI, MISO and SRCLK pins necessary for the SPI protocol. 

![alt text](https://github.com/wajdib/ECG-kl25z/blob/master/2.jpg "")

## Device Pinout:
The touch display is connected through SPI to the microcontroller. SPI is used to send display commands to the ILI9341 driver that handles the drawings. 
A 4-resistor model is used for the touch functionality. When a portion of the display is pressed, a resistance value is sent through the 4 analog pins to reflect the actual pressing area. 
Touch pins are:
*	X+ PTB3 
*	X- PTB1
*	Y+ PTB2
*	Y- PTB0

The pins used for the SPI communication are:

*	MOSI PTD2
*	MISO PTD3
*	SCLK PTD1
*	CS PTA5
*	DC PTD4

ECG sensor is sending raw signal data as an analog signal that is being captured through the analog in pin of the KL25z PTE20. 

An interrupt occurs every 8ms reading the analog value from that pin and encoding the data into a packet that will be sent through serial port to the PC application. A digital pin is used as input/output to send notification that the microcontroller has finished processing the previous data and is ready for new one. 
Pins used for the ECG connection:
*	RAW ECG analog in PTE20
*	Digital in and out PTD4

![alt text](https://github.com/wajdib/ECG-kl25z/blob/master/3.png "")

