serdac is two channel a serial to analog interface, designed to control the powerchair through a modified joystick.
The serdac PCB and a 5-pin XLR connector replace the analog joystick. Serial data is transmitted in a packet format,
each channel value is represented by a signed 16-bit integer (although the dac is only 12-bit, the low 4 bits are ignored).
If the incoming serial data stream stops for more than 100ms the device reverts to neutral (1/2 vref) output voltage.

serdac receives serial data at 115200 baud, 8N1 the data packet format is as follows:
	0xaa (sync)
	0xaa (sync)
	0x04 (length of payload, ie. 4 bytes)
	Xmsb
	Xlsb
	Ymsb
	Ylsb
	chksum ( (Xmsb + Xlsb + Ymsb + Ylsb) % 256 )

Other notes:
I have tried modifying two different Invacare 1812 joysticks and both exhibit low sensitivity and poor linearity on the left side of the throw.
I originally thought that the problem was either with the joysticks or the serdac d/a converter but I have confirmed that the problem is not related to
either. I suspect that the problem is with the Invacare joystick firmware or possibly even the TDX3 firmware. The remedy is to swap the X/Y
axes in both the chair programming and the serial data source. This moves the problem from the left to the reverse direction, where it is less of an issue.


