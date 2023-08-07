# STM32_ETH_W5500
 W5500 Library


Module communications.
Registers read/write procedure

	1 - Send 16 bit Address Phase (16bits) - register address (First High and then low)
	2 - Send Control Phase (8 bits) -  Byte composition is listed in the next image.
	3 - For read operation, you'll receive the data, in opposite you'll put the data to be written.

