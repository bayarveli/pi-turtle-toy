/*
 * serialPort.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: root
 */

#include "serialLib.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>

serialPort::serialPort()
{
	// TODO Auto-generated constructor stub
	this->uart0_filestream = -1;

}

void serialPort::openUART()
{
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	this->uart0_filestream = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

	if (this->uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	else
	{
		printf("Success - Able to open UART\n");
	}
}

void serialPort::configureUART()
{
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(this->uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(this->uart0_filestream, TCIFLUSH);
	tcsetattr(this->uart0_filestream, TCSANOW, &options);
}

serialPort::SERIAL_PORT_DATA serialPort::readUART()
{

	SERIAL_PORT_DATA serialPortData;

	//unsigned char rx_buffer[256];
	//----- CHECK FOR ANY RX BYTES -----
	if (this->uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there

		int rx_length = read(this->uart0_filestream, (void*)serialPortData.rxBuffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			serialPortData.rxBufferLength = -1;
			return serialPortData;
		}
		else if (rx_length == 0)
		{
			//No data waiting
			serialPortData.rxBufferLength = 0;
			return serialPortData;
		}
		else if (rx_length == 12)
		{
			//Bytes received
			serialPortData.rxBuffer[rx_length] = '\0';
			serialPortData.rxBufferLength = rx_length;
			printf("%i bytes read : %s\n", rx_length, serialPortData.rxBuffer);

		}
		return serialPortData;
	}
	else
	{
		serialPortData.rxBufferLength = -99;
		return serialPortData;
	}
}

void serialPort::closeUART()
{
	//----- CLOSE THE UART -----
	close(this->uart0_filestream);
}

serialPort::~serialPort() {
	// TODO Auto-generated destructor stub
}

