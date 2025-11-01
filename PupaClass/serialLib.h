/*
 * serialPort.h
 *
 *  Created on: Sep 8, 2015
 *      Author: root
 */

#ifndef PUPACLASS_SERIALPORT_H_
#define PUPACLASS_SERIALPORT_H_

class serialPort {
public:

	int uart0_filestream;

	serialPort();

	typedef struct
	{
		unsigned char * rxBuffer;
		int rxBufferLength;
	} SERIAL_PORT_DATA;


	SERIAL_PORT_DATA serialPortData;

	void openUART();
	void configureUART();
	void closeUART();
	SERIAL_PORT_DATA readUART();

	virtual ~serialPort();
};

#endif /* PUPACLASS_SERIALPORT_H_ */
