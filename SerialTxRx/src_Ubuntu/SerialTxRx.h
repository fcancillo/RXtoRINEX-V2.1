/** @file SerialTxRxLnx.h
 * Contains the SerialTxRx class definition.
 * A SerialTxRx object can be used to send/receive OSP and NMEA message data to/from SiRF IV receivers through a serial port.
 *<p>This implementation uses Linux resources.
 *
 *Copyright 2018 Francisco Cancillo
 *<p>
 *This file is part of the RXtoRINEX tool.
 *<p>
 *RXtoRINEX is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
 *as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *RXtoRINEX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *See the GNU General Public License for more details.
 *<p>
 *A copy of the GNU General Public License can be found at <http://www.gnu.org/licenses/>.
 *
 *Ver.	|Date	|Reason for change
 *------+-------+------------------
 *V1.0  |2/2018 |Linux implementation derived from Windows implementation of this class
 */
#ifndef SERIALTXRX_H
#define SERIALTXRX_H

#include <string>
#include <forward_list>
//needed by (3) termios
#include <termios.h>
#include <unistd.h>

#include "SerialTxRxErrorMSG.h"

///Macro for reporting debugging data; actual definition will depend on defined(_DEBUG)
#if defined(_DEBUG)
#define DBGRPT(format, ...) printf(format,##__VA_ARGS__);
#else
#define DBGRPT(format, ...)
#endif

using namespace std;

#define MAXBUFFERSIZE 2052	//Maximum payload size (2048) + length (2) + checksum (2)
//@cond DUMMY
#define START1 160	//0xA0	//OSP messages from/to receiver are preceded by the synchro
#define START2 162	//0xA2	//sequence of two bytes with values START1, START2
#define END1 176	//0XB0	//OSP messages from/to receiver are followed by the end
#define END2 179	//0XB3	//sequence of two bytes with values END1, END2
#define LF	0x0A			//ASCII Line Feed
#define CR	0x0D			//ASCII Carriage Return
#define DOLAR	0x24		//ASCII Dolar sign
#define CHK	0x2A			//ASCII * (chacksum start: two ASCII hex follow with XOR of chars between $ and *)

//Some identifiers, derived from Windows implementation, are kept here for convenience. F.e. DWORD, CBR, DCB, ...)
#define DWORD speed_t

class CBRrate {		//a class to identify baud rates
friend class SerialTxRx;
	int baudR;		//baud rate value
	DWORD DCBbaud;	//baud rate identifier as per termios
	CBRrate(int, DWORD);
};
//@endcond 

/**SerialTxRx class defines a data and method to manage the serial comm port where a receiver
 * is connected, and allows sending and receiving messages through it.
 * A program using SerialTxRx would perform the following steps after declaring an object of this class:
 * -# To open the serial port with the name given
 * -# To set or get the port parameters (baud rate, timeout, mode)
 * -# To skip input bytes / chars until appears the start of an OSP or NMEA message
 * -# To read or write OSP or NMEA messages
 * -# To close the port
 */
class SerialTxRx {
//@cond DUMMY
	friend class CBRrate;
//@endcond 
	int hSerial;		//the handler for the serial port
	string hName;		//the name of the currently open serial port
	struct termios tio;	//the termios place for their parameters
	unsigned long readLimit;	//maximum number of bytes that can be read in one read call
	forward_list<CBRrate> CBRrateLst;
	string baudRate;	//the baud rate used
	string portName;	//the device port name (like /dev/ttyUSB0)

	void addCBRrate(int rate, DWORD CBRrt);
	DWORD getCBRrate(int);	//get the baud rate identifier as per termios for a given baud rate
	int getBaudRate(DWORD);	//get the baud rate in bps for a given baud rate identifier as per termios
	bool synchOSPmsg(int patience = MAXBUFFERSIZE*2);	//skip bytes until start of OSP message is reached
	bool synchNMEAmsg(int patience = MAXBUFFERSIZE);	//skip bytes until start of NMEA message is reached

public:
	unsigned char paylenBuff[2];		///< a 2 bytes buffer for the payload length bytes
	unsigned char payBuff[MAXBUFFERSIZE];	///< message payload data buffer of MAXBUFFERSIZE bytes
	unsigned int payloadLen;		///< the current payload length, for convenience

	SerialTxRx(void);
	~SerialTxRx(void);
	void openPort(string portName);		//open the serial port with the name given
	void setPortParams(int baudRate, int timeout = 0);	//set port params in the current open port
	void getPortParams(int& baudRate, int& timeout, bool& rawMode);	//get port params in the current open port
	int readOSPmsg(int patience = MAXBUFFERSIZE*2);		//read a OSP message from the serial port
	int readNMEAmsg(int patience = MAXBUFFERSIZE);		//read a NMEA message from the serial port
	void writeOSPcmd(int mid, string cmdArgs, int base = 16);	//generate and send a OSPMessage object containing a command to the receiver
	void writeNMEAcmd(int mid, string cmdArgs);	//generate and send a NMEA message object containing a command to the receiver
	void closePort();						//close the currently open serial port
};
#endif
