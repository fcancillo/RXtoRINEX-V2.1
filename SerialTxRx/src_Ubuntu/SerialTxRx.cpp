/** @file SerialTxRxLnx.cpp
 * Contains the implementation of the SerialTxRx class used to send/receive OSP message data from SiRF IV
 * receivers through a serial port using Linux resources.
 */

#include "SerialTxRx.h"
#include <string.h>
#include <vector>
#include <sstream>

//needed by Linux (2) open, read, write, close
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//needed to compute type limits
#include <limits.h>

CBRrate::CBRrate(int r, DWORD CBRr) {
	baudR = r;
	DCBbaud = CBRr;
}

/**Constructs an empty SerialTxRx object
  */
SerialTxRx::SerialTxRx(void) {
	payloadLen = 0;
	addCBRrate (50, B50);
	addCBRrate (75, B75);
	addCBRrate (110, B110);
	addCBRrate (134, B134);
	addCBRrate (150, B150);
	addCBRrate (200, B200);
	addCBRrate (300, B300);
	addCBRrate (600, B600);
	addCBRrate (1200, B1200);
	addCBRrate (1800, B1800);
	addCBRrate (2400, B2400);
	addCBRrate (4800, B4800);
	addCBRrate (9600, B9600);
	addCBRrate (19200, B19200);
	addCBRrate (38400, B38400);
	addCBRrate (57600, B57600);
	addCBRrate (115200, B115200);
	addCBRrate (230400, B230400);
	readLimit = ((unsigned long)1 << (sizeof(tio.c_cc[0]) * CHAR_BIT)) - 1;
}

/**Destructs SerialTxRx objects.
 */
SerialTxRx::~SerialTxRx(void) {
	CBRrateLst.clear();
}

/**addCBRrate adds a Linux baud rate identifier and its related baud rate to the list of rates the port can use
 *
 * @param rate the baud rate
 * @param CBRrt the Linux rate identifier
 */
void SerialTxRx::addCBRrate (int rate, DWORD CBRrt) {
	CBRrateLst.emplace_front(CBRrate(rate, CBRrt));
}

/**getCBRrate gets the baud rate identifier (as per termios) associated to a given baud rate
 *
 * @param rate the baud rate
 * @return the rate identifier
 * @trow error string message if the given baud rate is not defined for this port 
 */
DWORD SerialTxRx::getCBRrate (int rate) {

	forward_list<CBRrate>::iterator iterator;
	for (iterator = CBRrateLst.begin(); iterator != CBRrateLst.end(); iterator++) {
		if (iterator->baudR == rate) return iterator->DCBbaud;
	}
	throw string(MSG_UnkBaudR);
}

/**getBaudRate gets the baud rate associated to a give baud rate identifier (as per termios)
 *
 * @param r the the rate identifier
 * @return the baud rate
 * @trow error string message if the given rate identifier is not defined for this port 
 */
int SerialTxRx::getBaudRate (DWORD r) {

	forward_list<CBRrate>::iterator iterator;
	for (iterator = CBRrateLst.begin(); iterator != CBRrateLst.end(); iterator++) {
		if (iterator->DCBbaud == r) return iterator->baudR;
	}
	throw string(MSG_UnkBaudR);
}

/**openPort opens the serial port portName to send or receive messages to/from the receiver.
 * Port parameters (baud rate, mode, timeout, etc.) are not modified.
 *
 * @param portName the name of the port to be opened
 * @throw error message when the port cannot be open, explaining the reason 
 */
void SerialTxRx::openPort(string portName) {

	hSerial = open(portName.c_str(), O_RDWR|O_NOCTTY); //|O_NDELAY);
	if (hSerial == -1) throw string(MSG_OpenError) + string(strerror(errno));
	hName = portName;
	if (tcgetattr(hSerial, &tio) == -1) throw string(MSG_InitState) + string(strerror(errno));
}

/**setPortParams sets port baud rate and timeout in the currently open serial port.
 * Other relevant port parameters are set for allowing transfer of OSP and NEMEA messages.
 *
 * @param baudRate the value of the baud rate to be set, if different from 0 bps
 * @param timeout the limit for a timer in tenths of a second to wait for input data
 * @throw error string with the message explaining it
 */
void SerialTxRx::setPortParams(int baudRate, int timeout) {

	//set speed parameter
	try {
		if (baudRate != 0)
			if (cfsetspeed(&tio, getCBRrate(baudRate)) == -1) throw string(MSG_UnkBaudR) + string(strerror(errno));
	} catch (string error) { throw (error); }
	//set raw mode parameters
	cfmakeraw(&tio);
	//set circumstances for read completion
	tio.c_cc[VMIN] = readLimit;
	tio.c_cc[VTIME] = timeout;
	//program port with parameters set
	if (tcsetattr(hSerial, TCSAFLUSH, &tio) == -1) throw string(MSG_SetState);
}

/**getPortParams gets current port parameters: baud rate, timeout and mode.
 *
 * @param baudRate a variable to place the value of the baud rate
 * @param timeout a variable to place the value of the timeout limit when reading input
 * @param mode a variable to place a true value if raw mode (non canonical) is used, or false otherwise
 * @throw error string with the message explaining it
 */
void SerialTxRx::getPortParams(int& baudRate, int& timeout, bool& rawmode) {

	if (tcgetattr(hSerial, &tio) == -1 ) throw string(MSG_InitState);
	try {
		baudRate = getBaudRate(cfgetospeed(&tio));
		if (baudRate != getBaudRate(cfgetispeed(&tio))) throw string(MSG_IODifBaudR);
		timeout = (int)tio.c_cc[VTIME];
		if (tio.c_lflag & ICANON) rawmode = false;
		else rawmode = true;
	} catch (string error) { throw (error); }
}

/**closePort closes the currently open serial port.
 */
void SerialTxRx::closePort() {
	close(hSerial);
	hName.clear();
}

/**synchroOSPmsg skips bytes from input until start of OSP message is reached.
 * Note that start of OSP message is preceded by the sequence of the two bytes START1 (A0) START2 (A2)
 *
 * @param patience is the maximum number of bytes to skip or unsuccessful reads from the serial port before returning a false value
 * @return true if the sequence START1 START2 has been detected, false otherwise
 */
bool SerialTxRx::synchOSPmsg(int patience) {
	ssize_t  nBytesRead;
	unsigned char inData;
	#if defined (_DEBUG)
	int n0read = 0;
	#endif
	DBGRPT("synchOSPmsg: ")

	//A state machine automata is used to skip bytes from input until START1 START2 appears
	//States: 1=is waiting to START1; 2=is waiting for STAR2; 3=START1+START2 detected
	int state = 1;
	while (state!=3 && patience>0) {
		nBytesRead = read(hSerial, &inData, 1);
		if (nBytesRead == 1) {
			DBGRPT ("%02X ", (unsigned int) inData)
			switch (state) {
			case 1:
				switch (inData) {
				case START1:	state = 2; break;
				case START2:	break;
				default:		patience--; break;
				}
				break;
			case 2:
				switch (inData) {
				case START1:	break;
				case START2:	state = 3; break;
				default:		state = 1; patience--; break;
				}
			}
		} else {
			#if defined (_DEBUG)
			n0read++;
			#endif
			patience--;
		}
	}
	DBGRPT("state=%d;patience=%d;n0read=%d\n", state, patience, n0read)
	return state == 3;
}

/**readOSPmsg reads a OSP message from the serial port.
 *
 * @param patience is the maximum number of bytes to skip or unsuccessful reads from the serial port before returning error
 * @return the exit status according to the following values and meaning:
 *		- (0) when a correct formatted OSP message has been received;
 *		- (1) if the message has incorrect checksum;
 *		- (2) if error occurred when reading payload or not enough bytes were received
 *		- (3) if the payload length read is out of margin (>MAXBUFFERSIZE)
 *		- (4) if unable to read the two bytes of the OSP payload length 
 *		- (5) if read error occurred reading OSP payload bytes
 *		- (6) if OSP start bytes have not been received before "exhaust patience"
 */
int SerialTxRx::readOSPmsg(int patience) {
	ssize_t nBytesRead;

	//skip bytes until beginning of a message
	if (!synchOSPmsg(patience)) return 6;
	//read payload length field (2 bytes)
	DBGRPT("readOSPmsg:")
	nBytesRead = read(hSerial, paylenBuff, 2);
	if (nBytesRead != 2) {
		DBGRPT("error 4\n")
		return 4;
	}
	payloadLen = (paylenBuff[0] << 8) | paylenBuff[1];	//numbers in OSP msg are big endians
	DBGRPT("pllen=%d;", payloadLen)
	if (!((payloadLen > 0) && (payloadLen < MAXBUFFERSIZE-1-2))) {
		DBGRPT("error 3\n")
		return 3;
	}
	//read payload data plus 2 checksum bytes
	ssize_t nBytesStored = 0;
	do {
		nBytesRead = read(hSerial, payBuff+nBytesStored, payloadLen+2 - nBytesStored);
		nBytesStored += nBytesRead;
	} while ((nBytesRead != 0) && (nBytesStored < payloadLen+2));
	DBGRPT("pl [")
	#if defined (_DEBUG)
	for(int i=0; i<(int) nBytesStored; i++) DBGRPT ("%02X ", (unsigned int) payBuff[i])
	#endif
	DBGRPT("] %d bytes; ", (int) nBytesStored)
	if (nBytesStored < payloadLen+2) return (2);
	//compute checksum of payload contents
	unsigned int computedCheck;
	computedCheck = payBuff[0];
	for (unsigned int i=1; i<payloadLen; i++) {
		computedCheck += payBuff[i];
		computedCheck &= 0x7FFF;
	}
	//get checksum received after message payload
	unsigned int messageCheck;
	messageCheck = (payBuff[payloadLen] << 8) | payBuff[payloadLen+1];
	if (computedCheck != messageCheck) {
		DBGRPT("error 1\n")
		return 1;	//checksum does not match!
	}
	DBGRPT("End OK\n")
	return 0;
}

/**writeOSPcmd builds a OSP message command and sends it to receiver through the serial port.
 *
 * @param mid the message identification of the OSP command to be generated
 * @param cmdArgs the rest of message payload, as a whitespace-separated sequence of bytes
 * @param base the base used to write the parameters (16, 10, ...)
 * @throw error string when message cannot be send
 */
void SerialTxRx::writeOSPcmd(int mid, string cmdArgs, int base) {
	//extract command arguments in cmrArgs: bytes in hexadecimal separated by blanks
	stringstream ss(cmdArgs);	// Insert the string with payload data into a stream
	vector<string> tokens;		//create a vector for command arguments
	string strBuf;
	DBGRPT("writeOSPmsg:");
	while (ss >> strBuf)		//extract arguments: each one is a token in the string
		tokens.push_back(strBuf);
	unsigned int payloadLen = 1 + tokens.size();
	if ((2 + 2 + 1 + payloadLen + 2 + 2) > MAXBUFFERSIZE) {
		string error = "Error OSP cmd too long = " + to_string((long long) payloadLen);
		DBGRPT("%s\n", error.c_str());
		throw error;
	}
	//start filling the message buffer (pyload buffer used here for that) with command data
	unsigned int bufferIndex = 0;
	payBuff[bufferIndex++] = START1;
	payBuff[bufferIndex++] = START2;
	payBuff[bufferIndex++] = (unsigned char) (payloadLen >> 8);
	payBuff[bufferIndex++] = (unsigned char) (payloadLen & 0xFF);
	payBuff[bufferIndex++] = (unsigned char) mid;
	unsigned long ul;
	for (vector<string>::iterator it = tokens.begin(); it != tokens.end(); it++) {
		ul = stoul(*it, nullptr, base);
		payBuff[bufferIndex++] = (unsigned char) ( ul & 0xFF);
	}
	//compute checksum and put its value in the buffer
	unsigned int computedCheck = payBuff[4];
	for (unsigned int i=5; i<bufferIndex; i++) {
		computedCheck += payBuff[i];
		computedCheck &= 0x7FFF;
	}
	payBuff[bufferIndex++] = (unsigned char) (computedCheck >> 8);
	payBuff[bufferIndex++] = (unsigned char) (computedCheck & 0xFF);
	//append end sequence
	payBuff[bufferIndex++] = END1;
	payBuff[bufferIndex++] = END2;
	//write the message to the output stream
	ssize_t nBytesWritten = 0;
	nBytesWritten = write(hSerial, payBuff, bufferIndex);
	DBGRPT("pllen=%d;msg [ ",payloadLen);
	#if defined (_DEBUG)
	for(unsigned int i=0; i<bufferIndex; i++) DBGRPT("%02X ", (unsigned int) payBuff[i]);
	#endif
	DBGRPT("] %d bytes\n", (int) nBytesWritten)
	if (tcdrain(hSerial) == -1) {
		string error = "Error draining OSP cmd " + to_string((long long) payBuff[4]);
		DBGRPT("%s. %s\n", error.c_str(), strerror(errno));
		throw error;
	}
	if (bufferIndex != nBytesWritten) {
		string error = "Error sending OSP cmd " + to_string((long long) payBuff[4]);
		DBGRPT("%s.\n", error.c_str());
		throw error;
	}
/*
	if ((bufferIndex != nBytesWritten) || (fdatasync(hSerial) != 0)) {
		string error = "Error sending OSP cmd " + to_string((long long) payBuff[4]);
		DBGRPT("%s. %s\n", error.c_str(), strerror(errno));
		throw error;
	}
*/
}

/**synchroNMEAmsg skips bytes until start of NMEA message is reached.
 *The NMEA message is preceded by the sequence <LineFeed><$> in the input stream of ASCII chars.
 *
 * @param patience the maximum number of skipped bytes or unsuccessful reads before returning 
 * @return true if the sequence <LineFeed>$ has been detected in the ASCII input sequence, false otherwise
 */
bool SerialTxRx::synchNMEAmsg(int patience) {
	ssize_t nBytesRead;
	unsigned char inData;
	#if defined (_DEBUG)
	int n0read = 0;
	#endif
	//A state machine automata is used to skip bytes from input until <LineFeed><Dolar> appears
	//States: 1=is waiting to <LineFeed>; 2=is waiting for <$>; 3=<LineFeed><$> detected
	int state = 1;
	while (state!=3 && patience>0) {
		nBytesRead = read(hSerial, &inData, 1);
		if (nBytesRead == 1) {
			switch (state) {
			case 1:
				switch (inData) {
				case LF:	state = 2; break;
				case DOLAR:	break;
				default:	patience--; break;
				}
				break;
			case 2:
				switch (inData) {
				case LF:	break;
				case DOLAR:	state = 3; break;
				default:	state = 1; patience--; break;
				}
			}
		} else {
			#if defined (_DEBUG)
			n0read++;
			#endif
			patience--;
		}
	}
	DBGRPT("synchNMEA:state=%d;patience=%d;n0read=%d\n", state, patience, n0read);
	return state == 3;
}

/**readNMEAmsg reads a NMEA message from the serial port.
 *
 * @param patience the maximum number of skipped chars or unsuccessful reads before returning 
 * @return the exit status according to the following values and meaning:
 *		- (0) when a correct message has been received;
 *		- (1) if the received message has incorrect checksum;
 *		- (2) if message received has less than five chars. Minimum NMEA message is $XXX*SS
 *		- (3) if the input stream does not contains NMEA messages or an input error occurred or EOF
 *		- (4) if NMEA start bytes have not been received before "exhaust patience"
 */
int SerialTxRx::readNMEAmsg(int patience) {
	ssize_t nBytesRead = 0;
	unsigned int computedCheck = 0;
	unsigned int messageCheck = 0;
	int returnValue = 3;
	payloadLen = 0;
	//skip bytes until beginning of a message found (<LF><DOLLAR>)
	if (!synchNMEAmsg(patience)) return 4;
	//read NMEA message bytes (up to CR) and put them into payBuff
	DBGRPT("readNMEAmsg:")
	//TBD
	while ((nBytesRead = read(hSerial, (payBuff+payloadLen), 1) == 1)) {
		if (*(payBuff+payloadLen) == CR) {	//is the last char in a NMEA message
			*(payBuff+payloadLen) = 0;	//convert chars received to a C-string
			if (payloadLen < 5) {		//minimum NMEA message is $XXX*SS<CR>
				returnValue = 2;
				break;
			}
			payloadLen -= 3;	//last three bytes are the checksum: *SS
			*(payBuff+payloadLen) = 0;	//mark end of message
			returnValue = 0;
			break;
		}
		if (payloadLen < MAXBUFFERSIZE-1) payloadLen++;
		else break;
	}
	DBGRPT("pllen=%d", payloadLen)
	if (returnValue == 0) {	//a NMEA message has been receive
		//compute and verfy its checksum
		DBGRPT(";msg=%s", payBuff)
		computedCheck = payBuff[0];
		for (unsigned int i=1; i<payloadLen; i++) computedCheck ^= payBuff[i];
		//get checksum received after message payload
		sscanf((char *) (payBuff+payloadLen+1), "%x", &messageCheck);
		if (computedCheck != messageCheck) returnValue = 1;	//checksum does not match
	}
	DBGRPT(";return=%d\n", returnValue)
	return returnValue;
}

/**writeNMEAcmd builds a NMEA message command and sends it to receiver through to the serial port.
 *
 * @param mid the identification of the command to be generated in the form $PSRF<mid>, ...
 * @param cmdArgs the message parameters: a comma separated list of arguments
 * @throw  error string message explaining it
 */
void SerialTxRx::writeNMEAcmd(int mid, string cmdArgs) {
	ssize_t nBytesWritten = 0;
	char checksumBuff[10];
	//init buffer with command header data and append arguments
	sprintf((char*) payBuff, "$PSRF%3d,", mid);
	strncat((char*) payBuff, cmdArgs.c_str(), MAXBUFFERSIZE);
	//compute checksum and append its value
	int plLen = strlen((char*) payBuff);
	unsigned int computedCheck = payBuff[1];
	for (int i=2; i<plLen; i++) computedCheck ^= (unsigned int) payBuff[i];
	sprintf(checksumBuff, "*%02X\r\n", computedCheck);
	strncat((char*) payBuff, checksumBuff, MAXBUFFERSIZE);
	//send command
	plLen = strlen((char*) payBuff);
	nBytesWritten = write(hSerial, payBuff, plLen);
	DBGRPT("writeNMEAmsg:(%d)=%s",(int) nBytesWritten, payBuff)
	if (tcdrain(hSerial) == -1) {
		string error = "Error draining NMEA $PSRF data";
		DBGRPT("%s. %s\n", error.c_str(), strerror(errno));
		throw error;
	}
	if (nBytesWritten != plLen) {
		throw string("Error sending NMEA $PSRF"
				+ to_string((long long) mid)
				+ "," + cmdArgs);
	}
}
