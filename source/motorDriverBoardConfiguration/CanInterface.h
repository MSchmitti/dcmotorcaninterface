/*
 * canInterface.h
 *
 *  Created on: 30 Sep 2012
 *      Author: Alexander Lenz
 */

#ifndef CANINTERFACE_H_
#define CANINTERFACE_H_
#include <stdint.h>
#include <stdio.h>
#include <iostream>
using namespace std;

typedef  int CHANNEL_HANDLE;
typedef int STATUS_CODE;

typedef struct STRUCT_CAN_MESSAGE
		{
			 long messageID;
			unsigned int extended;
			unsigned int flags;
			unsigned int dlc;
			unsigned long time;
			union{
					int16_t data16[4];
					unsigned char data8[8];
			};

		}CAN_MESSAGE;


class CanInterface
{
public:
	//CanInterface(void);

	 void printCanMessage(const CAN_MESSAGE &canMessage);
	/*
	 *
	 */
	virtual STATUS_CODE getAvailableCanChannels(int &numberOfChannels){};

	/*
	 *
	 */
	virtual CHANNEL_HANDLE openCanChannel(int channelId){};

	/*
	 *
	 */

	/*Disconnect can transceiver from bus.
	 *
	 */
	virtual STATUS_CODE disconnectCanChannel(CHANNEL_HANDLE channelId){};


	/* Connects can transceiver to bus.
	 *
	 */
	virtual STATUS_CODE connectCanChannel(CHANNEL_HANDLE channelId){};


	virtual STATUS_CODE closeCanChannel(CHANNEL_HANDLE channelId){};
	/*
	 *
	 */
	virtual STATUS_CODE configureCanChannel(CHANNEL_HANDLE busHandle, int baudRate, int silent){};
	/*
	 *
	 */
	virtual void getErrorText(STATUS_CODE currentStatus, string & errorString){};

	/*
	 *
	 */
	virtual STATUS_CODE sendCanMessage(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage){};

	/* \brief waits until message is queued in Tx buffer or timOut is exceeded
	 *
	 */
	virtual STATUS_CODE sendCanMessageWait(CHANNEL_HANDLE busHandle,const CAN_MESSAGE &canMessage, long timeOut){};

	/* \brief Waits until all CAN messages for the specified handle are sent, or the timeout period expires. d
		 *
		 */
	virtual STATUS_CODE sendCanMessageSync(CHANNEL_HANDLE busHandle,long timeOut){};


	virtual STATUS_CODE readCanMessage(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage ){};

	virtual STATUS_CODE readCanMessageWait(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessag, unsigned long timeOut ){};

	virtual ~CanInterface(){};
};




#endif /* CANINTERFACE_H_ */
