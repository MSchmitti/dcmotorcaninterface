/*! \file KvaserCanInterface.h
 *
 *  Created on: 30 Sep 2012
 *      Author: Alexander Lenz
 */

#ifndef KVASERCANINTERFACE_H_
#define KVASERCANINTERFACE_H_

#include "CanInterface.h"
#include "canlib.h" //include the KVASER an library
#include "canstat.h"


/*Definition of  possible bitrates based on KVASER definition
 */
#define CAN_BAUDRATE__1M   canBITRATE_1M
#define CAN_BAUDRATE__500K canBITRATE_500K
#define CAN_BAUDRATE__250K canBITRATE_250K
#define CAN_BAUDRATE__125K canBITRATE_125K
#define CAN_BAUDRATE__100K canBITRATE_100K
#define CAN_BAUDRATE__62K  canBITRATE_62K
#define CAN_BAUDRATE__50K  canBITRATE_50K
#define CAN_BAUDRATE__83K  canBITRATE_83K
#define CAN_BAUDRATE__10K  canBITRATE_10K



/* \brief C-type global function to handle the callback
 *
 */
void RxNotifyCallback(canNotifyData *);


/*! /brief encapsulates the Kvaser CAN API
 *
 */
class KvaserCanInterface : public CanInterface
{
public:
	KvaserCanInterface(void);
	STATUS_CODE getAvailableCanChannels(int &numberOfChannels);

	/*
	 *
	 */
	 CHANNEL_HANDLE openCanChannel(int channelId);


	/*
	 *
	 */
	 STATUS_CODE configureCanChannel(int busHandle, int baudRate, int silent);

		/*Disconnect can transceiver from bus.
		 *
		 */
		 STATUS_CODE disconnectCanChannel(CHANNEL_HANDLE channelId);


		/* Connects can transceiver to bus.
		 *
		 */
		 STATUS_CODE connectCanChannel(CHANNEL_HANDLE channelId);


	/*
	 *
	 */
	 void getErrorText(STATUS_CODE currentStatus, string & errorString);

	 /* sends can message
	  *
	  */
	  STATUS_CODE sendCanMessage(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage);

	 /*\brief waits until message is queued in Tx buffer or timOut is exceeded
	 *
	 */
	  STATUS_CODE sendCanMessageWait(CHANNEL_HANDLE busHandle,const CAN_MESSAGE &canMessage, long timeOut);

	 /* \brief Waits until all CAN messages for the specified handle are sent, or the timeout period expires. d
	  *
	 */
	 STATUS_CODE sendCanMessageSync(CHANNEL_HANDLE busHandle,long timeOut);


	 STATUS_CODE readCanMessage(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage);

	 STATUS_CODE readCanMessageWait(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage, long timeOut );

	 static void rxCallback(void);

	 STATUS_CODE setUprxCallback(CHANNEL_HANDLE busHandle, void (*callback)(canNotifyData *));

private:

	 //some kvaser specific functions


};



#endif /* KVASERCANINTERFACE_H_ */
