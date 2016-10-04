/*! \file KvaserCanInterface.cpp
 *
 *  Created on: 30 Sep 2012
 *      Author: alex
 */



#include <canlib.h>
#include <stdio.h>
#include <iostream>

#include "KvaserCanInterface.h"

using namespace std;

canNotifyData rxNotifyData;

KvaserCanInterface::KvaserCanInterface(void)
{

}

STATUS_CODE KvaserCanInterface::getAvailableCanChannels(int &numberOfChannels)
{
	int channelCount;
	int status;
	status=canGetNumberOfChannels(&channelCount);
	numberOfChannels=channelCount;
	return status;
}


/*
 *
 */
 CHANNEL_HANDLE KvaserCanInterface::openCanChannel(int channelId)
 {
	 int handle;

	 //opens channel for exclusive access
	 handle =  canOpenChannel(channelId , canWANT_EXCLUSIVE);
	 return handle;
 }
/*
 *
 */
 STATUS_CODE KvaserCanInterface::configureCanChannel(int busHandle, int baudRate, int silent)
 {
	 //values below from Kvaser example
	 int canStatus;
	 long freq = baudRate;
	 unsigned int tseg1=4;
	 unsigned int tseg2=3;
	 unsigned int sjw=1;
	 unsigned int noSamp=1;
	 unsigned int syncmod=0;
	 canStatus = canSetBusParams(busHandle, freq, tseg1,tseg2, sjw, noSamp,  syncmod);

	 if (canStatus>-1)//which means we have no error
	 {
		//set bus to silent mode if this is requested
		 if (silent)
		 {
			 canStatus=canSetBusOutputControl(busHandle,canDRIVER_SILENT);
		 }
		 else
		 {
			 canStatus=canSetBusOutputControl(busHandle,canDRIVER_NORMAL);
		 }

	 }
		 return canStatus;
 }


 STATUS_CODE KvaserCanInterface::disconnectCanChannel(CHANNEL_HANDLE channelId)
 {
	 return canBusOff(channelId);
 }


/* Connects can transceiver to bus.
 *
 */
 STATUS_CODE KvaserCanInterface::connectCanChannel(CHANNEL_HANDLE channelId)
 {
	return canBusOn(channelId);

 }


/*
 *
 */
 void KvaserCanInterface::getErrorText(STATUS_CODE currentStatus, string & errorString)
 {
	 char textBuffer[100];
	 canGetErrorText((canStatus)currentStatus,textBuffer,100);
	 string kvaserErrorString(textBuffer);
	 errorString=kvaserErrorString;
	 return;
 }

 STATUS_CODE KvaserCanInterface::sendCanMessage(CHANNEL_HANDLE busHandle,CAN_MESSAGE &canMessage)
 {

	 //check if extended message is requested
	 unsigned int flag;
	 if (canMessage.extended)
	 {
		 flag=canMSG_EXT;
	 }
	 else
	 {
		 flag=canMSG_STD;
	 }


	 if (canMessage.dlc>8)
	 {
		 canMessage.dlc=8;
	 }
	 return canWrite(busHandle,		  canMessage.messageID,			  &(canMessage.data8[0]),	 canMessage.dlc,  	  flag);
 }


 STATUS_CODE KvaserCanInterface::sendCanMessageWait(CHANNEL_HANDLE busHandle,const CAN_MESSAGE &canMessage, long timeOut)
 {
	 return 0;
 }
	 /* \brief Waits until all CAN messages for the specified handle are sent, or the timeout period expires. d
	  *
	 */
STATUS_CODE KvaserCanInterface::sendCanMessageSync(CHANNEL_HANDLE busHandle,long timeOut)
 {
	 return 0;
 }

 STATUS_CODE KvaserCanInterface::readCanMessage(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage )
 {

	 unsigned long rxTime;
	 return canRead(busHandle,
			 &(canMessage.messageID),
			 &(canMessage.data8[0]),
	    	 &(canMessage.dlc),
	     	 &(canMessage.flags),
	     	 &rxTime);

 }
 STATUS_CODE KvaserCanInterface::readCanMessageWait(CHANNEL_HANDLE busHandle, CAN_MESSAGE &canMessage,  long timeOut )
 {
	 return 0;
 }


 void KvaserCanInterface::rxCallback(/*CAN_MESSAGE &canMessage*/)
 {
	 cout<<endl<<"RxCallback"<<endl;
	 //CAN_MESSAGE &canMessage;
	 //readCanMessage();

 }

 STATUS_CODE KvaserCanInterface::setUprxCallback(CHANNEL_HANDLE busHandle, void (*callback)(canNotifyData *))
 {
	 return   canSetNotify (busHandle,
			 	 	 	 	 callback,
			 	 	 	 	  canNOTIFY_RX,
			 	 	 	 	  this);
 }


 void RxNotifyCallback(canNotifyData * rxNotifyData)
 {

	 //KvaserCanInterface *myInstance=(KvaserCanInterface*) rxNotifyData->tag;
	 //myInstance->rxCallback();
 }
