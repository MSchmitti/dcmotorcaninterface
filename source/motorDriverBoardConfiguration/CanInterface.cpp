#include "CanInterface.h"
#include <iostream>
#include <iomanip>
void CanInterface::printCanMessage(const CAN_MESSAGE &canMessage)
{

	cout<<hex<<setw(8)<<setfill('0')<<canMessage.messageID<<"  "<<setw(1)<<canMessage.dlc<<"  ";
	for (int i=0;i<canMessage.dlc && i<8; i++)
	{
		cout<<" "<<setw(2)<< setfill('0')<<hex<< (unsigned int) canMessage.data8[i];
	}
	cout<<endl;
}
