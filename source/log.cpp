/*
 * log.cpp
 *
 *  Created on: 10 Jul 2015
 *      Author: tobias
 */

#include <time.h> 
#include <iostream> 
#include <string>
#include <iomanip>

#include <sys/types.h>
#include <sys/stat.h>

#include "log.h"

using namespace std;

// Full log
int fullLogLine;
fstream fullLog;

//static int initDesc = 1;

// Measurement log (during RUN)
int measurementLogOpen 			= 0;
int measurementLogReady 		= 0;
int measurementLogRequested		= 0;
int measurementLogInProgress 	= 0;
fstream measurementLog;

// returns the system date and time
string getTime()
{
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
	char buffer [80];
	strftime (buffer,80,"%F-%T",timeinfo);
	return buffer;
}

/*** Permanent Logging ***/
// open log files
void startLog()
{
	mkdir("log",0711);
}

// close log files
void stopLog()
{
	// to adapt
	// statusLog.close();
}

/*** Punctual Full Logging ***/
// open full log file
void startFullLog(int line, long line_counter)
{
	char file [50];
	sprintf (file, "./log/fullLog%d-%s-%ld.txt", line, getTime().c_str(),line_counter);
	fullLog.open(file, ios::out | ios::app);	
	fullLogLine = line;
}
// close full log file
void stopFullLog()
{
	fullLogLine = -1;
	fullLog.close();
}

/*** Measurement Logging ***/
// open log file
void setupMeasurementLog(string* filename)
{
	if(measurementLogOpen)
	{
		measurementLog.close();
		measurementLogOpen = 0;
	}	

	char file [40];
	sprintf (file, "./log/%s.txt", filename->c_str());
	measurementLog.open(file, ios::out | ios::app);
	measurementLogOpen 	= 1;
	measurementLogReady = 1;
}

int isReadyMeasurementLog()
{
	return measurementLogReady;
}
int isRequestedMeasurementLog()
{
	return measurementLogRequested;
}
int isInProgressMeasurementLog()
{
	return measurementLogInProgress;
}

int requestMeasurementLog()
{
	if(measurementLogReady)
	{
		measurementLogReady 	= 0;
		measurementLogRequested = 1;
		return 1;
	}
	else
	{
		cout<<"Not ready for logging, define a log file first"<<endl;
		return 0;
	}
}
void startMeasurementLog()
{
	measurementLogInProgress	= 1;
	measurementLogRequested 	= 0;
}
void stopMeasurementLog()
{
	measurementLogInProgress	= 0;
}

// write log entry
void performMeasurementLog(unsigned int m_cycleCount, double targetPos, int32_t motorEncoderPos, double forceSensor, int16_t posSensor)
{
	measurementLog <<setfill(' ')<<setw(10)<<m_cycleCount;
	measurementLog <<","<<setfill(' ')<<setw(10)<<targetPos;
	measurementLog <<","<<setfill(' ')<<setw(10)<<motorEncoderPos;
	measurementLog <<","<<setfill(' ')<<setw(10)<<forceSensor;
	measurementLog <<","<<setfill(' ')<<setw(10)<<posSensor;
	measurementLog <<endl;
}

// write log entry when full log is enabled for this line
/*
void logFullDataSet(int line, unsigned int m_cycleCount, int32_t motorEncoderPosition, 
	float motorOmega, int16_t motorDisplacement, int16_t motorCurrent, double forceSensor, 
	double distSensor, float motorDutyCycle, double targetPos)
{
	if (fullLogLine == line)
	{
	//	fullLog<<m_cycleCount<<","<<motorEncoderPosition<<","<<motorOmega<<","<<motorDisplacement
	//	<<","<<motorCurrent<<","<<forceSensor<<","<<distSensor<<","<<motorDutyCycle<<","<<targetPos<<endl;
	
		fullLog <<setfill(' ')<<setw(10)<<m_cycleCount;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorEncoderPosition;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorOmega;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorDisplacement;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorCurrent;
		fullLog <<","<<setfill(' ')<<setw(10)<<forceSensor;
		fullLog <<","<<setfill(' ')<<setw(10)<<distSensor;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorDutyCycle;
		fullLog <<","<<setfill(' ')<<setw(10)<<targetPos;
		fullLog <<endl;
	}
}
*/

void logFullDataSet(int line, unsigned int m_cycleCount,
	double xlin, double vlin, double targetPos, double targetSpeed,
	int32_t motorEncoderPosition,
	double forceSensor0, double forceSensor1, 
	float motorDutyCycle, double pressureValue)
{
	/*
	// init and write fulllog description/title of values
	if (initDesc == 1)
	{
	

		fullLog <<setfill(' ')<<setw(10)<<line;
		
		fullLog <<setfill(' ')<<setw(10)<<'m_cycleCount';
		fullLog <<","<<setfill(' ')<<setw(10)<<'xlin';
		
		fullLog <<","<<setfill(' ')<<setw(10)<<setfill('vlin');
		fullLog <<","<<setfill(' ')<<setw(10)<<setfill('targetPos');
		fullLog <<","<<setfill(' ')<<setfill('targetSpeed');
		fullLog <<","<<setfill(' ')<<setfill('motorEncoderPosition');
		fullLog <<","<<setfill(' ')<<setfill('motorDutyCycle');
		//fullLog <<","<<setfill(' ')<<setw(10)<<setfill('motorOmega');
		//fullLog <<","<<setfill(' ')<<setw(10)<<setfill('motorDisplacement');
		//fullLog <<","<<setfill(' ')<<setw(10)<<setfill('motorCurrent');
		fullLog <<","<<setfill(' ')<<setfill('forceSensor0');
		fullLog <<","<<setfill(' ')<<setfill('forceSensor1');
		//fullLog <<","<<setfill(' ')<<setw(10)<<setfill('distSensor');
		fullLog <<","<<setfill(' ')<<setfill('pressureValue');
		fullLog <<endl;
		sleep(500);
		initDesc = 2;
	 }*/
	
	if (fullLogLine == line)
	{
		/*fullLog<<m_cycleCount<<","<<motorEncoderPosition<<","<<motorOmega<<","<<motorDisplacement
		<<","<<motorCurrent<<","<<forceSensor<<","<<distSensor<<","<<motorDutyCycle<<","<<targetPos<<endl;
		*/
		fullLog <<setfill(' ')<<setw(10)<<m_cycleCount;
		fullLog <<","<<setfill(' ')<<setw(10)<<xlin;
		fullLog <<","<<setfill(' ')<<setw(10)<<vlin;
		fullLog <<","<<setfill(' ')<<setw(10)<<targetPos;
		fullLog <<","<<setfill(' ')<<setw(10)<<targetSpeed;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorEncoderPosition;
		fullLog <<","<<setfill(' ')<<setw(10)<<motorDutyCycle;
		//fullLog <<","<<setfill(' ')<<setw(10)<<motorOmega;
		//fullLog <<","<<setfill(' ')<<setw(10)<<motorDisplacement;
		//fullLog <<","<<setfill(' ')<<setw(10)<<motorCurrent;
		fullLog <<","<<setfill(' ')<<setw(10)<<forceSensor0;
		fullLog <<","<<setfill(' ')<<setw(10)<<forceSensor1;
		//fullLog <<","<<setfill(' ')<<setw(10)<<distSensor;
		fullLog <<","<<setfill(' ')<<setw(10)<<pressureValue;
		fullLog <<endl;
	}
}

