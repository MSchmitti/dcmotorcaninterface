/*
 * muscleDriverCANInterface.h
 *
 *  Created on: 17 Dec 2013
 *      Author: alex
 */

#ifndef MUSCLEDRIVERCANINTERFACE_H_
#define MUSCLEDRIVERCANINTERFACE_H_

#include <QObject>
#include <QTimer>
#include <QMutex>
#include <QThread>
#include <QMap>
#include <QFile>
#include "motorDriverBoardConfiguration/KvaserCanInterface.h"

#include <stdint.h>
#include <QElapsedTimer>

#include "controlCycle.h"

// CAN communication parameters
#define RX_FIELDS_PER_DRIVER 2


// Log every 10 mins = 10 * 60 * 200
#define STATUS_LOG_PERIOD 120000

union motorControlCommand{
	char data[8];
	struct{
		float dutyCycle;
		float free;
	}s;

};

union motorDataSet1 {
	char data[8];
    struct {
	  float omega;
	  int32_t encoderPosition;
    } s;

} ;

union motorDataSet2 {
	char data[8];
    struct {
		int16_t current;
		int16_t displacement;
		uint16_t free1;
		uint16_t free2;
    } s;

} ;

//overwrite CAN interface to implement Callback
class CANInstantiation : public KvaserCanInterface
{
public:
	static void rxCallback(canNotifyData * rxNotifyData);
};




class MuscleDriverCANInterface : public QObject
{
     Q_OBJECT

 public:
     MuscleDriverCANInterface(int cycleTimeInMilliSeconds);


     int mode() const { return m_mode; }

     void handleRxData(canNotifyData * rxNotifyData);

     // getters and setters which are used by the userinterface
     int getState(int line);
     void setStopCommand(int line);
     void setInitCommand(int line);
     void setPauseCommand(int line);
     void setResumeCommand(int line);
     void setCounter(int line, long counter);
     long getCounter(int line);
     double getForceSensor(int line, int id);
     double getDistSensor(int line);
     double getPreSensor(int line);
     int16_t getMotorDisplacement(int line);
     int32_t getMotorEncoderPosition(int line);
     std::string getErrorCode(int line);
     void setErrorCode(int line, std::string error);
     int16_t getMotorCurrentMean(int line);

	 bool getRunParamState(int line);
	 void setRunVnom(int line, double value);
	 void setRunLmax(int line, double value);
	 void updateRunTrajParam(int line);
	 void displayRunTrajParam(int line);

     /* to adapt if needed
     double getForceMax(int line);
     */

     void start();
     void stop();
     void detachCAN();


 private slots:
     void cyclicProcessor(void);


 private:
    int initCAN();
    int readInit();
    int sendMotorCommands();
	CANInstantiation myCan;
	CHANNEL_HANDLE busHandle0;
	float m_reference;
	// motor and joint ids
	int motorTXID[N_DRIVERS];
	int motorRXID[N_DRIVERS];
	// sensor settings
	double forceSensorGain[N_FORCE_SENSORS];
	double forceSensorZero[N_FORCE_SENSORS];
	double pressureSensorGain;
	double pressureSensorZero;
	// hardware settings
	int motorCombiConfig[N_TEST_LINES];
	double ballScrewLead[N_TEST_LINES];
	 

	//array with all ids
	int rxIDFields[RX_FIELDS_PER_DRIVER*N_DRIVERS]; //each motor has 2 tx message and each joint has one
	QMap<int, int> CANIDmap;
	//array with the associated data,  motor drives with two messages each plus  joint
	char rxDataFields[RX_FIELDS_PER_DRIVER*N_DRIVERS] [8];
//signals:
 	//void valueChanged(int newValue);

//private slots:
  	//int dist_sensor(int firstTime);
  	//int force_sensor(int firstTime);

 private:

     motorDataSet1 motorTransmitData[N_DRIVERS];
     motorDataSet2 motorTransmitAuxData[N_DRIVERS];
	 motorControlCommand motorCommand[N_DRIVERS];

	 ControlCycle* controlCycles[N_TEST_LINES];

	 int newDataArrived;
	 int motorDriveOn;
	 QElapsedTimer elapsedTime;
	 QFile outputFile;
	 QMutex mutex; //

	 void bufferCANData();
     void printRxData(void);
     void transmitMotorControlCommand(void);

     unsigned long m_cycleCount;
     int m_mode;
     int count;
     QThread *timerThread;
     qint64 nanoSec;

 };


#endif /* MUSCLEDRIVERCANINTERFACE_H_ */
