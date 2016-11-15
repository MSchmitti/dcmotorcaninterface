/*
 * muscleDriverCANInterface.cpp
 *
 *  Created on: 18 Dec 2013
 *      Author: alex
 */

#include "muscleDriverCANInterface.h"
#include <stdio.h>
#include <iostream>
#include <ios>
#include <iomanip>
#include <math.h> 

#include <QSettings>
#include <QDir>
#include <QTextStream>

#include "controlCycle.h"
#include "phidget.h"
#include "log.h"

#include <phidget21.h>//phidget lib

//pointer to current instantiation of CAN interface
MuscleDriverCANInterface * currentCanInterface;


/*
 * This is the main timer driven event loop where
 * control functionality should be implemented.
 */

void MuscleDriverCANInterface::cyclicProcessor()
{
	static int firstTime=1;
	static int preVal;
	m_cycleCount++; //cycle counter

	//make a copy of the CAN data, this is safe and contains mutex
	bufferCANData();

    // read sensor data
    // ==> is done in a phidget thread
 
    //only called once!
	if (firstTime)
	{
		for (int i = 0; i < N_TEST_LINES; i++)
		{
			updateRunTrajParam(i);
			displayRunTrajParam(i);
		}
		firstTime=0;
		motorDriveOn=1;
	}

	// go thru all controlCycles
    for (int i = 0; i < N_TEST_LINES; i++)
	{
		// pass inputs to controlCycle
		controlCycles[i]->m_cycleCount = m_cycleCount;
		
		// MDB - motor sensors
		controlCycles[i]->motorEncoderPosition = motorTransmitData[i].s.encoderPosition;
		controlCycles[i]->motorOmega = motorTransmitData[i].s.omega;
		controlCycles[i]->motorCurrent = motorTransmitAuxData[i].s.current;
		
		// MDB - displacement sensor
		controlCycles[i]->motorDisplacement = motorTransmitAuxData[i].s.displacement;
		
		// Phidget - distance sensor(s)
		//cmval is a global table (see phidget.h) with values of dist sensors
		controlCycles[i]->distSensor = cmval[i];

		// Phidget - force sensor(s)
		//nval is a global table (see phidget.h) with values of force sensors
		for(int j=0;j<N_FORCE_SENSOR_PER_LINE;j++)
		{
			controlCycles[i]->forceSensor[j] = (forceSensorGain[(i*N_FORCE_SENSOR_PER_LINE)+j] * nval[(i*N_FORCE_SENSOR_PER_LINE)+j]) + forceSensorZero[(i*N_FORCE_SENSOR_PER_LINE)+j];
		}		

		// Phidget - pressure sensor(s)
		// CPhidgetInterfaceKit_getSensorCount(ifKit, &phidget_numSensors); <--------- counted enough?? 
		CPhidgetInterfaceKit_getSensorValue(ifKit, 1, &preVal);
		// to convert the value to bar: pressureBar = (pressureValue/45-4)*10/16 
		controlCycles[i]->pressureValue = ((preVal / pressureSensorGain + pressureSensorZero)*10/16); 	// (preVal/45) formula to get value in mA http://www.phidgets.com/docs/1132_User_Guide

		// cycle
		controlCycles[i]->cycle();

		// get outputs from controlCycle
		motorCommand[i].s.dutyCycle = controlCycles[i]->motorDutyCycle;

		// full log
		/*
		logFullDataSet(i, m_cycleCount, controlCycles[i]->motorEncoderPosition, 
			controlCycles[i]->motorOmega, controlCycles[i]->motorDisplacement,
			controlCycles[i]->motorCurrent, controlCycles[i]->forceSensor[0], 
			controlCycles[i]->distSensor, controlCycles[i]->motorDutyCycle,
			controlCycles[i]->targetPos);
		*/
		logFullDataSet(i, m_cycleCount, 
			controlCycles[i]->xlin, controlCycles[i]->vlin, controlCycles[i]->targetPos, controlCycles[i]->targetSpeed,
			controlCycles[i]->motorEncoderPosition, controlCycles[i]->forceSensor[0], controlCycles[i]->forceSensor[1], 
			controlCycles[i]->motorDutyCycle, controlCycles[i]->pressureValue);

		// check if the measurement log is requested
		if(isInProgressMeasurementLog())
		{
			// perform logging
			performMeasurementLog(m_cycleCount, controlCycles[i]->targetPos, controlCycles[i]->motorEncoderPosition,
				controlCycles[i]->forceSensor[0], controlCycles[i]->motorDisplacement, controlCycles[i]->pressureValue);

			// if not in state run anymore, stop logging
			if(controlCycles[i]->state != STATE_RUN)
			{
				stopMeasurementLog();
			}
		}
		else if(isRequestedMeasurementLog() && (controlCycles[i]->state == STATE_RUN))
		{
			startMeasurementLog();
		}
	}

	//provide data on CAN bus
	sendMotorCommands();
}


//CAN Receive Callback, invoked when CAN data on bus.
void CANInstantiation::rxCallback(canNotifyData * rxNotifyData)
{
    //call RX method
	if (rxNotifyData->eventType==canEVENT_RX)
	{
		if (currentCanInterface!=NULL)
			currentCanInterface->handleRxData(rxNotifyData);
	}
}


void MuscleDriverCANInterface::handleRxData(canNotifyData * rxNotifyData)
{
	int handle;
	long id;
	char data[8];
	unsigned int dlc, flags;
	unsigned long timestamp;
	STATUS_CODE stat;


	 while (canERR_NOMSG != canRead(busHandle0, &id, data, &dlc, &flags, &timestamp))
	 {

		 QMap<int, int>::const_iterator it=CANIDmap.find(id); //find the data index from CAN ID
		 if (it!=CANIDmap.end())
		 {
			 mutex.lock();
			 newDataArrived=1; //set the new Data Flag
			 //we found the id, copy data into storage array
			 //thereby creating a local copy of the latest CAN-bus data;
			 //cout<<"id: 0x"<<hex <<id<<" "<<"datafield index: "<< it.value() ;
			 for (int i=0;i<8;i++)
			 {
				 rxDataFields[it.value()][i]=data[i];
				 //std::cout<<"  0x"<<std::hex<< (unsigned int) data[i]<<"  ";


			 }
			 //cout<<endl;
			 mutex.unlock();
		 }
	 }
}




MuscleDriverCANInterface::MuscleDriverCANInterface(int cycleTimeInMilliSeconds)
{
		// variables phidget
		const char *phidget_err;
		int phidget_result, phidget_numSensors;

		m_cycleCount=0;
		motorDriveOn=0;
		m_reference=0.0 ; //default reference signal is 0

		// Init controlCycles
		for (int i = 0; i < N_TEST_LINES; i++)
		{
			controlCycles[i] = new ControlCycle(i);
		}

		
		/*** PHIDGET BOARDS INITIALIZATION ***/
		std::cout<<"*** Initializing phidget boards"<<endl;

		/* BRIDGE: Create, defined handle functions and open phidget bridge */
		std::cout<<"* Bridge"<<endl;

		//create the bridge object
		CPhidgetBridge_create(&bridget);
		
		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)bridget, AttachHandlerF, NULL);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)bridget, DetachHandlerF, NULL);
		CPhidget_set_OnError_Handler((CPhidgetHandle)bridget, ErrorHandlerF, NULL);
		CPhidgetBridge_set_OnBridgeData_Handler(bridget, data, NULL);
		
		// open the phidget object (! after setting all handle functions, if not possible lost events)
		CPhidget_open((CPhidgetHandle)bridget, -1);
		
		//get the program to wait (1c) for a bridge device to be attached
		if(phidget_result = CPhidget_waitForAttachment((CPhidgetHandle)bridget, 1000))
		{
			CPhidget_getErrorDescription(phidget_result, &phidget_err);
			printf("Problem waiting for attachment: %s\n", phidget_err);
		}
		else
		{
			//display the bridge properties
			display_properties_bridge((CPhidgetHandle)bridget);
		}
		
		/* INTERFACE KIT (1011): Create, define handle functions and open interface kit (dist sensors) */
		std::cout<<"* Interface kit"<<endl;

		//create the InterfaceKit object
		CPhidgetInterfaceKit_create(&ifKit);
		
		//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
		CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
		CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
		CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
		
		//Registers a callback that will run if an input changes.
		//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		//CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InputChangeHandler, NULL);
		//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
		//Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
		//Registers a callback that will run if an output changes.
		//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
		//CPhidgetInterfaceKit_set_OnOutputChange_Handler (ifKit, OutputChangeHandler, NULL);
		
		//open the interfacekit for device connections
		CPhidget_open((CPhidgetHandle)ifKit, -1);
		
		//get the program to wait (1s) for an interface kit device to be attached
		if(phidget_result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 1000))
		{
			CPhidget_getErrorDescription(phidget_result, &phidget_err);
			printf("Problem waiting for attachment: %s\n", phidget_err);
		}
		else
		{
			//get the number of sensors available
			CPhidgetInterfaceKit_getSensorCount(ifKit, &phidget_numSensors);

			
			for(int i = 0; i < phidget_numSensors; i++)
			{
				//change the sensitivity trigger of the sensors
				CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, i, 1);

				// set the data rate (valid: 1, 2, 4, 8, and every multiple of 8 until DataRateMin)
				// !!! remark from the documentation:
				// "The analog inputs cannot all be set to the fastest data rate at the same time"
				// => we set all rate to 8ms (as the bridge)
				CPhidgetInterfaceKit_setDataRate(ifKit,i,8);
			}

			//Display the properties of the attached interface kit device
			display_properties(ifKit);
		}
		
		/*** END PHIDGET BOARDS INITIALIZATION ***/

		//read init file
		readInit();

		std::cout<<"Creating CAN connection:"<<endl;
		initCAN();

		std::cout<<"Creating timer event loop:"<<endl;
		timerThread =new QThread(this);
		cout<<"timer: "<< timerThread->currentThread()<<endl;
		//create timer with 5ms interval
		QTimer* timer = new QTimer(0);
		timer->setInterval(cycleTimeInMilliSeconds);
		std::cout<<"Timers Created"<<endl;
		//let timer run in this thread
		timer->moveToThread(timerThread);
		connect(timer, SIGNAL(timeout()), this, SLOT(cyclicProcessor()));
	    	connect(timerThread, SIGNAL(started()), timer, SLOT(start()));
		std::cout<<"Started Timer Event loop:"<<endl;
		//start the timer thread
	    	timerThread->start();
	    	cout<<"timer thread:"<<timerThread->currentThreadId()<<endl;

		std::cout<<"Set-up time verification system:"<<endl;

		std::cout<<endl<<endl<<"--------------------------------"<<endl;
		cout<<"Thread ID init Thread: "<<QThread::currentThreadId()<<endl;
}

int MuscleDriverCANInterface::initCAN()
{

		 string myString;
		 CAN_MESSAGE myCanMessage, canSentMessage;
		 STATUS_CODE myCanStatus;
		 busHandle0=myCan.openCanChannel(0);

		 newDataArrived=0; //reset the new data flag


		 if (busHandle0>-1)
		 {
			 cout<<"CAN bus available:"<<endl;
			 currentCanInterface=this;

			 //configure bus
			  myCanStatus=myCan.configureCanChannel(busHandle0,CAN_BAUDRATE__1M,0); //1Mbit/s not silent
			  myCan.getErrorText(myCanStatus,myString);
			  myCan.connectCanChannel(busHandle0);
			  cout<<"CAN configured, 1Mbit/s, status is: "<<myString<<endl;
			 // cout<<"Reading CAN, waiting for START_MESSAGE:"<<endl;

			  //create Rx Callback connection

			  myCanStatus=myCan.setUprxCallback(busHandle0,CANInstantiation::rxCallback );
			  myCan.getErrorText(myCanStatus,myString);
			  cout<<"Rx call-back set-up: "<<myString<<endl;

		 }
		 else
		 {
			 cout<<"!!!Kvaser CAN bus adapter not availabl! Error!!!"<<endl;
			 return 0;
		 }
	return 1;
}


int MuscleDriverCANInterface::sendMotorCommands()
{

	CAN_MESSAGE canSentMessage;
	for (int i=0;i<N_DRIVERS;i++)
	{
		//assembel CAN message
		canSentMessage.dlc=8;
		canSentMessage.messageID=motorRXID[i];
		canSentMessage.extended=0;

		//copy data
		if (motorDriveOn==1)
		{
			for (int j=0;j<8;j++)
			{
				canSentMessage.data8[j]=motorCommand[i].data[j];
			}
		}
		else
		{
			for (int j=0;j<8;j++)
			{
				canSentMessage.data8[j]=0;//stop motors
			}
		}
		myCan.sendCanMessage(busHandle0,canSentMessage);
      }
}

int MuscleDriverCANInterface::readInit()
{
	for (int i=0;i<N_DRIVERS;i++)
	{
		motorTXID[i]=-1;
		motorRXID[i]=-1;

		motorCommand[i].s.dutyCycle=0.0; //reset motor commands
		motorCommand[i].s.free=0.0;
		count=0; //loop counter
	}

	for (int i=0;i< RX_FIELDS_PER_DRIVER*N_DRIVERS;i++)
	{
		rxIDFields[i]=-1;
		for (int j=0;j<8;j++)
		{
			rxDataFields[i][j]=0; //reset data fields to 0
		}
	}

	QSettings settings( "settings.ini", QSettings::IniFormat );

	for (int i=0;i<N_TEST_LINES;i++)
	{
		// motor driver config
		for (int j=0;j<N_DRIVERS_PER_LINE;j++)
		{
			char str_motor_data [100];

			sprintf(str_motor_data,"%s/MotorTxID%d","nodes",((i*N_DRIVERS_PER_LINE)+j));
			motorTXID[(i*N_DRIVERS_PER_LINE)+j] = settings.value(str_motor_data, -1).toUInt();

			sprintf(str_motor_data,"%s/MotorRxID%d","nodes",((i*N_DRIVERS_PER_LINE)+j));
			motorRXID[(i*N_DRIVERS_PER_LINE)+j] = settings.value(str_motor_data, -1).toUInt();			
		}

		// force sensor config
		for (int j=0;j<N_FORCE_SENSOR_PER_LINE;j++)
		{
			char str_fs_data [100];

			sprintf(str_fs_data,"%s/ForceSensorGain%d","sensors",((i*N_FORCE_SENSOR_PER_LINE)+j));
			forceSensorGain[(i*N_FORCE_SENSOR_PER_LINE)+j] = settings.value(str_fs_data, 1).toDouble();

			sprintf(str_fs_data,"%s/ForceSensorZero%d","sensors",((i*N_FORCE_SENSOR_PER_LINE)+j));
			forceSensorZero[(i*N_FORCE_SENSOR_PER_LINE)+j] = settings.value(str_fs_data, 0).toDouble();			
		}

		// hardware config (motor and ball screw)
		char str_hc_data [100];

		sprintf(str_hc_data,"%s/BallScrewLead%d","hardwareConfig",i);
		ballScrewLead[i] 	= settings.value(str_hc_data, 0).toDouble();

		sprintf(str_hc_data,"%s/MotorCombiConfig%d","hardwareConfig",i);
		motorCombiConfig[i] = settings.value(str_hc_data, -1).toInt();
	
	}

	/* Assign and use the values for the initialization */

	// Motors
	//the RX data is ordered in the following way
	//motor 0:  tx data message 1
	//motor 0:  tx data message 2
	//motor 1:  tx  data message 1
	//motor 1:  tx  data message 2
	// ...
	for (int i=0;i<N_DRIVERS;i++)
	{
		rxIDFields[(i*RX_FIELDS_PER_DRIVER) + 0]=motorTXID[i];
		rxIDFields[(i*RX_FIELDS_PER_DRIVER) + 1]=motorTXID[i]+1;
	}
	for (int i=0;i<RX_FIELDS_PER_DRIVER*N_DRIVERS;i++)
	{
		CANIDmap[rxIDFields[i]]=i;
	}

	// Lines
	// Hardware config
	for (int i=0;i<N_TEST_LINES;i++)
	{
		if(controlCycles[i]->updateHardwareConfig(motorCombiConfig[i],ballScrewLead[i]))
		{
			std::cout<<"Error during configuration of controlCycles["<<i<<"], stopping the program!"<<endl;
			exit(1);
		}
	}

	/* Display the config */
	/*
	for (int i=0;i<N_DRIVERS;i++)
	{
		if (motorTXID[i]>0)
			std::cout<<"motorTXID["<<i<<"] = "<<hex<<motorTXID[i]<<endl;
		if (motorRXID[i]>0)
		  	std::cout<<"motorRXID["<<i<<"] = "<<hex<<motorRXID[i]<<endl;
	}
	for (int i=0;i<RX_FIELDS_PER_DRIVER*N_DRIVERS;i++)
	{
		cout<<" rxIDFields["<<i<<"]: "<<hex<<rxIDFields[i]<<endl;
	}
	for (int i=0;i<N_TEST_LINES;i++)
	{
		// force sensors
		for (int j=0;j<N_FORCE_SENSOR_PER_LINE;j++)
		{
		std::cout<<"forceSensorGain["<<(i*N_FORCE_SENSOR_PER_LINE)+j<<"] = "<<dec<<forceSensorGain[(i*N_FORCE_SENSOR_PER_LINE)+j]<<endl;
		std::cout<<"forceSensorZero["<<(i*N_FORCE_SENSOR_PER_LINE)+j<<"] = "<<dec<<forceSensorZero[(i*N_FORCE_SENSOR_PER_LINE)+j]<<endl;
		}
		// hardware confid
		std::cout<<"motorCombiConfig["<<i<<"] = "<<dec<<motorCombiConfig[i]<<endl;
		std::cout<<"ballScrewLead["<<i<<"] = "<<dec<<ballScrewLead[i]<<endl;
	}
	*/

	cout<<dec;
}

/*
 * make a copy of current CAN bus data
 */
void MuscleDriverCANInterface::bufferCANData()
{
	mutex.lock();
	 for (int i=0;i< N_DRIVERS;i++)
	 {
		for (int j=0;j<8;j++)
		{
			motorTransmitData[i].data[j]=rxDataFields[i*RX_FIELDS_PER_DRIVER + 0][j];
			motorTransmitAuxData[i].data[j]=rxDataFields[i*RX_FIELDS_PER_DRIVER + 1][j];
		}
	 }
	 mutex.unlock();
}


void MuscleDriverCANInterface::printRxData(void)
{
	if (newDataArrived==1)
		{
			//print new data
			 for (int i=0;i<N_DRIVERS;i++)
			 {
				 std::cout<<dec;
				 std::cout<<"index: "<<i<<" pos: "<<motorTransmitData[i].s.encoderPosition<<",   omega "<<motorTransmitData[i].s.omega<<" displ: "<<motorTransmitAuxData[i].s.displacement <<"  current: "<<motorTransmitAuxData[i].s.current<<endl;
			 }
			 //reset new data flag
			 newDataArrived=0;
		}

}

/*
 * getters and setters which are used by the userinterface
 */

int  MuscleDriverCANInterface::getState(int line)
{
	return controlCycles[line]->state;
}

void MuscleDriverCANInterface::setStopCommand(int line)
{
	controlCycles[line]->stopCommand = true;
}

void MuscleDriverCANInterface::setInitCommand(int line)
{
	controlCycles[line]->initCommand = true;
}

void MuscleDriverCANInterface::setPauseCommand(int line)
{
	controlCycles[line]->pauseCommand = true;
}

void MuscleDriverCANInterface::setResumeCommand(int line)
{
	controlCycles[line]->resumeCommand = true;
}

void MuscleDriverCANInterface::setCounter(int line, long counter)
{
	controlCycles[line]->counter = counter;
}

long MuscleDriverCANInterface::getCounter(int line)
{
	return controlCycles[line]->counter;
}

double MuscleDriverCANInterface::getForceSensor(int line, int id)
{
	return controlCycles[line]->forceSensor[id];
}

/* to adapt if needed
double MuscleDriverCANInterface::getForceMax(int line)
{
	return controlCycles[line]->forceMax;
}
*/

double MuscleDriverCANInterface::getDistSensor(int line)
{
	return controlCycles[line]->distSensor;
}

double MuscleDriverCANInterface::getPreSensor(int line)
{
	return controlCycles[line]->pressureValue;
}

int16_t MuscleDriverCANInterface::getMotorDisplacement(int line)
{
	return controlCycles[line]->motorDisplacement;
}

int32_t MuscleDriverCANInterface::getMotorEncoderPosition(int line)
{
	return controlCycles[line]->motorEncoderPosition;
}

int16_t MuscleDriverCANInterface::getMotorCurrentMean(int line)
{
	return controlCycles[line]->motorCurrentMean;
}

std::string MuscleDriverCANInterface::getErrorCode(int line)
{
	return controlCycles[line]->errorCode;
}

void MuscleDriverCANInterface::setErrorCode(int line, std::string error)
{
	controlCycles[line]->errorCode = error;
}

bool MuscleDriverCANInterface::getRunParamState(int line)
{
	return controlCycles[line]->run_param_initialized;
}

void MuscleDriverCANInterface::setRunVnom(int line, double value)
{
	controlCycles[line]->vnom = value;
}

void MuscleDriverCANInterface::setRunLmax(int line, double value)
{
	controlCycles[line]->lmax = value;
}

void MuscleDriverCANInterface::updateRunTrajParam(int line)
{
	double acc, lmax, vnom, tramp, tvnom, ttot;
	double t1, t2, thp;
	int ncount;

	// base parameters
	acc 	= controlCycles[line]->acc;
	lmax 	= controlCycles[line]->lmax;
	vnom  	= controlCycles[line]->vnom;

	// derived parameters, for friction tests
	tramp 	= vnom / acc;
	tvnom	= (lmax - (vnom * tramp))/vnom;
	ttot 	= 2.0 * (tvnom + 2.0 * tramp);
	
	ncount  = (int) floor(ttot/CONTROL_LOOP_PERIOD_SEC);
	tvnom 	= ((ncount*CONTROL_LOOP_PERIOD_SEC)-(4.0*tramp))/2.0;
	t1 		= tramp;
	t2 		= tramp + tvnom;
	thp 	= (2.0 * tramp) + tvnom;

	controlCycles[line]->tramp 	= tramp;
	controlCycles[line]->tvnom 	= tvnom;
	controlCycles[line]->run_traj_period_ncount = ncount;
	controlCycles[line]->t1 	= t1;
	controlCycles[line]->t2 	= t2;
	controlCycles[line]->thp 	= thp;

	if(tvnom > 0)
	{
		controlCycles[line]->run_param_initialized =  true;
	}
	else
	{
		controlCycles[line]->run_param_initialized =  false;
		std::cout<<endl;
		std::cout<<"ERROR: tvnom is negative !!! Change paramter values:"<<endl;
	}
}

void MuscleDriverCANInterface::displayRunTrajParam(int line)
{
	std::cout<<endl;
	std::cout<<"Current parameter settings:"<<endl;
	std::cout<<"   acc:     " << controlCycles[line]->acc << endl;
	std::cout<<"   lmax:    " << controlCycles[line]->lmax << endl;
	std::cout<<"   vnom:    " << controlCycles[line]->vnom << endl;
	std::cout<<"   tramp:   " << controlCycles[line]->tramp << endl;
	std::cout<<"   tvnom:   " << controlCycles[line]->tvnom << endl;
	std::cout<<"   period:  " << controlCycles[line]->run_traj_period_ncount << endl;
	std::cout<<"   t1:      " << controlCycles[line]->t1 << endl;
	std::cout<<"   t2:      " << controlCycles[line]->t2 << endl;
	std::cout<<"   thp:     " << controlCycles[line]->thp << endl;
	std::cout<<endl;
}


void MuscleDriverCANInterface::start()
{
	cout<<"MOTOR START REQUESTED!"<<endl;
	motorDriveOn=1;
}

void MuscleDriverCANInterface::stop()
{
	cout<<"MOTOR STOP REQUESTED!"<<endl;
	motorDriveOn=0;
	m_reference=0.0;
	//close phidget force sensor
	CPhidget_close((CPhidgetHandle)bridget);
	CPhidget_delete((CPhidgetHandle)bridget);
	//close phidget dist sensor
	CPhidget_close((CPhidgetHandle)ifKit);
	CPhidget_delete((CPhidgetHandle)ifKit);
}

void MuscleDriverCANInterface::detachCAN()
{
	cout<<"CAN detached from BUS!"<<endl;
	cout<<"Restart Application to connect again."<<endl;
	motorDriveOn=0;
	outputFile.close();

	//myCan.closeCanChannel(busHandle0);
	timerThread->exit();

}
