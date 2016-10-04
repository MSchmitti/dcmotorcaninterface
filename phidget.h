#ifndef PHIDGET_H_
#define PHIDGET_H_

#include <stdio.h>
#include <phidget21.h>//phidget lib

double cmval[8];//global table with values of dist sensors
double nval[4];//global table with values of force sensors from bridge

//Declare an InterfaceKit and bridge handle
CPhidgetInterfaceKitHandle ifKit = 0;
CPhidgetBridgeHandle bridget;

/***********************PHIDGETS FUNCTIONS***************************/

/**For distance sensor**/
int CCONV AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	//printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int CCONV DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	//printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	/*
	if (ErrorCode == 36866)
	{
		return 0;
	}
	*/
	printf("Error handled. %d - %s", ErrorCode, unknown);
	return 0;
}

// NOT USED
//callback that will run if an input changes.
//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
int CCONV InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	//printf("Digital Input: %d > State: %d\n", Index, State);
	return 0;
}

// NOT USED
//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int CCONV OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	//printf("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int index, int value)
{
	//printf("Sensor: %d > Value: %d\n", index, value);
	
	// raw output
	//cmval[index] = value;
	
	// processed output
	//(using formula from http://www.phidgets.com/docs/1101_User_Guide)
	double prev_cmval;
	prev_cmval = cmval[index];
	// formula valid between 80 and 530
	if(value<80)
		value = 80;
	else if(value>530)
		value = 530;
	// formula
	cmval[index]=2076.0/(value-11.0);
	
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also 	display the number of inputs, outputs, and analog inputs on the interface kit as well as the state of the ratiometric flag
//and the current analog sensor sensitivity.
int display_properties(CPhidgetInterfaceKitHandle phid)
{
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	printf("# Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);
		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}
	
	return 0;
}


/*** HANDLE FUNCTIONS FOR PHIDGET BRIDGE (FORCE SENSOR) ***/
int CCONV AttachHandlerF(CPhidgetHandle phid, void *userptr)
{
	CPhidgetBridgeHandle bridge = (CPhidgetBridgeHandle)phid;
	int rate_max = 0;
	int rate_min = 0; 

	// enable all four channels
	CPhidgetBridge_setEnabled(bridge, 0, PTRUE);
	CPhidgetBridge_setEnabled(bridge, 1, PTRUE);
	CPhidgetBridge_setEnabled(bridge, 2, PTRUE);
	CPhidgetBridge_setEnabled(bridge, 3, PTRUE);

	// set the gains (1, 8, 16, 32, 64 or 128) for all channels
	// note: higher gains have higher resolution and lower noise but smaller range 
	CPhidgetBridge_setGain(bridge, 0, PHIDGET_BRIDGE_GAIN_128);
	CPhidgetBridge_setGain(bridge, 1, PHIDGET_BRIDGE_GAIN_128);
	CPhidgetBridge_setGain(bridge, 2, PHIDGET_BRIDGE_GAIN_128);
	CPhidgetBridge_setGain(bridge, 3, PHIDGET_BRIDGE_GAIN_128);
	
	/*
	// display the min and max data rate
	CPhidgetBridge_getDataRateMax(bridge, &rate_max);
	CPhidgetBridge_getDataRateMin(bridge, &rate_min);
	printf("Phidget bridge: rate min = %d, rate max = %d\n",rate_min,rate_max);
	*/

	// set the data rate for the bridge
	// note: max rate = 8ms, min rate = 1000ms, only multiple of 8ms allowed
	// ==> we set to max data rate = 8ms 
	CPhidgetBridge_setDataRate(bridge, 8);

	//printf("Phidget bridge: attached!\n");
	return 0;
}

int CCONV DetachHandlerF(CPhidgetHandle phid, void *userptr)
{
	//printf("Phidget bridge: detached!\n");
	return 0;
}

int CCONV ErrorHandlerF(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *errorStr)
{
	printf("Phidget bridge: Error event: %s\n",errorStr);
	return 0;
}

// store the sensor values to gloabl structure nval
// called periodically, at a frequency defined by "CPhidgetBridge_setDataRate" 
int CCONV data(CPhidgetBridgeHandle phid, void *userPtr, int index, double val)
{
	nval[index] = val;
	return 0;
}

//Display the properties of the attached bridge to the screen
void display_properties_bridge(CPhidgetHandle phid)
{
	int sernum, version;
	const char *deviceptr;
	CPhidget_getDeviceType(phid, &deviceptr);
	CPhidget_getSerialNumber(phid, &sernum);
	CPhidget_getDeviceVersion(phid, &version);

	printf("%s\n", deviceptr);
	printf("Version:      %10d\nSerialNumber: %10d\n", version, sernum);
	return;
}

/*** END: HANDLE FUNCTIONS FOR PHIDGET BRIDGE (FORCE SENSOR) ***/


#endif /* PHIDGET_H_ */
