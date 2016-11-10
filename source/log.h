/*
 * log.h
 *
 *  Created on: 10 Jul 2015
 *      Author: tobias
 */

#ifndef LOG_H_
#define LOG_H_

#include <QObject>
#include <stdio.h>
#include <fstream>

#include "muscleDriverCANInterface.h"

	void startLog();
	void stopLog();
	void startFullLog(int line, long line_counter);
	void stopFullLog();
	void startEffTestLog(int lineIn, int lineOut);
	void stopEffTestLog();

	//void logFullDataSet(int line, unsigned int m_cycleCount, 
	//	int32_t motorEncoderPosition, float motorOmega, int16_t motorDisplacement, 
	//	int16_t motorCurrent, double forceSensor, double distSensor, float motorDutyCycle, double targetPos);
	void logFullDataSet(int line, unsigned int m_cycleCount,
		double xlin, double vlin, double targetPos, double targetSpeed,
		int32_t motorEncoderPosition, double forceSensor0, double forceSensor1, 
		float motorDutyCycle, double pressureValue);

	void setupMeasurementLog(string* filename);
	int isReadyMeasurementLog();
	int isRequestedMeasurementLog();
	int isInProgressMeasurementLog();
	int requestMeasurementLog();
	void startMeasurementLog();
	void stopMeasurementLog();
	void performMeasurementLog(unsigned int m_cycleCount, double targetPos, int32_t motorEncoderPos, double forceSensor, int16_t posSensor, double pressureValue);

#endif /* LOG_H_ */
