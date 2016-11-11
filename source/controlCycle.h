/*
 * controlCycle.h
 *
 *  Created on: 8 Jul 2015
 *      Author: tobias
 */

#ifndef CONTROLCYCLE_H_
#define CONTROLCYCLE_H_

#include <QObject>
#include <stdio.h>

// Setup configuration
#define N_TEST_LINES            1
#define N_DRIVERS_PER_LINE      1
#define N_FORCE_SENSOR_PER_LINE 2
#define N_DRIVERS               N_TEST_LINES*N_DRIVERS_PER_LINE
#define N_FORCE_SENSORS         N_TEST_LINES*N_FORCE_SENSOR_PER_LINE

//#define CONTROL_LOOP_PERIOD         5       // in ms
#define CONTROL_LOOP_PERIOD_SEC     0.005   // in s

// Setup states
#define STATE_STOP 0
#define STATE_INIT 1
#define STATE_PAUSE 2
#define STATE_RUN 3


std::string stateToString(int state);

class ControlCycle : public QObject
{

	public:
    	ControlCycle(int line);

        void PDcontroller(double targetPosition, double targetSpeed);
    	void cycle();
        int updateHardwareConfig(int motorCombiConfig, double BSLead);

        // Trajectory functions
        void traj_lin_friction(double time);

        // line parameters
        int line;
        int state;
        long counter;
        unsigned long m_cycleCount;

        // hardware config
        int32_t encoderCountPerRev;
        float gearingTranslation;            
        float ballScrewLead;        // in mm
        float motorSpeedConstant;   // in RPM/V
        float k_lin2rot;            // enc_count per mm

        //Flags
        bool stopCommand;
        bool initCommand;
        bool pauseCommand;
        bool resumeCommand;

        /* SENSORS */
        // motor driver - motor sensors
        int32_t motorEncoderPosition;
	    float   motorOmega;
	    int16_t motorCurrent;
        int32_t motorCurrentSum;
        int16_t motorCurrentMean;
        // motor driver - diplacement sensor
        int16_t motorDisplacement;
        // phidget force sensors
        double forceSensor[N_FORCE_SENSOR_PER_LINE];
        /* to adapt if needed
        double forceSumm;
        double forceMax;
        double forceMaxCur;
        */
        // phidget distance sensor
        double distSensor; 
	
	    // pressure sensor
	    double pressureValue;

    	//Outputs
    	float motorDutyCycle;
        std::string errorCode;

        /* Trajectory generation and control */
        
        // General
        int32_t         motorInitPosition; 
        unsigned long   motorCycleOffset;
	    int             initDistFin;       
        double          initForcePos;       
        double          forceInit;                  
        double          currentForce;     
	
        // Trajectories
        bool    run_param_initialized;
        int     run_traj_period_ncount;     // nb of count for one period (1 count = 5ms)
        double  xlin;           
        double  vlin;
        // for friction tests (ramps)
        double acc;         // unit: mm/s^2
        double lmax;        // unit: mm
        double vnom;        // unit: mm/s
        double tramp;       // unit: s
        double tvnom;       // unit: s
        double t1;          // unit: s
        double t2;          // unit: s
        double thp;         // unit: s
        // sine wave
        //double run_SinWave_Amplitude;
        //double omega;

        // PD controller 
        // target values and memories
        double targetPos;       // in enc_count
        double targetSpeed;     // in enc_count / 
        float olderror;
        // gains parameters
        float pidKp;
        float pidKd;

};

#endif /* CONTROLCYCLE_H_ */
