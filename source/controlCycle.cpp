/*
 * controlCycle.cpp
 *
 *  Created on: 8 Jul 2015
 *      Author: tobias
 */
#include <math.h> 
#include <stdio.h>

#include "controlCycle.h"
#include "log.h"

// Tecchnical data
#define EC30_48V_LOWGEAR	1
#define EC40_24V_LOWGEAR	2
#define MOTOR_DUTY_CYCLE 	4000
#define MOTOR_MAX_VOLTAGE 	24		// V

// RUN trajectory parameters
#define RUN_DIRECTION 		1
// friction test
#define RUN_NUM_CYCLES	 	1		// number of cycles for one measurement
#define RUN_TRAJ_LIN_ACC	100.0	// default linear acceleration, in mm/s^2
#define RUN_TRAJ_L_MAX		10.0	// default maximum way, in mm
#define RUN_TRAJ_LIN_VNOM	5.0		// default nominal speed, in mm/s

// INIT min/max target position and speed
#define INIT_DIST_MIN 5.0
#define INIT_DIST_MAX 5.5
#define INIT_DUTY_CYCLE 400			// automatic initialization
#define INIT_FORCE_MIN -0.8			// there is a difference between pushing an pulling the force sensor
#define INIT_FORCE_MAX  0.5

// STOP conditions
#define CABLE_BREAK_MIN_FORCE 350
#define MAX_FORCE 850
#define MIN_FORCE -850
#define MAX_MEAN_CURRENT 400

// PD-Controller
#define KP 0.25
#define KD 0.0
#define MIN_MOTOR_DUTY_CYCLE 100
#define MAX_DUTY_CYCLE 4000

std::string stateToString(int state)
{
    switch (state)
    {
        case STATE_STOP:
            return "Stop";
        case STATE_INIT:
            return "Init";
        case STATE_PAUSE:
            return "Pause";
        case STATE_RUN:
            return "Run";
    }
}

ControlCycle::ControlCycle(int i)
{
	line = i;
	state = STATE_STOP;
	errorCode = "";
	counter = 0;
	olderror = 0;
	motorCurrentSum = 0;
	motorCurrentMean = 0;
	initDistFin = 0;			//<---- NEW  //printf("initDistFin %d \n", initDistFin);
	
	stopCommand = false;
    	initCommand = false;
    	pauseCommand = false;
    	resumeCommand = false;

    // Tecchnical data
    	motorSpeedConstant 	= 1.0; 			// RPM/V
	gearingTranslation 	= 1.0;			
	encoderCountPerRev 	= 1;
	ballScrewLead 		= 1.0;			// mm
	k_lin2rot		= 1.0;			// enc_count / mm

	// Trajectories
	run_param_initialized = false;
	run_traj_period_ncount = 1;
	xlin = 0.0;
	vlin = 0.0;
	// for friction tests (ramps)
	acc     = RUN_TRAJ_LIN_ACC;			// in mm/s^2
	lmax    = RUN_TRAJ_L_MAX;			// in mm
	vnom 	= RUN_TRAJ_LIN_VNOM;		// in mm/s
	tramp 	= 0.0;
	tvnom	= 0.0;
	t1 		= 0.0;
	t2 		= 0.0;
	thp 	= 0.0;
	
	// PID parameters
	pidKp = KP;
	pidKd = KD;
}

int ControlCycle::updateHardwareConfig(int motorCombiConfig, double BSLead)
{
	// Ball screw
	if(BSLead < 0)
	{
		std::cout<<"[Error] ControlCycle[" << line <<"]: value for ball screw lead is negative" << endl;
		return 1;
	}
	else
	{
		ballScrewLead = BSLead;
	}

	// Motor combi
	switch (motorCombiConfig)
	{
  		case EC30_48V_LOWGEAR:
     		encoderCountPerRev 	= 2000;
			gearingTranslation 	= 4.3;			
			motorSpeedConstant 	= 346.0; 			// RPM/V
	   		break;
  		case EC40_24V_LOWGEAR:
    		encoderCountPerRev 	= 2000;
			gearingTranslation 	= 4.3;			
			motorSpeedConstant 	= 412.0; 			// RPM/V
     		break;
  		default:
     		std::cout<<"[Error] ControlCycle[" << line <<"]: unknown motor combination id" << endl;
			return 1;
	}

	k_lin2rot = encoderCountPerRev * gearingTranslation / ballScrewLead ; // enc_count /mm

	// Display
	std::cout<<"Config of ControlCycle[" << line <<"]:" << endl;
	std::cout<<"motorSpeedConstant: " 	<< motorSpeedConstant		<< endl;
	std::cout<<"gearingTranslation:" 	<< gearingTranslation		<< endl;
	std::cout<<"encoderCountPerRev:" 	<< encoderCountPerRev 		<< endl;
	std::cout<<"ballScrewLead:" 		<< ballScrewLead 			<< endl;
	std::cout<<"k_lin2rot:"	<< k_lin2rot	<< endl;
		
	return 0;
}

void ControlCycle::PDcontroller(double targetPosition, double targetSpeed)
{
	// Feed forward
	motorDutyCycle = (targetSpeed * 60 / encoderCountPerRev) * 
		MOTOR_DUTY_CYCLE / (MOTOR_MAX_VOLTAGE * motorSpeedConstant);

	// PD controller
	int32_t error = targetPosition - motorEncoderPosition;
	motorDutyCycle += pidKp * error + (pidKd * (error - olderror) / CONTROL_LOOP_PERIOD_SEC);
	olderror = error;

	// MIN_MOTOR_DUTY_CYCLE
	if(motorDutyCycle < 0)
	{
		motorDutyCycle -= MIN_MOTOR_DUTY_CYCLE;
	}
	else
	{
		motorDutyCycle += MIN_MOTOR_DUTY_CYCLE;
	}

	// motorDutyCycle limitation
	if (motorDutyCycle > MAX_DUTY_CYCLE)
	{
		motorDutyCycle = MAX_DUTY_CYCLE;
	}
	else if (motorDutyCycle < -MAX_DUTY_CYCLE)
	{
		motorDutyCycle = -MAX_DUTY_CYCLE;
	}
}

// Trajectories
// Trajectory for friction test (ramps)
void ControlCycle::traj_lin_friction(double tcur)
{
	int dir;
	double x_offset, x1, x2;

	// check if currently in the positive or negative direction
	if(tcur < thp)
	{
		dir 		= -1;
		x_offset  	= 0.0;
	}
	else
	{	
		tcur   		= tcur - thp;
		dir 		= 1;
		x_offset  	= -lmax;
	}

	// intermediate positions
	x1 = acc * pow(tramp,2.0) / 2.0;
	x2 = vnom*tvnom;

	// compute speed and position accordingly
	if(tcur < t1)
	{
		xlin = x_offset + dir * ((acc * pow(tcur,2.0)) / 2.0);
		vlin = dir * acc * tcur;
	}
	else if(tcur<t2)
	{
		tcur = tcur - t1;
		xlin = x_offset + dir * (x1 + vnom*tcur);
		vlin = dir * vnom;
	}
	else
	{
		tcur = tcur - t2;
		xlin = x_offset + dir * (x1 + x2 + (vnom*tcur) - ((acc * pow(tcur,2.0)) / 2.0));
		vlin = dir * (vnom -(acc * tcur));
	}
}


// the main controlCycle for one testline
// this function is called every 5ms
void ControlCycle::cycle()
{
	/*
	 * a basic Statemachine whith 4 States:
	 *		STOP, INIT, PAUSE, RUN
	 * STOP: do nothing
	 * INIT: use the distSensor to reach an init position
	 * PAUSE: keep init position
	 * RUN: pull the cable in a sine wave
	 */
	switch (state)
	{
		case STATE_STOP: //Stop
			// do nothing
			motorDutyCycle = 0;

			if(initCommand)
			{
				// switch state
				state = STATE_INIT;
				// reset Flag
				initCommand = false;
				initDistFin = 0;
				// log state change
			}
			break;

		case STATE_INIT: //Init
			// aim for init position
			if ((distSensor < (INIT_DIST_MIN - 0.5)) && (initDistFin == 0))
				{
					motorDutyCycle = INIT_DUTY_CYCLE;
				} 

			else if ((distSensor < INIT_DIST_MIN ) && (initDistFin == 0))
				{
					motorDutyCycle = (INIT_DUTY_CYCLE - 100);
				}	 

			else if ((distSensor > (INIT_DIST_MAX + 0.5)) && (initDistFin == 0))
				{
					motorDutyCycle = -INIT_DUTY_CYCLE;
				}
			// to stop after arraving the border
			else if ((distSensor > (INIT_DIST_MAX - 0.03)) && (initDistFin == 0))		
				{
					motorDutyCycle = -(INIT_DUTY_CYCLE - 0);
				}
			else if ((distSensor > INIT_DIST_MIN) && (distSensor < INIT_DIST_MAX) && (initDistFin == 0))
				{	
					//Init Dist
					initDistFin = 1;
					forceInit = forceSensor[0];
					currentForce = forceSensor[0];
				}			

			else if ((currentForce < (forceInit + INIT_FORCE_MAX)) && (currentForce > (forceInit + INIT_FORCE_MIN)) && (initDistFin == 1))
				{
						motorDutyCycle = -300;
						currentForce = forceSensor[0];

				}
			else if((currentForce > (forceInit + INIT_FORCE_MAX)) || (currentForce < (forceInit + INIT_FORCE_MIN)) && (initDistFin == 1))
				{				
						initForcePos = motorEncoderPosition/4300;
						initDistFin = 2;
				}
			//pressure is negative
			else if (currentForce < -10)			
				{
					motorDutyCycle = 0;
				}
			else if (currentForce > 10)
				{
					motorDutyCycle = 0;
				}			
			else if (initDistFin == 2 && ((motorEncoderPosition/4300) < (initForcePos + 1) ))	
				{		
					motorDutyCycle = 300;
				} 
			else
				{	
				//Init completed
				motorDutyCycle = 0;
				motorInitPosition = motorEncoderPosition;
				currentForce = forceSensor[0];
				state = STATE_PAUSE;
				}
			
			if (stopCommand)
			{
				state = STATE_STOP;
				stopCommand = false;
			}
			break;

		case STATE_PAUSE: //Pause
			// keep init position
			PDcontroller(motorInitPosition, 0);
			
			if (resumeCommand)
			{
				motorCycleOffset = m_cycleCount;
				
				// Initialize the variables for the trajectories
				// general variable
				xlin = 0.0;
				vlin = 0.0;
				
				// initialize the variables for computing averaged values 
				motorCurrentSum = 0;

				// switch to state run
				state = STATE_RUN;
				
				resumeCommand = false;
			}
			if (stopCommand)
			{
				state = STATE_STOP;
				stopCommand = false;
			}
			break;

		case STATE_RUN: //Run
			// recalculate m_cycleCount to count from 0 to RUN_PERIOD
			m_cycleCount = (m_cycleCount - motorCycleOffset) % run_traj_period_ncount;
			
			// Calculate the linear target position and speed
			// for friction tests (ramps) 
			traj_lin_friction(m_cycleCount * CONTROL_LOOP_PERIOD_SEC);

			// Compute the corresponding motor rot. position and speed
			targetPos 	= motorInitPosition + RUN_DIRECTION * k_lin2rot * xlin;
			targetSpeed	= RUN_DIRECTION * k_lin2rot * vlin;
			
			// Generate motor command
			PDcontroller(targetPos, targetSpeed);
			
			// update motor current-related variables
			motorCurrentSum += motorCurrent;

					
			/* TRANSITIONS TO PAUSE STATE */
			// only if run cycle is over
			if (m_cycleCount == 0)
			{
				// check if achieved desired number of cycles
				counter++;
				if(counter==RUN_NUM_CYCLES)
				{
					state = STATE_PAUSE;
					counter = 0;
				}
				// check if mean current too high (overheating)
				motorCurrentMean = motorCurrentSum / run_traj_period_ncount;
				if(motorCurrentMean > MAX_MEAN_CURRENT)
				{
					state = STATE_PAUSE;
					errorCode = "Mean current too high";
				}
				motorCurrentSum = 0;
				// check if incoming pause command
				if(pauseCommand)	
				{
					state = STATE_PAUSE;
					pauseCommand = false;
				}				
			}

			/* TRANSITIONS TO STOP STATE */
			// check if too high force
			if (forceSensor[0] >= MAX_FORCE)
			{
				state = STATE_STOP;
				errorCode = "Force to high";
			}
			//pressure is negative
			if (forceSensor[0] <= MIN_FORCE)
			{
				state = STATE_STOP;
				errorCode = "Force to high";
			}
			if (forceSensor[1] >= MAX_FORCE)
			{
				state = STATE_STOP;
				errorCode = "Force to high";
			}
			if (forceSensor[1] <= MIN_FORCE)
			{
				state = STATE_STOP;
				errorCode = "Force to high";
			}
			// check if incoming stop command
			if (stopCommand)
			{
				state = STATE_STOP;
				stopCommand = false;
			}
			break;

		default:
			state = STATE_STOP;
			break;
	}
}
