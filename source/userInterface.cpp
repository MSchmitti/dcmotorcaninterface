/*
 * userInterface.cpp
 *
 *  Created on: 9 Jul 2015
 *      Author: tobias
 */

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <QTime>
#include <QCoreApplication>
#include <limits.h>


#include "userInterface.h"
#include "log.h"

using namespace std;

void delay( int millisecondsToWait )
{
    QTime dieTime = QTime::currentTime().addMSecs( millisecondsToWait );
    while( QTime::currentTime() < dieTime )
    {
        QCoreApplication::processEvents( QEventLoop::AllEvents, 100 );
    }
}

// returns an user input between min and max
long inputNumber(long min, long max)
{
	int input;
	while (!(cin>>input) || (input < min || input > max))
	{
		cout<<"Please enter a number between "<<min<<" and "<<max<<"."<<endl;
		cin.clear();
		cin.ignore(10000,'\n');
	}
	return input;
}

UserInterface::UserInterface(MuscleDriverCANInterface* canInterface)
{
	motorDriver = canInterface;
	cout<<"User Interface started:"<<endl;
}

// is executed in its own thread
void UserInterface::run()
{
	bool open = true;
	int menu = 0;
	int line = 0;
	int lineIn = -1; 
	int lineOut = -1;	
	long input = 0;
	int state = 0;
	double param_value = 0.0;
	string inputStr = "";
	// wait for the sensors to connect
	delay(200);

	while(open)
	{
		cout<<endl<<endl;
		switch (menu)
		{
			case 0: //Main Manu
				cout<<"MAIN MENU:"<<endl;
				// list all testlines
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<i + 1<<". Testline "<<i<<endl;
				}
				cout<<"    "<<N_TEST_LINES + 1<<". Status"<<endl;
				cout<<"    "<<N_TEST_LINES + 2<<". Change parameters"<<endl;
				cout<<"    "<<N_TEST_LINES + 3<<". New log file"<<endl;
				cout<<"    "<<N_TEST_LINES + 4<<". Quit"<<endl;
								
				// read user input
				input = inputNumber(1, N_TEST_LINES + 4);				
				if(input <= N_TEST_LINES)
				{
					line = input - 1;
					// switch to testline menu
					menu = 1;
				} 
				else if (input == N_TEST_LINES + 1)
				{
					// switch to status menu
					menu = 2;
				}
				else if (input == N_TEST_LINES + 2)
				{
					// select line
					// here we only have on line => line = 0
					line = 0;

					// get the state
					state = motorDriver->getState(line);
					
					if(state == STATE_RUN)
					{
						cout<<endl;
						cout<<"Cannot set params when line "<< line <<" is running"<< endl;
						
						// back to main menu
						menu = 0;
					}
					else
					{
						// switch to parameter menu
						menu = 20;
					}
				}
				else if (input == N_TEST_LINES + 3)
				{
					/* Define a new log file */
					string user_input;

					// ask for the new log file name
					cout<<endl;
					cout<<"Please input new log file name (max 30 characters):"<<endl;
					
					// read user input
					do
					{
						getline(cin,user_input);
						if(user_input.length()>29)
						{
							cout<<endl;
							cout<<"Input is empty or too long, please retry"<<endl;
							user_input = "";
						}
					}
					while(user_input.empty());
					
					setupMeasurementLog(&user_input);
					
					// back to main menu
					menu = 0;
				} 
				else
				{
					// switch to quit menu
					menu = 3;
				}
				break;

			case 1: //Test Line N
				cout<<"TestLine "<<line<<":"<<endl;
				// list available options
				state = motorDriver->getState(line);
				if (state != 0)
				{
					cout<<"    1. Stop"<<endl;
				}
				if (state == 0)
				{
					cout<<"    2. Init"<<endl;
				}
				if (state == 3)
				{
					cout<<"    3. Pause"<<endl;
				}
				if (state == 2)
				{
					cout<<"    4. Start/Resume (without log)"<<endl;
					cout<<"    5. Start/Resume (with log)"<<endl;
				}
				cout<<"    6. Start full log"<<endl;
				cout<<"    7. Reset ErrorCode"<<endl;
				cout<<"    0. Back"<<endl;
				// read user input
				input = inputNumber(0, 7);
				state = motorDriver->getState(line);
				if(input == 1 && state != 0)
				{
					motorDriver->setStopCommand(line);
				}
				if(input == 2 && state == 0)
				{
					motorDriver->setInitCommand(line);
				}
				if(input == 3 && state == 3)
				{
					motorDriver->setPauseCommand(line);
				}
				if(((input == 4)||(input == 5)) && state == 2)
				{
					// Start/Resume
					if(motorDriver->getRunParamState(line))
					{
						if(input == 4)	// without log
						{
							motorDriver->setResumeCommand(line);
						}
						else			// with log
						{
							if(requestMeasurementLog())
							{
								motorDriver->setResumeCommand(line);
							}
						}
					
					}
					else
					{
						std::cout<<endl;
						std::cout<<"Parameters have invalid values, change them first!"<< endl;
					}				
				}
				if(input == 6)
				{
					startFullLog(line,motorDriver->getCounter(line));
					cout<<"Wait 10s while logging Data ..."<<endl;
					delay(10 * 1000);
					stopFullLog();
				}
				if(input == 0)
				{
					// switch to main menu
					menu = 0;
				}
				// wait until state has changed
				delay(100);
				break;

			case 2: //Status
				cout<<"Status:"<<endl;
				// output information
				cout<<"    Testline   :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<i;
				}
				cout<<endl;
				cout<<"    State       :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<stateToString(motorDriver->getState(i));
				}
				cout<<endl;
				cout<<"    Counter     :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getCounter(i);
				}
				cout<<endl;
				cout<<"    IR Sensor   :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getDistSensor(i);
				}
				cout<<endl;
				cout<<"    Hall Sensor :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getMotorDisplacement(i);
				}
				cout<<endl;
				cout<<"    Encoder     :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getMotorEncoderPosition(i);
				}
				cout<<endl;
				for (int j = 0; j < N_FORCE_SENSOR_PER_LINE; j++)
				{
					cout<<"    ForceSensor"<<j<<":";
					for (int i = 0; i < N_TEST_LINES; i++)
					{
						cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getForceSensor(i,j);
					}
					cout<<endl;
				}	
				cout<<"    PressureSensor:";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"  "<<setfill(' ')<<setw(10)<<motorDriver->getPreSensor(i);
				}
				cout<<endl;	
				/* to adapt if needed
				cout<<"    ForceMax    :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getForceMax(i);
				}
				cout<<endl;
				*/
				cout<<"    Mean Current:";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getMotorCurrentMean(i);
				}
				cout<<endl;
				cout<<"    ErrorCode  :";
				for (int i = 0; i < N_TEST_LINES; i++)
				{
					cout<<"    "<<setfill(' ')<<setw(10)<<motorDriver->getErrorCode(i);
				}
				cout<<endl;
				cout<<"Type in 0 to go back."<<endl;
				// read direct user input
				cin>>inputStr;
				if (inputStr.compare("0") == 0)
				{
					// switch to main menu
					menu = 0;
				}
				break;

			case 3: //Quit
				cout<<"QUIT:"<<endl;
				cout<<"    Enter [YES] to Quit."<<endl;
				// read direct user input
				cin >> inputStr;
				if (inputStr.compare("YES") == 0)
				{
					cout<<"ending CAN application"<<endl;
					motorDriver->stop();
					delay(200);
					motorDriver->detachCAN();
					open = false;
				} 
				else 
				{
					menu = 0;
				}
				break;

			case 10: //Set Counter
				cout<<"Set counter for line "<<line<<" :"<<endl;
				// read user input
				input = inputNumber(0, LONG_MAX);
				motorDriver->setCounter(line, input);
				// switch back to testline menu
				menu = 1;
				break;

			case 20: //Parameter menu
				
				// list available options
				// for the moment, only possible to change vnom
				std::cout<<"PARAMETER MENU"<<endl;
				std::cout<<"    1. Change vnom"<<endl;
				std::cout<<"    2. Change lmax"<<endl;
				std::cout<<"    3. Display all parameters"<<endl;
				std::cout<<"    0. Go back to main menu"<<endl;

				// read user input
				input = inputNumber(0, 3);
				
				if(input == 1)
				{
					// change vnom
					std::cout<<endl;
					std::cout<<"Please input value for vnom (in mm/s):"<<endl;

					while (!(cin>>param_value) || (param_value < 0.0))
					{
						cout<<"Please enter a positive value"<<endl;
						cin.clear();
						cin.ignore(10000,'\n');
					}
					motorDriver->setRunVnom(line,param_value);
					motorDriver->updateRunTrajParam(line);
					motorDriver->displayRunTrajParam(line);
				}
				else if(input == 2)
				{
					// change l_max
					std::cout<<endl;
					std::cout<<"Please input value for lmax (in mm):"<<endl;

					while (!(cin>>param_value) || (param_value < 0.0))
					{
						cout<<"Please enter a positive value"<<endl;
						cin.clear();
						cin.ignore(10000,'\n');
					}
					motorDriver->setRunLmax(line,param_value);
					motorDriver->updateRunTrajParam(line);
					motorDriver->displayRunTrajParam(line);
				}
				else if(input == 3)
				{
					// display all trajectory parameters
					motorDriver->displayRunTrajParam(line);
				}
				else
				{
					// switch to main menu
					menu = 0;
				}
				break;
		}
	}
	stopLog();
	// kill aplication
	exit(0);
}

