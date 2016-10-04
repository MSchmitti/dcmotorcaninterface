/*
 * userInterface.h
 *
 *  Created on: 27 Feb 2014
 *      Author: alex
 */

#ifndef USERINTERFACE_H_
#define USERINTERFACE_H_
#include <QObject>

#include "muscleDriverCANInterface.h"

class UserInterface : public QObject
{
	Q_OBJECT
	public:
		UserInterface(MuscleDriverCANInterface* canInterface);

	private:
		MuscleDriverCANInterface* motorDriver;

	public slots:
		void run();

	private:
		int  m_MuscleDriverCANInterface;
};


#endif /* USERINTERFACE_H_ */
