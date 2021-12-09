/*
  kinematics_RS.h - Library for calculating kinematics.
  Created by Risnanda S., January 13, 2020.
  Released into the public domain.
*/
#ifndef kinematics_RS_H
#define kinematics_RS_H

#include "Arduino.h"

class kinematicAUMR_RS{
	public:
		kinematicAUMR_RS(int encoderLeftA, int encoderLeftB, int encoderRightA, int encoderRightB, float wheelRadius, float rangeBetweenWheels, float encoderPPR);
		// Timer process
		void checkEncoderTimer();
		void checkLoopTimer();
		
		// Calculate process
		void convertEncoderToPositionAndSpeed();
		void calculate();
		void calculatePosition();
		
		// Encoding process
		void processLeftForward();
		void processLeftBackward();
		void processRightForward();
		void processRightBackward();
		
		// Get data from function
		float getXPositionInCM();
		float getYPositionInCM();
		int getThetaInDegree();
		float getLeftPositionInCM();
		float getRightPositionInCM();
		float getLeftSpeed();
		float getRightSpeed();
	private:
		// define phi
		#define _phi 3.14285714286
		// pin variable
		int	_encoderLeftA,
			_encoderLeftB,

			_encoderRightA,
			_encoderRightB;

		// encoder variable
		int	_leftCounter,
			_rightCounter;
			
		float 	_leftLinearSpeed,
				_rightLinearSpeed,
      
				_lastLeftLinearSpeed,
				_lastRightLinearSpeed,
        
				_leftPosition,
				_rightPosition,
        
				_lastLeftPosition,
				_lastRightPosition;
				
		// forward kinematics variable
		float 	_wheelRadius,
				_rangeBetweenWheels,
				_encoderPPR,
				_encoderCPR,

				_xPosition,
				_yPosition,
				_thetaPosition,

				_lastXPosition,
				_lastYPosition,
				_lastThetaPosition = _phi/2,
				
				_xPositionInCM,
				_yPositionInCM,
				_thetaPositionInDegree,

				_Iccx,
				_Iccy,
				_radiusIcc,
				_angularSpeed,
				_loopTimer;
				
		// millis variable
		unsigned long 	_startEncoderMillis,      // Millis for checking left loop timer
						_endEncoderMillis,
						_encoderTimer,
						
						_startTimeMillis,      // Millis for checking loop timer
						_endTimeMillis,
						_loopTimerCheck;
};

class kinematicDOPER_RS{
	public:
		kinematicDOPER_RS(int encoderOneA, int encoderOneB, int encoderTwoA, int encoderTwoB, 
						  int encoderThreeA, int encoderThreeB, int encoderFourA, int encoderFourB,
						  float wheelRadius, float rangeBetweenWheels, float encoderPPR);
		// Timer process
		void checkEncoderTimer();
		void checkLoopTimer();
		
		// Calculate process
		void convertEncoderToPositionAndSpeed();
		void calculate();
		void calculatePosition();
		
		// Encoding process
		void processOneForward();
		void processOneBackward();
		void processTwoForward();
		void processTwoBackward();
		void processThreeForward();
		void processThreeBackward();
		void processFourForward();
		void processFourBackward();
		
		// Get data from function
		float getXPositionInCM();
		float getYPositionInCM();
		int getThetaInDegree();
		float getLeftPositionInCM();
		float getRightPositionInCM();
		float getLeftSpeed();
		float getRightSpeed();
	private:
		// define phi
		#define _phi 3.14285714286
		// pin variable
		int	_encoderOneA,
			_encoderOneB,
			
			_encoderTwoA,
			_encoderTwoB,

			_encoderThreeA,
			_encoderThreeB,
			
			_encoderFourA,
			_encoderFourB;
			
		// encoder variable
		int	_oneCounter,
			_twoCounter,
			_threeCounter,
			_fourCounter;
			
		float 	_leftLinearSpeed,
				_rightLinearSpeed,
      
				_lastLeftLinearSpeed,
				_lastRightLinearSpeed,
        
				_leftPosition,
				_rightPosition,
        
				_lastLeftPosition,
				_lastRightPosition;
				
		// forward kinematics variable
		float 	_wheelRadius,
				_rangeBetweenWheels,
				_encoderPPR,
				_encoderCPR,

				_xPosition,
				_yPosition,
				_thetaPosition,

				_lastXPosition,
				_lastYPosition,
				_lastThetaPosition = _phi/2,
				
				_xPositionInCM,
				_yPositionInCM,
				_thetaPositionInDegree,

				_Iccx,
				_Iccy,
				_radiusIcc,
				_angularSpeed,
				_loopTimer;
				
		// millis variable
		unsigned long 	_startEncoderMillis,      // Millis for checking left loop timer
						_endEncoderMillis,
						_encoderTimer,
						
						_startTimeMillis,      // Millis for checking loop timer
						_endTimeMillis,
						_loopTimerCheck;
};

#endif