/*
  kinematics_RS.h - Library for calculating kinematics.
  Created by Risnanda S., January 13, 2020.
  Released into the public domain.
*/
#include "Arduino.h"
#include "kinematics_RS.h"

kinematicAUMR_RS::kinematicAUMR_RS(int encoderLeftA, int encoderLeftB, int encoderRightA, int encoderRightB, float wheelRadius, float rangeBetweenWheels, float encoderPPR){
	_encoderLeftA = encoderLeftA;
	_encoderLeftB = encoderLeftB;
	
	_encoderRightA = encoderRightA;
	_encoderRightB = encoderRightB;
	
	_wheelRadius = wheelRadius;
	_rangeBetweenWheels = rangeBetweenWheels;
	_encoderPPR = encoderPPR;
	_encoderCPR = 4*encoderPPR;
}

void kinematicAUMR_RS::checkEncoderTimer(){
    _endEncoderMillis = millis();
    _encoderTimer = (_endEncoderMillis - _startEncoderMillis);
    _startEncoderMillis = millis();
}

void kinematicAUMR_RS::checkLoopTimer(){
    _endTimeMillis = millis();
    _loopTimerCheck = _endTimeMillis - _startTimeMillis;
    _startTimeMillis = millis();
	_loopTimer = (float)_loopTimerCheck/1000;
}

void kinematicAUMR_RS::convertEncoderToPositionAndSpeed(){
	// Convert Position
    _leftPosition = (2*_phi*_wheelRadius*_leftCounter)/_encoderCPR;
    _rightPosition = (2*_phi*_wheelRadius*_rightCounter)/_encoderCPR;
  
    // Check timer to calculate the speed
    checkEncoderTimer();

    // Convert Speed
    _leftLinearSpeed = (_leftPosition - _lastLeftPosition)*1000/(_encoderTimer);
    _rightLinearSpeed = (_rightPosition - _lastRightPosition)*1000/(_encoderTimer);

    // Save last variable
    _lastLeftPosition = _leftPosition;
    _lastRightPosition = _rightPosition;
}

float kinematicAUMR_RS::getLeftPositionInCM(){
	return _leftPosition;
}
float kinematicAUMR_RS::getRightPositionInCM(){
	return _rightPosition;
}
float kinematicAUMR_RS::getLeftSpeed(){
	return _leftLinearSpeed;
}
float kinematicAUMR_RS::getRightSpeed(){
	return _rightLinearSpeed;
}
float kinematicAUMR_RS::getXPositionInCM(){
	return _xPositionInCM;
}
float kinematicAUMR_RS::getYPositionInCM(){
	return _yPositionInCM;
}
int kinematicAUMR_RS::getThetaInDegree(){
	return _thetaPositionInDegree;
}

void kinematicAUMR_RS::calculate(){
	kinematicAUMR_RS::checkLoopTimer();
	kinematicAUMR_RS::convertEncoderToPositionAndSpeed();
	kinematicAUMR_RS::calculatePosition();
}

void kinematicAUMR_RS::calculatePosition(){
    // Calculate with the equation, every condition means different equation
    if( (_leftLinearSpeed != _rightLinearSpeed) && ((_leftLinearSpeed > 0 && _rightLinearSpeed > 0) || (_leftLinearSpeed < 0 && _rightLinearSpeed < 0)) ){
		// CIRCULAR MOTION
		// Calculate w
		_angularSpeed = (_leftLinearSpeed-_rightLinearSpeed)/_rangeBetweenWheels;

		// Calculate radiusIcc
		_radiusIcc = (_rangeBetweenWheels*(_leftLinearSpeed+_rightLinearSpeed))/(2*(_leftLinearSpeed-_rightLinearSpeed));

		// Calculate position in x
		_xPosition = -_radiusIcc*sin(_lastThetaPosition) + _radiusIcc*sin(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastXPosition;
		_yPosition = _radiusIcc*cos(_lastThetaPosition) - _radiusIcc*cos(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastYPosition;
		_thetaPosition = _lastThetaPosition + _angularSpeed*_loopTimer;

	} else if( (_leftLinearSpeed == _rightLinearSpeed) ){ 
		// LINEAR MOTION
		// w and radiusIcc is not defined

		// Calculate position in x
		_xPosition = _lastXPosition + _leftLinearSpeed*_loopTimer*cos(_lastThetaPosition);
		_yPosition = _lastYPosition + _leftLinearSpeed*_loopTimer*sin(_lastThetaPosition);
		_thetaPosition = _lastThetaPosition;

    } else if( (_leftLinearSpeed != -_rightLinearSpeed) && ( ((_leftLinearSpeed > 0) && (_rightLinearSpeed < 0)) || ((_leftLinearSpeed < 0) && (_rightLinearSpeed > 0)) ) ){
		// ROTATION MOTION WITH SMALL ERROR
		// Calculate w
		_angularSpeed = (_leftLinearSpeed-_rightLinearSpeed)/_rangeBetweenWheels;

		// Calculate radiusIcc
		_radiusIcc = (_rangeBetweenWheels*(_leftLinearSpeed+_rightLinearSpeed))/(2*(_leftLinearSpeed-_rightLinearSpeed));

		// Calculate position in x
		_xPosition = -_radiusIcc*sin(_lastThetaPosition) + _radiusIcc*sin(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastXPosition;
		_yPosition = _radiusIcc*cos(_lastThetaPosition) - _radiusIcc*cos(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastYPosition;
		_thetaPosition = _lastThetaPosition + _angularSpeed*_loopTimer;

    } else if( (_leftLinearSpeed == -_rightLinearSpeed) ){
		// SMOOTH ROTATION MOTION
		// w and radiusIcc is not needed in x position when smooth rotation
		_angularSpeed = (_leftLinearSpeed-_rightLinearSpeed)/_rangeBetweenWheels;

		// Calculate position in x
		_xPosition = _lastXPosition;
		_yPosition = _lastYPosition;
		_thetaPosition = _lastThetaPosition + _angularSpeed*_loopTimer;

    } else if( ((_leftLinearSpeed == 0) && (_rightLinearSpeed != 0)) || ((_leftLinearSpeed != 0) && (_rightLinearSpeed == 0)) ){
		// ROTATION MOTION WITH CENTER ROTATION AT EACH WHEEL
		// Calculate w
		_angularSpeed = (_leftLinearSpeed-_rightLinearSpeed)/_rangeBetweenWheels;

		// Calculate radiusIcc
		_radiusIcc = 0.5;

		// Calculate position in x
		_xPosition = -_radiusIcc*sin(_lastThetaPosition) + _radiusIcc*sin(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastXPosition;
		_yPosition = _radiusIcc*cos(_lastThetaPosition) - _radiusIcc*cos(_lastThetaPosition + _angularSpeed*_loopTimer) + _lastYPosition;
		_thetaPosition = _lastThetaPosition + _angularSpeed*_loopTimer;
    }
	
	// Save the last variable
    _lastXPosition = _xPosition;
	_lastYPosition = _yPosition;
	_lastThetaPosition = _thetaPosition;

    // Save in the variable sent later
    _xPositionInCM = _xPosition;
	_yPositionInCM = _yPosition;
	_thetaPositionInDegree = ((int)(_thetaPosition*360/(2*_phi))%360) >=0? ((int)(_thetaPosition*360/(2*_phi))%360): 360+((int)(_thetaPosition*360/(2*_phi))%360);   // Make it always in 360 degree
}

void kinematicAUMR_RS::processLeftForward(){
	if(digitalRead(_encoderLeftA) == LOW){
		if(digitalRead(_encoderLeftB) == LOW){
			_leftCounter--;
		} else{
			_leftCounter++;
		}
    } else{
		if(digitalRead(_encoderLeftB) == HIGH){
			_leftCounter--;
		} else{
			_leftCounter++;
		}
    }
}

void kinematicAUMR_RS::processLeftBackward(){
	if(digitalRead(_encoderLeftB) == LOW){
		if(digitalRead(_encoderLeftA) == LOW){
			_leftCounter++;
		} else{
			_leftCounter--;
		}
    } else{
		if(digitalRead(_encoderLeftA) == HIGH){
			_leftCounter++;
		} else{
			_leftCounter--;
		}
    }
}

void kinematicAUMR_RS::processRightForward(){
	if(digitalRead(_encoderRightA) == LOW){
		if(digitalRead(_encoderRightB) == LOW){
			_rightCounter--;
		} else{
			_rightCounter++;
		}
    } else{
		if(digitalRead(_encoderRightB) == HIGH){
			_rightCounter--;
		} else{
			_rightCounter++;
		}
    }
}

void kinematicAUMR_RS::processRightBackward(){
	if(digitalRead(_encoderRightB) == LOW){
		if(digitalRead(_encoderRightA) == LOW){
			_rightCounter++;
		} else{
			_rightCounter--;
		}
    } else{
		if(digitalRead(_encoderRightA) == HIGH){
			_rightCounter++;
		} else{
			_rightCounter--;
		}
    }
}

// ===================================================================
// ===================================================================
// ===================================================================
// ===================================================================
// ===================================================================
// ===================================================================

kinematicDOPER_RS::kinematicDOPER_RS(int encoderOneA, int encoderOneB, int encoderTwoA, int encoderTwoB, 
						  			 int encoderThreeA, int encoderThreeB, int encoderFourA, int encoderFourB,
						  			 float wheelRadius, float rangeBetweenWheels, float encoderPPR){
	_encoderOneA = encoderOneA;
	_encoderOneB = encoderOneB;
	
	_encoderTwoA = encoderTwoA;
	_encoderTwoB = encoderTwoB;

	_encoderThreeA = encoderThreeA;
	_encoderThreeB = encoderThreeB;
			
	_encoderFourA = encoderFourA;
	_encoderFourB = encoderFourB;
	
	_wheelRadius = wheelRadius;
	_rangeBetweenWheels = rangeBetweenWheels;
	_encoderPPR = encoderPPR;
	_encoderCPR = 4*encoderPPR;
}

void kinematicDOPER_RS::checkEncoderTimer(){
    _endEncoderMillis = millis();
    _encoderTimer = (_endEncoderMillis - _startEncoderMillis);
    _startEncoderMillis = millis();
}

void kinematicDOPER_RS::checkLoopTimer(){
    _endTimeMillis = millis();
    _loopTimerCheck = _endTimeMillis - _startTimeMillis;
    _startTimeMillis = millis();
	_loopTimer = (float)_loopTimerCheck/1000;
}

void kinematicDOPER_RS::convertEncoderToPositionAndSpeed(){
	// Convert Position
    _leftPosition = (2*_phi*_wheelRadius*_oneCounter)/_encoderCPR;
    _rightPosition = (2*_phi*_wheelRadius*_twoCounter)/_encoderCPR;
  
    // Check timer to calculate the speed
    checkEncoderTimer();

    // Convert Speed
    _leftLinearSpeed = (_leftPosition - _lastLeftPosition)*1000/(_encoderTimer);
    _rightLinearSpeed = (_rightPosition - _lastRightPosition)*1000/(_encoderTimer);

    // Save last variable
    _lastLeftPosition = _leftPosition;
    _lastRightPosition = _rightPosition;
}

float kinematicDOPER_RS::getLeftPositionInCM(){
	return _leftPosition;
}
float kinematicDOPER_RS::getRightPositionInCM(){
	return _rightPosition;
}
float kinematicDOPER_RS::getLeftSpeed(){
	return _leftLinearSpeed;
}
float kinematicDOPER_RS::getRightSpeed(){
	return _rightLinearSpeed;
}
float kinematicDOPER_RS::getXPositionInCM(){
	return _xPositionInCM;
}
float kinematicDOPER_RS::getYPositionInCM(){
	return _yPositionInCM;
}
int kinematicDOPER_RS::getThetaInDegree(){
	return _thetaPositionInDegree;
}

void kinematicDOPER_RS::calculate(){
	kinematicDOPER_RS::checkLoopTimer();
	kinematicDOPER_RS::convertEncoderToPositionAndSpeed();
	kinematicDOPER_RS::calculatePosition();
}

void kinematicDOPER_RS::calculatePosition(){
    // different equation with 2 differential wheels
    
}

void kinematicDOPER_RS::processOneForward(){
	if(digitalRead(_encoderOneA) == LOW){
		if(digitalRead(_encoderOneB) == LOW){
			_oneCounter--;
		} else{
			_oneCounter++;
		}
    } else{
		if(digitalRead(_encoderOneB) == HIGH){
			_oneCounter--;
		} else{
			_oneCounter++;
		}
    }
}

void kinematicDOPER_RS::processOneBackward(){
	if(digitalRead(_encoderOneB) == LOW){
		if(digitalRead(_encoderOneA) == LOW){
			_oneCounter++;
		} else{
			_oneCounter--;
		}
    } else{
		if(digitalRead(_encoderOneA) == HIGH){
			_oneCounter++;
		} else{
			_oneCounter--;
		}
    }
}

void kinematicDOPER_RS::processTwoForward(){
	if(digitalRead(_encoderTwoA) == LOW){
		if(digitalRead(_encoderTwoB) == LOW){
			_twoCounter--;
		} else{
			_twoCounter++;
		}
    } else{
		if(digitalRead(_encoderTwoB) == HIGH){
			_twoCounter--;
		} else{
			_twoCounter++;
		}
    }
}

void kinematicDOPER_RS::processTwoBackward(){
	if(digitalRead(_encoderTwoB) == LOW){
		if(digitalRead(_encoderTwoA) == LOW){
			_twoCounter++;
		} else{
			_twoCounter--;
		}
    } else{
		if(digitalRead(_encoderTwoA) == HIGH){
			_twoCounter++;
		} else{
			_twoCounter--;
		}
    }
}

void kinematicDOPER_RS::processThreeForward(){
	if(digitalRead(_encoderThreeA) == LOW){
		if(digitalRead(_encoderThreeB) == LOW){
			_threeCounter--;
		} else{
			_threeCounter++;
		}
    } else{
		if(digitalRead(_encoderThreeB) == HIGH){
			_threeCounter--;
		} else{
			_threeCounter++;
		}
    }
}

void kinematicDOPER_RS::processThreeBackward(){
	if(digitalRead(_encoderThreeB) == LOW){
		if(digitalRead(_encoderThreeA) == LOW){
			_threeCounter++;
		} else{
			_threeCounter--;
		}
    } else{
		if(digitalRead(_encoderThreeA) == HIGH){
			_threeCounter++;
		} else{
			_threeCounter--;
		}
    }
}

void kinematicDOPER_RS::processFourForward(){
	if(digitalRead(_encoderFourA) == LOW){
		if(digitalRead(_encoderFourB) == LOW){
			_twoCounter--;
		} else{
			_twoCounter++;
		}
    } else{
		if(digitalRead(_encoderFourB) == HIGH){
			_twoCounter--;
		} else{
			_twoCounter++;
		}
    }
}

void kinematicDOPER_RS::processFourBackward(){
	if(digitalRead(_encoderFourB) == LOW){
		if(digitalRead(_encoderFourA) == LOW){
			_twoCounter++;
		} else{
			_twoCounter--;
		}
    } else{
		if(digitalRead(_encoderFourA) == HIGH){
			_twoCounter++;
		} else{
			_twoCounter--;
		}
    }
}