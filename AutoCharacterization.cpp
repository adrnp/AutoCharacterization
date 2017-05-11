#include "AutoCharacterization.h"

AutoCharacterization::AutoCharacterization(Type type, Mode mode, RFPowerMonitor* powerMonitor, AngleStepper* azStepper, AngleStepper* elStepper) :
_type(type),
_mode(mode),
_powerMonitor(powerMonitor),
_azStepper(azStepper),
_elStepper(elStepper),
_numMeasurements(5),
_frequency(10),
_minAzAngle(0),
_maxAzAngle(359),
_azDesiredTravel(359),
_minElAngle(0),
_maxElAngle(90),
_elDesiredTravel(90),
_lastMeasurementTime(0),
_measurementCount(0),
_azCompleted(false),
_elCompleted(false),
_completed(false)
{
	// calculate the timeout time
	_timeout = (unsigned long) (1.0/_frequency * 1000.0);

}

AutoCharacterization::AutoCharacterization(Type type, Mode mode, RFPowerMonitor* powerMonitor, AngleStepper* stepper) : 
AutoCharacterization(type, mode, powerMonitor, nullptr, nullptr)
{
	switch (type) {
		case Type::FULL:
			// TODO: need to throw an error here
			break;

		case Type::AZIMUTH:
			_azStepper = stepper;
			break;

		case Type::ELEVATION:
			_elStepper = stepper;
			break;
	}
}


void AutoCharacterization::setNumMeasurements(int numMeasurements) {
	_numMeasurements = numMeasurements;
}


void AutoCharacterization::setMeasurementFrequency(int frequency) {
	_frequency = frequency;
}


void AutoCharacterization::setAzimuthStepIncrement(unsigned int numSteps) {
	if (_azStepper != nullptr) {
		_azStepper->setNextStepSize(numSteps);
	}
}


void AutoCharacterization::setElevationStepIncrement(unsigned int numSteps) {
	if (_elStepper != nullptr) {
		_elStepper->setNextStepSize(numSteps);
	}
}


void AutoCharacterization::setAzimuthSweep(int minAngle, int maxAngle) {
	_minAzAngle = minAngle;
	_maxAzAngle = maxAngle;
	_azDesiredTravel = abs(_maxAzAngle - _minAzAngle);
}


void AutoCharacterization::setElevationSweep(int minAngle, int maxAngle) {
	_minElAngle = minAngle;
	_maxElAngle = maxAngle;
	_elDesiredTravel = abs(_maxElAngle - _minElAngle);
}

void AutoCharacterization::setToStart() {
	if (_azStepper != nullptr) {
		_azStepper->moveTo(_minAzAngle);
		_azStepper->resetAngleSwept();
	}

	if (_elStepper != nullptr) {
		_elStepper->moveTo(_minElAngle);
		_elStepper->resetAngleSwept();
	}
}


bool AutoCharacterization::isCompleted() {
	return _completed;
}


void AutoCharacterization::run() {

	// want to do this at a specific frequency
	if (millis() - _lastMeasurementTime < _timeout) {
		return;
	}
	

	// move motors, only if have made enough measurements
	// TODO: move to class
	if (_measurementCount >= _numMeasurements) {
		// reset the measurement count flag
		_measurementCount = 0;

		// motor movement is based on which characterization is being run
		switch(_type) {

			/* sweeping azimuth and elevation */
			case Type::FULL:
				setAzimuth();
				setElevation();

				// update completed status
				_completed = (_elCompleted && _azCompleted);
			break;

			/* sweeping only azimuth */
			case Type::AZIMUTH:
				setAzimuth();

				// update completed status
				_completed = _azCompleted;
			break;

			/* sweeping only elevation */
			case Type::ELEVATION:
				setElevation();

				// update completed status
				_completed = _elCompleted;
			break;
		}

		// end the run if the characterization is completed
		if (_completed) {
			sendStatus(Status::FINISHED);
			return;
		}
	}

	// make a single measurement
	float measurement = _powerMonitor->makeMeasurement();
	_measurementCount++;
	_lastMeasurementTime = millis();

	// send the data to the computer
	sendMeasurement(measurement);
}


void AutoCharacterization::setAzimuth() {
	if (_azStepper == nullptr) {
		// TODO: need to throw some kind of error
		return;
	}

	// if still have azimuth to sweep out, move to the next step
	if (_azStepper->getAngleSwept() < _azDesiredTravel) {
		_azStepper->moveToNext();

	} else {  // once we've swept through the entire azimuth range, reset
		_azStepper->moveTo(_minAzAngle);
		_azStepper->resetAngleSwept();
		_azCompleted = true;
	}
}


void AutoCharacterization::setElevation() {
	if (_elStepper == nullptr) {
		// TODO: need to throw some kind of error
		return;
	}

	// only move elevaiton motor when the azimuth is completed
	if (!_azCompleted) {
		return;
	}

	// if still have elevation to sweep out, move to the next step
	if (_elStepper->getAngleSwept() < _elDesiredTravel) {
		_elStepper->moveToNext();
		_azCompleted = false;

	} else {  // once we've swept through the entire elevation range, reset
		_elStepper->moveTo(_minElAngle);
		_elStepper->resetAngleSwept();
		_elCompleted = true;
	}
}


void AutoCharacterization::sendMeasurement(float measurement) {

	float currentAzAngle = 0;
	if (_azStepper != nullptr) {
		currentAzAngle = _azStepper->getCurrentAngle();
	}

	float currentElAngle = 0;
	if (_elStepper != nullptr) {
		currentElAngle = _elStepper->getCurrentAngle();
	}

	// pack the message
	MeasurementMessage msg;
	msg.timestamp = _lastMeasurementTime;
	msg.signalStrength = measurement;
	msg.azimuth = currentAzAngle;
	msg.elevation = currentElAngle;


	// send the data over the serial port
	// sync bytes first
	Serial.write(SYNC_1);
	Serial.write(SYNC_2);

	// send the message ID next
	Serial.write(static_cast<uint8_t> (MessageID::MEASUREMENT));

	// the actual data
	Serial.write((uint8_t*) &msg, sizeof(msg));

	return;
}


void AutoCharacterization::sendStatus(Status status) {

	// pack the message
	StatusMessage msg;
	msg.timestamp = millis();
	msg.status = static_cast<uint8_t> (status);

	// send the data over the serial port
	// sync bytes first
	Serial.write(SYNC_1);
	Serial.write(SYNC_2);

	// send the message ID next
	Serial.write(static_cast<uint8_t> (MessageID::STATUS));

	// the actual data
	Serial.write((uint8_t*) &msg, sizeof(msg));

	return;

}
