/**
 * AutoCharacterization.h
 *
 * Class to control an automated characterization run.
 * 
 */

#ifndef AutoCharacterization_h
#define AutoCharacterization_h

#include <Arduino.h>
#include <AngleStepper.h>
#include <RFPowerMonitor.h>

class AutoCharacterization {

public:

	/**
	 * the type of characterization to run (which axes to sweep)
	 */
	enum class Type : uint8_t {
		FULL = 0,
		AZIMUTH,
		ELEVATION
	};

	/**
	 * the mode the characterization is running in.
	 *
	 * AZ_EL - defining the platforms as azimuth and elevation (antenna mounted horizontally)
	 * PHI_THETA - defining the platforms as phi and theta (antenna mounted vertically)
	 */
	enum class Mode : uint8_t {
		AZ_EL = 0,
		PHI_THETA,
	};

	/**
	 * the current status of the characterization run.
	 */
	enum class Status : uint8_t {
		RUNNING = 0,
		PAUSED,
		FINISHED,
	};



	/**
	 * Constructor
	 */
	AutoCharacterization(Type type, Mode mode, RFPowerMonitor* powerMonitor, AngleStepper* azStepper, AngleStepper* elStepper);
	AutoCharacterization(Type type, Mode mode, RFPowerMonitor* powerMonitor, AngleStepper* stepper);

	void setNumMeasurements(int numMeasurements);
	void setMeasurementFrequency(int frequency);

	void setAzimuthStepIncrement(int numSteps);
	void setElevationStepIncrement(int numSteps);

	void setAzimuthSweep(int minAngle, int maxAngle);
	void setElevationSweep(int minAngle, int maxAngle);

	void setToStart();

	bool isCompleted();

	void run();


private:

	/** the characterization run type */
	Type _type;

	Mode _mode;

	RFPowerMonitor* _powerMonitor;

	AngleStepper* _azStepper;
	AngleStepper* _elStepper;

	int _numMeasurements;
	int _frequency;
	unsigned long _timeout;

	int _minAzAngle;
	int _maxAzAngle;

	int _minElAngle;
	int _maxElAngle;

	unsigned long _lastMeasurementTime;
	int _measurementCount;
	bool _azCompleted;
	bool _elCompleted;
	bool _completed;

	static const byte SYNC_1 = 0xA0;
	static const byte SYNC_2 = 0xB1;

	enum class MessageID : uint8_t {
		MEASUREMENT = 0,
		STATUS = 1,
	};

	struct __attribute__((__packed__)) MeasurementMessage {
		unsigned long timestamp;
		float signalStrength;
		float azimuth;
		float elevation;
	};

	struct __attribute__((__packed__)) StatusMessage {
		unsigned long timestamp;
		uint8_t status;
	};


	void setAzimuth();
	void setElevation();


	void sendMeasurement(float measurement);

	void sendStatus(Status status);




};





#endif /* AutoCharacterization_h */