/*
 * Magdwick.h
 *
 *  Created on: Feb 11, 2023
 *      Author: OHYA Satoshi
 */

#ifndef MAGDWICK_MADGWICK_H_
#define MAGDWICK_MADGWICK_H_

#include <cmath>
#include <array>
#include "Quaternion/Quaternion.h"
#include "elapsedTimer/elapsedTimer.h"


class Madgwick {
public:
	Madgwick(ElapsedTimer *elapsedTimer, Quaternion<float> imuFrameDiff=Quaternion<float>())
	:elapsedTimer(elapsedTimer),imuFrameDiff(imuFrameDiff){
		__isInitialized = false;
	};

	/* 
	 * brief Calculate quaternion
	 */
	// void update(std::array<float, 3> accelValue, std::array<float, 3> gyroValue);
	void update();

	//These functions are used to set sensor values, gyroscope and accelerometer.
	/*
	 * brief Set accelerometer output
	 * param arg Gyroscope value in radian per second (rad/sec)
	 */
	void setGyroValue(const Vector3D<float> &arg){
		gyroValue = arg;
	}

	/*
	 * brief Set accelerometer output
	 * param arg Accelerometer value in G
	 */
	void setAccelValue(const Vector3D<float> &arg){
		accelValue = arg;
	}

	/*
	 * brief Getter function of attitude
	 * retval Attitude
	 */
	Quaternion<float> getAttitude(){
		return imuFrameDiff * quaternion;
	}

	bool isInitialized(){
		return __isInitialized;
	}

private:
	Quaternion<float> quaternion;
	Quaternion<float> imuFrameDiff;

	ElapsedTimer *elapsedTimer;
	float elapsedTime;
	float deltaTime;

	Vector3D<float> gyroValue;
	Vector3D<float> accelValue;

	bool __isInitialized;
};

#endif /* MAGDWICK_MADGWICK_H_ */
