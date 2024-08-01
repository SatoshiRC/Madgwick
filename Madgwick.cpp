/*
 * Magdwick.cpp
 *
 *  Created on: Feb 11, 2023
 *      Author: OHYA Satoshi
 */

#include "Madgwick.h"

void Madgwick::update()
{
	if(__isInitialized == false){
		__isInitialized = true;
	}

	auto tmp = elapsedTimer->getTimeMS();
	float stepTime = tmp - elapsedTime;
	elapsedTime = tmp;

	Quaternion qDotOmega;
	std::array<float,3> f;
	Quaternion qDotEpsilon;
	Quaternion qDot;
	const float beta=std::sqrt(3/4.0)*M_PI*(5.0/180.0);

	Quaternion gyroQuaternion(gyroValue);

	qDotOmega = quaternion * gyroQuaternion * 0.5;

	f[0]=2*(quaternion[1]*quaternion[3]-quaternion[0]*quaternion[2])-accelValue[0];
	f[1]=2*(quaternion[0]*quaternion[1]+quaternion[2]*quaternion[3]-accelValue[1]);
	f[2]=2*(1/2.0-std::pow(quaternion[1],2)-std::pow(quaternion[2],2))-accelValue[2];

	std::array<std::array<float,3>,4> j =  {
			-2*quaternion[2], 2*quaternion[1],0,
			2*quaternion[3],2*quaternion[0],-4*quaternion[1],
			-2*quaternion[0],2*quaternion[3],-4*quaternion[2],
			2*quaternion[1],2*quaternion[2],0
	};

	 for(uint8_t n=0;n<4;n++){
		 qDotEpsilon[n]=0;
		 for(uint8_t m=0;m<3;m++){
			 qDotEpsilon[n]+= j[n][m]*f[m];
		 }
	 }

	 qDot = qDotOmega - qDotEpsilon.normalize()*beta;
	 quaternion=quaternion+qDot*stepTime/1000.0;

	 quaternion.normalize();
}
