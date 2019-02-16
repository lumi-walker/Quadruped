#ifndef _DERIVATIVE_H
#define _DERIVATIVE_H

#include "Cache.h"

/*
brief: this class implements the averaging LPP filter discussed in 
"On Algorithms for Velocity Estimation Using Discrete Position Encoders"
*/

class Derivative {

public:
	Derivative( const double& Ts , const uint8_t& m);
	double step( double dx );

	uint8_t getWindowWidth();
	double getSampleTime();

private:

	// number of samples in the averaging window
	uint8_t _m;

	// sampling time 
	double _Ts;

	// window width in seconds
	double _dt;

	// window
	double* _window;

	// estimate
	double _estimate;

	// cache of last _m samples
	Cache _cache;

protected:

};


#endif