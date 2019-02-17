#ifndef _Integrator_H
#define _Integrator_H

#include "Cache.h"


class Integrator {

public:
	Integrator( const double& Ts);

	//implements forward euler
	double step( double curr_vel );

private:



	// sampling time 
	double _Ts;

	// previous pos and vel
	double _prev_pos;
	double _prev_vel;

	// pos estimate
	double _pos;

protected:

};


#endif