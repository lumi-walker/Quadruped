#include "Integrator.h"

Integrator::Integrator( const double& Ts) {
		_Ts = Ts;
		_pos = 0;
		_prev_pos = 0;
		_prev_vel = 0;
}

	//implements forward euler
double Integrator::step( double curr_vel ) {
	_pos = _prev_pos + _prev_vel * _Ts;
	_prev_pos = _pos;
	_prev_vel = curr_vel;

	return _pos;
}

double Integrator::stept( double curr_vel, double dt ) {
	_pos = _prev_pos + _prev_vel * dt;
	_prev_pos = _pos;
	_prev_vel = curr_vel;

	return _pos;
}
