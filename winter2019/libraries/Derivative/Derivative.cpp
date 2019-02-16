#include "Derivative.h"

Derivative::Derivative( const double& Ts , const uint8_t& m) : _cache(Cache(m)) {

	_m = m;
	_Ts = Ts;
	_dt = _m*_Ts;

	_window = new double[_m];
	_estimate = 0;

	for(int i = 0; i < _m; i++) {
		_window[i] = 0;
	}
}

	
double Derivative::step( double dx ) {
	_window = _cache.step(dx);

	_estimate = 0;
	for(int i = 0; i < _m; i++) {
		_estimate += _window[i];
	}

	_estimate /= _dt;

	return _estimate;
}
