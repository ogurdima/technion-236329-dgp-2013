#pragma once

class RBF
{
public:
	virtual float operator()(float dist) const = 0;
};


class TriharmonicRbf : public RBF
{
public:
	float operator()(float dist) const {
		return dist * dist * dist;
	}
};

class CubicBSplineRbf : public RBF
{
public:
	CubicBSplineRbf(float _betha = 1.0) : betha(_betha) {}
	float operator()(float dist) const {
		float s = dist/betha;
		float s2 = s*s;
		float s3 = s2*s;
		if ( s < -2 || s > 2) {
			return 0;
		}
		if (s > -2 && s < -1) {
			return (2 + s)*(2 + s)*(2 + s)/6;
		}
		if (s > -1 && s < 0) {
			return ( 4 - 6*s2 - 3*s3 )/6;
		}
		if (s > 0 && s < 1) {
			return ( 4 - 6*s2 + 3*s3 )/6;
		}
		if (s > 1 && s < 2) {
			return (2 - s)*(2 - s)*(2 - s)/6;
		}
		return 0;
	}
	float betha;
};