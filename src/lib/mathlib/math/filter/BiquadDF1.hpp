#pragma once

#include <math.h>

// Biquad direct form 1 (integer)

// TODO:
//   template argument
//    - filter type
//    - data type

enum class TYPE : uint8_t {
	LOWPASS = 0,
	HIGHPASS,
	BANDPASS,
	NOTCH,
};

template<typename T>
class Biquad
{
public:
	Biquad(double Fc)
	{
		setFc(Fc);
	}

	void setFc(double Fc)
	{
		_fc = Fc;
		calcBiquad();
	}

	float process(float in)
	{
		double out = in * _a0 + _z1;
		_z1 = in * _a1 + _z2 - _b1 * out;
		_z2 = in * _a2 - _b2 * out;
		return out;
	}

private:

	void calcBiquad()
	{
		double norm;
		double V = powf(10, fabsf(_peak_gain) / 20.f);
		double K = tanf(M_PI_F * _fc);

		switch (T) {
		case TYPE::LOWPASS:
			norm = 1 / (1 + K / Q + K * K);
			_a0 = K * K * norm;
			_a1 = 2 * _a0;
			_a2 = _a0;
			_b1 = 2 * (K * K - 1) * norm;
			_b2 = (1 - K / Q + K * K) * norm;
			break;

		case TYPE::HIGHPASS:
			norm = 1 / (1 + K / Q + K * K);
			_a0 = 1 * norm;
			_a1 = -2 * _a0;
			_a2 = _a0;
			_b1 = 2 * (K * K - 1) * norm;
			_b2 = (1 - K / Q + K * K) * norm;
			break;

		case TYPE::BANDPASS:
			norm = 1 / (1 + K / Q + K * K);
			_a0 = K / Q * norm;
			_a1 = 0;
			_a2 = -_a0;
			_b1 = 2 * (K * K - 1) * norm;
			_b2 = (1 - K / Q + K * K) * norm;
			break;

		case TYPE::NOTCH:
			norm = 1 / (1 + K / Q + K * K);
			_a0 = (1 + K * K) * norm;
			_a1 = 2 * (K * K - 1) * norm;
			_a2 = _a0;
			_b1 = _a1;
			_b2 = (1 - K / Q + K * K) * norm;
			break;
		}

		return;
	}

	double _a0{1};
	double _a1{0};
	double _a2{0};
	double _b1{0};
	double _b2{0};

	double _fc{0.50};
	static constexpr double Q{0.707};
	double _peak_gain{0};

	double _z1{0};
	double _z2{0};
};
