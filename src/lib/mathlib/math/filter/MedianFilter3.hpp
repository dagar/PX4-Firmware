

#pragma once

template<typename T>
class MedianFilter3 {

public:
	MedianFilter() = default;

	T apply(const T& sample)
	{
		// add new element to ring buffer
		// return median ()

		// N=3 sorting using Bose-Nelson Algorithm.
		SWAP(1, 2);
		SWAP(0, 2);
		SWAP(0, 1);

		T median = max(min(_buffer[0], _buffer[1]), min(max(_buffer[0], _buffer[1]), _buffer[2]));

		// MAX(MIN(a, b), MIN(MAX(a, b), c));
	}

private:
	T _buffer[3]{};

	uint8_t _head{0};
};
