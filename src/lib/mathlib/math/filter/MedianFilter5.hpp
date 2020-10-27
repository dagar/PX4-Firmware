

#pragma once

template<typename T>
class MedianFilter5 {

public:
	MedianFilter() = default;

	T apply(const T& sample)
	{
		// add new element to ring buffer
		// return median ()

		// sort ring buffer into sorted

		T sorted[5];

		for (int i = 0; i < 5; i++) {

		}

		// Network for N=5, using Bose-Nelson Algorithm.
		// o--^--------^--^-----------o
		//    |        |  |
		// o--v--------|--|--^--^--^--o
		//             |  |  |  |  |
		// o-----^--^--|--v--|--|--v--o
		//       |  |  |     |  |
		// o--^--|--v--v-----|--v-----o
		//    |  |           |
		// o--v--v-----------v--------o

		// There are 9 comparators in this network,
		// grouped into 6 parallel operations.

		// [[0,1],[3,4]]
		// [[2,4]]
		// [[2,3],[1,4]]
		// [[0,3]]
		// [[0,2],[1,3]]
		// [[1,2]]



		// N=5 sorting using Bose-Nelson Algorithm.
		sorted[0] = _buffer[1]; // SWAP(0, 1);
		sorted[1] = _buffer[0]; // SWAP(0, 1);
		sorted[2] = _buffer[2];
		sorted[3] = _buffer[4]; // SWAP(3, 4);
		sorted[4] = _buffer[3]; // SWAP(3, 4);

		SWAP(2, 4);
		SWAP(2, 3);
		SWAP(0, 3);
		SWAP(0, 2);
		SWAP(1, 4);
		SWAP(1, 3);
		SWAP(1, 2);

		// pop oldest element

		return sorted[2]; // middle element
	}

private:
	void swap(T& a, T& b) {
		const T tmp = a;
		a = b;
		b = tmp;
	}

	T _buffer[5]{};

	uint8_t _head{0};
};
