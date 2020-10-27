

#pragma once

template<typename T, int WINDOW = 3>
class MedianFilter {
public:
	static_assert(WINDOW >= 3);
	static_assert(WINDOW % 2); // odd

	MedianFilter() = default;

	T apply(const T& sample)
	{
		// push into buffer and remove oldest
		_buffer[_head++] = sample;

		T sorted[WINDOW]{_buffer};
		qsort(&sorted, WINDOW, sizeof(T), cmp);

		return sorted[WINDOW / 2];
	}

private:

	static int cmp(const void *elem1, const void *elem2)
	{
		if (*(const T *)elem1 < * (const T *)elem2) {
			return -1;
		}

		return *(const T *)elem1 > *(const T *)elem2;
	}

	T _buffer[WINDOW]{};
	uint8_t _head{0};
};
