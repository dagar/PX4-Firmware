
#include <math.h>
#include <stdint.h>

#ifdef USE_GYRO_DATA_ANALYSE

#include "sdft.h"

#include "gyroanalyse.h"

#define DYN_NOTCH_SMOOTH_HZ        4
#define DYN_NOTCH_CALC_TICKS       (XYZ_AXIS_COUNT * STEP_COUNT) // 3 axes and 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

typedef enum {
	STEP_WINDOW,
	STEP_DETECT_PEAKS,
	STEP_CALC_FREQUENCIES,
	STEP_UPDATE_FILTERS,
	STEP_COUNT
} step_e;

typedef struct peak_s {
	uint8_t bin;
	float value;
} peak_t;

static sdft_t      sdft[XYZ_AXIS_COUNT];
static peak_t      peaks[DYN_NOTCH_COUNT_MAX];
static float       sdftData[SDFT_BIN_COUNT];
static uint16_t    sdftSampleRateHz;
static float       sdftResolutionHz;
static uint8_t     sdftStartBin;
static uint8_t     sdftEndBin;
static float       sdftMeanSq;
static uint16_t    dynNotchBandwidthHz;
static uint16_t    dynNotchMinHz;
static uint16_t    dynNotchMaxHz;
static uint16_t    dynNotchMaxFFT;
static float       smoothFactor;
static uint8_t     numSamples;

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
	static bool gyroAnalyseInitialized;

	if (gyroAnalyseInitialized) {
		return;
	}

	gyroAnalyseInitialized = true;
#endif

	dynNotchBandwidthHz = gyroConfig()->dyn_notch_bandwidth_hz;
	dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;
	dynNotchMaxHz = MAX(2 * dynNotchMinHz, gyroConfig()->dyn_notch_max_hz);

	// gyroDataAnalyse() is running at targetLoopRateHz (which is PID loop rate aka. 1e6f/gyro.targetLooptimeUs)
	const int32_t targetLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
	numSamples = MAX(1, targetLoopRateHz / (2 * dynNotchMaxHz)); // 600hz, 8k looptime, 6.00

	sdftSampleRateHz = targetLoopRateHz / numSamples;
	// eg 8k, user max 600hz, int(8000/1200) = 6 (6.666), sdftSampleRateHz = 1333hz, range 666Hz
	// eg 4k, user max 600hz, int(4000/1200) = 3 (3.333), sdftSampleRateHz = 1333hz, range 666Hz
	// eg 2k, user max 600hz, int(2000/1200) = 1 (1.666) sdftSampleRateHz = 2000hz, range 1000Hz
	// eg 2k, user max 400hz, int(2000/800) = 2 (2.5) sdftSampleRateHz = 1000hz, range 500Hz
	// eg 1k, user max 600hz, int(1000/1200) = 1 (max(1,0.8333)) sdftSampleRateHz = 1000hz, range 500Hz
	// the upper limit of DN is always going to be the Nyquist frequency (= sampleRate / 2)

	sdftResolutionHz = (float)sdftSampleRateHz / SDFT_SAMPLE_SIZE; // 13.3hz per bin at 8k
	sdftStartBin = MAX(2, lrintf(dynNotchMinHz / sdftResolutionHz + 0.5f)); // can't use bin 0 because it is DC.
	sdftEndBin = MIN(SDFT_BIN_COUNT - 1,
			 lrintf(dynNotchMaxHz / sdftResolutionHz + 0.5f)); // can't use more than SDFT_BIN_COUNT bins.
	smoothFactor = pt1FilterGain(DYN_NOTCH_SMOOTH_HZ,
				     DYN_NOTCH_CALC_TICKS / (float)targetLoopRateHz); // minimum PT1 k value

	for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		sdftInit(&sdft[axis], sdftStartBin, sdftEndBin, numSamples);
	}
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
	// initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
	gyroDataAnalyseInit(targetLooptimeUs);
	state->maxSampleCount = numSamples;
	state->maxSampleCountRcp = 1.0f / state->maxSampleCount;

	for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
			// any init value is fine, but evenly spreading centerFreqs across frequency range makes notch filters stick to peaks quicker
			state->centerFreq[axis][p] = (p + 0.5f) * (dynNotchMaxHz - dynNotchMinHz) / (float)gyro.notchFilterDynCount +
						     dynNotchMinHz;
		}
	}
}

// Collect gyro data, to be downsampled and analysed in gyroDataAnalyse() function
void gyroDataAnalysePush(gyroAnalyseState_t *state, const uint8_t axis, const float sample)
{
	state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state);

// Downsample and analyse gyro data
void gyroDataAnalyse(gyroAnalyseState_t *state)
{
	// samples should have been pushed by `gyroDataAnalysePush`
	// if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
	if (state->sampleCount == state->maxSampleCount) {
		state->sampleCount = 0;

		// calculate mean value of accumulated samples
		for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			const float sample = state->oversampledGyroAccumulator[axis] * state->maxSampleCountRcp;
			state->downsampledGyroData[axis] = sample;

			if (axis == 0) {
				DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
			}

			state->oversampledGyroAccumulator[axis] = 0;
		}

		// We need DYN_NOTCH_CALC_TICKS ticks to update all axes with newly sampled value
		// recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
		// at 8kHz PID loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
		// at 4kHz PID loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
		state->updateTicks = DYN_NOTCH_CALC_TICKS;
	}

	// 2us @ F722
	// SDFT processing in batches to synchronize with incoming downsampled data
	for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		sdftPushBatch(&sdft[axis], &state->downsampledGyroData[axis], &state->sampleCount);
	}

	state->sampleCount++;

	// Find frequency peaks and update filters
	if (state->updateTicks > 0) {
		gyroDataAnalyseUpdate(state);
		--state->updateTicks;
	}
}

// Find frequency peaks and update filters
static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state)
{
	uint32_t startTime = 0;

	if (debugMode == (DEBUG_FFT_TIME)) {
		startTime = micros();
	}

	DEBUG_SET(DEBUG_FFT_TIME, 0, state->updateStep);

	switch (state->updateStep) {
	case STEP_WINDOW: { // 6us @ F722
			sdftWinSq(&sdft[state->updateAxis], sdftData);

			// Calculate mean square over frequency range (= average power of vibrations)
			sdftMeanSq = 0.0f;

			for (uint8_t bin = (sdftStartBin + 1); bin < sdftEndBin;
			     bin++) {   // don't use startBin or endBin because they are not windowed properly
				sdftMeanSq += sdftData[bin]; // sdftData is already squared (see sdftWinSq)
			}

			sdftMeanSq /= sdftEndBin - sdftStartBin - 1;

			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

			break;
		}

	case STEP_DETECT_PEAKS: { // 6us @ F722
			// Get memory ready for new peak data on current axis
			for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
				peaks[p].bin = 0;
				peaks[p].value = 0.0f;
			}

			// Search for N biggest peaks in frequency spectrum
			for (uint8_t bin = (sdftStartBin + 1); bin < sdftEndBin; bin++) {
				// Check if bin is peak
				if ((sdftData[bin] > sdftData[bin - 1]) && (sdftData[bin] > sdftData[bin + 1])) {
					// Check if peak is big enough to be one of N biggest peaks.
					// If so, insert peak and sort peaks in descending height order
					for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
						if (sdftData[bin] > peaks[p].value) {
							for (uint8_t k = gyro.notchFilterDynCount - 1; k > p; k--) {
								peaks[k] = peaks[k - 1];
							}

							peaks[p].bin = bin;
							peaks[p].value = sdftData[bin];
							break;
						}
					}

					bin++; // If bin is peak, next bin can't be peak => jump it
				}
			}

			// Sort N biggest peaks in ascending bin order (example: 3, 8, 25, 0, 0, ..., 0)
			for (uint8_t p = gyro.notchFilterDynCount - 1; p > 0; p--) {
				for (uint8_t k = 0; k < p; k++) {
					// Swap peaks but ignore swapping void peaks (bin = 0). This leaves
					// void peaks at the end of peaks array without moving them
					if (peaks[k].bin > peaks[k + 1].bin && peaks[k + 1].bin != 0) {
						peak_t temp = peaks[k];
						peaks[k] = peaks[k + 1];
						peaks[k + 1] = temp;
					}
				}
			}

			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

			break;
		}

	case STEP_CALC_FREQUENCIES: { // 4us @ F722
			for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {

				// Only update state->centerFreq if there is a peak (ignore void peaks) and if peak is above noise floor
				if (peaks[p].bin != 0 && peaks[p].value > sdftMeanSq) {

					// accumulate sdftSum and sdftWeightedSum from peak bin, and shoulder bins either side of peak
					float squaredData = peaks[p].value; // peak data already squared (see sdftWinSq)
					float sdftSum = squaredData;
					float sdftWeightedSum = squaredData * peaks[p].bin;

					// accumulate upper shoulder unless it would be sdftEndBin
					uint8_t shoulderBin = peaks[p].bin + 1;

					if (shoulderBin < sdftEndBin) {
						squaredData = sdftData[shoulderBin]; // sdftData already squared (see sdftWinSq)
						sdftSum += squaredData;
						sdftWeightedSum += squaredData * shoulderBin;
					}

					// accumulate lower shoulder unless lower shoulder would be bin 0 (DC)
					if (peaks[p].bin > 1) {
						shoulderBin = peaks[p].bin - 1;
						squaredData = sdftData[shoulderBin]; // sdftData already squared (see sdftWinSq)
						sdftSum += squaredData;
						sdftWeightedSum += squaredData * shoulderBin;
					}

					// get centerFreq in Hz from weighted bins
					float centerFreq = dynNotchMaxHz;
					float sdftMeanBin = 0;

					if (sdftSum > 0) {
						sdftMeanBin = (sdftWeightedSum / sdftSum);
						centerFreq = sdftMeanBin * sdftResolutionHz;
						centerFreq = constrainf(centerFreq, dynNotchMinHz, dynNotchMaxHz);
						// In theory, the index points to the centre frequency of the bin.
						// at 1333hz, bin widths are 13.3Hz, so bin 2 (26.7Hz) has the range 20Hz to 33.3Hz
						// Rav feels that maybe centerFreq = (sdftMeanBin + 0.5) * sdftResolutionHz is better
						// empirical checking shows that not adding 0.5 works better

						// PT1 style dynamic smoothing moves rapidly towards big peaks and slowly away, up to 8x faster
						// DYN_NOTCH_SMOOTH_HZ = 4 & dynamicFactor = 1 .. 8  =>  PT1 -3dB cutoff frequency = 4Hz .. 41Hz
						const float dynamicFactor = constrainf(peaks[p].value / sdftMeanSq, 1.0f, 8.0f);
						state->centerFreq[state->updateAxis][p] += smoothFactor * dynamicFactor * (centerFreq -
								state->centerFreq[state->updateAxis][p]);
					}
				}
			}

			if (calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
				for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
					dynNotchMaxFFT = MAX(dynNotchMaxFFT, state->centerFreq[state->updateAxis][p]);
				}
			}

			if (state->updateAxis == gyro.gyroDebugAxis) {
				for (uint8_t p = 0; p < gyro.notchFilterDynCount && p < 3; p++) {
					DEBUG_SET(DEBUG_FFT_FREQ, p, lrintf(state->centerFreq[state->updateAxis][p]));
				}

				DEBUG_SET(DEBUG_DYN_LPF, 1, lrintf(state->centerFreq[state->updateAxis][0]));
			}

			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

			break;
		}

	case STEP_UPDATE_FILTERS: { // 7us @ F722
			for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
				// Only update notch filter coefficients if the corresponding peak got its center frequency updated in the previous step
				if (peaks[p].bin != 0 && peaks[p].value > sdftMeanSq) {
					// Choose notch Q in such a way that notch bandwidth stays constant (improves prop wash handling)
					float dynamicQ = state->centerFreq[state->updateAxis][p] / (float)dynNotchBandwidthHz;
					dynamicQ = constrainf(dynamicQ, 2.0f, 10.0f);
					biquadFilterUpdate(&gyro.notchFilterDyn[state->updateAxis][p], state->centerFreq[state->updateAxis][p],
							   gyro.targetLooptime, dynamicQ, FILTER_NOTCH);
				}
			}

			DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

			state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
		}
	}

	state->updateStep = (state->updateStep + 1) % STEP_COUNT;
}

#endif // USE_GYRO_DATA_ANALYSE
