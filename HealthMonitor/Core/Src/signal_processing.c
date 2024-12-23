/*
 * signal_processing.c
 *
 *  Created on: Nov 20, 2024
 *      Author: Mohsen PC
 */

#include <stdio.h>
#include <math.h>
#include <signal_processing.h>
#include <stdlib.h>


/**
 * Calculate the mean of an array.
 *
 * @param array Pointer to the array of floats
 * @param length Number of elements in the array
 * @return The calculated mean of the array
 */
float mean(float *array, int length) {
	float sum = 0;
	for (int i = 0; i < length; i++) {
		sum += array[i];
	}
	return sum / length;
}

/**
 * Calculate the median of an array.
 *
 * @param array Pointer to the array of floats
 * @param count Number of elements in the array
 * @return The calculated median of the array
 */
float median(float *array, int count) {
    // Sort the array
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (array[i] > array[j]) {
                // Swap elements
                float temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }

    // Find and return the median
    if (count % 2 == 0) {
        // Even number of elements: median is average of the two middle elements
        return (array[count / 2 - 1] + array[count / 2]) / 2.0f;
    } else {
        // Odd number of elements: median is the middle element
        return array[count / 2];
    }
}

/**
 * Apply a high-pass filter to a signal.
 *
 * @param input Current signal input
 * @param prevInput Pointer to the previous input value
 * @param prevOutput Pointer to the previous output value
 * @param alpha Filter coefficient
 * @return The filtered signal output
 */
float highPassFilter(float input, float *prevInput, float *prevOutput, float alpha) {
    float inputF = input; // No need to cast, input is already float
    float output = alpha * (*prevOutput + inputF - *prevInput);
    *prevInput = inputF;
    *prevOutput = output;
    return output;
}

/**
 * Process a PPG signal by buffering and calculating the mean when the buffer is full.
 *
 * @param ppg_signal_rdc Current PPG signal value
 * @param buffer Pointer to the buffer array for storing values
 * @param M Size of the buffer
 * @param i Pointer to the current index in the buffer
 * @param filled Pointer to the flag indicating if the buffer is full
 * @return The calculated mean of the buffer, or 0.0 if the buffer is not yet full
 */
float process_ppg_signal(float ppg_signal_rdc, float *buffer, int M, int *i, int *filled) {
    float output = 0.0;

    if (*i < M) {
        // Fill the buffer until it is full
        buffer[*i] = ppg_signal_rdc;
//        (*i)++;
        if (*i == M) {
            *filled = 1; // Mark buffer as full
        }
    } else if (*filled) {
        // Compute the mean of the buffer
        output = mean(buffer, M);

        // Shift buffer elements to make space for the new data
        for (int j = 0; j < M - 1; j++) {
            buffer[j] = buffer[j + 1];
        }
        buffer[M - 1] = ppg_signal_rdc;
    }

    return output; // Return the calculated mean (0.0 if the buffer is not yet full)
}


/**
 * Detect peaks in the data buffer to identify R-peaks.
 *
 * @param dataBuffer Pointer to the input signal array
 * @param length Number of samples in the dataBuffer
 * @param R Pointer to an array to store detected R-peak indices
 * @param R_count Pointer to store the number of detected R-peaks
 */
void findPeaks(float *dataBuffer, int length, uint32_t *R, uint32_t *R_count) {
	int Nd = 3;
//	int N = 4;
	int RRmin = (int)(30); // Minimum refractory period
	int QRSint = (int)(40); // Window for QRS complex
	int pth = (int)(1); // Exponential decay factor

	// Temporary buffers
	uint32_t preData[length];
	uint32_t dif_d[length - Nd];
	float max_val[length];
//	float thPlot[length];
	int max_pos[length];

	for (int i = 0; i < length; i++) {
		if (dataBuffer[i]<mean(dataBuffer,length)) {
			preData[i] = 0;
		} else {
			preData[i] = dataBuffer[i];
		}
//        printf("%d, ", preData[i]);
	}

// Compute differences
	for (int i = 0; i < length - Nd; i++) {
		dif_d[i] = preData[i + Nd] - preData[i]; // No need to cast explicitly
		dif_d[i] = dif_d[i] * dif_d[i];
	}

	// Dynamic threshold and peak detection
	float th = 10; // Initial threshold
	int n = 0, i = 0;
	*R_count = 0; // Initialize R-peak count

	while (n < length - Nd) {
		if (dif_d[n] > th) {
			float local_max = 0;
			int local_max_pos = 0;

			// Find local maximum in the window
			for (int k = 0; k < RRmin + QRSint && n + k < length - Nd; k++) {
				if (dif_d[n + k] > local_max) {
					local_max = preData[n + k];
					local_max_pos = k;
				}
			}

			// Store the peak information
			max_val[i] = local_max;
			max_pos[i] = local_max_pos;
			R[i] = n + local_max_pos + 2;
			(*R_count)++;

			// Update indices and threshold
			int d = RRmin + QRSint - local_max_pos;
			n += RRmin + QRSint + RRmin - d;
			th = mean(max_val, i + 1);
//			for(int i=s; i<s+n; i++) {
//				thPlot[i] = th;
//			}
//			s = n;
			i++;
		} else {
			th *= exp(-pth / (float)FS);
//			thPlot[s] = th;
//			s++;
			n++;
		}
	}
// 	for (int i = 0; i < 1000; i++) {
// 		printf("%.2f, ", thPlot[i]);
// 	}
}

/**
 * Calculate heart rate from detected R-peak indices.
 *
 * @param R Pointer to the array of R-peak indices
 * @param R_count Number of detected R-peaks
 * @return Calculated heart rate in beats per minute (BPM), or 0 on error
 */
uint16_t heartRate(uint32_t *R, int R_count) {
    if (R_count < 3 || R_count >= 12) {
        return 0; // Return 0 if insufficient data
    }

    // Dynamically allocate memory for dR
    float *dR = (float *)malloc((R_count - 1) * sizeof(float));
    if (dR == NULL) {
        // Memory allocation failed
        return 0;
    }

    // Calculate RR intervals
    for (int i = 0; i < R_count - 1; i++) {
        dR[i] = R[i + 1] - R[i];
//        printf("RR = %0.2f, ", dR[i]);
    }

    // Calculate median of RR intervals
    float medianRR = median(dR, R_count - 1);
//    free(dR); // Free allocated memory

    // Check for division by zero
    if (medianRR == 0) {
        return 0;
    }

    // Calculate and return heart rate
    uint16_t HR = (uint16_t)((60 * FS) / medianRR);
    return HR;
}


/**
 * Detect whether a finger is present based on signal thresholds.
 *
 * @param dataBuffer Pointer to the signal data array
 * @param bufferSize Number of samples in the dataBuffer
 * @return 1 if a finger is detected, 0 otherwise
 */
uint16_t isFingerDetected(float *dataBuffer, size_t bufferSize) {
    // Check for invalid inputs
    if (dataBuffer == NULL || bufferSize == 0) {
        return 0; // No finger detected in case of invalid input
    }

    // Calculate the mean of the data buffer
    float meanValue = mean(dataBuffer, bufferSize);

    // Check if the mean value falls within the defined thresholds
    if ((meanValue > FINGER_THRESHOLD_LOW && meanValue < FINGER_THRESHOLD_HIGH) ||
        (meanValue < -FINGER_THRESHOLD_LOW && meanValue > -FINGER_THRESHOLD_HIGH)) {
        return 1; // Finger detected
    } else {
        return 0; // No finger detected
    }
}


/**
 * Calculate SpO2 from red and IR data.
 *
 * @param redBuffer Array of red light PPG data
 * @param irBuffer Array of infrared light PPG data
 * @param length Number of samples in each buffer
 * @param SpO2 Pointer to store the calculated SpO2 value
 * @param ratio Pointer to store the calculated ratio (optional)
 */
void calculate_SpO2(float *redSignal, float *irSignal, int length, float *SpO2, float *ratio) {
    float acRed = 0, acIr = 0;

    // Calculate the RMS of the signals (AC component)
    for (int i = 0; i < length; i++) {
        acRed += redSignal[i] * redSignal[i];
        acIr += irSignal[i] * irSignal[i];
    }
    acRed = sqrt(acRed / length);
    acIr = sqrt(acIr / length);

    // Calculate the ratio of AC components
    *ratio = acRed / acIr;

    // Estimate SpO2 using the ratio
    *SpO2 = 110.0 - 25.0 * (*ratio);  // Adjust coefficients as needed
}
