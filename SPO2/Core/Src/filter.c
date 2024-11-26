/*
 * filter.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#include "filter.h"
#include <stdio.h>
#include <math.h>


// Main processing function
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
        output = calculate_mean(buffer, M);

        // Shift buffer elements to make space for the new data
        for (int j = 0; j < M - 1; j++) {
            buffer[j] = buffer[j + 1];
        }
        buffer[M - 1] = ppg_signal_rdc;
    }

    return output; // Return the calculated mean (0.0 if the buffer is not yet full)
}


// Helper function to calculate mean of an array
float calculateMean(float *array, int length) {
	float sum = 0;
	for (int i = 0; i < length; i++) {
		sum += array[i];
	}
	return sum / length;
}
// Function to calculate the median
float calculateMedian(float *array, int count) {
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




// Main function to find peaks
void findPeaks(float *dataBuffer, int length, uint32_t *R, uint32_t *R_count) {
	int Nd = 3;
	int N = 4;
	int RRmin = (int)(30); // Minimum refractory period
	int QRSint = (int)(40); // Window for QRS complex
	int pth = (int)(1); // Exponential decay factor

	// Temporary buffers
	uint32_t preData[length];
	uint32_t dif_d[length - Nd];
	float max_val[length];
	float thPlot[length];
	int max_pos[length];

	for (int i = 0; i < length; i++) {
		if (dataBuffer[i]<calculateMean(dataBuffer,length)) {
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
	int n = 0, s = 1, i = 0;
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
			th = calculateMean(max_val, i + 1);
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
    float medianRR = calculateMedian(dR, R_count - 1);
    free(dR); // Free allocated memory

    // Check for division by zero
    if (medianRR == 0) {
        return 0;
    }

    // Calculate and return heart rate
    uint16_t HR = (uint16_t)((60 * FS) / medianRR);
    return HR;
}


