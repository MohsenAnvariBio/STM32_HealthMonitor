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

// Helper function to calculate mean of an array
float calculateMean(float *array, int length) {
    float sum = 0;
    for (int i = 0; i < length; i++) {
        sum += array[i];
    }
    return sum / length;
}

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



// Main function to find peaks
void findPeaks(float *dataBuffer, int length, uint32_t *R, uint32_t *R_count) {
    int Nd = 3;
    int N = 4;
    int RRmin = (int)(10); // Minimum refractory period
    int QRSint = (int)(20); // Window for QRS complex
    int pth = (int)(2); // Exponential decay factor

    // Temporary buffers
    uint32_t dif_d[length - Nd];
    float max_val[length];
    int max_pos[length];

 // Compute differences
    for (int i = 0; i < length - Nd; i++) {
        dif_d[i] = dataBuffer[i + Nd] - dataBuffer[i]; // No need to cast explicitly
        dif_d[i] = dif_d[i] * dif_d[i];
    }

    // Dynamic threshold and peak detection
    float th = 10; // Initial threshold
    int n = 0, s = 0, i = 0;
    *R_count = 0; // Initialize R-peak count

    while (n < length - Nd) {
        if (dif_d[n] > th) {
            float local_max = 0;
            int local_max_pos = 0;

            // Find local maximum in the window
            for (int k = 0; k < RRmin + QRSint && n + k < length - Nd; k++) {
                if (dif_d[n + k] > local_max) {
                    local_max = dif_d[n + k];
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
            i++;
        } else {
            th *= exp(-pth / (float)FS);
            n++;
        }
    }
}
