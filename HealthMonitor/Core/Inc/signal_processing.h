/*
 * filter.h
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#ifndef __FILTER_H__
#define __FILTER_H__

#include "stm32f4xx_hal.h"

#define FS 230          // Sampling frequency
#define FINGER_THRESHOLD_LOW 0.1f
#define FINGER_THRESHOLD_HIGH 5.0f


float process_ppg_signal(float ppg_signal_rdc, float *buffer, int M, int *i, int *filled);
float mean(float *array, int length);
void findPeaks(float *dataBuffer, int length, uint32_t *R, uint32_t *R_count);
uint16_t heartRate(uint32_t *R, int R_count);
float median(float *array, int count);
uint16_t isFingerDetected(float *dataBuffer, size_t bufferSize);
float process_ppg_signal(float ppg_signal_rdc, float *buffer, int M, int *i, int *filled);
float highPassFilter(float input, float *prevInput, float *prevOutput, float alpha);


#endif /* __FILTER_H__ */
