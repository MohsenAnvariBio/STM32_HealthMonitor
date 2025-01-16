# Real-Time Heart Rate and SpO₂ Monitoring System with STM32F446RE

## Overview

This project demonstrates a real-time heart rate and SpO₂ monitoring system using the **STM32F446RE Nucleo board**. It integrates the **MAX301102 Pulse Oximeter and Heart-Rate Sensor** for PPG signal acquisition, and the **ILI9341 LCD** for real-time signal visualization and user interaction. The system features custom drivers for efficient data acquisition, filtering, peak detection, and parameter estimation.

## Features

### PPG Signal Acquisition:
- **Interfacing with MAX301102** using **I2C** for heart rate and SpO₂ data acquisition.
- Custom I2C communication driver to manage sensor data and interrupts efficiently.

### Real-Time Signal Filtering:
- **Moving Average Filter** to smooth raw PPG signals.
- **Drift Removal Filter** to eliminate DC offset from the signal.
- **User control** via the LCD touch interface to toggle the filter on/off.

### Finger Detection:
- **Real-time detection** of finger placement on the MAX301102 sensor.
- **Visual feedback** on the LCD display with a heart icon.
- **Automatic suspension** of peak detection and SpO₂ estimation when no finger is detected.

### Peak Detection Algorithm:
- **Real-time peak detection** based on adaptive thresholding.
- Algorithm based on IEEE paper: *A Real-Time QRS Complex Detector Based on Adaptive Thresholding* (https://ieeexplore.ieee.org/abstract/document/7138573).

### Heart Rate Calculation:
- **Heart rate calculation** based on the time between detected peaks.

### SpO₂ Estimation:
- **Real-time SpO₂ estimation** using the Red and IR PPG signals from the MAX301102 sensor.
- Display of the SpO₂ value on the LCD.

### Real-Time Data Visualization:
- Display of **raw and filtered PPG signals**, **heart rate**, **SpO₂**, and **temperature** on the ILI9341 LCD.
- **Touch-based control** to adjust signal gain and toggle filters.

## Hardware Setup

- **STM32F446RE Nucleo Board**
- **MAX301102 Pulse Oximeter & Heart-Rate Sensor**
  - Interface: **I2C**
- **ILI9341 LCD Display**
  - Interface: **SPI**

## Software Components

- **STM32 HAL Libraries**
- **Custom Drivers** for MAX301102 and ILI9341

## Usage

1. Place a finger on the **MAX301102 sensor** and enable **MA Filter**.
2. Observe the heart icon on the display for finger detection status.
3. Monitor **heart rate**, **SpO₂**, and **temperature** in real-time.
4. Use the **LCD touch interface** to adjust signal gain and toggle filters.


## Connection
![connection](https://github.com/user-attachments/assets/845c73fb-bd37-4d37-a146-4427d0e62c21)

## Device 
- **Original Signal Distorted by Noise**

![with noise](https://github.com/user-attachments/assets/50f2d540-353e-4222-b874-07062c853134)

- **Signal Smoothing using Low-pass Filter**

![clear signal](https://github.com/user-attachments/assets/e82233f5-fa9f-4a1e-8a8d-29842b2b88be)

