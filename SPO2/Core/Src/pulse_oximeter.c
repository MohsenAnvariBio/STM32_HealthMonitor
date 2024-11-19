/*
 * pulse_oximiter.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Mario Regus
 *
 *  Note: Contains code adapted from Raivis Strogonivs
 *  https://morf.lv/implementing-pulse-oximeter-using-max30100
 *  https://github.com/xcoder123/MAX30100
 */

#include "pulse_oximeter.h"
#include "filter.h"
#include "math.h"
#include "system.h"
//I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c2;
MAX30102 pulseOximeter = {0};
FIFO_LED_DATA fifoData = {0};

MEASUREMENT_MODE measurementMode = SPO2;
POWER_MODE powerMode = NORMAL;
SAMPLE_RATE sampleRate = _1000SPS;
#define MIN_IR_LED_THRESHOLD  50000  // Adjust this value based on your sensor's IR LED raw data when a finger is detected
#define MIN_RED_LED_THRESHOLD 50000  // Adjust this value based on your sensor's Red LED raw data when a finger is detected


uint8_t fifoRdPtr = 0;

DC_FILTER_T dcFilterIR = {0};
DC_FILTER_T dcFilterRed = {0};
MEAN_DIFF_FILTER_T meanDiffIR = {0};
BUTTERWORTH_FILTER_T lpbFilterIR = {0};

float currentBPM;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
float valuesBPMSum = 0;
uint8_t valuesBPMCount = 0;
uint8_t bpmIndex = 0;
uint32_t lastBeatThreshold = 0;

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint16_t samplesRecorded = 0;
uint16_t pulsesDetected = 0;
float currentSpO2Value = 0;

uint8_t redLEDCurrent = 0;
float lastREDLedCurrentCheck = 0;
PULSE_STATE_MACHINE currentPulseDetectorState = PULSE_IDLE;

LEDCurrent IrLedCurrent;

/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
//void I2C_Init(void)
//{
//	hi2c2.Instance = I2C1;
//	hi2c2.Init.ClockSpeed = 100000;
//	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//	hi2c2.Init.OwnAddress1 = 0;
//	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	hi2c2.Init.OwnAddress2 = 0;
//	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//
//	// Call the HAL error handler on error
//	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
//	{
//		errorHandler();
//	}
//}

int8_t pulseOximeter_readRegister(uint8_t reg, uint8_t* value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = 0x03;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);

	retStatus = HAL_I2C_Master_Transmit(&hi2c2, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	address = (I2C_SLAVE_ID | I2C_READ);
	retStatus = HAL_I2C_Master_Receive(&hi2c2, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	*value = buf[0];

	return 0;
}

HAL_StatusTypeDef pulseOximeter_writeRegister(uint8_t reg, uint8_t value)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = value;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);
	retStatus = HAL_I2C_Master_Transmit(&hi2c2, address, buf, 2, HAL_MAX_DELAY);

	return retStatus;
}


void pulseOximeter_setMeasurementMode(MEASUREMENT_MODE mode)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x7 << 0);

	switch(mode){
	case HEART_RATE:	readResult = readResult | (0x02 << 0); break;
	case SPO2:	readResult = readResult | (0x03 << 0); break;
	case MULTI_LED:	readResult = readResult | (0x07 << 0); break;
	default: return; break;
	}

	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{
		measurementMode = mode;
	}
}

MEASUREMENT_MODE pulseOximeter_getMeasurementMode(void)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return MEASUREMENT_MODE_FAIL;
	}

	readResult &= 0x07;

	return (MEASUREMENT_MODE)readResult;

	switch(readResult)
	{
		case 2: return HEART_RATE;	break;
		case 3: return SPO2;	break;
		case 7: return MULTI_LED;	break;
		default: return HEART_RATE;	break;
	}
}

void pulseOximeter_setPowerMode(POWER_MODE mode)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x80 << 0);

	switch(mode){
	case NORMAL:	readResult = readResult | (0x00 << 0); break;
	case LOW_POWER:	readResult = readResult | (0x80 << 0); break;
	default: return; break;
	}

	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{
		measurementMode = mode;
	}
}

POWER_MODE pulseOximeter_getPowerMode(void)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return POWER_MODE_FAIL;
	}

	readResult &= 0x80;

	switch(readResult)
	{
		case 0: return NORMAL;	break;
		case 0x80: return LOW_POWER;	break;
		default: return NORMAL; break;
	}
}

void pulseOximeter_resetRegisters(void)
{
	int8_t readStatus;
	uint8_t readResult;


	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x01 << 6);
	readResult = readResult | (0x01 << 6);
	if( pulseOximeter_writeRegister(MODE_CONFIG, readResult) != HAL_OK){
		return;
	}
}

void pulseOximeter_setLedCurrent(uint8_t led, float currentLevel)
{
	uint8_t value = 0;
	uint8_t ledRegister = 0;

	switch(led){
	case RED_LED: ledRegister = LED_PULSE_AMP_1; break;
	case IR_LED:	ledRegister = LED_PULSE_AMP_2; break;
	}

	// slope derived from MAX30102 DataSheet
	value = (uint8_t)(5.0 * currentLevel);

	if( pulseOximeter_writeRegister(ledRegister, value) != HAL_OK){
		return;
	}
	else{

	}
}

float pulseOximeter_getLedCurrent(uint8_t led)
{
	int8_t readStatus = 0;
	float currentLevel;
	uint8_t ledRegister = 0;
	uint8_t readResult;

	switch(led){
	case RED_LED: ledRegister = LED_PULSE_AMP_1; break;
	case IR_LED:	ledRegister = LED_PULSE_AMP_2; break;
	}

	readStatus = pulseOximeter_readRegister(ledRegister, &readResult);
	if( readStatus == -1){
		return -1;
	}

	currentLevel = readResult / 5.0;

	return currentLevel;
}

void pulseOximeter_setSampleRate(uint8_t sampleRate)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x1C << 0);

	readResult = readResult | (sampleRate << 2);

	if( pulseOximeter_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{

	}
}

SAMPLE_RATE pulseOximeter_getSampleRate(void)
{
	int8_t readStatus = 0;
	uint8_t result;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return _SAMPLE_RATE_FAIL;
	}

	result = readResult;
	result &= 0x1C;
	result = result >> 2;

	return (SAMPLE_RATE)result;
}

void pulseOximeter_setPulseWidth(uint8_t pulseWidth)
{
	int8_t readStatus = 0;
	uint8_t readResult;

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		return;
	}

	readResult &= ~(0x03 << 0);

	switch(pulseWidth)
	{
	case _69_US: readResult = readResult | 0; break;
	case _118_US: readResult = readResult | (0x01 << 0); break;
	case _215_US: readResult = readResult | (0x02 << 0); break;
	case _411_US: readResult = readResult | (0x03 << 0); break;
	case _PULSE_WIDTH_FAIL: break;
	}

	if( pulseOximeter_writeRegister(SPO2_CONFIG, readResult) != HAL_OK){
		return;
	}
	else{

	}
}

PULSE_WIDTH pulseOximeter_getPulseWidth(void)
{
	int8_t readStatus = 0;
		uint8_t result;
		uint8_t readResult;

		readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
		if( readStatus == -1){
			return _PULSE_WIDTH_FAIL;
		}

		result = readResult;
		result &= 0x03;

		return (PULSE_WIDTH)result;
}

// Write zero to all FIFO registers
void pulseOximeter_resetFifo(void)
{
	pulseOximeter_writeRegister(FIFO_WRITE_PTR, 0);
	pulseOximeter_writeRegister(FIFO_READ_POINTER, 0);
	pulseOximeter_writeRegister(FIFO_OVF_COUNTER, 0);
}

//
void pulseOximeter_initFifo(void)
{
	// Check if polling/interrupt mode
	if( PUSLE_OXIMETER_INTERRUPT == 1 )
	{
		pulseOximeter_writeRegister(FIFO_CONFIG, 0x0F);

		// FIFO almost full interrupt enable
		pulseOximeter_writeRegister(INT_ENABLE_1|INT_ENABLE_2, 0x40);

		pulseOximeter_clearInterrupt();
	}else{
		pulseOximeter_writeRegister(FIFO_CONFIG, 0x0F);
		pulseOximeter_writeRegister(INT_ENABLE_1, 0x00);
	}
}

FIFO_LED_DATA pulseOximeter_readFifo(void)
{
	uint8_t address;uint8_t buf[12];
	uint8_t numBytes = 6;

	buf[0] = FIFO_DATA;

	address = (I2C_SLAVE_ID | I2C_WRITE);

	HAL_I2C_Master_Transmit(&hi2c2, address, buf, 1, HAL_MAX_DELAY);

	address = (I2C_SLAVE_ID | I2C_READ);
	HAL_I2C_Master_Receive(&hi2c2, address, buf, numBytes, HAL_MAX_DELAY);

	fifoData.irLedRaw = 0;
	fifoData.redLedRaw = 0;

	fifoData.irLedRaw = (buf[4] << 8) | (buf[5] << 0);
	fifoData.redLedRaw =(buf[1] << 8) | (buf[0] << 0);

	return fifoData;
}


int8_t pulseOximeter_readFIFO(uint8_t* dataBuf, uint8_t numBytes)
{
	HAL_StatusTypeDef retStatus;
	uint8_t buf[12];

	buf[0] = FIFO_DATA;

	uint8_t address = (I2C_SLAVE_ID | I2C_WRITE);

	retStatus = HAL_I2C_Master_Transmit(&hi2c2, address, buf, 1, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	address = (I2C_SLAVE_ID | I2C_READ);
	retStatus = HAL_I2C_Master_Receive(&hi2c2, address, buf, numBytes, HAL_MAX_DELAY);
	if( retStatus != HAL_OK ){
		return -1;
	}

	for(int i = 0; i < numBytes; i++)
	{
		*dataBuf = buf[i];
		dataBuf++;
	}

	return 0;
}

// Read the INT STATUS register to clear interrupt
void pulseOximeter_clearInterrupt(void)
{
	uint8_t readResult;

	pulseOximeter_readRegister(INT_STATUS_1, &readResult);
}


float pulseOximeter_readTemperature(void)
{
	uint8_t tempDone = 1;
	uint8_t readResult;
	int8_t tempFraction;
	uint8_t tempInteger;
	float temperature;

	// Initiate a temperature conversion
	pulseOximeter_writeRegister(DIE_TEMP_CONFIG, 1);

	// Wait for conversion finish
	while( tempDone != 0 )
	{
		pulseOximeter_readRegister(DIE_TEMP_CONFIG, &tempDone);
	}

	// Read Die temperature integer register
	pulseOximeter_readRegister(DIE_TEMP_INTEGER, &readResult);
	tempInteger = readResult;

	// Read Die temperature fraction register
	pulseOximeter_readRegister(DIE_TEMP_FRACTION, &readResult);
	tempFraction = readResult;

	// Conversion factor found in MAX30102 DataSheet
	temperature = tempInteger + (tempFraction*0.0625);

	return temperature;
}





MAX30102 pulseOximeter_update(FIFO_LED_DATA m_fifoData)
{
	MAX30102 result = {
	/*bool pulseDetected*/ false,
	/*float heartBPM*/ 0.0,
	/*float irCardiogram*/ 0.0,
	/*float irDcValue*/ 0.0,
	/*float redDcValue*/ 0.0,
	/*float SaO2*/ currentSpO2Value,
	/*uint32_t lastBeatThreshold*/ 0,
	/*float dcFilteredIR*/ 0.0,
	/*float dcFilteredRed*/ 0.0,
	/*float temperature;*/ 0.0
  };

	result.temperature = pulseOximeter_readTemperature();

	dcFilterIR = dcRemoval( (float)m_fifoData.irLedRaw, dcFilterIR.w, ALPHA );
	dcFilterRed = dcRemoval( (float)m_fifoData.redLedRaw, dcFilterRed.w, ALPHA );

	float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
	lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

	irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
	redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
	samplesRecorded++;

	if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
	{
		result.pulseDetected=true;
		pulsesDetected++;

//		float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

        // RMS ratio for Red/IR signals
        float irRMS = sqrt(irACValueSqSum / samplesRecorded);
        float redRMS = sqrt(redACValueSqSum / samplesRecorded);
        float ratioRMS = (redRMS / irRMS);

        // Improved SpO2 formula
        currentSpO2Value = 104.0 - (17.0 * ratioRMS); // Calibrated formula
        if (currentSpO2Value < 70 || currentSpO2Value > 100) {
            currentSpO2Value = 0; // Out of range, invalid
        }


		result.SpO2 = currentSpO2Value;

		if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
		{
			irACValueSqSum = 0;
			redACValueSqSum = 0;
			samplesRecorded = 0;
		}
	}

	balanceIntesities( dcFilterRed.w, dcFilterIR.w );


	result.heartBPM = currentBPM;
	result.irCardiogram = lpbFilterIR.result;
	result.irDcValue = dcFilterIR.w;
	result.redDcValue = dcFilterRed.w;
	result.lastBeatThreshold = lastBeatThreshold;
	result.dcFilteredIR = dcFilterIR.result;
	result.dcFilteredRed = dcFilterRed.result;

	return result;
}


bool detectPulse(float sensor_value) {
    static float prev_sensor_value = 0;
    static float dynamic_threshold = 40; // Initial threshold
    static float decay_factor = 1;        // Threshold decay factor
    static uint32_t refractory_period = 200; // Minimum interval between peaks (ms)
    static uint32_t last_peak_time = 0;
    static float max_val_in_window = 0;
    static uint32_t max_pos_in_window = 0;
    static uint8_t in_peak_window = 0;

    uint32_t current_time = millis();
    uint32_t time_since_last_peak = current_time - last_peak_time;

    // Decay threshold over time
    dynamic_threshold *= decay_factor;

    if (sensor_value > dynamic_threshold && time_since_last_peak > refractory_period) {
        if (!in_peak_window) {
            // Entering a potential peak window
            in_peak_window = 1;
            max_val_in_window = sensor_value;
            max_pos_in_window = current_time;
        } else if (sensor_value > max_val_in_window) {
            // Update peak within the window
            max_val_in_window = sensor_value;
            max_pos_in_window = current_time;
        }
    } else if (in_peak_window) {
        // Exiting peak window, confirm peak
        in_peak_window = 0;
        last_peak_time = max_pos_in_window;

        // Update threshold based on detected peak
        dynamic_threshold = 0.7 * dynamic_threshold + 0.3 * max_val_in_window;

        return true; // Pulse detected
    }

    // No pulse detected
    prev_sensor_value = sensor_value;
    return false;
}


void balanceIntesities( float redLedDC, float IRLedDC )
{
	uint32_t currentTime = millis();
  if( currentTime - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
  {
	  if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA)
    {
      redLEDCurrent++;
      pulseOximeter_setLedCurrent(RED_LED, redLEDCurrent);
	  pulseOximeter_setLedCurrent(IR_LED, IrLedCurrent);
    }
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
    {
      redLEDCurrent--;pulseOximeter_setLedCurrent(RED_LED, redLEDCurrent);
      pulseOximeter_setLedCurrent(IR_LED, IrLedCurrent);
    }

    lastREDLedCurrentCheck = millis();
  }
}

// Print registers out to uart
void pulseOximiter_registerDump(void)
{
	int8_t readStatus;
	uint8_t readResult;
	//uint8_t buf[12];

	readStatus = pulseOximeter_readRegister(INT_STATUS_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_STATUS_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_STATUS_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_STATUS_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_ENABLE_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	//itoa(readResult, buf, 16);   // here 16 means Hexadecimal
	uart_PrintString("INT_ENABLE_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(INT_ENABLE_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("INT_ENABLE_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_WRITE_PTR, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_WRITE_PTR: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_OVF_COUNTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_OVF_COUNTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_READ_POINTER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_READ_POINTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_DATA, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_DATA_REGISTER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(FIFO_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("FIFO_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(MODE_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("MODE_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(SPO2_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("SPO2_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_PULSE_AMP_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_PULSE_AMP_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_PULSE_AMP_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_PULSE_AMP_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_MODE_CONTROL_1, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_MODE_CONTROL_1: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(LED_MODE_CONTROL_2, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("LED_MODE_CONTROL_2: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_INTEGER, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_INTEGER: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_FRACTION, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_FRACTION: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(DIE_TEMP_CONFIG, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("DIE_TEMP_CONFIG: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(REVISION_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("REVISION_ID: 0x");
	uart_PrintInt(readResult, 16);

	readStatus = pulseOximeter_readRegister(PART_ID, &readResult);
	if( readStatus == -1){
		uart_PrintString("I2C read error!");
		return;
	}

	uart_PrintString("PART_ID: 0x");
	uart_PrintInt(readResult, 16);

	uart_PrintString("\r\n");
}

void pulseOximeter_displayData(void)
{
	uart_PrintString("MAX30102 Pulse Oximiter Data:\r\n");

	uart_PrintString("Die Temperature: ");
	uart_PrintFloat(pulseOximeter.temperature);

	uart_PrintString("Heart Rate (BPM): ");
	uart_PrintFloat(pulseOximeter.heartBPM);

	uart_PrintString("Oxygen Saturation SpO2 (%): ");
	uart_PrintFloat(pulseOximeter.SpO2);
	uart_PrintString("\r\n");
}

uint32_t pulseOximeter_getSpo2(void)
{
	return pulseOximeter.SpO2;
}
