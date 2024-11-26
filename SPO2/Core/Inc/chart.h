/*
 * chart.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Mohsen PC
 */

#ifndef INC_CHART_H_
#define INC_CHART_H_

#ifdef __cplusplus
extern "C" {
#endif
	void update_chart_with_gain(float output);
	void update_SPO2(float spo2) ;
	void update_HR(uint32_t hr);
	void setup_ui(void);
	bool event_handler(lv_event_t * e);

	bool is_moving_average_enabled();
#ifdef __cplusplus
}
#endif

#endif /* INC_CHART_H_ */
