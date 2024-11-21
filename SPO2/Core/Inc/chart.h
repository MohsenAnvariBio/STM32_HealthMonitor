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
	void lv_example_spinbox_with_chart(void);
	uint8_t event_handler(lv_event_t * e);
#ifdef __cplusplus
}
#endif

#endif /* INC_CHART_H_ */
