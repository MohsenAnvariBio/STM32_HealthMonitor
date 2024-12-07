/*
 * ui.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Mohsen PC
 */

#ifndef INC_UI_H_
#define INC_UI_H_

#ifdef __cplusplus
extern "C" {
#endif

	#define COLOR_BACKGND lv_color_make(131, 165, 133)
	#define COLOR_FONT lv_color_make(0x00, 0x0, 0x00)
	#define INVALID_VALUE 0xFFFFFFFF

	void update_chart_with_gain(float output);
	void setup_ui(void);
	bool event_handler(lv_event_t * e);
	bool is_moving_average_enabled();
	void update_heartimg(bool wimg);
	extern const uint8_t rb1cm_map[];

	void update_SPO2(uint32_t spo2);
	void update_HR(uint32_t hr);
	void update_temp(uint32_t t);



#ifdef __cplusplus
}
#endif

#endif /* INC_UI_H_ */
