/*
 * start.h
 *
 *  Created on: Nov 30, 2024
 *      Author: demom
 */

#ifndef INC_START_H_
#define INC_START_H_

#ifdef __cplusplus
extern "C" {
#endif


//
//	void start_screen(void);
//	void setup_start_button();

	void create_splash_screen(void);
	void show_main_screen(void);
	void splash_screen_timer_cb(lv_timer_t * timer) ;

#ifdef __cplusplus
}
#endif

#endif /* INC_START_H_ */

