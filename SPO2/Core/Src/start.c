/*
 * start.c
 *
 *  Created on: Nov 30, 2024
 *      Author: demom
 */
//#include "lvgl/examples/lv_examples.h"
#include "image.h"
#include "start.h"
#include "main.h"

#define SPLASH_SCREEN_DELAY 5000
lv_obj_t *detail_img= NULL;
lv_obj_t *name_img= NULL;
lv_obj_t *title_img = NULL;
lv_obj_t *heart_blk_img = NULL;

// Flag to check if splash screen has been displayed
bool splash_shown = false;
uint8_t startFinish = 0;

void show_main_screen(void) {
    if (detail_img != NULL) {
        lv_obj_del(detail_img);
        detail_img = NULL;
    }

    if(name_img != NULL){
        lv_obj_del(name_img);
        name_img = NULL;
    }
    if (title_img != NULL) {
        lv_obj_del(title_img);
        title_img = NULL;
    }

    if(heart_blk_img != NULL){
        lv_obj_del(heart_blk_img);
        heart_blk_img = NULL;
    }

    // Create the main screen (main interface setup)
    setup_ui();
    startFinish = 1;
}

// Timer callback to transition from splash screen to the main interface
void splash_screen_timer_cb(lv_timer_t * timer) {
    // After 5 seconds, transition to the main interface
    show_main_screen();
}

void create_splash_screen(void) {
    if (splash_shown) {
        return; // Ensure splash screen is only created once
    }

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(131, 165, 133), 0);

    // Image heart black
    heart_blk_img = lv_img_create(lv_scr_act());
    lv_img_set_src(heart_blk_img, &b25mm);
    lv_obj_align(heart_blk_img, LV_ALIGN_TOP_RIGHT, -17, 4); // Position to the

    // Image title
    title_img = lv_img_create(lv_scr_act());
    lv_img_set_src(title_img, &hm);
    lv_obj_align(title_img, LV_ALIGN_TOP_LEFT, 10, 8); // Position to the

    // Image title
    detail_img = lv_img_create(lv_scr_act());
    lv_img_set_src(detail_img, &detail);
    lv_obj_align(detail_img, LV_ALIGN_TOP_LEFT, 10, 50); // Position to the

    // Image title
    name_img = lv_img_create(lv_scr_act());
    lv_img_set_src(name_img, &name);
    lv_obj_align(name_img, LV_ALIGN_BOTTOM_LEFT, 10, -20); // Position to the


    // Set the flag to indicate splash screen has been shown
    splash_shown = true;

    // Create a timer to wait for 5 seconds before transitioning to the main screen
    lv_timer_t *splash_timer = lv_timer_create(splash_screen_timer_cb, 5000, NULL);
    lv_timer_set_repeat_count(splash_timer, 1); // Run only once
}



