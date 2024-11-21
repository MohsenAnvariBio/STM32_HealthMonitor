/*
 * spinbox.c
 *
 *  Created on: Nov 20, 2024
 *      Author: Mohsen PC
 */

#include"lvgl/lvgl.h"
#include"chart.h"

#if LV_USE_SPINBOX && LV_BUILD_EXAMPLES

static lv_obj_t *spinbox; // Declare the spinbox globally to use its value elsewhere
static lv_obj_t *chart;   // Chart object
static lv_chart_series_t *ser2; // Chart series
lv_obj_t * label_t;
bool use_moving_average = false;

// Increment event callback
static void lv_spinbox_increment_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox);
    }
}

// Decrement event callback
static void lv_spinbox_decrement_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox);
    }
}

// Function to update the chart dynamically
void update_chart_with_gain(float output) {
    // Get the value from the spinbox
    int gain = lv_spinbox_get_value(spinbox);

    // Use the spinbox value as the gain in the chart calculation
    lv_chart_set_next_value(chart, ser2, (-output / gain) );
}

void switch_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *obj = lv_event_get_target(e);
        use_moving_average = lv_obj_has_state(obj, LV_STATE_CHECKED);
    }
}
bool is_moving_average_enabled() {
    return use_moving_average;
}
// Spinbox and chart example initialization
void setup_ui(void) {

	// Title
	lv_obj_t *label_t = lv_label_create(lv_scr_act());
	lv_label_set_long_mode(label_t, LV_LABEL_LONG_WRAP);     /* Break the long lines if needed */
	lv_label_set_recolor(label_t, true);                    /* Enable re-coloring by commands in the text */
	lv_label_set_text(label_t, "#0000ff SPO2 Measurement#"); /* Title text with blue color */
	// Style the title
	lv_obj_set_style_text_font(label_t, &lv_font_montserrat_22, LV_PART_MAIN); /* Larger, modern font */
	lv_obj_set_style_text_color(label_t, lv_color_hex(0xff0000), LV_PART_MAIN); /* Add a secondary color (red for title text) */
	lv_obj_set_style_text_align(label_t, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);   /* Center the text alignment */
	lv_obj_align(label_t, LV_ALIGN_TOP_MID, 0, 2); /* Adjust alignment and position */


    // Create a chart
    chart = lv_chart_create(lv_scr_act());
	lv_obj_set_size(chart, 310, 120);
	lv_obj_align_to(chart, label_t, LV_ALIGN_TOP_MID,0,25);
//	lv_obj_align(chart, LV_ALIGN_CENTER, 0, -25);
	lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
	lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
	lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -200, 200);
	uint16_t cnt = 1000;
	lv_chart_set_point_count(chart, cnt);
    // Add a series to the chart
    ser2 = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    // Create a spinbox
	spinbox = lv_spinbox_create(lv_scr_act());
	lv_spinbox_set_range(spinbox, 1, 100); // Set the gain range
	lv_spinbox_set_digit_format(spinbox, 1, 0);
	lv_spinbox_set_value(spinbox, 40); // Default gain value
	lv_obj_set_width(spinbox, 0);
	lv_obj_set_height(spinbox, 20);
	lv_obj_align_to(spinbox, chart, LV_ALIGN_TOP_RIGHT, -20, 85);
	lv_coord_t h = lv_obj_get_height(spinbox);


	// Add a "+" button for incrementing the spinbox
	lv_obj_t *btn = lv_btn_create(lv_scr_act());
	lv_obj_set_size(btn, h, h);
	lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_RIGHT_MID, 1, 0);
	lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_MINUS, 0);
	lv_obj_add_event_cb(btn, lv_spinbox_increment_event_cb, LV_EVENT_ALL, NULL);

	// Add a "-" button for decrementing the spinbox
	btn = lv_btn_create(lv_scr_act());
	lv_obj_set_size(btn, h, h);
	lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_PLUS, 0);
	lv_obj_add_event_cb(btn, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);

    // Create the switch
    lv_obj_t *sw = lv_switch_create(lv_scr_act());
    lv_obj_set_size(sw, 40, 20);
    lv_obj_align(sw, LV_ALIGN_TOP_RIGHT, -10, 10); // Align switch on screen
    lv_obj_align_to(sw, chart, LV_ALIGN_TOP_LEFT, +80, 85);
    lv_obj_add_event_cb(sw, switch_event_handler, LV_EVENT_ALL, NULL);

    // Optionally, add a label for the switch
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Enable MA");
    lv_obj_align_to(label, sw, LV_ALIGN_OUT_LEFT_MID, -5, 0);

}

#endif
