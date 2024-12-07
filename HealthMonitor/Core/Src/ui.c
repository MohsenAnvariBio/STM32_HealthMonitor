/*
 * ui.c
 *
 *  Created on: Nov 20, 2024
 *      Author: Mohsen PC
 */

#include <image.h>
#include"lvgl/lvgl.h"
#include "system.h"
#include "string.h"
#include <stdio.h>
#include "ui.h"


#if LV_USE_SPINBOX && LV_BUILD_EXAMPLES

static lv_obj_t *spinbox;
static lv_obj_t *chart;
static lv_chart_series_t *ser2;


lv_obj_t *label_t;
bool use_moving_average = false;
lv_obj_t *labelspo2;
lv_obj_t *labelHR;
lv_obj_t *labelTE;
LV_FONT_DECLARE(lv_font_montserrat_20); // Modern medium-sized font for labels

// Increment event callback
static void lv_spinbox_decrement_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox);
    }
}

// Decrement event callback
static void lv_spinbox_increment_event_cb(lv_event_t *e) {
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

void update_SPO2(uint32_t spo2) {
	if (spo2 == INVALID_VALUE){
		lv_label_set_text(labelspo2, "--");
	}else{
		char buffer[20];  // Ensure the buffer is large enough to hold the text
	    snprintf(buffer, sizeof(buffer), "%d", spo2);  // Convert the value to a string
	    lv_label_set_text(labelspo2, buffer);  // Update the label text with the formatted string
	}
}

void update_HR(uint32_t hr) {
	if (hr == INVALID_VALUE){
		lv_label_set_text(labelHR, "--");
	}else{
	    char buffer[20];
	    snprintf(buffer, sizeof(buffer), "%d", hr);
	    lv_label_set_text(labelHR, buffer);
	}
}

void update_temp(uint32_t t) {
	if (t == INVALID_VALUE){
		lv_label_set_text(labelTE, "--");
	}else{
	    char buffer[20];
	    snprintf(buffer, sizeof(buffer), "%d", t);
	    lv_label_set_text(labelTE, buffer);
	}
}


void update_heartimg(bool wimg) {
    static lv_obj_t *heart_img = NULL;
    // Create the image object only if it doesn't exist
    if (heart_img == NULL) {
        heart_img = lv_img_create(lv_scr_act());
        lv_obj_align(heart_img, LV_ALIGN_TOP_RIGHT, -17, 4); // Initial position
    }
    // Update the image source based on the `wimg` flag
    if (wimg) {
        lv_img_set_src(heart_img, &rb25mm); // Red-black image
    } else {
        lv_img_set_src(heart_img, &b25mm); // Black-only image
    }
}


void switch_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t *obj = lv_event_get_target(e);
        use_moving_average = lv_obj_has_state(obj, LV_STATE_CHECKED);
        // Update the chart series color based on `use_moving_average`
        if (use_moving_average) {
            lv_chart_set_series_color(chart, ser2, COLOR_FONT); // Set to green
        } else {
            lv_chart_set_series_color(chart, ser2, COLOR_FONT);  // Set to blue
        }
    }
}
bool is_moving_average_enabled() {
    return use_moving_average;
}


void setup_ui(void) {
    // Set screen background color
    lv_obj_set_style_bg_color(lv_scr_act(), COLOR_BACKGND, 0);

    // Image heart black
    lv_obj_t *heart_blk_img = lv_img_create(lv_scr_act());
    lv_img_set_src(heart_blk_img, &b25mm);
    lv_obj_align(heart_blk_img, LV_ALIGN_TOP_RIGHT, -17, 4); // Position to the

    // Image title
    lv_obj_t *title_img = lv_img_create(lv_scr_act());
    lv_img_set_src(title_img, &hm);
    lv_obj_align(title_img, LV_ALIGN_TOP_LEFT, 10, 8); // Position to the

    // Chart setup (unchanged)
    chart = lv_chart_create(lv_scr_act());
    lv_obj_set_size(chart, 330, 120);
    lv_obj_align(chart, LV_ALIGN_OUT_BOTTOM_MID, -5, 40);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -200, 200);
    lv_chart_set_point_count(chart, 1000);
    lv_chart_set_div_line_count(chart, 10, 12);
    ser2 = lv_chart_add_series(chart, COLOR_FONT, LV_CHART_AXIS_PRIMARY_Y);

    // Set chart background color
    lv_obj_set_style_bg_color(chart, COLOR_BACKGND, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(chart, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(chart, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(chart, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(chart, 10, LV_PART_MAIN | LV_STATE_DEFAULT); // Rounded corners

    // Define points for the lines
    static lv_point_t top_line_points[] = { {0, 0}, {330, 0} };   // Width of the chart
    static lv_point_t bottom_line_points[] = { {0, 0}, {330, 0} };

    // Create the top line
    lv_obj_t *top_line = lv_line_create(lv_scr_act());
    lv_line_set_points(top_line, top_line_points, 2);  // Set points for the line
    lv_obj_set_style_line_width(top_line, 1, LV_PART_MAIN); // Line thickness
    lv_obj_set_style_line_color(top_line, lv_palette_main(LV_PALETTE_NONE), LV_PART_MAIN); // Line color
    lv_obj_set_style_line_opa(top_line, LV_OPA_COVER, LV_PART_MAIN); // Line opacity
    lv_obj_align_to(top_line, chart, LV_ALIGN_OUT_TOP_MID, 0, 0);  // Position above the chart
    // Create the bottom line
    lv_obj_t *bottom_line = lv_line_create(lv_scr_act());
    lv_line_set_points(bottom_line, bottom_line_points, 2); // Set points for the line
    lv_obj_set_style_line_width(bottom_line, 1, LV_PART_MAIN); // Line thickness
    lv_obj_set_style_line_color(bottom_line, lv_palette_main(LV_PALETTE_NONE), LV_PART_MAIN); // Line color
    lv_obj_set_style_line_opa(bottom_line, LV_OPA_COVER, LV_PART_MAIN); // Line opacity
    lv_obj_align_to(bottom_line, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, 2); // Position below the chart

    // Spinbox
    spinbox = lv_spinbox_create(lv_scr_act());
    lv_spinbox_set_range(spinbox, 1, 100);
    lv_spinbox_set_digit_format(spinbox, 1, 0);
    lv_spinbox_set_value(spinbox, 40);
//    lv_spinbox_set_cursor_pos(spinbox, LV_SPINBOX_CURSOR_NONE); // Remove the cursor
    lv_obj_set_width(spinbox, 0);
    lv_obj_align_to(spinbox, chart, LV_ALIGN_OUT_RIGHT_TOP, -50, 85);
	// Common Style for Buttons
	static lv_style_t btn_style;
	lv_style_init(&btn_style);
	lv_style_set_bg_color(&btn_style, COLOR_FONT); // Light green color
	lv_style_set_bg_opa(&btn_style, LV_OPA_COVER);                  // Full opacity
	lv_style_set_radius(&btn_style, 8);                             // Rounded corners
	lv_style_set_border_width(&btn_style, 2);                       // Border width
	lv_style_set_border_color(&btn_style, lv_color_make(0, 0, 0));  // Black border
	lv_style_set_text_color(&btn_style, lv_color_white());          // Black text for label
	// "+" Button
	lv_obj_t *btn_plus = lv_btn_create(lv_scr_act());
	lv_obj_add_style(btn_plus, &btn_style, LV_PART_MAIN);           // Apply style
	lv_obj_set_size(btn_plus, 40, 20);
	lv_obj_align_to(btn_plus, spinbox, LV_ALIGN_OUT_RIGHT_MID, 0, 0);
	lv_obj_add_event_cb(btn_plus, lv_spinbox_increment_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_t *btn_plus_label = lv_label_create(btn_plus);
	lv_label_set_text(btn_plus_label, "+");
	lv_obj_set_style_text_font(btn_plus_label, &lv_font_montserrat_22, 0);
	lv_obj_center(btn_plus_label);
	// "-" Button
	lv_obj_t *btn_minus = lv_btn_create(lv_scr_act());
	lv_obj_add_style(btn_minus, &btn_style, LV_PART_MAIN);          // Apply style
	lv_obj_set_size(btn_minus, 40, 20);
	lv_obj_align_to(btn_minus, spinbox, LV_ALIGN_OUT_LEFT_MID, -10, 0);
	lv_obj_add_event_cb(btn_minus, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);
	lv_obj_t *btn_minus_label = lv_label_create(btn_minus);
	lv_label_set_text(btn_minus_label, "-");
	lv_obj_set_style_text_font(btn_minus_label, &lv_font_montserrat_22, 0); // Slightly smaller font
//	    lv_obj_center(btn_minus_label);
	lv_obj_align(btn_minus_label, LV_ALIGN_CENTER, 0, -2);

	// Switch
	lv_obj_t *sw = lv_switch_create(lv_scr_act());
	lv_obj_set_size(sw, 40, 20);
	lv_obj_align(sw, LV_ALIGN_TOP_RIGHT, -10, 10); // Align switch on screen
	lv_obj_align_to(sw, chart, LV_ALIGN_TOP_LEFT, +30, 85);
	lv_obj_add_event_cb(sw, switch_event_handler, LV_EVENT_ALL, NULL);
	lv_obj_set_style_bg_color(sw, COLOR_FONT, LV_PART_INDICATOR | LV_STATE_CHECKED);
	lv_obj_set_style_bg_opa(sw, LV_OPA_COVER, LV_PART_INDICATOR | LV_STATE_CHECKED);
	// Optionally, add a label for the switch
	lv_obj_t *label = lv_label_create(lv_scr_act());
	lv_label_set_text(label, "MA");
	lv_obj_align_to(label, sw, LV_ALIGN_OUT_LEFT_MID, -5, 0);

	// Container for SpO2 Label
	lv_obj_t *container_spo2 = lv_obj_create(lv_scr_act());
	lv_obj_set_size(container_spo2, 90, 70);
	lv_obj_align_to(container_spo2, chart, LV_ALIGN_OUT_BOTTOM_LEFT, 10, 7); // Position below the chart
	lv_obj_set_style_radius(container_spo2, 10, 0); // Rounded corners
	lv_obj_set_style_bg_color(container_spo2, COLOR_BACKGND, 0); // Light green background
	lv_obj_set_style_border_color(container_spo2, COLOR_FONT, 0); // Black border
	lv_obj_set_style_border_width(container_spo2, 2, 0);
	// SpO2 Fixed Text
	lv_obj_t *label_spo2_fixed = lv_label_create(container_spo2);
	lv_label_set_text(label_spo2_fixed, "SPO2");
	lv_obj_set_style_text_font(label_spo2_fixed, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_spo2_fixed, COLOR_FONT, 0); // Black text
	lv_obj_align(label_spo2_fixed, LV_ALIGN_TOP_MID, 0, -8); // Align near the top
	// SpO2 Unit
	lv_obj_t *label_spo2_unit = lv_label_create(container_spo2);
	lv_label_set_text(label_spo2_unit, "%");
	lv_obj_set_style_text_font(label_spo2_unit, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_spo2_unit, COLOR_FONT, 0); // Black text
	lv_obj_align(label_spo2_unit, LV_ALIGN_BOTTOM_RIGHT, 10, 9); // Align near the top
	// SpO2 Changing Text
	labelspo2 = lv_label_create(container_spo2);
	lv_label_set_text(labelspo2, "--");
	lv_obj_set_style_text_font(labelspo2, &lv_font_montserrat_28, 0); // Larger font for dynamic data
	lv_obj_set_style_text_color(labelspo2, COLOR_FONT, 0); // Black text
	lv_obj_align(labelspo2, LV_ALIGN_CENTER, 0, 2); // Align near the bottom

	// Container for Heart Rate Label
	lv_obj_t *container_hr = lv_obj_create(lv_scr_act());
	lv_obj_set_size(container_hr, 90, 70);
	lv_obj_align_to(container_hr, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, 7); // Position below SpO2 container
	lv_obj_set_style_radius(container_hr, 10, 0); // Rounded corners
	lv_obj_set_style_bg_color(container_hr, COLOR_BACKGND, 0); // Light green background
	lv_obj_set_style_border_color(container_hr, COLOR_FONT, 0); // Black border
	lv_obj_set_style_border_width(container_hr, 2, 0);
	// HR Fixed Text
	lv_obj_t *label_hr_fixed = lv_label_create(container_hr);
	lv_label_set_text(label_hr_fixed, "HR");
	lv_obj_set_style_text_font(label_hr_fixed, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_hr_fixed, COLOR_FONT, 0); // Black text
	lv_obj_align(label_hr_fixed, LV_ALIGN_TOP_MID, 0, -8); // Align near the top
	lv_obj_t *label_hr_unit = lv_label_create(container_hr);
	lv_label_set_text(label_hr_unit, "bpm");
	lv_obj_set_style_text_font(label_hr_unit, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_hr_unit, COLOR_FONT, 0); // Black text
	lv_obj_align(label_hr_unit, LV_ALIGN_BOTTOM_RIGHT, 10, 9); // Align near the top
	// HR Changing Text
	labelHR = lv_label_create(container_hr);
	lv_label_set_text(labelHR, "--");
	lv_obj_set_style_text_font(labelHR, &lv_font_montserrat_28, 0); // Larger font for dynamic data
	lv_obj_set_style_text_color(labelHR, COLOR_FONT, 0); // Black text
	lv_obj_align(labelHR, LV_ALIGN_CENTER, 0, 2); // Align near the bottom

	// Container for Temperature Label
	lv_obj_t *container_te = lv_obj_create(lv_scr_act());
	lv_obj_set_size(container_te, 90, 70);
	lv_obj_align_to(container_te, chart, LV_ALIGN_OUT_BOTTOM_RIGHT, -10, 7); // Position below SpO2 container
	lv_obj_set_style_radius(container_te, 10, 0); // Rounded corners
	lv_obj_set_style_bg_color(container_te, COLOR_BACKGND, 0); // Light green background
	lv_obj_set_style_border_color(container_te, COLOR_FONT, 0); // Black border
	lv_obj_set_style_border_width(container_te, 2, 0);
	// TEMP Fixed Text
	lv_obj_t *label_temp_fixed = lv_label_create(container_te);
	lv_label_set_text(label_temp_fixed, "TEMP");
	lv_obj_set_style_text_font(label_temp_fixed, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_temp_fixed, COLOR_FONT, 0); // Black text
	lv_obj_align(label_temp_fixed, LV_ALIGN_TOP_MID, 0, -8); // Align near the top
	lv_obj_t *label_temp_unit = lv_label_create(container_te);
	lv_label_set_text(label_temp_unit, "\u00B0C");
	lv_obj_set_style_text_font(label_temp_unit, &lv_font_montserrat_16, 0); // Slightly smaller font
	lv_obj_set_style_text_color(label_temp_unit, COLOR_FONT, 0); // Black text
	lv_obj_align(label_temp_unit, LV_ALIGN_BOTTOM_RIGHT, 10, 9); // Align near the top
	// TEMP Changing Text
	labelTE = lv_label_create(container_te);
	lv_label_set_text(labelTE, "--");
	lv_obj_set_style_text_font(labelTE, &lv_font_montserrat_28, 0); // Larger font for dynamic data
	lv_obj_set_style_text_color(labelTE, COLOR_FONT, 0); // Black text
	lv_obj_align(labelTE, LV_ALIGN_CENTER, 0, 2); // Align near the bottom
}

#endif
