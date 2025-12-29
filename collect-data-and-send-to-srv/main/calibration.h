#pragma once
#include <stdbool.h>

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_READY,
    CAL_STATE_MEASURING,
    CAL_STATE_SAVED,
    CAL_STATE_DONE
} cal_state_t;

void calibrate_start(void); // 0x03
void calibrate_module_start(void); // 0x04
void calibrate_module_stop(void);  // 0x05

cal_state_t calibrate_get_state(void);
bool calibrate_is_ready_for_module_start(void); // which point is being calibrated (0-5)
bool calibrate_is_ready_for_save(void);
void measurement_add_sample(float tilt);  // dla main.c
float calculate_tilt_from_accel(float ax, float ay, float az);

float calibrate_tilt_to_sg(float tilt);
float sg_to_plato(float sg);

void calibration_init(void);
bool calibration_is_loaded(void);  // czy coeffs z NVS?