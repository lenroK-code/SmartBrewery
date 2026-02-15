#pragma once
#include <stdbool.h>

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_READY,
    CAL_STATE_MEASURING,
    CAL_STATE_SAVED,
    CAL_STATE_DONE
} cal_state_t;

void calibration_init(void); // reads NVS
void calibrate_start(uint16_t conn_handle); // 0x03
void calibrate_module_start(void); // 0x04
void calibrate_module_stop(uint16_t conn_handle); // 0x05
bool calibration_is_loaded(void);  // czy coeffs z NVS?

void calibrate_notify_start(uint16_t conn_handle);
void calibrate_notify_complete(uint16_t conn_handle);

cal_state_t calibrate_get_state(void);
bool calibrate_is_ready_for_module_start(void); // which point is being calibrated (0-5)
bool calibrate_is_ready_for_save(void);
void measurement_add_sample(float tilt);  // dla main.c
float calculate_tilt_from_accel(int ax, int ay, int az);

float calibrate_tilt_to_sg(float tilt);
float sg_to_plato(float sg);

extern int broadcast_cal_notify(uint16_t conn_handle, const char* message);
