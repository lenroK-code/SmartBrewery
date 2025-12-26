enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_READY,
    CAL_STATE_MEASURING,
    CAL_STATE_SAVED
};

void calibrate_module_start();
void calibrate_module_stop();
void calculate_polynomial();
void solve_linear_system(float A[][4], float b[], float X[], int n);