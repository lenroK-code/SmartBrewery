#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"

#include <math.h>
#include "calibration.h"

#define MEAS_SAMPLES 100
#define TILT_QUEUE_SIZE 256

typedef struct {
    float sg;      // specific gravity
    float reading; // raw tilt / sensor reading
} CalPoint;

typedef struct {
    float tilt[TILT_QUEUE_SIZE];
    int head, tail, count;
} tilt_queue_t;

static tilt_queue_t tilt_queue = {0};
static bool measuring = false;
static int measurement_count = 0;


static int cal_state = CAL_STATE_IDLE;
static int cal_point = 0;
static const float sg_table[6] = {1.000, 1.050, 1.040, 1.030, 1.020, 1.010};

static CalPoint points[6];
static float coeffs[4];
static bool complete = false;

static const char *TAG = "CALIBRATION";

void load_calibration_from_nvs(void);
void save_calibration_to_nvs(void);

bool calibration_is_loaded(void) {
    return complete;
}

void calibration_init(void) {
    load_calibration_from_nvs();
}

void load_calibration_from_nvs(void) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("calibration", NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Brak kalibracji w NVS");
        return;
    }
    
    size_t blob_size = 0;
    nvs_get_blob(nvs, "coeffs", NULL, &blob_size);
    if (blob_size != sizeof(float)*5) {
        nvs_close(nvs);
        return;
    }
    
    float coeffs_blob[5];
    nvs_get_blob(nvs, "coeffs", coeffs_blob, &blob_size);
    
    coeffs[0] = coeffs_blob[0];
    coeffs[1] = coeffs_blob[1];
    coeffs[2] = coeffs_blob[2];
    coeffs[3] = coeffs_blob[3];
    complete = (coeffs_blob[4] > 0.5f);
    
    ESP_LOGI(TAG, "NVS LOADED: A3=%.6f A2=%.6f A1=%.6f A0=%.6f", 
             coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    nvs_close(nvs);
}

void save_calibration_to_nvs(void) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open("calibration", NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return;
    }
    
    // BLOB: A3,A2,A1,A0,complete (20B)
    float coeffs_blob[5];
    coeffs_blob[0] = coeffs[0];  // A3
    coeffs_blob[1] = coeffs[1];  // A2
    coeffs_blob[2] = coeffs[2];  // A1
    coeffs_blob[3] = coeffs[3];  // A0
    coeffs_blob[4] = complete ? 1.0f : 0.0f;
    
    nvs_set_blob(nvs, "coeffs", coeffs_blob, sizeof(coeffs_blob));
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "NVS SAVED coeffs OK");
}

void tilt_queue_push(float tilt) {
    tilt_queue.tilt[tilt_queue.head] = tilt;
    tilt_queue.head = (tilt_queue.head + 1) % TILT_QUEUE_SIZE;
    if (tilt_queue.count < TILT_QUEUE_SIZE) tilt_queue.count++;
    else tilt_queue.tail = (tilt_queue.tail + 1) % TILT_QUEUE_SIZE;
}

float measurement_get_average(void) {
    // tu trzeba zaimplementować uśrednianie z tilt_queue - zobaczyc czy limity dzialaja czy nie
    // if (tilt_queue.count < 10) return 0.0f;
    
    float sum = 0;
    int idx = tilt_queue.head;
    int samples = (measurement_count > MEAS_SAMPLES) ? MEAS_SAMPLES : measurement_count;
    
    for(int i = 0; i < samples; i++) {
        idx = (tilt_queue.head - 1 - i + TILT_QUEUE_SIZE) % TILT_QUEUE_SIZE;
        sum += tilt_queue.tilt[idx];
    }
    measuring = false;
    measurement_count = 0;
    return sum / samples;
}

void measurement_add_sample(float tilt) {
    tilt_queue_push(tilt);
    if (measuring) measurement_count++;
}

void solve_linear_system(float A[][4], float b[], float X[], int n) {
      float aug[6][5];  // [A|b] rozszerzona macierz
    
    // Buduj rozszerzoną macierz [A|b]
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < 4; j++) {
            aug[i][j] = A[i][j];
        }
        aug[i][4] = b[i];
        ESP_LOGD(TAG, "Row %d: %.3f %.3f %.3f %.3f | %.3f", 
                 i, aug[i][0], aug[i][1], aug[i][2], aug[i][3], aug[i][4]);
    }
    
    // ELIMINACJA Gaussa
    for(int p = 0; p < 4; p++) {
        // Pivot 
        int maxr = p;
        for(int i = p+1; i < n; i++) {
            if((aug[i][p] > 0 && aug[maxr][p] < 0) || 
               (aug[i][p] < 0 && aug[maxr][p] > 0) ||
               (aug[i][p] > aug[maxr][p])) {
                maxr = i;
            }
        }
        ESP_LOGD(TAG, "Pivot col %d: row %d = %.7f", p, maxr, aug[maxr][p]);
        
        // Zamień wiersze
        for(int j = 0; j < 5; j++) {
            float tmp = aug[p][j];
            aug[p][j] = aug[maxr][j];
            aug[maxr][j] = tmp;
        }
        
        // Sprawdź singularność (ZAMIAST fabsf())
        if(aug[p][p] > -1e-6f && aug[p][p] < 1e-6f) {
            ESP_LOGE(TAG, "Singular matrix at column %d!", p);
            return;
        }
        
        // Eliminacja
        for(int i = p+1; i < n; i++) {
            float alpha = aug[i][p] / aug[p][p];
            for(int j = p; j < 5; j++) {
                aug[i][j] -= alpha * aug[p][j];
            }
        }
    }
    
    // Back-substitution
    for(int i = 3; i >= 0; i--) {
        X[i] = aug[i][4];
        for(int j = i+1; j < 4; j++) {
            X[i] -= aug[i][j] * X[j];
        }
        X[i] /= aug[i][i];
    }

    ESP_LOGI(TAG, "SOLVED: A3=%.6f A2=%.6f A1=%.6f A0=%.6f", 
             X[0], X[1], X[2], X[3]);
}

void calculate_polynomial() {
   float V[6][4], Y[6];
    for(int i = 0; i < 6; i++) {
        float t = points[i].reading;
        V[i][0] = t*t*t; V[i][1] = t*t; V[i][2] = t; V[i][3] = 1.0f;
        Y[i] = points[i].sg;
    }
    solve_linear_system(V, Y, coeffs, 6);
    complete = true;
    ESP_LOGI(TAG, "A3=%.6f A2=%.6f A1=%.6f A0=%.6f", 
             coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    save_calibration_to_nvs();
    ESP_LOGI(TAG, "KALIBRACJA ZAPISANA do NVS!");
}


// potencjalnie zmienić to na własną funkcję tilt calculation
float calculate_tilt_from_accel(float ax_raw, float ay_raw, float az_raw) {
    // Oficjalny wzór iSpindel [web:3]
    float scale = 1.0f / 16384.0f;  // MPU6050 ±2g standard
    float ax = ax_raw * scale;
    float ay = ay_raw * scale; 
    float az = az_raw * scale;
    
    // Tilt = Ax/Az (iSpindel uproszczony)
    float tilt = (az != 0.0f) ? (ax / az) * 57.3f : 0.0f;
    
    ESP_LOGD(TAG, "RAW:%.0f,%.0f,%.0f → NORM:%.3f,%.3f,%.3f → Tilt=%.2f°", 
             ax_raw, ay_raw, az_raw, ax, ay, az, tilt);
    return tilt;
}


void calibrate_start(void) {
    if (cal_state == CAL_STATE_IDLE) {
        cal_state = CAL_STATE_READY;
        cal_point = 0;
        ESP_LOGI(TAG, "CAL: START punkt %d/6 (SG=%.3f)", 
                 cal_point+1, sg_table[cal_point]);
    }
}

void calibrate_module_start(void) {
    if (cal_state == CAL_STATE_READY && cal_point < 6) {
        cal_state = CAL_STATE_MEASURING;
      measuring = true;
        measurement_count = 0;
        ESP_LOGI(TAG, "CAL: POMIAR %d/6... (queue=%d)", cal_point+1, tilt_queue.count);
    }
}

void calibrate_module_stop(void) {
    if (cal_state == CAL_STATE_MEASURING) {
       float avg_tilt = measurement_get_average();  // *** uśrednij ***
        
        // ZAPIS PUNKTU
        points[cal_point].reading = avg_tilt;
        points[cal_point].sg = sg_table[cal_point];
        
        cal_point++;
        if (cal_point >= 6) {
            calculate_polynomial();
            cal_state = CAL_STATE_DONE;
            ESP_LOGI(TAG, "CAL: KONIEC! Wielomian gotowy!");
        } else {
            cal_state = CAL_STATE_READY;
            ESP_LOGI(TAG, "CAL: ZAPISANO %d/6 Tilt=%.2f SG=%.3f", 
            cal_point, avg_tilt, sg_table[cal_point-1]);
        }
    }
}

// Guards dla BLE
bool calibrate_is_ready_for_module_start(void) {
    return cal_state == CAL_STATE_READY;
}

bool calibrate_is_ready_for_save(void) {
    return cal_state == CAL_STATE_MEASURING;
}


float calibrate_tilt_to_sg(float tilt) {
    if (!complete) return 1.000f;
    float sg = coeffs[0]*tilt*tilt*tilt + 
               coeffs[1]*tilt*tilt + 
               coeffs[2]*tilt + 
               coeffs[3];
    ESP_LOGI(TAG, "Tilt=%.2f° → SG=%.3f", tilt, sg);
    return sg;
}

float sg_to_plato(float sg) {
    return (-616.868 + 1111.14*sg - 630.272*sg*sg + 135.997*sg*sg*sg);
}