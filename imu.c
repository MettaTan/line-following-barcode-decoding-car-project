#include "imu.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>


// Low-pass filtered accelerometer values
static float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;


void imu_init(void) {
    // Initialise I2C AT 400 kHz frequency (fast mode: faster data transf, reduce read latency)
    i2c_init(I2C_PORT, 400 * 1000);

    // Set up GPIO pins for I2C
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Enable internal pull-up resistors for I2C
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    sleep_ms(100);
    printf("I2C initialised for IMU\n");

    // Accelerometer setup
    uint8_t accel_ctrl1[2] = {CTRL_REG1_A, 0x57};
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, accel_ctrl1, 2, false);

    uint8_t accel_ctrl4[2] = {CTRL_REG4_A, 0x00};
    i2c_write_blocking(I2C_PORT, ACCEL_ADDR, accel_ctrl4, 2, false);

    // // Magnetometer setup
    uint8_t mag_cfgA[2] = {MAG_CRA_REG_M, 0x7C}; // 16-bit
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfgA, 2, false);

    uint8_t mag_cfgB[2] = {MAG_CRB_REG_M, 0x20}; // Continuous mode
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mag_cfgB, 2, false);

    uint8_t mode[2] = {MAG_MR_REG_M, 0x00}; // 100Hz, 16-bit
    i2c_write_blocking(I2C_PORT, MAG_ADDR, mode, 2, false);
}

// Raw Sensor Readers
bool read_accel(float *ax, float *ay, float *az) {
    uint8_t data[6];
    uint8_t reg = OUT_X_L_A | 0x80; // Set MSB for auto-increment

    if (i2c_write_blocking(I2C_PORT, ACCEL_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    if (i2c_read_blocking(I2C_PORT, ACCEL_ADDR, data, 6, false) != 6) {
        return false;
    }

    int16_t x_raw = (int16_t)(data[1] << 8 | data[0]);
    int16_t y_raw = (int16_t)(data[3] << 8 | data[2]);
    int16_t z_raw = (int16_t)(data[5] << 8 | data[4]);
    
    *ax = x_raw * ACCEL_LSB_TO_MS2;
    *ay = y_raw * ACCEL_LSB_TO_MS2;
    *az = z_raw * ACCEL_LSB_TO_MS2;

    return true;
    }

// Returns magnetometer X, Y in raw units (only need heading = atan2(Y, X))
bool read_mag_xy(float* mx, float* my) {
    uint8_t data[6];
    uint8_t reg = MAG_OUT_X_H_M;

    if (i2c_write_blocking(I2C_PORT, MAG_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    if (i2c_read_blocking(I2C_PORT, MAG_ADDR, data, 6, false) != 6) {
        return false;
    }

    // Little-endian: XL, XH, YL, YH, ZL, ZH
    int16_t x_raw = (int16_t)(data[0] << 8 | data[1]);
    int16_t y_raw = (int16_t)(data[4] << 8 | data[5]);

    const float scale_xy = 1.0f / 1100.0f;
    *mx = (float)x_raw * scale_xy;
    *my = (float)y_raw * scale_xy;

    return true;
}

void imu_get_heading(float* heading_deg) {
    static float last_valid_heading = 0.0f;
    float mx, my;

    if (!read_mag_xy(&mx, &my)) {
        *heading_deg = last_valid_heading;;
        return;
    }

    // Calibration and scaling
    float mx_corr = (mx - MAG_X_OFFSET) * MAG_SCALE_X;
    float my_corr = (my - MAG_Y_OFFSET) * MAG_SCALE_Y;

    // Normalise vector
    float norm = sqrtf(mx_corr * mx_corr + my_corr * my_corr);
    if (norm < 1e-3f) {
        *heading_deg = last_valid_heading;
        return;
    }
    mx_corr /= norm;
    my_corr /= norm;

    float heading = atan2f(my_corr, mx_corr) * (180.0f / M_PI);
    
    if (heading < 0) {
        heading += 360.0f;
    }
    
    // Mount offset
    heading = fmodf(heading - MOUNT_OFFSET + 360.0f, 360.0f);

    // Exponential Moving Average
    static float heading_filtered = 0.0f;
    static bool initialised = false;

    if (!initialised) {
        heading_filtered = heading;
        initialised = true;
    } else {
        float delta_h = heading - heading_filtered;
        if (delta_h > 180.0f) {
            delta_h -= 360.0f;
        } else if (delta_h < -180.0f) {
            delta_h += 360.0f;
        }

        float alpha = (fabsf(delta_h) > 5.0f) ? MAG_LPF_FAST_ALPHA :
                      (fabsf(delta_h) > 2.0f) ? MAG_LPF_MED_ALPHA :
                                                MAG_LPF_SLOW_ALPHA;

        heading_filtered += (1.0f - alpha) * delta_h;
        heading_filtered = fmodf(heading_filtered + 360.0f, 360.0f);
    }

    *heading_deg = heading_filtered;
    last_valid_heading = *heading_deg;
}

// Pitch / Roll tilt from accelerometer
void imu_get_pitch_roll(float* pitch_deg, float* roll_deg) {
    static float last_pitch = 0.0f, last_roll = 0.0f;
    float ax, ay, az;

    if (!read_accel(&ax, &ay, &az)) {
        *pitch_deg = last_pitch;
        *roll_deg = last_roll;
        return;
    }

    // Low-pass filter to reduce noise
    ax_f = ACCEL_LPF_ALPHA_X * ax + (1 - ACCEL_LPF_ALPHA_X) * ax_f;
    ay_f = ACCEL_LPF_ALPHA_Y * ay + (1 - ACCEL_LPF_ALPHA_Y) * ay_f;
    az_f = ACCEL_LPF_ALPHA_Z * az + (1 - ACCEL_LPF_ALPHA_Z) * az_f;

    // Compute tilt angles in degrees
    // pitch: forward / back tilt
    // roll: left / right tilt
    float raw_pitch = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * (180.0f / M_PI);
    float raw_roll  = atan2f(ay_f, az_f) * (180.0f / M_PI);

    *pitch_deg = raw_pitch - PITCH_OFFSET;
    *roll_deg  = raw_roll - ROLL_OFFSET;

    last_pitch = *pitch_deg;
    last_roll = *roll_deg;
}

void calibrate_mag_offsets(void) {
    float mx, my;
    float minX = 9999, maxX = -9999;
    float minY = 9999, maxY = -9999;

    printf("\n=== Magnetometer Hard-Iron Calibration ===\n");
    printf("Rotate the robot slowly in all directions for about 10 seconds...\n");

    // collect ~500 samples (~10 s at 20 ms interval)
    for (int i = 0; i < 500; i++) {
        if (read_mag_xy(&mx, &my)) {
            if (mx < minX) minX = mx;
            if (mx > maxX) maxX = mx;
            if (my < minY) minY = my;
            if (my > maxY) maxY = my;
        }
        sleep_ms(20);
    }

    float offsetX = (maxX + minX) / 2.0f;
    float offsetY = (maxY + minY) / 2.0f;
    float scaleX  = (maxX - minX) / 2.0f;
    float scaleY  = (maxY - minY) / 2.0f;
    float avgScale = (scaleX + scaleY) / 2.0f;

    printf("\n=== Calibration Complete ===\n");
    printf("MAG_X_OFFSET (%.3ff)\n", offsetX);
    printf("MAG_Y_OFFSET (%.3ff)\n", offsetY);
    printf("// scale correction factors:\n");
    printf("MAG_SCALE_X (%.3ff)\n", scaleX / avgScale);
    printf("MAG_SCALE_Y (%.3ff)\n", scaleY / avgScale);
    printf("\n");
}

static float wrap360(float a){ 
    a=fmodf(a,360.0f); 
    return (a<0)?a+360.0f:a; 
}

void calibrate_mount_offset(void) {
    const int N = 100;           // ~2s at 50 Hz
    float sumC = 0.0f, sumS = 0.0f;

    for (int i=0; i<N; ++i) {
        float mx,my;
        if (!read_mag_xy(&mx,&my)) { sleep_ms(10); continue; }

        // raw heading (before any mount offset), same convention as runtime:
        float h = atan2f(my, mx) * 57.29578f; // rad->deg
        h = wrap360(h);

        // circular mean accumulate
        sumC += cosf(h * (float)M_PI/180.0f);
        sumS += sinf(h * (float)M_PI/180.0f);
        sleep_ms(20); // ~50 Hz
    }

    float mean = atan2f(sumS, sumC) * 57.29578f; // (-180,180]
    if (mean < 0) mean += 360.0f;

    // This is the value to subtract inside your heading code:
    printf("\n=== Mount Offset Calibration ===\n");
    printf("Set MOUNT_OFFSET to: %.2f\n", mean);
    printf("// MOUNT_OFFSET (%.2ff)\n\n", mean);
}

// IMU data unified getter function
void imu_get_all_data(imu_data_t* data) {
    float ax, ay, az, mx, my;

    // Accelerometer
    if (read_accel(&ax, &ay, &az)) {
        // Raw Pitch / Roll
        data->raw_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
        data->raw_roll  = atan2f(ay, az) * (180.0f / M_PI);

        // Filterd Pitch / Roll
        ax_f = ACCEL_LPF_ALPHA_X * ax + (1 - ACCEL_LPF_ALPHA_X) * ax_f;
        ay_f = ACCEL_LPF_ALPHA_Y * ay + (1 - ACCEL_LPF_ALPHA_Y) * ay_f;
        az_f = ACCEL_LPF_ALPHA_Z * az + (1 - ACCEL_LPF_ALPHA_Z) * az_f;

        data->filt_pitch = atan2f(-ax_f, sqrtf(ay_f * ay_f + az_f * az_f)) * (180.0f / M_PI) - PITCH_OFFSET;
        data->filt_roll  = atan2f(ay_f, az_f) * (180.0f / M_PI) - ROLL_OFFSET;
    }

    // Magnetometer
    if (read_mag_xy(&mx, &my)) {
        float mx_corr = (mx - MAG_X_OFFSET) * MAG_SCALE_X;
        float my_corr = (my - MAG_Y_OFFSET) * MAG_SCALE_Y;
        data->raw_heading = atan2f(my_corr, mx_corr) * (180.0f / M_PI);

        if (data->raw_heading < 0) {
            data->raw_heading += 360.0f;
        }

        // Mount offset
        float delta_h = data->raw_heading - data->filt_heading;
        if (delta_h > 180.0f) {
            delta_h -= 360.0f;
        } else if (delta_h < -180.0f) {
            delta_h += 360.0f;
        }

        float alpha = (fabsf(delta_h) > 5.0f) ? MAG_LPF_FAST_ALPHA :
                      (fabsf(delta_h) > 2.0f) ? MAG_LPF_MED_ALPHA :
                                                MAG_LPF_SLOW_ALPHA;

        data->filt_heading += (1.0 - alpha) * delta_h;
        data->filt_heading = fmodf(data->filt_heading + 360.0f, 360.0f);
    }
}

// End of file