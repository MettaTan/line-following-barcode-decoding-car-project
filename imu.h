#ifndef imu_h
#define imu_h

#include <pico/stdlib.h>

//Return codes
#define IMU_OK 0
#define IMU_ECOMM -1
#define IMU_EINVAL -2

// I2C ports and pins
#define I2C_PORT i2c0        // Inter-Integrated Circuit (comm protocol, allow multiple devices to talk to each other over 2 wires)
#define SDA_PIN 16            // Serial Data (actual data bits)
#define SCL_PIN 17            // Serial Clock (Timing signals to sync devices)

// I2C Addresses
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1E

// Accelerometer Registers
#define OUT_X_L_A 0x28
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define ACCEL_LSB_TO_MS2 (0.000061f * 9.80665f) // assuming ±2g range

// Magnetometer Registers
#define MAG_CRA_REG_M 0x00
#define MAG_CRB_REG_M 0x01
#define MAG_MR_REG_M 0x02
#define MAG_OUT_X_H_M 0x03
#define MAG_SR_REG_M 0x09
#define MAG_WHO_AM_I 0x0F

// Accelerometer calibration offsets
#define PITCH_OFFSET (3.2f)
#define ROLL_OFFSET (1.0f)

// Magnetometer calibration offsets
#define MAG_X_OFFSET (0.031f)
#define MAG_Y_OFFSET (-0.020f)
#define MOUNT_OFFSET (270.0f)

// #define MAG_SCALE_X (0.989f)
#define MAG_SCALE_X (0.935f)
#define MAG_SCALE_Y (1.065f)

// Filter configuration
#define ACCEL_LPF_ALPHA_X 0.20f
#define ACCEL_LPF_ALPHA_Y 0.10f
#define ACCEL_LPF_ALPHA_Z 0.20f
#define MAG_LPF_FAST_ALPHA 0.60f   // Large motion (>5-deg)
#define MAG_LPF_MED_ALPHA 0.75f    // Moderate motion (2 to 5-deg)
#define MAG_LPF_SLOW_ALPHA 0.90f   // Small motion (<2-deg)

// IMU Sampling Rate
#define IMU_SAMPLE_RATE_HZ 50.0f

// IMU Data Strcture
typedef struct {
    float raw_pitch;
    float raw_roll;
    float raw_heading;
    float filt_pitch;
    float filt_roll;
    float filt_heading;
} imu_data_t;

/**
 * @brief Initialize the accelerometer via I2C
 * 
 *  Configure I2C0, sets GPIO functions and enable all axes
 * 
 * !! NEED TO UPDATE !! 
 */
void imu_init(void);
bool read_accel(float *ax, float *ay, float *az);
void imu_get_pitch_roll(float* pitch_deg, float* roll_deg);
void imu_get_heading(float* heading_deg);
bool read_mag_xy(float* mx, float* my);
void calibrate_mag_offsets(void);
static float wrap360(float a);
void calibrate_mount_offset(void);
void imu_get_all_data(imu_data_t* data);

#endif

// End of file