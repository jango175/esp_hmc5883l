/**
 * @file esp_hmc5883l.h
 * @author JanG175
 * @brief ESP-IDF component for HMC5883L magnetometer
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// #define HMC5883L_I2C_INIT            1 // uncomment to initialize I2C driver

// compensation values (paste your calibration values here)
#define HMC5883L_X_OFFSET            -118.500000
#define HMC5883L_Y_OFFSET            163.500000
#define HMC5883L_Z_OFFSET            208.500000
#define HMC5883L_X_SCALE             0.998750
#define HMC5883L_Y_SCALE             0.957472
#define HMC5883L_Z_SCALE             1.047853

#define HMC5883L_MAG_DEC_DEG         (-6.0 + 39.0 / 60.0) // magnetic declination for your location (6*39'E)

// I2C configuration values
#define HMC5883L_MAX_FREQ            400000
#define HMC5883L_TIMEOUT_MS          100
#define HMC5883L_ADDR                0x1E

// register map
#define HMC_CONFIG_A                 0 // Configuration Register A Read/Write
#define HMC_CONFIG_B                 1 // Configuration Register B Read/Write
#define HMC_MODE                     2 // Mode Register Read/Write
#define HMC_DATA_OUT_X_MSB           3 // Data Output X MSB Register Read
#define HMC_DATA_OUT_X_LSB           4 // Data Output X LSB Register Read
#define HMC_DATA_OUT_Z_MSB           5 // Data Output Z MSB Register Read
#define HMC_DATA_OUT_Z_LSB           6 // Data Output Z LSB Register Read
#define HMC_DATA_OUT_Y_MSB           7 // Data Output Y MSB Register Read
#define HMC_DATA_OUT_Y_LSB           8 // Data Output Y LSB Register Read
#define HMC_STATUS                   9 // Status Register Read
#define HMC_ID_A                     10 // Identification Register A Read
#define HMC_ID_B                     11 // Identification Register B Read
#define HMC_ID_C                     12 // Identification Register C Read

// constants
#define HMC_ID_A_VALUE               0b01001000
#define HMC_ID_B_VALUE               0b00110100
#define HMC_ID_C_VALUE               0b00110011

enum hmc5883l_over_sample
{
    HMC5883L_OVER_SAMPLE_1 = 0,
    HMC5883L_OVER_SAMPLE_2,
    HMC5883L_OVER_SAMPLE_4,
    HMC5883L_OVER_SAMPLE_8 // default
};

enum hmc5883l_data_output_rate
{
    HMC5883L_DATA_OUTPUT_RATE_0_75_HZ = 0,
    HMC5883L_DATA_OUTPUT_RATE_1_5_HZ,
    HMC5883L_DATA_OUTPUT_RATE_3_HZ,
    HMC5883L_DATA_OUTPUT_RATE_7_5_HZ,
    HMC5883L_DATA_OUTPUT_RATE_15_HZ, // default
    HMC5883L_DATA_OUTPUT_RATE_30_HZ,
    HMC5883L_DATA_OUTPUT_RATE_75_HZ
};

enum hmc5883l_measure_mode
{
    HMC5883L_MODE_NORMAL = 0, // default
    HMC5883L_MODE_POS_BIAS,
    HMC5883L_MODE_NEG_BIAS,
};

enum hmc5883l_gain
{
    HMC5883L_GAIN_1370 = 0,
    HMC5883L_GAIN_1090, // default
    HMC5883L_GAIN_820,
    HMC5883L_GAIN_660,
    HMC5883L_GAIN_440,
    HMC5883L_GAIN_390,
    HMC5883L_GAIN_330,
    HMC5883L_GAIN_230
};

enum hmc5883l_operating_mode
{
    HMC5883L_CONTINUOUS_MODE = 0,
    HMC5883L_SINGLE_MODE, // default
    HMC5883L_IDLE_MODE_1,
    HMC5883L_IDLE_MODE_2
};

// HMC5883L configuration struct
typedef struct hmc5883l_conf_t
{
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
    gpio_num_t drdy_pin;
} hmc5883l_conf_t;


void hmc5883l_init(hmc5883l_conf_t hmc);

void hmc5883l_write_config(hmc5883l_conf_t hmc, enum hmc5883l_over_sample over_sample, 
                            enum hmc5883l_data_output_rate output_rate,
                            enum hmc5883l_measure_mode mode, enum hmc5883l_gain gain);

void hmc5883l_read_config(hmc5883l_conf_t hmc, uint8_t* configA, uint8_t* configB);

void hmc5883l_write_mode(hmc5883l_conf_t hmc, enum hmc5883l_operating_mode mode);

void hmc5883l_read_mode(hmc5883l_conf_t hmc, uint8_t* mode);

void hmc5883l_read_raw_magnetometer(hmc5883l_conf_t hmc, int16_t* x, int16_t* y, int16_t* z);

void hmc5883l_read_status(hmc5883l_conf_t hmc, uint8_t* status);

void hmc5883l_read_id(hmc5883l_conf_t hmc, uint8_t* id);

void hmc5883l_calibrate(hmc5883l_conf_t hmc);

int32_t hmc5883l_get_azimuth(hmc5883l_conf_t hmc);

void hmc5883l_read_magnetometer(hmc5883l_conf_t hmc, float* x, float* y, float* z);
