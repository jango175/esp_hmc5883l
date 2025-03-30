/**
 * @file esp_hmc5883l.c
 * @author JanG175
 * @brief ESP-IDF component for HMC5883L magnetometer
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_hmc5883l.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool measurement_ready = false;

#ifdef HMC5883L_I2C_INIT
static i2c_master_bus_handle_t bus_handle;
#else
extern i2c_master_bus_handle_t bus_handle;
#endif
static i2c_master_dev_handle_t dev_handle;

static const char* TAG = "HMC5883L";


/**
 * @brief interrupt handler for magnetometer readings
 * 
 * @param arg pointer to HMC5883L parameters
*/
static void isr_handler(void* arg)
{
    portENTER_CRITICAL(&mux);
    measurement_ready = true;
    portEXIT_CRITICAL(&mux);
}


/**
 * @brief initialize HMC5883L
 * 
 * @param hmc struct with HMC5883L parameters
*/
void hmc5883l_init(hmc5883l_conf_t hmc)
{
#ifdef HMC5883L_I2C_INIT
    if (hmc.i2c_freq > HMC5883L_MAX_FREQ)
    {
        hmc.i2c_freq = HMC5883L_MAX_FREQ;
        ESP_LOGW(TAG, "I2C frequency too high, set to value: %d Hz", HMC5883L_MAX_FREQ);
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = hmc.i2c_port,
        .scl_io_num = hmc.scl_pin,
        .sda_io_num = hmc.sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
#endif

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HMC5883L_ADDR,
        .scl_speed_hz = hmc.i2c_freq
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // interrupt mode
    if (hmc.drdy_pin != -1)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << hmc.drdy_pin;
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        ESP_ERROR_CHECK(gpio_isr_handler_add(hmc.drdy_pin, isr_handler, (void*)&hmc));
    }
}


/**
 * @brief configure HMC5883L
 * 
 * @param hmc struct with HMC5883L parameters
 * @param over_sample over sample ratio
 * @param output_rate data output rate
 * @param mode measure mode
 * @param gain gain
*/
void hmc5883l_write_config(hmc5883l_conf_t hmc, enum hmc5883l_over_sample over_sample, 
                            enum hmc5883l_data_output_rate output_rate,
                            enum hmc5883l_measure_mode mode, enum hmc5883l_gain gain)
{
    // set config A register
    uint8_t config = (uint8_t)over_sample << 5 | (uint8_t)output_rate << 2 | (uint8_t)mode;
    uint8_t data[2] = {HMC_CONFIG_A, config};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, HMC5883L_TIMEOUT_MS));

    vTaskDelay(1);

    // set config B register
    config = (uint8_t)gain << 5;

    data[0] = HMC_CONFIG_B;
    data[1] = config;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, HMC5883L_TIMEOUT_MS));
}


/**
 * @brief read HMC5883L configuration
 * 
 * @param hmc struct with HMC5883L parameters
 * @param configA pointer to config A data
 * @param configB pointer to config B data
*/
void hmc5883l_read_config(hmc5883l_conf_t hmc, uint8_t* configA, uint8_t* configB)
{
    uint8_t reg = HMC_CONFIG_A;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, configA, 1, HMC5883L_TIMEOUT_MS));
    *configA = *configA & ~(1 << 7);

    vTaskDelay(1);

    reg = HMC_CONFIG_B;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, configB, 1, HMC5883L_TIMEOUT_MS));
    *configB = *configB & ~0b00011111;

    uint8_t over_sample = *configA >> 5;
    uint8_t output_rate = (*configA & 0b00011100) >> 2;
    uint8_t mode = *configA & 0b00000011;
    uint8_t gain = *configB >> 5;

    switch (over_sample)
    {
        case HMC5883L_OVER_SAMPLE_1:
            ESP_LOGI(TAG, "Over sample ratio: 1");
            break;
        case HMC5883L_OVER_SAMPLE_2:
            ESP_LOGI(TAG, "Over sample ratio: 2");
            break;
        case HMC5883L_OVER_SAMPLE_4:
            ESP_LOGI(TAG, "Over sample ratio: 4");
            break;
        case HMC5883L_OVER_SAMPLE_8:
            ESP_LOGI(TAG, "Over sample ratio: 8");
            break;
        default:
            ESP_LOGE(TAG, "Unknown over sample ratio: %u", over_sample);
            break;
    }

    switch (output_rate)
    {
        case HMC5883L_DATA_OUTPUT_RATE_0_75_HZ:
            ESP_LOGI(TAG, "Data output rate: 0.75 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_1_5_HZ:
            ESP_LOGI(TAG, "Data output rate: 1.5 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_3_HZ:
            ESP_LOGI(TAG, "Data output rate: 3 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_7_5_HZ:
            ESP_LOGI(TAG, "Data output rate: 7.5 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_15_HZ:
            ESP_LOGI(TAG, "Data output rate: 15 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_30_HZ:
            ESP_LOGI(TAG, "Data output rate: 30 Hz");
            break;
        case HMC5883L_DATA_OUTPUT_RATE_75_HZ:
            ESP_LOGI(TAG, "Data output rate: 75 Hz");
            break;
        default:
            ESP_LOGE(TAG, "Unknown data output rate: %u", output_rate);
            break;
    }

    switch (mode)
    {
        case HMC5883L_MODE_NORMAL:
            ESP_LOGI(TAG, "Measure mode: Normal");
            break;
        case HMC5883L_MODE_POS_BIAS:
            ESP_LOGI(TAG, "Measure mode: Positive bias");
            break;
        case HMC5883L_MODE_NEG_BIAS:
            ESP_LOGI(TAG, "Measure mode: Negative bias");
            break;
        default:
            ESP_LOGE(TAG, "Unknown measure mode: %u", mode);
            break;
    }

    switch (gain)
    {
        case HMC5883L_GAIN_1370:
            ESP_LOGI(TAG, "Gain: 1370");
            break;
        case HMC5883L_GAIN_1090:
            ESP_LOGI(TAG, "Gain: 1090");
            break;
        case HMC5883L_GAIN_820:
            ESP_LOGI(TAG, "Gain: 820");
            break;
        case HMC5883L_GAIN_660:
            ESP_LOGI(TAG, "Gain: 660");
            break;
        case HMC5883L_GAIN_440:
            ESP_LOGI(TAG, "Gain: 440");
            break;
        case HMC5883L_GAIN_390:
            ESP_LOGI(TAG, "Gain: 390");
            break;
        case HMC5883L_GAIN_330:
            ESP_LOGI(TAG, "Gain: 330");
            break;
        case HMC5883L_GAIN_230:
            ESP_LOGI(TAG, "Gain: 230");
            break;
        default:
            ESP_LOGE(TAG, "Unknown gain: %u", gain);
            break;
    }
}


/**
 * @brief write HMC5883L operating mode register
 * 
 * @param hmc struct with HMC5883L parameters
 * @param mode operating mode
*/
void hmc5883l_write_mode(hmc5883l_conf_t hmc, enum hmc5883l_operating_mode mode)
{
    uint8_t data[2] = {HMC_MODE, mode};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, 2, HMC5883L_TIMEOUT_MS));
}


/**
 * @brief read HMC5883L operating mode register
 *
 * @param hmc struct with HMC5883L parameters
 * @param mode pointer to operating mode data
*/
void hmc5883l_read_mode(hmc5883l_conf_t hmc, uint8_t* mode)
{
    uint8_t reg = HMC_MODE;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, mode, 1, HMC5883L_TIMEOUT_MS));

    *mode = *mode & ~0b11111100;

    switch (*mode)
    {
        case HMC5883L_CONTINUOUS_MODE:
            ESP_LOGI(TAG, "Operating mode: Continuous");
            break;
        case HMC5883L_SINGLE_MODE:
            ESP_LOGI(TAG, "Operating mode: Single");
            break;
        case HMC5883L_IDLE_MODE_1:
            ESP_LOGI(TAG, "Operating mode: Idle");
            break;
        case HMC5883L_IDLE_MODE_2:
            ESP_LOGI(TAG, "Operating mode: Idle");
            break;
        default:
            ESP_LOGE(TAG, "Unknown operating mode: %u", *mode);
            break;
    }
}


/**
 * @brief read magnetometer raw data
 * 
 * @param hmc struct with HMC5883L parameters
 * @param x pointer to x axis data
 * @param y pointer to y axis data
 * @param z pointer to z axis data
*/
void hmc5883l_read_raw_magnetometer(hmc5883l_conf_t hmc, int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t data[6];
    uint8_t reg = HMC_DATA_OUT_X_MSB;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, 6, HMC5883L_TIMEOUT_MS));

    *x = (int16_t)data[0] << 8 | (int16_t)data[1];
    *z = (int16_t)data[2] << 8 | (int16_t)data[3];
    *y = (int16_t)data[4] << 8 | (int16_t)data[5];

    if (hmc.drdy_pin != -1)
    {
        portENTER_CRITICAL(&mux);
        measurement_ready = false;
        portEXIT_CRITICAL(&mux);
    }
}


/**
 * @brief read HMC5883L status
 * 
 * @param hmc struct with HMC5883L parameters
 * @param status pointer to status data
*/
void hmc5883l_read_status(hmc5883l_conf_t hmc, uint8_t* status)
{
    uint8_t reg = HMC_STATUS;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, status, 1, HMC5883L_TIMEOUT_MS));

    *status = *status & ~0b11111100;

    uint8_t lock = *status >> 1;
    uint8_t rdy = *status & 1;

    if (lock == 1)
        ESP_LOGI(TAG, "Data output register locked");
    else
        ESP_LOGI(TAG, "Data output register not locked");

    if (rdy == 1)
        ESP_LOGI(TAG, "New data available");
    else
        ESP_LOGI(TAG, "No new data available");
}


/**
 * @brief read HMC5883L identification registers
 * 
 * @param hmc struct with HMC5883L parameters
 * @param id pointer to id data array
*/
void hmc5883l_read_id(hmc5883l_conf_t hmc, uint8_t* id)
{
    uint8_t reg = HMC_ID_A;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, id, 3, HMC5883L_TIMEOUT_MS));

    if (id[0] != HMC_ID_A_VALUE)
        ESP_LOGE(TAG, "HMC5883L id A not found");
    else
        ESP_LOGI(TAG, "HMC5883L id A found");

    if (id[1] != HMC_ID_B_VALUE)
        ESP_LOGE(TAG, "HMC5883L id B not found");
    else
        ESP_LOGI(TAG, "HMC5883L id B found");
    
    if (id[2] != HMC_ID_C_VALUE)
        ESP_LOGE(TAG, "HMC5883L id C not found");
    else
        ESP_LOGI(TAG, "HMC5883L id C found");
}


/**
 * @brief calibrate HMC5883L (10 seconds of stable measurements)
 * 
 * @param hmc struct with HMC5883L parameters
*/
void hmc5883l_calibrate(hmc5883l_conf_t hmc)
{
    int16_t mag_cal[3][2]; // 0 - min, 1 - max
    for (uint32_t i = 0; i < 3; i++)
    {
        mag_cal[i][0] = 32767;
        mag_cal[i][1] = -32768;
    }

    int16_t mag_val[3] = {0, 0, 0};
    bool is_changed[3] = {false, false, false};
    uint32_t count = 0;

    ESP_LOGW(TAG, "Calibration started - move the sensor in all directions...");

    while (count < 1000) // while measurements are not stable for 10 seconds
    {
        hmc5883l_read_raw_magnetometer(hmc, mag_val, mag_val + 1, mag_val + 2);

        for (uint8_t i = 0; i < 3; i++)
        {
            if (mag_val[i] < mag_cal[i][0])
            {
                mag_cal[i][0] = mag_val[i];
                is_changed[i] = true;
            }
            else
                is_changed[i] = false;

            if (mag_val[i] > mag_cal[i][1])
            {
                mag_cal[i][1] = mag_val[i];
                is_changed[i] = true;
            }
            else
                is_changed[i] = false;
        }

        if (is_changed[0] == false && is_changed[1] == false && is_changed[2] == false)
            count++;
        else
            count = 0;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // calculate compensation values
    float avg_delta[3];
    float cal_offset[3];
    float cal_scale[3];

    for (uint32_t i = 0; i < 3; i++)
        avg_delta[i] = (float)(mag_cal[i][1] - mag_cal[i][0]) / 2.0f;

    float total_avg_delta = (avg_delta[0] + avg_delta[1] + avg_delta[2]) / 3.0f;

    for (uint32_t i = 0; i < 3; i++)
    {
        cal_offset[i] = (mag_cal[i][0] + mag_cal[i][1]) / 2.0f;
        cal_scale[i] = total_avg_delta / avg_delta[i];
    }

    ESP_LOGW(TAG, "Calibration ended");

    ESP_LOGI(TAG, "Paste these values to esp_hmc5883l.h compensation values section:");
    printf("#define HMC5883L_X_OFFSET            %ff\n", cal_offset[0]);
    printf("#define HMC5883L_Y_OFFSET            %ff\n", cal_offset[1]);
    printf("#define HMC5883L_Z_OFFSET            %ff\n", cal_offset[2]);

    printf("#define HMC5883L_X_SCALE             %ff\n", cal_scale[0]);
    printf("#define HMC5883L_Y_SCALE             %ff\n", cal_scale[1]);
    printf("#define HMC5883L_Z_SCALE             %ff\n", cal_scale[2]);

    vTaskDelay(portMAX_DELAY);
}


/**
 * @brief get azimuth (XY plane) from magnetometer readings
 *
 * @param hmc struct with HMC5883L parameters
 *
 * @return azimuth
*/
int32_t hmc5883l_get_azimuth(hmc5883l_conf_t hmc)
{
    float x, y, z;
    hmc5883l_read_magnetometer(hmc, &x, &y, &z);

    float heading = atan2(y, x) * 180.0 / M_PI;
    heading += HMC5883L_MAG_DEC_DEG;

    return (int32_t)heading % 360;
}


/**
 * @brief read magnetometer compensated data
 * 
 * @param hmc struct with HMC5883L parameters
 * @param x pointer to x axis data
 * @param y pointer to y axis data
 * @param z pointer to z axis data
*/
void hmc5883l_read_magnetometer(hmc5883l_conf_t hmc, float* x, float* y, float* z)
{
    int16_t x_raw, y_raw, z_raw;
    hmc5883l_read_raw_magnetometer(hmc, &x_raw, &y_raw, &z_raw);

    *x = ((float)x_raw - HMC5883L_X_OFFSET) * HMC5883L_X_SCALE;
    *y = ((float)y_raw - HMC5883L_Y_OFFSET) * HMC5883L_Y_SCALE;
    *z = ((float)z_raw - HMC5883L_Z_OFFSET) * HMC5883L_Z_SCALE;

    if (hmc.drdy_pin != -1)
    {
        portENTER_CRITICAL(&mux);
        measurement_ready = false;
        portEXIT_CRITICAL(&mux);
    }
}