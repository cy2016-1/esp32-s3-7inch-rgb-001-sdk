/**
 * @file bsp_i2c.c
 * @brief I2C config on dev board.
 * @version 0.1
 * @date 2021-03-07
 * 
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include "bsp_i2c.h"
#define GPIO_I2C_SCL (GPIO_NUM_15)
#define GPIO_I2C_SDA (GPIO_NUM_16)
static const char *TAG= "bsp_i2c";

static i2c_bus_handle_t i2c_bus_handle = NULL;

static i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .scl_io_num = GPIO_I2C_SCL,
    .sda_io_num = GPIO_I2C_SDA,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
};

esp_err_t bsp_i2c_init(i2c_port_t i2c_num, uint32_t clk_speed)
{
    /* Check if bus is already created */
    if (NULL != i2c_bus_handle) {
        ESP_LOGE(TAG, "I2C bus already initialized.");
        return ESP_FAIL;
    }

    conf.master.clk_speed = clk_speed;

    i2c_bus_handle = i2c_bus_create(i2c_num, &conf);

    if (NULL == i2c_bus_handle) {
        ESP_LOGE(TAG, "Failed create I2C bus");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bsp_i2c_add_device(i2c_bus_device_handle_t *i2c_device_handle, uint8_t dev_addr)
{
    if (NULL == i2c_bus_handle) {
        ESP_LOGE(TAG, "Failed create I2C device");
        return ESP_FAIL;
    }
    
    *i2c_device_handle = i2c_bus_device_create(i2c_bus_handle, dev_addr, 400000);

    if (NULL == i2c_device_handle) {
        ESP_LOGE(TAG, "Failed create I2C device");
        return ESP_FAIL;
    }

    return ESP_OK;
}


#define WRITE_BIT 0x00 /*!< I2C master write */
#define READ_BIT 0x01  /*!< I2C master read  */

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave     */
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value  */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define AT24C02_DeviceAddr 0xA0 


static esp_err_t at24c02_read(uint8_t *data_rd, uint16_t ReadAddr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AT24C02_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ReadAddr % 256, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0xA0 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t at24c02_write(uint8_t data_wr, uint16_t WriteAddr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AT24C02_DeviceAddr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, WriteAddr % 256, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_wr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void sys_int(void)
{
    uint8_t data_buf = 0x00;
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));
   
    if (at24c02_write(0x55, 0x03) == ESP_OK)
        ESP_LOGI(TAG, "Write 0x55 to at24c02 in 0x03 \n");
    else
        ESP_LOGI(TAG, "Write at24c02 err \n");
    vTaskDelay(10 / portTICK_PERIOD_MS); 
     
    if (at24c02_write(0x56, 0x05) == ESP_OK)
        ESP_LOGI(TAG, "Write 0x55 to at24c02 in 0x03 \n");
    else
        ESP_LOGI(TAG, "Write at24c02 err \n");
         vTaskDelay(10 / portTICK_PERIOD_MS); 
   
    if (at24c02_read(&data_buf, 0x03) == ESP_OK)
    {
        if (data_buf == 0x55)
        {
            ESP_LOGI(TAG, "sys_int\n");
        }
        else
        {
            ESP_LOGE(TAG, "illegal board,Please contact us for authorization!\n");
            ESP_LOGE(TAG, "Email: ss559550@aliyun.com\n");
            esp_restart();
        }
    }
    else
    {
        ESP_LOGE(TAG, "illegal board,Please contact us for authorization!\n");
        ESP_LOGE(TAG, "Email: ss559550@aliyun.com\n");
        esp_restart();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
    if (at24c02_read(&data_buf, 0x05) == ESP_OK)
    {
        if (data_buf == 0x56)
        {
            ESP_LOGI(TAG, "sys_int\n");
        }
        else
        {
            ESP_LOGE(TAG, "illegal board,Please contact us for authorization!\n");
            ESP_LOGE(TAG, "Email: ss559550@aliyun.com\n");
            esp_restart();
        }
    }
    else
    {
        ESP_LOGE(TAG, "illegal board,Please contact us for authorization!\n");
        ESP_LOGE(TAG, "Email: ss559550@aliyun.com\n");
        esp_restart();
    }
}
