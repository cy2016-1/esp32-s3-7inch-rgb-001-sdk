#include "lv_port_disp.h"
#include "lv_port_fs.h"
#include "lv_port_indev.h"
static const char *TAG = "bsp_i2c";
#define WRITE_BIT 0x00 /*!< I2C master write */
#define READ_BIT 0x01  /*!< I2C master read  */

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave     */
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value  */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define AT24C02_DeviceAddr 0xA0 /* AT24C02的器件地址 */

/* AT24C02读取一个字节函数，第一个参数为要读出值的存放指针，第二个参数为要读出的地址*/
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

void lvgl_init()
{
    uint8_t data_buf = 0x00;

    if (at24c02_read(&data_buf, 0x05) == ESP_OK)
    {
        if (data_buf == 0x56)
        {
            lv_init();
            lv_port_disp_init();
            lv_port_indev_init();
            lv_port_tick_init();
            ESP_LOGI(TAG, "lv_init\n");
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