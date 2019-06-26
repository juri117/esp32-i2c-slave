#include <stdio.h>
#include <cstring>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-slave";

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS \
  20 /*!< delay time between different test items */

#define I2C_SLAVE_SDA_IO GPIO_NUM_21
#define I2C_SLAVE_SCL_IO GPIO_NUM_22

#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN 256  //(2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN 256  //(2 * DATA_LENGTH)

#define ESP_SLAVE_ADDR 0x04
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define SLAVE_REQUEST_WAIT_MS 80

const uint8_t testCmd[10] = {0x00, 0x01, 0x02, 0x03, 0x04,
                             0x05, 0x06, 0x07, 0x08, 0x09};

uint8_t outBuff[256];
uint16_t outBuffLen = 0;

uint8_t inBuff[256];
uint16_t inBuffLen = 0;

extern "C" {
void app_main(void);
}

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init() {
  i2c_port_t i2c_slave_port = I2C_SLAVE_NUM;
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
  i2c_param_config(i2c_slave_port, &conf_slave);
  return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                            I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

bool check_for_data() {
  size_t size =
      i2c_slave_read_buffer(I2C_SLAVE_NUM, inBuff, 256, 10 / portTICK_RATE_MS);
  // ESP_LOGI(TAG, "len: %d", size);
  if (size == 1) {
    if (inBuff[0] == 0x01) {
      uint8_t replBuff[2];
      replBuff[0] = (uint8_t)(outBuffLen >> 0);
      replBuff[1] = (uint8_t)(outBuffLen >> 8);
      int ret = i2c_reset_tx_fifo(I2C_SLAVE_NUM);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to reset fifo");
      }
      i2c_slave_write_buffer(I2C_SLAVE_NUM, replBuff, 2,
                             1000 / portTICK_RATE_MS);
      ESP_LOGI(TAG, "got len request, put(%d):", outBuffLen);
      vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
      // ESP_LOG_BUFFER_HEX(TAG, replBuff, 2);
      return false;
    }
    if (inBuff[0] == 0x02) {
      int ret = i2c_reset_tx_fifo(I2C_SLAVE_NUM);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to reset fifo");
      }
      i2c_slave_write_buffer(I2C_SLAVE_NUM, outBuff, outBuffLen,
                             1000 / portTICK_RATE_MS);
      outBuffLen = 0;
      ESP_LOGI(TAG, "got write request");
      vTaskDelay(pdMS_TO_TICKS(SLAVE_REQUEST_WAIT_MS));
      return false;
    }
  }
  if (size > 1) {
    inBuffLen = size;
    return true;
  }
  return false;
}

static void i2cs_test_task(void *arg) {
  // uint32_t task_idx = (uint32_t)arg;
  while (1) {
    if (check_for_data()) {
      ESP_LOGI(TAG, "got data:");
      ESP_LOG_BUFFER_HEX(TAG, inBuff, inBuffLen);
      inBuffLen = 0;
    }
    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
  }
  // vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

static void send_task(void *arg) {
  while (1) {
    vTaskDelay((10000) / portTICK_RATE_MS);
    memcpy(outBuff, testCmd, 10);
    outBuffLen = 10;
    ESP_LOGI(TAG, "new data ready for send");
  }
  // vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

void app_main() {
  ESP_LOGI(TAG, "SLAVE--------------------------------");
  ESP_ERROR_CHECK(i2c_slave_init());

  xTaskCreate(i2cs_test_task, "slave", 1024 * 2, (void *)1, 10, NULL);
  xTaskCreate(send_task, "send_task", 1024 * 2, (void *)1, 10, NULL);
}
