/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512    /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS \
  10000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO GPIO_NUM_22
#define I2C_SLAVE_SDA_IO GPIO_NUM_21
#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)

#define I2C_MASTER_SCL_IO GPIO_NUM_19
#define I2C_MASTER_SDA_IO GPIO_NUM_18
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR 0x28
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

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

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len) {
  int i;
  for (i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
}

static void i2cs_test_task(void *arg) {
  uint32_t task_idx = (uint32_t)arg;
  uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
  int i = 0;
  while (1) {
    for (i = 0; i < DATA_LENGTH; i++) {
      data[i] = i;
    }
    // i2c_reset_tx_fifo(I2C_SLAVE_NUM);
    size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH,
                                           1000 / portTICK_RATE_MS);
    printf("====TASK[%d] Slave write: [%d] bytes ====\n", task_idx, d_size);
    disp_buf(data, d_size);

    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);

    for (i = 0; i < DATA_LENGTH; i++) {
      data[i] = 0x00;
    }
    size_t size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH,
                                        1000 / portTICK_RATE_MS);

    printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
    disp_buf(data, size);

    vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS) / portTICK_RATE_MS);
  }
  // vSemaphoreDelete(print_mux);
  vTaskDelete(NULL);
}

void app_main() {
  ESP_LOGI(TAG, "SLAVE--------------------------------");
  ESP_ERROR_CHECK(i2c_slave_init());

  xTaskCreate(i2cs_test_task, "slave", 1024 * 2, (void *)1, 10, NULL);
}
