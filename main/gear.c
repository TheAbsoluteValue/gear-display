#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/can.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Assign pin numbers
#define A_GPIO 23
#define B_GPIO 22
#define C_GPIO 21
#define D_GPIO 19
#define E_GPIO 18
#define F_GPIO 17
#define G_GPIO 16
#define GPIO_SEL (1ULL << A_GPIO | 1ULL << B_GPIO | 1ULL << C_GPIO | 1ULL << D_GPIO | 1ULL << E_GPIO | 1ULL << F_GPIO | 1ULL << G_GPIO)
#define TX_GPIO_NUM 15
#define RX_GPIO_NUM 4

static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();

static const can_general_config_t g_config = {
  .mode = CAN_MODE_NORMAL,
  .tx_io = TX_GPIO_NUM,
  .rx_io = RX_GPIO_NUM,
  .clkout_io = CAN_IO_UNUSED,
  .bus_off_io = CAN_IO_UNUSED,
  .tx_queue_len = 0,
  .rx_queue_len = 5,
  .alerts_enabled = CAN_ALERT_NONE,
  .clkout_divider = 0
};

static SemaphoreHandle_t rx_sem;

enum Gear {            // The enum which describes the state to display on the seven-segment display.
  Park = 0b1100111,    // P
  One = 0b0110000,     // 1
  Two = 0b1101101,     // 2
  Three = 0b1111001,   // 3
  Four = 0b0110011,    // 4
  Five = 0b1011011,    // 5
  Reverse = 0b1000110, // r
  Change = 0b1001110,  // C
  Neutral = 0b1110110, // n
};

void setLevel(enum Gear gear) { // Sets the gpio levels to display on the seven-segment correctly.
  uint8_t num = (uint8_t)gear;
    gpio_set_level(G_GPIO, num & 1);
    gpio_set_level(F_GPIO, (num >> 1) & 1);
    gpio_set_level(E_GPIO, (num >> 2) & 1);
    gpio_set_level(D_GPIO, (num >> 3) & 1);
    gpio_set_level(C_GPIO, (num >> 4) & 1);
    gpio_set_level(B_GPIO, (num >> 5) & 1);
    gpio_set_level(A_GPIO, num >> 6);
}

void setSEG(uint8_t num) {
  switch (num) {
  case 0:
    setLevel(Park);
    break;
  case 1:
    setLevel(One);
    break;
  case 2:
    setLevel(Two);
    break;
  case 3:
    setLevel(Three);
    break;
  case 4:
    setLevel(Four);
    break;
  case 5:
    setLevel(Five);
    break;
  case 6:
    setLevel(Reverse);
    break;
  case 7:
    setLevel(Change);
    break;
  case 8:
    setLevel(Neutral);
  }
}

static uint8_t g_gear = 0; // This value should be atomic since it's being accessed by multiple processes.
                           // Making it atomic is not necessary in this case since there's only one processes writing to it.

static void display_task(void* arg) {
  for (;;) {
    setSEG(g_gear);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void read_can_task(void* arg) {
  xSemaphoreTake(rx_sem, portMAX_DELAY);
  can_message_t rx_msg;
  for (;;) {
    can_receive(&rx_msg, portMAX_DELAY);
    if (rx_msg.identifier == 0x188) // Arb ID for transmission is 0x188
      g_gear = (rx_msg.data[0] == 0 && rx_msg.data[3] == 4 ? (uint8_t) 8 : rx_msg.data[0]);
      
    /* The 0th byte contains the transmission data. However so does the 1st.
     * The difference is the 0th byte contains an extra state where
     * it shows it's 'changing gear.' I choose the 0th byte. */
    taskYIELD(); // Allow display_task a chance to run by yielding self.
  }
}

void app_main(void) {
  rx_sem = xSemaphoreCreateBinary();

  // Configure GPIO pins
  gpio_config_t config = {};
  config.pin_bit_mask = GPIO_SEL;
  config.mode = GPIO_MODE_OUTPUT;
  gpio_config(&config);

  // Create tasks
  xTaskCreate(display_task, "display_task", 4096, NULL, 6, NULL);
  xTaskCreate(read_can_task, "read_can_task", 4096, NULL, 5, NULL);

  // Install and start CAN peripheral
  ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
  ESP_ERROR_CHECK(can_start());
  vTaskDelay(100 / portTICK_PERIOD_MS);
  xSemaphoreGive(rx_sem);
}
