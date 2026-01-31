# DFRobot LTR308 ESP-IDF Component

This is a port of the [DFRobot LTR308 Arduino Library](https://github.com/DFRobot/DFRobot_LTR308) to ESP-IDF.

## Usage

1. Copy the `DFRobot_LTR308` folder to your ESP-IDF project's `components` directory.
2. Add the component to your project configuration (automatically handled by IDF).
3. Initialize the I2C driver in your `main` code before using the sensor.

### Example

```cpp
#include "DFRobot_LTR308.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    // 1. Initialize I2C Master Bus
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = -1, // Auto-select port
        .sda_io_num = CONFIG_I2C_MASTER_SDA, // Define or usage specific GPIO
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // 2. Add Device to Bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LTR308_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // 3. Create Sensor Instance
    DFRobot_LTR308 sensor(dev_handle);

    if (sensor.begin()) {
        ESP_LOGI("LTR308", "Sensor begin success");
    } else {
        ESP_LOGE("LTR308", "Sensor begin failed");
    }

    while (1) {
        uint32_t val = sensor.getData();
        double lux = sensor.getLux(val);
        ESP_LOGI("LTR308", "Light: %.2f Lux", lux);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Control IR LED based on Light Level (Polling Mode)

This example demonstrates how to control an IR LED by polling the sensor data.

```cpp
// ... (existing polling code) ...
```

### Control IR LED using Interrupts (Efficient Mode)

The LTR308 has an interrupt pin (INT) that can go active (low) when light levels exceed a configured range. This is more efficient than polling.

**Hardware Setup:**
- Connect LTR308 `INT` pin to ESP32 `GPIO 4`.
- Connect IR LED to ESP32 `GPIO 47`.

```cpp
#include "DFRobot_LTR308.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IR_LED_PIN      GPIO_NUM_47
#define SENSOR_INT_PIN  GPIO_NUM_4
#define LOW_THRESHOLD   10   // Lux
#define HIGH_THRESHOLD  500  // Lux

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void app_main(void) {
    // 1. Initialize I2C Master Bus
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = -1,
        .sda_io_num = CONFIG_I2C_MASTER_SDA,
        .scl_io_num = CONFIG_I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // 2. Add Device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LTR308_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // 3. Initialize IR LED GPIO
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << IR_LED_PIN);
    gpio_config(&io_conf);
    
    // 4. Initialize Interrupt GPIO
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Trigger on falling edge (Active Low)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SENSOR_INT_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Install ISR service
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SENSOR_INT_PIN, gpio_isr_handler, (void*) SENSOR_INT_PIN);

    // 5. Initialize Sensor
    DFRobot_LTR308 sensor(dev_handle);
    while (!sensor.begin()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // 6. Configure Sensor Interrupts
    // Set thresholds (raw values, approx lux conversion needed based on gain)
    // For Gain=3X, Res=18bit(100ms), 1 Lux ~= 1 count (roughly, verify with getLux)
    sensor.setThreshold(HIGH_THRESHOLD / 0.6, LOW_THRESHOLD / 0.6); 
    sensor.setIntrPersist(DFRobot_LTR308::eInterruptTrigger_1); // Trigger after 1 occurrence
    sensor.setInterruptControl(true); // Enable Interrupt Mode

    ESP_LOGI("LTR308", "Interrupt Mode Enabled. Waiting for trigger...");

    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(1000))) {
            ESP_LOGI("LTR308", "Interrupt Triggered!");
            
            // Read status to clear interrupt
            DFRobot_LTR308::sMainStatus_t status = sensor.getStatus();
            if (status.intrStatus) {
                 uint32_t val = sensor.getData();
                 double lux = sensor.getLux(val);
                 ESP_LOGI("LTR308", "New Lux: %.2f", lux);
                 
                 if (lux < LOW_THRESHOLD) {
                     gpio_set_level(IR_LED_PIN, 1);
                 } else {
                     gpio_set_level(IR_LED_PIN, 0);
                 }
            }
        }
    }
}
```

## Dependencies

- `driver` (I2C) - specifically `driver/i2c_master.h` (ESP-IDF v5.x)
- `esp_log`
- `freertos`

## Notes

- This component uses the **new** `driver/i2c_master.h` API for ESP-IDF v5 compatibility.
- Blocking delays (`vTaskDelay`) are used instead of `delay()`.
