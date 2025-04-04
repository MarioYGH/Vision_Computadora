#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "encoder_motors.h"

const char *TAG = "DC_Motors";

void app_main(void){
    int speed = PWM_MOTOR_MAX_VALUE;

    ESP_LOGI(TAG, "Inicializing motors");
    ESP_ERROR_CHECK(init_motor_m1());

    ESP_LOGI(TAG, "init pent driver to encoder");
    ESP_ERROR_CHECK(init_encoder_m1());

    char sstr[20];

    while(true){
        ESP_LOGI(TAG, "Count: %d", get_count_m1());

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}
