#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "encoder_motors.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

const char *TAG = "Open-Loop-Ctrl";
static QueueHandle_t uart_queue;
float w = 0.0;
float set_point = 0.0;

esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static void rx_task(void *arg){
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uart_event_t rx_event;
    char sstr[20];

    while (true) {
        if(xQueueReceive(uart_queue, &rx_event, portMAX_DELAY)){

            /* Clear space memory */
            bzero(data, BUF_SIZE);

            switch(rx_event.type){
                case UART_DATA:
                    /* Read data from the UART */
                    uart_read_bytes(UART_PORT, data, rx_event.size, pdMS_TO_TICKS(100));
                    
                    set_point = strtof((char *)data, NULL);
                    
                    /* Write data back to the UART */
                    //set_speed_m1(percent2w(set_point));
                    set_speed_m1(set_point);
                    
                    /* Clear the buffer */
                    uart_flush(UART_PORT);
                    break;

                default:
            }


        }
    }
}

void app_main(){
    // Declare a variable to store the last time
    uint64_t lastTime = 0;
    uint32_t sampleTime = SAMPLE_TIME; // Example sample time in milliseconds

    ESP_LOGI(TAG, "Initializing motors");
    ESP_ERROR_CHECK(init_motor_m1());
    ESP_ERROR_CHECK(init_motor_m2());

    ESP_LOGI(TAG, "Initializing pcnt driver to decode");
    ESP_ERROR_CHECK(init_encoder_m1());
    ESP_ERROR_CHECK(init_encoder_m2());

    set_speed_all(0, 0);

    ESP_LOGI(TAG, "Initializing uart port");
    ESP_ERROR_CHECK(uart_initialize());

    xTaskCreate(rx_task, "uart_rx_task", TASK_MEMORY, NULL, configMAX_PRIORITIES-1, NULL);

    //char sstr[20];
    int speed = 0;

    while(true){
        //uint64_t currentTime = esp_timer_get_time() / 1000; // Convert microseconds to milliseconds

        //if ((currentTime - lastTime) >= sampleTime) {
            //w = get_w_m1();
                
            // Convert float to string using sprintf
            //sprintf(sstr, "%.4f", w2percent(w));
            //uart_write_bytes(UART_PORT, (const char *)sstr, strlen(sstr));
            //uart_write_bytes(UART_PORT, "\n", 1);

            //lastTime = currentTime; /* Update last time */
        //}

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
