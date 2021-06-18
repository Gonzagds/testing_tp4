
#include "stdint.h"
#include "stdbool.h"

#define PCA9555_I2C_ADDR0   32
#define ESP_FAIL    1
#define ESP_OK      0
#define PORT_MAX    1

#ifndef NULL
#define NULL 0
#endif

#define portMAX_DELAY 0xFFFFFFFF
#define portTICK_PERIOD_MS 1/1000

typedef int32_t esp_err_t;
typedef void* QueueHandle_t;
typedef void*  SemaphoreHandle_t;
typedef void*  StaticSemaphore_t;
typedef uint32_t TickType_t;

esp_err_t  pca9555a_config_port(uint8_t address, uint8_t port, uint8_t conf);

esp_err_t  pca9555a_write_port(uint8_t address, uint8_t port, uint8_t data_wr);

esp_err_t  pca9555a_read_port(uint8_t address, uint8_t port,uint8_t* data_rd);

int xQueueSend( QueueHandle_t teclado_queue, uint32_t* tecla_bit,  int time );


SemaphoreHandle_t xSemaphoreCreateMutexStatic( StaticSemaphore_t key_mutex_buffer );

#define configASSERT(key_semaphore)

void vTaskDelete(void* param);


void xSemaphoreTake( SemaphoreHandle_t key_semaphore,  uint32_t  time );

void xSemaphoreGive(SemaphoreHandle_t key_semaphore);

void vTaskDelay(uint32_t portTICK );

