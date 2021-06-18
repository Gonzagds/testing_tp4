#include "prototipos.h"

//funciones de Reemplazo para simular la comunicacion con el
//GPIO expander via i2c.


/************************************************************************
* Las variables definas a continuación simularán los registros
* GPIO expander
*************************************************************************/
uint8_t port_conf0 = 0;
uint8_t port_conf1 = 0;

uint8_t port_out0 = 0;
uint8_t port_out1 = 0;

uint8_t port_in0 = 0;
uint8_t port_in1 = 0;

/*************************************************************************
* Variables que simulan el envío de datos por una cola de FreeRtos
**************************************************************************/
uint32_t queue_data = 0;
bool queue_new_data = false;




esp_err_t  pca9555a_config_port(uint8_t address, uint8_t port, uint8_t conf)
{
	//uint8_t buff[2];
	if(port > PORT_MAX)
		return ESP_FAIL;

    //asumimos que la escritura por i2c es siempre exitosa 
    if(port == 0)
        port_conf0  = conf;
    else
        port_conf1 = conf;
       
	return ESP_OK;
}

esp_err_t  pca9555a_write_port(uint8_t address, uint8_t port, uint8_t data_wr)
{
	//uint8_t buff[2];
	if(port > PORT_MAX)
		return ESP_FAIL;

    //asumimos que la escritura por i2c es siempre exitosa 
    if(port == 0)
        port_out0 = data_wr;
    else
        port_out1 = data_wr;
       
	return ESP_OK;
}

esp_err_t  pca9555a_read_port(uint8_t address, uint8_t port,uint8_t* data_rd)
{

	if(port > PORT_MAX)
		return ESP_FAIL;

    //asumimos que la escritura y lectura por i2c es siempre exitosa 
    if(port == 0)
        *data_rd = port_in0;
    else
        *data_rd = port_in1;
       
	return ESP_OK;
}

int xQueueSend( QueueHandle_t teclado_queue, uint32_t* tecla_bit,  int time )
{
    queue_data = *tecla_bit;
    queue_new_data = true;

    return 0;
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic( StaticSemaphore_t key_mutex_buffer )
{
    return 0;
}


void vTaskDelete(void* param){};


void xSemaphoreTake( SemaphoreHandle_t key_semaphore,  uint32_t  time ){};

void xSemaphoreGive(SemaphoreHandle_t key_semaphore){};

void vTaskDelay(uint32_t portTICK ){};


//Codigo original
/*esp_err_t  pca9555a_config_port(uint8_t address, uint8_t port, uint8_t conf)
{
	uint8_t buff[2];
	if(port > 1)
		return ESP_FAIL;
	buff[0] = PCA9555_CONFIG_0 + port;
	buff[1] = conf;
	return write_i2c_int(address, buff, 2, ACK_CHECK_EN);
}

esp_err_t  pca9555a_write_port(uint8_t address, uint8_t port, uint8_t data_wr)
{
	uint8_t buff[2];
	if(port > 1)
		return ESP_FAIL;
	buff[0] = PCA9555_OUTPUT_0 + port;
	buff[1] = data_wr;
	return write_i2c_int(address, buff, 2, ACK_CHECK_EN);
}

esp_err_t  pca9555a_read_port(uint8_t address, uint8_t port,uint8_t* data_rd)
{

	if(port > 1)
		return ESP_FAIL;

	port = PCA9555_INPUT_0 + port;
	write_i2c_int(address, &port, 1, ACK_CHECK_DIS);

	return read_i2c_int(address, data_rd, 1);
}*/