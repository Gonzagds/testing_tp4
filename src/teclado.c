/*
 * teclado.c
 *
 *  Created on: Mar 11, 2021
 *      Author: na.gonzalo@gmail.com
 *
 * Si bien el fin principal de este archivo es manejar
 * el teclado frontal del equipo, tambien maneja las se�ales
 * del cargador de bateia. EN esta primera versi�n el segundo
 * puerto no se utiliza.
 * Se mapea el GPIO expander como un �nico puerto de 16 bits
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/semphr.h"
//#include "PCA9555A.h"
#include "teclado.h"
// #include "I2C.h"

//se agrega para testing



/******************************************************************************
 * Local variables
 *****************************************************************************/
// previamente expander_io era estática, se cambia visibilidad para poder hacer testing
IO_t expander_io;

// previamente teclas era estática, se cambia visibilidad para poder hacer testing
tecla_t teclas[t_max] =
{
		{
            .pin = PIN_T_MENU,
			.estado = up,
			.index = t_menu,
		},

		{
			.pin = PIN_T_CERO,
			.estado = up,
			.index = t_cero,
		},

		{
			.pin = PIN_T_TARA,
			.estado = up,
			.index = t_tara,
		},
		{
			.pin = PIN_T_PRIN,
			.estado = up,
			.index = t_print,
		},
		{
			.pin = PIN_T_FUNC,
			.estado = up,
			.index = t_func,
		}
};

static SemaphoreHandle_t key_semaphore = NULL;
static StaticSemaphore_t key_mutex_buffer;
// previamente tecla_bit era estática, se cambia visibilidad para poder hacer testing
tecla_bit_t tecla_bit;

QueueHandle_t teclado_queue = NULL;
/******************************************************************************
 * Local Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
esp_err_t teclado_config(void)
{
	esp_err_t ret;
	ret = pca9555a_config_port(DIR_EXP_INTER,0, CONF_PORT0);
	if(ret)
		return ret;
	ret = pca9555a_config_port(DIR_EXP_INTER,1, CONF_PORT1);

	return ret;
}

esp_err_t read_imputs()
{
	esp_err_t ret;
	ret = pca9555a_read_port(DIR_EXP_INTER,0, &expander_io.ports_in[0]);
	if(ret)
		return ret;
	ret = pca9555a_read_port(DIR_EXP_INTER,1, &expander_io.ports_in[1]);

	return ret;
}

esp_err_t write_outputs()
{
	esp_err_t ret;
	ret = pca9555a_write_port(DIR_EXP_INTER, 0, expander_io.ports_out[0]);
	if(ret)
		return ret;
	ret = pca9555a_write_port(DIR_EXP_INTER, 1, expander_io.ports_out[1]);

	return ret;
}

void read_key(tecla_t* tecla)
{
    if((void*) tecla == NULL)
        return;
    if(tecla->pin > CANT_IO)
    {
    	return;
    }

    switch (tecla->estado)
    {
    case up:
		if(check_bit(tecla->pin , expander_io.inputs))
		{
			tecla->estado = up;
			clear_tecla(tecla_bit,tecla->index);
		}
		else
		{
			tecla->estado = falling;
		}
		break;
    case down:
		if(check_bit(tecla->pin , expander_io.inputs))
		{
			tecla->estado = rising;
		}
		else
		{
			tecla->estado = down;
			set_tecla(tecla_bit, tecla->index);
		}
		break;
    case falling:
	   if(check_bit(tecla->pin , expander_io.inputs))
	   {
		   tecla->estado = up;
	   }
	   else
	   {
		   tecla->estado = down;
	   }
       break;
    case rising:
	   if(check_bit(tecla->pin , expander_io.inputs))
	   {
		   tecla->estado = up;
	   }
	   else
	   {
		   tecla->estado = down;
	   }
    }
}

void read_keyboard()
{
	uint8_t i;
	tecla_bit_t tec_aux = tecla_bit;

	for (i = 0 ; i < (sizeof(teclas)/sizeof(tecla_t)) ; i++)
	{
		read_key( teclas+i );
	}

	if(tec_aux != tecla_bit)
	{
		//hay un nuevo estado de teclado y debemos encolarlo
		xQueueSend( teclado_queue, &tecla_bit,  0 );
	}

}

void teclado_task(void* args)
{
    key_semaphore = xSemaphoreCreateMutexStatic( &key_mutex_buffer );

    configASSERT( key_semaphore );

	if(teclado_config() != ESP_OK)
	{
		vTaskDelete(NULL);
	}

	while(1)
	{


		xSemaphoreTake( key_semaphore, ( TickType_t ) portMAX_DELAY );
		read_imputs();
		xSemaphoreGive(key_semaphore);

		read_keyboard();

		vTaskDelay(DELAY_REBOTE/portTICK_PERIOD_MS );
	}
}


/******************************************************************************
 * Public Functions
 *****************************************************************************/

void teclado_init(void)
{
	/*internal_i2c_init();

	teclado_queue = xQueueCreate( 2, sizeof( tecla_bit_t ) );
	if( teclado_queue == 0 )
	{
	    //todo Queue was not created and must not be used.
	}

    portBASE_TYPE res = xTaskCreatePinnedToCore(&teclado_task,
    											"teclado",
                                                2048,
												NULL,
												TECLADO_TASK_PRIORITY,
												NULL,
												TECLADO_TASK_CORE);
    assert(res == pdTRUE);*/
}


/*uint8_t get_input_state(uint8_t input)
{
	uint8_t in;
	if(input > CANT_IO)
		return 0;

	xSemaphoreTake( key_semaphore, ( TickType_t ) portMAX_DELAY );
	in = (expander_io.inputs & (1 << input))?1:0;
	xSemaphoreGive(key_semaphore);

	return in;
}

void set_output_state(uint8_t output, uint8_t state)
{
	if(output > CANT_IO)
		return;

	xSemaphoreTake( key_semaphore, ( TickType_t ) portMAX_DELAY );
	if(state)
		expander_io.outputs |= (1 << output);
	else
		expander_io.outputs &= ~(1 << output);
	xSemaphoreGive(key_semaphore);

	return;
}*/






