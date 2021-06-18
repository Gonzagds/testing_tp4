/*
 * teclado.h
 *
 *  Created on: Mar 27, 2021
 *      Author: na.gonzalo@gmail.com
 */
#include "prototipos.h"

#ifndef MAIN_TECLADO_H_
#define MAIN_TECLADO_H_

typedef enum {t_menu = 0, t_cero, t_tara, t_print, t_func, t_max} tecla_index_t;
typedef uint32_t tecla_bit_t;

#define get_tecla(var,key)		((var)&(1 << key))
#define set_tecla(var,key)		var |=  (1 << key)
#define clear_tecla(var, key)	var &= ~(1 << key)
#define reset_teclas(var)		var = 0

//NOTA!!!: Todos los defines que estan debajo eran privados del archivo
//teclado.c y se hicieron publicos para poder hacer el testing
/******************************************************************************
 * Defines and typedefs
 *****************************************************************************/
//#define TECLADO_TASK_PRIORITY	2
//#define TECLADO_TASK_CORE		tskNO_AFFINITY

#define DIR_EXP_INTER PCA9555_I2C_ADDR0 //direccion del expansor interno
#define DELAY_REBOTE  50
#define CANT_IO		16

#define IN 1
#define OUT 0
#define Pin(pin , conf) (conf << pin)

//Macros que permiten configurar los pines como entrada/salida individualmente
#define CONF_PORT0 (Pin(7,IN) | Pin(6,IN) | Pin(5,IN)| Pin(4,IN)| Pin(3,IN)| \
		Pin(2,IN)| Pin(1,IN)| Pin(0,IN))

#define CONF_PORT1 (Pin(7,OUT) | Pin(7,OUT) | Pin(5,OUT)| Pin(4,OUT)| \
		Pin(3,OUT)| Pin(2,OUT)| Pin(1,OUT)| Pin(0,OUT))


//definicion de pines. Se enumeran del 0 al 15
#define PIN_T_MENU	0
#define PIN_T_CERO	1
#define PIN_T_TARA	2
#define PIN_T_PRIN	3
#define PIN_T_FUNC	4

#define check_bit(bit , reg) (reg & (1<<bit))

typedef struct
{
	union
	{
		uint16_t inputs;
		uint8_t ports_in[2];
	};
	union
	{
		uint16_t outputs;
		uint8_t ports_out[2];
	};
}IO_t;

typedef enum
{
    up = 0,
    down,
	falling,
    rising
}est_tec;

typedef struct
{
    uint8_t pin;		//pin donde est� mapeada la tecla, 0 a 15
    est_tec estado;
    tecla_index_t index;
}tecla_t;


void teclado_init(void);

uint8_t get_input_state(uint8_t input);

esp_err_t teclado_config(void);

esp_err_t read_imputs();

esp_err_t write_outputs();

void read_key(tecla_t* tecla);

void read_keyboard(void);
#endif /* MAIN_TECLADO_H_ */
