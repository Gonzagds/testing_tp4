/*
* Tests unitarios de las funciones que componen la tarea de lectura
* teclado frontal del equipo. El teclado frontal se lee mediante los 
* puertos de un GPIO expander con comunicación i2c
*/

#include "unity.h"
#include "teclado.h"
#include "prototipos.h"


extern uint8_t port_conf0;
extern uint8_t port_conf1;

extern uint8_t port_out0;
extern uint8_t port_out1 ;

extern uint8_t port_in0 ;
extern uint8_t port_in1 ;


extern uint32_t queue_data ;
extern uint32_t queue_new_data;


extern tecla_t teclas[t_max]; 

extern IO_t expander_io;

void setUp(void)
{
    uint8_t port_conf0 = 0;
    uint8_t port_conf1 = 0;

    uint8_t port_out0 = 0;
    uint8_t port_out1 = 0;

    uint8_t port_in0 = 0xFF;
    uint8_t port_in1 = 0xFF; 

    uint32_t queue_data = 0;
    uint32_t valor_cola = 0;

    queue_data = 0 ;
    queue_new_data = false;


    expander_io.inputs = 0;
    expander_io.outputs = 0;
}

void tearDown(void)
{
}

/*void test_LedsOffAfterCreate(void)
{
    uint16_t ledsVirtuales = 0xFFFF;
    Leds_Create(&ledsVirtuales);
    TEST_ASSERT_EQUAL_HEX16(0, ledsVirtuales);
}*/

void test_teclado_config(void)
{
    teclado_config();
    TEST_ASSERT_EQUAL_HEX8(CONF_PORT0, port_conf0);
    TEST_ASSERT_EQUAL_HEX8(CONF_PORT1, port_conf1);
}

void test_read_imputs(void)
{
    port_in0 = 0XAA;
    port_in1 = 0XBB;
    read_imputs(); 
    TEST_ASSERT_EQUAL_HEX16(0xBBAA , expander_io.inputs);
}

void test_write_outputs(void)
{
    expander_io.outputs = 0xBBAA;
    write_outputs();

    TEST_ASSERT_EQUAL_HEX8(0xAA , port_out0);
    TEST_ASSERT_EQUAL_HEX8(0xBB , port_out1);
}

void test_read_key_detec_falling(void)
{
     //las teclas tiene lógica inversa
    port_in0 = ~(1<<teclas[0].pin);
    read_imputs();
    read_key(teclas);

    TEST_ASSERT_EQUAL_HEX8(falling , teclas[0].estado);
}

void test_read_key_detec_down(void)
{
    port_in0 = ~(1<<teclas[0].pin);
    read_imputs();
    read_key(teclas);
    
    read_imputs();
    read_key(teclas);

    TEST_ASSERT_EQUAL_HEX8(down , teclas[0].estado);
}

void test_read_key_detec_rising(void)
{
    port_in0 = ~(1<<teclas[0].pin);
    read_imputs();
    read_key(teclas);
    
    read_imputs();
    read_key(teclas);

    port_in0 = 0xFF;
    read_imputs();
    read_key(teclas);

    TEST_ASSERT_EQUAL_HEX8(rising , teclas[0].estado);
}

void test_read_key_detec_up(void)
{
    port_in0 = ~(1<<teclas[0].pin);
    read_imputs();
    read_key(teclas);
    
    read_imputs();
    read_key(teclas);

    port_in0 = 0xFF;
    read_imputs();
    read_key(teclas);

    read_imputs();
    read_key(teclas);

    TEST_ASSERT_EQUAL_HEX8(up , teclas[0].estado);
}

void test_read_key_detec_boun(void)
{
    port_in0 = ~(1<<teclas[0].pin);
    read_imputs();
    read_key(teclas);
    
    port_in0 = 0xFF;
    read_imputs();
    read_key(teclas);

    TEST_ASSERT_EQUAL_HEX8(up , teclas[0].estado);
}


void test_read_keyboard_no_event(void)
{
    //función que se llama periodicamente en una tarea
    //se debe llamar al menos dos veces para detectar un 
    //evento de tecla por el antirebote
    read_keyboard();
    read_keyboard();

    TEST_ASSERT_EQUAL_HEX8((uint8_t) false , (uint8_t) queue_new_data);
}

void test_read_keyboard_key_press(void)
{
    //función que se llama periodicamente en una tarea
    //se debe llamar al menos dos veces para detectar un 
    //evento de tecla por el antirebote
    read_keyboard();
    read_keyboard();

    port_in0 = ~(1<<teclas[0].pin);

    read_keyboard();
    read_keyboard();

    //arribo evento de tecla por la cola?
    TEST_ASSERT_EQUAL_HEX8((uint8_t) true , (uint8_t) queue_new_data);
    //la tecla informada es la correcta
    TEST_ASSERT_EQUAL_HEX32((uint32_t) 1<<teclas[0].index , queue_new_data);
}

