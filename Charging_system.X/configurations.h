#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#include <xc.h>
/**Configuración de timers*/
void conf_timer0();
void conf_timer1();
void conf_timer2();
/*Configuración para el ADC*/
void conf_adc();
/*Configuración para el UART*/
void conf_uart();
/*Configuración para los puertos*/
void conf_ports();

#endif