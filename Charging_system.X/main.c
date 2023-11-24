/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on November 10, 2023, 5:52 AM
 */


#define F_CPU 8000000L
// --------------- LIBRERIAS -----------------------------
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "pid.h"
#include "limit_constants.h"
#include "flags.h"
#include "configurations.h"

#define REF_STEP 10 // incrementa en 1.6mA la corriente de carga cada 10ms
// --------------- VARIABLES -----------------------------
/*Lectura del ADC*/
volatile uint16_t vLion = 0; // => ADC2
volatile uint16_t vNimh = 0; // => ADC1
volatile uint16_t vAlimentacion = 0; // => ADC7
/*Variables del sistema de carga*/
volatile uint16_t buck_voltage = 0; // voltaje del convertidor  ADC0
volatile uint16_t buck_current = 0; // corriente de carga  ADC3
/*referncia del pid*/
volatile uint16_t ref = 0; // referencia del pid
volatile uint16_t ref_top = 0; // referencia maxima del pid
volatile uint8_t ref_cont = 0; // contador de incremento de la referencia
/*banderas de estado del sistema (significado de cada bit en flags.h)*/
volatile uint8_t flag=0; // banderas de estado del sistema
volatile uint8_t flag_prev=0; // previas banderas de estado del sistema

volatile uint8_t firt_time = 1; //si es la primera vez ingresando al estado de carga
volatile uint8_t cont_frec = 0; // contador de frecuencia de carga
// -------------- PROTOTIPOS DE FUNCIONES ----------------
void charge_LiON();
int main(void) {
    // ----------------- CONFIGURACION -----------------------
    conf_ports();
    conf_timer0();
    conf_timer1();
    conf_timer2();
    conf_adc();
    conf_uart();
    //----------------- OTRAS INICIALIZCIONES-----------------
    sei(); // Habilitar interrupciones globales
    PORTB |= (1 << PB2); // Conectar bateria NiMH al carro
    PORTD |= (1 << PD5); // Apagar el convertidor buck
    //-------------------------------------------------------

    while(1){
        //se actualiza la bandera anterior
        flag_prev = flag;
       // revisar si el voltaje de alimentación es mayor a 10V
        if(vAlimentacion > V_IN_MIN) flag |= (1<<PWR_EXT_FLAG);
        else flag &= ~(1<<PWR_EXT_FLAG);

        /*-------- revisar estado de baterias ------------*/
        //si el voltaje de la bateria LiON es mayor a 4.2V *ver tabla en flags.h
        if(vLion > V_LECT_MIN_LION) flag |= (1<<FLAG_LION_1);
        else{
            flag &= ~(1<<FLAG_LION_1);
           
        }
        //si el voltaje de la bateria LiON es mayor a 3V
        if(vLion >= V_LECT_MAX_LION) flag |= (1<<FLAG_LION_0);
        else{
            flag &= ~(1<<FLAG_LION_0);
        } 
        
        // si la bateria se esta cargando se invierten las banderas
        /*if((flag&CHARGING_FLAG) == 5 || (flag&CHARGING_FLAG) ==6 ){
            flag &= ~(1<<FLAG_LION_0);
            flag |= (1<<FLAG_LION_1);
            //PORTB |= (1 << PB6)
        }*/

        if(flag&(1<<PWR_EXT_FLAG)){
            charge_LiON();
        }

    }
    return 0;
}

// ----------------- FUNCIONES -------------------------------
/**
 * Establece el valor de la salida del controlador, la carga de baterias
 * tiene 2 etapas: en la primera se  inicia la carga de la bateria con 
 * corriente constante y por ultimo al llegar al voltaje maximo (4.2) se 
 * coloca en modo de voltaje constante, deteniendo el proceso de carga
 * cuando la corriente de carga cae por debajo de los 100mA*/
void charge_LiON(){
    // Etapa 1: Carga de bateria con corriente constante, hasta que el
    // voltaje de la bateria alcance el valor maximo (4.2V)
    // Etapa 2: Carga de bateria con voltaje constante, hasta que la
    // corriente de carga caiga por debajo de 100mA
    // si esta descargada
    if((flag&(3<<FLAG_LION_0))==0 && firt_time==1  ){
        firt_time = 1;
        // en este punto se debe iniciar la carga de la bateria
        // por lo que se debe encender el convertidor buck
        PORTD &= ~(1 << PD5); // Encender el convertidor buck
        // se debe establecer la referencia del pid en 150mA
        ref = 0;
        // establecer la referencia maxima del pid en 1A
        ref_top = 0;
        // reiniiar el pid
        reset_pid();
        // establecer las banderas
        flag |= (1<<CHARGING_FLAG_2); // 1: carga LiON 0: carga NiMH
        flag |= (1<<CHARGING_FLAG_0); // 1:corriente constante
        flag &= ~(1<<CHARGING_FLAG_1); // 1:voltaje constante
        //encender Led 2
        //PORTB |= (1 << PB7);
        PORTB &= ~(1 << PB6);
        // conectar bateria LiON al buck
        PORTD |= (1 << PD7);
        // desconectar liOn del carro
        PORTB &= ~(1 << PB2);
    
    }
    // si el voltaje es mayor a los 4.2V
    // else if((flag&FLAG_LION)==(3<<FLAG_LION_0) && buck_current > I_LION_MIN){
    //     // se debe detener la carga de la bateria
    //     // por lo que se debe apagar el convertidor buck
    //     // se debe establecer la referencia del pid en 0mA
    //     ref = 600; // inicia con 4v
    //     // establecer la referencia maxima del pid en 1A
    //     ref_top = V_LECT_MAX_LION;
    //     // reiniiar el pid
    //     reset_pid();
    //     // establecer las banderas
    //     flag |= (1<<CHARGING_FLAG_2); // 1: carga LiON 0: carga NiMH
    //     flag &= ~(1<<CHARGING_FLAG_0); // 1:corriente constante
    //     flag |= (1<<CHARGING_FLAG_1); // 1:voltaje constante

    //     //encender led 1
    //     PORTB |= (1 << PB6);
    //     PORTB &= ~(1 << PB7);


    // }else if(buck_current < I_LION_MIN) {
    //     // deja de cargar la bateria
    //     PORTD |= (1 << PD5); // Apagar el convertidor buck
    //     // se coloca el estado de carga en 0
    //     flag &= ~(CHARGING_FLAG);
    // }
    return;
}

float e_anterior = 0;
float e = 0;
float e_integral = 0;
void calculate_pid(){
    e = ref  - buck_current;

    e_integral += e/PID_Fs;
    
    float u = PID_P_CUR*e + PID_I_CUR*e_integral + PID_D_CUR*(e - e_anterior)*PID_Fs;

    if(u > 1023) u = 1023;
    else if(u < 0) u = 0;

    OCR1A =(uint16_t) u;    
    return;
}
//---------------- INTERRUPCIONES ------------------------


ISR(TIMER2_COMPA_vect){
    // Se muestrea el ADC cada 1ms
    ADCSRA |= (1 << ADSC);
    
    ref_cont=(ref_cont+1)%REF_STEP;

    if(ref < ref_top && ref_cont == 0 ){    
            ref++; // aumentar la referencia del pid
        }  
    

    return;
}

ISR(ADC_vect){
    // Se calcula el valor de la señal de control
    switch (ADMUX & 0x0F){
    case 0:
        buck_voltage = ADC;
        ADMUX &= 0xF0;
        ADMUX |= 0x01;
        break;
    case 1:
        vNimh = ADC;
        ADMUX &= 0xF0;
        ADMUX |= 0x02;

        break;
    case 2:
        vLion = ADC;
        ADMUX &= 0xF0;
        ADMUX |= 0x03;

        break;
    case 3:
        buck_current = ADC;
        ADMUX &= 0xF0;
        ADMUX |= 0x07;
        
        break;
    case  7:
        vAlimentacion = ADC;
        ADMUX &= 0xF0;
        break;
    default:
        ADMUX &= 0xF0;
        break;
    }

    calculate_pid();

    return;
}