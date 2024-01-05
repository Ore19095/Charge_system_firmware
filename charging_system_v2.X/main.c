/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on December 27, 2023, 7:49 PM
 */


#include <xc.h>
#define F_CPU 8000000UL 
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

#include "controller_io_utils.h"
#include "adc_utils.h"
#include "LiON_definitions.h"
#include "NiMH_definitions.h"

//------------------ Prototipos ------------------
void conf_uart(void);
void send_data(const char* data, uint8_t num);
void externFunct(void);
void configureTimer0(void);
void configureTimer1(void);
void setPWM(uint16_t dutyCycle);

void main(void){
    conf_uart();
    configureADC();
    configureTimer2();
    sei();

    while (1){
      

        
    }
    

    return;
}

void externFunct(void){
    // funcion para probar la ejecucion de una funcion externa con el timer2
    send_data("HOLA\n",5);
}
// ------------------ UART ------------------
void send_data(const char* data, uint8_t num){
    for (int i=0; i<num && data[i]!=0; i++ ) {
        while( !( UCSR0A&(1<<UDRE0))  );
        UDR0 = data[i];
        //uartBuffer[writePointer] = data[i];
        //writePointer=(writePointer+1)%UART_BUFFER;
    }
    //UCSR0B |= 1<<UDRIE0; // HAbilitar interrupcion
    return;
}

void conf_uart(void){
     UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad  
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
    UBRR0 = 103; //1M baud rate con Fosc = 8MHz
}

// ------------------ Timer 0 ---------------- 
#define DDR_OC0A DDRD
#define PIN_OC0A 6
void  configureTimer0(){
    /*
        Usado para generar una señal Cuadrada con una frecuencia de 
        7.812Khz, para el funcionamiento del charge pump que genera
    */
    // Configuración de los pines OC0A como salida
    DDR_OC0A |= (1 << PIN_OC0A);
    // Temporizador en modo Fast PWM, TOP = OCRA WGM0[2:0] = 0b111
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << WGM02);
    // Prescalador de 8, CS0[2:0] = 0b010
    TCCR0B |= (1 << CS01) ;
    TCCR0B &= ~((1 << CS02) | (1 << CS00) );
    // Configuración de los pines OC0A como salida, COM0A[1:0] = 0b10
    TCCR0A |= (1 << COM0A0);
    TCCR0A &= ~(1 << COM0A1); // Toggle on compare match
    OCR0A = 7; // Fs = Fosc / (2*N * (1 + OCR0A)) ,N = 8 (prescalador)
    return;
}

// ------------------ Timer 1 ----------------
#define DDR_OC1A DDRB
#define PIN_OC1A 1
void configureTimer1(){
    // Usado para generar una señal PWM de 10 bits, que controlará
    // la salida del convertidor buck
    // Configuración de los pines OC1A como salida
    DDR_OC1A |= (1 << PIN_OC1A);
    // Temporizador en modo Fast PWM 10 bits WGM1[3:0] = 0b0111
    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    TCCR1B &= ~(1 << WGM13);
    TCCR1B |= (1 << WGM12);
    // Prescalador de 1, CS1[2:0] = 0b001
    TCCR1B |= (1 << CS10);
    TCCR1B &= ~(1 << CS11 | 1 << CS12);
    // Configuración de los pines OC1A como salida, COM1A[1:0] = 0b10
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0);
    OCR1A = 1023; // duty cycle = 100% (salida mas baja del convertidor)
    return;
}

void setPWM(uint16_t dutyCycle){
    // dutyCycle = 0 -> 0% dutyCycle = 1023 -> 100%
    OCR1A = dutyCycle;
    return;
}
