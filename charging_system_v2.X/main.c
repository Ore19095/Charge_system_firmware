/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on December 27, 2023, 7:49 PM
 */


#include <xc.h>
#define F_CPU 8000000UL 
#include <util/delay.h>

#include "controller_io_utils.h"
#include "adc_utils.h"

void main(void) {

    configureTimer2();
    configureADC();
    conf_uart();
    while (1){
        _delay_ms(1000);
        send_data("Hola mundo\n", 11);
    }
    

    return;
}

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

void conf_uart(){
     UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad
    
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
   
    UBRR0 = 103; //1M baud rate con Fosc = 8MHz
}