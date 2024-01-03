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

void conf_uart(void);
void send_data(const char* data, uint8_t num);
void main(void){
    configureADC();
    configureTimer2();
    conf_uart();
    sei();

    char buf[10];

    while (1){
        sprintf(buf,"%d",readADC(0));
        send_data(buf,10);
        send_data(",",1);
        sprintf(buf,"%d",readADC(1));
        send_data(buf,10);
        send_data(",",1);
        sprintf(buf,"%d",readADC(2));
        send_data(buf,10);
        send_data(",",1);
        sprintf(buf,"%d",readADC(3));       
        send_data(buf,10);
        send_data(",",1);
        sprintf(buf,"%d",readADC(7));  
        send_data(buf,10);
        send_data("\n",1);
        _delay_ms(100);
        
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

void conf_uart(void){
     UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad
    
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
   
    UBRR0 = 103; //1M baud rate con Fosc = 8MHz
}