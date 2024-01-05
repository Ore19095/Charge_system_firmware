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
// ------------------ Definiciones ------------------
#define BUCK_V_CHANNEL 0 // canal de voltaje de salida del buck
#define NiMH_V_CHANNEL 1 // canal de voltaje bateria NiMH
#define LION_V_CHANNEL 2 // canal de voltaje bateria LiON
#define BUCK_I_CHANNEL 3 // Canal de la medicion de corriente
#define SOURCE_CHANNEL 7 // Canal de la fuente de voltaje
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
// ----------------- funciones --------------

/** Funcion que determina si la placa esta conectada a la estación de
 *  carga, esto se determina midiendo el voltaje de la fuente de voltaje
 *  y si es mayor a 9V se considera que la placa esta conectada a la 
 *  estación de carga, si es menor se considera que está conectada a la
 *  las baterias del carro.
 */
#define STATION_TRESHOLD 431
uint8_t isConnected(void){
    uint16_t sourceVoltage = readADC(SOURCE_CHANNEL);
    
    if (sourceVoltage > STATION_TRESHOLD){
        return 1;
    }
    return 0;
}

// ----------------- funciones FSM ----------
uint8_t chargeLionState = 0;
void FSMChargeLiON(){
    switch(chargeLionState){
        case 0:
            // Esperar a que el voltaje de la bateria sea menora 3V
            if (readADC(LION_V_CHANNEL) < V_LION_EMPTY && isConnected()){
                chargeLionState = 1; //inicia la carga
            }
            break;
        case 1:
            // iniciar estados del PID
            startPID(I_LION_CHARGE); // se establece la corrinte que debe de cargar
            chargeLionState = 2;
        case 2:
            // Cargar la bateria hasta que el voltaje sea mayor a 4.2V
            if (readADC(LION_V_CHANNEL) > V_LION_FULL){
                chargeLionState = 2; // carga completa
            }
            break;
        case 3:
            // se completa la carga hasta que la corriente sea menor a 0.15A
            if (readADC(BUCK_I_CHANNEL) < I_LION_STOP_CHARGE){
                chargeLionState = 0; // carga completa
            }
            break;

    }
    return;
}

// ------------------ PID -------------------
#define PID_Fs 1000 // frecuencia de muestreo del PID
#define D_VOLTAGE (-2.244e-4*PID_Fs)
#define I_VOLTAGE (-500.5/PID_Fs)
#define P_VOLTAGE -1.2515
//PID for current control
#define D_CURRENT ((-4.244e-4/5)*PID_Fs)
#define I_CURRENT ((-350.5/10)/PID_Fs)
#define P_CURRENT -1.2515/10
#define PID_INCREMENT 10
// pid variables
float error = 0;
float errorPrev = 0;
float errorSum = 0;
float pidReference = 0;
float pidReferenceTop = 0;
// pid a ejecutar
void (*pidFunction)(void) = &calculatePIDVoltage;

typedef enum {
    PID_VOLTAGE,
    PID_CURRENT
    } PID_MODES;

void startPID(float reference, PID_MODES mode){
    error = 0;
    errorPrev = 0;
    errorSum = 0;
    pidReference = (mode==PID_CURRENT)? 0: reference;
    pidFunction = (mode==PID_CURRENT)? &calculatePIDCurrent: &calculatePIDVoltage;
    pidReferenceTop = reference;
    return;
}

void calculatePIDCurrent(){
    static uint8_t counter = 0;
    if (counter++ == PID_INCREMENT){
        counter = 0;
        if(pidReference<pidReferenceTop) pidReference++;
    }
    float pidOutput;
    // se calcula el error y se actualiza la suma de errores
    error = pidReference - readADC(BUCK_I_CHANNEL);
    errorSum += error;
    // se calcula la salida del PID
    pidOutput = P_CURRENT*error + I_CURRENT*errorSum + D_CURRENT*(error-errorPrev);
    // se actualiza el valor previo del error
    errorPrev = error;
    // se limita la salida del PID
    if(pidOutput>1023) pidOutput = 1023;
    if(pidOutput<0) pidOutput = 0;
    // se actualiza el valor del PWM
    setPWM((uint16_t)pidOutput);
    return;
}

void calculatePIDVoltage(){
    float pidOutput;
    // se actualiza el valor top del PID
    // se calcula el error y se actualiza la suma de errores
    error = pidReference - readADC(BUCK_V_CHANNEL);
    errorSum += error;
    // se calcula la salida del PID
    pidOutput = P_VOLTAGE*error + I_VOLTAGE*errorSum + D_VOLTAGE*(error-errorPrev);
    // se actualiza el valor previo del error
    errorPrev = error;
    // se limita la salida del PID
    if(pidOutput>1023) pidOutput = 1023;
    if(pidOutput<0) pidOutput = 0;
    // se actualiza el valor del PWM
    setPWM((uint16_t)pidOutput);
    return;
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
