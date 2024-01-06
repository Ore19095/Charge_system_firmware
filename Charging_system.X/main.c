/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 20, 2023, 7:03 PM
 */

//---------------- CONSTANTES --------------------------

#define F_CPU 8000000L

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "controller_io_utils.h"
#include "LiON_definitions.h"
#include "NiMH_definitions.h"

// --------------- DEFINICIONES --------------------------
#define UART_BUFFER 64
//----------------- limites ----------------

#define V_IN_MIN 359 // detecta cuando hay 10V en la alimentacion 10*1024/5 = 359.3

#define V_LECT_MAX_LION 860 // 4.2V
#define V_LECT_MIN_LION 860 // 3V 3*1024/5 = 614.4

#define V_LECT_MAX_NIMH  922// V_NiMH = 6V, 6*(3/4)(1024/5) = 921.6
#define V_LECT_MIN_NIMH  614// V_NiMH = 4V, 4*(3/4)(1024/5) = 614.4

#define I_LION 614 // I = 1A, 1*3*(1024/5) = 614.4 //corriente maxima de carga LiON
#define I_NIMH 184 // I = 0.3A, 0.3*3*(1024/5) = 184.3 //corriente maxima de carga NiMH

#define I_LION_MIN 92 // I = 150mA, 0.15*3*(1024/5) = 92.16 //corriente minima de carga LiON
#define I_LION_MAX 614 // I = 1A, 1*3*(1024/5) = 614.4 //corriente maxima de carga LiON
//------------------------------------------------------------

#define PID_D -2.244e-4
#define PID_I -500.5
#define PID_P -1.2515
#define PID_Fs 1000 // frecuencia de muestreo del PID
//PID for current control
#define PID_D_CUR -4.244e-4/5
#define PID_I_CUR -350.5/10
#define PID_P_CUR -1.2515/10
#define PID_Fs_CUR 1000 // frecuencia de muestreo del PID
// --------------- VARIABLES -----------------------------

/*Para el envio de datos por medio de uart*/
uint8_t writePointer = 0;
uint8_t readPointer = 0;
char uartBuffer[UART_BUFFER]; // no usar directamente, usar send_data
/*Lectura del ADC*/
/*Lectura del ADC*/
volatile uint16_t vLion = 0; // => ADC2
volatile uint16_t vNimh = 0; // => ADC1
volatile uint16_t vAlimentacion = 0; // => ADC7
/*Variables del sistema de carga*/
volatile uint16_t buck_voltage = 0; // voltaje del convertidor  ADC0
volatile uint16_t buck_current = 0; // corriente de carga  ADC3
uint8_t nextValue = 0; //siguiente canal del ADC a leer
/*variables pid*/
float e_anterior = 0;
float e = 0;
float e_integral = 0;
float ref = 100;
float ref_top = 614;

uint8_t charge_lion = 2;
uint8_t charge_nimh = 2;
uint8_t charge_lion_prev = 0;
uint8_t contador = 0;                               
/*control del tiempo*/
volatile uint32_t miliSeconds = 0;
uint32_t blinkPeriod = 500;
uint32_t blinkPeriodNiMH = 500;

// 1 = encendido, 0 = apagado
uint8_t blinkLionON = 0;
// -------------- PROTOTIPOS DE FUNCIONES ----------------
void FSMBlink();
void FSMChargeLiON();

void send_data(const char* data, uint8_t num);
/**Configuración de periféricos*/
void conf_timer0();
void conf_timer1();
void conf_timer2();
/*Configuración para el ADC*/
void conf_adc();
/*Configuración para el UART*/
void conf_uart();
/*Configuración para los puertos*/
void conf_ports();
uint8_t isConnected(void);
/*pid*/
void calculate_pid(void);
void calculate_pid_voltage(void);

void pid(void);


int main(void) {

    //------ CONFIGURACION DE PUERTOS ---------------------
    conf_ports();
    //------ CONFIGURACION DE TIMERS ---------------------  
    conf_timer0();
    conf_timer1();
    conf_timer2();
    //------ CONFIGURACION DE ADC -------------------------
    conf_adc();
    //  ------ CONFIGURACIÓN UART -----------------
    conf_uart();
    //----------------- INTERRUPCIONES ----------------------
    sei(); // Habilitar interrupciones globales
    // encender buck
    
    // activar switch de carga LiON
    
    /*variables para blink*/
    uint32_t lastBlinkTime = 0;
    uint32_t lastBlinkTimeNiMH = 0;
    uint32_t lastChargeFSMTime = 0;

    while(1){
        //if(isConnected() ) nimhLedOn();
        //nimhLedOn();
        /*ejecutar las fsm de carga*/
        if(miliSeconds - lastChargeFSMTime > 1){
            
            lastChargeFSMTime = miliSeconds;
            FSMChargeLiON();
        
        }


        if(miliSeconds - lastBlinkTime > blinkPeriod && blinkLionON){
            
            lastBlinkTime = miliSeconds;
            FSMBlink();
        }
    }
    return 0;
}

#define STATION_TRESHOLD 323
uint8_t isConnected(void){
    
    if (vAlimentacion > STATION_TRESHOLD){
        return 1;
    }
    return 0;
}
// --------------- FSMs ----------------------------------
/*Debe de ejecutarse cada 1 ms*/
// variable para establecer si una bateria esta vacia o no, 1 = vacia, 0 = no vacia
uint8_t emptyNiMH = 0;

uint8_t chargeLionState = 0;
void FSMChargeLiON(){
    switch(chargeLionState){
        case 0:
            blinkLionON = 0;
            // Esperar a que el voltaje de la bateria sea menora 3V
            if (vLion < V_LION_EMPTY && isConnected()){
                chargeLionState = 1; //inicia la carga
            }
            break;
        case 1:
            // iniciar estados del PID
            blinkLionON = 1;
            e = 0;
            e_anterior = 0;
            e_integral = 0;
            ref = 0;
            ref_top = I_LION_CHARGE;

            buckOn(); //se enciende el controlador buck
            chargeLionOn(); // se enciende el controlador de carga
            supplyLionOff(); // se apaga la fuente de voltaje
            // se parpadea cada 250ms
            blinkPeriod = 250;
            chargeLionState = 2;
        case 2:
            // Cargar la bateria hasta que el voltaje sea mayor a 4.2V
            calculate_pid();
            if (vLion > V_LION_FULL){
                chargeLionState = 3; // carga completa
            }
            break;
        case 3:
            // se cambia a modo de voltaje constante
            e = 0;
            e_anterior = 0;
            e_integral = 0;
            ref = V_LION_FULL;
            ref_top = V_LION_FULL;
            // se parpadea cada 500ms
            blinkPeriod = 500;
            chargeLionState = 4;
        case 4:
            // se completa la carga hasta que la corriente sea menor a 0.15A
            calculate_pid_voltage();
            if (buck_current < I_LION_STOP_CHARGE){
                blinkLionON = 0;
                chargeLionOn(); // se apaga el controlador de carga
                buckOff(); // se apaga el controlador buck
                chargeLionState = 0; // carga completa
            }
            break;

    }
    return;
}


#define TIKLE_TIME (1000*60*15) // 15 minutos
#define TIKLE_COOLDOWN (1000*60) // 1 minutos
uint8_t chargeNiMHState = 0;
void FSMChargeNiMH(){
    static uint32_t lastChargeTime = 0;

    switch (chargeNiMHState){
    case 0:
        if ( emptyNiMH && isConnected()) chargeNiMHState = 1;
        break;
    case 1:
        // iniciar estados del PID
        blinkLionON = 1;
        e = 0;
        e_anterior = 0;
        e_integral = 0;
        ref = 0;
        ref_top = I_NIMH_CHARGE;
        // se enciende el buck
        buckOn();
        // se enciende el controlador de carga
        chargeNiMHOn();
        // se quita la bateria como fuente principal 
        supplyNiMHOff();
        // se parpadea cada 250ms
        blinkPeriodNiMH = 250;
        chargeNiMHState = 2;
        break;
    
    default:
        break;
    }
}
uint8_t blinkState = 0;
void FSMBlink(){
    switch(blinkState){
        case 0:
            blinkState = 1;
            lionLedOn();
            break;
        case 1:
            blinkState = 0;
            lionLedOff();
            break;
    }
    return;
}

// --------------- FUNCIONES -----------------------------
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

void  conf_timer0(){
    /*
        Usado para generar una señal Cuadrada con una frecuencia de 
        62.5Khz, para el funcionamiento del charge pump que genera
    */
    // Temporizador en modo Fast PWM, TOP = OCRA WGM0[2:0] = 0b111
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << WGM02);
    // Prescalador de 64, CS0[2:0] = 0b010
    TCCR0B |= (1 << CS01) ;
    TCCR0B &= ~((1 << CS02) | (1 << CS00) );
    // Configuración de los pines OC0A como salida, COM0A[1:0] = 0b10
    TCCR0A |= (1 << COM0A0);
    TCCR0A &= ~(1 << COM0A1); // Toggle on compare match
    OCR0A = 16; // Fs = Fosc / (2*N * (1 + OCR0A)) ,N = 64 (prescalador)
    return;
}

void conf_timer1(){
    // Usado para generar una señal PWM de 10 bits, que controlará
    // la salida del convertidor buck
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
    OCR1A = 1023; // duty cycle = 0%
    return;
}

void conf_timer2(){
    // Usado para muestrear el ADC a la frecuencia de 1kHz
    // Temporizador en modo CTC WGM2[2:0] = 0b010
    TCCR2A |= (1 << WGM21);
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22);
    // Prescalador de 32 CS2[2:0] = 0b011
    TCCR2B |= (1 << CS21) | (1 << CS20);
    TCCR2B &= ~(1 << CS22);
    // Se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    // Frecuencia de interrupcion = 1kHz
    OCR2A = 49;  // Fs = Fosc / (N * (1 + OCR2A)) ,N = 32 (prescalador)
    return;
}

void conf_adc(){
    // Referencia de voltaje en AVCC con capacitor en AREF
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    ADMUX |= ((1 << MUX1) | (1 << MUX0));
    // activar la interrupcion por ADC
    ADCSRA |= (1 << ADIE);
    // prescalador de 32 para el reloj del modulo
    ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
    ADCSRA &= ~((1 << ADPS1));
    // Habilitar el ADC
    ADCSRA |= (1 << ADEN);
    //habilidar interrupcion por ADC
    ADCSRA |= (1 << ADIE);
    return;
}

void conf_uart(){
     UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad
    
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
   
    UBRR0 = 103; //1M baud rate con Fosc = 8MHz
}

void conf_ports(){
    // Deshabilita la funcion digital de los pines ADC0-ADC3
    DIDR0 = 0b00001111; 
    /*  PD0: RXD (Entrada)  PD4: PWR_LiON (Salida)
        PD1: TXD (Salida)   PD5: ON_Buck (Salida)
        PD2: N.C. (Entrada) PD6: PWM_PUMP (Salida)
        PD3: N.C. (Entrada) PD7: MUX_LiON (Salida)*/
    DDRD = 0b11110010;
    /*  PB0: MUX_NIMH (Salida)   PB4: MISO_PROG (Entrada)
        PB1: PWM_DAC (Salida)    PB5: SCK_PROG (Entrada)
        PB2: PWR_NIMH (Salida)   PB6: LED_NiMH (Salida)
        PB3: MOSI_PROG (Entrada) PB7: LED_LiON (Entrada)*/
    DDRB = 0b11000111;
    /*  PC0: ADC0 (Entrada) PC4: SDA (Entrada)
        PC1: ADC1 (Entrada) PC5: SCL (Entrada)
        PC2: ADC2 (Entrada) PC6: RESET (Entrada)
        PC3: ADC3 (Entrada) PC7: XTAL2 (Entrada)*/
    DDRC = 0b00000000;
}

void calculate_pid(){
    e = ref  - buck_current;

    e_integral += e/PID_Fs;
    
    float u = PID_P_CUR*e + PID_I_CUR*e_integral + PID_D_CUR*(e - e_anterior)*PID_Fs;

    if(u > 1023) u = 1023;
    else if(u < 0) u = 0;

    OCR1A =(uint16_t) u;    
    return;
}

void calculate_pid_voltage(){
    e = ref  - buck_voltage;

    e_integral += e/PID_Fs;
    
    float u = PID_P*e + PID_I*e_integral + PID_D*(e - e_anterior)*PID_Fs;

    if(u > 1023) u = 1023;
    else if(u < 0) u = 0;

    OCR1A =(uint16_t) u;    
    return;
}

//---------------- INTERRUPCIONES ------------------------
ISR(USART_UDRE_vect){
     
    if( writePointer == readPointer ){
        UCSR0B&=~(1<<UDRIE0);//desabilitar interrupcion   
    }
    else{
        UDR0 = uartBuffer[readPointer];
        readPointer = (readPointer+1)%64;
    }
    
    return;
}

ISR(TIMER2_COMPA_vect){
    static uint8_t contadorMilis = 0;
    // Se muestrea el ADC cada 0.2ms
    ADCSRA |= (1 << ADSC);
    contador = (contador+1)%50;
    if(contador == 0 && ref <ref_top){
        ref += 1;   
    }
    // Se actualiza el tiempo
    contadorMilis++;
    if(contadorMilis==5){
        miliSeconds++;
        contadorMilis = 0;
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
    if(charge_lion==1){
        calculate_pid();
    }
    

    return;
}