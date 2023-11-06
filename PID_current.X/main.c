/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 20, 2023, 7:03 PM
 */

// ---------------   FUSES    ---------------------------


//---------------- CONSTANTES --------------------------

#define F_CPU 8000000L

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
// --------------- DEFINICIONES --------------------------
#define UART_BUFFER 64

// --------------- VARIABLES -----------------------------
/*Para el envio de datos por medio de uart*/
uint8_t writePointer = 0;
uint8_t readPointer = 0;
char uartBuffer[UART_BUFFER]; // no usar directamente, usar send_data
/*Lectura del ADC*/
uint16_t adcValue = -1; // valor del ADC
uint16_t vLion = -1; // => ADC2
uint16_t vNimh = -1; // => ADC1
uint16_t current = -1; // => ADC3
uint16_t vBuck = -1; // => ADC0
uint8_t nextValue = 0; //siguiente canal del ADC a leer
/*variables pid*/
int16_t u[3] = {0,0,0}; // u[k] => u(n-k) n = muestra actual
int16_t e[3] = {0,0,0}; // x[k] => e(n-k) n = muestra actual
int16_t ref = 512; // valor de referencia
int16_t voltage_b[3] = {-3.074,
                        - 2.65,
                        - 0.5711}; // coeficientes del numerador, pid voltaje
int16_t voltage_a[3] = {0,0,-1}; // coeficientes del denominador, pid voltaje
int16_t current_b[3] = {0,0,0}; // coeficientes del numerador, pid corriente
int16_t current_a[3] = {0,0,0}; // coeficientes del denominador, pid corriente

int16_t *a_coefficients; // puntero a los coeficientes del denominador
int16_t *b_coefficients; // puntero a los coeficientes del numerador

/*Controlador de carga  */
/* Bit 0: READ_CHAN0
   
*/
uint8_t controller_flags = 0; 
                                

// -------------- PROTOTIPOS DE FUNCIONES ----------------
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

void pid(void);

int main(void) {
    a_coefficients = voltage_a;
    b_coefficients = voltage_b;
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
    PORTD &= ~(1 << PD5);

    char adc_data[6];
    while(1){
        //_delay_ms(10);
        //send_data("hola\n",7);
        /*Enviar el valor del ADC por uart*/
        _delay_ms(100);
        sprintf(adc_data,"%d\n",OCR1A);
        send_data(adc_data,5);
    }
    return 0;
}

// --------------- FUNCIONES -----------------------------
void send_data(const char* data, uint8_t num){
    for (int i=0; i<num && data[i]!=0; i++ ) {
        /*while( !( UCSR0A&(1<<UDRE0))  );
        UDR0 = data[i];*/
        uartBuffer[writePointer] = data[i];
        writePointer=(writePointer+1)%UART_BUFFER;
    }
    UCSR0B |= 1<<UDRIE0; // HAbilitar interrupcion
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
    OCR1A = 0; // duty cycle = 0%
    return;
}

void conf_timer2(){
    // Usado para muestrear el ADC a la frecuencia de 1kHz
    // Temporizador en modo CTC WGM2[2:0] = 0b010
    TCCR2A |= (1 << WGM21);
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22);
    // Prescalador de 64 CS2[2:0] = 0b011
    TCCR2B |= (1 << CS21) | (1 << CS20);
    TCCR2B &= ~(1 << CS22);
    // Se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    // Frecuencia de interrupcion = 1kHz
    OCR2A = 124;  // Fs = Fosc / (N * (1 + OCR2A)) ,N = 1024 (prescalador)
    return;
}

void conf_adc(){
    // Referencia de voltaje en AVCC con capacitor en AREF
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    ADMUX &= ~((1 << MUX1) | (1 << MUX0));
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
   
    UBRR0 = 0; //1M baud rate con Fosc = 8MHz
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
    // Se calcula el valor de la señal de control
    e[2] = e[1]; // x[k+1] = x[k]
    e[1] = e[0]; // x[k] = x[k-1]

    u[2] = u[1]; // u[k+1] = u[k
    u[1] = u[0]; // u[k] = u[k-1]
    // Se lee el valor del ADC
    e[0] = ref - adcValue;
    // Se calcula el valor de la señal de control
    for(int i = 0 ; i < 3 ; i++){
        u[0] += b_coefficients[i]*e[i];
        u[0] -= a_coefficients[i]*u[i];
    }

    if(u[0] > 512) u[0] = 512;
    else if(u[0] < -512) u[0] = -512;

    OCR1A = 512 + u[0]; // duty cycle = u[0]/1023
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
    // Se muestrea el ADC cada 1ms
    ADCSRA |= (1 << ADSC);
    
    return;
}

ISR(ADC_vect){
    // Se calcula el valor de la señal de control
    adcValue = ADC;
    calculate_pid();

    return;
}