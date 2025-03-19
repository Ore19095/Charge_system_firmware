/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef HARDWARE_ABSTRACTION_H
#define HARDWARE_ABSTRACTION_H

#include <xc.h> // include processor files - each processor file is guarded.
// ----------------------- REGISTER DEFINITIONS ----------------------------------
#define LED_DDR DDRB // registro para definir la direccion de los puertos
#define LED_PORT PORTB // registro para realizar la escritura en el puerto de los leds

#define PWM_PUMP_DDR DDRD // registro para la direccion del puerto para pwm para el 
                          // generador de voltaje negativo
#define PWM_DAC_DDR DDRB // registro para la direccion del puerto para pwm del adc

// ------------------------- PIN DEFINITIONS -------------------------------------
#define NIMH_LED PORTB7
#define LION_LED PORTB6
\

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation


#endif	/* XC_HEADER_TEMPLATE_H */

