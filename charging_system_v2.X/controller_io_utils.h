/* 
 * File: Controller_utils.h
 * Author: Angel Orellana
 * Comments: Libreria para abstraer el uso de pines digitales para el cargador 
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef CONTROLLER_IO_UTILS_H
#define	CONTROLLER_IO_UTILS_H

#include <xc.h> 

// Definiciones que indican que pin se utilizar� para en cada funcion, no se 
// recomienda cambiar a menos que se redise�e el circuito impreso

// --------------- INDICADORES LED ---------------------------------------
#define PORT_LED_NIMH PORTB //  puerto al que pertenece el IO del led NIMH
#define PIN_PORT_NIMH 6 // pin del puerto utilizado para led NIMH
#define PORT_LED_LION PORTB //  puerto al que pertenece el IO del led LION
#define PIN_PORT_LION 7 // pin del puerto utilizado para led LION
//---------------- CONTROL BUCK ------------------------------------------
#define PORT_BUCK_CTR PORTD // puerto al que pertenece el io para encender el buck
#define PIN_PORT_BUCK_CTR 5// pin del puerto utilizado para controlar el buck
//---------------- POWER SWITCHES ----------------------------------------
#define PORT_CHARGE_LION PORTD // puerto que pertenece el io para cargar lion
#define PIN_CHARGE_LION 7 // pin del puerto PORT_CHARGE_LION usado para activar el switch
#define PORT_CHARGE_NIMH PORTB // puerto que pertenece el io para cargar nimh
#define PIN_CHARGE_NIMH 0 // pin del puerto PORT_CHARGE_NIMH usado para activar el switch

#define PORT_SUPPLY_LION PORTD // puerto que pertenece el io para alimentar con lion
#define PIN_SUPPLY_LION 4 // pin del puerto PORT_SUPPLY_LION usado para activar el switch
#define PORT_SUPPLY_NIMH PORTB // puerto que pertenece el io para alimentar nimh
#define PIN_SUPPLY_NIMH 2 // pin del puerto PORT_SUPPLY_NIMH usado para activar el switch


// -------------------- PROTOTIPOS DE FUNCIONES --------------------------

void nimhLedOn(); // enciende el led indicador de bateria NIMH
void nimhLedOff(); // apaga el led indicador de bateria NIMH
void lionLedOn(); // enciende el led indicador de bateria LION
void lionLedOff(); // apaga el led indicador de bateria LION

void buckOn(); // enciende el buck
void buckOff(); // apaga el buck

void chargeLionOn(); // activa el switch para cargar bateria LION
void chargeLionOff(); // desactiva el switch para cargar bateria LION
void chargeNimhOn(); // activa el switch para cargar bateria NIMH
void chargeNimhOff(); // desactiva el switch para cargar bateria NIMH

void supplyLionOn(); // activa el switch para alimentar con bateria LION
void supplyLionOff(); // desactiva el switch para alimentar con bateria LION
void supplyNimhOn(); // activa el switch para alimentar con bateria NIMH
void supplyNimhOff(); // desactiva el switch para alimentar con bateria NIMH

#endif

