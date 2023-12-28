#include "controller_io_utils.h"
#include <stdint.h>

void nimhLedOn(){
    PORT_LED_NIMH |= (1 << PIN_PORT_NIMH);
}

void nimhLedOff(){
    PORT_LED_NIMH &= ~(1 << PIN_PORT_NIMH);
}

void lionLedOn(){
    PORT_LED_LION |= (1 << PIN_PORT_LION);
}

void lionLedOff(){
    PORT_LED_LION &= ~(1 << PIN_PORT_LION);
}

void buckOn(){
    PORT_BUCK_CTR &= ~(1 << PIN_PORT_BUCK_CTR);
}

void buckOff(){
    PORT_BUCK_CTR |= (1 << PIN_PORT_BUCK_CTR);
}

void chargeLionOn(){
    PORT_CHARGE_LION |= (1 << PIN_CHARGE_LION);
}

void chargeLionOff(){
    PORT_CHARGE_LION &= ~(1 << PIN_CHARGE_LION);
}

void chargeNimhOn(){
    PORT_CHARGE_NIMH |= (1 << PIN_CHARGE_NIMH);
}

void chargeNimhOff(){
    PORT_CHARGE_NIMH &= ~(1 << PIN_CHARGE_NIMH);
}

void supplyLionOn(){
    PORT_SUPPLY_LION |= (1 << PIN_SUPPLY_LION);
}

void supplyLionOff(){
    PORT_SUPPLY_LION &= ~(1 << PIN_SUPPLY_LION);
}

void supplyNimhOn(){
    PORT_SUPPLY_NIMH |= (1 << PIN_SUPPLY_NIMH);
}

void supplyNimhOff(){
    PORT_SUPPLY_NIMH &= ~(1 << PIN_SUPPLY_NIMH);
}


