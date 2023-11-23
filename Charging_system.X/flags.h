#ifndef FLAGS_H
#define FLAGS_H

// bits de variable system_flags    CHARGING_FLAG_0: 1:corriente constante
// -------------------------------  CHARGING_FLAG_1: 1:voltaje constante
// |       CHARGING_FLAG_n       |  CHARGING_FLAG_2: 1: carga LiON 0: carga NiMH  
// -------------------------------  
// Bit 2 | Bit 1 | Bit 0 | Meaning      
// -------------------------------
//   X   |   0   |   0   |      No se esta cargando
//   0   |   0   |   1   |   Corriente constante NiMH
//   0   |   1   |   0   |    Voltaje constante NiMH
//   0   |   1   |   1   |           Reservado
//   1   |   0   |   1   |   Corriente constante LiON
//   1   |   1   |   0   |    Voltaje constante LiON
//   1   |   1   |   1   |           Reservado
// -------------------------------

#define CHARGING_FLAG_0 0 
#define CHARGING_FLAG_1 1
#define CHARGING_FLAG_2 2
#define CHARGING_FLAG   0x07 // mascara para los 3 bits de carga

#define PWR_EXT_FLAG    3 // 1: alimentacion externa 0: alimentacion bateria

// Estado de carga de baterias


// bits de estado de carga baterias

// -------------------------------     FLAG_NIMH_0: 1: Voltaje  >=6V  0: Voltaje  < 6V
// |       Estado de bateria     |     FLAG_NIMH_1: 0: Voltaje  <= 4V 1: Voltaje  > 4V
// -------------------------------
// Bit 1 | Bit 0 |  Meaning      
// -------------------------------
//   0   |   0   |    Descargada
//   0   |   1   | 4<V<6 (sin cargar)
//   1   |   0   | 4<V<6   Cargando
//   1   |   1   |     Cargada
// -------------------------------
#define FLAG_NIMH_0     4 
#define FLAG_NIMH_1     5
#define FLAG_NIMH       0x30

// -------------------------------     FLAG_NIMH_0: 1: Voltaje  >=4.2V  0: Voltaje  < 4.2V
// |       Estado de bateria     |     FLAG_NIMH_1: 1: Voltaje  > 3V    0: Voltaje  <= 3V   
// -------------------------------
// Bit 1 | Bit 0 |  Meaning      
// -------------------------------
//   0   |   0   |    Descargada
//   0   |   1   | 3<V<4.2 (sin cargar)
//   1   |   0   | 3<V<4.2   Cargando
//   1   |   1   |     Cargada
// -------------------------------
#define FLAG_LION_0     6 
#define FLAG_LION_1     7
#define FLAG_LION       0xC0 

#endif // FLAGS_H