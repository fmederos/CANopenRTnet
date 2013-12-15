#ifndef sync_RTnet_H_
#define sync_RTnet_H_
#include <xs1.h>

#define _DEBUG_

#define N_PUERTOS_PWM   4
#define RES_PWM         1024                    // resolución de los puertos PWM
#define GRANO_PWM       5                     // granularidad en veintenas de nS del incremento de
                                                  // ancho de pulso de la señal PWM
                                                  // grano=0 -> incremento=10nS, grano=1 -> incremento 20nS, 2->40nS, 3->60nS, etc.
                                                // 4->80ns, 5->100ns
// máscaras para registro de Control CANopen
#define MSK_CONTROL_ENCENDER    0b000000001
#define MSK_CONTROL_DESHABV     0b000000010
#define MSK_CONTROL_PARAR       0b000000100
#define MSK_CONTROL_HABOPER     0b000001000
#define MSK_CONTROL_RESET       0b010000000
#define MSK_CONTROL_PAUSA       0b100000000
// Máscaras para registro de Status CANopen
#define MSK_STATUS_LISTO        0b000000000001
#define MSK_STATUS_ENCENDIDO    0b000000000010
#define MSK_STATUS_OPERHAB      0b000000000100
#define MSK_STATUS_FALLO        0b000000001000
#define MSK_STATUS_VOLTHAB      0b000000010000
#define MSK_STATUS_PARAR        0b000000100000
#define MSK_STATUS_ENCDESHAB    0b000001000000
#define MSK_STATUS_ADVERTENCIA  0b000010000000
#define MSK_STATUS_REMOTO       0b001000000000
#define MSK_STATUS_POSDEST      0b010000000000
#define MSK_STATUS_LIMINT       0b100000000000
// Modos de operación CANopen posibles para el dispositivo
#define MODO_COPEN_VEL          3

// Máscaras para campos de trama CANopen sobre Ethernet
#define MSK_EFF_FRAME           0x1fffffff      // bits válidos cob-id en trama extendida
#define MSK_SFF_FRAME           0x000007ff      // bits válidos cob-id en trama simple
#define MSK_EFF_FLAG            0x80000000
#define MSK_RTR_FLAG            0x40000000
#define MSK_ERR_FLAG            0x20000000


// Tamaño máximo de trama Ethernet en bytes
// DEBE ser multiplo de 4 para poder transferir de a 32bits
#define ETHERNET_MAXBYTES 512

// Tamaño máximo de trama RTNet en bytes
// DEBE ser multiplo de 4 para poder transferir de a 32bits
#define RTNET_MAXBYTES  128
// Tamaño de trama normal CANopenRTnet
#define RTNET_NBYTES_COPEN      60
// cantidad máxima de slots a utilizar
#define RTNET_NSLOTS    4
// tamaño del bufferes de recepcion/trasmisión de tramas CANopen (cant.tramas)
#define RTNET_FRAME_BUFFER_SIZE 8

#define __mac_custom_filter_h_exists__ 1

// *** OJO! common.h CONTIENE CONFIGURACIONES IMPORTANTES CANopen ***

// estructura tipo para definir un slot
typedef struct rtnet_slot_t {
  int delay;                            // tiempo correspondiente al slot en decenas de nS
  int mifase;                           // fase de la cual somos propietarios
  int nfases;                           // número de fases en las que se divide el slot
} rtnet_slot_t;


// estructura tipo de entradas a la agenda de trasmision
typedef struct rtnet_agenda_t{
  unsigned int buf[RTNET_MAXBYTES / 4];    // tenemos un buffer por cada slot
  int nbytes;
  int ifnum;
  int tipo;                             // tipo de trama agendada para ser transmitida (cal.request, etc.)
#define TRAMA_RTNET_CALREQ      1
#define TRAMA_ICMP              2
#define TRAMA_COPEN             3
#define TRAMA_ARP               4
  int ciclo;                            // ciclo en el que fué trasmitido el último paquete en este slot
} rtnet_agenda_t;


#ifndef RTNET_MAX_FILTER_SIZE
#define RTNET_MAX_FILTER_SIZE 4
#endif

// *************************
// registros de estado RTnet
// *************************
extern unsigned RTnet_calibrado;              // indicación de calibración completa
extern unsigned error_status;                // estados de RTnet

//status defines
#define RTNET_STATE_ACTIVE          (0)
#define RTNET_STATE_PASSIVE         (1)
#define RTNET_STATE_BUS_OFF         (2)

//Return values
#define CAN_FILTER_ADD_SUCCESS    (0)
#define CAN_FILTER_ADD_FAIL       (1)

#define CAN_FILTER_REMOVE_SUCCESS (0)
#define CAN_FILTER_REMOVE_FAIL    (1)

#define RTNET_TX_SUCCESS            (0)
#define RTNET_TX_FAIL               (1)

#define RTNET_RX_SUCCESS            (0)
#define RTNET_RX_FAIL               (1)


void rtnet_sync(chanend tx, chanend rx, chanend c_rx_tx);



/*
typedef struct can_frame {
  unsigned remote;   //true for remote
  unsigned extended; //true for extended
  unsigned id;
  unsigned dlc;
  char data[8];
} can_frame;
*/

/*
typedef enum {
  TX_FRAME        = 0,
  TX_FRAME_NB     = 1,
  ADD_FILTER      = 2,
  REMOVE_FILTER   = 3,
  GET_STATUS      = 4,
  RESET           = 5,
  PEEK_LATEST     = 6,
  RX_BUF_ENTRIES  = 7,
  RX_FRAME        = 8
} CAN_COMMANDS;
*/


#ifdef __XC__
// declaraciones de funciones

#endif
#endif /* sync_RTnet_H_ */
