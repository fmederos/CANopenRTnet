// Copyright (c) 2011, XMOS Ltd, All rights reserved
// This software is freely distributable under a derivative of the
// University of Illinois/NCSA Open Source License posted in
// LICENSE.txt and at <http://github.xcore.com/>


// Interfaz de capa de enlace RTnet con capa de aplicación CANopen
// ---------------------------------------------------------------
//
// Utiliza module_ethernet compilado en versión FULL para acceso a puerto Ethernet
// Utiliza module_canopen para implementación de objetos CANopen
// Utiliza partes de module_can (principalmente can_client.xc) para interfaseamiento con module_canopen
//      En particular no se utiliza can_server() de can.xc, su funcionalidad está cubierta por rtnet_sync().
//
//
//
// Etapa 1: Sincronizar con maestro RTNet
// --------------------------------------
// -Admitir las tramas RTNet al filtrado (tramas RTmac)
// -Implementar temporizador RT para manejo del TDMA
// -Armar tramas de solicitud de sincronización
// -Implementar algoritmo de sinconización, cálculo de time-shifts etc.
// -Optimizar captura de tramas en forma interruptiva en vez de poleada (usando línea RX_data_valid?)
//




/*************************************************************************
 *
 * Ethernet MAC Layer Client Test Code
 * IEEE 802.3 MAC Client
 *
 *
 *************************************************************************
 *
 * ARP/ICMP demo
 * Note: Only supports unfragmented IP packets
 *
 *************************************************************************/

#include <xs1.h>
#include <xclib.h>
#include <print.h>
#include <platform.h>
#include <stdlib.h>
#include "otp_board_info.h"
#include "ethernet.h"
#include "ethernet_board_support.h"
#include "checksum.h"
#include "xscope.h"
#include "canopen.h"
#include "mutual_thread_comm.h"
#include "xccompat.h"
#include "od.h"
#include "pwm_singlebit_port.h"
#include "qei_server.h"
//#include "qei_client.h"
//#include "qei_commands.h"
#include "spi_master.h"
#include "sync_rtnet.h"



// If you have a board with the xscope xlink enabled (e.g. the XC-2) then
// change this define to 0, make sure you also remove the -lxscope from
// the build flags in the Makefile
#define USE_XSCOPE 1


#if USE_XSCOPE
void xscope_user_init(void) {
  xscope_register(0);
  xscope_config_io(XSCOPE_IO_BASIC);
}
#endif

// Definiciones de puertos:

// Acceso a LED cuando GPIO está conectada al puerto cuadrado que va al núcleo 1 (tile 1)
//on tile[1]: out port p_led=XS1_PORT_4A;

// GPIO Module conectado al triángulo:
// -----------------------------------

// Header 20 hacia driver servo y módulo wireless conectado a P2 de la placa GPIO (puertos GPIO y GPO)
// (ver esquema eléctrico GPIO por pines de los headers)
// GPIO-0       PWM a driver servomotor
// GPIO-1
// GPIO-2
// GPIO-3
// GPO-0/LED0
// GPO-1/LED1
// GPO-2
// GPO-3
// GPO-4
// GPO-5
// GPO-6/LED2
// GPO-7/LED3

// Header 20 hacia encoder 0 conectado a P4 de la GPIO
// GPI-0        Fase A encoder 0
// GPI-1        Fase B encoder 0
// GPI-2
// GPI-3
// GPI-4
// GPI-5
// GPI-6/ButtonA
// GPI-7/ButtonB

// Pines 19 y 20 de ambos headers P2 y P4 de la GPIO suministran 3.3V y 5V resp.

// Header 20 pin en J4 de placa XMOS
// X0D1/P1B     SCK módulo wireless
// X0D10/P1C    MOSI módulo wireless
// X0D13/P1F    MISO módulo wireless
// X0D22/P1G    CSN módulo wireless


// pines GPO-0..7 del GPIO module
on tile[0]: out port p_out8=XS1_PORT_8C;
// LEDs corresponden a bits 0,1,6 y 7 de p_out8
#define MSK_LED_1     0b00000001
#define MSK_LED_2     0b00000010
#define MSK_LED_3     0b01000000
#define MSK_LED_4     0b10000000

// 2 botones y 6 pines GPI-0..5 del GPIO module
// sólo los 4 LSb son accesibles de forma individual a través de XS1_PORT_1M-P
on tile[0]: in port p_in8=XS1_PORT_8D;
// los botones corresponden a bits 0 y 1 de p_in8
#define MSK_BOTON_1     0b00000001
#define MSK_BOTON_2     0b00000010

// 4 salidas PWM GPIO_0..3 del módulo GPIO
on tile[0] : out buffered port:32 pwmPorts[] = { XS1_PORT_1A, XS1_PORT_1D, XS1_PORT_1E, XS1_PORT_1H };
// eliminamos 2 salidas PWM para poder usar pines IO para otras cosas
//on tile[0] : out buffered port:32 pwmPorts[] = { XS1_PORT_1A, XS1_PORT_1D };
on tile[0] : clock clk = XS1_CLKBLK_1;

// conexiones al módulo wireless no pueden lograrse desde placa GPIO, hay que colocar un header directo
// sobre placa XMOS y usar líneas de 1 bit que no están disponibles en la GPIO conectada el puerto triángulo
// Se puede acceder a los puertos simples correspondientes al puerto estrella: P1B, P1C, P1F, P1G (todos de tile 0)
// 2 puertos entrada desde módulo wireless

on tile[0] : out port spi_ss = XS1_PORT_1G;
// estructura interfaz del módulo SPI para comunicación con módulo wireless
spi_master_interface spi_if =
{
    on tile[0]: XS1_CLKBLK_2,
    on tile[0]: XS1_CLKBLK_3,
    on tile[0]: XS1_PORT_1C,
    on tile[0]: XS1_PORT_1B,
    on tile[0]: XS1_PORT_1F
};


//on stdcore[0] : port clockLed0 = XS1_PORT_1A;
//on stdcore[1] : port clockLed1 = XS1_PORT_1D;
//on stdcore[2] : port clockLed2 = PORT_CLOCKLED_2;
//on stdcore[3] : port clockLed3 = PORT_CLOCKLED_3;


// Módulo ETH conectado a puerto circulo (nucleo 1)
// These ports are for accessing the OTP memory
//on ETHERNET_DEFAULT_TILE: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;
on tile[1]: otp_ports_t otp_ports = OTP_PORTS_INITIALIZER;

// Here are the port definitions required by ethernet
// The intializers are taken from the ethernet_board_support.h header for
// XMOS dev boards. If you are using a different board you will need to
// supply explicit port structure intializers for these values
smi_interface_t smi = ETHERNET_DEFAULT_SMI_INIT;
mii_interface_t mii = ETHERNET_DEFAULT_MII_INIT;
ethernet_reset_interface_t eth_rst = ETHERNET_DEFAULT_RESET_INTERFACE_INIT;

//::ip_address_define
// NOTE: YOU MAY NEED TO REDEFINE THIS TO AN IP ADDRESS THAT WORKS
// FOR YOUR NETWORK
#define OWN_IP_ADDRESS {192, 168, 3, 2}
//::



// ************************************
// Definiciones para análisis de tramas
// ************************************
// Tipos de trama Ethernet (desplazamiento 12 y 13):
unsigned char ethertype_ip[] = {0x08, 0x00};
unsigned char ethertype_arp[] = {0x08, 0x06};
unsigned char ethertype_RTmac[] = {0x90, 0x21};
unsigned char ethertype_RTcfg[] = {0x90, 0x22};
unsigned char ethertype_RTCANopen[] = {0x90, 0x23};
// Luego de campo Ethertype empieza payload (para Ethernet)
//
// El payload es una trama RTNet (3 tipos definidos)
// El payload de la trama RTnet cuando lleva trama CANopen tiene la siguiente estructura:
//
// Campo de 16bits para COB-ID de los cuales sólo los 11LSb son válidos (despl.14-15 dentro de trama Ethernet)
// Campo de 24bits cuyos 2MSb son los bits SRR (substitute-remote-req.) e IDE y sus 18LSb son COB-ID-b. (despl.16-18)
// Luego un campo 8 bits cuyos 3MSb son RTR, r1 y r0 y sus 4LSb son DLC (data-length). (despl.19)
// Luego de 0 a 8 bytes de datos. Eventualmente se podrían enviar hasta 16 bytes de datos pero la
// librería actual CANopen no soporta esto.  (despl.20-...)


// Cabecera RTmac:
// --------------
// Tipos de trama RT-mac (despl. 14 y 15):
unsigned char RTmac_type_TDMA[] = {0x00, 0x01};
// Versión de trama RT-mac (despl. 16):
unsigned char RTmac_ver[] = {0x02};
// Flags de trama (despl. 17):
// bit 0 indica tunneling y el campo RTmac_type lleva el Ethertype encapsulado

// Tramas TDMA:
// -----------
// Campo versión de trama TDMA (despl. 18 y 19)
unsigned char RTmac_TDMA_type_ver[] = {0x02, 0x01};
// Campo identificación de trama TDMA (despl. 20 y 21)
unsigned char RTmac_TDMA_type_sync[] = {0x00, 0x00};
unsigned char RTmac_TDMA_type_calreq[] = {0x00, 0x10};
unsigned char RTmac_TDMA_type_calreply[] = {0x00, 0x11};
// Trama Sync.: Campo Numero de Ciclo (despl. 22 a 25)
//              Campo TX Timestamp (despl 26 a 33)
//              Campo Scheduled TX time (despl. 34 a 41)
// Trama Calib.Request.:        Campo TX Timestamp (despl 22 a 29)
//                              Campo Numero de Ciclo de Respuesta (despl. 30 a 33)
//                              Campo Offset de Slot de Respuesta (despl. 34 a 41)
// Trama Calib.Reply:   Campo Request TX Time (despl. 22 a 29)
//                      Campo RX Timestamp (despl. 30 a 37)
//                      Campo TX Timestamp (despl. 38 a 45)


// almacenamiento para MACs propia y del maestro. La del maestro es dinámica, puede cambiar si asume
// un backup-master
unsigned char own_mac_addr[6];
unsigned char master_mac_addr[6];

#define ARP_RESPONSE 1
#define ICMP_RESPONSE 2
#define UDP_RESPONSE 3


#pragma unsafe arrays
int is_ethertype(unsigned char data[], unsigned char type[]){
	int i = 12;
	return data[i] == type[0] && data[i + 1] == type[1];
}

//      Identificación de tramas RTmac
int is_RTmactype(unsigned char data[], unsigned char type[]){
        int i = 14;
        return data[i] == type[0] && data[i + 1] == type[1];
}

//      Identificación de tramas TDMA (subtipo de RTmac)
int is_TDMAtype(unsigned char data[], unsigned char type[]){
        int i = 20;
        return data[i] == type[0] && data[i + 1] == type[1];
}

#pragma unsafe arrays
int is_mac_addr(unsigned char data[], unsigned char addr[]){
	for (int i=0;i<6;i++){
          if (data[i] != addr[i]){
			return 0;
		}
	}
	return 1;
}

#pragma unsafe arrays
int is_broadcast(unsigned char data[]){
	for (int i=0;i<6;i++){
          if (data[i] != 0xFF){
			return 0;
		}
	}
	return 1;
}

//::custom-filter
// Filtramos tramas que no sean:
//      IP, ARP, RTmac, RTcfg

int mac_custom_filter(unsigned int data[]){
	if (is_ethertype((data,char[]), ethertype_arp)){
		return 1;
	}else if (is_ethertype((data,char[]), ethertype_ip)){
		return 1;
	}
	else if (is_ethertype((data,char[]), ethertype_RTmac)){
	    return 1;
        }
        else if (is_ethertype((data,char[]), ethertype_RTcfg)){
            return 1;
        }
        else if (is_ethertype((data,char[]), ethertype_RTCANopen)){
            return 1;
        }
	return 0;
}

//::

// ***********************************
// Comprobación de trama solicitud ARP
// ***********************************
int is_valid_arp_packet(const unsigned char rxbuf[], int nbytes)
{
  static const unsigned char own_ip_addr[4] = OWN_IP_ADDRESS;

  if (rxbuf[12] != 0x08 || rxbuf[13] != 0x06)
    return 0;

  printstr("ARP packet received\n");

  if ((rxbuf, const unsigned[])[3] != 0x01000608)
  {
    printstr("Invalid et_htype\n");
    return 0;
  }
  if ((rxbuf, const unsigned[])[4] != 0x04060008)
  {
    printstr("Invalid ptype_hlen\n");
    return 0;
  }
  if (((rxbuf, const unsigned[])[5] & 0xFFFF) != 0x0100)
  {
    printstr("Not a request\n");
    return 0;
  }
  for (int i = 0; i < 4; i++)
  {
    if (rxbuf[38 + i] != own_ip_addr[i])
    {
      printstr("Not for us\n");
      return 0;
    }
  }
  return 1;
}



// ************************************
// Comprobación de trama solicitud PING
// ************************************
int is_valid_icmp_packet(const unsigned char rxbuf[], int nbytes)
{
  static const unsigned char own_ip_addr[4] = OWN_IP_ADDRESS;
  unsigned totallen;
  unsigned char c;


  if (rxbuf[23] != 0x01)
    return 0;

  printstr("ICMP packet received\n");

  if ((rxbuf, const unsigned[])[3] != 0x00450008)
  {
    printstr("Invalid et_ver_hdrl_tos\n");
    return 0;
  }
  if (((rxbuf, const unsigned[])[8] >> 16) != 0x0008)
  {
    printstr("Invalid type_code\n");
    return 0;
  }
  for (int i = 0; i < 4; i++)
  {
    if (((c=rxbuf[30 + i]) != 0xFF) && (c != own_ip_addr[i]))
    {
      printstr("Not for us\n");
      return 0;
    }
  }

  totallen = byterev((rxbuf, const unsigned[])[4]) >> 16;
  if (nbytes > 60 && nbytes != totallen + 14)
  {
    printstr("Invalid size\n");
    printintln(nbytes);
    printintln(totallen+14);
    return 0;
  }
  if (checksum_ip(rxbuf) != 0)
  {
    printstr("Bad checksum\n");
    return 0;
  }

  return 1;
}



// *********************************************************
// Arma trama ARP para indicar cuál es nuestra dirección MAC
// *********************************************************
int build_arp_response(unsigned char rxbuf[], unsigned int txbuf[], const unsigned char own_mac_addr[6])
{
  unsigned word;
  unsigned char byte;
  const unsigned char own_ip_addr[4] = OWN_IP_ADDRESS;

  for (int i = 0; i < 6; i++)
    {
      byte = rxbuf[22+i];
      (txbuf, unsigned char[])[i] = byte;
      (txbuf, unsigned char[])[32 + i] = byte;
    }
  word = (rxbuf, const unsigned[])[7];
  for (int i = 0; i < 4; i++)
    {
      (txbuf, unsigned char[])[38 + i] = word & 0xFF;
      word >>= 8;
    }

  (txbuf, unsigned char[])[28] = own_ip_addr[0];
  (txbuf, unsigned char[])[29] = own_ip_addr[1];
  (txbuf, unsigned char[])[30] = own_ip_addr[2];
  (txbuf, unsigned char[])[31] = own_ip_addr[3];

  for (int i = 0; i < 6; i++)
  {
    (txbuf, unsigned char[])[22 + i] = own_mac_addr[i];
    (txbuf, unsigned char[])[6 + i] = own_mac_addr[i];
  }
  txbuf[3] = 0x01000608;
  txbuf[4] = 0x04060008;
  (txbuf, unsigned char[])[20] = 0x00;
  (txbuf, unsigned char[])[21] = 0x02;

  // Typically 48 bytes (94 for IPv6)
  for (int i = 42; i < 64; i++)
  {
    (txbuf, unsigned char[])[i] = 0x00;
  }

  return 64;
}



// *******************
// Arma respuesta PING
// *******************
int armar_respuesta_icmp(unsigned char rxbuf[], unsigned char txbuf[], const unsigned char own_mac_addr[6])
{
  static const unsigned char own_ip_addr[4] = OWN_IP_ADDRESS;
  unsigned icmp_checksum;
  int datalen;
  int totallen;
  const int ttl = 0x40;
  int pad;

  // Precomputed empty IP header checksum (inverted, bytereversed and shifted right)
  unsigned ip_checksum = 0x0185;

  // Copiamos source MAC de la trama recibida al dest.MAC
  for (int i = 0; i < 6; i++)
    {
      txbuf[i] = rxbuf[6 + i];
    }
  // copiamos source IP addr. de trama recibida a dest.IP address
  for (int i = 0; i < 4; i++)
    {
      txbuf[30 + i] = rxbuf[26 + i];
    }

  icmp_checksum = byterev((rxbuf, const unsigned[])[9]) >> 16;

  // copiamos campos length y checksum de trama ICMP
  for (int i = 0; i < 4; i++)
    {
      txbuf[38 + i] = rxbuf[38 + i];
    }

  totallen = byterev((rxbuf, const unsigned[])[4]) >> 16;
  datalen = totallen - 28;
  // copiamos payload ICMP (suceción ASCII)
  for (int i = 0; i < datalen; i++)
    {
      txbuf[42 + i] = rxbuf[42+i];
    }

  // colocamos source MAC addr.
  for (int i = 0; i < 6; i++)
  {
    txbuf[6 + i] = own_mac_addr[i];
  }
  // llenamos campo ethertype (0800->trama IP), versión de trama (4) y tamaño de cabecera (5 bytes)
  (txbuf, unsigned[])[3] = 0x00450008;
  // llenamos campo largo de payload
  totallen = byterev(28 + datalen) >> 16;
  (txbuf, unsigned[])[4] = totallen;
  ip_checksum += totallen;
  // campo Flags y Fragment Offset quedan en 0, campo TTL y protocol ID hay que llenarlos
  // TTL serían bits 16..23, protocol del 24 al 31 del 5to unsigned (32 bits c/u)
  // protocolo ICMP tiene código 0x01, ttl lo tenemos como constante predefinida
  (txbuf, unsigned[])[5] = 0x01000000 | (ttl << 16);
  (txbuf, unsigned[])[6] = 0;
  // colocamos source IP address
  for (int i = 0; i < 4; i++)
  {
    txbuf[26 + i] = own_ip_addr[i];
  }
  // actualizamos checksum
  ip_checksum += (own_ip_addr[0] | own_ip_addr[1] << 8);
  ip_checksum += (own_ip_addr[2] | own_ip_addr[3] << 8);
  ip_checksum += txbuf[30] | (txbuf[31] << 8);
  ip_checksum += txbuf[32] | (txbuf[33] << 8);

  txbuf[34] = 0x00;
  txbuf[35] = 0x00;

  icmp_checksum = (icmp_checksum + 0x0800);
  icmp_checksum += icmp_checksum >> 16;
  txbuf[36] = icmp_checksum >> 8;
  txbuf[37] = icmp_checksum & 0xFF;

  while (ip_checksum >> 16)
  {
    ip_checksum = (ip_checksum & 0xFFFF) + (ip_checksum >> 16);
  }
  ip_checksum = byterev(~ip_checksum) >> 16;
  txbuf[24] = ip_checksum >> 8;
  txbuf[25] = ip_checksum & 0xFF;

  // rellenamos si necesario hasta completar la trama de 64 bytes
  for (pad = 42 + datalen; pad < 64; pad++)
  {
    txbuf[pad] = 0x00;
  }
  return pad;
}



// ********************************************************************************
// Armado de trama RTNet TDMA para solicitud de calibración de tiempo de transporte
//
// Entrada:
//      -Buffer conteniendo trama sync del maestro para obtener dir.MAC del maestro.
//      -Buffer de transmisión con al menos 50bytes
//      -Dirección MAC de este esclavo
//      -Offset en nS del slot donde se desea recibir la respuesta
// ********************************************************************************
int armar_trama_cal_req(unsigned char rxbuf[], unsigned int txbuf[], const unsigned char own_mac_addr[6], unsigned long offset)
{
  unsigned long long ull;

  // Copiamos source MAC de la trama recibida al dest.MAC
  for (int i = 0; i < 6; i++)
    {
      (txbuf, unsigned char[])[i] = rxbuf[6 + i];
    }
  // colocamos source MAC addr.
  for (int i = 0; i < 6; i++)
  {
    (txbuf, unsigned char[])[6 + i] = own_mac_addr[i];
  }

  // llenamos campo ethertype (0x9021: trama RTmac) y RTmacType (0x0001: trama TDMA)
  txbuf[3] = 0x01002190;
  // llenamos campo versión RTmac (0x02), Flags (0x00), y TDMA version (0x0201)
  txbuf[4] = 0x01020002;
  // llenamos campo TDMA type (0x0010: Calibration Request)
  (txbuf, unsigned char[])[20] = 0x00;
  (txbuf, unsigned char[])[21] = 0x10;

  // campo txTimeStamp. son 8 bytes que van del 22 al 29, dejamos este campo vacío.

  // el campo reply-cycle-num va del 30 al 33 y también queda vacío

  // llenamos campo slot-offset de 8 bytes entre el 34 al 41
  // el valor debe ir en nS y nosotros tenemos offset en decenas de nS
  ull = offset * 10;
  for(int i=0; i<8 ; i++){
      (txbuf, unsigned char[])[34+i] = (ull,unsigned char[])[7-i];
  }
  // rellenamos con ceros desde final de trama calreq hasta completar 60 bytes, el PHY agrega los 4 de checksum
  for(int i=42; i<60 ; i++){
      (txbuf, unsigned char[])[i] = 0;
  }

  return 60;
}





// **************************************************************************
// Extrae payload CANopen de trama Ethernet y la eleva a la hebra del server CANopen
//
// La trama CANopen sobre CAN lleva el formato del tipo can_frame definido en can.h del
// módulo CAN de XMOS
//
// El empaquetado de trama CANopen en Ethernet sebe seguir el formato del tipo can_frame de Xenomai
// sefinido en el archivo rtcan.h
// La trama ethernet se le asigna el Ethertype 9023h como si fuese un sub-tipo de trama RTnet.
// El tipo can_frame de Xenomai especifica 3 secciones:
// 1) Campo de 32bits (bytes 14..17 de trama ethernet)
// Se utilizan como máximo los 29LSb (least-significant-bits) (bits 0..28) para COB-id,
// el bit 31 indica EFF (extended-frame-format) y el bit 30 indica RTR (return-request) y el bit
// 29 indica ERR.
// Si el frame no es EFF (no tiene activo el bit 31 del primer campo) entonces sólo se cuentan
// para el COB-id los 11LSb.
// 2) Campo de 8 bits (byte 18 de trama eth.)
// Para el DLC (data-length en bytes) de 0 hasta 8 como máximo para mantener compatibilidad con el formato can_frame
// 3) Campo variable de hasta 8 bytes para datos (bytes 19 .. 26 de trama eth.)
//
// El formato en que se envía la trama ala hebra server CANopen es el definido por el tipo can_frame del módulo
// CAN estándard de XMOS en el archivo can.h
// **************************************************************************
void
elevar_trama_canopen(chanend server, unsigned char rxbuf[])
{
  unsigned u;

  // armamos una trama CAN a partir de lo que trae la trama Ethernet
  can_frame f;
  // extraemos los 16LSb de COB-id de los bytes 14-15
  f.id = byterev((rxbuf, const unsigned[])[3] & 0xffff0000);
  f.id |= byterev((rxbuf, const unsigned[])[4] & 0x0000ff1f);
  // extraemos flag extended
  f.extended = ((rxbuf, const unsigned[])[3] & 0x00000080) >> 7;
  // extraemos el bit RTR
  f.remote = ((rxbuf, const unsigned[])[3] & 0x00000040) >> 6;
  // extraemos el data length
  f.dlc = rxbuf[18];
  // extraemos los 8 bytes de datos
  for(int i=0; i < f.dlc; i++){
      f.data[i] = rxbuf[19+i];
  }
  // pasamos la trama CAN armada al server CANopen
  slave {
    server <: f.remote;                    // RTR Remote Transmission Request (unsigned)
    server <: f.extended;                  // IDE Identificator Extension (unsigned)
    server <: f.id;                        // COB-ID (unsigned)
    server <: f.dlc;                       // data length (unsigned)
    server <: (f.data, unsigned[])[0];    // 4 Bytes
    server <: (f.data, unsigned[])[1];    // 4 bytes entre ambos char[8]
  }
}



// ************************************************************************************
// Recibe del servidor CANopen una trama CAN y arma una trama Ethernet a partir de ésta
//
// El formato en que es recibida la trama CAN es el mismo formato en que son elevadas
// las tramas al server por la función elevar_trama_canopen(), utilizando el tipo can_frame del
// módulo CAN estándar de XMOS en el archivo can.h
//
// El formato en que se empaqueta la trama CANopen en Ethernet es el establecido por el tipo can_frame de
// Xenomai en las definiciones de rtcan.h y con el ethertype 0x9023 como un sub-tipo de trama RTnet:
// 1) Campo de 32bits (bytes 14..17 de trama ethernet)
// Se utilizan como máximo los 29LSb (least-significant-bits) (bits 0..28) para COB-id,
// el bit 31 indica EFF (extended-frame-format) y el bit 30 indica RTR (return-request) y el bit
// 29 indica ERR.
// Si el frame no es EFF (no tiene activo el bit 31 del primer campo) entonces sólo se cuentan
// para el COB-id los 11LSb.
// 2) Campo de 8 bits (byte 18 de trama eth.)
// Para el DLC (data-length en bytes) de 0 hasta 8 como máximo para mantener compatibilidad con el formato can_frame
// 3) Campo variable de hasta 8 bytes para datos (bytes 19 .. 26 de trama eth.)
//
// Solamente verifica RTNET_MAXBYTES para controlar el tamaño de la trama
// pasada al buffer.
// Devuelve el número de bytes escritos en el buffer
//
// La idea es copiar el buffer devuelto por esta función al campo
// deseado de slot_agenda[] según el tipo de trama CANopen.
//
// Esta función no escribe directamente en la agenda de transmisión.
//
// Como resultado de esta función se devuelve el nro de bytes pasados al buffer
//
// Para seleccionar slot en el cual trasmitir esta trama se debería observar
// los 4MSb del COB-id
// El orden en que figuran aquí es según la preferencia que deberían tener (aprox.) para la selección del slot.
//      EMCY :        0001
//      PDO:          0011, 0100, 0101, 0110, 0111, 1000, 1001, 1010
//      SDOrx :       1011
//      SDOtx :       1100
//      NMT err.ctrl: 1110
//      NMT :         0000
//      SYNC :        0001
//      TIME STAMP :  0010
// **************************************************************************
unsigned
bajar_trama_canopen(chanend server, unsigned char txbuf[])
{
  unsigned u,i;
  can_frame f;
  // recibimos la trama CAN del servidor CANopen
  slave {
    server :> f.remote;
    server :> f.extended;
    server :> f.id;
    server :> f.dlc;
    server :> (f.data, unsigned[])[0];
    server :> (f.data, unsigned[])[1];
  }
  // encapsulamos la trama CANopen en una Ethernet a partir del byte 14 (donde empieza el payload Ethernet)

  // ponemos mac del maestro en campo MAC destino
  (txbuf, unsigned[])[0] = (master_mac_addr, unsigned[])[0];
  (txbuf, unsigned short[])[2] = (master_mac_addr, unsigned short[])[2];
  // ponemos mac propia en campo MAC origen, tenemos que usar cast a short porque vienen corridos los campos
  (txbuf, unsigned short[])[3] = (own_mac_addr, unsigned short[])[0];
  (txbuf, unsigned short[])[4] = (own_mac_addr, unsigned short[])[1];
  (txbuf, unsigned short[])[5] = (own_mac_addr, unsigned short[])[2];

  // armamos los 32 bits para el 1er campo con cob-id y flags EFF (IDE) y RTR
  u = (f.id & 0x1fffffff);
  if(f.extended)
    u |= MSK_EFF_FRAME;
  if(f.remote)
    u |= MSK_RTR_FLAG;

  // pasamos los 16LSb del id a los bytes 14-15
  // y la designación Ethertype a los bytes 12-13
  (txbuf, unsigned[])[3] = byterev(byterev((ethertype_RTCANopen, short)) | (u & 0xffff0000));
  // pasamos los 13MSb del id a bytes 16-17 y el dlc al byte 18
  (txbuf, unsigned[])[4] = byterev((u & 0x0000ffff)<<16 | (f.dlc << 8));

  // pasamos los 8 bytes de datos a partir del byte 19 del buffer de tx
  for(u=0;u<f.dlc;u++){
      txbuf[19+u] = f.data[u];
  }

  // rellenamos con 0x00 el resto de payload
  for (u+=19; u < RTNET_NBYTES_COPEN; u++)
  {
    txbuf[u] = 0x00;
  }

  return u;

}



// ***********************************************************************************************
// Sincronización con tramas RTnet para trasmisión de paquetes dentro de time-slots asignados
// ***********************************************************************************************
void rtnet_sync(chanend tx, chanend rx, chanend c_rx_tx)
{
  // bufferes de recepción/tranmsisión. son arrays de integers (32bits) y quiero que tengan 1600 bytes para
  // poder recibir sin problemas cualquier trama Ethernet
  unsigned int rxbuf[ETHERNET_MAXBYTES/4];
  unsigned int txbuf[ETHERNET_MAXBYTES/4];
  // buffer para tramas CANopen, 128bytes deberían sobrar para cualquier trama CANopen
  unsigned rxbuf_copen[RTNET_FRAME_BUFFER_SIZE][RTNET_MAXBYTES/4];
  // buffer TX tiene 1 int más para poner tamaño de trama (para no trasmitir siempre los 128 bytes)
  unsigned txbuf_copen[RTNET_FRAME_BUFFER_SIZE][RTNET_MAXBYTES/4+1];
  // punteros para utilizar buffers:
  // head-1 es donde se escribió por última vez en el buffer (al escribir lo hacemos en head e incrementamos)
  // tail es la entrada más vieja, o sea donde deberíamos leer el buffer
  unsigned rx_buffer_head;
  unsigned rx_buffer_tail;
  unsigned tx_buffer_head;
  unsigned tx_buffer_tail;
  // estado inicial
  unsigned Bus_Status = RTNET_STATE_BUS_OFF;
  unsigned transmit_error_counter = 0;
  unsigned receive_error_counter = 0;
  unsigned message_filter_count=0;
  unsigned tx_enabled = 0;

  // temporizador para registrar detección de tramas
  timer temporizador;
  // arreglo con la especificación de los slots que tenemos disponibles para transmitir
  rtnet_slot_t slot[RTNET_NSLOTS];
  // agenda de uso para los slots. Cada entrada especifica una tarea para transmitir en el slot correspondiente.
  rtnet_agenda_t slot_agenda[RTNET_NSLOTS];
  // tiempo de deteccion de última trama sync
  unsigned int t_sync;
  unsigned int t_trans=2000;               // tiempo en decenas de nS de demora desde que el RTNet maestro
                                          // intenta enviar una trama hasta que ella es detectada por este esclavo.
                                          // este tiempo incluye el tiempo de tránsito en el cable, el tiempo de
                                          // armado y desarmado de la trama en los chips PHY y las latencias en los
                                          // interfaces recorridos (PCI, MII, etc.)
  unsigned int n_cal_t_trans=100;       // rondas de calibracion de t_transmision
  unsigned int n_acum_t_trans=0;          // contador de medidas de t_transmision
  unsigned int t_trans_acum=0;            // acumulador de medidas de t_transmision
  unsigned RTnet_calibrado=0;                 // indicación de calibración completa

  // registros de 64 bits time-stams del maestro
  unsigned long long reqrxTime;         // time-stamp del maestro al recibir trama de solicitud de calibración
  unsigned long long reptxTime;         // time-stamp del maestro al enviar trama respuesta de calibración

  unsigned int nCiclo=0;          // Contador de ciclos de control RTNet
  unsigned int nCiclo_ant=0;
  unsigned int puerto=0xf;          // latch para el puerto GPIO, led prende con un 0

  // variables de uso gral
  int i,j,k;
  unsigned long long ull;
  unsigned int t;
  unsigned int timeout;

  // estructuras para arbitraje de intercambio por canales
  mutual_comm_state_t mstate;
  int is_response_to_notification;
  int is_data_request;

  // Configuración manual de slots
  // delay es tiempo en decenas de ns de cada uno de nuestros slots contando desde detectada la trama sync
  // esto debe estar en concordancia con la configuración del maestro RTNet (/usr/rtnet/etc/rtnet.conf y tdma.conf)
  // aquí asumimos que poseemos el 2do, el 4to, 6to y 7mo slots y que  cada slot dura 100uS.
  // el 6to y 7mo slot es compartido entre 3 esclavos y poseemos sólo 1 de las 3 fases de cada uno de ellos
  // el 2do y 4to los poseemos por completo
  // Para el correcto funcionamiento de la rutina rtnet_sync los slots deben estar asignados con delays en aumento,
  // p.ej. el slot 3 no puede tener un delay menor al del slot 2, etc.
  // además debe haber varios uS de tiempo libre entre tramas (20uS deberían sobrar)
  // *** mifase va de 0 hasta (nfases-1) ***
  slot[0].delay = 200*100;
  slot[0].mifase = 0;
  slot[0].nfases = 1;
  slot[1].delay = 400*100;
  slot[1].mifase = 0;
  slot[1].nfases = 1;
  slot[2].delay = 600*100;
  slot[2].mifase = 1;
  slot[2].nfases = 3;
  slot[3].delay = 700*100;
  slot[3].mifase = 2;
  slot[3].nfases = 3;

  // inicializamos agenda de transmisión
  for(i=0;i < RTNET_NSLOTS;i++){
      slot_agenda[i].nbytes=0;
      slot_agenda[i].tipo=0;
  }

  //::get-macaddr
  mac_get_macaddr(tx, own_mac_addr);
  //::

  //::setup-filter
  mac_set_custom_filter(rx, 0x1);

  // Inicializamos comunicacion con la hebra CANopen
  mutual_comm_init_state(mstate);

  // TODO RTnet: es requerido inicializar el canal?
  //mutual_comm_notify(c_rx_tx, mstate);
  //mutual_comm_transaction(c_rx_tx, is_data_request, mstate);
  //mutual_comm_complete_transaction(c_rx_tx, is_data_request, mstate);


  // Comunicaciones desactivadas por ahora
  //Bus_Status = RTNET_STATE_PASSIVE;
  tx_enabled = 0;
  transmit_error_counter = 0;
  receive_error_counter = 0;
  rx_buffer_head = 0;
  rx_buffer_tail = 0;
  tx_buffer_head = 0;
  tx_buffer_tail = 0;
  message_filter_count=0;

  printstr("Inicio sincronizacion...\n");
  while (1)
  {
    unsigned int src_port;
    unsigned int nbytes;
    unsigned int rxTime_Stamp;          // registro del instante de llegada de la trama marcado por el PHY
    unsigned int rxTime;                // registro del momento de obtención de la trama (luego de la transfer.
                                          // PHY->XMOS y luego del filtrado)
    unsigned int reqtxTime;             // contenedor para campo Calib-Request-TX-TimeStamp recibido desde el maestro
                                          // en cada trama cal-reply

    // ******************************
    // Esperamos llegada de una trama
    // esta función bloquea en espera de una trama
    // ******************************
    // Usamos la función safe...timed para controlar el máximo número de tramas y registrar el time-stamp
    safe_mac_rx_timed(rx, (rxbuf,char[]), nbytes, rxTime_Stamp, src_port, RTNET_MAXBYTES);

    // TODO RTNET: obtener time-stamp directamente del servicio mii en lugar de aquí (está disponible en rxTime_Stamp)
    // tomamos tiempo en que obtenemos la trama (pasó tiempo el PHY, tiempo de transmisión por MII y filtrado)
    temporizador :> rxTime;

    // Análisis de tramas recibidas

    // ***      Comprobación trama RTmac      ***
    if (is_ethertype((rxbuf,char[]), ethertype_RTmac)){
        // Si recibimos trama sync establecemos referencia para nuestro slot reservado
        if(is_TDMAtype((rxbuf,char[]), RTmac_TDMA_type_sync)){
            // registramos el tiempo de detección de la trama sync
            // esto servirá de referencia para la temporización de las tramas salientes
            t_sync = rxTime;
            // guardamos nro. de ciclo para luego saber si perdimos alguno
            nCiclo_ant = nCiclo;
            // Trama Sync.: Campo Numero de Ciclo (despl. 22 a 25)
            // Obtenemos nro. de ciclo
            nCiclo = (rxbuf[6] & 0x0000ffff) << 16;
            nCiclo |= (rxbuf[5] & 0xffff0000) >> 16;
            nCiclo = byterev(nCiclo);

            // guardamos la MAC del Maestro (puede cambiar si asume un secondary-master)
            (master_mac_addr, unsigned short[])[0] = (rxbuf, unsigned short[])[3];
            (master_mac_addr, unsigned short[])[1] = (rxbuf, unsigned short[])[4];
            (master_mac_addr, unsigned short[])[2] = (rxbuf, unsigned short[])[5];

            /*
            // cada 1024 tramas sync notificamos
            if(0 == (nCiclo & 255)){
                // cambiamos estado del LED1
                puerto ^= 0b0010;
                p_led <: puerto;

              //printstr("llegaron");
              //printstr(" otras 1024 tramas sync...\n");
            }
            */
        }
        // Si recibimos trama respuesta de calibración tomamos dato de t_transmision y promediamos
        else if(is_TDMAtype((rxbuf,char[]), RTmac_TDMA_type_calreply)){
            // la calibración del tiempo de transmision la hacemos a partir de 4 time-stamps:
            // 1-instante de envío del request medido por este esclavo
            // 2-instante de llegada del request medido por el maestro
            // 3-instante de salida del reply medido por el maestro
            // 4-instante de llegada del reply medido por el esclavo
            //
            // asumimos dos cosas:      -el reloj del maestro y el del esclavo están en las mismas unidades
            //                          -el reloj del maestro y del esclavo están desfasados por una cantidad fija
            // de esta forma podemos calcular el tiempo de demora entre el envío y la llegada de un paquete:
            // t_transmision = ((T4-T1)-(T3-T2))/2

            // reqtxTime es repetida en la reply-calib. por el maestro, son 8 bytes pero los 4 mas significativos
            // deberían ser cero ya que el esclavo así los envió...
            // Ojo que este timestamp viene en decenas de nS, el resto vienen en nS!!!
            reqtxTime = (rxbuf[29/4] & 0x0000ffff) << 16;
            reqtxTime |= (rxbuf[27/4] & 0xffff0000) >> 16;
            reqtxTime = byterev(reqtxTime);
            //request-rx-time son 8 bytes puestos por el maestro en los desplazamientos 30 a 37
            reqrxTime = (unsigned long long)byterev((rxbuf[36/4] & 0x0000ffff)) >> 16;
            reqrxTime |= (unsigned long long)byterev(rxbuf[32/4]) << 16;
            reqrxTime |= (unsigned long long)byterev(rxbuf[28/4] & 0xffff0000) << 48;
            //reply-tx-time son 8 bytes puestos por el maestro en los desplazamientos 38 a 45
            reptxTime = (unsigned long long)byterev((rxbuf[44/4] & 0x0000ffff)) >> 16;
            reptxTime |= (unsigned long long)byterev(rxbuf[40/4]) << 16;
            reptxTime |= (unsigned long long)byterev(rxbuf[36/4] & 0xffff0000) << 48;

            // TODO RTNET: en vez de esperar hasta completar las 100 muestras podríamos ya tomar la primera
            // para no seguir con defasajes grandes también en las próximas 99 rondas...
            // acumulamos otra muestra del tiempo total (ida y vuelta)
            // los tiempos del maestro vienen en nS, los del esclavo en decenas de nS
            t_trans_acum += ((rxTime - reqtxTime)-(reptxTime - reqrxTime)/10);

            // vemos si completamos las muestras
            if(++n_acum_t_trans == n_cal_t_trans){
              // habíamos acumulado tiempo de ida y vuelta, queremos tener en t_trans sólo el de ida hacia el esclavo
              t_trans = (t_trans_acum / n_cal_t_trans) / 2;
              // borramos el flag que usamos para indicar que estamos calibrando
              slot_agenda[0].tipo = 0;
              // e indicamos que completamos la calibración
              RTnet_calibrado=1;
              // ***
              // Activamos CANopen
              // ***
              tx_enabled = 1;
              Bus_Status = RTNET_STATE_PASSIVE;
            }
            // vemos si corresponde solicitar otra ronda de calibración
            else if(n_acum_t_trans < n_cal_t_trans){
                // agendamos para transmitir en el próximo slot 0 una trama cal-request
                nbytes = armar_trama_cal_req((rxbuf, char[]), slot_agenda[0].buf, own_mac_addr, slot[0].delay);
                slot_agenda[0].nbytes = nbytes;
                // usamos el mismo source port por donde llegan las tramas sync
                slot_agenda[0].ifnum = src_port;
                // marcamos que es calreq para que lean llenos los campos nciclo-reply y req-txtimestamp
                slot_agenda[0].tipo = TRAMA_RTNET_CALREQ;
            }
        }
    }

    // ********************************************
    // ***      Comprobación trama CANopen      ***
    // ********************************************
    else if (is_ethertype((rxbuf,char[]), ethertype_RTCANopen)){
        // Recibimos una trama CANopen encapsulada -> la pasamos al server CANopen
        // notificamos que queremos enviar al master (esta función es slave)
        mutual_comm_notify(c_rx_tx, mstate);
        // para no perder la trama, tenemos que guardarla a la cabeza del buffer de recepción.
        // copiamos sólo los primeros RTNET_MAXBYTES
        j = nbytes/4;
        if(j > RTNET_MAXBYTES/4)
          j = RTNET_MAXBYTES/4;
        for(int i=0;i<j;i++){
            rxbuf_copen[rx_buffer_head][i] = rxbuf[i];
        }
        // llenamos una posición del buffer así que incrementamos puntero
        rx_buffer_head++;
        // aseguramos no escaparnos del rango
        rx_buffer_head = rx_buffer_head % RTNET_FRAME_BUFFER_SIZE;
        if(rx_buffer_head == rx_buffer_tail){
            // llenamos el buffer de recepción, tendremos que descartar la trama más vieja
            rx_buffer_tail ++;
            rx_buffer_tail = rx_buffer_tail % RTNET_FRAME_BUFFER_SIZE;
        }
    }

    // ***      Comprobación trama ARP      ***
    else if (is_valid_arp_packet((rxbuf,char[]), nbytes)){
        // vamos a responder la solicitud de resolución en un slot de baja prioridad
        // buscamos slot libre para agendar empezando en el slot 1
        for(i=1;i<RTNET_NSLOTS;i++){
            // al encontrar un slot vacío cortamos el FOR
            if(!slot_agenda[i].nbytes) break;
        }
        // si el FOR salió antes de terminar es que encontró slot libre...
        // si no encontramos slot libre entonces la trama quedará en el buffer
        if(i != RTNET_NSLOTS){
            // agendamos la trama en el slot (j) que encontramos libre
            // armamos respuesta con trama recibida como ejemplo (source IP, etc.)
            nbytes = build_arp_response((rxbuf,char[]), (slot_agenda[i].buf, unsigned int[]), own_mac_addr);
            slot_agenda[i].nbytes = nbytes;
            // usamos el mismo source port por donde llegan las tramas sync
            slot_agenda[i].ifnum = src_port;
            // marcamos que es ICMP
            slot_agenda[i].tipo = TRAMA_ARP;
        }
        //mac_tx(tx, txbuf, nbytes, ETH_BROADCAST);
        printstr("ARP response sent\n");
    }

    // ***      Comprobación trama solicitud PING      ***
    else if (is_valid_icmp_packet((rxbuf,char[]), nbytes)){
        // vamos a responder el PING en un slot de baja prioridad
        // buscamos slot libre para agendar empezando en el slot 1
        for(i=1;i<RTNET_NSLOTS;i++){
            // al encontrar un slot vacío cortamos el FOR
            if(!slot_agenda[i].nbytes) break;
        }
        // si el FOR salió antes de terminar es que encontró slot libre...
        // si no encontramos slot libre entonces la trama quedará en el buffer
        if(i != RTNET_NSLOTS){
            // agendamos la trama en el slot (j) que encontramos libre
            // armamos respuesta con trama recibida como ejemplo (source IP, etc.)
            nbytes = armar_respuesta_icmp((rxbuf,char[]), (slot_agenda[i].buf, unsigned char[]), own_mac_addr);
            slot_agenda[i].nbytes = nbytes;
            // usamos el mismo source port por donde llegan las tramas sync
            slot_agenda[i].ifnum = src_port;
            // marcamos que es ICMP
            slot_agenda[i].tipo = TRAMA_ICMP;
        }
    }



    // ***
    // Chequeamos si la hebra CANopen está pronta para recibir o tiene algo para trasmitir:
    // ***
    #pragma ordered
    select{
      case mutual_comm_transaction(c_rx_tx, is_response_to_notification, mstate): {
        // vemos si la transacción se inicia como respuesta a una notificación de ésta hebra
        if (is_response_to_notification) {
            // ***
            // si, servidor CANopen está en espera para recibir
            // ***
            // Contamos las tramas que tenemos para trasmitir
            unsigned count = (rx_buffer_head - rx_buffer_tail);
            unsigned buf_tail_index;
            // Si tenemos más tramas que el tamaño del buffer descartamos las más viejas
            if(count > RTNET_FRAME_BUFFER_SIZE)
              rx_buffer_tail = rx_buffer_head - RTNET_FRAME_BUFFER_SIZE;
            // Aseguramos tener el indice dentro del rango del buffer
            buf_tail_index = rx_buffer_tail % RTNET_FRAME_BUFFER_SIZE;
            // pasamos la trama más vieja en la cola a la hebra CANopen
            elevar_trama_canopen(c_rx_tx, (rxbuf_copen[buf_tail_index], char[]));
            // actualizamos indice de trama más vieja
            rx_buffer_tail++;
            // finaliza la transacción
            mutual_comm_complete_transaction(c_rx_tx,
                            is_response_to_notification,
                            mstate);
            // si habían más de 1 tramas para enviar, notificamos devuelta para solicitar otra transacción
            if(count>1)
              mutual_comm_notify(c_rx_tx, mstate);
        }
        else {
          // ***
          // no, la transacción NO se inicia en respuesta a nuestra solicitud. REcibiremos un comando.
          // ***
          int e, id;
          unsigned char cmd;
          // recibimos el comando
          c_rx_tx :> cmd;
          //cmd = 999999;
          switch(cmd){
            case TX_FRAME:
            case TX_FRAME_NB:{
              // ***
              // comando para transmitir trama al bus
              // ***
              // recibimos la trama y la pasamos a la agenda de transmisión.
              e=bajar_trama_canopen(c_rx_tx, (txbuf_copen[tx_buffer_head], char[]));
              // en el último elemento del buffer pongo el tamaño de trama
              txbuf_copen[tx_buffer_head][RTNET_MAXBYTES/4] = e;

              tx_buffer_head++;
              tx_buffer_head = tx_buffer_head % RTNET_FRAME_BUFFER_SIZE;
              // si llenamos el buffer tendremos que borrar la trama más vieja
              if(tx_buffer_head == tx_buffer_tail){
                  tx_buffer_tail ++;
                  tx_buffer_tail = tx_buffer_tail % RTNET_FRAME_BUFFER_SIZE;
              }
              // si el comando así lo especifica reportamos resultado
              if(cmd == TX_FRAME){
                if(Bus_Status == RTNET_STATE_BUS_OFF)
                  c_rx_tx <: RTNET_TX_FAIL;
                else
                  c_rx_tx <: RTNET_TX_SUCCESS;
              }
              break;
            }
            case PEEK_LATEST:{
              // ***
              // comando leer la trama más vieja sin eliminarla del buffer
              // ***
              // calculamos la tramas que tenemos en buffer
              unsigned count = (rx_buffer_head - rx_buffer_tail);
              unsigned buf_tail_index;
              // aseguramos no pasarnos del límite impuesto al tamaños del buffer
              if(count > RTNET_FRAME_BUFFER_SIZE)
                rx_buffer_tail = rx_buffer_head - RTNET_FRAME_BUFFER_SIZE;
              buf_tail_index = rx_buffer_head % RTNET_FRAME_BUFFER_SIZE;
              // pasamos la hebra CANopen el número de tramas en buffer
              c_rx_tx <: count;
              // si hay algo pasamos lo mas viejo
              if(count){
                  elevar_trama_canopen(c_rx_tx, (rxbuf_copen[buf_tail_index], char[]));
              }
              break;
            }
            case RX_BUF_ENTRIES:{
              // ***
              // comando obtener cantidad de tramas en el buffer
              // ***
              // calculamos la tramas que tenemos en buffer
              unsigned count = (rx_buffer_head - rx_buffer_tail);
              if(count > RTNET_FRAME_BUFFER_SIZE)
                rx_buffer_tail = rx_buffer_head - RTNET_FRAME_BUFFER_SIZE;
              count = (rx_buffer_head - rx_buffer_tail);
              c_rx_tx <: count;
              break;
            }

  /*
   *      Filtros no implementados.
   *      Hay que ver como se haría esto sobre RTnet
   *
            case ADD_FILTER:{
              unsigned filter_id;
              c_rx_tx :> filter_id;
              if(message_filter_count < CAN_MAX_FILTER_SIZE){
                message_filters[message_filter_count] = filter_id;
                message_filter_count++;
                c_rx_tx <: CAN_FILTER_ADD_SUCCESS;
              } else {
                c_rx_tx <: CAN_FILTER_ADD_FAIL;
              }
              break;
            }
            case REMOVE_FILTER:{
              unsigned filter_id;
              unsigned index=0;
              unsigned found=0;
              c_rx_tx :> filter_id;
              for(index=0;index<message_filter_count;index++){
                if(message_filters[index] == filter_id){
                  found = 1;
                  break;
                }
              }

              if(found){
                for(unsigned i=index;i<message_filter_count;i++){
                  if(i+1<CAN_MAX_FILTER_SIZE)
                    message_filters[i] = message_filters[i+1];
                }
                message_filter_count--;
                c_rx_tx <: CAN_FILTER_REMOVE_SUCCESS;
              } else {
                c_rx_tx <: CAN_FILTER_REMOVE_FAIL;
              }

              break;
            }
  */

            case GET_STATUS:{
              c_rx_tx <: Bus_Status;
              break;
            }
            case RESET:{
              int is_data_request;
              mutual_comm_complete_transaction(c_rx_tx, is_data_request, mstate);
              // At this point the notification flag may or may not be set.
              // Set the notification flag so that the client can unconditionally
              // call mutual_comm_notified() to clear the flag without blocking.
              mutual_comm_notify(c_rx_tx, mstate);
              mutual_comm_transaction(c_rx_tx, is_data_request, mstate);
              mutual_comm_complete_transaction(c_rx_tx, is_data_request, mstate);

              // Restablecemos completamente el nodo
              Bus_Status = RTNET_STATE_PASSIVE;
              transmit_error_counter = 0;
              receive_error_counter = 0;
              rx_buffer_head = 0;
              rx_buffer_tail = 0;
              message_filter_count=0;
              tx_enabled = 0;
              RTnet_calibrado = 0;
              n_cal_t_trans=100;
              n_acum_t_trans=0;
              t_trans_acum=0;
              t_trans=2000;
              // inicializamos agenda de transmisión
              for(i=0;i < RTNET_NSLOTS;i++){
                  slot_agenda[i].nbytes=0;
                  slot_agenda[i].tipo=0;
              }
              nCiclo=0;
              nCiclo_ant=0;
              puerto=0xf;

              break;
            }
          }
          // recibimos el comando y damos por terminada su ejecución
          // marcamos transacción completada
          mutual_comm_complete_transaction(c_rx_tx, is_response_to_notification, mstate);
        }
        break;
      }
      default: {
        break;
      }
    //break;
    }


    // ***
    // Chequeamos si tenemos algo en el buffer de trasmisión CANopen
    // ***
    // TODO RTnet por ahora se trasmite máximo 1 trama CANopen por ciclo, habría que revisar esto
    // parecería lógico poder trasmitir al menos tantas tramas como slots disponibles (RTNET_NSLOTS)
    if((tx_buffer_head != tx_buffer_tail) && tx_enabled){
        // Seleccionamos un slot RTnet al cual agendar la trasmisión de la trama
        // nos fijamos que tipo de trama es según el COB-id.
        // serían bytes 14/15
        k = (txbuf_copen[tx_buffer_tail], unsigned short[])[7];
        // nos quedamos con los 4 MSb de los 11 que componen el COB-id y en base a eso seleccionamos un slot
        // inicialmente seleccionamos slot inválido
        j = RTNET_NSLOTS;
        switch(k & 0x0780){
          case EMERGENCY_MESSAGE:
          //case (SYNC & 0x0780):
          case (TPDO_0_MESSAGE & 0x0780):
          case (TPDO_1_MESSAGE & 0x0780):
          case (TPDO_2_MESSAGE & 0x0780):
          case (TPDO_3_MESSAGE & 0x0780):
          {
            // trama de alta prioridad, buscamos slot libre para agendar empezando en el slot 0
            for(j=0;j<RTNET_NSLOTS;j++){
                // al encontrar un slot vacío cortamos el FOR
                if(!slot_agenda[j].nbytes) break;
            }
            break;
          }
          case (TSDO_MESSAGE & 0x0780):
          case (NMT_MESSAGE & 0x0780):
          //case (NMT_MESSAGE_BROADCAST):
          case (TIME_STAMP & 0x0780):
          case (NG_HEARTBEAT & 0x0780):
          case (TLSS_MESSAGE):
          {
            // trama de baja prioridad, buscamos slot libre para agendar empezando en el slot 1
            for(j=1;j<RTNET_NSLOTS;j++){
                // al encontrar un slot vacío cortamos el FOR
                if(!slot_agenda[j].nbytes) break;
            }
            break;
          }
        }
        // si el FOR salió antes de terminar es que encontró slot libre...
        // si no encontramos slot libre entonces la trama quedará en el buffer
        if(j != RTNET_NSLOTS){
            // agendamos la trama en el slot (j) que encontramos libre
            // en el último elemento del buffer tenemos el tamaño de trama
            slot_agenda[j].nbytes = txbuf_copen[tx_buffer_tail][RTNET_MAXBYTES/4];
            // pasamos los nbytes al buffer de la agenda en el slot correspondiente
            for(i=0;i<(slot_agenda[j].nbytes/4);i++){
                slot_agenda[j].buf[i] = txbuf_copen[tx_buffer_tail][i];
            }
            slot_agenda[j].tipo = TRAMA_COPEN;
            slot_agenda[j].ifnum = src_port;
            // avanzamos puntero del buffer
            tx_buffer_tail ++;
            tx_buffer_tail = tx_buffer_tail % RTNET_FRAME_BUFFER_SIZE;
        }
    }


    // chequeamos si perdimos alguna trama sync
    if((nCiclo - nCiclo_ant) > 1){
        if(nCiclo_ant){
            nCiclo_ant=0;
        }
    }

    // Inicio de proceso de calibración de tiempo de transporte de tramas
    // Verificamos tener calibrado t_trans
    if(!RTnet_calibrado){
        // verificamos no estar en proceso de calibración
        // para eso nos fijamos en el último tipo de trama que sacamos por el slot 0
        if(slot_agenda[0].tipo != TRAMA_RTNET_CALREQ){
            // preparamos una solicitud (puede no ser la primera que se envió...)
            // agendamos para transmitir en el próximo slot 0 una trama cal-request
            nbytes = armar_trama_cal_req((rxbuf, char[]), slot_agenda[0].buf, own_mac_addr, slot[0].delay);
            slot_agenda[0].nbytes = nbytes;
            // usamos el mismo source port por donde llegan las tramas sync
            slot_agenda[0].ifnum = src_port;
            // marcamos que es calreq para que lean llenos los campos nciclo-reply y req-txtimestamp
            slot_agenda[0].tipo = TRAMA_RTNET_CALREQ;
        }
        else{
            // TODO RTNET: esta espera por 10 ciclos podría ser aleatorizada para que todos los esclavos no
            // esperen siempre lo mismo...
            // durante proceso de calibración, vemos si NO llegó respuesta de calibración luego de 10 ciclos
            if(nCiclo > (slot_agenda[0].ciclo + 10)){
                // agendamos otra solicitud de calibración
                // agendamos para transmitir en el próximo slot 0 una trama cal-request
                nbytes = armar_trama_cal_req((rxbuf, char[]), slot_agenda[0].buf, own_mac_addr, slot[0].delay);
                slot_agenda[0].nbytes = nbytes;
                // usamos el mismo source port por donde llegan las tramas sync
                slot_agenda[0].ifnum = src_port;
                // marcamos que es calreq para que lean llenos los campos nciclo-reply y req-txtimestamp
                slot_agenda[0].tipo = TRAMA_RTNET_CALREQ;
            }
        }
    }



    // **********************************************
    // Atendemos transmisiones agendadas para los slots
    //
    // Si existen transmisiones agendadas vamos a bloquear
    // la hebra hasta que llegue el tiempo correspondiente
    // al slot en el que debemos transmitir.
    // Durante este periodo no vamos a atender tramas arribadas
    // pero tampoco se supone que debieran llegar tramas que
    // debamos atender.
    // **********************************************
    for(i=0 ; i < RTNET_NSLOTS ; i++){
        // buscamos algun slot que tenga agendado transmitir bytes
        if(slot_agenda[i].nbytes){
            // verificamos que esta fase del slot nos corresponda...
            if((nCiclo % slot[i].nfases) == slot[i].mifase){
                // verificamos que el momento de transmitir no haya pasado!
                // tenemos en cuenta el valor de tiempo de transporte calibrado
                timeout = t_sync + slot[i].delay - 2*t_trans;
                temporizador :> t;
                // Tenemos agendado transmitir, vemos si aún tenemos tiempo para hacerlo, o ya pasó el momento
                // TODO RTNET: esta comparación de tiempos hay que revisarla para prevenir rollover
                if(t < timeout){
                    // si la trama requiere rellenar campos lo hacemos
                    if(slot_agenda[i].tipo == TRAMA_RTNET_CALREQ){
                        // rellenamos campos txtimestamp y ncicloreply en la trama que vamos a transmitir
                        // el tiempo de salida de la trama asumimos que será el programado...
                        // son 64bits (unsigned long long):
                        ull = timeout;
                        for(int j=0; j<8 ; j++){
                            (slot_agenda[i].buf, unsigned char[])[22+j] = (ull , unsigned char[])[7-j];
                        }
                        // TODO RTNET: tener en cuenta con slots compartidos el nciclo de respuesta no es nciclo+1 sino +nfases
                        // pedimos respuesta en el próximo ciclo, llenamos campo response-cycle (32 bits)
                        k = nCiclo + 1;
                        for(int j=0; j<4 ; j++){
                            (slot_agenda[i].buf, unsigned char[])[30+j] = (k, unsigned char[])[3-j];
                        }
                    }
#ifdef _DEBUG_
                    else{
                        k=k;
                    }
#endif
                    // ponemos el ciclo en el que estamos transmitiendo para que quede el registro que
                    // permita detectar si no llega la respuesta en tiempo
                    slot_agenda[i].ciclo = nCiclo;
                    // esperamos al momento de transmitir la trama
                    temporizador when timerafter(timeout) :> void;

                    /*
                    // cambiamos estado del LED0
                    puerto ^= 0b0001;
                    p_led <: puerto;
                    */

                    // transmitir trama ya armada en txbuf
                    mac_tx_full(tx, slot_agenda[i].buf, slot_agenda[i].nbytes, slot_agenda[i].ifnum);

                    /*
                    // cambiamos estado del LED0
                    puerto ^= 0b0001;
                    p_led <: puerto;
                    */

                    // borramos entrada de la agenda
                    slot_agenda[i].nbytes=0;
                    printstr("Se envió trama\n");
                }
            }
        }
    }
  }
}


// funciones accesorias para acceso a puerto SPI
static inline void slave_select()
{
    spi_ss <: 0;
}

static inline void slave_deselect()
{
    spi_ss <: 1;
}



// ***************************************************************************************************
// Código de aplicación para implementar conectividad a través del bus de campo CANopen/RTnet en
// un dispositivo con perfil CiA-DSP-402 (manejador de motores).
// Esta implementación agrega sobre el perfil estándar Cia-DSP-402 la lectura de un
// codificador rotativo extra al perfil estándar, ésto con objeto de controlar a través del bus de
// campo un péndulo invertido de Furuta.
//
// Este código recibe solicitudes de acceso al diccionario de objetos desde el maestro CANopen a través
// del servidor canopen_server() por medio del canal c_application.
// Aquí se resuelven además las actualizaciones necesarias entre D.O. y actuadores/sensores.
// En ciertos casos la modificación desde el maestro CANopen de un
// objeto conlleva la modificación del actuador asociado, al igual que el cambio de lectura de
// un sensor puede conllevar la actualización de un objeto asociado.
//
// Interfaz con actuadores y sensores:
// -----------------------------------
// Los actuadores en este caso es el motor de un servomotor.
// Los sensores son dos encoders rotativos, uno midiendo giro del servomotor (encoder 0) otro midiendo
// el giro del brazo de un péndulo (encoder 1)
//
// El giro del servomotor (motor de corriente contínua) se controla mediante 2 señales de salida: una señal PWM
// cuya modulación (ancho de pulso) controla el voltaje medio aplicado al motor y otra señal DIR (digital) para
// controlar la polaridad del voltaje aplicado.
//
// El encoder 0 se lee directamente a través sus 2 señales de cuadratura.
// El encoder 1 se lee a través de un módulo inalámbrico basado en el C.I. nRF24L01+ . Dentro del brazo del
// péndulo existe un módulo inalámbrico detectando la señal en cuadratura del encoder 1 y enviando
// la cuenta a otro módulo inalámbrico conectado al XMOS.

// La comunicación de los módulos inalámbricos es SPI con 4 señales: CSN (chip-select, SCK (clock),
// MOSI (dato hacia módulo), MISO (dato hacia micro). Si se requiere operación low-power se precisa
// controlar la señal CE, de lo contrario se la deja permanentemente activa (alta). Para optimizar
// los tiempos de respuesta en recpeción y/o tasa de transf. de
// transmisión conviene leer la señal IRQ generada por el módulo.
// Se requieren mínimo 3 señales de salida y 1 señal de entrada al mico para la comunicación con el módulo.
//
// Comunicación con maestro CANopen:
// ---------------------------------
// La principal interacción entre esta hebra y CANopen se da a través de escrituras/lecturas al
// diccionario de objetos usando las funciones od_read_data, od_write_data y accesorias.
// La hebra CANopen resuelve las solicitudes de escritura, mapeo, reporte automático, etc. entre el
// dispositivo remoto (maestro) y el diccionario de objetos (de este disposivito).
//
// La hebra CANopen en general no avisa cuando objetos notables son actualizados por el maestro, eso obliga a
// efectuar polling. Sólamente los RPDOs configurados como asíncronos (a través de los objetos 0x1400) son
// pasados a esta hebra para que se tome acción instantáneamente.
//
// Para monitorear y actualizar eficientemente los objetos del diccionario hay que inicializar
// indices que permitan acceder al object_dictionary con od_read y od_write sin buscar el índice cada vez
//
// TODO RTnet: lectura encoder 0 mediante módulo QEI
// TODO RTnet: comunicaciones SPI con módulo wireless para obtención de lectura encoder 1
//
// TODO RTnet: a implementar en aplicación:
// Objetos que requieren ser actualizados/supervisados por esta hebra:
//      0x6040: Estado establecido desde maestro.
//      0x6041: Estado reportado hacia el maestro
//      0x6060: Modo establecido por maestro
//      0x6061: Reportar Modo
//      0x6063: Reportar Posición actual en cuentas (6064 reporta en unidades SI)
//      0x606C: Reportar Velocidad actual (reporta voltaje del motor)
//      0x60FF: Establecer Velocidad (establece voltaje motor)
//      0x6100: Reportar estado entradas digitales (8 subindices de 1 bit c/u)
//      0x6200: Establecer salidas digitales (8 subindices de 1 bit c/u)
//      0x6300: Establecer Salidas PWM (8 subindices de 8 bit c/u)
//      0x6400: Reportar Posición actual en cuentas de 2do. eje (enlace inalámbrico)
//      0x6502: Reportar Modos soportados
//
// Mensaje de arranque? corresponde a aplicación?:
//      boot-up message (COB ID 700h + node ID and 1 data byte with the content 00h).
//
// ***************************************************************************************************
void aplicacion ( streaming chanend c_application, chanend c_pwm, streaming chanend c_encoder[NUMBER_OF_MOTORS])
{
  timer t;
  unsigned time ;
  unsigned char pdo_data [8];
  unsigned char c,cc;
  int i;
  unsigned char data_buffer[8];
  unsigned static char s;
  unsigned int valores_pwm[N_PUERTOS_PWM], valores_pwm_ant[N_PUERTOS_PWM];
  unsigned static char p_out8_ant, p_in8_ant;
  // controlword y statusword conviene tenerlos en variables accesibles
  // si son modificados hay que actualizar el OD de lo contrario se pierde la modificación
  unsigned int Control, Status;

  // Variables de estado del sistema CANopen con perfil 402 (motor driver):
  // Controlword (unsigned16, rw):
  //    Función de los bits en modo velocidad:
  //    Bit 0: Encender
  //    Bit 1: Deshabilitar voltaje
  //    Bit 2: Parada rápida
  //    Bit 3: Habilitar operación
  //    Bit 7: Reset por fallo
  //    Bit 8: Pausa, el drive se detiene pero mantiene todos los setpoints
  // Statusword (unsigned16, rw):
  //    Función de los bits:
  //    Bit 0: Listo para encender
  //    Bit 1: Encendido
  //    Bit 2: Operación habilitada
  //    Bit 3: Fallo
  //    Bit 4: Voltaje habilitado
  //    Bit 5: Parada rápida activa
  //    Bit 6: Encendido deshabilitado
  //    Bit 7: Advertencia, ver objeto 20?
  //    Bit 8: -
  //    Bit 9: Comando remoto (siempre 1)
  //    Bit 10: Posición destino alcanzada (o velocidad destino alcanzada?)
  //    Bit 11: Limitación interna activa
  // Modes_of_operation (integer8, rw):
  //    Actualmente sólo se soporta el modo 3 o sea perfil de velocidad, se carga el valor 3 con power-up
  //    Si es modificado a algún valor distinto de 3 debe dejar de funcionar.


  // direcciones de datos notables en el diccionario de objetos:
  // los valores por defecto de estos objetos vienen inicializados en el object_dictionary.h
  int Controlword =                     od_find_data_address(od_find_index(0x6040),0);
  int Statusword =                      od_find_data_address(od_find_index(0x6041),0);
  int Modes_of_operation =              od_find_data_address(od_find_index(0x6060),0);
  int Modes_of_operation_display =      od_find_data_address(od_find_index(0x6061),0);
  int Position_actual_value =           od_find_data_address(od_find_index(0x6063),0);
  int Velocity_actual_value =           od_find_data_address(od_find_index(0x606C),0);
  int Target_velocity =                 od_find_data_address(od_find_index(0x60FF),0);
  int Entradas_digitales =              od_find_data_address(od_find_index(0x6100),1);
  int Salidas_digitales =               od_find_data_address(od_find_index(0x6200),1);
  int Salidas_pwm =                     od_find_data_address(od_find_index(0x6300),1);
  int Supported_drive_modes =           od_find_data_address(od_find_index(0x6502),0);

  // inicializamos salidas pwm a 0
  for(i=0;i<N_PUERTOS_PWM;i++){
      od_write_byte(Salidas_pwm + i , 0);
  }

  // inicializamos Status
  // remote, deshabilitar encendido, voltaje habilitado (listo para encender permanece inactivo)
  Status = 0b1001010000;
  od_write_short(Statusword, Status);

  // iniciamos maestro SPI
  spi_master_init(spi_if, DEFAULT_SPI_CLOCK_DIV);
  slave_deselect();

  // TODO: RTnet: esto es sólo para pruebas...
  // inicializamos timer para dentro de .5 segundos
  t :> time;
  time += 50000000;

  while(1){
      // Actualizamos registro de estado para revisarlo...
      Status = od_read_short(Statusword);

      // *********************
      // Revisamos modo de funcionamiento
      // *********************
      if(od_read_byte(Modes_of_operation) != MODO_COPEN_VEL){
          // modo no es velocidad, activamos parada rápida
          Status |= MSK_STATUS_PARAR;
          od_write_short(Statusword,Status);
          // TODO RTnet: cuando el eje se detenga tendríamos que hacer algo más?
      }
      // *********************
      // Revisamos controlword
      // *********************
      Control = od_read_short(Controlword);
      // si solicitan parar rápido
      if(Control & MSK_CONTROL_PARAR){
          // activamos parada rápida
          Status |= MSK_STATUS_PARAR;
          od_write_short(Statusword,Status);
      }
      // Si solicitan deshabilitar voltaje
      if(Control & MSK_CONTROL_DESHABV){
          // deshabilitamos voltaje y operación
          Status &= 0xffff - MSK_STATUS_VOLTHAB;
          Status &= 0xffff - MSK_STATUS_OPERHAB;
          od_write_short(Statusword,Status);
      }
      // Si solicitan habilitar operacion
      if(Control & MSK_CONTROL_HABOPER){
          // verificamos estar encendidos
          if(Status & MSK_STATUS_ENCENDIDO){
              Status |= MSK_STATUS_OPERHAB;
              od_write_short(Statusword,Status);
          }
      }
      // reset por fallo
      if(Control & MSK_CONTROL_DESHABV){
          // apagamos...
          // activamos fallo, desactivamos habilitación, encendido y listo
          Status |= MSK_STATUS_FALLO;
          Status &= 0xffff - MSK_STATUS_OPERHAB;
          Status &= 0xffff - MSK_STATUS_ENCENDIDO;
          Status &= 0xffff - MSK_STATUS_LISTO;
          od_write_short(Statusword,Status);
      }
      // si solicitan encender
      if(Control & MSK_CONTROL_ENCENDER){
          // verificamos estar listos
          if(Status & MSK_STATUS_LISTO){
              Status |= MSK_STATUS_ENCENDIDO;
              od_write_short(Statusword,Status);
          }
      }
      // *****************************************
      // Revisamos situación de RTnet y de CANopen
      // *****************************************
      // Si están los sistemas en funcionamiento activamos status listo
      // verificamos no estar en fallo
      if(!(Status & MSK_STATUS_FALLO)){
          // TODO RTnet: deberíamos verificar bus rtnet activo, módulo CANopen estándar XMOS no permite ésto
          // podríamos esperar a recibir alguna trama NMT, o LSS o SDO para pasar al estado listo...
          Status |= MSK_STATUS_LISTO;
          od_write_short(Statusword,Status);
      }

      // ***********************************************
      // Revisamos objetos de salidas digitales por cambios hechos desde el maestro
      // ***********************************************
      // actualizamos salidas digitales (LEDs y pines) según objeto correspondiente
      // son 8 valores en el diccionario, cada uno lleva un solo bit.
      // obtengo un char con los 8 bits cargados juntos
      c=0;
      for(i=7;i>=0;i--){
          c <<= 1;                                              // decalamos 1 bit a la izq
          c += (od_read_byte(Salidas_digitales+i) & 0x01);      // ponemos valor al bit 0
      }
      // sólo actualizamos salidas si cambió algún bit
      if(c != p_out8_ant){
          p_out8 <: c;
          p_out8_ant = c;
      }

      // ************************************
      // Leemos estado entradas digitales y si hay cambio actualizamos diccionario
      // ************************************
/*    // Utilizando p_in8 para encoders no podemos utilizarlo como puerto de entradas general
      p_in8 :> c;
      // si hay cambio vamos a actualizar diccionario
      if(c != p_in8_ant){
          p_in8_ant = c;
          // actualizamos objeto entradas_digitales según estado de pines de entrada
          for(i=0;i<8;i++){
              od_write_byte(Entradas_digitales + i, c & 0x01);  // un solo bit en cada indice
              c >>= 1;                                          // decalamos para el próximo bit
          }
      }
*/
      // *******************************************************
      // Revisamos objetos de salidas PWM por si maestro hizo cambios
      // *******************************************************
      cc=0;             // vamos a usar como marcador de cambio
      for(i=0;i<N_PUERTOS_PWM;i++){
          if((c=od_read_byte(Salidas_pwm + i)) != valores_pwm[i]){
              // actualizamos salidas pwm
              valores_pwm[i] = c;
              // flag marcador de cambio
              cc=1;
          }
      }
      // si hubo algún cambio actualizamos salidas del generador PWM
      if(cc) pwmSingleBitPortSetDutyCycle(c_pwm, valores_pwm, N_PUERTOS_PWM);

      // **********************************************************************************************
      // Atendemos canales de comunicación hacia hebra CANopen, contador encoder y eventos temporizados
      // **********************************************************************************************
      select{
        // ************************
        // Comprobamos recepción de trama CANopen
        // ************************
        case c_application :> char pdo_number :
          // recibimos la trama sin hacer nada con ella...

          //printhex(pdo_number);
          c_application :> c;           // entramos nro. bytes a seguir...
          //printhex(c);
          for(int i=0;i<c;i++){
              c_application :> cc;
              //printhex(cc);
          }

          break ;

        // TODO RTnet: actualmente sólo para ensayos:
        // eventos temporizados
        case t when timerafter(time) :> time :
          //canopen_client_send_data_to_stack ( c_application , 2 , 1 , pdo_data );
          // app_tpdo_number
          // app_length (bytes)
          // app_data (bytes)
          //printstr("Enviando PDO basura...\n");
          //c_application <: TPDO_0_MESSAGE;
          //c_application <: 2;
          //c_application <: 0xAA;
          //c_application <: 0x55;

          // boton apretado hace bit bajo
          if(!(p_in8_ant & MSK_BOTON_1)){
              // modificamos registros de salidas pwm, el bucle luego se encargará de actualizar generador PWM
              c = od_read_byte(Salidas_pwm + 3);
              c += 1;
              c %= RES_PWM;
              od_write_byte(Salidas_pwm + 3 , c);
          }
          else if(!(p_in8_ant & MSK_BOTON_2)){
              // modificamos registros de salidas pwm, el bucle luego se encargará de actualizar generador PWM
              c = od_read_byte(Salidas_pwm + 3);
              c -= 1;
              c %= RES_PWM;
              od_write_byte(Salidas_pwm + 3 , c);
          }

          // alternamos bit 7 (4to LED) de salidas digitales
          c = od_read_byte(Salidas_digitales + 7);
          c ^= 0x01;
          od_write_byte(Salidas_digitales + 7, c);

          // próxima vez dentro de .5 segundos
          time += 50000000;
          break ;

        default:
          break;
      }
  }
  spi_master_shutdown(spi_if);
}


void enableClockLeds(port clockLed) {
        clockLed <: 0x70;
}



// ******************************************************************
// Arranque de los servicios y aplicaciones que componen el sistema
//
//      -servicios ethernet para envío, recepción y filtrado de tramas (ocupa 5 núcleos)
//      -servicio de sincronización RTnet (ocupa 1 núcleo)
//      -servicio de comunicaciones CANopen (1 núcleo)
//
//      -aplicación
//              -captura de cuenta encoder local
//              -cap.enc.remoto (SPI?)
//              -actualizacion señal PWM al driver
//      TODO: implementación del contador posición y medición velocidad
//
// ******************************************************************
int main()
{
  // canales para comunicación interna entre servicios ethernet
  // se definen como arreglos de canales porque ethernet_server así lo pide, en este caso de 1 solo elem. c/u
  chan rx[1], tx[1];
  // canal para comunicación servicio RTnet <-> servicio CANopen
  chan c_rx_tx;
  // canal para comunicación CANopen <-> aplicación
  streaming chan c_application;
  // canal para comunicación contadores cuadratura <-> aplicación
  streaming chan c_qei[NUMBER_OF_MOTORS];
  chan c_pwm;

  par{
    on tile [1]:{
      char mac_address[6];
      otp_board_info_get_mac(otp_ports, 0, mac_address);
      eth_phy_reset(eth_rst);
      smi_init(smi);
      eth_phy_config(1, smi);
      // se inicia servicio de tx/rx con interfaz mii, sin interfaz smi y capacidad de 1 cliente RX y 1 cliente TX
      ethernet_server_full(mii,
                      null,
                      mac_address,
                      rx, 1,
                      tx, 1);
    }
    // se inicia servicio de sincronización RTnet
    // 1 canal de salida y 1 canal de entrada hacia los servicios Ethernet
    // 1 canal de entrada/salida hacia servidor CANopen
    on tile [1]: rtnet_sync(tx[0], rx[0], c_rx_tx);
//    on tile [1]: rtnet_sync(tx[0], rx[0]);

    // se inicia sevicio CANopen con 1 canal doble vía para comunicar con RTnet y otro doble vía para aplicación
    on tile [1]: canopen_server ( c_rx_tx , c_application );

    // se inicia la aplicación con comunicaciones a CANopen, sync_RTnet, módulo PWM y módulo contador de encoder
    on tile [0]: aplicacion ( c_application , c_pwm , c_qei);

    // se inicia el servicio para múltiples encoders.
    // Se lee cuenta y velocidad a través de array de chanends.
    // Esta versión adaptada utiliza puerto de 8 bits para leer hasta 4 encoders en cuadratura
    on tile[0]: do_multi4_qei ( c_qei, p_in8 );

    //on stdcore[0] : enableClockLeds(clockLed0);
    //on stdcore[1] : enableClockLeds(clockLed1);

    // Se inicia hebra para generación PWM.
    // Se envían actualizaciones a través del canal c_pwm y los pines de salida se pasan en el arreglo
    // de puertos pwmPorts (1 bit cada puerto)
    on tile[0] : pwmSingleBitPort(c_pwm, clk, pwmPorts, N_PUERTOS_PWM, RES_PWM, GRANO_PWM , 1);
  }
  return 0;
}
