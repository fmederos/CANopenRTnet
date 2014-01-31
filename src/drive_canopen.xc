/*
  Implementación de drive CANopen conforme a CiA-DSP402 con conexión Ethernet (RTnet).

  La conexión Ethernet implementa RTnet para acceso compartido y en tiempo real al medio físico.

  Se agregan funciones para demostración controlando un péndulo de Furuta.

*/


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
#include "qei_client.h"
//#include "qei_commands.h"
#include "rtnet.h"
#include "drive_canopen.h"
#include "spi_master.h"



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
// ------------------------

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



// Header 24 pin conectado a J5 de placa XMOS:
// TODO revisar implementación de este conexionado:
// Pin1         X0D0/GPIO-0          PWM a driver servomotor #6400sub1 replicado en #60FF
// Pin3         X0D11/GPIO-1         Salida PWM2 CANopen #6400sub2
// Pin2         X0D12/GPIO-2         Salida PWM3 CANopen #6400sub3
// Pin4         X0D23/GPIO-3         Salida PWM4 CANopen #6400sub4
// Pin5         X0D26/GPO-0/LED0     Salida dig.1 CANopen #6300sub1
// Pin7         X0D27/GPO-1/LED1     Salida dig.2 CANopen #6300sub2
// Pin9         X0D28/GPO-2          DIR (signo) a driver servomotor
// Pin13        X0D29/GPO-3          Salida DIR (signo) de PWM2
// Pin12        X0D30/GPO-4          Salida DIR (signo) de PWM3
// Pin14        X0D31/GPO-5          Salida DIR (signo) de PWM4
// Pin6         X0D32/GPO-6/LED2     Salida dig.3 CANopen #6300sub3
// Pin8         X0D33/GPO-7/LED3     Salida dig.4 CANopen #6300sub4

// Pin15        X0D36/ButtonA        Entrada dig.1 CANopen #6100sub1
// Pin17        X0D37/ButtonB
// Pin21        X0D38/GPI-0          Fase A encoder servomotor CANopen #6063
// Pin23        X0D39/GPI-1          Fase B encoder servomotor CANopen #6063
// Pin22        X0D40/GPI-2
// Pin24        X0D41/GPI-3
// Pin16        X0D42/GPI-4
// Pin18        X0D43/GPI-5

// Header 20 pin en J4 de placa XMOS
// Pin6         X0D1/P1B     SCK módulo wireless
// Pin7         X0D10/P1C    MOSI módulo wireless
// Pin16        X0D13/P1F    MISO módulo wireless
// Pin15        X0D22/P1G    CSN módulo wireless

// GPIO board conectada a zócalo triángulo para tomar voltajes 3.3V y 5.0V en P4
// Pines 19 y 20 de P4 de la GPIO suministran 3.3V y 5V resp.
// Pines 5,6,11,12,17,18 de P4 de la placa GPIO suministran GND


// Puerto de salidas digitales CANopen y salidas DIR (signo) de PWM
//on tile[0]: out port p_out8=XS1_PORT_8C;
on tile[0]: out port p_out4=XS1_PORT_4E;        // (4 salidas digitales)
on tile[0]: out port p_out_dir=XS1_PORT_4F;
// LEDs corresponden a bits 0,1,2 y 3 de p_out4
#define MSK_LED_1     0b0001
#define MSK_LED_2     0b0010
#define MSK_LED_3     0b0100
#define MSK_LED_4     0b1000

// Puertos de entradas encoder y accesorias
// 2 botones vienen de placa GPIO
on tile[0]: in port p_in=XS1_PORT_1M;           // (1 sola entrada digital)
on tile[0]: in port p_in_boton2=XS1_PORT_1N;    // (1 sola entrada digital)
on tile[0]: in port p_in_enc=XS1_PORT_8D;
// los botones corresponden a bits 0 y 1 de p_in8 o de p_in4
#define MSK_BOTON_1     0b00000001
#define MSK_BOTON_2     0b00000010

// puerto de 4 salidas independientes para señales PWM
on tile[0] : out buffered port:32 pwmPorts[] = { XS1_PORT_1A, XS1_PORT_1D, XS1_PORT_1E, XS1_PORT_1H };
on tile[0] : clock clk = XS1_CLKBLK_1;

// Conexiones al módulo wireless no pueden lograrse desde placa GPIO, hay que colocar un header directo
// sobre placa XMOS y usar líneas de 1 bit que no están disponibles en la GPIO conectada el puerto triángulo
// Se puede acceder a los puertos simples correspondientes al puerto estrella: P1B, P1C, P1F, P1G (todos de tile 0)
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


// Conexiones al pendulo Furuta:
// -----------------------------

// Conector 8pin a driver PWM servo y encoder servo
// Pin1         PWM (1 de J5)
// Pin2         DIR (9 de J5)
// Pin3         Enc-A (21 de J5)
// Pin4         GND (5,6,11 de P4)
// Pin5         Sal.Dig. (5 de J5)
// Pin6         Ent.Dig. (15 de J5)
// Pin7         Enc-B (23 de J5)
// Pin8         5V (20 de P4)

// Conector 6pin a módulo wireless para obtención de cuenta encoder brazo
// Pin1         MISO (16 de J4)
// Pin2         SCK (6 de J4)
// Pin3         GND (12,17,18 de P4)
// Pin4         CSN (15 de J4)
// Pin5         MOSI (7 de J4)
// Pin6         3.3V (19 de P4)



// **************************************************************************************
// Funciones accesorias para operación del módulo wireless nRF24L01
//
// Comandos al 24L01 empiezan con flanco desc. de línea CSN (llamada a función slave_select() )
// Para leer registro STATUS se puede hacer una lectura de 8 bits sin previamente especificar
// el registro a leer, siempre que se inicialice la opción SPI_MASTER_SD_CARD_COMPAT en 1
// Como no usamos línea IRQ no se necesita configurar las máscaras de IRQ
//
// Se asume línea CE alta para habilitar modo RX contínuamente
// En transmisor el CE también queda alto para modo TX continuo
//
// Lectura de registro de STATUS del chip nRF24L01+
// ------------------------------------------------
// Alcanza con una llamada a spi_master_in_byte() (devuelve unsigned char)
// el significado de cada bit devuelto es:
// bit7:        siempre 0 (no escribirlo)
// bit6:        flag Data-Ready (activo alto), escribir 1 para borrarlo (se pone a 0)
// bit5:        flag Data-Sent and acknowledged (activo alto), escribir 1 para borrarlo (se pone a 0)
// bit4:        flag Retransmission-Limit-overrun (activo alto), escribir 1 para borrarlo (se pone a 0)
// bits3,2,1:   Pipe-Number of data ready. valido 000..101, 111 es buffer vacío
// bit0:        TX-Buffer full
//
// **************************************************************************************

static inline void slave_select()
{
    spi_ss <: 0;
}

static inline void slave_deselect()
{
    spi_ss <: 1;
}


// Herramienta para lectura de registros de 8 bits (1 sólo byte)
unsigned char leer_reg_nrf(spi_master_interface &spi_if, unsigned char reg)
{
  unsigned char res;

  // aseguramos nro registro con sólo 5 bits activos
  // bit 5 a 0 hace una lectura
  reg &= 0b00011111;

  slave_select();
  spi_master_out_byte(spi_if, reg);
  // obtenemos 8 bits de respuesta
  res = spi_master_in_byte(spi_if);
  slave_deselect();

  return res;
}

// Herramienta para escritura de registros de 8 bits (1 sólo byte)
// El módulo nrf DEBE estar en POWER-DOWN o STAND-BY para aceptar escrituras a registros
void escribir_reg_nrf(spi_master_interface &spi_if, unsigned char reg, unsigned char val)
{
  // aseguramos nro.reg 5 bits y bit 5 a 1 para que sea una escritura
  reg &= 0b00011111;
  reg |= 0b00100000;

  slave_select();
  spi_master_out_byte(spi_if, reg);
  spi_master_out_byte(spi_if, val);
  slave_deselect();
}

// Inicializar registro 00h del nrf: bit0 en 1 (modo PRX), bit1 en 1 (powerup)
// CRC activo con 1 byte, IRQ no enmascarada, solo data-pipe 0 habilitado
// direccion de 3 bytes, sin auto-retransmit
// canal RF según prestablecido, data-rate de 1Mbps, potencia 0dBm,
// Dirección de recepción pipe0 predefinida
// Dirección de transmisión igual rx
void configurar_nrf(spi_master_interface &spi_if)
{
  unsigned char uc;
  unsigned char direccion[3]={ NRF_ADDRESS };

  // primero asegurarse power-down:
  // leemos STATUS
  slave_select();
  uc = spi_master_in_byte(spi_if);
  slave_deselect();
  uc &= 0b11111101;     // aseguramos bit1 a cero (power-down)
  // primer escritura sólo aseguramos estado power-down
  escribir_reg_nrf(spi_if, 0, uc); // STATUS es registro 0
  // en la 2da escritura vamos a configurar el STATUS
  uc |= 0b01110001;     // bit0 a 1 (primary-RX), bits6,5,4 a 1 (borramos señales IRQ)
  escribir_reg_nrf(spi_if, 0, uc);
  // en la 3er escritura hacemos power-up con bit1 a 1 (power-up)
  uc |= 0b00000010;
  escribir_reg_nrf(spi_if, 0, uc);              // recién se termina de configurar primary RX, power-up

  // deshabilitamos auto-ack de todos los pipes
  escribir_reg_nrf(spi_if, 0x01, 0b00000000);
  // habilitamos sólamente el pipe 0
  escribir_reg_nrf(spi_if, 0x02, 0b00000001);
  // configuramos direcciones de 3 bytes
  escribir_reg_nrf(spi_if, 0x03, 0x01);
  // deshabilitamos las retransmisiones
  escribir_reg_nrf(spi_if, 0x04, 0x00);
  // seleccionamos canal
  escribir_reg_nrf(spi_if, 0x05, NRF_CHANNEL);
  // seleccionamos velocidad de 1Mbps y potencia TX de 0dBm
  escribir_reg_nrf(spi_if, 0x06, 0b00000110);
  // seleccionamos tamaño del payload esperado por el receptor
  escribir_reg_nrf(spi_if, 0x11, 4);

  // configuramos dirección (3bytes) de RX (sólo pipe 0)
  slave_select();
  spi_master_out_byte(spi_if, 0x2A);    // enviamos comando escritura en registro 0x0A con bit 5 activo
  spi_master_out_byte(spi_if, direccion[0]);
  spi_master_out_byte(spi_if, direccion[1]);
  spi_master_out_byte(spi_if, direccion[2]);
  slave_deselect();

  // configuramos dirección (3bytes) de TX
  slave_select();
  spi_master_out_byte(spi_if, 0x30);    // enviamos comando de escritura en registro 0x10 con bit 5 activo
  spi_master_out_byte(spi_if, direccion[0]);
  spi_master_out_byte(spi_if, direccion[1]);
  spi_master_out_byte(spi_if, direccion[2]);
  slave_deselect();

  // REstablecemos flags y borramos FIFOs del nRF (flush)
  escribir_reg_nrf(spi_if, 0x00, 0x70);         // borramos flags
  // enviamos comando flush-rx
  slave_select();
  spi_master_out_byte(spi_if, 0b11100010);
  slave_deselect();
  // enviamos comando flush-tx
  slave_select();
  spi_master_out_byte(spi_if, 0b11100001);
  slave_deselect();
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
  unsigned char c,cc,uc;
  int i;
  unsigned int ui;
  unsigned char data_buffer[8];
  unsigned static char s;
  unsigned int valores_pwm[N_PUERTOS_PWM];                      // valores_pwm va de 0..1024
  unsigned char dirs_pwm;                                       // lleva bits de dirección de cada salida PWM
  unsigned int valores_pwm_off[N_PUERTOS_PWM]={PWM_OFF};
  unsigned int voltajes[N_PUERTOS_PWM];                         // voltajes va de -1024 a +1023
  unsigned static char p_out4_ant, p_in_ant;
  // velocidad, posición y validez del dato del encoder0
  unsigned velocidad, posicion, ok;
  unsigned posicion_ant;
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
  int Position1_actual_value =          od_find_data_address(od_find_index(0x6400),0);
  int Supported_drive_modes =           od_find_data_address(od_find_index(0x6502),0);

  // inicializamos salidas pwm a 0
  pwmSingleBitPortSetDutyCycle(c_pwm, valores_pwm_off, N_PUERTOS_PWM);
  for(i=0;i<N_PUERTOS_PWM;i++){
      od_write_int(Salidas_pwm + (i<<2) , 0);
  }

  // inicializamos Status
  // remote, deshabilitar encendido, voltaje habilitado (listo para encender permanece inactivo)
  Status = 0b1001010000;
  od_write_short(Statusword, Status);

  // iniciamos maestro SPI
  spi_master_init(spi_if, DEFAULT_SPI_CLOCK_DIV);
  // aseguramos desactiva línea de selección del módulo nRF antes de iniciarlo
  slave_deselect();
  // configuramos e iniciamos módulo nRF
  configurar_nrf(spi_if);

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
      if(c != p_out4_ant){
          p_out4 <: c;
          p_out4_ant = c;
      }

      // ************************************
      // Leemos estado entradas digitales y si hay cambio actualizamos diccionario
      // ************************************
    // Utilizando p_in8 para encoders no podemos utilizarlo como puerto de entradas general
      p_in :> c;
      // si hay cambio vamos a actualizar diccionario
      if(c != p_in_ant){
          p_in_ant = c;
          // actualizamos objeto entradas_digitales según estado de pines de entrada
          for(i=0;i<N_ENT_DIGITALES;i++){
              od_write_byte(Entradas_digitales + i, c & 0x01);  // un solo bit en cada indice
              c >>= 1;                                          // decalamos para el próximo bit
          }
      }

      // *******************************************************
      // Si estamos encendidos actualizamos valores PWM
      // Revisamos objetos de salidas PWM por si maestro hizo cambios
      // *******************************************************
//      if(Status & MSK_STATUS_ENCENDIDO){
          cc=0;             // vamos a usar como marcador de cambio
          uc=1;             // en uc llevamos máscara del bit correspondiente de las salidas DIR
          for(c=0;c<N_PUERTOS_PWM;c++){
              // leemos los objetos de salidas pwm (cada objeto ocupa 4 bytes)
              if((i=od_read_int(Salidas_pwm + (c<<2))) != voltajes[c]){
                  // actualizamos salidas pwm
                  voltajes[c] = i;
                  if(i < 0){
                      dirs_pwm |= uc;
                      valores_pwm[c] = -i;
                  }
                  else{
                      dirs_pwm &= (0xFF ^ uc);
                      valores_pwm[c] = i;
                  }
                  // flag marcador de cambio
                  cc=1;
              }
              // la próxima vuelta precisamos máscara al siguiente bit
              uc <<= 1;
          }
          // si hubo algún cambio actualizamos
          if(cc) {
              // actualizamos salidas generador PWM
              pwmSingleBitPortSetDutyCycle(c_pwm, valores_pwm, N_PUERTOS_PWM);
              // actualizamos direcciones (signos) de salidas PWM
              p_out_dir <: dirs_pwm;
          }
//      }
//      else{
//          pwmSingleBitPortSetDutyCycle(c_pwm, valores_pwm_off, N_PUERTOS_PWM);
//      }


      // ********************************************************
      // Revisamos cuenta encoder 1 (encoder servo) por si cambió
      // ********************************************************
      // TODO
      {velocidad, posicion, ok} = get_qei_data(c_encoder[1]);
      //if(ok){
          if (posicion != posicion_ant){
              // actualizamos objeto del diccionario
              od_write_int(Position_actual_value, posicion);
              od_write_int(Velocity_actual_value, velocidad);
              // registramos posicion para la próxima comparación
              posicion_ant = posicion;
          }
      //}


      // **********************************************************
      // Revisamos modulo wireless por si recibió dato de encoder 1
      // **********************************************************
      // leemos STATUS del módulo wl (modo rápido)
      slave_select();
      c = spi_master_in_byte(spi_if);
      slave_deselect();
      // chequeamos flag RX_DR
      if (c & 0b01000000){
          // hay dato/s RX
          // verificar que dato sea del pipe0
          if(c & 0b00001110){
              // no es del pipe0, descartamos el dato (un sólo byte)
              slave_select();
              spi_master_out_byte(spi_if, 0b01100001);      // comando R_RX_PAYLOAD
              cc = spi_master_in_byte(spi_if);
              slave_deselect();
              // volvemos sin restablecer el flag RX_DR para que la próxima se vuelva a
              // chequear si el dato siguiente es del pipe correspondiente y haya el nro
              // necesario de bytes en la FIFO
          }
          else{
              // lo recibido es del pipe0, verificamos que ya hayan 4 bytes
              cc = leer_reg_nrf(spi_if, 0x11);  // leemos registro RX_PW_P0
              if(cc >= 4){
                  // leemos 32 bits y pasamos al contador
                  slave_select();
                  spi_master_out_byte(spi_if, 0b01100001);      // comando R_RX_PAYLOAD
                  // obtenemos 32 bits
                  ui = spi_master_in_word(spi_if);
                  slave_deselect();
                  // actualizamos objeto del diccionario
                  od_write_int(Position1_actual_value, ui);
              }
              // mientras no se llega a los 4 bytes recibidos borramos flag para que avise con
              // el próximo byte que llegue
              escribir_reg_nrf(spi_if, 0x07, 0b01000000);
          }
      }


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
          if(!(p_in_ant & MSK_BOTON_1)){
              // modificamos registros de salidas pwm, el bucle luego se encargará de actualizar generador PWM
              i = od_read_int(Salidas_pwm);
              i += 10;
              od_write_int(Salidas_pwm , i);
          }
          p_in_boton2 :> c;
          if(!c){
              // modificamos registros de salidas pwm, el bucle luego se encargará de actualizar generador PWM
              i = od_read_int(Salidas_pwm);
              i -= 10;
              od_write_int(Salidas_pwm, i);
          }

          // alternamos bit 3 (4to LED) de salidas digitales accediendo al 4to subíndice (Sal_dig apunta al 1er sub.)
          c = od_read_byte(Salidas_digitales + 3);
          c ^= 0x01;
          od_write_byte(Salidas_digitales + 3, c);

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
    // Esta versión adaptada utiliza puertos de 8 o 4 bits para leer hasta 4 o 2 encoders en cuadratura
    on tile[0]: do_multi4_qei ( c_qei, p_in_enc );

    //on stdcore[0] : enableClockLeds(clockLed0);
    //on stdcore[1] : enableClockLeds(clockLed1);

    // Se inicia hebra para generación PWM.
    // Se envían actualizaciones a través del canal c_pwm y los pines de salida se pasan en el arreglo
    // de puertos pwmPorts (1 bit cada puerto)
    on tile[0] : pwmSingleBitPort(c_pwm, clk, pwmPorts, N_PUERTOS_PWM, RES_PWM, GRANO_PWM , 1);
  }
  return 0;
}
