
Implementación de nodo CANopen sobre RTnet en código abierto
************************************************************

Daniel García - Fernando Mederos
Oct.-2013


Función sync_rtnet() implementa la sincronización con las tramas sync recibidas desde el maestro RTnet
	-Corre en una hebra independiente, utiliza los servicios ethernet (de module_ethernet) para transmitir
		tramas en los time-slot asignados y recibir en cualquier time-slot.
		Comunica con module_canopen para intercambiar tramas CANopen.
		No comunica directamente con aplicación, la aplicación envía y recibe hacia y desde module_canopen
		por medio de un canal streaming.
	-Realiza la calibración del tiempo en tránsito de las tramas que es función del largo del cable y
		de la latencia en los interfaces.
	-Atiende solicitudes PING respondiéndolas en time-slot secundario.
	-Utiliza module_ethernet para el filtrado de tramas, sólo recibe broadcast y unicast al propio dispositivo.
	-Implementa una agenda para transmisión de tramas que tiene tantas entradas como slots asignados.
		-En cada entrada de la agenda se coloca una trama a ser trasmitida en el slot correspondiente, sólo hay
			espacio para una trama en cada entrada.
		-La agenda es leída progresivamente al inicio de cada ciclo y si se encuentra una entrada no vacía su
			contenido es trasmidido en el momento correspondiente al time-slot.
	-Las tramas recibidas del módulo canopen a través del canal c_rx_tx son agendadas para ser trasmitidas en
		un time-slot seleccionado según la prioridad de la trama CANopen. 

	TODO:
	-Actualmente los slots asignados se definen en el propio código, esto debería ser definible con DIP-switches p.ej.
	-Lo mismo ocurre con la IP del dispositivo.
	-Se podría implementar la trasmisión de tramas en una hebra independiente en lugar de la misma hebra que recibe.
	
	