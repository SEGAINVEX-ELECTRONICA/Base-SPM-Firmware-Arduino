/*
	UNIVERSIDAD AUTONÓMA DE MADRID
	SEGAINVEX: Electrónica
	OT: 20190335 
	Proyecto: Base SPM con Arduino DUE (Programación)
	Aplicación para placa PCB_A con el Arduino DUE
	Patricio Coronado. Mayo de 2019
	26/05/2020
*/
#include <Arduino.h>
#include "Muestras.h" //Array para guardar muestras del ADC
/******************************************************************
		Definición de pines
*******************************************************************/
#define LM500_SHDWN 58 //Activa la fuente de 48V
#define P15V_ON 71 //Activa la fuente de 48V
//	Definición de pines
//Driver de piezomotores TCMC90
#define RES0  33 
#define RES1  31 
#define MODE0 27 
#define MODE1 29 
#define SENTIDO   25
#define CLK  23 // Arduino pin 23 es port A pin 14.
#define CLK_1 PIOA->PIO_SODR=(1<<14);   // set pin
#define CLK_0 PIOA->PIO_CODR=(1<<14);  // clear pin
//Relés para rutear la potencia del driver de piezomotores TCMC90
#define RELE_HD 53
#define RELE_Z1 51
#define RELE_Z2 49
#define RELE_Z3 47
#define RELE_X  45
#define RELE_Y  43
//LEDS AUXILIARES
#define LED0 41 //Led frontal PC9. Arduino pin 41 es port C pin 9.
#define LED0_1 PIOC->PIO_SODR=(1<<9);   // set pin
#define LED0_0 PIOC->PIO_CODR=(1<<9);  // clear pin
#define LED1  39//Led 1 Arduino pin 37 es port C pin 7.
#define LED1_1 PIOC->PIO_SODR=(1<<7);   // set pin
#define LED1_0 PIOC->PIO_CODR=(1<<7);  // clear pin
#define LED2  37//Led 2 Arduino pin 37 es port C pin 5.
#define LED2_1 PIOC->PIO_SODR=(1<<5);   // set pin
#define LED2_0 PIOC->PIO_CODR=(1<<5);  // clear pin
#define LED3  35 //Led 3 Arduino pin 35 es port C pin 3.
#define LED3_1 PIOC->PIO_SODR=(1<<3);   // set pin
#define LED3_0 PIOC->PIO_CODR=(1<<3);  // clear pin
// Acelerómetros
#define I2_1 14	
#define I1_1 15
#define I2_2 16 
#define I1_2 17
// Sensor humedad temperatura
#define SEN_DATA  63 //Datos
#define SEN_CLK  64 //CLK
//Infrarojo
#define IR_DATA 63
//Tensión de 48V potencia del driver
#define SALIDA_48V 61
//Motores de la cabeza
#define MHD1 3
#define MHD0 2
// Lineas de control desde Dulcinea 
#define DSP_CLK 5
#define DSP_48V 4
// Timer para clk del driver de motores piezoleg 
#define TIMER_CLK Timer5
#define TIMER_ADC Timer4
#define TIMER_FOTO_ACEL Timer3 

// Entradas analógicas
#define F_NORMAL A5
#define F_LATERAL A7
#define SUM A6
// Pines de test
#define TEST_ADC 22// Arduino pin 22 es port B pin 26.
#define TEST_ADC_1 PIOB->PIO_SODR=(1<<26);   // set pin
#define TEST_ADC_0 PIOB->PIO_CODR=(1<<26);  // clear pin
#define TEST_CLK 26
#define TEST_SENSORHT 30// Arduino pin 30 es port D pin 9.
#define TEST_SENSORHT_1 PIOD->PIO_SODR=(1<<9);   // set pin
#define TEST_SENSORHT_0 PIOD->PIO_CODR=(1<<9);  // clear pin
#define TEST_ACELEROMETRO 34 // Arduino pin 34 es port C pin 2.
#define TEST_ACELEROMETRO_1 PIOC->PIO_SODR=(1<<2);   // set pin
#define TEST_ACELEROMETRO_0 PIOC->PIO_CODR=(1<<2);  // clear pin
#define TEST_FOTODIODO 38 // Arduino pin 38 es port C pin 6.
#define TEST_FOTODIODO_1 PIOC->PIO_SODR=(1<<6);   // set pin
#define TEST_FOTODIODO_0 PIOC->PIO_CODR=(1<<6);  // clear pin
#define TEST_1 42
/******************************************************************
				MODOS DE ONDA
	Solo se va a utilizar el modo de onda OMEGA64 y sus
	posibles resoluciones 256, 512, 1024, 2048.
	Con el mando la resolución es siempre 256 				
*******************************************************************/
//Onda y Resolucion
// omega564 -> 256,	sine1s85 -> 64,  Rhomb->32,   RhombF -> 32
// omega564 -> 512,	sine1s85 -> 128, Rhomb->64,   RhombF -> 64
// omega564 -> 1024,sine1s85 -> 256, Rhomb->128,  RhombF -> 128
// omega564 -> 2048,sine1s85 -> 512, Rhomb->256,  RhombF -> 256
#define OMEGA564 3
#define SINE1S85 2
#define RHOMB    1  
#define RHOMBF	 0
#define RES_BTH 256
#define RES_INICIAL 2028
/******************************************************************
					MOTORES
*******************************************************************/
#define NINGUNO 0
#define Z1      1
#define Z2      2
#define Z3      3
#define Z1Z2    4
#define Z1Z3    5
#define Z2Z3    6
#define Z1Z2Z3  7
#define X       8
#define Y       9
#define FotodiodoX	10
#define LaserX		11
#define LaserY		12
#define FotodiodoY	13
#define MIN_MOTOR 0
#define MAX_MOTOR 13
/******************************************************************
 * 		MARCHA PARO
*******************************************************************/
#define MARCHA 1
#define PARO 0
/******************************************************************
 * 	SENTIDO DEL MOVIMIENTO
*******************************************************************/
#define BAJAR 0	 // Los motores z bajan	la cabeza
#define SUBIR 1	 //	Los motores z suben la cabeza
/******************************************************************
*	  	FRECUENCIA
*******************************************************************/
#define MINIMA_FRECUENCIA 0   // Frecuencia del step mÍnima en KHz
//La frecuencia la puedo subier a 150KHz pero hay que modificar el ajuste resolución-frecuencia
#define MAXIMA_FRECUENCIA 100 // Frecuencia del step máxima en KHz
#define MAXIMA_FRECUENCIA_BTH 64 // Frecuencia máxima con mando
#define FRECUENCIA_1 1  //KHz Escala de frecuencias para seleccionar según
#define FRECUENCIA_2 5	//KHz la posiciOn del mando 1,2,3,4,5 
#define FRECUENCIA_3 15 //KHz
#define FRECUENCIA_4 30 //KHz
#define FRECUENCIA_5 63 //KHz
#define PASOS_MAXIMOS 400000 //Máximo número de pasos que se pueden programar
//#define PASOS_MAXIMOS 1000000 //Máximo número de pasos que se pueden programar
#define CONTADOR_MAXIMO 800000
/************************************************************************
				VARIOS
************************************************************************/
#define ENCENDIDO 1
#define APAGADO 0
#define MAX_FREC_I2C 400000 //Frecuencia I2C máxima
#define NORMAL_FREC_I2C 100000 //Frecuencia I2C standar
//#define ACCEL_HEXADECIMAL 1 //Comentar para enviar los datos como float si no complemento a 2
#define SI 1
#define NO 0
/********************************************************************
				Prototipos de funciones
*********************************************************************/
/**********************************************************************
				Funciones que responden al PC
**********************************************************************/
void pc_marcha_motor_pasos(void);
void pc_fotodiodo(void);
void pc_sensor_temperatura_humedad(void);
void pc_sentido(void);
void pc_frecuencia(void);
void pc_motor_activo(void);
void pc_resolucion(void);
void pc_onda(void);
void pc_marcha_paro(void);
void pc_marcha_motor(void);
void pc_contador(void);
void pc_anda_numero_de_pasos(void);
void pc_version(void);// Envía al PC la versión del software
void pc_reset(void);//Pone la base en el estado inicial
void pc_variables(void);//Devuelve al PC las variables de estado
void pc_acelerometro(void);
void pc_inicia_fotodiodo(void);
void pc_fin_fotodiodo(void);
void pc_inicia_acelerometro(void);
void pc_fin_acelerometro(void);
//Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
//Funciones que no hacen nada, implementadas por compatibilidad.
void pc_final_de_carrera(void);

/**********************************************************************
		Funciones que responden al mando bluethoot
**********************************************************************/
void bluetooth_marcha_motor(void);
void bluetooth_para_motor(void);
void bluetooth_estado(void);
/**********************************************************************
	Funciones que testean y cambian las variables del sistema
**********************************************************************/
void timer_clk(void); //rutina de atención a la interrupción del timer del clk
void clk_externo(void);//rutina de atención a la interrupción del pin DSP_CLK
void timer_ADC(void);//rutina de atención a la interrupción del timer de adquisición
void timer_foto_acel(void);//interrupción del timer para enviar datos del fotodiodo
int cambia_onda(unsigned int);
int cambia_frecuencia_resolucion(unsigned int Frec,unsigned int Res);
int cambia_motor(unsigned int);
int cambia_sentido(unsigned int);
int marcha_paro_motor(unsigned int);
void parar_clk_step(void);//Función auxiliar de marcha_paro_motor()
void activa_48V(void);//Función fs1: Activa la fuente de 48V
void desactiva_48V(void);// Desactiva la fuente de 48V
void programa_pasos(int);//Programa el número de pasos
bool busca_acelerometro(void);//Busca el acelerómetro
/**********************************************************************
					Funciones para test
**********************************************************************/
// Relés
void modo_depuracion_no(void);
void modo_depuracion_si(void);
void desactiva_rele_z1(void);
void activa_rele_z1(void);
void desactiva_rele_z2(void);
void activa_rele_z2(void);
void desactiva_rele_z3(void);
void activa_rele_z3(void);
void desactiva_rele_y(void);
void activa_rele_y(void);
void desactiva_rele_x(void);
void activa_rele_x(void);
void desactiva_rele_hd(void);
void activa_rele_hd(void);
//Leds
void desactiva_led_3(void);
void activa_led_3(void);
void desactiva_led_2(void);
void activa_led_2(void);
void desactiva_led_1(void);
void activa_led_1(void);
void desactiva_led_0(void);
void activa_led_0(void);
// Driver TCMC90
void desactiva_res_1(void);
void activa_res_1(void);
void desactiva_res_0(void);
void activa_res_0(void);
void activa_mode_0(void);
void desactiva_mode_0(void);
void activa_mode_1(void);
void desactiva_mode_1(void);
void activa_dir(void);
void desactiva_dir(void);
void activa_clk(void);
void desactiva_clk(void);
// Motores de la cabeza
void desactiva_motor_head_1(void);
void activa_motor_head_1(void);
void desactiva_motor_head_0(void);
void activa_motor_head_0(void);
//Acelerómetros
void desactiva_i22(void);
void activa_i22(void);
void desactiva_i21(void);
void activa_i21(void);
void desactiva_i12(void);
void activa_i12(void);
void desactiva_i11(void);
void activa_i11(void);
// Movimiento de motores (activa el clk de steps 200us)
void mueve_motor(void);
void para_motor(void);
void test_step(void);
/************************************************************************
		          // Menú de comandos SCPI
************************************************************************/
tipoNivel BLUETOOTH[] = //BTH  Array de estructuras tipo Nivel
{
	SCPI_COMANDO(MARCHA,MARCHA, bluetooth_marcha_motor)
	SCPI_COMANDO(PARO,PARO, bluetooth_para_motor)
	SCPI_COMANDO(ESTADO,EST, bluetooth_estado)
};
tipoNivel MOTORES[] = //Comandos del PC que hacen funcionar el sistema
{
	SCPI_COMANDO(FOTODIODO,FOT,pc_fotodiodo)//La base envía las señales del fotodiodo
	SCPI_COMANDO(TH,TH,pc_sensor_temperatura_humedad)//La base envía la temperatura y humedad
	SCPI_COMANDO(MARCHAMOTORPASOS,MMP,pc_marcha_motor_pasos)//Programa un motor con resolución, frecuencia, sentido y pasos y lo pone en marcha
	SCPI_COMANDO(MARCHAMOTOR,MM,pc_marcha_motor)//Programa un motor con resolución, frecuencia, sentido y pasos y lo pone en marcha
	SCPI_COMANDO(MARCHAPARO,MP, pc_marcha_paro)//Pone en marcha el motor seleccionado
	SCPI_COMANDO(FRECUENCIA,FR,pc_frecuencia)//Actualiza el valor de frecuencia de paso (micropaso)
	SCPI_COMANDO(ANPASOS,AN,pc_anda_numero_de_pasos)// Programa un número de pasos a dar "Pasos" y se decremente cada paso 
	SCPI_COMANDO(MOTORACTIVO,MA,pc_motor_activo)//Cambia el motor seleccionado
	SCPI_COMANDO(RESOLUCION,RE,pc_resolucion)//Cambia la resolución
	SCPI_COMANDO(SENT,SE,pc_sentido)//Para cambiar el sentido
	SCPI_COMANDO(CONTADOR,CO,pc_contador)//Contador que se inicializa trás un comando de "marcha"
	SCPI_COMANDO(ACELEROM,AC,pc_acelerometro)//Lee el acelerómetro
	SCPI_COMANDO(INIFOT,IFO,pc_inicia_fotodiodo)//El fotodiodo envia datos cada 100ms
 	SCPI_COMANDO(FINFOT,FIF,pc_fin_fotodiodo)//El fotodiodo deja de envia datos cada 100ms
	SCPI_COMANDO(INIACEL,IAC,pc_inicia_acelerometro)//El acelerometro envia datos cada 100ms
 	SCPI_COMANDO(FINACEL,FNA,pc_fin_acelerometro)//El acelerometro deja de envia datos cada 100ms
	SCPI_COMANDO(RESET,RS,pc_reset)//Pone la base en su estado inicial
	SCPI_COMANDO(VARIABLES,VAR,pc_variables)//Pone las variables en un estado determinado
	SCPI_COMANDO(VERSION,VER,pc_version)// Envía al PC la versión del software
	SCPI_COMANDO(ONDA,ON,pc_onda)//No la utiliza su software. Se programa para uso futuro
	SCPI_COMANDO(HABILITAFC,HF,pc_final_de_carrera)// Por compatibilidad. No hay finales de carrera
	SCPI_COMANDO(FLAGERRORFC,FE,pc_final_de_carrera) //Por compatibilidad. No hay finales de carrera
};
tipoNivel TEST[] = // Comandos de test
{
	//Fuente de 48V 
	SCPI_COMANDO(TESTSTEP,TS,test_step)
	SCPI_COMANDO(MODODEPURACIONSI,MDS,modo_depuracion_si)
	SCPI_COMANDO(MODODEPURACIONNO,MDN,modo_depuracion_no)
	SCPI_COMANDO(ACTIVA_48V,A48,activa_48V)
	SCPI_COMANDO(DESACTIVA_48,D48,desactiva_48V)
	//Mueve o para motores
	SCPI_COMANDO(MUEVE_MOTOR,MM,mueve_motor)
	SCPI_COMANDO(PARA_MOTOR,PM,para_motor)
	// Relés
	SCPI_COMANDO(ACTIVA_X,AX,activa_rele_x) 
	SCPI_COMANDO(DESACTIVA_X,DX,desactiva_rele_x) 
	SCPI_COMANDO(ACTIVA_Y,AY,activa_rele_y)
	SCPI_COMANDO(DESACTIVA_Y,DY,desactiva_rele_y)
	SCPI_COMANDO(ACTIVA_Z1,AZ1,activa_rele_z1)
	SCPI_COMANDO(DESACTIVA_Z1,DZ1,desactiva_rele_z1)
	SCPI_COMANDO(ACTIVA_Z2,AZ2,activa_rele_z2)
	SCPI_COMANDO(DESACTIVA_Z2,DZ2,desactiva_rele_z2)
	SCPI_COMANDO(ACTIVA_Z3,AZ3,activa_rele_z3)
	SCPI_COMANDO(DESACTIVA_Z3,DZ3,desactiva_rele_z3)
	SCPI_COMANDO(ACTIVA_HD,AH,activa_rele_hd)
	SCPI_COMANDO(DESACTIVA_HD,DH,desactiva_rele_hd)
	//Leds
	SCPI_COMANDO(ACTIVA_LED0,AL0,activa_led_0)
	SCPI_COMANDO(DESACTIVA_LED0,DL0,desactiva_led_0)
	SCPI_COMANDO(ACTIVA_LED1,AL1,activa_led_1)
	SCPI_COMANDO(DESACTIVA_LED1,DL1,desactiva_led_1)
	SCPI_COMANDO(ACTIVA_LED2,AL2,activa_led_2)
	SCPI_COMANDO(DESACTIVA_LED2,DL2,desactiva_led_2)
	SCPI_COMANDO(ACTIVA_LED3,AL3,activa_led_3)
	SCPI_COMANDO(DESACTIVA_LED3,DL3,desactiva_led_3)
	// Driver TCMC90
	SCPI_COMANDO(ACTIVA_RES0,AR0,activa_res_0)			
	SCPI_COMANDO(DESACTIVA_RES0,DR0,desactiva_res_0)
	SCPI_COMANDO(ACTIVA_RES1,AR1,activa_res_1)
	SCPI_COMANDO(DESACTIVA_RES1,DR1,desactiva_res_1)
	SCPI_COMANDO(ACTIVA_MODE0,AM0,activa_mode_0)
	SCPI_COMANDO(DESACTIVA_MODE0,DM0,desactiva_mode_0)
	SCPI_COMANDO(ACTIVA_MODE1,AM1,activa_mode_1)
	SCPI_COMANDO(DESACTIVA_MODE1,DM1,desactiva_mode_1)
	SCPI_COMANDO(ACTIVA_DIR,AD,activa_dir)
	SCPI_COMANDO(DESACTIVA_DIR,DD,desactiva_dir)
	SCPI_COMANDO(ACTIVA_CLK,ACK,activa_clk)
	SCPI_COMANDO(DESACTIVA_CLK,DCK,desactiva_clk)
	// Motores de la cabeza
	SCPI_COMANDO(ACTIVA_MH0,AMH0,activa_motor_head_0)
	SCPI_COMANDO(DESACTIVA_MH0,DMH0,desactiva_motor_head_0)
	SCPI_COMANDO(ACTIVA_MH1,AMH1,activa_motor_head_1)
	SCPI_COMANDO(DESACTIVA_MH1,DMH1,desactiva_motor_head_1)
	//Acelerómetros
	SCPI_COMANDO(ACTIVA_I1_1,AI11,activa_i11)
	SCPI_COMANDO(DESACTIVA_I1_1,DI11,desactiva_i11)
	SCPI_COMANDO(ACTIVA_I1_2,AI12,activa_i12)
	SCPI_COMANDO(DESACTIVA_I1_2,DI12,desactiva_i12)
	SCPI_COMANDO(ACTIVA_I2_1,AI21,activa_i21)
	SCPI_COMANDO(DESACTIVA_I2_1,DI21,desactiva_i21)
	SCPI_COMANDO(ACTIVA_I2_2,AI22,activa_i22)
	SCPI_COMANDO(DESACTIVA_I2_2,DI22,desactiva_i22)
};
MENU_SCPI  //menú de  comandos
{
	SCPI_SUBMENU(MOTORES,MOT)	//COMANDOS DEL PC
	SCPI_SUBMENU(BLUETOOTH,BTH)	//COMANDOS DEL BLUETOOTH
	//Comandos que ejecutan funciones definidas en la librería Segainvex_SCPI_Serial
	SCPI_COMANDO(ERROR,ERR,errorSCPI)// Envía el ultimo error
  	SCPI_COMANDO(*IDN,*IDN,idnSCPI)// Identifica el instrumento
	SCPI_COMANDO(*OPC,*OPC,opcSCPI)// Devuelve un 1 al PC
	SCPI_COMANDO(*CLS,*CLS,clsSCPI)// Borra la pila de errores
	//Comandos para test
	SCPI_SUBMENU(TEST,TEST)	//COMANDOS DEL PC PARA TEST	
};
tipoNivel Raiz[]= SCPI_RAIZ //// Declaración OBLIGATORIA del nivel Raiz. Siempre igual
//Lista de errores:
String ErroresBaseSPM[]=
{
  // Errores del sistema SCPI 0...6
  // Errores personalizados por el usuario 
  "7 Frecuencia de onda excesiva",
  "8 Resolucion incorrecta",
  "9 Motor incorrecto",		
  "10 Modo de onda no permitido",
  "11 Frecuencia y resolucion incompatibles",
  "12 Frecuencia incorrecta",
  "13 Sentido incorrecto",
  "14 Resolucion ajustada",
  "15 Numero de pasos incorrectos",
  "16 Frecuencia ajustada",
  "17 Resolucion incorrecta",
  "18 Potencia desactivada por el DSP",
  "19 Modo de onda incorrecto",
  "20 No se permite cambiar de onda",
  "21 Error lectura sensor humedad-temperatura",
  "22 No hay motor seleccionado",
  "23 No hay acelerometro conectado",
};
/************************************************************************
          Variables de estado del sistema	(globales)	
************************************************************************/
//Acelerómetro MMA8452
MMA8452Q Acelerometro;
bool AcelerometroConectado=NO;
bool FotoAcel;//Para indicar si se envian datos del fotodiodo o del acelerómetro
#define ACELEROMETRO 1
#define FOTODIODO 0
#define T300ms 300000 
#define T250ms 250000 
#define T200ms 200000 
#define T100ms 100000 
#define T150ms 150000
#define T50ms   50000
int contadorEnvios=0;
#define ENVIOS_MAXIMOS 10000 
//Sensor humedad temperatura
#ifdef SENSOR_SHT11
	SHT1x SHT11(SEN_DATA, SEN_CLK);
#endif
#ifdef SENSOR_DHT22
	DHT dht(SEN_DATA, DHT22);
#endif
//Objeto SCPI
SegaSCPI BaseScpi(Raiz,"Base SPM",ErroresBaseSPM);
//Para no tener que teclear todo al aludir al puerto serie 
 //BaseScpi.PuertoActual->println(); ahora sería puerto->println();;
#define Println BaseScpi.PuertoActual->println 
//#define Printf BaseScpi.PuertoActual->printf //No soporta Serial.printf
#define Print BaseScpi.PuertoActual->print
char Version[]="Base SPM V1.2";//Las distintas "Bases SPM" pueden tener versión
bool depuracion=false; //1 para hacer depuración del software. 0 servicio 
#define debug Serial1.print //Puerto para depuracion
//normal. El modo depuración se usa en la fase de desarrollo del software
//para que el sistema envia al PC cadenas con información relevante
bool EstadoCLK;// Estado de la linea de CLK
bool Estado_48V; //Estado de la fuente de 48V. 0 apagada, 1 encendida. 
unsigned int Pasos;//Para programar un número de pasos. Si Pasos !=0 estamos
// en modo "pasos programados" pero si vale 0 estamos en modo free runing  
unsigned int Contador;//Contador de pasos. Se pone a 0 cada vez que se pone
//en marcha un motor (o por comando de pc o mando)
unsigned int EstadoMarchaParo;//Refleja el estado del timer del clk.
unsigned int Frecuencia;// Del clk de steps de motor
unsigned int Resolucion;
unsigned int MotorActivo;
unsigned int Sentido=SUBIR;
unsigned int ModoDeOnda;/*=OMEGA564 siempre*/;
//Arrays para meter muestras de la adquisición del ADC
#define LONG_MUESTRAS 16 // Al princio 64
Muestras FuerzaNormal(LONG_MUESTRAS);
Muestras FuerzaLateral(LONG_MUESTRAS);
Muestras Suma(LONG_MUESTRAS);
//Tabla de frecuencias-periodos 
//para frecuencia de 1KHz Periodos[0]=1000 hasta 200KHz Periodos[199]=5
int Periodo[]={1000, 1000, 500, 333, 250, 200, 167, 143, 125, 111, 100, 91, 83,
77, 71, 67, 63, 59, 56, 53, 50, 48, 45, 43, 42, 40, 38, 37, 36, 34, 33, 32,
31, 30, 29, 29, 28, 27, 26, 26, 25, 24, 24, 23, 23, 22, 22, 21, 21, 20, 20,
20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 16, 16, 16, 16, 15, 15, 15, 15,
14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12,
11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};//Periodos 201 valores 
/************************************************************************
	              macros
************************************************************************/
#define DESACTIVA_MOTORES \
	{digitalWrite(RELE_Z1,LOW);\
	digitalWrite(RELE_Z2,LOW);\
	digitalWrite(RELE_Z3,LOW);\
	digitalWrite(RELE_X,LOW);\
	digitalWrite(RELE_Y,LOW);\
	digitalWrite(RELE_HD,LOW);}
//Macro para la lectura del pin DSP_48V 
//#define _DSP_48V digitalRead(DSP_48V) //Lectura del pin del DSP
#define _DSP_48V 1 //Anulo la lectura y fuerzo el valor a 1	
//Interrupción del CLK del DSP activación desactivación
#define DSP_CLK_ON  attachInterrupt(digitalPinToInterrupt(DSP_CLK),clk_externo,FALLING);
#define DSP_CLK_OFF detachInterrupt(digitalPinToInterrupt(DSP_CLK));	
/************************************************************************
 * Firmas de las  cadenas enviadas por serial
************************************************************************/
#define FMARCHAMOTORPASOS  "VM" //pc_marcha_motor_pasos(void); 
#define FMARCHAMOTOR       "HX" //pc_marcha_motor(void);
#define FVARIABLES         "BL" //pc_variables(void);
#define FCONTADOR          "XT" //pc_contador(void);
#define FANDANUMERODEPASOS "SZ" //pc_anda_numero_de_pasos(void);
#define FMARCHAPARO        "PM" //pc_marcha_paro(void);
#define FSENTIDO           "WD" //pc_sentido(void); 
#define FFRECUENCIA        "CR" //pc_frecuencia(void); 
#define FMOTORACTIVO       "MV" //pc_motor_activo(void);
#define FRESOLUCION        "RS" //pc_resolucion(void);
#define FONDA              "NN" //pc_onda(void);
#define FFOTODIODO         "FT" //pc_fotodiodo(void); 
#define FTEMPERATURA       "T "  //pc_sensor_temperatura_humedad(void); 
#define FACELEROMETRO      "LC" //pc_acelerometro(void);
#define FVERSION           "KK" //pc_version(void); 
#define FIDN               "DW" //void idnSCPI(void); 
#define FSTOP              "ZP" //Mensaje de parada. Se envía para informar de parada de motor
#define FBLUETOOTHESTADO   "YY" //void  bluetooth_estado(void)
#define FESTADO48V         "JJ" //La cadena es 1 o 0 estado del DC/DC activo o no
/************************************************************************
************************************************************************/