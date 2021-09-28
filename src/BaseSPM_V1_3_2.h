/*
	UNIVERSIDAD AUTONÓMA DE MADRID
	SEGAINVEX: Electrónica
	OT: 20190335 
	Proyecto: Base SPM con Arduino DUE (Programación)
	Aplicación para placa PCB_A con el Arduino DUE
	Patricio Coronado. Mayo de 2019
	26/05/2020
	Editado el 26/09/2020
*/
#include <Arduino.h>
#include "Muestras.h" //Calase para muestras del ADC
/******************************************************************
		Definición de pines
*******************************************************************/
//	Definición de pines
#define LM500_SHDWN 58 //Activa el integrado DC/DC que genera 48V
#define P15V_ON 12 //Alimenta el DC/DC Que genera los 48V
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
//LEDs
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
#define LED4  68 //Led 4 Arduino pin 68 es port A pin 1.
#define LED4_1 PIOA->PIO_SODR=(1<<1);   // set pin
#define LED4_0 PIOA->PIO_CODR=(1<<1);  // clear pin
#define LED5  69 //Led 5 Arduino pin 69 es port A pin 0.
#define LED5_1 PIOA->PIO_SODR=(1<<0);   // set pin
#define LED5_0 PIOA->PIO_CODR=(1<<0);  // clear pin
//Motores de la cabeza
#define MHD1 3
#define MHD0 2
// Lineas de control desde Dulcinea 
#define DSP_CLK A8 //Pin 62 CLK desde Dulcinea cambiado, antes era el pin 5 16/09/2020
#define DSP_48V A9// Pin 63 (no se utiliza, era para apagar los 48V desde Dulcinea pin) cambiado, antes era el pin 4 16/09/2020
// Entradas analógicas desde el fotodiodo
#define F_NORMAL A5
#define F_LATERAL A7
#define SUM A6
// Pines de test
#define TEST_ADC 22// Arduino pin 22 es port B pin 26.
#define TEST_ADC_1 PIOB->PIO_SODR=(1<<26);   // set pin
#define TEST_ADC_0 PIOB->PIO_CODR=(1<<26);  // clear pin
#define TEST_SENSORHT 30// Arduino pin 30 es port D pin 9.
#define TEST_SENSORHT_1 PIOD->PIO_SODR=(1<<9);   // set pin
#define TEST_SENSORHT_0 PIOD->PIO_CODR=(1<<9);  // clear pin
#define TEST_ACELEROMETRO 34 // Arduino pin 34 es port C pin 2.
#define TEST_ACELEROMETRO_1 PIOC->PIO_SODR=(1<<2);   // set pin
#define TEST_ACELEROMETRO_0 PIOC->PIO_CODR=(1<<2);  // clear pin
#define TEST_FOTODIODO 38 // Arduino pin 38 es port C pin 6.
#define TEST_FOTODIODO_1 PIOC->PIO_SODR=(1<<6);   // set pin
#define TEST_FOTODIODO_0 PIOC->PIO_CODR=(1<<6);  // clear pin
/******************************************************************
		Fin definición de pines
*******************************************************************/
/*****************************************************************
        Timers 
*******************************************************************/
#define TIMER_CLK Timer5 //para clk del driver de motores piezoleg
#define TIMER_ADC Timer4 //Para muestrerar las señales del fotodiodo
#define TIMER_FOTO_ACEL Timer3 //Para temporizar envios al PC de señales de accel. y fotodiodo
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
#define RES_INICIAL 2028 //Resolución por defecto
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
#define PASOS_MAXIMOS 600000 //Máximo número de pasos que se pueden programar
//#define PASOS_MAXIMOS 500000 //Máximo número de pasos que se pueden programar
#define CONTADOR_MAXIMO 800000
/************************************************************************
				CONFIGURACION I2C
************************************************************************/
#define MAX_FREC_I2C 400000 //Frecuencia I2C máxima
#define NORMAL_FREC_I2C 100000 //Frecuencia I2C standar
/********************************************************************
				Prototipos de funciones
*********************************************************************/
/**********************************************************************
				Funciones que responden al PC
**********************************************************************/
void pc_marcha_motor_pasos(void);
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
void pc_variables(void);//Devuelve al PC las variables del sistema
void pc_inicia_fotodiodo(void);
void pc_fin_fotodiodo(void);
void pc_inicia_acelerometro(void);
void pc_fin_acelerometro(void);
void pc_activa_48v(); //Activa/desactiva ed DC/DC 48V
void pc_sensor_bme280(void);//TO DO También puede leer la presión
//Funciones scpi comunes a todos los sistemas
void errorSCPI(void);
void opcSCPI(void);
void idnSCPI(void);
void clsSCPI(void);
//Funciones que no hacen nada, implementadas por compatibilidad.
void pc_final_de_carrera(void);

/**********************************************************************
	Funciones que actuan sobre las variables del sistema
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
void parar_clk_step(void);//Función inline, auxiliar de marcha_paro_motor()
void activa_48V(void);//Función fs1: Activa la fuente de 48V
void desactiva_48V(void);// Desactiva la fuente de 48V
void programa_pasos(int);//Programa el número de pasos
bool busca_acelerometro(void);//Busca el acelerómetro
/**********************************************************************
					Funciones para test
	Si el sistema está en modo depuración la variable "depuracion"
	es true. En fase de depuración Se pueden enviar cadenas al PC
	en puntos críticos del programa con información sensible que 
	ayude a detectar problemas. 
**********************************************************************/
void modo_depuracion_no(void);
void modo_depuracion_si(void);
/************************************************************************
		          // Menú de comandos SCPI
************************************************************************/
tipoNivel MOTORES[] = //Comandos del PC que hacen funcionar el sistema
{
	SCPI_COMANDO(TH,TH,pc_sensor_temperatura_humedad)//La base envía la temperatura y humedad *
	SCPI_COMANDO(MARCHAMOTORPASOS,MMP,pc_marcha_motor_pasos)//Programa un motor con resolución, frecuencia, sentido y pasos y lo pone en marcha *
	SCPI_COMANDO(MARCHAMOTOR,MM,pc_marcha_motor)//Programa un motor con resolución, frecuencia, sentido y lo pone en marcha *
	SCPI_COMANDO(MARCHAPARO,MP, pc_marcha_paro)//Pone en marcha el motor seleccionado *
	SCPI_COMANDO(ACTIVA48V,AV,pc_activa_48v)//Activa 48V *
	SCPI_COMANDO(FRECUENCIA,FR,pc_frecuencia)//Actualiza el valor de frecuencia de paso (micropaso) *
	SCPI_COMANDO(ANPASOS,AN,pc_anda_numero_de_pasos)// Programa un número de pasos a dar "Pasos" y se decremente cada paso * 
	SCPI_COMANDO(MOTORACTIVO,MA,pc_motor_activo)//Cambia el motor seleccionado
	SCPI_COMANDO(RESOLUCION,RE,pc_resolucion)//Cambia la resolución
	SCPI_COMANDO(SENT,SE,pc_sentido)//Para cambiar el sentido *
	SCPI_COMANDO(BME280,BM,pc_sensor_bme280)//Lee el sensor de humedad y temperatura BME280 
	SCPI_COMANDO(CONTADOR,CO,pc_contador)//Contador que se inicializa trás un comando de "marcha" *
	SCPI_COMANDO(INIFOT,IFO,pc_inicia_fotodiodo)//El fotodiodo envia datos cada 200ms *
 	SCPI_COMANDO(FINFOT,FIF,pc_fin_fotodiodo)//El fotodiodo deja de envia datos cada 200ms *
	SCPI_COMANDO(INIACEL,IAC,pc_inicia_acelerometro)//El acelerometro envia datos cada 200ms *
 	SCPI_COMANDO(FINACEL,FNA,pc_fin_acelerometro)//El acelerometro deja de envia datos cada 200ms *
	SCPI_COMANDO(RESET,RS,pc_reset)//Pone la base en su estado inicial *
	SCPI_COMANDO(VARIABLES,VAR,pc_variables)//Pone las variables en un estado determinado *
	SCPI_COMANDO(VERSION,VER,pc_version)// Envía al PC la versión del software
	SCPI_COMANDO(ONDA,ON,pc_onda)//No la utiliza su software. Se programa para uso futuro
	SCPI_COMANDO(HABILITAFC,HF,pc_final_de_carrera)// Por compatibilidad. No hay finales de carrera
	SCPI_COMANDO(FLAGERRORFC,FE,pc_final_de_carrera) //Por compatibilidad. No hay finales de carrera
    // Pone el sistema en modo depuración. Solo para la fase de desarrollo 
	SCPI_COMANDO(MODODEPURACIONSI,MDS,modo_depuracion_si)
	SCPI_COMANDO(MODODEPURACIONNO,MDN,modo_depuracion_no)
};

MENU_SCPI  //menú de  comandos
{
	SCPI_SUBMENU(MOTORES,MOT)	//COMANDOS DEL PC
	//Comandos que ejecutan funciones definidas en la librería Segainvex_SCPI_Serial
	SCPI_COMANDO(ERROR,ERR,errorSCPI)// Envía el ultimo error
  	SCPI_COMANDO(*IDN,*IDN,idnSCPI)// Identifica el instrumento
	SCPI_COMANDO(*OPC,*OPC,opcSCPI)// Devuelve un 1 al PC
	SCPI_COMANDO(*CLS,*CLS,clsSCPI)// Borra la pila de errores
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
  "24 No hay sensor conectado",
  "25 Error en el sensor",
  "26 Falta la cantidad de envios a realizar",

};
/************************************************************************
    Objetos, constantes y variables de estado del sistema	(globales)	
************************************************************************/
/***********************************************************************
		Fotodiodo, acelerómetro y sensor de  temperatura-humedad
************************************************************************/
// Pendiente  y término independiente para ajustar la respuesta del ADC 
// float Vfotod=mFotoDiodo*ADC+bFotodiodo
// Ajuste para una Vref de 3.3V (por defecto) Si se cambia a externa 3V hay que reajustar 
// los valores de mFotoDiodo y bFotodiodo (ver cuaderno de taller "Patricio 9", pg 19 y 33)

//En general se pueden utilizar los valres calculados sin cometer un error mayor del 2%
float mFotoDiodo =-0.007120; //Valor calculado
float bFotoDiodo = 13.253; //Valor calculado

/*Base 20200148
//Parámetro b de calibración del fotodiodo. Valor empírico que sustituyen al valor calculado mFotoDiodo
float mFn =-0.00705; //Valor empírico
float mFl =-0.00702; //Valor empírico
float mSum =-0.007; //Valor empírico
//Parámetro b de calibración del fotodiodo.Valor empírico que sustituyen al valor calculado bFotoDiodo
float b_fn = 12.908;
float b_fl = 12.797;
float b_sum = 12.9646;
*/
/*Base 20191398. He utilizado los valores calculados y no hay error mayor del 2%*/
/*En el resto de las bases he utilizado el valor calculado y los errores no pasan del 2%*/

//Parámetro b de calibración del fotodiodo. Poner aquí el valor empírico (si se llega a calcular) que sustituyen al valor calculado mFotoDiodo
float mFn =-0.00712; //Valor empírico
float mFl =-0.00712; //Valor empírico
float mSum =-0.00712; //Valor empírico

//Parámetro b de calibración del fotodiodo. Poner aquí el valor empírico (si se llega a calcular) que sustituyen al valor calculado bFotoDiodo
float b_fn = 13.253;
float b_fl = 13.253;
float b_sum = 13.253;

//Acelerómetro MMA8452
MMA8452Q Acelerometro;
bool AcelerometroConectado=false;
//Retardos para el apagado y encendido del DC/DC de 48V
#define RETARDO_APAGADO 200 //ms
#define RETARDO_ENCENDIDO1 25 //ms
#define RETARDO_ENCENDIDO2 100 //ms
//Periodos de muestreo para el timer del ADC
#define TS_ADC_400us 400 //microsegundos
#define TS_ACD200us 200 //microsegundos
#define TS_ADC_300us 300 //microsegundos
bool FotoAcel;//Para indicar si se envian datos del fotodiodo o del acelerómetro
#define ACELEROMETRO 1
#define FOTODIODO 0
//Opciones para tiempo entre envios de señales de acelerómetro o fotodiodo
#define T300ms 300000 
#define T250ms 250000 
#define T200ms 200000 //Por defecto
#define T100ms 100000 
#define T150ms 150000 
#define T50ms   50000
int contadorEnvios=0; //Cantidad de cadenas con las señales del fotodiodo o acelerómetro a enviar al PC
#define ENVIOS_MAXIMOS 10000 //Contador de envios de fotodiodo o acelerómetro 60' si se envía cada 200ms
//Sensor de temperatura humedad. Instanciación del objeto 
Adafruit_BME280 bme; // Instancia el sensor para I2C
unsigned statusBME280;
String NombreDelSistema = "Base SPM"; //Puesto para depuración. Se puede quitar.
SegaSCPI BaseScpi(Raiz,"Base SPM",ErroresBaseSPM); //Objeto SCPI. Le indico el menú raiz, el nombre del sistema y el array de errores
bool StopPasos=false; //Flag para informar de que se han dado los pasos pedidos
char Version[]="Base SPM V1.3.2";//Las distintas "Bases SPM" pueden tener versión
//Variables para depuración. Activación con "depuracion" y puerto para depuración
bool depuracion=false; //1 para hacer depuración del software. 0 servicio 
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
#define LONG_MUESTRAS 50 // Con 50 muestras y Ts 400us lee un ciclo de red para filtrar 50Hz
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
//Macro para desactivar todos los motores
#define DESACTIVA_MOTORES\
	{digitalWrite(RELE_Z1,LOW);\
	digitalWrite(RELE_Z2,LOW);\
	digitalWrite(RELE_Z3,LOW);\
	digitalWrite(RELE_X,LOW);\
	digitalWrite(RELE_Y,LOW);\
	digitalWrite(RELE_HD,LOW);}
//Macro para la lectura del pin DSP_48V 
// TODO quitar toda alusión al pin DSP_48V
#define _DSP_48V 1 //Anulo la lectura de DSP_48 y fuerzo el valor a 1
// Serial
//Para no tener que teclear todo al aludir al puerto serie actual 
#define Println BaseScpi.PuertoActual->println 
#define Print BaseScpi.PuertoActual->print
#define debug Serial.print //Puerto mini USB
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
#define FFOTODIODO         "FT" //pc_inicia_fotodiodo(void); 
#define FTEMPERATURA       "T "  //pc_sensor_temperatura_humedad(void); 
#define FACELEROMETRO      "LC" //pc_inicia_acelerometro(void);
#define FVERSION           "KK" //pc_version(void); 
#define FIDN               "DW" //void idnSCPI(void); 
#define FSTOP              "ZP" //Mensaje de parada. Se envía para informar de parada de motor
#define FESTADO48V         "JJ" //La cadena es 1 o 0 estado del DC/DC activo o no
#define BME280             "UA" //Respuesta del sensor MBE280
/*****************************************************************
 *                          fin
 *****************************************************************/