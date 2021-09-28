/*
	UNIVERSIDAD AUTONÓMA DE MADRID
	SEGAINVEX: Electrónica
	OT: 20190335 
	Proyecto: Programación de la Base SPM con Arduino DUE 
	Aplicación para placa PCB_A con el Arduino DUE
	Patricio Coronado. Mayo de 2019
	Revisión Abril-mayo 2020
	Versión 1.2.1
	NOTAS 
	mejoras en esta versión:
	1-	Esta versión no contempla la programación del mando
		infrarojo.
	2- He programado el acelerómetro	
	3- He programado  acelerómetro	y fotodiodo enviando datos
		cada 150ms
	4- En las partes críticas se ha sustituido digitalWrite
		por una macro que cambia los pines 8 veces más rápido
	5- La frecuencia de paso (micropaso) ha subido de 90KHz
		a 120KHz y hay margen para subirla.


	25/05/2026
*/
/**************************************************************************
	Copyright © 2020 Patricio Coronado
	
	This file is part of Base SPM
    Base SPM is free software: you  can redistribute  it and/or modify 
	itunder the terms of the GNU General Public License as published by
	the Free Software Foundation, either version  3 of  the License, or
	(at your option) any later version.
	
    Base SPM is distributed in the  hope that it will be 
	useful,but WITHOUT ANY WARRANTY ; without even the implied warranty 
	of MERCHANTABILITY or FITNESS FOR A PARTICULARPURPOSE.  See the GNU 
	General Public License for more details.
    You  should have received a copy  of the GNU General Public License
    along with BaseSPM.c  If not, see <http://www.gnu.org/licenses/>
***************************************************************************/	
/*
		TO DO
	-1.	Hay un filtro fir diseñado para las señales del fotodiodo
		habría que implantarlo en este programa y comprobar tiempos
		de ejecución y efectividad. Hay que ajustar el filtro antialiasing
		con la capacidad del polo o con la frecuencia de muestreo
	-2.	Sería interesante una función que envie las señales del fotodiodo 
		y/o el acelerómetro con una tasa alta (he comprobado que podría llegar
		a 35ms)
	
	-3. Habría que estudiar los timers para ver si puedo subir la frecuencia de 
		micropaso por encima de 90KHz: Hecho. El problema es que la función
		digotalWrite() es muy lenta. Hay que hacerlo con unas macros que he
		programado con los que he bajado el pulso de step de 2.65us a 300ns.
		Ahora se puede bajar el periodo del timer. He conseguido que suba
		a 200KHz comunicando sin mucho impacto en la comunicación con el PC.
		De todas formas se va a quedar a 90 o 100 KHz.
*/

/***********************************************************************
	Sensor de humedad y temperatura a utilizar. Comentar el que no sea
	o los 2 si no se usa ninguno
************************************************************************/
//#define SENSOR_SHT11
#define SENSOR_DHT22
/**********************************************************************/
#include <Arduino.h>
#include "DueTimer.h"//Necesario para usar los timers con facilidad
#include <Wire.h>   //Para utilizar el I2C del acelerómetro
#include "SHT1x.h" //Sensor de humedad temperatura
#ifdef SENSOR_SHT11
	#include "SHT1x.h" //Sensor de humedad temperatura SHT11
#endif
#ifdef SENSOR_DHT22
	#include "DHT.h"//Sensor de humedad temperatura DHT22
#endif
#include "SparkFun_MMA8452Q.h"//acelerómetro MMA8452Q
#include "SegaSCPI.h"
#include "BaseSPM_V1_2_1_Diego.h"//Constantes, tipos, prototipos y variables globales
/***********************************************************************
 * 							SETUP
 ***********************************************************************/
void setup()
{
	//Configuración de pines------------------------------------------------------
	{
		//Driver piezomotores 
		pinMode(LM500_SHDWN,OUTPUT);     
		pinMode(P15V_ON,OUTPUT);     
		pinMode(RES0,OUTPUT);     
		pinMode(RES1,OUTPUT);     
		pinMode(MODE0,OUTPUT);     
		pinMode(MODE1,OUTPUT);     
		pinMode(SENTIDO,OUTPUT);
		pinMode(CLK,OUTPUT);
		//Test
		pinMode(TEST_SENSORHT,OUTPUT);
		pinMode(TEST_ACELEROMETRO,OUTPUT);
		pinMode(TEST_FOTODIODO,OUTPUT);
		pinMode(TEST_ADC,OUTPUT);
		pinMode(TEST_CLK,OUTPUT);
		//Leds
		pinMode(LED_BUILTIN,OUTPUT);     
		pinMode(LED0,OUTPUT); 
		pinMode(LED1,OUTPUT); 
		pinMode(LED2,OUTPUT); 
		pinMode(LED3,OUTPUT); 
		//Relés
		pinMode(RELE_X,OUTPUT);     
		pinMode(RELE_Y,OUTPUT);     
		pinMode(RELE_Z3,OUTPUT);     
		pinMode(RELE_Z2,OUTPUT);     
		pinMode(RELE_Z1,OUTPUT);
		pinMode(RELE_HD,OUTPUT);
		//Acelerómetros     
		pinMode(I2_1,OUTPUT);//Son interrupciones de acelerómetro que no se utilizan
		pinMode(I1_1,OUTPUT);//de momento
		//pinMode(I2_2,OUTPUT); //Voy a utilizar el Serial2 para el bluethoot
		//pinMode(I1_2,OUTPUT);
		//Sensor humedad temperatura SHT11
		pinMode(SEN_CLK,OUTPUT);
		pinMode(SEN_DATA,INPUT);
		// Infrarojo
		pinMode(IR_DATA,INPUT);//No se utiliza
		// Selección de motores en la cabeza
		pinMode(MHD1,OUTPUT);
		pinMode(MHD0,OUTPUT);
		// Salida 48V
		pinMode(SALIDA_48V,INPUT); //Entrada analógica para el ADC
		//DSP-Dulcinea
		pinMode(DSP_CLK,INPUT);//INPUT_PULLUP da problemas (ver cuaderno Patricio 9 pg 4)
		pinMode(DSP_48V,INPUT);
	}//Fin configuración de piner-------------------------------------------------
	//Inicializa al estado por defecto de las  variables del sistema--------------
	{
		parar_clk_step();//Para motor 
		// DC-DC de 48V
		Estado_48V=true; desactiva_48V();//Desactiva el DC-DC de 48V
		// Estado del módulo TMCM-090 por defecto
		cambia_onda(OMEGA564);//Valor de onda por defecto y no se cambia
		//Resolución y velocidad por defecto
		cambia_frecuencia_resolucion(1,2048);//Frecuencia a 1KHz
		//Motor seleccionado por defecto
		cambia_motor(NINGUNO);//Desactiva motores
		//Pasos programados y contador
		programa_pasos(0); //Pasos programados a cero
		cambia_sentido(SUBIR);//Sentido inicial
		Contador=0; //Contador de pasos. Se incrementa con cada step de clk interno Timer_CLK 
		//o externo, pin DSP_CLK. Se pone a cero cada vez que se pone en marcha un motor. Se
		//puede modificar con un comado SCPI
		//Interrupción del pin DSP_CLK clk externo proporcinado por el DSP
	}//Fin inicializa al estado por defecto de las  variables del sistema-----
	//Puertos serie Serial Serial1 y Serial2 ---------------------------------
	{
		Serial.begin(115200); //Programing port
		Serial1.begin(57600); //Comunicación con Dulcinea
		//Serial1.begin(115200); //Comunicación con Dulcinea
		Serial2.begin(9600);  //Comunicación con Android 
		//Serial2.begin(115200);  //Comunicación con Android CAMBIAR 
	}// Fin puertos serie Serial Serial1 y Serial2 ---------------------------
	// I2C y acelerómetro-----------------------------------------------------
	{
		Wire.begin();
		#define I2C_ACC Wire //Wire del acelerómetro scl y sda (Wire o Wire1)
  		Wire.setClock(MAX_FREC_I2C);//Velocidad del I2C 1MHz 
  		AcelerometroConectado = busca_acelerometro();//Si hay acelerómetro pone AcelerometroConectado a true
		//  if (Acelerometro.begin() == false){ AcelerometroConectado=NO;}
		//else{AcelerometroConectado=SI;}
	}
	// Convertidor AD, Timers e interrupciones externas ----------------------
	{
		// La interrupción del CLK_DSP se habilita solo si Frecuencia==0 en la función
		// Que cambia la MOT:AN ?
		//Aumentaba la velocidad del ADC (ya no, Arduino lo cambió)
		// REG_ADC_MR = (REG_ADC_MR & 0xFFF0F0FF) | 0x00020100;
		TIMER_CLK.attachInterrupt(timer_clk);//Timer para clk
		TIMER_ADC.attachInterrupt(timer_ADC);
		TIMER_FOTO_ACEL.attachInterrupt(timer_foto_acel);
		TIMER_ADC.start(500/*microsegundos*/); //Periodo de muestreo ADC
	}// Fin convertidor AD, Timers e interrupciones externas -----------------
}
/***********************************************************************
 * 							LOOP
 ***********************************************************************/
void loop() 
{
  	/* 
	BIT DSP_48V. Funcionamiento:
	Si el bit del DSP DSP_48V está a 0 y los 48V del TMCM-090 están activados
	para motores y desactiva los 48V
	Este bit se comprueba en:
	activa_48V() activar los 48V
	cambia_motor() al intentar cambiar el motor (activa el motor NINGUNO)
	marcha_paro_motor() poner en marcha motores
	y no se activan los 48V y no se ponen en marcha motores.
	 */
	if(Estado_48V && !_DSP_48V) //Situación de funcionamiento normal
	{
		marcha_paro_motor(PARO);
		desactiva_48V(); 
	}
	// Escucha los puertos
	// Escucha  Serial1 de Dulcinea
	if (Serial1.available())
	{
		LED0_1 //digitalWrite(LED0,HIGH);
		BaseScpi.scpi(&Serial1);	
		LED0_0 //digitalWrite(LED0,LOW);
	}// Escucha Serial2 del mando Bluetooth-Android
	if (Serial2.available())
	{
		LED0_1 //digitalWrite(LED0,HIGH);
		BaseScpi.scpi(&Serial2);	
		LED0_0 //digitalWrite(LED0,LOW);
	}//SCPI_SERIAL2 //Escucha al mando
	// Escucha el serial programing port
	if (Serial.available())
	{
		LED0_1 //digitalWrite(LED0,HIGH);
		BaseScpi.scpi(&Serial);	
		LED0_0 //digitalWrite(LED0,LOW);
	}
}
/**************************************************************************
*	busca el acelerómetro en las 2 direcciones 0x1c y 0x1d
	un par de veces en cada dirección
*************************************************************************/
bool busca_acelerometro(void)
{
	if (Acelerometro.begin(I2C_ACC, 0x1c) == true) return true;
	if (Acelerometro.begin(I2C_ACC, 0x1c) == true) return true;
	if (Acelerometro.begin(I2C_ACC, 0x1d) == true) return true;
	if (Acelerometro.begin(I2C_ACC, 0x1d) == true) return true;
	return false;
}
/************************************************************************
        Conjunto de funciones que tocan y/o programan las variable
		y los pines del sistema.
 ************************************************************************/
/************************************************************************
                Las tres funciones siguientes  son las 
				únicas que tocan el pin CLK (step del TMCM-090)
 ************************************************************************/
/************************************************************************
                INTERRUPCION DEL TIMER 4  para el ADC
				Tarda 18us, convierte cada 500us.
 ************************************************************************/
void timer_ADC()
{
	TEST_ADC_1 //digitalWrite(TEST_ADC,HIGH);
	FuerzaNormal.nuevoDato(analogRead(F_NORMAL)); //Lee  F.Normal
	FuerzaLateral.nuevoDato(analogRead(F_LATERAL)); //Lee F. Lateral
	Suma.nuevoDato(analogRead(SUM)); //Lee Suma
	TEST_ADC_0 //digitalWrite(TEST_ADC,LOW);
}
/************************************************************************
    INTERRUPCION DEL TIMER 3  para enviar datos del fotodiodo
	Tarda 325us en enviar los datos
	Función experimental
 ************************************************************************/
void timer_foto_acel()
{
	char respuesta[64];
	if(FotoAcel==FOTODIODO)
	{
		TEST_FOTODIODO_1 //PIN 38
		float fl,fn,sum;
		unsigned int fuerzaNormal,fuerzaLateral,suma;
		fuerzaNormal=FuerzaNormal.media();
		fn=-0.0062*fuerzaNormal+12.625;
		fuerzaLateral=FuerzaLateral.media();
		fl=-0.0064*fuerzaLateral+13.035;
		suma=Suma.media();
		sum=-0.0064*suma+12.948;
		//sprintf	(respuesta,"FOT %f %f %f",fn,fl,sum);
		sprintf(respuesta,"%s %.2f %.2f %.2f",FFOTODIODO,fn,fl,sum);
		Println(respuesta);
		TEST_FOTODIODO_0
	}
	else //ACELEROMETRO
	{
		if(AcelerometroConectado)
		{
			TEST_ACELEROMETRO_1
			if (Acelerometro.available()) 
			{      // Wait for new data from accelerometer
				float EjeX=Acelerometro.getCalculatedX(); 
				float EjeY=Acelerometro.getCalculatedY();
				//float EjeZ=Acelerometro.getCalculatedZ();
				//sprintf(respuesta,"AC %.4f %.4f %.4f",EjeX,EjeY,EjeZ);
				sprintf(respuesta,"%s %.3f %.3f",FACELEROMETRO,EjeX,EjeY);
				Println(respuesta);
			}
			TEST_ACELEROMETRO_0
		}
		else //Si no hay acelerómetro da error y detiene el timer
		{
			BaseScpi.errorscpi(23);
			TIMER_FOTO_ACEL.stop();
		}
		
	}//else //ACELEROMETRO
	//Si ha completado el número de envios para el timer
	if(--contadorEnvios <= 0)
	{
		TIMER_FOTO_ACEL.stop();		
	}
}
/************************************************************************
     INTERRUPCION DEL TIMER 5  para clk del step del módulo  TMCM-090
	 Con digitalWrite() tarda 2.6us 
	 Con las macros CLK_1 y CLK_0 tarda 260ns (thmin=100ns) suficiente,
	 con ese tiempo puedo alcanzar una frecuencia de 200KHz.
	 y ¡llega a 858KHz! aunque la comunicación se ralentiza pero funciona.
	 y si le pongo un delay de 1us tarda 1.26us
	 El datasheet del módulo TMCM-090 requiere 100ns mínimo
*************************************************************************/
void timer_clk()
{
		//Flanco de subida
		CLK_1 // set pin = digitalWrite(CLK,HIGH); pero más rápido
		Contador++;//Contamos un paso
		//Si estamos en modo pasos programados..
		if(Pasos)//y quedan pasos por dar... 
		{
			//Si llegan a cero los pasos programados para el clk del step
			if(--Pasos<=0) parar_clk_step();
		}
		//Flanco de bajada
		//delayMicroseconds(1);
		CLK_0  // clear pin = digitalWrite(CLK,LOW); pero más rápido
		
}
/************************************************************************
            INTERRUPCION DEL PIN DSP_CLK CON FLANCO FALLING
				Clk externo. Lo proporciona el DSP.
				//Da un pulso rising-falling en el pin CLK
 ************************************************************************/
void clk_externo(void)
{
	if(Frecuencia==0)//Si no es cero sale
	{	//Da un pulso rising-falling en el pin CLK
		//Flanco de subida
		CLK_1 // set pin = digitalWrite(CLK,HIGH); pero más rápido
		Contador++;//Contamos de pasos ascendente
		//Flanco de bajada
		delayMicroseconds(1);//Garantizo 1 microsegundo
		CLK_0 //Reset pin = digitalWrite(CLK,LOW); pero más rápido
	}
}    
/**************************************************************************
*	Activa o desactiva el timer del STEP del módulo TMCM-090
	Si el bit del DSP DSP_48V está a 0 no se pone en marcha
	Solo aquí se activan los 48V del TMCM-090, en caso de "MARCHA"
	Actualiza la variable EstadoMarchaParo.
	El estado del timer solo se cambia aquí.
***************************************************************************/
int marcha_paro_motor(unsigned int MarchaParo)
{
	//Impide que se active el timer del step si no hay motor activo
	if(MotorActivo==NINGUNO && MarchaParo==MARCHA)
	{
	 BaseScpi.errorscpi(22);//Ningún motor activo
	 return 0;
	}
	switch (MarchaParo)
	{
		case MARCHA:
		if(!_DSP_48V) //DSP_48V macro para leer pin o forzar DSP_48V = 1
		{
			BaseScpi.errorscpi(18);//Potencia desactivada por el DSP
			return 0; //Si el bit del DSP DSP_48V está a 0 no se pone en marcha
		}
		activa_48V(); //Activa la alimentación de potencia del módulo TMCM-090
		Contador=0;//Cada vez que se pone en marcha se resetea el contador
		//Si Frecuencia==0 el clk lo aporta el DSP por el pin CSP_CLK
		if (Frecuencia!=0) 
		{
			TIMER_CLK.start(Periodo[Frecuencia]);//Arranca el timer
			LED2_1//digitalWrite(LED2,HIGH);
			EstadoMarchaParo=MARCHA;		
		}
		break;
		case PARO:
			parar_clk_step();
		break;
	}
	return 1;
}
// Función inline para poder parar el timer del clk
inline void parar_clk_step(void) 
{
  		TIMER_CLK.stop();//Detiene el timer
		LED2_0//digitalWrite(LED2,LOW); 
		EstadoMarchaParo=PARO;
		//Al parar, hay que dejar el clk a LOW
		EstadoCLK=false;//Para informar del estado de la linea de clk
		CLK_0 //digitalWrite(CLK,LOW);
}
/**************************************************************************
 *	Cambia el motor/es activos
	Si el bit del DSP DSP_48V, está a cero, selecciona como motor NINGUNO
	Devuelve el Motor activo o -1 si se pidio un motor incorrecto
	Si hay que cambiar el motor y los 48 voltios están encendidos los
	apaga, cambia el motor y vuelve a encender los 48V.
	Actualiza la variable global  MotorActivo
 *************************************************************************/
int cambia_motor(unsigned int numMotor)
{
	
	bool FlagEstado48V=false;//Para ver si los 48 voltios están activados al entrar en la función
	//Si el bit del DSP DSP_48V, está a cero o la macro fuerza a 0, no selecciona motor
	if(!_DSP_48V) numMotor=NINGUNO; 
	if(numMotor < MIN_MOTOR || numMotor > MAX_MOTOR )//Test de rangos
	{
		 BaseScpi.errorscpi(9);//Motor incorrecto
		 return -1; 
	}
	if(numMotor == MotorActivo) return MotorActivo;//Ya está seleccionado el motor pedido
		//Cambia el motor activo
		if (Estado_48V)// Si hay que cambiar el motor y los 48 voltios están activos...
		{
			FlagEstado48V=true; //Para activar los 48V antes de salir
			desactiva_48V();
		}
	DESACTIVA_MOTORES //Macro para desactivar todos los motores
	switch (numMotor)
	{
		case Z1:
		digitalWrite(RELE_Z1,HIGH);
		break;
		case Z2:
		digitalWrite(RELE_Z2,HIGH);
		break;
		case Z3:
		digitalWrite(RELE_Z3,HIGH);
		break;
		case Z1Z2:
		digitalWrite(RELE_Z1,HIGH);
		digitalWrite(RELE_Z2,HIGH);
		break;
		case Z1Z3:
		digitalWrite(RELE_Z1,HIGH);
		digitalWrite(RELE_Z3,HIGH);
		break;
		case Z2Z3:
		digitalWrite(RELE_Z2,HIGH);
		digitalWrite(RELE_Z3,HIGH);
		break;
		case Z1Z2Z3:
		digitalWrite(RELE_Z1,HIGH);
		digitalWrite(RELE_Z2,HIGH);
		digitalWrite(RELE_Z3,HIGH);
		break;
		case X:
			digitalWrite(RELE_X,HIGH);
		break;
		case Y:
			digitalWrite(RELE_Y,HIGH);
		break;
		case FotodiodoX:// motor 10 (11)
			digitalWrite(RELE_HD,HIGH);
			digitalWrite(MHD1,HIGH);
			digitalWrite(MHD0,HIGH);// Motor 3 de la cabeza
		break;
		case FotodiodoY: //motor 13 (01)
			digitalWrite(RELE_HD,HIGH);
			digitalWrite(MHD1,HIGH);// digitalWrite(MHD1,LOW); //Cambiado
			digitalWrite(MHD0,LOW);//digitalWrite(MHD0,HIGH);  //Cambiado
		break;
		case LaserY: //motor 12 (10)
			digitalWrite(RELE_HD,HIGH);
			digitalWrite(MHD1,LOW);//digitalWrite(MHD1,HIGH); //Cambiado
			digitalWrite(MHD0,HIGH);// digitalWrite(MHD0,LOW);// Cambiado
			break;
		case LaserX: //motor 11 (00)
			digitalWrite(RELE_HD,HIGH);
			digitalWrite(MHD1,LOW);
			digitalWrite(MHD0,LOW);// Motor 0 de la cabeza
		break;
		case NINGUNO://Los motores se desactivaron al entrar en esta funcion
		break;
	}
	if(FlagEstado48V)//Si los 48V estaban encencidos y se apagaron al entrar se encienden
	{
			activa_48V();
	}
	return MotorActivo=numMotor;
}
/*************************************************************************
	Encendido y apagado de la alimentación del módulo TMCM-090
	Solo aquí se tocan los pines del DC/DC y se actualiza la 
	variable "Estado_48V"  
 * **********************************************************************/
void activa_48V(void)// Activa la fuente de 48V
{		 
	#define RETARDO_APAGADO 200 //ms
	#define RETARDO_ENCENDIDO 50 //ms
	/*
		 TO DO
		 ver como detectar que no se mueven
		 motores para desactivar los 48 voltios
		 automáticamente
	*/
	if(!_DSP_48V) return; //Si el bit del DSP está a 0, no se encienden los 48V
	 
	 if(Estado_48V) return;//Si las llamada es para encender 48V y ya lo estan, sale.
	 digitalWrite(P15V_ON,HIGH); //Pone 15V a la entrada de la fuete de 48V
	 delay(RETARDO_ENCENDIDO);//Espera antes de poner en marcha la fuente
	 digitalWrite(LM500_SHDWN,HIGH); //Activa la fuente de 48V 
	 delay(RETARDO_ENCENDIDO);//Espera antes de salir
	 LED1_1//digitalWrite(LED1,HIGH);
	 Estado_48V=true; //48 voltios encendidos
}
void desactiva_48V(void)// Desactiva la fuente de 48V
{
	if(Estado_48V)
	{
		digitalWrite(P15V_ON,LOW); //Corta los 15V de entrada
		digitalWrite(LM500_SHDWN,LOW);//Desactiva la fuente
		delay(RETARDO_APAGADO); //Para que se descargen los condensadores
		LED1_0//digitalWrite(LED1,LOW);
		Estado_48V=false; //48 voltios apagados
	}
}
/**************************************************************************
 *				 	Pone el modo de onda 
	En princio solo vamos a trabajar con OMEGA564. No se puede cambiar 
	a otro modo.
	Devuelve el modo de onda final o -1 si el modo de onda es erroneo o
	no permitido. Actualiza la variable global ModoDeOnda al modo actual
	
	TO DO
	SI SE PERMITIERA EN UN FUTURO CAMBIAR DE ONDA HABRÍA QUE COMPROBAR
	QUE LA RESOLUCION ES COMPATIBLE CON EL MODO DE ONDA ELEGIDO
	Los modos de onda posibles son:
	#define OMEGA564 3
	#define SINE1S85 2
	#define RHOMB    1  
	#define RHOMBF	 0
**************************************************************************/
 int cambia_onda(unsigned int ModoOn)
{ 
	// En principio solo vamos a trabajar con OMEGA564
// NO SE PERMITEN CAMBIOS DE ONDA.......................	
	if(ModoOn!=OMEGA564) 
	{	
		BaseScpi.errorscpi(20);
		return -1; 
	}
// SE TRABAJA SIEMPRE CON OMEGA564.....................
	// Actualiza los pines de modo de onda
	switch(ModoOn)
	{
		case OMEGA564:
				digitalWrite(MODE0,HIGH);
				digitalWrite(MODE1,HIGH);
		break;
		case SINE1S85:
				digitalWrite(MODE0,LOW);
				digitalWrite(MODE1,HIGH);
		break;
		case RHOMB:
				digitalWrite(MODE0,HIGH);
				digitalWrite(MODE1,LOW);
		break;
		case RHOMBF:
				digitalWrite(MODE0,LOW);
				digitalWrite(MODE1,LOW);
		break;
		default:
			BaseScpi.errorscpi(10);//Modo de onda no permitido no cambia nada
			return -1; 
		break;
	}
	ModoDeOnda=ModoOn;
	return ModoDeOnda;
}	
/*************************************************************************
*	Cambia la resolución y frecuencia del clk del módulo de potencia 
	Las variables globales "Frecuencia" y "Resolución" se cambian aquí.
	 
	 Si ambas variables están en rango comprueba la compatibilidad.
	El problema es que la frecuencia sea excesiva para la resolución.
	Lo que implica que la fecuencia de onda sea mayor que 
	Frecuencia/Resolucion > 234. Hay 2 opciones: Cambia la resolución
	y ajusta la velocidad a la máxima permitida por esa resolución o
	sale con error.
	La situación actual es que si la frecuencia es excesiva para la
	resolución, ajusta la frecuencia.
	Se ejecuta en 1.1ms
	Comando: MOT:FR <frecuencia>
*************************************************************************/
int cambia_frecuencia_resolucion(unsigned int Frec,unsigned int Res)
{
	//Testea las variables y si hay algo mal sale con error
	if(Frec > MAXIMA_FRECUENCIA || Frec < MINIMA_FRECUENCIA)
	{BaseScpi.errorscpi(12); return 0;}
	if(Res!=256 && Res!=512 && Res!=1024 && Res!=2048)
	{BaseScpi.errorscpi(8);return 0;}
	//Comprueba la compatibilidad
	switch(Res)
	{
		case 256:
			if(Frec>60)
			{
				Frec = 60;
				BaseScpi.errorscpi(16);//Frecuencia ajustada
			}
		break;
		case 512:
			if(Frec>120)
			{
				Frec = 120;
				BaseScpi.errorscpi(16);//Frecuencia ajustada
			}
		break;
	}
	//Para motores antes de cambiar nada
	TIMER_CLK.stop();
	switch(Res)//Cambia la variable global "Resolución"
		{ 
			case 2048:
				digitalWrite(RES0,HIGH);
				digitalWrite(RES1,HIGH);
			break;
			case 1024:
				digitalWrite(RES0,LOW);
				digitalWrite(RES1,HIGH);
			break;
			case 512:
				digitalWrite(RES0,HIGH);
				digitalWrite(RES1,LOW);
			break;
			case 256:
				digitalWrite(RES0,LOW);
				digitalWrite(RES1,LOW);
			break;
		}
	Frecuencia=Frec;//Actualiza la variable global "Frecuencia"
	Resolucion=Res; //Actualiza la variable global "Resolucion"
	//Si se estaba moviendo un motor lo deja moviendose
	//Si la frecuencia es cero hay que inhabilitar Timer3_CLK
	//y la interrupción del pin DSP_CLK está siempre habilitada
	//pero si la frecuencia no es cero sale de la función de interrupción
	/*
	TO DO
	Ver si es mejor habilitar e inhabilitar la interrupción del pin DSP_CLK
	*/
	if(Frecuencia==0)	
	{
		TIMER_CLK.stop();//Inhabilita interrupción del pin TIMER_CLK
		DSP_CLK_ON //Como la frecuencia es 0 el CLK lo controla del DSP
		return 1;
	}
	else 
		{
			DSP_CLK_OFF //Si la frecuencia no es 0 el CLK lo controla el TIMER_CLK
		}
	//Si había un motor en marcha arranca el timer del clk 
	if(EstadoMarchaParo==MARCHA)
	{
		TIMER_CLK.start(Periodo[Frecuencia]);
	}
	if(depuracion)
	{
		debug("ejecutado cambio_frecuencia_resolucion\r\n");
		debug("Frecuencia = ");debug(Frecuencia);debug("\r\n");
		debug("Periodo = ");debug(Periodo[Frecuencia]);debug("\r\n");	
		debug("Resolucion =");debug(Resolucion);debug("\r\n");
	}
	return 1;		
}
/*************************************************************************
 * Función que programa un número de pasos a dar
 * El contador de pasos se decrementa cada step dado por el 
 * Timer CLK interno, no con el externo. Cuando el contador llega a 0
 * se detiene el timer. Se programa su valor con varios comandos SCPI 
 * Se ejecuta en 1.5ms
 * Comando: MOT:AN <pasos>
*************************************************************************/
void programa_pasos(int pasos)
{
	if(Pasos >=0 && Pasos <=PASOS_MAXIMOS)	Pasos=pasos;
	else BaseScpi.errorscpi(15);
}
/**************************************************************************
*	Función que toca el pin SENTIDO. Cambian el sentido del movimiento
	según el parrámetro de entrada (1 o 0)) o devuelve el sentido actual. 
	Se ejecuta en 1ms
	Comando MOT:SE <0|1>
*************************************************************************/
int cambia_sentido(unsigned int Sen)
{
		switch (Sen)
	{
	case BAJAR: // Toca el pin de sentido para BAJAR
		Sentido = BAJAR; 
		digitalWrite(SENTIDO,BAJAR); 
	break;
	case SUBIR: //Toca el pin de sentido para SUBIR
		Sentido = SUBIR; 
		digitalWrite(SENTIDO,SUBIR); 
	break;
	}
	return Sentido;
}
/************************************************************************
        Fin del conjunto de funciones que tocan y/o programan las 
		variable y los pines del sistema.
 ************************************************************************/
/************************************************************************
		FUNCIONES SCPI QUE RESPONDEN A COMANDOS DEL PC
************************************************************************/
/**************************************************************************
  Función que responde al comando del pc para programar: un motor, 
  frecuencia, resolución, sentido, número de pasos y ponerlo en marcha.
  Se ejecuta en 2.2ms si no cambia motor y 302ms si cambia motor
  Comando  "MOT:MMP <MotorActivo Resolucion Frecuencia Sentido Pasos>"
**************************************************************************/
void pc_marcha_motor_pasos(void)
{
	unsigned int numParametros = 5;//Parametros esperados
	unsigned int np; // numero de parametros leido por sscanf
	//MotorActivo,Frecuencia,Sentido,Pasos
	unsigned int MotorActivo_,Resolucion_,Frecuencia_,Sentido_,Pasos_;
	char respuesta[128];
	
	// Si solo pregunta por los datos, se responde y sale
	if(BaseScpi.FinComando[0]=='?')	
	{
		sprintf(respuesta,"%s %u %u %u %u %u",FMARCHAMOTORPASOS,MotorActivo,Resolucion,Frecuencia,Sentido,Pasos);//Envía las variables globales del sistema
		Println(respuesta);
		 return;
	}
   		// Si el primer caracter de FinComado es 'espacio' lee parametros
  		if(BaseScpi.FinComando[0]==' ')  
	{   // Lee la cadena de parametros
	 	np = sscanf(BaseScpi.FinComando,"%u %u %u %u %u",&MotorActivo_,&Resolucion_,&Frecuencia_,&Sentido_,&Pasos_);
		if(np != numParametros){BaseScpi.errorscpi(6);return;}// Si no lee lo parametros esperados Error!
	} 	
	else {BaseScpi.errorscpi(5);return;} // Si el comando no empieza por 'espacio' Error!Parametro inexistente.
	// Si el número de parámetros leidos es correcto Procesa los datos recibidos
	// Primero tengo que comprobar que todo esta en rango. Si no lo esta, sale con error.
	//1 Motor
	if (MotorActivo_ < MIN_MOTOR || MotorActivo_ > MAX_MOTOR )
	{BaseScpi.errorscpi(9);return;}// sale con error:motor incorrecto
	//2 Resolución
	if(Resolucion_!=256 && Resolucion_!=512 && Resolucion_!=1024 && Resolucion_!=2048 )
		{BaseScpi.errorscpi(17);return;}//Resolución incorrecta
	//2 Frecuencia
	//La máxima frecuencia con mando es 64KHz (64000Hz/256 = 250Hz) 256=resolucion_mando
	if (!((Frecuencia_ <= MAXIMA_FRECUENCIA && Frecuencia_ >=  MINIMA_FRECUENCIA))) 
		{BaseScpi.errorscpi(12);return;}
	//4  Sentido
	if (!(Sentido_ == 0 || Sentido_ == 1) ){BaseScpi.errorscpi(13);return;}// error:parametro sentido incorrecto
	//Número de pasos a dar
	if (!(Pasos_ >= 0 && Pasos_ <=PASOS_MAXIMOS) ){BaseScpi.errorscpi(15);return;}// error:número de pasos incorrectos
	//
	// Si todos los parametros estan en rango...
	// Si frecuencia y resolución son compatibles actualiza todos los parámetros
	if (cambia_frecuencia_resolucion(Frecuencia_,Resolucion_))
	{
		cambia_motor(MotorActivo_); // Cambia motor
		cambia_sentido(Sentido_); // Cambia el sentido
		programa_pasos(Pasos_);//Programa el número de pasos
		marcha_paro_motor(MARCHA); // Al final pone el motor en marcha
	}
}
/**************************************************************************
  Función que responde al comando del pc para programar: un motor, 
  frecuencia, resolución, sentido y ponerlo en marcha.
  Se ejecuta en 2.2ms si no cambia motor y 302ms si cambia motor
  Comando  "MOT:MM <MotorActivo Resolucion Frecuencia Sentido>"
**************************************************************************/
void pc_marcha_motor(void)
{
	//Lee motor, resolución, frecuencia y sentido
	unsigned int numParametros = 4;//Parametros esperados
	unsigned int np; // numero de parametros leido por sscanf
	unsigned int parametro1,parametro2,parametro3,parametro4; // Son 4 parametros
	char respuesta[16];
	
	// Si solo pregunta por los datos, se responde y sale
	if(BaseScpi.FinComando[0]=='?'|| (BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?'))	
	{
		//Envía las variables globales del sistema
		//sprintf(respuesta,"MM %u %u %u %u",MotorActivo,Resolucion,Frecuencia,Sentido);
		sprintf(respuesta,"%s %u %u %u %u",FMARCHAMOTOR, MotorActivo,Resolucion,Frecuencia,Sentido);
		Println(respuesta);
		return;
	}
   		// Si el primer caracter de FinComado es 'espacio' lee parametros
  		if(BaseScpi.FinComando[0]==' ')  
	{   // Lee la cadena de parametros
	 	np = sscanf(BaseScpi.FinComando,"%u %u %u %u",&parametro1,&parametro2,&parametro3,&parametro4);
		if(np != numParametros){BaseScpi.errorscpi(6);return;}// Si no lee lo parametros esperados Error!
	} 	
	else {BaseScpi.errorscpi(5);return;} // Si el comando no empieza por 'espacio' Error!Parametro inexistente.
	// Si el numero de parAmetros leidos es correcto Procesa los datos recibidos
	// Primero tengo que comprobar que todo esta en rango. Si no lo esta, sale con error.
	//1 Motor
	if (parametro1 < 0 || parametro1 > 13 ) {BaseScpi.errorscpi(22);return;}// sale con error:motor inexistente
	//2 Resolución
	if(parametro2!=256 && parametro2!=512 && parametro2!=1024 && parametro2!=2048 )
		{BaseScpi.errorscpi(17);return;}//Resolución incorrecta
	//3 Frecuencia
	if (!((parametro3 <= MAXIMA_FRECUENCIA && parametro3 >=  MINIMA_FRECUENCIA) || !parametro3)) 
		{BaseScpi.errorscpi(12);return;}//Frecuencia incorrecta
	//4  sentido
	if (!(parametro4 == 0 || parametro4 == 1) ){BaseScpi.errorscpi(13);return;}// error:parametro sentido incorrecto
	//
	//  Si todos los parametros estan en rango...
	//Si son compatibles resolución Y frecuencia cambia los parámetros
	if(cambia_frecuencia_resolucion(parametro3,parametro2))
	{
		cambia_motor(parametro1); // Cambia motor
		cambia_sentido(parametro4); // Cambia el sentido
		marcha_paro_motor(MARCHA); // Al final activa el clk
	}
}
/**************************************************************************
  Función que envía  las 7 variables de la base.
  Se ejecuta en 1.12ms
  Comando  "MOT:VAR?" 
**************************************************************************/
void pc_variables(void)
{
	// Envía a la PC todas las variables
	char respuesta[64];
	if( BaseScpi.FinComando[0]=='?' || (BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?') )	
	{
		//Envía TODAS las variables globales del sistema
		//sprintf(respuesta,"VAR %u %u %u %u %u %u %u",MotorActivo,Resolucion,Frecuencia,Sentido,Pasos,EstadoMarchaParo,ModoDeOnda);
		sprintf(respuesta,"%s %u %u %u %u %u %u %u",FVARIABLES,MotorActivo,Resolucion,Frecuencia,Sentido,Pasos,EstadoMarchaParo,ModoDeOnda);
		Println(respuesta);
		return;
	}
	else {BaseScpi.errorscpi(4);return;}
}
/*************************************************************************
		Actualiza el valor de la variable contador a un valor
		entre 0 y CONTADOR_MAXIMO
		Se ejecuta en 1.42ms
		comando MOT:CO <contador>
 * **********************************************************************/
void pc_contador(void)
{
	int cont=Contador;//Por si el PC quiere leer el valor actual del dato
	char respuesta[16];
	//Si piden el dato con firma...
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"CO %u",Contador);
		sprintf(respuesta,"%s %u",FCONTADOR,Contador);
		Println(respuesta); 
    	return;
	}
	if (BaseScpi.actualizaVarEntera(&cont,CONTADOR_MAXIMO,0)==1)
	{
		Contador=cont;
	}
}
/*************************************************************************
		Programa un número de pasos
		En cada step (del TimerCLK no del pin DSP_CLK) se descuenta un paso.
		Cuando pasa de 1 a 0 se detiene el timerCLK.
		Se ejecuta en 1.4ms
		comando "MOT:AN <Pasos>"
 * **********************************************************************/
void pc_anda_numero_de_pasos(void)
{
	char respuesta[16];
	int pasos;
	//Si piden el dato con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"AN %u",Pasos);
		sprintf(respuesta,"%s %u",FANDANUMERODEPASOS,Pasos);
		Println(respuesta); 
    	return;
	}
	pasos = Pasos;// Por si el PC pregunta por el valor actual del dato
	//Lee el número de pasos y está en rango actualiza el contador "Pasos"
	if (BaseScpi.actualizaVarEntera(&pasos,PASOS_MAXIMOS,0)==1)
		programa_pasos(pasos);
}
/**************************************************************************
  Función que responde al comando del pc para mover o parar el motor activo
  Se ejecuta en 1ms
  Comando: MOT:MP <1|0> 1=Marcha, 0=Paro 
**************************************************************************/
void pc_marcha_paro(void)
{
	char respuesta[16];
	int maPa=EstadoMarchaParo;
	//Si pregunta por la variable con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"MP %u",EstadoMarchaParo);
		sprintf(respuesta,"%s %u",FMARCHAPARO,EstadoMarchaParo);
		Println(respuesta); 
    	return;
	}
	//Si lee el parámetro sin errores y es correcto actualiza el estado
	if (BaseScpi.actualizaVarEntera(&maPa,1,0)==1)
	{
		//LLama a la función que arranca o para el timer del clk
		marcha_paro_motor(maPa);
	}
}
/************************************************************************
		Función para cambiar el sentido
		Lee el parámetro sentido y si es correcto lo cambia
		Se ejecuta en 1ms
		comando "MOT:SE <Sentido>"
************************************************************************/
void pc_sentido(void)
{
	char respuesta[16];
	int sent=Sentido;//Por si hay que enviar el valor al PC
	//Si pregunta por la variable con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"SE %u",Sentido);
		sprintf(respuesta,"%s %u",FSENTIDO,Sentido);
		Println(respuesta); 
    	return;
	}
	//Si lee el parámetro sin errores y es correcto actualiza el sentido
	if (BaseScpi.actualizaVarEntera(&sent,1,0)==1)
	{
		//LLama a la función que toca el pin SENTIDO
		cambia_sentido(sent);
	}
}
/************************************************************************
		Función para cambiar la frecuencia clk del step
		Lee el parámetro frecuencia y si es correcto lo cambia
		Se ejecuta en 1ms
		Comando  "MOT:FR <Frecuencia>"
************************************************************************/
void pc_frecuencia(void)
{
	char respuesta[16];
	int frec=Frecuencia;//Por si hay que enviar el valor al PC
	//Si pregunta por la variable con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"FR %u",Frecuencia);
		sprintf(respuesta,"%s %u",FFRECUENCIA,Frecuencia);
		Println(respuesta); 
    	return;
	}
	//Si lee el parámetro sin errores y es correcto actualiza el sentido
	if (BaseScpi.actualizaVarEntera(&frec,MAXIMA_FRECUENCIA,MINIMA_FRECUENCIA)==1)
	{
		//Frecuencia nueva resolución actual
		cambia_frecuencia_resolucion(frec,Resolucion);
	}
}
/************************************************************************
		Función para cambiar el motor activo
		Se ejecuta en 300ms cuando cambiar de motor
		Comando  "MOT:MA <Motor>"
************************************************************************/
void pc_motor_activo(void)
{
	int mot=MotorActivo;//Por si hay que enviar el valor al PC
	char respuesta[16];
	//Si pregunta por la variable con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"MA %u",MotorActivo);
		sprintf(respuesta,"%s %u",FMOTORACTIVO,MotorActivo);
		Println(respuesta); 
    	return;
	}
	//Si lee el parámetro sin errores y es correcto actualiza el motor
	if (BaseScpi.actualizaVarEntera(&mot,MAX_MOTOR ,MIN_MOTOR)==1)
	cambia_motor(mot);
}
/************************************************************************
	Función para cambiar la resolución
	El comando se procesa en 1,3ms
	Comando  "MOT:RE <Resolucion>"
************************************************************************/
void pc_resolucion(void)
{
	int Res=Resolucion;//Por si hay que enviar el valor al PC
	int Resoluciones[]={256,512,1024,2048};
	int resultado;
	char respuesta[16];
	//Si pregunta por la variable con firma...
	if(BaseScpi.FinComando[0]==' '&& BaseScpi.FinComando[1]=='?')
	{
		//sprintf(respuesta,"RE %u",Resolucion);
		sprintf(respuesta,"%s %u",FRESOLUCION,Resolucion);
		Println(respuesta);
		return;
	}
	//Si lee el parámetro sin errores y es correcto actualiza el motor
	resultado=BaseScpi.actualizaVarDiscreta(&Res,Resoluciones,4);
	//Frecuencia actual Resolución nueva
	if (resultado==1 && (unsigned int)Res!=Resolucion) cambia_frecuencia_resolucion(Frecuencia,Res);
	if(resultado==0) BaseScpi.errorscpi(8); //Resolución incorrecta 
}
/************************************************************************
		Función para cambiar el modo de onda
		Esta función no va a estar accesible al usuario
		cambia_onda(onda); solo cambia a OMEGA564
		Comando  "MOT:ON <Onda>"
************************************************************************/
void pc_onda(void)
{
	int resultado;
	int onda=ModoDeOnda;//Por si hay que enviar el valor al PC
	int ModosDeOnda[]={RHOMBF,RHOMB,SINE1S85,OMEGA564};
	char respuesta[16];
	//Si pregunta por la variable con firma
	if(BaseScpi.FinComando[0]==' ' && BaseScpi.FinComando[1]=='?')  
	{
		//sprintf(respuesta,"ON %u",ModoDeOnda);
		sprintf(respuesta,"%s %u",FONDA,ModoDeOnda);
		Println(respuesta); 
    	return;
	}
	resultado=BaseScpi.actualizaVarDiscreta(&onda,ModosDeOnda,4);
	//Si lee el parámetro sin errores y es correcto actualiza el motor
	if (resultado ==1 && (unsigned int)onda!=ModoDeOnda)
	cambia_onda(onda);
	if(resultado==0) BaseScpi.errorscpi(19); //Onda incorrecta 
}
/************************************************************************
 * Funciones para hacer que envíe las señales del diodo cada 
 * 250ms y para que deje de hacerlo.
 * Comandos MOT:IFO inicia y MOT:FFO finaliza
 * la función de interrupción del timer es timer_foto_acel()
 * **********************************************************************/
void pc_inicia_fotodiodo(void)
{
	int envios;
	//Lee el número de envios a realizar
	if (sscanf(BaseScpi.FinComando,"%u",&envios)==1)
		contadorEnvios=envios; //Inicaliza el contador de envios a realizar;
	else contadorEnvios=1;//Si no envía el número de envios se realiza 1;	
	FotoAcel = FOTODIODO;
	TIMER_FOTO_ACEL.start(T250ms);//250ms	
}
void pc_fin_fotodiodo(void)
{
	TIMER_FOTO_ACEL.stop();
}
/************************************************************************
 * Funciones para hacer que envíe las señales del acelerometro cada 
 * 250ms y para que deje de hacerlo.
 * Comandos MOT:IAC inicia y MOT:FAC finaliza
 * * la función de interrupción del timer es timer_foto_acel()
 * **********************************************************************/
void pc_inicia_acelerometro(void)
{
	int envios;
	//Lee el número de envios a realizar
	if (sscanf(BaseScpi.FinComando,"%u",&envios)==1)
		contadorEnvios=envios; //Inicaliza el contador de envios a realizar;
	else contadorEnvios=1;//Si no envía el número de envios se realiza 1	
	FotoAcel = ACELEROMETRO;
	
	TIMER_FOTO_ACEL.start(T250ms);//250ms	
}
void pc_fin_acelerometro(void)
{
	TIMER_FOTO_ACEL.stop();
}
/*************************************************************************
		Envía las señales del fotodiodo al PC 
		El comando se procesa en 1,2ms
		Comando MOT:FOT?
 * **********************************************************************/
void pc_fotodiodo(void)
{
	if(BaseScpi.FinComando[0]!='?')  
	{
		BaseScpi.errorscpi(4);//Parámetro inexistente
		return;
	}
	char respuesta[64];
	float fl,fn,sum;
	unsigned int fuerzaNormal,fuerzaLateral,suma;
	fuerzaNormal=FuerzaNormal.media();
	fn=-0.0062*fuerzaNormal+12.625;
	fuerzaLateral=FuerzaLateral.media();
	fl=-0.0064*fuerzaLateral+13.035;
	suma=Suma.media();
	sum=-0.0064*suma+12.948;
	//sprintf(respuesta,"FOT %.2f %.2f %.2f",fn,fl,sum);
	sprintf(respuesta,"%s %.2f %.2f %.2f",FFOTODIODO,fn,fl,sum);
	Println(respuesta);
}
/*************************************************************************
		Lee el sensor de temperatura y humedad y lo envia al pc
		el DHT-22 tarda 5,68ms
		El comando se procesa en 6,32ms. No se puede bajar
		Comando MOT:TH?
 * **********************************************************************/
void pc_sensor_temperatura_humedad(void)
{
	if(BaseScpi.FinComando[0]!='?')  
	{
		BaseScpi.errorscpi(4);//Parámetro inexistente
		return;
	}
	TEST_SENSORHT_1 //digitalWrite(TEST_SENSORHT,HIGH);
	if(EstadoMarchaParo==MARCHA)//moviendo motores no lee el sensor
	{
		//sprintf(Datos,"T%5.1f H%5.1f",0.0,0.0);
		Println("T 0.0 H 0.0");
		return;
	}
	TIMER_ADC.stop();//Este comando desactiva las interrupciones
	LED3_1 //digitalWrite(LED3,HIGH);//Para test
#ifdef SENSOR_SHT11
	char Datos[64];
	//SHT1x SHT11(SEN_DATA, SEN_CLK);
	float Temperatura;
	float Humedad;
	Temperatura = SHT11.readTemperatureC();
	Humedad = SHT11.readHumidity();
	sprintf(Datos,"T%5.1f H%5.1f",Temperatura,Humedad);
	// Envía valores por el puerto
	Println(Datos)
#endif
#ifdef SENSOR_DHT22
	char Datos[64];	
	//DHT dht(SEN_DATA, DHT22);
	float Humedad = dht.readHumidity();
  	//delay(100);// TO DO intentar bajar 
  	float Temperatura = dht.readTemperature();
  	sprintf(Datos,"T%5.1f H%5.1f",Temperatura,Humedad);
	// Check if any reads failed and exit early (to try again).
  	if (isnan(Humedad) || isnan(Temperatura))
	{
    	BaseScpi.errorscpi(21);//Error lectura sensor humedad temperatura
	}
	Println(Datos);
#endif
	LED3_0//digitalWrite(LED3,LOW);
	TEST_SENSORHT_0 //digitalWrite(TEST_SENSORHT,LOW);
	TIMER_ADC.start();
}
/*************************************************************************
		Lee el acelerómetro y lo envia al pc
		El comando se procesa en 1,58ms
		el MMA8452 tarda 488us
		Comando MOT:AC?
 * **********************************************************************/
void pc_acelerometro(void)
{
	float EjeX,EjeY; //
	//float EjeZ;
	char respuesta[64];
	TEST_ACELEROMETRO_1
	if(BaseScpi.FinComando[0]!='?')
	{
		BaseScpi.errorscpi(4);
		TEST_ACELEROMETRO_0 
		return;
	}//Falta el parámetro
	if(!AcelerometroConectado)
	{
		BaseScpi.errorscpi(23);
		TEST_ACELEROMETRO_0 
		return;
	}//No hay acelerómetro conectado
	if (Acelerometro.available()) 
	{      // Wait for new data from accelerometer
		EjeX=Acelerometro.getCalculatedX(); 
		EjeY=Acelerometro.getCalculatedY();
		//EjeZ=Acelerometro.getCalculatedZ();
		//sprintf(respuesta,"AC %.2f %.2f %.2f",EjeX,EjeY,EjeZ);
		//sprintf(respuesta,"AC %f %f",EjeX,EjeY);
		sprintf(respuesta,"%s %.3f %.3f",FACELEROMETRO,EjeX,EjeY);
		Println(respuesta);
	}
	else 
	{
		BaseScpi.errorscpi(23);
		TEST_ACELEROMETRO_0 
		return;
	}//No hay acelerómetro conectado
	TEST_ACELEROMETRO_0
}
/**************************************************************************
  Función que atiende al comando del pc que pone el sistema en el 
  estado inicial.
  Se ejecuta en 735us
  Comando  "MOT:RS" 
**************************************************************************/
void pc_reset(void)
{
	//cambia_onda(unsigned int);
	parar_clk_step();//Paro de motor
	desactiva_48V();// Desactiva la fuente de 48V
	cambia_frecuencia_resolucion(1,2048);
	cambia_motor(NINGUNO);
	cambia_sentido(SUBIR);
	cambia_onda(OMEGA564);
	programa_pasos(0);//Programa el número de pasos
	Contador=0;
}	
/************************************************************************
	Envía al PC la versión de software de Arduino
	Se ejecuta en 919us
	Comando: MOT:VER?
************************************************************************/
void pc_version(void)
{
	String respuesta = FVERSION+(String)" "+(String)Version;
	if(BaseScpi.FinComando[0]=='?') Println(respuesta);
	else BaseScpi.errorscpi(4);//Parámetro inexistente
}
/************************************************************************
	Por compatibilidad. No hay finales de carrera en el sistema
	Se ejecuta en 730us
	MOT:HF o MOT:FE
************************************************************************/
void pc_final_de_carrera(void){return;}
/************************************************************************
		FIN FUNCIONES SCPI QUE RESPONDEN A COMANDOS DEL PC
************************************************************************/
/************************************************************************
		FUNCIONES QUE RESPONDEN A COMANDOS DEL BLUETOOTH
************************************************************************/
/************************************************************************
		Función que envia los valores adquiridos por el ADC
		Fuerza Nomal, Lateral, y Suma. Además del estado de marcha
		paro del sistema y el contador de pasos.
		Se ejecuta en 1.23ms
		comando BTH:EST
************************************************************************/
void  bluetooth_estado(void)
{
	char respuesta[256];//Si pongo la cadena más corta no cabe todo
	float fl,fn,sum;
	unsigned int fuerzaNormal,fuerzaLateral,suma,emp;
	//calcula las señales con ajuste de offset y ganancia 
	fuerzaNormal=FuerzaNormal.media();
	fn=-0.0062*fuerzaNormal+12.625;
	fuerzaLateral=FuerzaLateral.media();
	fl=-0.0064*fuerzaLateral+13.035;
	suma=Suma.media();
	sum=-0.0064*suma+12.948;
	if(EstadoMarchaParo) emp=10;
	else emp=5;
	sprintf	(respuesta,"EST %f %f %f %u %u",fn,fl,sum,emp,Pasos);
	Println(respuesta);
}
/**************************************************************************
  	Comando para seleccionar un motor activo, frecuencia,sentido
	y pone el motor en marcha. 
	La resolución con el mando es siempre 256.
	Se ejecuta en 2.5ms si no cambia motor y 300ms si cambia motor
	comando BTH:MARCHA <motor frecuencia sentido pasos>
   	Parámetros esperados:
	 motor (entero);
 	0=NINGUNO, 1=Z1, 2=Z2, 3=Z3, 4=(Z1&Z2),5=(Z1&Z3),6=(Z2&Z3), 7=(Z1&Z2&Z3),
 	8=X, 9=Y, 10=FotodiodoX, 11=LaserX, 12=LaserY, 13=FotodiodoY
  	frecuencia (entero) 10KHz =<frecuncia=< 200 KHz
  	Sentido (entero): 0=BAJAR/IZQUIERDA, 1=SUBIR/DERECHA.
	Pasos entre 0 y PASOS_MAXIMOS	  
**************************************************************************/
void bluetooth_marcha_motor(void)
{
	unsigned int numParametros = 4;//Parametros esperados
	unsigned int np; // numero de parametros leido por sscanf
	//MotorActivo,Frecuencia,Sentido,Pasos
	unsigned int parametro1,parametro2,parametro3,parametro4; // Son 4 parametros
	char respuesta[16];
	
	// Si solo pregunta por los datos, se responde y sale
	if(BaseScpi.FinComando[0]=='?')	
	{
		sprintf(respuesta,"%u %u %u %u",MotorActivo,Frecuencia,Sentido,Pasos);//Envía las variables globales del sistema
		Println(respuesta);
		 return;
	}
   		// Si el primer caracter de FinComado es 'espacio' lee parametros
  		if(BaseScpi.FinComando[0]==' ')  
	{   // Lee la cadena de parametros
	 	np = sscanf(BaseScpi.FinComando,"%u %u %u %u",&parametro1,&parametro2,&parametro3,&parametro4);
		if(np != numParametros){BaseScpi.errorscpi(6);return;}// Si no lee lo parametros esperados Error!
	} 	
	else {BaseScpi.errorscpi(5);return;} // Si el comando no empieza por 'espacio' Error!Parametro inexistente.
	// Si el numero de parAmetros leidos es correcto Procesa los datos recibidos
	// Primero tengo que comprobar que todo esta en rango. Si no lo esta, sale con error.
	//1 Motor
	if (parametro1 < MIN_MOTOR || parametro1 > MAX_MOTOR )
	{BaseScpi.errorscpi(9);return;}// sale con error:motor incorrecto
	//2 Frecuencia
	//La máxima frecuencia con mando es 64KHz (64000Hz/256 = 250Hz) 256=resolucion_mando
	if (!((parametro2 <= MAXIMA_FRECUENCIA_BTH && parametro2 >=  MINIMA_FRECUENCIA))) 
		{BaseScpi.errorscpi(12);return;}
	//4  Sentido
	if (!(parametro3 == 0 || parametro3 == 1) ){BaseScpi.errorscpi(13);return;}// error:parametro sentido incorrecto
	//Número de pasos a dar
	if (!(parametro4 >= 0 && parametro4 <=PASOS_MAXIMOS) ){BaseScpi.errorscpi(15);return;}// error:número de pasos incorrectos
	//
	// Si todos los parametros estan en rango...
	// Si frecuencia y resolución son compatibles actualiza todos los parámetros
	if (cambia_frecuencia_resolucion(parametro2,RES_BTH))
	{
		cambia_motor(parametro1); // Cambia motor
		cambia_sentido(parametro3); // Cambia el sentido
		programa_pasos(parametro4);//Programa el número de pasos
		marcha_paro_motor(MARCHA); // Al final pone el motor en marcha
	}
}
/**************************************************************************
  Función para el comando del bluetooth para  parar motor
  Se ejecuta en 1ms
  comando BTH:PARO
**************************************************************************/
void bluetooth_para_motor(void)
{
	marcha_paro_motor(PARO); 
	Println(FSTOP);
}
/************************************************************************
		FIN DE FUNCIONES QUE RESPONDEN A COMANDOS DEL BLUETOOTH
************************************************************************/
/************************************************************************
		FUNCIONES DE TEST QUE RESPONDEN A COMANDOS DEL COMPUTADOR
************************************************************************/
//Salir o entrar en modo depuración
void modo_depuracion_no(void){depuracion=false;}
void modo_depuracion_si(void){depuracion=true;}
// Relés
void desactiva_rele_z1(void){digitalWrite(RELE_Z1,LOW);}
void activa_rele_z1(void){digitalWrite(RELE_Z1,HIGH);}
void desactiva_rele_z2(void){digitalWrite(RELE_Z2,LOW);}
void activa_rele_z2(void){digitalWrite(RELE_Z2,HIGH);}
void desactiva_rele_z3(void){digitalWrite(RELE_Z3,LOW);}
void activa_rele_z3(void){digitalWrite(RELE_Z3,HIGH);}
void desactiva_rele_y(void){digitalWrite(RELE_Y,LOW);}
void activa_rele_y(void){digitalWrite(RELE_Y,HIGH);}
void desactiva_rele_x(void){digitalWrite(RELE_X,LOW);}
void activa_rele_x(void){digitalWrite(RELE_X,HIGH);}
void desactiva_rele_hd(void){digitalWrite(RELE_HD,LOW);}
void activa_rele_hd(void){digitalWrite(RELE_HD,HIGH);}	
//Leds
void desactiva_led_3(void){digitalWrite(LED3,LOW);}
void activa_led_3(void){digitalWrite(LED3,HIGH);}
void desactiva_led_2(void){digitalWrite(LED2,LOW);}
void activa_led_2(void){digitalWrite(LED2,HIGH);}
void desactiva_led_1(void){digitalWrite(LED1,LOW);}
void activa_led_1(void){digitalWrite(LED1,HIGH);}
void desactiva_led_0(void){digitalWrite(LED0,LOW);}
void activa_led_0(void){digitalWrite(LED0,HIGH);}
// Driver TCMC90
void desactiva_res_1(void){digitalWrite(RES1,LOW);}
void activa_res_1(void){digitalWrite(RES1,HIGH);}
void desactiva_res_0(void){digitalWrite(RES0,LOW);}
void activa_res_0(void){digitalWrite(RES0,HIGH);}
void activa_mode_0(void){digitalWrite(MODE0,HIGH);}
void desactiva_mode_0(void){digitalWrite(MODE0,LOW);}
void activa_mode_1(void){digitalWrite(MODE1,HIGH);}
void desactiva_mode_1(void){digitalWrite(MODE1,LOW);}
void activa_dir(void){digitalWrite(SENTIDO,HIGH);}
void desactiva_dir(void){digitalWrite(SENTIDO,LOW);}
void activa_clk(void){digitalWrite(CLK,HIGH);}
void desactiva_clk(void){digitalWrite(CLK,LOW);}
// Motores de la cabeza
void desactiva_motor_head_1(void){digitalWrite(MHD1,LOW);}
void activa_motor_head_1(void){digitalWrite(MHD1,HIGH);}
void desactiva_motor_head_0(void){digitalWrite(MHD0,LOW);}
void activa_motor_head_0(void){digitalWrite(MHD0,HIGH);}
//Acelerómetros
void desactiva_i22(void){digitalWrite(I2_2,LOW);}	
void activa_i22(void){digitalWrite(I2_2,HIGH);}
void desactiva_i21(void){digitalWrite(I2_1,LOW);}
void activa_i21(void){digitalWrite(I2_1,HIGH);}
void desactiva_i12(void){digitalWrite(I1_2,LOW);}
void activa_i12(void){digitalWrite(I1_2,HIGH);}
void desactiva_i11(void){digitalWrite(I1_1,LOW);}
void activa_i11(void){digitalWrite(I1_1,HIGH);}
// Movimiento de motores
void mueve_motor(void){TIMER_CLK.start(20 /* microsegundos */);} //Mueve motor	
void para_motor(void){TIMER_CLK.stop();} //Para motor		
void test_step(void)
{
	int per;
	if(BaseScpi.actualizaVarEntera(&per,1000,1)==1)
	 TIMER_CLK.start(per /* microsegundos */);
}
/************************************************************************
		FIN DE FUNCIONES QUE RESPONDEN A COMANDOS DEL COMPUTADOR
************************************************************************/
/************************************************************************
    Funciones scpi comunes a todos los sistemas
 *************************************************************************/
 /************************************************************************
    Función del Comando: ERROR ó ERR
    Envia por el puerto el último error registrado por SEGAINVEX-SCPI
 *************************************************************************/
void errorSCPI(void){BaseScpi.errorscpi(0);}
/*************************************************************************
  Función del Comando: *IDN"
   Envia por el puerto una cadena que identifica al sistema
   Se ejecuta en 1ms
 *************************************************************************/
void idnSCPI(void){BaseScpi.enviarNombreDelSistema();}
 /************************************************************************
  Función del Comando:*OPC
  Envia por el puerto el carácter uno
  Se ejecuta en 1ms
  *************************************************************************/
void opcSCPI(void){	BaseScpi.PuertoActual->println("1");}
/*************************************************************************
    Comando: CLS
    Limpia la pila de errores de SEGAINVEX-SCPI
	Se ejecuta en 430us
 *************************************************************************/
void clsSCPI(void){BaseScpi.errorscpi(-1);}
/*************************************************************************/