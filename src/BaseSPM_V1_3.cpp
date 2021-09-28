/*
	UNIVERSIDAD AUTONÓMA DE MADRID
	SEGAINVEX: Electrónica
	OT: 20190335, 20191136 
	Proyecto: Programación de la Base SPM con Arduino DUE 
	
	Versión 1.3 16/05/2020
	
	Aplicación para placa PCB_A con el Arduino DUE
	Patricio Coronado. Mayo de 2019
	Revisión Abril-mayo 2020
	Versión 1.2
	NOTAS 
	¡¡MUY IMPORTANTE!!
	Antes de flasear asegurarse de que los puertos se inicializan
	con los baudios correctos. El primer prototipo tiene el 
	Bluetooth a 9600 baudios. El resto a 115200 con el nº
	de orden en el nombre del Bluetooth BASE_SPM_XXXXXXXX 

	mejoras sobre la versión 1_1:
	
	-1.	Esta versión no contempla la programación del mando
		infrarojo. 
	-2. He programado el acelerómetro	
	-3. He programado  acelerómetro	y fotodiodo enviando datos
		cada 200ms
	-4. En las partes críticas se ha sustituido digitalWrite
		por una macro que cambia los pines 8 veces más rápido
	-5. La frecuencia de paso (micropaso) ha subido de 90KHz
		a 120KHz y hay margen para subirla.
	-6. Anulo la función del pin del DSP DSP_48V 
	-7. La comunicación con el mando se hace por Bluetooth
	
	mejoras sobre la versión 1_2:
	
	-1.	El sensor de humedad y temperatura va sobre I2C aunque
		hay que seguir probando dispositios.
		
			
	
	Ultimas modificaciones (experimental):
	
	-1.	He modificado las firmas de las cadenas. Ahora son dos mayúsculas 
		salvo temperatura-humedad que no lo he cambiado por compatibilidad.
	-2.	El envío periódico de fotodiodo y acelerómetro tiene un parámetro, 
		el número de envios.
	-3. El sensor AHT10 se maneja con el i2c Wire1 directamente sin librería.
	-4  He añadido el sensor BME280 para sustituir al AHT10. Ahora los dos
		sensores, acelerómetro y humedad/temperatura pueden ir en el mismo i2c
		y van en el Wire. El Wire1 no se inicializa, asi el Serial3 está
		disponible.
	-5. He bajado el periodo de muestreo del fotodiodo a 400us (2,5KHz). 
	-6. Los 48V se activan o desactivan desde el PC o Android (experimental)
	-7. He redefinido los pines DSP_CLK y DSP_48V. Así:	 
		#define DSP_CLK A8 //cambiado, antes era 5.16/09/2020
		#define DSP_48V A9// cambiado, antes era 4.16/09/2020
		Ahora las señales del DSP llegan a través de un integrado puesto
		en el backplane que adminte señales del DSP de 5 y 3,3V y su
		salida es de 3,3V.

TO DO
	-0. Antes de flasear para la base definitiva verificar que 
		los baudios para Dulcinea son 57600 y para el bluetooth
		según la base 9600 para la primera y 115200 el resto
	

	-1.	Quitar el código de los sensores de humedad y temperatura obsoletos
	-2. El Probar el Serial3 (es incompatible con el uso del Wire1).
	-3. Reconectar el Serial3 con el Serial. 
	-4. Probar el Seria con un Bluetooth, se puede utilizar aunque
		sea el utilizado por el programing port. 
	-5.	Alimentar los sensores I2C con 3V3 voltios distintos del DUE.
	-6. Utilizar el sensor BME280 sin librería 

	
	Ultimos cambios en el código 27/09/2020
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
		
	
*/
/**********************************************************************/
#include <Arduino.h>
#include "DueTimer.h"//Necesario para usar los timers con facilidad
#include <Wire.h>   //Para utilizar el I2C del acelerómetro y otros dipositivos
#include "SHT1x.h" //Sensor de humedad temperatura SHT11
#include "DHT.h"//Sensor de humedad temperatura DHT22
#include "SparkFun_MMA8452Q.h"//acelerómetro MMA8452Q
#include "Adafruit_Sensor.h"
#include "PacoAdafruit_BME280.h"//Modificado por mi
#include "SegaSCPI.h"
#include "BaseSPM_V1_3.h"//Constantes, tipos, prototipos y variables globales
/***********************************************************************
 * 							SETUP
 ***********************************************************************/
void setup()
{
	{//Configuración de pines--------------------------------------------------
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
		pinMode(LED4,OUTPUT); 
		pinMode(LED5,OUTPUT); 
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
	{//Inicializa al estado por defecto de las  variables del sistema----------
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
		//Leds 4 y 5 a 0
		LED4_0
		LED5_0
	}//Fin inicializa al estado por defecto de las  variables del sistema-----
	{//Puertos serie Serial Serial1 y Serial2 ---------------------------------
		Serial.begin(115200); //Programing port
		Serial1.begin(57600); //Comunicación con Dulcinea
		//Serial1.begin(115200); //Comunicación con Dulcinea
		//Serial2.begin(9600);  //Comunicación con Android primer prototipo "linvor" va a 9600 baudios
		Serial2.begin(115200);  //Comunicación con Android
	}// Fin puertos serie Serial Serial1 y Serial2 ---------------------------
	{ // I2C, acelerómetro y sensor de temperatura AHT10-----------------------
		//Se utiliza un i2c para el acelerómetro y otro para el AHT10 porque son incompatibles
		Wire.begin();
		Wire.setClock(MAX_FREC_I2C);//Velocidad del I2C 0.4MHz 
		#define I2C_ACC Wire //i2c del acelerómetro scl y sda
		#define I2C_SENSOR_HT Wire //o Wire1 i2c para el sensor de humedad y temperatura
		// Inicializar el Wire 1 solo si es necesario
		//Wire1.begin(); 
		//Wire1.setClock(MAX_FREC_I2C);//Velocidad del I2C 0.4MHz 
		#ifdef SENSOR_AHT10
			aht10Conectado = busca_aht10(); 
		#endif
		#ifdef SENSOR_BME280
			statusBME280 = bme.begin(0x76,&I2C_SENSOR_HT);  
		#endif
		//I2C.setClock(NORMAL_FREC_I2C);//Velocidad del I2C 0.1MHz 
  		AcelerometroConectado = busca_acelerometro();//Si hay acelerómetro pone AcelerometroConectado a true
	}
	{// Convertidor AD, Timers e interrupciones externas ----------------------
		// La interrupción del CLK_DSP se habilita solo si Frecuencia==0 en la función
		// Que cambia la frecuencia
		//Aumentaba la velocidad del ADC (ya no, Arduino lo cambió)
		// REG_ADC_MR = (REG_ADC_MR & 0xFFF0F0FF) | 0x00020100;
		analogReadResolution(12);//Resolución de los ADCs 12 bits
		TIMER_CLK.attachInterrupt(timer_clk);//Timer para clk
		TIMER_ADC.attachInterrupt(timer_ADC);
		TIMER_FOTO_ACEL.attachInterrupt(timer_foto_acel);
		//TIMER_ADC.start(TS_ADC_500us); //Arranca el timer del ADC
		TIMER_ADC.start(TS_ADC_400us); //Arranca el timer del ADC FS=1/(4e-4)=2,5KHz
		
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
	if(Estado_48V && !_DSP_48V) //Situación de funcionamiento normal.(Experimental)anular el pin DSP_48V
	{
		marcha_paro_motor(PARO);
		desactiva_48V(); //(Experimental)aquí no llega porque fuerzo !_DSP_48V = 0
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
		//digitalWrite(LED_BUILTIN,HIGH);  
		LED0_1 //digitalWrite(LED0,HIGH);
		BaseScpi.scpi(&Serial2);	
		LED0_0 //digitalWrite(LED0,LOW);
		//digitalWrite(LED_BUILTIN,LOW);  
	}//SCPI_SERIAL2 //Escucha al mando
	// Escucha el serial programing port
	if (Serial.available())
	{
		LED0_1 //digitalWrite(LED0,HIGH);
		BaseScpi.scpi(&Serial);	
		LED0_0 //digitalWrite(LED0,LOW);
	}
	//Completados los pasos pedidos, avisamos al que envió el comando y
	//Desactivamos los 48V
	if(StopPasos)
	{
		StopPasos=false;//Resetea el flag
		Println(FSTOP);//Envía stop al PC o Android
		//Si no está moviendo motor con el DSP desactiva los 48V
		// if (Frecuencia!=0) desactiva_48V(); //no desactivamos (experimental)
	}
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
	TEST_ADC_1 //pin 22 digitalWrite(TEST_ADC,HIGH);
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

		
		LED5_1
		TEST_FOTODIODO_1 //PIN 38
		float fl,fn,sum;
		unsigned int fuerzaNormal,fuerzaLateral,suma;
		//Cálculo de valores
		fuerzaNormal=FuerzaNormal.media();
		fn=mFn*fuerzaNormal+b_fn;//Calibrado
		//fn=mFotoDiodo*fuerzaNormal+bFotoDiodo; //Sin calibrar
		fuerzaLateral=FuerzaLateral.media();
		fl=mFl*fuerzaLateral+b_fl;//Calibrado
		//fl=mFotoDiodo*fuerzaLateral+bFotoDiodo;//Sin calibrar
		suma=Suma.media();
		sum=mSum*suma+b_sum;//Calibrado
		//sum=mFotoDiodo*suma+bFotoDiodo;//Sin calibrar
		//Fin cálculo de valores
		sprintf(respuesta,"%s %.2f %.2f %.2f",FFOTODIODO,fn,fl,sum);
		//sprintf(respuesta,"%s %f %f %f",FFOTODIODO,fn,fl,sum);
		Println(respuesta);
		TEST_FOTODIODO_0
		LED5_0
	}
	else //ACELEROMETRO
	{
		if(AcelerometroConectado)
		{
			LED4_1
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
			LED4_0
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
			if(--Pasos<=0) 
			{
				parar_clk_step();
				StopPasos = true;//Flag para que en el loop envie mensaje de stop
			}
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
		Contador++;//Contamos los pasos
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
		if(!_DSP_48V) 
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
	if(!_DSP_48V) numMotor=NINGUNO; //Si el bit del DSP DSP_48V, está a cero no selecciona motor
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
			desactiva_48V();//Para cambiar el motor activa hay que desactivar los 48V
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
	Encendido de los 48V del módulo TMCM-090
	Aquí se tocan los pines del DC/DC y se actualiza la 
	variable "Estado_48V"  
 * **********************************************************************/
void activa_48V(void)// Activa la fuente de 48V
{		 
	if(!_DSP_48V) return; //Si el bit del DSP está a 0, no se encienden los 48V
	 if(Estado_48V) return;//Si las llamada es para encender 48V y ya lo estan, sale.
	 digitalWrite(P15V_ON,HIGH); //Pone 15V a la entrada de la fuete de 48V
	 delay(RETARDO_ENCENDIDO1);//Espera antes de poner en marcha la fuente
	 digitalWrite(LM500_SHDWN,HIGH); //Activa la fuente de 48V 
	 delay(RETARDO_ENCENDIDO2);//Espera antes de salir
	 LED1_1//digitalWrite(LED1,HIGH);
	 Estado_48V=true; //48 voltios encendidos
}
/*************************************************************************
	Apagado de los 48V del módulo TMCM-090
	Aquí se tocan los pines del DC/DC y se actualiza la 
	variable "Estado_48V"  
 * **********************************************************************/
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
	//y Habilitar la interrupción del DSP_CLK
	//pero si la frecuencia no es cero sale de la función de interrupción
	if(Frecuencia==0)	
	{
		TIMER_CLK.stop();//Inhabilita interrupción del pin TIMER_CLK
		//Como la frecuencia es 0 el CLK lo controla del DSP
		activa_48V();//Si va a mover motores desde el DSP con DSP_CLK activamos los 48V
		attachInterrupt(digitalPinToInterrupt(DSP_CLK),clk_externo,FALLING);
		return 1;
	}
	else  
	{//Si la frecuencia no es 0 el CLK lo controla el TIMER_CLK
		detachInterrupt(digitalPinToInterrupt(DSP_CLK));	
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
	if(Pasos >=0 && Pasos <=PASOS_MAXIMOS)	
	{
		Pasos=pasos;
	}
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
/**************************************************************************
*	Busca e inicializa el sensor AHT10 en el I2C,
	si está presente devuelve true, si no false.
*************************************************************************/
#ifdef SENSOR_AHT10
bool busca_aht10(void)
{
	I2C_SENSOR_HT.beginTransmission(AHT10_ADD);
    uint8_t comandoCalibracion[3]={0xE1, 0x08, 0x00};
	I2C_SENSOR_HT.write(comandoCalibracion, 3);
    I2C_SENSOR_HT.endTransmission();
	delay(500);
 	I2C_SENSOR_HT.requestFrom(AHT10_ADD, 1);
    int8_t resultado = I2C_SENSOR_HT.read();
    if((resultado & 0x68) == 0x08)
    	return true;
	else
		 return false;
}
#endif
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
 * 200ms y para que deje de hacerlo.
 * Comandos MOT:IFO inicia y MOT:FFO finaliza
 * la función de interrupción del timer es timer_foto_acel()
 * **********************************************************************/
void pc_inicia_fotodiodo(void)
{
	int envios;
	//Lee el número de envios a realizar
	if (sscanf(BaseScpi.FinComando,"%u",&envios)==1)
		contadorEnvios=envios; //Inicaliza el contador de envios a realizar;
	else contadorEnvios=1;//Si no envía el número de envios se realiza un solo envío;	
	FotoAcel = FOTODIODO;
	TIMER_FOTO_ACEL.start(T200ms);//200ms	
}
void pc_fin_fotodiodo(void)
{
	TIMER_FOTO_ACEL.stop();
}
/************************************************************************
 * Funciones para hacer que envíe las señales del acelerometro cada 
 * 200ms y para que deje de hacerlo.
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
	
	TIMER_FOTO_ACEL.start(T200ms);//100ms	
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
	
	//Cálculo de valores
	fuerzaNormal=FuerzaNormal.media();
	fn=mFn*fuerzaNormal+b_fn;//Calibrado
	//fn=mFotoDiodo*fuerzaNormal+bFotoDiodo; //Sin calibrar
	fuerzaLateral=FuerzaLateral.media();
	fl=mFl*fuerzaLateral+b_fl;//Calibrado
	//fl=mFotoDiodo*fuerzaLateral+bFotoDiodo;//Sin calibrar
	suma=Suma.media();
	sum=mSum*suma+b_sum;//Calibrado
	//sum=mFotoDiodo*suma+bFotoDiodo;//Sin calibrar
	//Fin cálculo de valores
	
	sprintf(respuesta,"%s %.2f %.2f %.2f",FFOTODIODO,fn,fl,sum);
	Println(respuesta);
}
/*************************************************************************
		Lee el sensor de temperatura y humedad y lo envia al PC.
		El sensor SHT11 está obsoleto.
		Con el DHT-22 tarda 5,68ms el comando se procesa en 6,32ms.
		Con el sensor AHT10 el  comando se procesa en 525us. Tiene el
		inconveniente que no puede compartir el I2C.
		Comando MOT:TH?
 * **********************************************************************/
void pc_sensor_temperatura_humedad(void)
{
	TEST_SENSORHT_1 //pin 30 digitalWrite(TEST_SENSORHT,HIGH);
	LED3_1 //digitalWrite(LED3,HIGH);//Para test
	if(BaseScpi.FinComando[0]!='?')  
	{
		BaseScpi.errorscpi(4);//Parámetro inexistente
		LED3_0//digitalWrite(LED3,LOW);
		TEST_SENSORHT_0 //digitalWrite(TEST_SENSORHT,LOW);
		return;
	}
		if(EstadoMarchaParo==MARCHA)//moviendo motores no lee el sensor
	{
		//sprintf(Datos,"T%5.1f H%5.1f",0.0,0.0);
		Println("T 0.0 H 0.0");
		LED3_0//digitalWrite(LED3,LOW);
		TEST_SENSORHT_0 //digitalWrite(TEST_SENSORHT,LOW);	
		return;
	}
	
#ifdef SENSOR_SHT11
	char Datos[128];
	//SHT1x SHT11(SEN_DATA, SEN_CLK);
	float Temperatura;
	float Humedad;
	Temperatura = SHT11.readTemperatureC();
	Humedad = SHT11.readHumidity();
	sprintf(Datos,"%s %5.1f H%5.1f",FTEMPERATURA,Temperatura,Humedad);
	// Envía valores por el puerto
	Println(Datos)
#endif
#ifdef SENSOR_DHT22
	char Datos[128];	
	TIMER_ADC.stop();//Con el sensor DHT-22 se desactiva las interrupciones
	//DHT dht(SEN_DATA, DHT22);
	float Humedad = dht.readHumidity();
  	float Temperatura = dht.readTemperature();
  	sprintf(Datos,"%s %5.1f H%5.1f",FTEMPERATURA,Temperatura,Humedad);
	// Check if any reads failed and exit early (to try again).
  	if (isnan(Humedad) || isnan(Temperatura))
	{
    	BaseScpi.errorscpi(21);//Error lectura sensor humedad temperatura
	}
	Println(Datos);
	TIMER_ADC.start();//Con el sensor DHT-22 se desactiva las interrupciones
#endif
//El DHT10 se utiliza sin librería. Para ver como se utiliza estudiar alguna librería
#ifdef SENSOR_AHT10 //Este sensor tarda 520uS
	if(aht10Conectado)
	{
		char Datos[128];
		uint8_t rawData[6] = {0xFF, 0, 0, 0, 0, 0};// 0xFF es error
		//Leee humedad y temperatura
		I2C_SENSOR_HT.beginTransmission(AHT10_ADD);
		uint8_t comando[3]={0xAC, 0x33, 0x00};
		I2C_SENSOR_HT.write(comando,3); //Secuencia de comandos para leer la medida
		if (I2C_SENSOR_HT.endTransmission(true) != 0)
		{
			BaseScpi.errorscpi(25);
			LED3_0//digitalWrite(LED3,LOW);
			TEST_SENSORHT_0 //digitalWrite(TEST_SENSORHT,LOW);
			return;
		}//Si falla la transmisión hay error
		// lee 6 bytes del sensor
		I2C_SENSOR_HT.requestFrom(AHT10_ADD, 6, true);//Solicita datos al dispositivo
		if (I2C_SENSOR_HT.available() != 6){BaseScpi.errorscpi(25);}//Si no hay 6 datos sale con error
		for (uint8_t i = 0; i < 6 ; i++){rawData[i] = I2C_SENSOR_HT.read();}//Lee en el array de datos
		//Calcula la temperatura
		uint32_t temp = ((uint32_t)(rawData[3] & 0x0F) << 16) | ((uint16_t) rawData[4] << 8) | rawData[5];
		float Temperatura = (float)temp * 0.000191 - 50;
		//Calcula la humedad
		uint32_t humi = (((uint32_t) rawData[1] << 16) | ((uint16_t)rawData[2] << 8) | (rawData[3])) >> 4; //20-bit raw humidity data
		float Humedad = (float)humi * 0.000095;
		if (Humedad < 0.0)   Humedad= 0.0;
		if (Humedad > 100.0) Humedad = 100.0;
		//Envía los datos por el puerto
		sprintf(Datos,"%s %5.1f H%5.1f",FTEMPERATURA,Temperatura,Humedad);
		Println(Datos);
	}
	else BaseScpi.errorscpi(24);//Error no hay sensor conectado
 #endif
 
 #ifdef SENSOR_BME280
	if(statusBME280)
	{
		char Datos[128];
		float Temperatura,Humedad;
		bme.readHumidityTemperature(&Temperatura,&Humedad);
		sprintf(Datos,"%s %5.1f H%5.1f",FTEMPERATURA,Temperatura,Humedad);
		Println(Datos);
	}
	else BaseScpi.errorscpi(24);//Error no hay sensor conectado
 #endif	

	LED3_0//digitalWrite(LED3,LOW);
	TEST_SENSORHT_0 //digitalWrite(TEST_SENSORHT,LOW);
	return;
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
		//sprintf(respuesta,"%s %.2f %.2f %.2f",FACELEROMETRO,EjeX,EjeY,EjeZ);
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
/*************************************************************************
		Desactiva el DC/DC de 48V
		Comando MOT:DV 1
 * **********************************************************************/
void pc_activa_48v()
{
	if (BaseScpi.FinComando[0]=='?')
	{
		char Respuesta[32];
		sprintf(Respuesta,"%s %u",FESTADO48V,Estado_48V);
		Println(Respuesta);	
		return;
	}
	if (BaseScpi.FinComando[0]==' ')
	{
		switch (BaseScpi.FinComando[1])
		{
		case '0':
			desactiva_48V();
		break;
		case '1':
			activa_48V();
		break;
		case '?':
			char Respuesta[32];
			sprintf(Respuesta,"%s %u",FESTADO48V,Estado_48V);
			Println(Respuesta);	
		break;		
		default:
			BaseScpi.errorscpi(4);//Parámetro erroneo
		break;
		}
	}
	else BaseScpi.errorscpi(5);//Formato no válido
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
	//Cálculo de valores
	fuerzaNormal=FuerzaNormal.media();
	fn=mFn*fuerzaNormal+b_fn;//Calibrado
	//fn=mFotoDiodo*fuerzaNormal+bFotoDiodo; //Sin calibrar
	fuerzaLateral=FuerzaLateral.media();
	fl=mFl*fuerzaLateral+b_fl;//Calibrado
	//fl=mFotoDiodo*fuerzaLateral+bFotoDiodo;//Sin calibrar
	suma=Suma.media();
	sum=mSum*suma+b_sum;//Calibrado
	//sum=mFotoDiodo*suma+bFotoDiodo;//Sin calibrar
	//Fin cálculo de valores
	if(EstadoMarchaParo) emp=10;
	else emp=5;
	sprintf	(respuesta,"%s %.2f %.2f %.2f %u %u",FBLUETOOTHESTADO,fn,fl,sum,emp,Pasos);
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
void idnSCPI(void)
{
	BaseScpi.enviarNombreDelSistema();
}
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