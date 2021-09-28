# UNIVERSIDAD AUTÓNOMA DE MADRID
# [SEGAINVEX-Electrónica](https://www.uam.es/uam/segainvex)
**Patricio Coronado Collado septiembre 2021**
# DESCRIPCIÓN
## Firmware para la electrónica de la base del microscópio SPM 

# [Firmware para el Arduino DUE](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE)
## Desarrollado con [Visual Studio Code](https://code.visualstudio.com/) y [PlatformIO](https://platformio.org/)

## Por [Patricio Coronado, ](https://patriciocoronadocollado.000webhostapp.com/) en [SEGAINVEX-Electrónica](https://www.uam.es/uam/segainvex) para el [Departamento de Física de la Materia Condensada de la UAM](https://www.fmc.uam.es/research/nano-spm-lab/)

# Cambios de esta versión respecto a la V1_2_1 y anteriores:
## Cambia el sensor de humedad temperatura por el BME280 controlado por I2C
## Cambian los pines para comunicar con el sensor y activar el DC/DC de 48V
## Cambiado el pin de la señal DSP_CLK y DSP_48V

## 
# DEPENDENCIAS

## Librería para Arduino [SegaSCPI.](https://github.com/PatricioCoronado/SegaSCPI) Para comunicar la base con PC o Tablet

## Librerías para Arduino del sensor BME280 de Adafruit modificadas

## Librería para Arduino DUE DueTimer-1.4.6

## Librería para Arduino SparkFun_MMA8452Q_Arduino_Library-master, para utilizar un acelerómetro MMA8452Q 

## [SOFTWARE DE PC](https://github.com/PatricioCoronado/Base-SPM-Java)
Desarrollado en [Java](https://www.java.com/es/) con [Eclipse](https://www.eclipse.org/)

## [SOFTWARE PARA TABLET](https://github.com/PatricioCoronado/Base-SPM-tablet)
Desarrollado con [Android Studio 4.0.1](https://developer.android.com/studio?hl=es)

## [MANUAL](https://github.com/SEGAINVEX-ELECTRONICA/Base_SPM_V3/blob/main/Manual/Manual_Base_SPM_V3.pdf)
## 
# IMÁGENES DEL PROYECTO
##
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen1.png "primer prototipo")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen2.png "PCB_A")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen4.png "backplane")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen3.png "sistema midiendo")
