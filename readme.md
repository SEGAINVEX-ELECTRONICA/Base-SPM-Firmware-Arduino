# UNIVERSIDAD AUTÓNOMA DE MADRID
# SEGAINVEX-Electrónica
**Patricio Coronado Collado septiembre 2020**
# DESCRIPCIÓN
## Software para la Base del microscópio SPM 
## Desarrollado por SEGAINVEX-Electrónica para el [Departamento de Física de la Materia Condensada de la UAM](https://www.fmc.uam.es/research/nano-spm-lab/)

# Cambios de esta versión respecto a la V1_2_1 y anteriores:
## Cambia el sensor de humedad temperatura por el BME280 controlado por I2C
## Cambian los pines para comunicar con el sensor y activar el DC/DC de 48V
## Cambia el pin de la señal DSP_CLK y DSP_48V

## 
# DEPENDENCIAS

## Librería para Arduino [SegaSCPI](https://github.com/PatricioCoronado/SegaSCPI)

 Para comunicar el PC con la base
## Librerías para Arduino del sensor BME280 de Adafruit modificadas

## Librería para Arduino DUE DueTimer-1.4.6

## Librería para Arduino SparkFun_MMA8452Q_Arduino_Library-master, para utilizar un acelerómetro MMA8452Q 

# [SOFTWARE DE PC](https://github.com/PatricioCoronado/Base-SPM-CVI)
Desarrollado con LabWindos/CVI con licencia para SEGAINVEX-Electrónica.

# [SOFTWARE PARA TABLET](https://github.com/PatricioCoronado/Base-SPM-tablet)
Desarrollado con Android Studio 4.0.1

# [MANUAL](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/Manual.pdf)
## 
# IMAGENES DEL PROYECTO
##
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen1.png "primer prototipo")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen2.png "PCB_A")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen4.png "backplane")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_3/ficheros/imagen3.png "sistema midiendo")
