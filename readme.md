# UNIVERSIDAD AUTÓNOMA DE MADRID
# SEGAINVEX-Electrónica
**Patricio Coronado Collado abril 2020**
# DESCRIPCIÓN
## Software para la Base del microscópio SPM 
## Desarrollado por SEGAINVEX-Electrónica para el [Departamento de Física de la Materia Condensada de la UAM](https://www.fmc.uam.es/research/nano-spm-lab/)

# Cambios de esta versión respecto a la V1_2_0:
## -1. Las funciones que envian datos del fotodiodo y el acelerómetro cambian. Ahora el comando debe incluir el parámetro del número de muestar que se quieren recibir.
## -2. Las firmas que preceden a las cadenas de datos que se envian por los puertos serie han cambiado para compatibilizarlas con la APP que implementa el mando.    


## 
# DEPENDENCIAS

## Librería para Arduino [SegaSCPI](https://github.com/PatricioCoronado/SegaSCPI)

 Para comunicar el PC con la base
## Librería para Arduino DHT

Modificada para lee en 6,32 ms. el sensor DHT22 que monitoriza la temperatura y humedad de la cabina que aloja el equipo.
## Librería para Arduino DUE DueTimer-1.4.6

Para utilizar user friendly los timers de Arduino 
## Librería para Arduino SHT1x-master

Para el sensor de humedad y temperatura SHT11 (obsoleto)
## Librería para Arduino SparkFun_MMA8452Q_Arduino_Library-master
Para utilizar un acelerómetro MMA8452Q para nivelar la cabeza.

# [SOFTWARE DE PC](https://github.com/PatricioCoronado/Base-SPM-CVI)
Desarrollado con LabWindos/CVI con licencia para SEGAINVEX-Electrónica.

# [SOFTWARE PARA TABLET](https://github.com/PatricioCoronado/Base-SPM-tablet)
Desarrollado con Android Studio 4.0.1

# [MANUAL](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_2_1/ficheros/Manual.pdf)
## 
# IMAGENES DEL PROYECTO
##
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_2_1/ficheros/imagen1.png "primer prototipo")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_2_1/ficheros/imagen2.png "PCB_A")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_2_1/ficheros/imagen4.png "backplane")
![Alt text](https://github.com/PatricioCoronado/Base-SPM-Arduino-DUE/blob/V1_2_1/ficheros/imagen3.png "sistema midiendo")
