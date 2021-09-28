    #ifndef __MUESTRAS_H
    #define __MUESTRAS_H
    /***************************************************************************
        Clase que implementa un array de enteros para acumular adquisiciones
        del ADC. al constructor hay que pasarle un int con el tamaño del
        array (máximo MAX_LONGITUD_MUESTRAS)
        Devuelve la media con media() 
        Pone un dato nuevo en el array con nuevoDato(unsigned int Dato) 

    ****************************************************************************/
class Muestras
{
    #define MAX_LONGITUD_MUESTRAS 128
	private:
    unsigned int *Array;//Array para poner las muestras
	int NumMuestras;
    int Indice;
    public:
	//...........................................................................
	//Métodos públicos
	/*****************************************************************************
     *  Constructor. Hay que meterle el tamaño del array
     * ***************************************************************************/
    Muestras(int NumMuestras) 
    {
        //Evita crear un array mayor de MAX_LONGITUD_MUESTRAS
        if (NumMuestras>=MAX_LONGITUD_MUESTRAS || NumMuestras<=0) this->NumMuestras=MAX_LONGITUD_MUESTRAS; 
        else this->NumMuestras=NumMuestras;
        //Pide memoria para el array 
        Array=new unsigned int[this->NumMuestras];
        //Pone el Array a cero
        for(Indice=0; Indice < NumMuestras;Indice++)
        {
             Array[Indice]=0; //Borra la posición
        }
        Indice=0;
    }
    /*****************************************************************************
     * Adquisición. Pone un dato en el array
     * ***************************************************************************/
    int nuevoDato(int DatoADC) 
    {
        if(Indice >= this->NumMuestras) Indice=0;
        Array[Indice]=DatoADC;
        return Indice++;
    }
	/*****************************************************************************
        Calcula la media
    *****************************************************************************/
    unsigned int media(void)
    {
        unsigned long Suma;
        Suma=0;
        for(int i=0; i < NumMuestras;i++)
        {
             Suma+=Array[i]; //Acumula resultados  
        }
        return Suma/NumMuestras;
    }
};// class Muestras
/*********************************************************************************/
#endif//__MUESTRAS_H