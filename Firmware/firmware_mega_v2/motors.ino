
//MOTORS

#include <MegaServo.h>

//global variables
#define PWM1 5 //ORIGINAL 5
#define PWM2 6 //ORIGINAL 7
#define PWM3 7 //ORIGINAL 2
#define PWM4 8 //ORIGINAL 8

//configuration motors turning
const int direction_motor_left = -1; // set to -1 to invert turning sense
const int direction_motor_right = 1; // set to -1 to invert turning sense
const int direction_motor_center = -1; // set to -1 to invert turning sense
const int direction_motor_direction = -1; // set to -1 to invert turning sense

MegaServo Servos[5] ; // clase mega servo para generar pulsos PWM 

//variables para comando de motores (????NECESARIMENTE TIENEN QUE SER TIPO INT)
  float data1=1500; 
  float data2=1500;
  float data3=1500;
  float data4=1500;

  float data1_prev=1500; 
  float data2_prev=1500;
  float data3_prev=1500;
  float data4_prev=1500;

  float data1_prom=1500; 
  float data2_prom=1500;
  float data3_prom=1500;
  float data4_prom=1500;

  
  float data1_flag=0; 
  float data2_flag=0;
  float data3_flag=0;
  float data4_flag=0;


//variables de zona muerta artificial para forzar desconexion de motores
int umbral = 10;
int medio =1500;

int reconexion_esc=2000;

// Offsets individuales para corregir el centro
float offsetL = -0.06;
float offsetR = 0.05;
float offsetC = -0.05;
float offsetD = 0.0;


// ROS configuration -----------------------------------------------------------


//ESC variables for ROS topics
float data1_ch=0; //L
float data2_ch=0; //R
float data3_ch=0; //C
float data4_ch=0; //D


// SETUP MOTORS -----------------------------------------------------------
  
void setupMotor() 
  {
    
 
  float data1_ch=motorValues[0]; //L
  float data2_ch=motorValues[1]; //R
  float data3_ch=motorValues[2]; //C
  float data4_ch=motorValues[3]; //D
  // Genero objeto servo para cada pin del controlino que comanda a cada motor en el rango de pulso programado para el PPM de los esc.
  //Note: esc program with BLheli PPM Min throttle = 1000 (microsegundos) and PPM Max throttle = 2000 (microsegundos)
  Servos[0].attach(PWM1, 1000, 2000); // L 
  Servos[1].attach(PWM2, 1000, 2000); // R
  Servos[2].attach(PWM3, 1000, 2000); // C
  Servos[3].attach(PWM4, 1000, 2000); // D - unique motor with old not programable ESC

  //Fuerzo cero (ver que es mitad del intervalo 1000-2000us) al arranque para el armado de los esc.
  Servos[0].writeMicroseconds(1500);   
  Servos[1].writeMicroseconds(1500);   
  Servos[2].writeMicroseconds(1500);   
  Servos[3].writeMicroseconds(1500);   
  
  //Espero 10 segundos para el armado de los ESC - tiempo puesto a ojo, no estaba documentado realmente.
  delay(10000);

  Servos[0].writeMicroseconds(1500);   
  Servos[1].writeMicroseconds(1500);   
  Servos[2].writeMicroseconds(1500);   
  Servos[3].writeMicroseconds(1500);   

  }

// LOOP MOTORS ------------------------------------------------------------

void loopMotor() 
  {

//POr interferencia que genera interrupcion externa (pin recepcion RF), cuando los motores deben estar en cero (comando 1500)
// se genera un pequeño movimiento de glitch sobre los mismos la solucion es desconectar las patas de los esc durante estas condiciones
// este problema solo se observa en los ESC nuevos, el viejo que queda no se desconecta ya que no tiene el problema y si lo desconectamos
// genera bip en el motor por falta de señal.

if(Mando_canal[7-1]<-0.8 || Mando_canal[7-1]>0.8)
{
  Servos[0].attach(PWM1, 1000, 2000); // L 
  Servos[1].attach(PWM2, 1000, 2000); // R
  Servos[2].attach(PWM3, 1000, 2000); // C 
  //Servos[3].attach(CONTROLLINO_D3, 1000, 2000); // D - unique motor with old not programable ESC
}
else
{
  Servos[0].detach(); // L 
  Servos[1].detach(); // R
  Servos[2].detach(); // C
  //Servos[3].detach(); // D - unique motor with old not programable ESC
}
 
  // check if we are in manual or automatic mode
  if(!automatico) //manual control 
  {
    
      //data1 = map(data1, -2, 2, 0, 180); // funcion maps por alguna razon recorta a enteros
      data1=(Mando_canal[3-1]+Mando_canal[1-1])*250+1500; // mape amano de -1 1 (ojo en relaida -2 2) a 1000 2000
      data2 =(Mando_canal[3-1]-Mando_canal[1-1])*250+1500;
    

    // direction motor comment in version without it
    data4=(Mando_canal[2-1])*500+1500; //mapeo de -1 a 1 a 1000 a 2000
    if(Mando_canal[6-1]==1) //subccion activada
    {
      data3=1700; 
    }
    else if(Mando_canal[6-1]==-1) //subcion en reversa
    {
      data3=1300; 
    }
    else  // subcion apagada
    {
      data3=1500; 
    }



    // configure turning sense
    if (direction_motor_left == -1)
    {data1=map(data1,1000,2000,2000,1000);}
    if (direction_motor_right == -1)
    {data2=map(data2,1000,2000,2000,1000);}
    if (direction_motor_center == -1)
    {data3=map(data3,1000,2000,2000,1000);}
    if (direction_motor_direction == -1)
    {data4=map(data4,1000,2000,2000,1000);}
    
    Servos[0].writeMicroseconds(data1); 
    Servos[1].writeMicroseconds(data2); 
    Servos[2].writeMicroseconds(data3); 
    Servos[3].writeMicroseconds(data4); 
  
   
   }
   else if(automatico) //automatic control 
   { 
   

    data1_ch=motorValues[0] - offsetL;
    data2_ch=motorValues[1] - offsetR;
    data3_ch=motorValues[2] - offsetC;
    data4_ch=motorValues[3] - offsetD;
    data1=data1_ch*500+1500;  // mape amano de -1 1 a 1000 2000
    data2=data2_ch*500+1500;
    data3=data3_ch*500+1500;
    data4=data4_ch*500+1500;



    // configure turning sense
    if (direction_motor_left == -1)
    {data1=map(data1,1000,2000,2000,1000);}
    if (direction_motor_right == -1)
    {data2=map(data2,1000,2000,2000,1000);}
    if (direction_motor_center == -1)
    {data3=map(data3,1000,2000,2000,1000);}
    if (direction_motor_direction == -1)
    {data4=map(data4,1000,2000,2000,1000);}

    Servos[0].writeMicroseconds(data1);   // Motor left
    Servos[1].writeMicroseconds(data2);
    Servos[2].writeMicroseconds(data3);
    Servos[3].writeMicroseconds(data4);
    /* promedios no anduvo

//Motor left - unico con ESC viejo por eso no necesita el detach
Servos[0].writeMicroseconds(data1);   // Motor left


//Motro right
data2_prom=(data2+data2_prev)/2;
if(data2_prom< medio-umbral || data2_prom> medio+umbral)
{
  if(data2_flag==1)
  {
    Servos[1].attach(CONTROLLINO_D1, 1000, 2000); // R 
    data2_flag=0;
  }
  Servos[1].writeMicroseconds(data2);   
}
else
{
  Servos[1].detach(); // R
  data2_flag=1;
}

//Motor central
data3_prom=(data3+data3_prev)/2;
if(data3_prom< medio-umbral || data3_prom>medio+umbral)
{
  if(data3_flag==1)
  {
    Servos[2].attach(CONTROLLINO_D2, 1000, 2000); 
    data3_flag=0;
  }
  Servos[2].writeMicroseconds(data3);   
}
else
{
  Servos[2].detach(); 
  data3_flag=1;
}

//Motor transversal
data4_prom=(data4+data4_prev)/2;
if(data4_prom< medio-umbral || data4_prom>medio+umbral)
{
  if(data4_flag==1)
  {
    Servos[3].attach(CONTROLLINO_D3, 1000, 2000); 
    delay(reconexion_esc);
    data4_flag=0;
  }
  Servos[3].writeMicroseconds(data4);   
}
else
{
  Servos[3].detach(); 
  data4_flag=1;
}



*/
    
   }
  else
  {
    //do nothing program stop motors /en realida esta condicion no deberia entrar pq los motores estarian desconectados

    //probar detach
    data1=1500;
    data2=1500;
    data3=1500;
    data4=1500;

    Servos[0].writeMicroseconds(data1); 
    Servos[1].writeMicroseconds(data2); 
    Servos[2].writeMicroseconds(data3); 
    Servos[3].writeMicroseconds(data4); 
    
    Servos[0].detach(); // L 
    Servos[1].detach(); // R
    Servos[2].detach(); // C
    //Servos[3].detach(); // D - unique motor with old not programable ESC

    data1_flag=1;
    data1_flag=1;
    data1_flag=1;
    data1_flag=1;
    
    
  }



}
