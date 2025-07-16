
//Batery state sensor

/*

// global variables ------------------------------------------------------------


#define Voltage_pin CONTROLLINO_A6
#define Vcc 11
#define t10_min 143 // Valor de comparacion para ejecurtar una rutina cada 10 min con interrupciones de 4,2 seg aprox
//10min*60seg/min/(2elev(16) * 1024/16MHz) = 143

bool Flag_10_min = true;
uint8_t contador_de_tiempo = 0;

// ROS configuration -----------------------------------------------------------


std_msgs::Float32 msg_Voltage;
ros::Publisher Voltage("/vbattery_topic", &msg_Voltage);


// SETUP -----------------------------------------------------------------------

void setup_Batery_State()
{
  pinMode(Voltage_pin,INPUT);
  nh.advertise(Voltage);
  Timer3();
}

// LOOP ------------------------------------------------------------------------

void loop_Batery_State()
{
  if(Flag_10_min == true)
  {
    Flag_10_min = false;
    loop_Batery_State();
  }
}

// Auxiliary functions ---------------------------------------------------------

void batery_lecture()
{
  msg_Voltage.data=(float)analogRead(Voltage_pin)*Vcc/1024;// Puede ser que se tenga que modificar Vcc
  Voltage.publish(&msg_Voltage);
  sei();  //habilita las interrupciones globales 
}

void Timer3()
{
  TCCR3A = 0;                         //Limpio los registros de control
  TCCR3B = 0;
  TCCR3B |= (1<<CS30)|(1 << CS32);    // Prescaler para 1024: CS12 = 1 e CS10 = 1
  TIFR3 |= (1<<TOV3);                 // Configuro el la interrupcion por overflow
  
  TIMSK3 |= (1 << TOIE3);           // Habilita la interrupcion del TIMER1
}

*/
