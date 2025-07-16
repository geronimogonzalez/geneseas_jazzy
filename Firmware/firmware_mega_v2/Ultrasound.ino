
// Ultrasound sensors


// global variables ------------------------------------------------------------


#define PW1 24 
#define PW2 26 
#define PW3 32  
#define PW4 34 
#define PW5 40 
#define PW6 42 
#define PW7 A12 
#define PW8 48 


#define RX1 22 
#define RX2 28 
#define RX3 30 
#define RX4 36 
#define RX5 38 
#define RX6 44 
#define RX7 A11  
#define RX8 46 


  //#define SCALE 58              // uS per cm -- used to normalise measurement with input PW
  #define SCALE 5800              // uS per m -- used to normalise measurement with input PW
  
  const uint8_t START_BYTE_ULTRASOUND = 0xC5; // Byte de inicio para datos Ultrasonido (¡DEBE SER ÚNICO!)
  const int ULTRASOUND_COUNT = 8;             // Número de sensores de ultrasonido
  const size_t ULTRASOUND_VALUE_SIZE = sizeof(float); // getRange() retorna float (4 bytes)
  const size_t ULTRASOUND_DATA_SIZE = ULTRASOUND_COUNT * ULTRASOUND_VALUE_SIZE; // 8 * 4 = 32 bytes de datos
  const size_t ULTRASOUND_PACKET_SIZE = 1 + ULTRASOUND_DATA_SIZE;   // Tamaño total del paquete (1 + 32 = 33 bytes)

  unsigned long  timeout_ultrasound = 80000; //timeout in us for each ultrasound for model MB7062 aprox 32.3 ms //seteado a la mano pq sino tira rangos cero??
  // Maximum range in meters
  const float maxRange = 7;       
  // Minimum range in meters
  const float minRange = 0.02; 
  float ultrasoundRanges[ULTRASOUND_COUNT];
  // Trigger pin
  const int trigger_1 = RX1; //PATAS RX DE LOS ULTRASONIDOS
  const int trigger_2 = RX2;
  const int trigger_3 = RX3;
  const int trigger_4 = RX4;
  const int trigger_5 = RX5;
  const int trigger_6 = RX6;
  const int trigger_7 = RX7;
  const int trigger_8 = RX8;

  // PW measurement pin
  const int sensorPW_1 = PW1; // PATAS PW DE LOS ULTRASONIDOS
  const int sensorPW_2 = PW2;
  const int sensorPW_3 = PW3;
  const int sensorPW_4 = PW4;
  const int sensorPW_5 = PW5;
  const int sensorPW_6 = PW6;
  const int sensorPW_7 = PW7;
  const int sensorPW_8 = PW8;

  //Used to set sample time
  unsigned long range_timer=0; 


// SETUP -----------------------------------------------------------------------

void setupUltrasound() {

    pinMode(sensorPW_1, INPUT);                                     // set the digital I/O pin modes
    pinMode(trigger_1, OUTPUT); 
                           
    pinMode(sensorPW_2, INPUT);                                     
    pinMode(trigger_2, OUTPUT); 
                        
    pinMode(sensorPW_3, INPUT);                                     
    pinMode(trigger_3, OUTPUT); 
  
                        
    pinMode(sensorPW_4, INPUT);                                     
    pinMode(trigger_4, OUTPUT);
                      
    pinMode(sensorPW_5, INPUT);                                     
    pinMode(trigger_5, OUTPUT);
                         
    pinMode(sensorPW_6, INPUT);                                     
    pinMode(trigger_6, OUTPUT);
                         
    pinMode(sensorPW_7, INPUT);                                     
    pinMode(trigger_7, OUTPUT);
    
                          
    pinMode(sensorPW_8, INPUT);                                     
    pinMode(trigger_8, OUTPUT);
  }


// LOOP ------------------------------------------------------------------------


   void loopUltrasound() 
  {
  
// este retardo de ver cada un segundo capaz lo pdemos sacar o hacer mas chico
  if ( (millis() - range_timer) > 300)   //sample and publish data each 1 second
  {


    //sensor2
    ultrasoundRanges[1] = getRange(trigger_2,sensorPW_2);
    

    //sensor4
    ultrasoundRanges[3] = getRange(trigger_4,sensorPW_4);
    

    //sensor6
    ultrasoundRanges[5] = getRange(trigger_6,sensorPW_6);
    

    //sensor8
    ultrasoundRanges[7] = getRange(trigger_8,sensorPW_8);
   

    delay(22); //orden de sensores para evitar cross-talk entre ellos, y delay para separar entre grupos.
    
    //sensor1
    ultrasoundRanges[0] = getRange(trigger_1,sensorPW_1);
  

    //sensor3
    ultrasoundRanges[2] = getRange(trigger_3,sensorPW_3);
    
    //sensor5
    ultrasoundRanges[4] = getRange(trigger_5,sensorPW_5);


    //sensor7
    ultrasoundRanges[6] = getRange(trigger_7,sensorPW_7);
   
    sendUltrasoundDataToPython();
    range_timer =  millis();

  }
 }

// Auxiliary functions -------------------------------------------------------------


float getRange(int trigger, int sensorPW)                    //Get distance from sensor
{
  digitalWrite(trigger, HIGH);    //Send trigger signal for mesurement (20 ms tigger HIGH)
  delayMicroseconds(25);          //
  digitalWrite(trigger, LOW);     //
  return pulseIn(sensorPW, HIGH,timeout_ultrasound)/(float)SCALE;;  //normalise and return measurement
}
void sendUltrasoundDataToPython() {
    uint8_t packet[ULTRASOUND_PACKET_SIZE]; 
    packet[0] = START_BYTE_ULTRASOUND; 

    FloatConverter converter_float; // Usamos FloatConverter para convertir float a bytes

    // Empaquetar los 8 valores float del array ultrasoundRanges[].
    // El bucle va de 0 a ULTRASOUND_COUNT-1 (0 a 7).
    for (int i = 0; i < ULTRASOUND_COUNT; ++i) { 
        converter_float.f = ultrasoundRanges[i]; 
        memcpy(&packet[1 + i * sizeof(float)], converter_float.b, sizeof(float));
    }

    // --- Envío del paquete ---
    if (Serial.availableForWrite() >= sizeof(packet)) {
        Serial.write(packet, sizeof(packet));
        // Optional: Log
    } else {
        // Optional: Log buffer full
    }
}
