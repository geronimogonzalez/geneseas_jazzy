// RF control

// global variables ------------------------------------------------------------
#define numero_canales 8

const byte interruptPin = 19;
uint64_t pulso_instante[numero_canales * 2 + 2];
float Mando_canal[numero_canales]={0,0,0,0,0,0,0,0};
uint8_t contador_flanco = 1;
const uint8_t START_BYTE_RF = 0xB5; // Byte que marca el inicio de un paquete de DATOS RF
const size_t RF_DATA_SIZE = numero_canales * sizeof(float);
const size_t RF_PACKET_SIZE = 1 + RF_DATA_SIZE; // Tamaño total del paquete RF (1 byte inicio + 32 bytes datos = 33 bytes)

int pasadas=0;

// SETUP RF control -----------------------------------------------------------

  void setupRFcontrol() 
  {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), pulse_counter, CHANGE);
  pulso_instante[0] = micros();


  }

// LOOP RF control ------------------------------------------------------------

  void loopRFcontrol() 
  {
    
    if (contador_flanco == 18) 
    {
      for (int i = 1; i <= numero_canales; i++) 
      {//cambie aca el indice vector a i-1 en vez de i
        Mando_canal[i-1] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];
      }
      // Calls functions to normalise
      Channel1234(); //maps to -1 - 1
      Channel5(); // maps to 0 and 1
      Channel67(); //maps to -1 0 1
      Channel8(); //maps to 0 1
      sendRFDataToPython();
    }
  }

// Auxiliary functions ---------------------------------------------------------

// RF interrupt function ------------------------------------------------------

    void pulse_counter()  
    {
      
      int aux = micros();
      
      if (aux - pulso_instante[contador_flanco - 1] > 2100) 
      { 
        contador_flanco = 0;
      }
      pulso_instante[contador_flanco] = aux; 
      contador_flanco=contador_flanco+1;
    }
    

// auxiliary functions for RF controller --------------------------------------
  
  void Channel1234()
    {                                
      //fijarse sino hacer un mapeo aparte del canal4 tiene menos rango                              // Map analog values between -1 and 1
      for (int j = 0; j < 4; j++) 
      {
      Mando_canal[j]=map(Mando_canal[j],580, 1580,-100,100);
      Mando_canal[j]=Mando_canal[j]/100;
      }
    }

void Channel5()
  {                                                                // Scale the values to get digital values 0 and 1
        if (Mando_canal[4]>1500)
          {
          Mando_canal[4]=1.0;
          }
        else 
          {
          Mando_canal[4]=0.0;
          }  
      
  }


  void Channel67()
  {                                                                // Scale the values to get digital values -1, 1 and 0
      for (int k = 5; k <= 6; k++) 
      {
        if (Mando_canal[k]<600)
          {
          Mando_canal[k]=-1.0;
          automatico = false;
          }
        else if (Mando_canal[k]>1500) 
          {
          Mando_canal[k]=1.0;
          automatico = true;
          }
        else 
          {
          Mando_canal[k]=0.0;
          }  
      }
    }


  void Channel8()
  {                                                                 // Scale the values to get digital values 1 and 0
     if (int(Mando_canal[7])<1000) 
     {
        Mando_canal[7]=0.0;
     }
     else
     {
      Mando_canal[7]=1.0;
     }
  }  
  // --- DEFINICIÓN DE LA FUNCIÓN DE ENVÍO BINARIO (Arduino -> Python Datos RF) ---
// Esta función es llamada por un timer periódico Y/O por loopRFcontrol() 
// cuando un paquete RF completo ha sido procesado.
// Empaqueta los datos de Mando_canal[] (8 floats) en un paquete binario 
// (START_BYTE_RF + 32 bytes de datos) y los envía por serial al nodo Python.
// La función es 'void' y no retorna un estado de éxito o error específico de este envío.
  void sendRFDataToPython() {
      // Buffer local para construir el paquete binario completo antes de enviarlo.
      // El tamaño es el total esperado: 1 byte de inicio + bytes de datos RF.
      //Serial.println("enviando datos");
      uint8_t packet[RF_PACKET_SIZE]; 

      // El primer byte del paquete es nuestro marcador de inicio para datos RF.
      packet[0] = START_BYTE_RF; 

      // Usamos la unión para convertir de float a secuencia de bytes de 4 en 4.
      FloatConverter converter; 

      // Empaquetar los valores float del array Mando_canal[] en el buffer del paquete.
      // Mando_canal es de tamaño numero_canales (asumimos 8). Los índices válidos son 0 a 7.
      // El bucle va de 0 a RF_DATA_COUNT-1 (0 a 7) para cubrir los 8 elementos.
      for (int i = 0; i < numero_canales; ++i) { 
          // 1. Obtiene el valor float del índice actual del array Mando_canal.
          converter.f = Mando_canal[i]; 

          // 2. Copia los 4 bytes de ese float (ahora en converter.b) 
          //    a la posición correcta en el buffer del paquete.
          //    - '&packet[1 + i * sizeof(float)]': Dirección de memoria en el buffer
          //      - 1: para saltar el byte de inicio que ya pusimos en packet[0].
          //      - i * sizeof(float): desplazamiento para el float actual (0*4=0, 1*4=4, 2*4=8, etc.)
          //    - converter.b: La fuente de los bytes (los 4 bytes del float).
          //    - sizeof(float): Cuántos bytes copiar (4).
          memcpy(&packet[1 + i * sizeof(float)], converter.b, sizeof(float));
      }
      // --- Envío del paquete ---
      // Verificar si el buffer de transmisión serial tiene espacio suficiente 
      // para enviar el paquete completo (33 bytes).
      if (Serial.availableForWrite() >= sizeof(packet)) {
          // Si hay espacio, envía la secuencia completa de bytes.
          Serial.write(packet, sizeof(packet));
          
          // Opcional: Log de envío exitoso (consume tiempo, usar con cuidado)
          // Serial.print("Sent RF data packet ("); Serial.print(sizeof(packet)); Serial.print(" bytes), start="); Serial.println(START_BYTE_RF, HEX);
      } else {
          // Opcional: Log de que el buffer de transmisión está lleno y el paquete no se envió.
          // Esto puede ocurrir si estás enviando muy rápido o a un baud rate bajo.
          // Serial.println("Serial transmit buffer full. RF packet dropped.");
      }
}
