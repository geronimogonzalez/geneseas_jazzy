
//Programa controllino con implementacion de
// Control manual y automatico con respectivos topicos
// Sensores ultrasonicos y sus publicadores
// Motores y suscriptores
// Flasher (solo arranca con el controllino)
// Sensor de bateria (NO FUNCIONA -por eso comentado)

//TODO: 
// * Hay un glish en motores cuando mando comando 1500 (cero velocidad) que se da por interrupcion del
//   control RF (interrupcion externar y problema de timers), HAY QUE ARREGLAR
// * Sensor de bateria con interrupcion que cada tanto envia datos no funciona genera problema con timer
//   hay que acomodar funcion timer o directamente volararla, si lo ponemos continuo ver que no se sature puero serie
// * NO chekie si los flag de configuracion de direccion de motor funcionan bien, creo que habria que agregar alguna configuracion extra donde se configura 
//   que el comando que suma o resta del control para girar tipo un if-else



//---------------------------------------------------------------------------------



const int CANT_MOTORES = 4;
const int CANT_FLASHERS = 4;
float motorValues[CANT_MOTORES] = {0.0f, 0.0f, 0.0f, 0.0f};
union FloatConverter {
    float f;         // Un miembro para guardar el número como float (ocupa 4 bytes)
    uint8_t b[sizeof(float)]; // Un miembro para acceder a esos mismos 4 bytes como un array de bytes
};

uint8_t flasho = 0;
int flasherValues[CANT_FLASHERS] = {1,1,1,1};
//---------------------------------------------------------------------------------
bool automatico = false;
//PROGRAMA PRINCIPAL

void setup() 
{
  // put your setup code here, to run once:
  
  //nh.initNode();
  
  setupMotor();    
  setupRFcontrol();  
  setupUltrasound();
  setup_flasher();
  //setup_Batery_State();
  Serial.begin(115200);

}

void loop() 
{
  loopRFcontrol(); 
  readAndParseBinaryCommands(motorValues,CANT_MOTORES,flasherValues);
  loopFlasher();
  loopMotor();
  loopUltrasound();
}
