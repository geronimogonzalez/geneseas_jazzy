
//Flasher


// global variables ------------------------------------------------------------

const int flasher0 = A1; //PATAS RX DE LOS ULTRASONIDOS
const int flasher1 = A2;
const int flasher2 = A3;
const int flasher3 = A4;
  
// ROS configuration -----------------------------------------------------------



// SETUP -----------------------------------------------------------------------


void setup_flasher()
{
  
    pinMode(flasher0, OUTPUT);
    pinMode(flasher1, OUTPUT);
    pinMode(flasher2,OUTPUT);
    pinMode(flasher3,OUTPUT);
    
    
}


// LOOP ------------------------------------------------------------------------
void loopFlasher()
{
    digitalWrite(flasher0, flasherValues[0]);
    digitalWrite(flasher1, flasherValues[1]);
    digitalWrite(flasher2, flasherValues[2]);
    digitalWrite(flasher3, flasherValues[3]);
}

// Auxiliary functions
