const uint8_t START_BYTEM = 0xA5;
const uint8_t START_BYTEF0 = 0xF5;
const uint8_t START_BYTEF1 = 0xF6;
const uint8_t START_BYTEF2 = 0xF7;
const uint8_t START_BYTEF3 = 0XF8;
const size_t VALUE_SIZE_FLOAT = sizeof(float);


void readAndParseBinaryCommands(float motorValuesArray[], size_t arraySize,int flasherValues[]) {
    const size_t expectedDataBytesM = arraySize * VALUE_SIZE_FLOAT;
    uint8_t tempBufferM[expectedDataBytesM];

    bool motorOk = false;
    
    bool flashOk[CANT_FLASHERS]={false,false,false,false};

//  ACA ABAJO QUEDARIA AGREGAR LAS OTRAS 2 POSICIONES DEL VECTOR
    while (Serial.available() > 0 && (!motorOk || !flashOk[0])) {
        int byte = Serial.peek();

        // MOTOR
        if (!motorOk && byte == START_BYTEM && Serial.available() >= expectedDataBytesM + 1) {
            Serial.read();  // consumir byte de inicio
            if (Serial.readBytes(tempBufferM, expectedDataBytesM) == expectedDataBytesM) {
                FloatConverter f;
                for (size_t i = 0; i < arraySize; ++i) {
                    memcpy(&f.b, &tempBufferM[i * VALUE_SIZE_FLOAT], VALUE_SIZE_FLOAT);
                    motorValuesArray[i] = f.f;
                }
                motorOk = true;
            }
        }

        // FLASH
        else if (!flashOk[0] && byte == START_BYTEF0 && Serial.available() >= 2) {
            Serial.read();             // consumir START_BYTEF
            uint8_t value = Serial.read();
            if (value == 0 || value == 1) 
            {
                flasherValues[0] = value;
                flashOk[0] = true;
            }
        }
        
        else if (!flashOk[1] && byte == START_BYTEF1 && Serial.available() >= 2) {
            Serial.read();             // consumir START_BYTEF
            uint8_t value = Serial.read();
            if (value == 0 || value == 1) 
            {
                flasherValues[1] = value;
                flashOk[1] = true;
            }
        }
        
        else if (!flashOk[2] && byte == START_BYTEF2 && Serial.available() >= 2) {
            Serial.read();             // consumir START_BYTEF
            uint8_t value = Serial.read();
            if (value == 0 || value == 1) 
            {
                flasherValues[2] = value;
                flashOk[2] = true;
            }
        }
        else if (!flashOk[3] && byte == START_BYTEF3 && Serial.available() >= 2) {
            Serial.read();             // consumir START_BYTEF
            uint8_t value = Serial.read();
            if (value == 0 || value == 1) 
            {
                flasherValues[3] = value;
                flashOk[3] = true;
            }
        }
        
        


        // INVALIDO
        else {
            Serial.read(); // descartar byte no reconocido
        }
    }

    // Limpiar el puerto al final
    //while (Serial.available() > 0) Serial.read();
}
