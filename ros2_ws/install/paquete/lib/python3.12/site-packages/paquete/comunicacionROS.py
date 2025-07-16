
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Importa las librerías necesarias de ROS 2
import rclpy
from rclpy.node import Node

# --- Importar los tipos de mensajes ---
from std_msgs.msg import String  
from std_msgs.msg import Float32 # Para suscriptores de motor
# NUEVO: Importar tipo de mensaje Range para ultrasonido
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
# Importa para comunicación serial y tiempo
import serial
import time
# Importa la librería struct para empaquetado/desempaquetado binario
import struct

# Importar tipo de mensaje para timestamp (Header es parte de muchos mensajes como Range)
# from std_msgs.msg import Header # No es estrictamente necesario importar Header directamente si solo lo usas dentro de Range


# --- CONSTANTES DE AMBOS PROTOCOLOS SERIAL BINARIOS ---
# Mantén las constantes para Comandos Motor (PC -> Arduino - FLOAT)
START_BYTE_MOTORS = 0xA5 # Byte de inicio para comandos de motor
EXPECTED_NUM_VALUES_COMANDO = 4
VALUE_SIZE_COMANDO = 4 # float es 4 bytes
EXPECTED_DATA_SIZE_COMANDO = EXPECTED_NUM_VALUES_COMANDO * VALUE_SIZE_COMANDO
EXPECTED_PACKET_SIZE_COMANDO = 1 + EXPECTED_DATA_SIZE_COMANDO

# Mantén las constantes para Datos RF (Arduino -> Python - INT16)
START_BYTE_RF = 0xB5 # Byte de inicio para datos RF (¡DEBE SER DIFERENTE DE START_BYTE_COMANDO y START_BYTE_ULTRASOUND!)
RF_DATA_COUNT = 8 # Número de valores INT16 que Arduino envía (canales RF)
RF_VALUE_SIZE = 4 # float es 4 bytes
RF_DATA_SIZE = RF_DATA_COUNT * RF_VALUE_SIZE # 8 * 4 = 32 bytes de datos RF
RF_PACKET_SIZE = 1 + RF_DATA_SIZE # Tamaño total del paquete RF (1 byte inicio + 16 bytes datos = 17 bytes)

# --- NUEVAS CONSTANTES PARA EL PROTOCOLO SERIAL BINARIO (Arduino -> Python Datos Ultrasonido - FLOAT) ---
# Deben coincidir con el sketch de Arduino
START_BYTE_ULTRASOUND = 0xC5 # Byte de inicio para datos Ultrasonido (¡DEBE SER ÚNICO y diferente de los otros!)
ULTRASOUND_COUNT = 8 # Número de sensores
ULTRASOUND_VALUE_SIZE = 4 # float es 4 bytes
ULTRASOUND_DATA_SIZE = ULTRASOUND_COUNT * ULTRASOUND_VALUE_SIZE # 8 * 4 = 32 bytes
ULTRASOUND_PACKET_SIZE = 1 + ULTRASOUND_DATA_SIZE # Tamaño total (1 + 32 = 33 bytes)

# --- CONSTANTES PARA LLENAR MENSAJES sensor_msgs/Range (Usar los valores que tenías en Arduino) ---
ULTRASOUND_RADIATION_TYPE = Range.ULTRASOUND # Tipo de radiación para mensaje Range
ULTRASOUND_FIELD_OF_VIEW = 1.0               # Campo de visión (ajusta si conoces el valor exacto)
ULTRASOUND_MIN_RANGE = 0.02                  # Rango mínimo (metros)
ULTRASOUND_MAX_RANGE = 7.0                   # Rango máximo (metros)
# --- CONSTANTES PARA EL PROTOCOLO SERIAL BINARIO DEL FLASHER ---
START_BYTE_FLASHER = 0xF5
EXPECTED_NUM_VALUES_FLASHER = 1
EXPECTED_DATA_SIZE_FLASHER = EXPECTED_NUM_VALUES_FLASHER * 1  # 1 byte para el comando
EXPECTED_PACKET_SIZE_FLASHER = 1 + EXPECTED_DATA_SIZE_FLASHER  # 1 byte de inicio + 1 byte de dato = 2 bytes
class ArduinoSerialCommunicator(Node):
    """
    Nodo ROS 2 para comunicación serial bidireccional con Arduino.
    - Envía comandos de motor (4 floats binarios) a Arduino.
    - Recibe datos RF (8 int16_t binarios) y Ultrasonido (8 floats binarios) de Arduino
      y los publica en topics ROS 2 apropiados.
    """

    def __init__(self):
        """Constructor."""
        super().__init__('arduino_serial_communicator')

        # --- Parámetros del Nodo ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        # Timeout bajo para lectura no bloqueante, permite que read(1) no bloquee si el buffer se vacía justo
        self.declare_parameter('read_timeout', 0.005)
        self.declare_parameter('write_timeout', 0.1)

        self.serial_port_ = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate_ = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.read_timeout_ = self.get_parameter('read_timeout').get_parameter_value().double_value
        self.write_timeout_ = self.get_parameter('write_timeout').get_parameter_value().double_value

        self.serial_connection_ = None
        conexionSerial = False # Bandera para indicar si la conexión serial se estableció correctamente
        # --- Configuración de la Conexión Serial ---
        while not conexionSerial:
            try:
                self.serial_connection_ = serial.Serial(
                    port=self.serial_port_,
                    baudrate=self.baud_rate_,
                    timeout=self.read_timeout_, # Usar timeout bajo
                    write_timeout=self.write_timeout_
                    )
                self.get_logger().info(f'Conectado al puerto serial {self.serial_port_} a {self.baud_rate_} baudios.')
                time.sleep(0.5) # Esperar un poco después de abrir la conexión
                self.serial_connection_.flushInput() # Limpiar buffer de entrada
                self.serial_connection_.flushOutput() # Limpiar buffer de salida
                conexionSerial = True # Marcar como conexión exitosa

            except serial.SerialException as e:
                self.get_logger().error(f'Error al abrir el puerto serial {self.serial_port_}: {e}')
                # El nodo puede seguir existiendo pero sin conexión serial


        # --- Publicador para datos leídos de Arduino (String) ---
        # Mantenemos si Arduino todavía envía Strings para otros propósitos
        self.data_publisher_ = self.create_publisher(String, 'arduino_data', 10)

        # --- PUBLICADORES PARA DATOS RF RECIBIDOS (8 canales, como Float32) ---
        self.rf_pub_1 = self.create_publisher(Float32, '/geneseas/rf/channel1', 10)
        self.rf_pub_2 = self.create_publisher(Float32, '/geneseas/rf/channel2', 10)
        self.rf_pub_3 = self.create_publisher(Float32, '/geneseas/rf/channel3', 10)
        self.rf_pub_4 = self.create_publisher(Float32, '/geneseas/rf/channel4', 10)
        self.rf_pub_5 = self.create_publisher(Float32, '/geneseas/rf/channel5', 10)
        self.rf_pub_6 = self.create_publisher(Float32, '/geneseas/rf/channel6', 10)
        self.rf_pub_7 = self.create_publisher(Float32, '/geneseas/rf/channel7', 10)
        self.rf_pub_8 = self.create_publisher(Float32, '/geneseas/rf/channel8', 10)
        self.rf_publishers = [
            self.rf_pub_1, self.rf_pub_2, self.rf_pub_3, self.rf_pub_4,
            self.rf_pub_5, self.rf_pub_6, self.rf_pub_7, self.rf_pub_8
        ]
        self.get_logger().info(f'Creados {len(self.rf_publishers)} publicadores para canales RF.')

        # --- PUBLICADORES PARA DATOS ULTRASONIDO RECIBIDOS (8 sensores, como Range) ---
        # Definir los nombres de los topics y frame_ids en el ORDEN NUMÉRICO 1 a 8
        # porque Arduino está llenando ultrasoundRanges[] en este orden antes de empaquetar
        self.ultrasound_topics_ordered = [
            '/geneseas/sonar1/distance', '/geneseas/sonar2/distance',
            '/geneseas/sonar3/distance', '/geneseas/sonar4/distance',
            '/geneseas/sonar5/distance', '/geneseas/sonar6/distance',
            '/geneseas/sonar7/distance', '/geneseas/sonar8/distance'
        ]
        self.ultrasound_frame_ids_ordered = [
             "geneseas/sonar1_link", "geneseas/sonar2_link",
             "geneseas/sonar3_link", "geneseas/sonar4_link",
             "geneseas/sonar5_link", "geneseas/sonar6_link",
             "geneseas/sonar7_link", "geneseas/sonar8_link"
        ]
        # Crear los publicadores Range
        self.ultrasound_pubs_ordered = []
        if len(self.ultrasound_topics_ordered) == ULTRASOUND_COUNT:
            for i in range(ULTRASOUND_COUNT):
                pub = self.create_publisher(Range, self.ultrasound_topics_ordered[i], 10)
                self.ultrasound_pubs_ordered.append(pub)
            self.get_logger().info(f'Creados {len(self.ultrasound_pubs_ordered)} publicadores Range para ultrasonido (orden 1-8).')
        else:
             self.get_logger().error(f"Error: El número de topics de ultrasonido ({len(self.ultrasound_topics_ordered)}) no coincide con ULTRASOUND_COUNT ({ULTRASOUND_COUNT}).")

        self.timersubs = self.create_timer(10, self.buildSetup)
        self.get_logger().info('Nodo de comunicación serial bidireccional con Arduino (Binario) inicializado.')
        
    def buildSetup(self):
        # --- Suscriptores para comandos de Motor (Reciben Float32) ---
        self.motor_topics = [
            '/geneseas/motor_l', '/geneseas/motor_r', '/geneseas/motor_c', '/geneseas/motor_d'
        ]
        if len(self.motor_topics) != EXPECTED_NUM_VALUES_COMANDO:
            self.get_logger().error(f"Error: El número de topics de motor ({len(self.motor_topics)}) no coincide con EXPECTED_NUM_VALUES_COMANDO ({EXPECTED_NUM_VALUES_COMANDO}).")

        self.motor_values = {}
        self.received_since_last_send = {}

        self.subscribers = []
        for topic in self.motor_topics:
            subm = self.create_subscription(
                Float32,
                topic,
                lambda msg, t=topic: self.motor_command_callback(msg, t),
                10
            )
            self.subscribers.append(subm)
            self.motor_values[topic] = None
            self.received_since_last_send[topic] = False

        self.get_logger().info(f'Creados {len(self.motor_topics)} suscriptores (Float32) para comandos de motor.')



        # --- Temporizador para Leer Desde Arduino ---
        # Este temporizador llamará a la función que lee del serial y maneja múltiples protocolos
        self.read_timer_ = self.create_timer(0.01, self.read_from_arduino) # Timer cada 10ms
        # crear suscriptor a topico geneseas/flasher
        
        #hacer 3 suscriptores de flasher pero de distintos topicos flasher 1, flasher2, flasher3
        for i in range(0, 1):
            subflasher = self.create_subscription(
                        Bool,
                        f'/geneseas/flasher{i}',
                        lambda msg, i=i: self.send_flasher_command_to_arduino(msg.data,i),
                        10
                        )
            
            self.get_logger().info(f'Suscriptor creado para /geneseas/flasher{i}.')
        self.timersubs.cancel() # Cancelar el temporizador de configuración una vez que se hayan creado los suscriptores
        self.get_logger().info('Nodo de comunicación serial bidireccional con Arduino (Binario) inicializado.')
    # --- FUNCIÓN PRINCIPAL DE LECTURA DESDE ARDUINO ---
    # Maneja la recepción de diferentes protocolos binarios (y opcionalmente texto).
    def read_from_arduino(self):
        """
        Método llamado por el temporizador para leer datos del Arduino.
        Intenta leer paquetes binarios RF o Ultrasonido primero. Si no, intenta leer líneas de texto.
        """
        #self.get_logger().info("Reading from Arduino...")
        #flag_rf = False # Variable para indicar si se recibió un paquete RF
        #flag_ultrasound = False # Variable para indicar si se recibió un paquete Ultrasonido
        #self.get_logger().info('Llamando a read_from_arduino()...')
        if self.serial_connection_ and self.serial_connection_.is_open:
            try:
                # Procesar todos los bytes disponibles en el buffer en esta llamada al timer
                # Bucle while para manejar múltiples paquetes/mensajes si llegan juntos
                while self.serial_connection_.in_waiting > 0:
                    #self.get_logger().info(f'Bytes disponibles en el buffer: {self.serial_connection_.in_waiting}')


                    # Intentar leer el primer byte - esto lo consume del buffer
                    # Con timeout bajo, si no llega un byte inmediatamente, read(1) puede tardar hasta el timeout
                    # Pero el while self.in_waiting > 0 debería asegurar que hay al menos 1 byte
                    try:
                       # self.get_logger().info("Reading first byte from serial...")
                        first_byte_raw = self.serial_connection_.read(1)
                        #self.get_logger().info(f"Primer byte leído: {first_byte_raw.hex()} (in_waiting={self.serial_connection_.in_waiting})")
                    except serial.SerialTimeoutException:
                        # Esto no debería pasar si in_waiting > 0 a menos que haya un problema de driver/OS
                        self.get_logger().warn("Serial timeout during read(1) despite in_waiting > 0. Exiting read loop.")
                        break # Salir del bucle while
                    except Exception as e:
                         self.get_logger().error(f"Error reading first byte: {e}. Exiting read loop.")
                         break # Salir del bucle while

                    # Si read(1) no devolvió un byte (raro con in_waiting > 0) o timeout
                    if len(first_byte_raw) == 0:
                         # Esto puede ocurrir si el buffer se vació justo entre el while check y el read(1)
                         self.get_logger().debug("Read(1) returned 0 bytes when in_waiting > 0. Buffer likely just emptied.")
                         break # Salir del bucle while (no hay más bytes por ahora)


                    start_byte = ord(first_byte_raw) # Obtener el valor entero del byte leído

                    # --- Verificar si es un paquete Binario RF ---
                    #if start_byte == START_BYTE_RF and flag_rf == False:
                    if start_byte == START_BYTE_RF:
                        # Verificar si el RESTO del payload del paquete RF está disponible
                        if self.serial_connection_.in_waiting >= RF_DATA_SIZE:
                            # Leer los bytes de datos RF
                            data_bytes = self.serial_connection_.read(RF_DATA_SIZE)

                            if len(data_bytes) == RF_DATA_SIZE:
                                # --- ¡Paquete RF completo recibido y validado! ---
                                try:
                                    #self.get_logger().info(f"Received RF data packet: {data_bytes.hex()}")
                                    # '<8h' para 8 signed shorts (int16_t) en little-endian
                                    unpacked_data = struct.unpack('<8f', data_bytes)

                                    # --- Publicar los valores RF (int16_t) como mensajes Float32 ---
                                    if len(unpacked_data) == RF_DATA_COUNT: # Debería ser 8
                                        for i in range(RF_DATA_COUNT):
                                            msg = Float32()
                                            msg.data = float(unpacked_data[i]) # Conversión implícita de int a float
                                            if i < len(self.rf_publishers):
                                                #self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data}")
                                                """
                                                if( i==0):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 1)")
                                                elif(i==1):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 2)")
                                                elif(i==2):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 3)")
                                                elif(i==3):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 4)")
                                                elif(i==4):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 5)")
                                                elif(i==5):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 6)")
                                                elif(i==6):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 7)")
                                                elif(i==7):
                                                    self.get_logger().info(f"Publishing RF data to topic {self.rf_publishers[i].topic}: {msg.data} (canal 8)")
                                                """
                                                self.rf_publishers[i].publish(msg)
                                                
                                                #flag_rf = True # Indicar que se recibió un paquete RF
                                            else:
                                                 self.get_logger().warn(f"RF Publisher missing for index {i}.")
                                        #ProcesoRF = True
                                        return
                                    else: # El número de valores desempaquetados no coincide
                                        self.get_logger().error(f"RF Unpack Count Error: Expected {RF_DATA_COUNT} values, got {len(unpacked_data)} after unpacking.")

                                except struct.error as e: # struct.unpack falló
                                    self.get_logger().error(f"Error unpacking RF data ({len(data_bytes)} bytes): {e}")
                                    # Los bytes ya fueron consumidos
                                    pass # Continuar el bucle while para el siguiente byte

                            else: # No se leyó la cantidad esperada de bytes de datos después del byte de inicio
                                self.get_logger().warn(f"Partial RF packet data read after start byte: Expected {RF_DATA_SIZE}, got {len(data_bytes)}. Discarding partial data.")
                                # Los bytes leídos ya fueron consumidos, el bucle continuará intentando resincronizar en el siguiente byte

                        else:
                            # No hay suficientes bytes para el payload completo del paquete RF aún.
                            # El byte de inicio ya fue consumido. Salir del bucle while y esperar la próxima llamada al timer.
                            break # Salir del while self.in_waiting > 0

                    # --- Verificar si es un paquete Binario Ultrasonido ---
                    #elif start_byte == START_BYTE_ULTRASOUND and flag_ultrasound == False:
                    elif start_byte == START_BYTE_ULTRASOUND:
                        #self.get_logger().info(f"Received Ultrasonido start byte: {start_byte:02X} (in_waiting={self.serial_connection_.in_waiting})")
                         # Verificar si el RESTO del payload del paquete Ultrasonido está disponible
                        if self.serial_connection_.in_waiting >= ULTRASOUND_DATA_SIZE:
                            # Leer los bytes de datos Ultrasonido
                            data_bytes = self.serial_connection_.read(ULTRASOUND_DATA_SIZE)

                            if len(data_bytes) == ULTRASOUND_DATA_SIZE:
                                # --- ¡Paquete Ultrasonido completo recibido y validado! ---
                                try:
                                    # '<8f' para 8 floats en little-endian
                                    unpacked_data = struct.unpack('<8f', data_bytes)

                                    # --- Publicar los valores de Ultrasonido (float) como mensajes Range ---
                                    if len(unpacked_data) == ULTRASOUND_COUNT: # Debería ser 8
                                        for i in range(ULTRASOUND_COUNT):
                                            # Crear un nuevo mensaje Range para este sensor
                                            msg = Range()
                                            # Llenar campos fijos (usando constantes)
                                            msg.radiation_type = ULTRASOUND_RADIATION_TYPE
                                            msg.field_of_view = ULTRASOUND_FIELD_OF_VIEW
                                            msg.min_range = ULTRASOUND_MIN_RANGE
                                            msg.max_range = ULTRASOUND_MAX_RANGE

                                            # Asignar el valor de distancia (float) desempaquetado
                                            # unpacked_data[i] corresponde al Sonar (i+1) porque Arduino los ordena 1-8 en ultrasoundRanges[]
                                            msg.range = float(unpacked_data[i]) # Asegurarse de que es float
                                            self.get_logger().info(f"Publishing Ultrasonido data to topic {self.ultrasound_topics_ordered[i]}: {msg.range} m")
                                            # Llenar el encabezado con timestamp y frame_id
                                            msg.header.stamp = self.get_clock().now().to_msg() # Timestamp actual
                                            # Usar el ORDEN NUMÉRICO de frame_ids/topics (1-8)
                                            if i < len(self.ultrasound_frame_ids_ordered):
                                                msg.header.frame_id = self.ultrasound_frame_ids_ordered[i]
                                            else:
                                                self.get_logger().warn(f"Ultrasound frame_id missing for index {i}. Using empty string.")
                                                msg.header.frame_id = ""


                                            # Publicar el mensaje Range usando el publicador correspondiente (en el ORDEN NUMÉRICO 1-8)
                                            if i < len(self.ultrasound_pubs_ordered):
                                                 self.ultrasound_pubs_ordered[i].publish(msg)
                                            else:
                                                 self.get_logger().warn(f"Ultrasound publisher missing for index {i}.")

                                        #flag_ultrasound = True # Indicar que se recibió un paquete Ultrasonido
                                    else: # El número de valores desempaquetados no coincide
                                        self.get_logger().error(f"Ultrasound Unpack Count Error: Expected {ULTRASOUND_COUNT}, got {len(unpacked_data)} after unpacking.")

                                except struct.error as e: # struct.unpack falló
                                    self.get_logger().error(f"Error unpacking Ultrasound data ({len(data_bytes)} bytes): {e}")
                                    # Los bytes ya fueron consumidos
                                    pass # Continuar el bucle while

                            else: # No se leyó la cantidad esperada de bytes de datos después del byte de inicio
                                self.get_logger().warn(f"Partial Ultrasound packet data read after start byte: Expected {ULTRASOUND_DATA_SIZE}, got {len(data_bytes)}. Discarding partial data.")
                                # Los bytes leídos ya fueron consumidos, el bucle continuará intentando resincronizar

                        else:
                            # No hay suficientes bytes para el payload completo del paquete Ultrasonido aún.
                            # El byte de inicio ya fue consumido. Salir del bucle while.
                            break # Salir del while self.in_waiting > 0
                        """
                    elif flag_rf == True and flag_ultrasound == True:        
                        self.serial_connection_.flushInput() # Limpiar el buffer de entrada para evitar leer bytes no deseados     
                        flag_rf = False # Reiniciar la bandera RF
                        flag_ultrasound = False # Reiniciar la bandera Ultrasonido
                        break
                    """
                    # --- Manejar byte que NO es un Start Byte conocido ---
                    else:
                        # El byte leído (first_byte_raw) no fue un START_BYTE_RF ni START_BYTE_ULTRASOUND.
                        # Este byte probablemente es ruido o parte de un mensaje inesperado.
                        # Ya fue consumido por read(1). Simplemente continuamos el bucle
                        # para verificar el siguiente byte disponible. Esto efectivamente lo descarta.

                        # Si Arduino envía también Strings (terminadas en \n) y esperas recibirlos
                        # de forma confiable mezclados con los binarios, necesitarías una lógica
                        # mucho más compleja aquí (ej. bufferizar bytes hasta ver un \n,
                        # pero si el primer byte no fue un START_BYTE binario).
                        # Para la mayoría de los casos, si recibes un byte que no es un byte de inicio,
                        # lo más seguro es descartarlo y seguir buscando un byte de inicio válido.

                       

                        pass # El byte fue consumido por read(1). Seguir al siguiente en el buffer.

                # Fin de while self.serial_connection_.in_waiting > 0:

            except serial.SerialException as e:
                self.get_logger().error(f'General serial error in read_from_arduino: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in read_from_arduino: {e}')

        # Si la conexión serial no está abierta, se maneja en __init__ y simplemente no se lee.
    

    def motor_command_callback(self, msg, topic_name):
        """
        Callback para suscriptores de motor (recibe Float32). Almacena y verifica condición de envío.
        """
        self.get_logger().info(f'Received from {topic_name}: {msg.data}')
        self.motor_values[topic_name] = msg.data
        self.received_since_last_send[topic_name] = True
        all_topics_updated = all(self.received_since_last_send.values())
        if all_topics_updated:
            self.get_logger().debug('Condition met. Preparing BINARY COMMAND send to Arduino.')
            self.send_motor_commands_to_arduino_binary()
            self.received_since_last_send = {t: False for t in self.motor_topics}


    def send_motor_commands_to_arduino_binary(self, timeout=0.1):
        """
        Takes the latest motor values (Float), packs them into a binary packet
        (START_BYTE_COMANDO + 4 floats), and sends it over serial.
        Called from motor_command_callback when the condition is met.
        """
        if self.serial_connection_ and self.serial_connection_.is_open:
            try:
                float_values_to_send = []
                for topic_name in self.motor_topics:
                    motor_float_value = self.motor_values.get(topic_name)
                    if motor_float_value is None:
                        self.get_logger().error(f"Internal error: Condition met but value is None for {topic_name}. Aborting BINARY COMMAND send.")
                        self.received_since_last_send = {t: False for t in self.motor_topics}
                        return # Abort send

                    float_values_to_send.append(motor_float_value)

                if len(float_values_to_send) != EXPECTED_NUM_VALUES_COMANDO:
                     self.get_logger().error(f"Internal error: Expected {EXPECTED_NUM_VALUES_COMANDO} floats to send, but got {len(float_values_to_send)}. Aborting BINARY COMMAND send.")
                     self.received_since_last_send = {t: False for t in self.motor_topics}
                     return # Abort send


                # Pack START_BYTE_COMANDO (b) and the four floats (ffff)
                # '<' indicates little-endian byte order (common on Arduino AVR/x86).
                packed_data = struct.pack('<Bffff', START_BYTE_MOTORS, *float_values_to_send)

                # Optional: Check packed size
                if len(packed_data) != EXPECTED_PACKET_SIZE_COMANDO:
                     self.get_logger().error(f"Packing error: Incorrect BINARY COMMAND size ({len(packed_data)} bytes). Expected {EXPECTED_PACKET_SIZE_COMANDO}. Aborting send.")
                     # Don't reset received_since_last_send here, it's a code error.
                     return # Abort send

                # self.get_logger().info(f'Sending to Arduino (binary COMMAND, {len(packed_data)} bytes): {packed_data.hex()}')
                # Use serial_connection_.write() with optional timeout
                # The write_timeout is set during serial port configuration
                self.serial_connection_.write(packed_data)

            except serial.SerialTimeoutException:
                self.get_logger().warn(f'Timeout during serial write (BINARY COMMAND send).')
            except Exception as e:
                self.get_logger().error(f'Unexpected error during serial write (BINARY COMMAND send): {e}')

        else:
            self.get_logger().warn('Attempted to write to Arduino (BINARY COMMAND send), but serial connection is not open.')


    # -Método para enviar comando de flasher ---
    def send_flasher_command_to_arduino(self, value, flasher_id):
        if self.serial_connection_ and self.serial_connection_.is_open:
            try:
                byte_value = 1 if value else 0
                identifierByte = START_BYTE_FLASHER + flasher_id  # Asegurarse de que flasher_id es 1, 2 o 3
                packed_data = struct.pack('<BB', identifierByte, byte_value)
                self.serial_connection_.write(packed_data)
                self.get_logger().info(f"Sent flasher command to Arduino: {byte_value}")
            except Exception as e:
                self.get_logger().error(f"Error sending flasher command: {e}")
        else:
            self.get_logger().warn('Attempted to write flasher command, but serial connection is not open.')


    def destroy_node(self):
        """
        Method called when the node is about to be destroyed. Closes the serial connection.
        """
        self.get_logger().info('Shutting down serial communication node with Arduino.')
        if self.serial_connection_ and self.serial_connection_.is_open:
            self.serial_connection_.close()
            self.get_logger().info('Serial connection closed.')
        super().destroy_node()



def main(args=None):
    """
    Main function that runs when the script is executed.
    """
    print("Starting Arduino Serial Communicator Node...")
    rclpy.init(args=args)
    arduino_communicator_node = None # Inicializa a None
    try:
        arduino_communicator_node = ArduinoSerialCommunicator()
        print("Node started. Spinning...")
        rclpy.spin(arduino_communicator_node)
    except (KeyboardInterrupt):
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if arduino_communicator_node is not None: # Verifica si el nodo fue creado
            arduino_communicator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
