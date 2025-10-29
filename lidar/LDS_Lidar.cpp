#ifndef ESP32
  #error This example runs on ESP32
#endif


// IGNORE pins absent from your Lidar model (often EN, PWM)
const uint8_t LIDAR_GPIO_EN = 19; // ESP32 GPIO connected to Lidar EN pin
const uint8_t LIDAR_GPIO_RX = 17; // ESP32 GPIO connected to Lidar RX pin
//const uint8_t LIDAR_GPIO_TX = 16; // ESP32 GPIO connected to Lidar TX pin
const uint8_t LIDAR_GPIO_PWM = 15;// ESP32 GPIO connected to Lidar PWM pin


//#define INVERT_PWM_PIN // UNCOMMENT if using PWM pin and PWM LOW enables the motor

#define XIAOMI_LDS02RR


// UNCOMMENT debug option(s)
// and increase SERIAL_MONITOR_BAUD to MAX possible
#define DEBUG_GPIO
//#define DEBUG_PACKETS
//#define DEBUG_SERIAL_IN
//#define DEBUG_SERIAL_OUT

const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 2;
const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 1024;
const uint16_t PRINT_EVERY_NTH_POINT = 20;
const uint16_t HEX_DUMP_WIDTH = 16;

#define MAX_SCAN_POINTS 360  // Assuming ~1 point per degree
float scan_angles[MAX_SCAN_POINTS];
float scan_distances[MAX_SCAN_POINTS];
int scan_point_count = 0;

#include "lds_all_models.h"

HardwareSerial LidarSerial(1);
LDS *lidar;
uint16_t hex_dump_pos = 0;

void setupLidar() {

  #if defined(NEATO_XV11)
  lidar = new LDS_NEATO_XV11();
  #elif defined(SLAMTEC_RPLIDAR_A1)
  lidar = new LDS_RPLIDAR_A1();
  #elif defined(XIAOMI_LDS02RR)
  lidar = new LDS_LDS02RR();
  #elif defined(YDLIDAR_SCL)
  lidar = new LDS_YDLIDAR_SCL();
  #elif defined(YDLIDAR_X2_X2L)
  lidar = new LDS_YDLIDAR_X2_X2L();
  #elif defined(YDLIDAR_X3)
  lidar = new LDS_YDLIDAR_X3();
  #elif defined(YDLIDAR_X3_PRO)
  lidar = new LDS_YDLIDAR_X3_PRO();
  #elif defined(_3IROBOTIX_DELTA_2G)
  lidar = new LDS_DELTA_2G();
  #elif defined(_3IROBOTIX_DELTA_2A_115200)
  lidar = new LDS_DELTA_2A_115200();
  #elif defined(_3IROBOTIX_DELTA_2A)
  lidar = new LDS_DELTA_2A_230400();
  #elif defined(_3IROBOTIX_DELTA_2B)
  lidar = new LDS_DELTA_2B();
  #elif defined(LDROBOT_LD14P)
  lidar = new LDS_LDROBOT_LD14P();
  #elif defined(YDLIDAR_X4)
  lidar = new LDS_YDLIDAR_X4();
  #elif defined(YDLIDAR_X4_PRO)
  lidar = new LDS_YDLIDAR_X4_PRO();
  #elif defined(CAMSENSE_X1)
  lidar = new LDS_CAMSENSE_X1();
  #else
    #error "Define a Lidar model"
  #endif

  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);

  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);
  //LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_RX, LIDAR_GPIO_TX);



  lidar->init();
  //lidar->stop();
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);

  Serial.println();
  Serial.print("ESP IDF version ");
  Serial.println(ESP_IDF_VERSION_MAJOR);

  setupLidar();

  Serial.print("LiDAR model ");
  Serial.print(lidar->getModelName());
  Serial.print(", baud rate ");
  Serial.print(lidar->getSerialBaudRate());
  Serial.print(", TX GPIO ");
  Serial.print(LIDAR_GPIO_TX);
  Serial.print(", RX GPIO ");
  Serial.println(LIDAR_GPIO_RX);

  LDS::result_t result = lidar->start();
  Serial.print("startLidar() result: ");
  Serial.println(lidar->resultCodeToString(result));

  // Set desired rotations-per-second for some LiDAR models
  // Lower value = slower spin (e.g., 2.0f for ~2 Hz, 1.0f for ~1 Hz)
  // Temporarily increase for testing detection
  lidar->setScanTargetFreqHz(4.0f);
}

void printByteAsHex(uint8_t b) {
  if (b < 16)
    Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

void printBytesAsHex(const uint8_t * buffer, uint16_t length) {
  if (length == 0)
    return;

  for (uint16_t i = 0; i < length; i++) {
    printByteAsHex(buffer[i]);
  }
}

int lidar_serial_read_callback() {
  #ifdef DEBUG_SERIAL_IN
  int ch = LidarSerial.read();
  if (ch != -1) {
    if (hex_dump_pos++ % HEX_DUMP_WIDTH == 0)
      Serial.println();
    printByteAsHex(ch);
  }
  return ch;
  #else
  return LidarSerial.read();
  #endif
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  #ifdef DEBUG_SERIAL_OUT
  Serial.println();
  Serial.print('>');
  printBytesAsHex(buffer, length);
  Serial.println();
  #endif
  
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  static int i=0;

  // Debug: Log quality values to check detection
  if (i % 50 == 0) {
    Serial.print("Quality: ");
    Serial.println(quality);
  }

  // Collect all scan points for visualization
  if (scan_point_count < MAX_SCAN_POINTS) {
    scan_angles[scan_point_count] = angle_deg;
    scan_distances[scan_point_count] = distance_mm;
    scan_point_count++;
  }

  if (scan_completed) {
    // Output full scan data in CSV format for Processing sketch
    Serial.println("SCAN_START");
    for (int j = 0; j < scan_point_count; j++) {
      Serial.print(scan_angles[j]);
      Serial.print(",");
      Serial.println(scan_distances[j]);
    }
    Serial.println("SCAN_END");

    // Reset for next scan
    scan_point_count = 0;

    Serial.print("Scan completed; scans-per-second ");
    Serial.println(lidar->getCurrentScanFreqHz());
  }

  if (i % PRINT_EVERY_NTH_POINT == 0) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.println(angle_deg);
  }
  i++;
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {

  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_GPIO_EN : LIDAR_GPIO_PWM;

  #ifdef DEBUG_GPIO
  Serial.print("GPIO ");
  Serial.print(pin);
  Serial.print(' ');
  Serial.print(lidar->pinIDToString(lidar_pin));
  Serial.print(" mode set to ");
  Serial.println(lidar->pinStateToString((LDS::lds_pin_state_t) int(value)));
  #endif

  if (value <= (float)LDS::DIR_INPUT) {

    // Configure pin direction
    if (value == (float)LDS::DIR_OUTPUT_PWM) {

      #if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
      #else
      if (!ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
      #endif
    } else
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);

    return;
  }

  if (value < (float)LDS::VALUE_PWM) {
    // Set constant output
    // TODO invert PWM as needed
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);

  } else {
    #ifdef DEBUG_GPIO
    Serial.print("PWM value set to ");
    Serial.print(value);
    #endif

    // set PWM duty cycle
    #ifdef INVERT_PWM_PIN
    value = 1 - value;
    #endif

    int pwm_value = ((1<<LIDAR_PWM_BITS)-1)*value;

    #if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
    #else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
    #endif

    #ifdef DEBUG_GPIO
    Serial.print(' ');
    Serial.println(pwm_value);
    #endif
  }
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  #ifdef DEBUG_PACKETS
  Serial.println();
  Serial.print("Packet callback, length=");
  Serial.print(length);
  Serial.print(", scan_completed=");
  Serial.println(scan_completed);
  #endif
  
  return;
}

void loop() {
  lidar->loop();
}
