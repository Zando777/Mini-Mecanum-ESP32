// ------------------------------
// ESP32 Mecanum Robot Control
// ------------------------------
// Controls 4 DC motors with AS5600 PWM encoders.
// Web interface for movement control and live encoder readout.
// Author: Alexander Steffen
// Date: October 2025
// Project: Weekend exploration of mecanum kinematics and ESP32
// ------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "wifi_config.h"

WebServer server(80);

// ------------------------------
// Motor & Encoder Configuration
// ------------------------------
// Robot uses 4 DC motors with mecanum wheels in X configuration
// AS5600 encoders provide analog voltage output for position feedback
// Note: AS5600s all have same I2C address (0x36) - using analog voltage mode as workaround
// TODO: Add I2C multiplexer for proper encoder communication
#define NUM_MOTORS 4

// Encoder analog input pins (AS5600 analog voltage output mode)
const int ENC_ANALOG_PINS[NUM_MOTORS] = {32, 33, 34, 35};

// ADC configuration for ESP32
#define ADC_RESOLUTION 12    // 12-bit ADC (0-4095)
#define ADC_VREF 3.3f        // Reference voltage in volts

// Motor driver pins (TB6612FNG H-bridge)
struct MotorPins { int IN1, IN2; };
MotorPins motorPins[NUM_MOTORS] = {
  {14, 27}, // Front Left (FL)
  {5, 18},  // Front Right (FR)
  {13, 12}, // Back Left (BL)
  {26, 25}  // Back Right (BR)
};

// Movement control variables (-1.0 to 1.0 range)
int robotSpeed = 150;  // default speed (0–255 PWM)
float moveX = 0;       // strafe left (-) / right (+) (-1..1)
float moveY = 0;       // forward (+) / back (-) (-1..1)
float rotate = 0;      // rotate CCW (-) / CW (+) (-1..1)

// PWM setup constants for motor control
#define PWM_FREQ 20000  // 20kHz PWM frequency for quiet operation
#define PWM_RES 8       // 8-bit resolution (0-255)

// ------------------------------
// Encoder Reading Functions
// ------------------------------
/**
 * Read analog voltage from AS5600 encoder
 * @param idx Motor index (0-3)
 * @return Voltage (0.0-1.0 normalized) or -1.0 on error
 */
float readEncoderVoltage(int idx) {
  int pin = ENC_ANALOG_PINS[idx];
  int adcValue = analogRead(pin);
  
  // Check for invalid reading (0 might indicate no signal or error)
  if(adcValue == 0) return -1.0f;
  
  // Convert ADC value to normalized voltage (0.0-1.0)
  return (float)adcValue / ((1 << ADC_RESOLUTION) - 1);
}

/**
 * Convert analog voltage to angle in degrees
 * @param voltage Normalized voltage (0.0-1.0)
 * @return Angle in degrees (0-360)
 */
float voltageToDeg(float voltage) {
  return voltage * 360.0f;
}

/**
 * Calculate motor speeds for mecanum wheel kinematics
 * Implements proper wheel vectoring for omnidirectional movement
 * @param mx Strafe component (-1=left, +1=right)
 * @param my Forward component (-1=back, +1=forward)
 * @param rot Rotation component (-1=CCW, +1=CW)
 * @param speeds Output array for motor speeds (-255 to +255)
 */
void calculateMotorSpeeds(float mx, float my, float rot, int speeds[4]) {
  // Mecanum wheel kinematic equations for X configuration
  // FL and BR wheels:  my + mx + rot
  // FR and BL wheels:  my - mx - rot
  float fl =  my + mx + rot;  // Front Left
  float fr =  my - mx - rot;  // Front Right
  float bl =  my - mx + rot;  // Back Left
  float br =  my + mx - rot;  // Back Right

  // Normalize to prevent exceeding motor limits
  float maxMag = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
  if (maxMag > 1.0f) {
    fl /= maxMag; fr /= maxMag; bl /= maxMag; br /= maxMag;
  }

  // Apply minimum PWM threshold and speed scaling
  const int minPWM = 70;  // Motors won't start below this PWM value
  float speedRange = robotSpeed - minPWM;

  for (int i = 0; i < 4; i++) {
    float v = (i==0)?fl:(i==1)?fr:(i==2)?bl:br;
    int pwm = (int)(fabs(v) * speedRange + (v != 0 ? minPWM : 0));
    speeds[i] = (v >= 0) ? pwm : -pwm;  // Negative = reverse direction
  }
}

/**
 * Initialize PWM channels for motor control
 * Uses ESP32 LEDC peripheral for hardware PWM generation
 */
void setupPWM() {
  // Setup PWM channels for motors (2 channels per motor for H-bridge)
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledcSetup(i*2, PWM_FREQ, PWM_RES);       // IN1 pin
    ledcSetup(i*2+1, PWM_FREQ, PWM_RES);     // IN2 pin
    ledcAttachPin(motorPins[i].IN1, i*2);    // Attach IN1 to channel
    ledcAttachPin(motorPins[i].IN2, i*2+1);  // Attach IN2 to channel
  }
}

/**
 * Drive individual motor with specified speed and direction
 * @param idx Motor index (0-3)
 * @param speed Speed value (-255 to +255, negative = reverse)
 */
void driveMotor(int idx, int speed) {
  int pwm = constrain(abs(speed), 0, 255);  // Ensure PWM is within 0-255 range

  if (speed > 0) {
    // Forward: IN1 = PWM, IN2 = 0
    ledcWrite(idx*2, pwm);
    ledcWrite(idx*2+1, 0);
  } else if (speed < 0) {
    // Reverse: IN1 = 0, IN2 = PWM
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, pwm);
  } else {
    // Stop: Both pins low
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, 0);
  }
}

// ------------------------------
// Web Server Handlers
// ------------------------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Mecanum Robot Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial; text-align:center; }
.button { padding:10px 20px; margin:5px; font-size:16px; }
.slider { width:200px; }
.encoder-display { background:#f0f0f0; padding:10px; margin:10px; border-radius:5px; }
</style>
</head>
<body>
<h1>Mecanum Robot Control</h1>
<div>
  <h3>Speed</h3>
  <input type="range" min="0" max="255" value="150" class="slider" id="spd" onchange="setSpeed(this.value)">
  <span id="spv">150</span>
</div>
<div>
  <h3>Move</h3>
  <button class="button" onclick="move('forward')">↑ Forward</button><br>
  <button class="button" onclick="move('left')">← Left</button>
  <button class="button" onclick="move('stop')">Stop</button>
  <button class="button" onclick="move('right')">Right →</button><br>
  <button class="button" onclick="move('backward')">↓ Backward</button>
</div>
<div>
  <h3>Rotate</h3>
  <button class="button" onclick="rotate('ccw')">⟲ CCW</button>
  <button class="button" onclick="rotate('stop')">Stop</button>
  <button class="button" onclick="rotate('cw')">⟳ CW</button>
</div>
<div>
  <h3>Maneuvers</h3>
  <button class="button" onclick="maneuver('straight_forward')">Straight Forward</button>
  <button class="button" onclick="maneuver('straight_backward')">Straight Backward</button>
  <button class="button" onclick="maneuver('sideways_right')">Sideways Right</button>
  <button class="button" onclick="maneuver('sideways_left')">Sideways Left</button>
  <button class="button" onclick="maneuver('diagonal_45')">Diagonal 45°</button>
  <button class="button" onclick="maneuver('diagonal_135')">Diagonal 135°</button>
  <button class="button" onclick="maneuver('diagonal_225')">Diagonal 225°</button>
  <button class="button" onclick="maneuver('diagonal_315')">Diagonal 315°</button>
  <button class="button" onclick="maneuver('pivot_right_forward')">Pivot Right Forward</button>
  <button class="button" onclick="maneuver('pivot_right_backward')">Pivot Right Backward</button>
  <button class="button" onclick="maneuver('pivot_left_forward')">Pivot Left Forward</button>
  <button class="button" onclick="maneuver('pivot_left_backward')">Pivot Left Backward</button>
  <button class="button" onclick="maneuver('rotate_clockwise')">Rotate Clockwise</button>
  <button class="button" onclick="maneuver('rotate_counterclockwise')">Rotate Counterclockwise</button>
  <button class="button" onclick="maneuver('pivot_sideways_front_right')">Pivot Front Right</button>
  <button class="button" onclick="maneuver('pivot_sideways_front_left')">Pivot Front Left</button>
  <button class="button" onclick="maneuver('pivot_sideways_rear_right')">Pivot Rear Right</button>
  <button class="button" onclick="maneuver('pivot_sideways_rear_left')">Pivot Rear Left</button>
</div>
<div class="encoder-display">
  <h3>Encoder Values</h3>
  <pre id="encoders">Loading...</pre>
</div>
<script>
function setSpeed(v){document.getElementById("spv").innerText=v;fetch('/speed?value='+v);}
function move(d){fetch('/move?dir='+d);}
function rotate(d){fetch('/rotate?dir='+d);}
function maneuver(m){fetch('/maneuver?type='+m);}
setInterval(()=>{fetch('/encoders').then(r=>r.text()).then(t=>{document.getElementById('encoders').innerText=t;});},700);
</script>
</body></html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleSpeed() {
  if (server.hasArg("value")) {
    robotSpeed = constrain(server.arg("value").toInt(), 0, 255);
  }
  server.send(200, "text/plain", "OK");
}

void handleMove() {
  String dir = server.arg("dir");
  if (dir == "forward")  { moveY = 1; moveX = 0; }
  else if (dir == "backward") { moveY = -1; moveX = 0; }
  else if (dir == "left") { moveX = -1; moveY = 0; }
  else if (dir == "right") { moveX = 1; moveY = 0; }
  else if (dir == "stop") { moveX = 0; moveY = 0; }
  server.send(200, "text/plain", "OK");
}

void handleRotate() {
  String dir = server.arg("dir");
  if (dir == "cw") rotate = 1;
  else if (dir == "ccw") rotate = -1;
  else rotate = 0;
  server.send(200, "text/plain", "OK");
}

void handleEncoders() {
  String res;
  for (int i = 0; i < NUM_MOTORS; i++) {
    float voltage = readEncoderVoltage(i);
    if (voltage < 0) res += String("Motor ") + char('A'+i) + ": No Signal\n";
    else res += String("Motor ") + char('A'+i) + ": " + String(voltageToDeg(voltage), 1) + "°\n";
  }
  server.send(200, "text/plain", res);
}

void handleManeuver() {
  String type = server.arg("type");
  if (type == "straight_forward") { moveX = 0; moveY = 1; rotate = 0; }
  else if (type == "straight_backward") { moveX = 0; moveY = -1; rotate = 0; }
  else if (type == "sideways_right") { moveX = 1; moveY = 0; rotate = 0; }
  else if (type == "sideways_left") { moveX = -1; moveY = 0; rotate = 0; }
  else if (type == "diagonal_45") { moveX = 0.707; moveY = 0.707; rotate = 0; }
  else if (type == "diagonal_135") { moveX = 0.707; moveY = -0.707; rotate = 0; }
  else if (type == "diagonal_225") { moveX = -0.707; moveY = -0.707; rotate = 0; }
  else if (type == "diagonal_315") { moveX = -0.707; moveY = 0.707; rotate = 0; }
  else if (type == "pivot_right_forward") { moveX = 0; moveY = 1; rotate = 1; }
  else if (type == "pivot_right_backward") { moveX = 0; moveY = -1; rotate = 1; }
  else if (type == "pivot_left_forward") { moveX = 0; moveY = 1; rotate = -1; }
  else if (type == "pivot_left_backward") { moveX = 0; moveY = -1; rotate = -1; }
  else if (type == "rotate_clockwise") { moveX = 0; moveY = 0; rotate = 1; }
  else if (type == "rotate_counterclockwise") { moveX = 0; moveY = 0; rotate = -1; }
  else if (type == "pivot_sideways_front_right") { moveX = 1; moveY = 0; rotate = 1; }
  else if (type == "pivot_sideways_front_left") { moveX = -1; moveY = 0; rotate = -1; }
  else if (type == "pivot_sideways_rear_right") { moveX = 1; moveY = 0; rotate = -1; }
  else if (type == "pivot_sideways_rear_left") { moveX = -1; moveY = 0; rotate = 1; }
  server.send(200, "text/plain", "OK");
}

// ------------------------------
// Setup & Loop
// ------------------------------
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Booting Mecanum Robot ===");

  // Configure motor driver pins as outputs
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorPins[i].IN1, OUTPUT);
    pinMode(motorPins[i].IN2, OUTPUT);
    pinMode(ENC_ANALOG_PINS[i], INPUT);  // Encoder analog inputs
  }

  // Initialize PWM for motor control
  setupPWM();

  // Connect to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Configure web server routes
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/move", handleMove);
  server.on("/rotate", handleRotate);
  server.on("/maneuver", handleManeuver);
  server.on("/encoders", handleEncoders);
  server.begin();
  Serial.println("Web server started on port 80.");
}

void loop() {
  // Process incoming web server requests
  server.handleClient();

  // Calculate and apply motor speeds based on current movement commands
  int speeds[4];
  calculateMotorSpeeds(moveX, moveY, rotate, speeds);
  for (int i = 0; i < 4; i++) {
    driveMotor(i, speeds[i]);
  }

  // Periodic status logging (every 2 seconds)
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 2000) {
    Serial.printf("Speed=%d  X=%.2f  Y=%.2f  R=%.2f\n",
                  robotSpeed, moveX, moveY, rotate);
    lastStatusTime = millis();
  }

  // Small delay to prevent overwhelming the processor
  delay(50);
}

