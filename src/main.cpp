/**
 * ESP32 Mecanum Robot Control System - Simplified & Robustified
 *
 * Controls 4 DC motors with mecanum wheels using basic X/Y movement.
 * Simplified web interface with essential movements only.
 * Maintains OTA functionality with improved motor control reliability.
 * RIGHT-SIDE MOTORS (FR, BR) INVERTED for proper mecanum operation.
 *
 * Author: Alexander Steffen
 * Date: October 2025
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include "wifi_config.h"

WebServer server(80);

// ------------------------------
// Motor Configuration (Preserved pins)
// ------------------------------
#define NUM_MOTORS 4

// Motor driver pins (TB6612FNG H-bridge) - PRESERVED
struct MotorPins { int IN1, IN2; };
MotorPins motorPins[NUM_MOTORS] = {
  {14, 27}, // Front Left (FL)
  {5, 18},  // Front Right (FR) - DIRECTIONS INVERTED
  {13, 12}, // Back Left (BL)
  {26, 25}  // Back Right (BR) - DIRECTIONS INVERTED
};

// Encoder analog input pins (AS5600 analog output mode)
const int ENC_ANALOG_PINS[NUM_MOTORS] = {32, 33, 34, 35};

// Movement control variables (simplified)
int robotSpeed = 150;  // PWM speed (0-255)
float moveX = 0;       // strafe: -1 (left) to +1 (right)
float moveY = 0;       // forward: -1 (back) to +1 (forward)
float moveRot = 0;     // rotation: -1 (CCW) to +1 (CW)

// Encoder calibration data
float encoderOffsets[NUM_MOTORS] = {0, 0, 0, 0};  // Calibration offsets
bool encoderCalibrated = false;                   // Calibration status

// PWM Configuration - Improved for reliability
#define PWM_FREQ 20000  // 20kHz (quiet operation)
#define PWM_RES 8       // 8-bit resolution (0-255)

/**
 * Calibrate all encoders to have the same baseline value
 * Reads encoder values and calculates offsets for uniform starting positions
 */
void calibrateEncoders() {
  Serial.println("\n=== Encoder Calibration Starting ===");
  Serial.println("Place all wheels in neutral position...");
  delay(2000);  // Give user time to position wheels
  
  // Take multiple readings and average for stability
  const int NUM_READINGS = 10;
  float avgEncoderVoltages[NUM_MOTORS] = {0};
  
  for (int reading = 0; reading < NUM_READINGS; reading++) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      int pin = ENC_ANALOG_PINS[i];
      int adcValue = analogRead(pin);
      if (adcValue >= 0 && adcValue <= 4095) {
        float voltage = (float)adcValue * 3.3f / 4095.0f;
        avgEncoderVoltages[i] += voltage;
      }
      delay(10);
    }
    delay(50);  // Small delay between readings
  }
  
  // Calculate averages
  for (int i = 0; i < NUM_MOTORS; i++) {
    avgEncoderVoltages[i] /= NUM_READINGS;
  }
  
  // Calculate target voltage (average of all encoders)
  float targetVoltage = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    targetVoltage += avgEncoderVoltages[i];
  }
  targetVoltage /= NUM_MOTORS;
  
  // Calculate calibration offsets
  for (int i = 0; i < NUM_MOTORS; i++) {
    encoderOffsets[i] = targetVoltage - avgEncoderVoltages[i];
  }
  
  // Apply calibration and display results
  Serial.println("Calibration Results:");
  Serial.println("Motor | Raw (V) | Offset (V) | Calibrated (V)");
  Serial.println("------|---------|------------|---------------");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    char motorName = 'A' + i;
    float calibratedVoltage = avgEncoderVoltages[i] + encoderOffsets[i];
    Serial.printf("  %c   |  %.3f   |   %.3f     |   %.3f\n",
                  motorName, avgEncoderVoltages[i], encoderOffsets[i], calibratedVoltage);
  }
  
  Serial.println("=== Encoder Calibration Complete ===\n");
  encoderCalibrated = true;
}

/**
 * Read analog voltage from AS5600 encoder with calibration applied
 * @param idx Motor index (0-3)
 * @return Voltage (0.0-3.3V) or -1.0 on error
 */
float readEncoderVoltage(int idx) {
  int pin = ENC_ANALOG_PINS[idx];
  int adcValue = analogRead(pin);
  if (adcValue < 0 || adcValue > 4095) return -1;  // Invalid reading
  
  float voltage = (float)adcValue * 3.3f / 4095.0f;
  
  // Apply calibration offset if available
  if (encoderCalibrated) {
    voltage += encoderOffsets[idx];
    // Clamp voltage to valid range
    voltage = constrain(voltage, 0.0f, 3.3f);
  }
  
  return voltage;
}

/**
 * Convert analog voltage to angle in degrees with 0° baseline
 * @param voltage Voltage (0.0-3.3V)
 * @return Angle in degrees (-180 to +180, where 0° is calibrated baseline)
 */
float voltageToDeg(float voltage) {
  if (voltage < 0.0f) return -1.0f;  // Error condition
  // Convert voltage to 360° circle, then shift to -180° to +180° for easier interpretation
  float angle = (voltage / 3.3f) * 360.0f;
  // Shift so calibrated baseline becomes 0°
  if (angle > 180.0f) angle -= 360.0f;
  return angle;
}

/**
 * Full mecanum wheel speed calculation with rotation and diagonal support
 * RIGHT-SIDE MOTORS (indices 1=FR, 3=BR) have inverted directions
 * @param mx Strafe component (-1=left, +1=right)
 * @param my Forward component (-1=back, +1=forward)
 * @param mr Rotation component (-1=CCW, +1=CW)
 * @param speeds Output array for motor speeds (-255 to +255)
 */
void calculateMotorSpeeds(float mx, float my, float mr, int speeds[4]) {
  // Full mecanum equations with rotation
  // X configuration: FL,FR,BL,BR wheels
  float fl = my + mx + mr;  // Front Left
  float fr = my - mx - mr;  // Front Right
  float bl = my - mx + mr;  // Back Left
  float br = my + mx - mr;  // Back Right

  // Simple normalization to prevent exceeding limits
  float maxSpeed = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
  if (maxSpeed > 1.0f) {
    fl /= maxSpeed; fr /= maxSpeed; bl /= maxSpeed; br /= maxSpeed;
  }

  // Apply motor speed scaling with minimum threshold
  const int MIN_SPEED = 80;  // Minimum speed for reliable motor operation
  int effectiveSpeed = max(robotSpeed, MIN_SPEED);
  
  speeds[0] = (int)(fl * effectiveSpeed);
  speeds[1] = (int)(fr * effectiveSpeed);  // Front Right (INVERTED)
  speeds[2] = (int)(bl * effectiveSpeed);
  speeds[3] = (int)(br * effectiveSpeed);  // Back Right (INVERTED)

  // INVERT RIGHT-SIDE MOTORS (FR=1, BR=3) for proper mecanum operation
  speeds[1] = -speeds[1];  // Front Right
  speeds[3] = -speeds[3];  // Back Right
}

/**
 * Initialize PWM channels for motor control
 * Uses ESP32 LEDC peripheral for hardware PWM generation
 */
void setupPWM() {
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
  speed = constrain(speed, -255, 255);  // Ensure speed is within range
  
  if (speed > 0) {
    // Forward: IN1 = PWM, IN2 = 0
    ledcWrite(idx*2, speed);
    ledcWrite(idx*2+1, 0);
  } else if (speed < 0) {
    // Reverse: IN1 = 0, IN2 = PWM
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, -speed);
  } else {
    // Stop: Both pins low
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, 0);
  }
}

// ------------------------------
// Web Server Handlers - Simplified
// ------------------------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Mecanum Robot - Simple Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial, sans-serif; text-align:center; margin: 20px; }
.button { padding:15px 25px; margin:8px; font-size:18px; border:none; border-radius:8px; cursor:pointer; }
.forward { background:#4CAF50; color:white; }
.backward { background:#f44336; color:white; }
.left { background:#2196F3; color:white; }
.right { background:#FF9800; color:white; }
.stop { background:#757575; color:white; }
.slider-container { margin: 20px 0; }
.slider { width: 300px; }
.encoder-display { background:#f0f0f0; padding:15px; margin:20px; border-radius:8px; }
h1 { color: #333; }
.controls { margin: 30px 0; }
</style>
</head>
<body>
<h1>🤖 ESP32 Mecanum Robot</h1>
<div class="slider-container">
  <h3>Speed Control</h3>
  <input type="range" min="0" max="255" value="150" class="slider" id="spd" onchange="setSpeed(this.value)">
  <div>Current Speed: <span id="spv">150</span></div>
</div>
<div class="controls">
  <h3>Basic Movement</h3>
  <button class="button forward" onclick="move('forward')">↑ FORWARD</button><br>
  <button class="button left" onclick="move('left')">← LEFT</button>
  <button class="button stop" onclick="move('stop')">⏹ STOP</button>
  <button class="button right" onclick="move('right')">RIGHT →</button><br>
  <button class="button backward" onclick="move('backward')">↓ BACKWARD</button>
</div>
<div class="controls">
  <h3>Diagonal Movement</h3>
  <button class="button forward" onclick="move('forward_left')">↖ FWD+LEFT</button>
  <button class="button forward" onclick="move('forward_right')">↗ FWD+RIGHT</button><br>
  <button class="button backward" onclick="move('backward_left')">↙ BWD+LEFT</button>
  <button class="button backward" onclick="move('backward_right')">↘ BWD+RIGHT</button>
</div>
<div class="controls">
  <h3>Rotation</h3>
  <button class="button left" onclick="move('rotate_ccw')">⟲ ROTATE CCW</button>
  <button class="button right" onclick="move('rotate_cw')">⟳ ROTATE CW</button>
</div>
<div class="encoder-display">
  <h3>Motor Encoder Readings</h3>
  <pre id="encoders">Loading encoder data...</pre>
</div>
<script>
function setSpeed(v) {
  document.getElementById("spv").innerText = v;
  fetch('/speed?value=' + v);
}

function move(direction) {
  fetch('/move?dir=' + direction);
}

// Update encoder readings every second
setInterval(() => {
  fetch('/encoders')
    .then(response => response.text())
    .then(data => {
      document.getElementById('encoders').innerText = data;
    });
}, 1000);
</script>
</body>
</html>
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
  if (dir == "forward") {
    moveY = 1.0; moveX = 0.0; moveRot = 0.0;
  } else if (dir == "backward") {
    moveY = -1.0; moveX = 0.0; moveRot = 0.0;
  } else if (dir == "left") {
    moveX = -1.0; moveY = 0.0; moveRot = 0.0;
  } else if (dir == "right") {
    moveX = 1.0; moveY = 0.0; moveRot = 0.0;
  } else if (dir == "forward_left") {
    moveY = 0.707; moveX = -0.707; moveRot = 0.0;  // 45° diagonal
  } else if (dir == "forward_right") {
    moveY = 0.707; moveX = 0.707; moveRot = 0.0;   // 45° diagonal
  } else if (dir == "backward_left") {
    moveY = -0.707; moveX = -0.707; moveRot = 0.0; // 45° diagonal
  } else if (dir == "backward_right") {
    moveY = -0.707; moveX = 0.707; moveRot = 0.0;  // 45° diagonal
  } else if (dir == "rotate_cw") {
    moveY = 0.0; moveX = 0.0; moveRot = 1.0;       // Clockwise rotation
  } else if (dir == "rotate_ccw") {
    moveY = 0.0; moveX = 0.0; moveRot = -1.0;      // Counter-clockwise rotation
  } else if (dir == "stop") {
    moveX = 0.0; moveY = 0.0; moveRot = 0.0;
  }
  server.send(200, "text/plain", "OK");
}

void handleEncoders() {
  String response;
  if (!encoderCalibrated) {
    response = "Encoders calibrating... please wait.\n";
  } else {
    response = "Calibrated Encoder Readings (relative to baseline):\n";
    for (int i = 0; i < NUM_MOTORS; i++) {
      float voltage = readEncoderVoltage(i);
      if (voltage < 0) {
        response += "Motor " + String(char('A'+i)) + ": No Signal\n";
      } else {
        response += "Motor " + String(char('A'+i)) + ": " +
                    String(voltageToDeg(voltage), 1) + "°\n";
      }
    }
  }
  server.send(200, "text/plain", response);
}

// ------------------------------
// Setup & Loop - Simplified
// ------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 Mecanum Robot - Simplified Control ===");
  Serial.println("Initializing system...");

  // Configure pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorPins[i].IN1, OUTPUT);
    pinMode(motorPins[i].IN2, OUTPUT);
    pinMode(ENC_ANALOG_PINS[i], INPUT);
  }

  // Initialize PWM
  setupPWM();
  
  // Calibrate encoders at startup
  calibrateEncoders();

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // Setup OTA updates
  ArduinoOTA.setHostname("MecanumBot");
  ArduinoOTA.begin();
  Serial.println("OTA Ready - Use 'MecanumBot.local' in Arduino IDE");

  // Setup web server routes (simplified)
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/move", handleMove);
  server.on("/encoders", handleEncoders);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle OTA and web server requests
  ArduinoOTA.handle();
  server.handleClient();

  // Calculate and apply motor speeds (with rotation and diagonal support)
  int speeds[4];
  calculateMotorSpeeds(moveX, moveY, moveRot, speeds);
  for (int i = 0; i < NUM_MOTORS; i++) {
    driveMotor(i, speeds[i]);
  }

  // Status logging
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 3000) {
    Serial.printf("Speed: %d | X: %.2f | Y: %.2f | R: %.2f | Encoders: %s\n",
                  robotSpeed, moveX, moveY, moveRot, encoderCalibrated ? "Calibrated" : "Calibrating");
    lastStatusTime = millis();
  }

  delay(50);
}

