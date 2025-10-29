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
  // Mecanum wheel equations for X configuration
  float fl = my + mx + mr;  // Front Left
  float fr = my - mx - mr;  // Front Right
  float bl = my - mx + mr;  // Back Left
  float br = my + mx - mr;  // Back Right

  // Debug output for input values
  Serial.printf("Input: mx=%.2f, my=%.2f, mr=%.2f\n", mx, my, mr);

  // Simple normalization to prevent exceeding limits
  float maxSpeed = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
  if (maxSpeed > 1.0f) {
    fl /= maxSpeed; fr /= maxSpeed; bl /= maxSpeed; br /= maxSpeed;
  }

  // Apply motor speed scaling - ALWAYS use robotSpeed as base
  int baseSpeed = robotSpeed;
  
  // Calculate raw speeds
  speeds[0] = (int)(fl * baseSpeed);
  speeds[1] = (int)(fr * baseSpeed);
  speeds[2] = (int)(bl * baseSpeed);
  speeds[3] = (int)(br * baseSpeed);

  // INVERT RIGHT-SIDE MOTORS (FR=1, BR=3) for proper mecanum operation
  speeds[1] = -speeds[1];  // Front Right
  speeds[3] = -speeds[3];  // Back Right

  // Always show debug output for troubleshooting
  Serial.printf("Calculated: FL=%d FR=%d BL=%d BR=%d (baseSpeed=%d)\n",
                speeds[0], speeds[1], speeds[2], speeds[3], baseSpeed);
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
  
  // Diagnostic: Show which motor is being driven
  static const char* motorNames[] = {"FL", "FR", "BL", "BR"};
  Serial.printf("DRIVE MOTOR %s: Speed=%d (IN1=%d, IN2=%d)\n",
                motorNames[idx], speed, idx*2, idx*2+1);
  
  if (speed > 0) {
    // Forward: IN1 = PWM, IN2 = 0
    Serial.printf("  Forward: IN1=%d, IN2=%d\n", speed, 0);
    ledcWrite(idx*2, speed);
    ledcWrite(idx*2+1, 0);
  } else if (speed < 0) {
    // Reverse: IN1 = 0, IN2 = PWM
    Serial.printf("  Reverse: IN1=%d, IN2=%d\n", 0, -speed);
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, -speed);
  } else {
    // Stop: Both pins low
    Serial.printf("  Stop: IN1=%d, IN2=%d\n", 0, 0);
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
.rotation-slider { width: 300px; }
.encoder-display { background:#f0f0f0; padding:15px; margin:20px; border-radius:8px; }
h1 { color: #333; }
.controls { margin: 30px 0; }
.joystick-container { margin: 30px 0; }
.joystick-wrapper { display: flex; flex-direction: column; align-items: center; gap: 20px; }
.joystick-base {
  width: 280px;
  height: 280px;
  border-radius: 50%;
  background: radial-gradient(circle, #e0e0e0 30%, #b0b0b0 70%);
  border: 4px solid #666;
  position: relative;
  box-shadow: inset 0 0 20px rgba(0,0,0,0.3);
  margin: 20px auto;
}
.joystick-stick {
  width: 50px;
  height: 50px;
  border-radius: 50%;
  background: linear-gradient(145deg, #4a90e2, #357abd);
  border: 3px solid #2c5aa0;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  cursor: pointer;
  box-shadow: 0 6px 12px rgba(0,0,0,0.4);
  transition: transform 0.05s ease-out;
}
.joystick-labels {
  display: flex;
  justify-content: space-around;
  width: 100%;
  max-width: 300px;
  margin-top: 20px;
}
.axis-label {
  background: #f5f5f5;
  padding: 8px 12px;
  border-radius: 15px;
  font-size: 14px;
  border: 1px solid #ddd;
}
.rotation-container { margin: 30px 0; }
</style>
</head>
<body>
<h1> ESP32 Mecanum Robot</h1>
<div class="slider-container">
  <h3>Speed Control</h3>
  <input type="range" min="100" max="255" value="200" class="slider" id="spd" onchange="setSpeed(this.value)">
  <div>Current Speed: <span id="spv">200</span></div>
</div>

<div class="joystick-container">
  <h3>Virtual Joystick - Mix X, Y, and Rotation</h3>
  <div class="joystick-wrapper">
    <div class="joystick-base">
      <div class="joystick-stick" id="joystick"></div>
    </div>
    <div class="joystick-labels">
      <div class="axis-label">X: <span id="x-value">0.00</span></div>
      <div class="axis-label">Y: <span id="y-value">0.00</span></div>
      <div class="axis-label">Rotation: <span id="r-value">0.00</span></div>
    </div>
  </div>
</div>

<div class="rotation-container">
  <h3>Rotation Control</h3>
  <input type="range" min="-1" max="1" step="0.01" value="0" class="rotation-slider" id="rotation" oninput="updateRotation(this.value)">
  <div>Rotation: <span id="rotation-display">0.00</span></div>
</div>

<div class="controls">
  <h3>Manual Movement Buttons</h3>
  <button class="button forward" onclick="move('forward')">↑ Forward</button><br>
  <button class="button left" onclick="move('forward_left')">↖ Forward+Left</button>
  <button class="button stop" onclick="move('stop')">⏹ Stop</button>
  <button class="button right" onclick="move('forward_right')">↗ Forward+Right</button><br>
  <button class="button left" onclick="move('left')">← Left</button>
  <button class="button right" onclick="move('right')">→ Right</button><br>
  <button class="button backward" onclick="move('backward_left')">↙ Backward+Left</button>
  <button class="button stop" onclick="move('stop')">⏹ Stop</button>
  <button class="button backward" onclick="move('backward_right')">↘ Backward+Right</button><br>
  <button class="button backward" onclick="move('backward')">↓ Backward</button>
</div>

<div class="controls">
  <h3>Motor Testing</h3>
  <button class="button forward" onclick="testMotor('A')">Test Motor A (FL)</button>
  <button class="button forward" onclick="testMotor('B')">Test Motor B (FR)</button><br>
  <button class="button forward" onclick="testMotor('C')">Test Motor C (BL)</button>
  <button class="button forward" onclick="testMotor('D')">Test Motor D (BR)</button>
</div>

<div class="controls">
  <h3>Joystick Testing</h3>
  <button class="button forward" onclick="testJoystick()"> Test Joystick (Forward)</button>
</div>

<div class="encoder-display">
  <h3>Motor Encoder Readings</h3>
  <pre id="encoders">Loading encoder data...</pre>
</div>

<script>
let joystickData = { x: 0, y: 0 };
let isDragging = false;

// Speed control
function setSpeed(v) {
  document.getElementById("spv").innerText = v;
  fetch('/speed?value=' + v);
}

// Update rotation
function updateRotation(value) {
  document.getElementById("rotation-display").innerText = parseFloat(value).toFixed(2);
  document.getElementById("r-value").innerText = parseFloat(value).toFixed(2);
  sendJoystickData();
}

// Joystick initialization and event handling
function initJoystick() {
  const joystick = document.getElementById('joystick');
  const joystickWrapper = document.querySelector('.joystick-wrapper');
  
  const startDrag = (e) => {
    isDragging = true;
    handleJoystickMove(e);
  };
  
  const handleJoystickMove = (e) => {
    if (!isDragging) return;
    
    const rect = joystickWrapper.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    
    let clientX, clientY;
    if (e.type === 'touchmove') {
      clientX = e.touches[0].clientX;
      clientY = e.touches[0].clientY;
    } else {
      clientX = e.clientX;
      clientY = e.clientY;
    }
    
    const deltaX = clientX - centerX;
    const deltaY = clientY - centerY;
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    const maxDistance = rect.width / 2 - 20; // Larger margin for better control
    
    // Apply deadzone for better control near center
    let normalizedX, normalizedY;
    if (distance < 10) {
      // Deadzone - set to zero when very close to center
      normalizedX = 0;
      normalizedY = 0;
    } else {
      // Normalize to -1 to 1 range with improved calculation
      const clampedDistance = Math.min(distance, maxDistance);
      normalizedX = deltaX / maxDistance;
      normalizedY = -deltaY / maxDistance; // Invert Y axis (joystick up should be forward)
      
      // Apply smoothing curve for more natural control
      const smoothingFactor = 0.8;
      normalizedX = Math.sign(normalizedX) * Math.pow(Math.abs(normalizedX), smoothingFactor);
      normalizedY = Math.sign(normalizedY) * Math.pow(Math.abs(normalizedY), smoothingFactor);
    }
    
    // Clamp values to -1 to 1 range
    joystickData.x = Math.max(-1, Math.min(1, normalizedX));
    joystickData.y = Math.max(-1, Math.min(1, normalizedY));
    
    // Update visual stick position with improved calculation
    const stickRadius = maxDistance - 10;
    const stickX = joystickData.x * stickRadius;
    const stickY = joystickData.y * stickRadius;
    joystick.style.transform = `translate(${stickX}px, ${stickY}px)`;
    
    // Update display values
    document.getElementById("x-value").innerText = joystickData.x.toFixed(2);
    document.getElementById("y-value").innerText = joystickData.y.toFixed(2);
    
    // Send data to server (throttled for better performance)
    sendJoystickData();
  };
  
  const stopDrag = () => {
    if (!isDragging) return;
    isDragging = false;
    joystickData.x = 0;
    joystickData.y = 0;
    joystick.style.transform = 'translate(0px, 0px)';
    document.getElementById("x-value").innerText = '0.00';
    document.getElementById("y-value").innerText = '0.00';
    sendJoystickData();
  };
  
  // Mouse events
  joystickWrapper.addEventListener('mousedown', startDrag);
  document.addEventListener('mousemove', handleJoystickMove);
  document.addEventListener('mouseup', stopDrag);
  
  // Touch events
  joystickWrapper.addEventListener('touchstart', startDrag);
  document.addEventListener('touchmove', handleJoystickMove);
  document.addEventListener('touchend', stopDrag);
  document.addEventListener('touchcancel', stopDrag);
}

function sendJoystickData() {
  const rotation = parseFloat(document.getElementById('rotation').value);
  fetch('/joystick', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `x=${joystickData.x}&y=${joystickData.y}&rotation=${rotation}`
  });
}

// Manual movement function
function move(direction) {
  fetch('/move?dir=' + direction)
    .then(response => {
      console.log('Movement command sent:', direction);
    });
}

// Individual motor test function
function testMotor(motor) {
  fetch('/testMotor?motor=' + motor)
    .then(response => {
      console.log('Motor test started for:', motor);
    });
}

// Test joystick function
function testJoystick() {
  fetch('/testJoystick')
    .then(response => {
      console.log('Test joystick endpoint called');
    });
}

// Update encoder readings every second
setInterval(() => {
  fetch('/encoders')
    .then(response => response.text())
    .then(data => {
      document.getElementById('encoders').innerText = data;
    });
}, 1000);

// Initialize joystick when page loads
window.onload = initJoystick;
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
    Serial.println("FORWARD COMMAND");
  } else if (dir == "backward") {
    moveY = -1.0; moveX = 0.0; moveRot = 0.0;
    Serial.println("BACKWARD COMMAND");
  } else if (dir == "left") {
    moveX = -1.0; moveY = 0.0; moveRot = 0.0;
    Serial.println("LEFT COMMAND");
  } else if (dir == "right") {
    moveX = 1.0; moveY = 0.0; moveRot = 0.0;
    Serial.println("RIGHT COMMAND");
  } else if (dir == "forward_left") {
    moveY = 0.707; moveX = -0.707; moveRot = 0.0;  // 45° diagonal
    Serial.println("FORWARD+LEFT COMMAND");
  } else if (dir == "forward_right") {
    moveY = 0.707; moveX = 0.707; moveRot = 0.0;   // 45° diagonal
    Serial.println("FORWARD+RIGHT COMMAND");
  } else if (dir == "backward_left") {
    moveY = -0.707; moveX = -0.707; moveRot = 0.0; // 45° diagonal
    Serial.println("BACKWARD+LEFT COMMAND");
  } else if (dir == "backward_right") {
    moveY = -0.707; moveX = 0.707; moveRot = 0.0;  // 45° diagonal
    Serial.println("BACKWARD+RIGHT COMMAND");
  } else if (dir == "rotate_cw") {
    moveY = 0.0; moveX = 0.0; moveRot = 1.0;       // Clockwise rotation
    Serial.println("ROTATE CW COMMAND");
  } else if (dir == "rotate_ccw") {
    moveY = 0.0; moveX = 0.0; moveRot = -1.0;      // Counter-clockwise rotation
    Serial.println("ROTATE CCW COMMAND");
  } else if (dir == "stop") {
    moveX = 0.0; moveY = 0.0; moveRot = 0.0;
    Serial.println("STOP COMMAND");
  }
  server.send(200, "text/plain", "OK");
}

void handleTestMotor() {
  String motor = server.arg("motor");
  int testSpeed = 150;  // Test speed
  
  Serial.printf("Testing motor %s\n", motor.c_str());
  
  // Stop all motors first
  for (int i = 0; i < NUM_MOTORS; i++) {
    driveMotor(i, 0);
  }
  delay(100);
  
  if (motor == "A") {
    Serial.printf("Testing Motor A (FL): Setting speed to %d\n", testSpeed);
    driveMotor(0, testSpeed);
    Serial.println("Motor A drive command sent");
    delay(2000);
    Serial.println("Stopping Motor A");
    driveMotor(0, 0);
  } else if (motor == "B") {
    Serial.printf("Testing Motor B (FR): Setting speed to %d\n", testSpeed);
    driveMotor(1, testSpeed);
    Serial.println("Motor B drive command sent");
    delay(2000);
    Serial.println("Stopping Motor B");
    driveMotor(1, 0);
  } else if (motor == "C") {
    Serial.printf("Testing Motor C (BL): Setting speed to %d\n", testSpeed);
    driveMotor(2, testSpeed);
    Serial.println("Motor C drive command sent");
    delay(2000);
    Serial.println("Stopping Motor C");
    driveMotor(2, 0);
  } else if (motor == "D") {
    Serial.printf("Testing Motor D (BR): Setting speed to %d\n", testSpeed);
    driveMotor(3, testSpeed);
    Serial.println("Motor D drive command sent");
    delay(2000);
    Serial.println("Stopping Motor D");
    driveMotor(3, 0);
  }
  
  Serial.println("Motor test complete");
  server.send(200, "text/plain", "Motor test complete");
}

void handleJoystick() {
  Serial.println("=== JOYSTICK REQUEST RECEIVED ===");
  Serial.printf("Method: %s\n", server.method() == HTTP_POST ? "POST" : "GET");
  
  // Debug: Print all available arguments
  Serial.printf("Available arguments: %d\n", server.args());
  for (int i = 0; i < server.args(); i++) {
    Serial.printf("  %s = %s\n", server.argName(i).c_str(), server.arg(i).c_str());
  }
  
  // Handle analog joystick input for X, Y, and rotation
  if (server.hasArg("x")) {
    moveX = server.arg("x").toFloat();
    Serial.printf("Joystick X received: %.2f\n", moveX);
  }
  if (server.hasArg("y")) {
    moveY = server.arg("y").toFloat();
    Serial.printf("Joystick Y received: %.2f\n", moveY);
  }
  if (server.hasArg("rotation")) {
    moveRot = server.arg("rotation").toFloat();
    Serial.printf("Joystick Rotation received: %.2f\n", moveRot);
  }
  
  // If no arguments received, provide test values
  if (server.args() == 0) {
    Serial.println("No arguments received - setting test values");
    moveX = 1.0;  // Test forward movement
    moveY = 0.0;
    moveRot = 0.0;
  }
  
  // Clamp values to valid ranges
  moveX = constrain(moveX, -1.0f, 1.0f);
  moveY = constrain(moveY, -1.0f, 1.0f);
  moveRot = constrain(moveRot, -1.0f, 1.0f);
  
  Serial.printf("Processed values: X=%.2f, Y=%.2f, R=%.2f\n", moveX, moveY, moveRot);
  Serial.println("=== JOYSTICK REQUEST PROCESSED ===\n");
  
  server.send(200, "text/plain", "OK");
}

void handleTestJoystick() {
  Serial.println("=== TEST JOYSTICK ENDPOINT CALLED ===");
  // Test with hardcoded values
  moveX = 1.0;
  moveY = 0.0;
  moveRot = 0.0;
  Serial.printf("Test values set: X=%.2f, Y=%.2f, R=%.2f\n", moveX, moveY, moveRot);
  server.send(200, "text/plain", "Test joystick values applied");
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
  server.on("/testMotor", handleTestMotor);
  server.on("/joystick", handleJoystick);
  server.on("/testJoystick", handleTestJoystick);  // Added test endpoint
  server.on("/encoders", handleEncoders);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Handle OTA and web server requests
  ArduinoOTA.handle();
  server.handleClient();

  // Calculate and apply motor speeds (with rotation and diagonal support)
  static int lastSpeeds[4] = {0, 0, 0, 0};
  int speeds[4];
  calculateMotorSpeeds(moveX, moveY, moveRot, speeds);
  
  // Only print when speeds change significantly to avoid spam
  bool speedsChanged = false;
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (abs(speeds[i] - lastSpeeds[i]) > 5) {
      speedsChanged = true;
      break;
    }
  }
  
  // Apply motor speeds
  for (int i = 0; i < NUM_MOTORS; i++) {
    driveMotor(i, speeds[i]);
  }
  
  // Store current speeds for comparison
  for (int i = 0; i < NUM_MOTORS; i++) {
    lastSpeeds[i] = speeds[i];
  }

  // Enhanced status logging with motor commands
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 1000) {
    Serial.printf("\n=== STATUS UPDATE ===\n");
    Serial.printf("Speed: %d | X: %.2f | Y: %.2f | R: %.2f\n", robotSpeed, moveX, moveY, moveRot);
    Serial.printf("Motor Commands: FL=%d FR=%d BL=%d BR=%d\n", speeds[0], speeds[1], speeds[2], speeds[3]);
    Serial.printf("Moving: %s\n", (abs(moveX) > 0.01 || abs(moveY) > 0.01 || abs(moveRot) > 0.01) ? "YES" : "NO");
    Serial.printf("WiFi: %s | Encoders: %s\n",
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  encoderCalibrated ? "Calibrated" : "Calibrating");
    Serial.printf("====================\n\n");
    lastStatusTime = millis();
  }

  delay(10);
}

