// ------------------------------
// ESP32 Mecanum Robot Control + OTA Update
// ------------------------------
// Controls 4 DC motors with AS5600 PWM encoders.
// Web interface for movement control and live encoder readout.
// OTA (Over-The-Air) firmware updates via Arduino IDE.
// ------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include "wifi_config.h"

WebServer server(80);

// ------------------------------
// Motor & Encoder Config
// ------------------------------
#define NUM_MOTORS 4

const int ENC_PWM_PINS[NUM_MOTORS] = {32, 33, 34, 35};

struct MotorPins { int IN1, IN2; };
MotorPins motorPins[NUM_MOTORS] = {
  {14, 27}, // FL
  {5, 18},  // FR
  {13, 12}, // BL
  {26, 25}  // BR
};

int robotSpeed = 150;  // default speed (0–255)
float moveX = 0;       // strafe left/right (-1..1)
float moveY = 0;       // forward/back (-1..1)
float rotate = 0;      // rotate CCW/CW (-1..1)
const int PULSE_TIMEOUT = 50000; // 50ms timeout for encoders

// PWM setup constants
#define PWM_FREQ 20000
#define PWM_RES 8

// ------------------------------
// Helper Functions
// ------------------------------
float readEncoderDuty(int idx) {
  int pin = ENC_PWM_PINS[idx];
  unsigned long highTime = pulseIn(pin, HIGH, PULSE_TIMEOUT);
  if(highTime == 0) return -1;
  unsigned long lowTime = pulseIn(pin, LOW, PULSE_TIMEOUT);
  if(lowTime == 0) return -1;
  return (float)highTime / (highTime + lowTime);
}

float dutyToDeg(float duty) {
  return duty * 360.0f;
}

// --- proper Mecanum wheel mapping + min PWM compensation ---
void calculateMotorSpeeds(float mx, float my, float rot, int speeds[4]) {
  // mx = strafe (right +), my = forward (+), rot = clockwise (+)
  float fl =  my + mx + rot;
  float fr =  my - mx - rot;
  float bl =  my - mx + rot;
  float br =  my + mx - rot;

  // normalize to -1..1
  float maxMag = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
  if (maxMag < 1.0f) maxMag = 1.0f;
  fl /= maxMag; fr /= maxMag; bl /= maxMag; br /= maxMag;

  const int minPWM = 70;        // motors won't move below this
  for (int i = 0; i < 4; i++) {
    float v = (i==0)?fl:(i==1)?fr:(i==2)?bl:br;
    int pwm = (int)(abs(v) * (robotSpeed - minPWM) + (v!=0?minPWM:0));
    speeds[i] = (v >= 0) ? pwm : -pwm;
  }
}

void setupPWM() {
  // Setup PWM channels for motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledcSetup(i*2, PWM_FREQ, PWM_RES);
    ledcSetup(i*2+1, PWM_FREQ, PWM_RES);
    ledcAttachPin(motorPins[i].IN1, i*2);
    ledcAttachPin(motorPins[i].IN2, i*2+1);
  }
}

void driveMotor(int idx, int speed) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    ledcWrite(idx*2, pwm);
    ledcWrite(idx*2+1, 0);
  } else if (speed < 0) {
    ledcWrite(idx*2, 0);
    ledcWrite(idx*2+1, pwm);
  } else {
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
    float duty = readEncoderDuty(i);
    if (duty < 0) res += String("Motor ") + char('A'+i) + ": No Signal\n";
    else res += String("Motor ") + char('A'+i) + ": " + String(dutyToDeg(duty), 1) + "°\n";
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
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Booting Mecanum Robot ===");

  for (int i=0;i<NUM_MOTORS;i++){
    pinMode(motorPins[i].IN1, OUTPUT);
    pinMode(motorPins[i].IN2, OUTPUT);
    pinMode(ENC_PWM_PINS[i], INPUT);
  }

  setupPWM();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // OTA Setup
  ArduinoOTA.setHostname("MecanumBot");
  // ArduinoOTA.setPassword("admin"); // Remove password for testing
  ArduinoOTA.begin();
  Serial.println("OTA Ready. Use 'MecanumBot.local' in Arduino IDE.");

  // Web server routes
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/move", handleMove);
  server.on("/rotate", handleRotate);
  server.on("/maneuver", handleManeuver);
  server.on("/encoders", handleEncoders);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  int speeds[4];
  calculateMotorSpeeds(moveX, moveY, rotate, speeds);
  for (int i = 0; i < 4; i++) driveMotor(i, speeds[i]);

  static unsigned long t0 = 0;
  if (millis() - t0 > 2000) {
    Serial.printf("Speed=%d  X=%.2f  Y=%.2f  R=%.2f\n", robotSpeed, moveX, moveY, rotate);
    t0 = millis();
  }
  delay(50);
}

