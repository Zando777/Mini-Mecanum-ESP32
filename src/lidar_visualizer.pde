import processing.serial.*;

Serial myPort;
String val;
ArrayList<PVector> points = new ArrayList<PVector>();
boolean collectingScan = false;

float scale = 0.5;  // 🆕 Initial zoom level (smaller = see farther)

void setup() {
  size(800, 800);
  background(0);
  pixelDensity(1);
  printArray(Serial.list());

  String portName = Serial.list()[9];  // Adjust index
  myPort = new Serial(this, portName, 115200);
}

void draw() {
  background(0);

  // 🆕 Draw distance rings (for scale reference)
  stroke(60);
  noFill();
  for (int i = 1; i <= 5; i++) {
    float radius = i * 1000 * scale;  // every 1m if distance in mm
    ellipse(width/2, height/2, radius*2, radius*2);
  }

  // Draw center point
  stroke(255);
  fill(255);
  ellipse(width/2, height/2, 10, 10);

  // Safely copy points to avoid concurrent modification
  ArrayList<PVector> safeCopy;
  synchronized (points) {
    safeCopy = new ArrayList<PVector>(points);
  }

  // Draw scan points
  stroke(0, 255, 0);
  fill(0, 255, 0);
  for (PVector p : safeCopy) {
    ellipse(p.x, p.y, 3, 3);
  }

  // Connect points for visualization
  stroke(0, 255, 0, 100);
  noFill();
  beginShape();
  for (PVector p : safeCopy) {
    vertex(p.x, p.y);
  }
  endShape(CLOSE);

  // 🆕 Draw scale info
  fill(255);
  textSize(16);
  text("Scale: " + nf(scale, 1, 3) + " px/mm  (+/- to zoom)", 10, height - 10);
}

void serialEvent(Serial myPort) {
  val = myPort.readStringUntil('\n');
  if (val != null) {
    val = trim(val);

    if (val.equals("SCAN_START")) {
      collectingScan = true;
      synchronized (points) {
        points.clear();
      }
    } else if (val.equals("SCAN_END")) {
      collectingScan = false;
    } else if (collectingScan) {
      String[] parts = split(val, ',');
      if (parts.length == 2) {
        float angle = float(parts[0]);
        float distance = float(parts[1]);

        float x = width/2 + cos(radians(angle)) * distance * scale;
        float y = height/2 + sin(radians(angle)) * distance * scale;

        synchronized (points) {
          points.add(new PVector(x, y));
        }
      }
    }
  }
}

// 🆕 Keyboard control for zoom
void keyPressed() {
  if (key == '+') scale *= 1.1;  // Zoom in
  if (key == '-') scale *= 0.9;  // Zoom out
  scale = constrain(scale, 0.01, 5.0); // limit range to avoid overflow
  println("Scale: " + scale);
}
