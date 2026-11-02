// End-effector servo controller for dental_tool_control.
// Receives "S <angle>\n" over USB serial, moves the servo, replies "OK <angle>\n".
// Anything else gets "ERR <reason>\n".

#include <Servo.h>

const int SERVO_PIN = 9;               // change to the pin your servo signal is on
const unsigned long SETTLE_MS = 800;   // time for the servo to reach position
const int START_ANGLE = 90;

Servo servo;
String line;

void setup() {
  Serial.begin(9600);                  // must match baud_rate in the ROS node
  servo.attach(SERVO_PIN);
  servo.write(START_ANGLE);
}

void handle(const String &cmd) {
  if (!cmd.startsWith("S ")) {
    Serial.println("ERR unknown_command");
    return;
  }
  String arg = cmd.substring(2);
  arg.trim();
  if (arg.length() == 0) {
    Serial.println("ERR missing_angle");
    return;
  }
  for (unsigned int i = 0; i < arg.length(); ++i) {
    if (!isDigit(arg[i])) {               // toInt() would turn garbage into 0 deg
      Serial.println("ERR bad_angle");
      return;
    }
  }
  int angle = arg.toInt();
  if (angle < 0 || angle > 180) {
    Serial.println("ERR angle_out_of_range");
    return;
  }
  servo.write(angle);
  delay(SETTLE_MS);
  Serial.print("OK ");
  Serial.println(angle);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handle(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}
