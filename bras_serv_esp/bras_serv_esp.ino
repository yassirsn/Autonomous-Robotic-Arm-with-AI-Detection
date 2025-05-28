#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "Galaxy A519ED";       // Change to your WiFi name
const char* password = "ojxd2691"; // Change to your WiFi password

// TCP server settings
WiFiServer tcpServer(8888);
WiFiClient client;

// Create the servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

// Define servo channels
#define SERVO1 0  // Base servo on channel 0
#define SERVO2 1  // Shoulder servo on channel 1
#define SERVO3 2  // Elbow servo on channel 2
#define SERVO4 3  // Gripper/Wrist servo on channel 3

// Define servo parameters
#define SERVO_FREQ 50      // Analog servos run at ~50 Hz
#define SERVOMIN  125      // Minimum pulse length count (0 degrees)
#define SERVOMAX  575      // Maximum pulse length count (180 degrees)

// Define servo movement speed - higher values mean slower movement
int movementDelay = 50;  // Default delay between servo position updates in milliseconds

// Define I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Current servo positions
int servo1Pos = 90;  // Base - range 0 to 180
int servo2Pos = 0;  // Shoulder - range 0 to 100
int servo3Pos = 00;  // Elbow - range 0 to 60
int servo4Pos = 30;  // Gripper - range 0 to 60

// Servo range limits
const int SERVO1_MIN = 0;
const int SERVO1_MAX = 180;
const int SERVO2_MIN = 0;
const int SERVO2_MAX = 100;
const int SERVO3_MIN = 0;
const int SERVO3_MAX = 60;
const int SERVO4_MIN = 0;
const int SERVO4_MAX = 60;

// Function to convert angle (0-180) to pulse width
int angleToPulse(int angle) {
  // Map angle of 0 to 180 to Servo min and max
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to smoothly move a servo from current position to target position
void moveServoSmooth(uint8_t servoNum, int currentPos, int targetPos) {
  int increment = (currentPos < targetPos) ? 1 : -1;
  
  for (int pos = currentPos; pos != targetPos + increment; pos += increment) {
    pwm.setPWM(servoNum, 0, angleToPulse(pos));
    delay(movementDelay); // Controls speed of movement
  }
}

// Function to convert pixel coordinates to arm angles
// Note: This is a simplified conversion - you'll need to calibrate this for your setup
void pixelToArmCoordinates(int pixelX, int pixelY, int &baseAngle, int &shoulderAngle, int &elbowAngle) {
  // Map pixel coordinates to servo angles
  // This is just an example - you'll need to determine the proper mapping for your arm and camera setup
  
  // Assuming a 640x480 pixel camera frame
  // Map X (0-640) to base servo angle (0-180)
  baseAngle = map(pixelX, 0, 640, SERVO1_MAX, SERVO1_MIN);
  baseAngle = constrain(baseAngle, SERVO1_MIN, SERVO1_MAX);
  
  // Map Y (0-480) to shoulder and elbow servos
  // Lower Y means higher in the frame (further away objects)
  // Higher Y means lower in the frame (closer objects)
  
  if (pixelY < 160) {
    // Far object - arm extended
    shoulderAngle = 80;
    elbowAngle = 50;
  } else if (pixelY < 320) {
    // Medium distance
    shoulderAngle = 50;
    elbowAngle = 30;
  } else {
    // Close object - arm down
    shoulderAngle = 20;
    elbowAngle = 10;
  }
  
  // Constrain to valid ranges
  shoulderAngle = constrain(shoulderAngle, SERVO2_MIN, SERVO2_MAX);
  elbowAngle = constrain(elbowAngle, SERVO3_MIN, SERVO3_MAX);
}

// Function to move arm to a specific point in camera coordinates
void moveToCoordinates(int x, int y) {
  int baseAngle, shoulderAngle, elbowAngle;
  
  // Convert pixel coordinates to arm angles
  pixelToArmCoordinates(x, y, baseAngle, shoulderAngle, elbowAngle);
  
  Serial.print("Moving to pixel (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(") => Servo angles: Base=");
  Serial.print(baseAngle);
  Serial.print(", Shoulder=");
  Serial.print(shoulderAngle);
  Serial.print(", Elbow=");
  Serial.println(elbowAngle);
  
  // Store old positions
  int oldBase = servo1Pos;
  int oldShoulder = servo2Pos;
  int oldElbow = servo3Pos;
  
  // Update current positions
  servo1Pos = baseAngle;
  servo2Pos = shoulderAngle;
  servo3Pos = elbowAngle;
  
  // Move servos in sequence
  moveServoSmooth(SERVO2, oldShoulder, servo2Pos); // Shoulder first
  moveServoSmooth(SERVO3, oldElbow, servo3Pos);    // Then elbow
  moveServoSmooth(SERVO1, oldBase, servo1Pos);     // Base last
}

// Function to grab an object
void grabObject() {
  // Open gripper
  int oldGripper = servo4Pos;
  servo4Pos = SERVO4_MIN; // Fully open
  moveServoSmooth(SERVO4, oldGripper, servo4Pos);
  
  delay(500);
  
  // Close gripper
  oldGripper = servo4Pos;
  servo4Pos = SERVO4_MAX; // Fully closed
  moveServoSmooth(SERVO4, oldGripper, servo4Pos);
}

// Function to set all servos to home position
void homePosition() {
  int oldBase = servo1Pos;
  int oldShoulder = servo2Pos;
  int oldElbow = servo3Pos;
  int oldGripper = servo4Pos;
  
  servo1Pos = 90;  // Base center
  servo2Pos = 50;  // Shoulder default
  servo3Pos = 30;  // Elbow default
  servo4Pos = 30;  // Gripper default
  
  // Move servos in sequence
  moveServoSmooth(SERVO2, oldShoulder, servo2Pos);
  moveServoSmooth(SERVO3, oldElbow, servo3Pos);
  moveServoSmooth(SERVO4, oldGripper, servo4Pos);
  moveServoSmooth(SERVO1, oldBase, servo1Pos);
}

// Handle JSON commands from Flask server
void handleJsonCommand(String jsonStr) {
  // Create a buffer for the JSON document
  StaticJsonDocument<256> doc;
  
  // Parse JSON
  DeserializationError error = deserializeJson(doc, jsonStr);
  
  // Test if parsing succeeds
  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
    client.println("{\"status\":\"error\", \"message\":\"Invalid JSON\"}");
    return;
  }
  
  // Extract command
  const char* command = doc["command"];
  if (!command) {
    client.println("{\"status\":\"error\", \"message\":\"No command specified\"}");
    return;
  }
  
  Serial.print("Received command: ");
  Serial.println(command);
  
  // Process different commands
  String cmdStr = String(command);
  
  if (cmdStr == "move_to") {
    // Get coordinates
    if (doc.containsKey("x") && doc.containsKey("y")) {
      int x = doc["x"];
      int y = doc["y"];
      moveToCoordinates(x, y);
      client.println("{\"status\":\"success\", \"message\":\"Moved to coordinates\"}");
    } else {
      client.println("{\"status\":\"error\", \"message\":\"Missing x,y coordinates\"}");
    }
  }
  else if (cmdStr == "grab") {
    // Move to position and grab
    if (doc.containsKey("x") && doc.containsKey("y")) {
      int x = doc["x"];
      int y = doc["y"];
      moveToCoordinates(x, y);
      delay(500);
      grabObject();
      client.println("{\"status\":\"success\", \"message\":\"Object grabbed\"}");
    } else {
      client.println("{\"status\":\"error\", \"message\":\"Missing x,y coordinates\"}");
    }
  }
  else if (cmdStr == "home") {
    homePosition();
    client.println("{\"status\":\"success\", \"message\":\"Moved to home position\"}");
  }
  else if (cmdStr == "up") {
    // Move shoulder up
    int oldPos = servo2Pos;
    servo2Pos = min(servo2Pos + 10, SERVO2_MAX);
    moveServoSmooth(SERVO2, oldPos, servo2Pos);
    client.println("{\"status\":\"success\", \"message\":\"Moved up\"}");
  }
  else if (cmdStr == "down") {
    // Move shoulder down
    int oldPos = servo2Pos;
    servo2Pos = max(servo2Pos - 10, SERVO2_MIN);
    moveServoSmooth(SERVO2, oldPos, servo2Pos);
    client.println("{\"status\":\"success\", \"message\":\"Moved down\"}");
  }
  else if (cmdStr == "left") {
    // Move base left
    int oldPos = servo1Pos;
    servo1Pos = min(servo1Pos + 10, SERVO1_MAX);
    moveServoSmooth(SERVO1, oldPos, servo1Pos);
    client.println("{\"status\":\"success\", \"message\":\"Moved left\"}");
  }
  else if (cmdStr == "right") {
    // Move base right
    int oldPos = servo1Pos;
    servo1Pos = max(servo1Pos - 10, SERVO1_MIN);
    moveServoSmooth(SERVO1, oldPos, servo1Pos);
    client.println("{\"status\":\"success\", \"message\":\"Moved right\"}");
  }
  else if (cmdStr == "center") {
    // Center the base
    int oldPos = servo1Pos;
    servo1Pos = 90;
    moveServoSmooth(SERVO1, oldPos, servo1Pos);
    client.println("{\"status\":\"success\", \"message\":\"Centered\"}");
  }
  else if (cmdStr == "gripper") {
    // Control gripper
    if (doc.containsKey("action")) {
      const char* action = doc["action"];
      int oldPos = servo4Pos;
      
      if (String(action) == "open") {
        servo4Pos = SERVO4_MIN;
      } else if (String(action) == "close") {
        servo4Pos = SERVO4_MAX;
      }
      
      moveServoSmooth(SERVO4, oldPos, servo4Pos);
      client.println("{\"status\":\"success\", \"message\":\"Gripper action completed\"}");
    } else {
      client.println("{\"status\":\"error\", \"message\":\"Missing gripper action\"}");
    }
  }
  else {
    client.println("{\"status\":\"error\", \"message\":\"Unknown command\"}");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Robot Arm Controller - Flask Integration");
  
  // Initialize I2C with explicit SDA and SCL pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // Set to 100kHz for better reliability
  
  // Check if PCA9685 is responding
  Wire.beginTransmission(0x40);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("PCA9685 found at address 0x40");
  } else {
    Serial.print("Error accessing PCA9685: ");
    Serial.println(error);
    
    // Try scanning I2C bus to find devices
    Serial.println("Scanning I2C bus...");
    for (byte address = 8; address < 127; address++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.println(" !");
      }
    }
    
    Serial.println("Please check your connections and reset the ESP32");
    while(1) delay(100); // Stop execution here
  }
  
  // Initialize the PCA9685
  if (!pwm.begin()) {
    Serial.println("Failed to initialize PCA9685. Check your wiring!");
    while(1);
  }
  
  pwm.setPWMFreq(SERVO_FREQ);  // Set frequency for servos
  
  delay(10);  // Allow time for PCA9685 to initialize
  
  // Set initial servo positions (home position)
  pwm.setPWM(SERVO1, 0, angleToPulse(servo1Pos));
  pwm.setPWM(SERVO2, 0, angleToPulse(servo2Pos));
  pwm.setPWM(SERVO3, 0, angleToPulse(servo3Pos));
  pwm.setPWM(SERVO4, 0, angleToPulse(servo4Pos));
  
  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  
  Serial.println();
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());
  
  // Start TCP server
  tcpServer.begin();
  Serial.println("TCP server started on port 8888");
  Serial.println("Waiting for commands from Flask server...");
}

void loop() {
  // Check if a client has connected
  if (!client || !client.connected()) {
    client = tcpServer.available();
    
    if (client) {
      Serial.println("New client connected");
      client.setTimeout(5000); // 5 second timeout
    }
  }
  
  // If client is connected, check for incoming data
  if (client && client.connected()) {
    if (client.available()) {
      String jsonCommand = client.readStringUntil('\n');
      Serial.print("Received data: ");
      Serial.println(jsonCommand);
      
      // Process the JSON command
      handleJsonCommand(jsonCommand);
    }
  }
  
  delay(10);
}