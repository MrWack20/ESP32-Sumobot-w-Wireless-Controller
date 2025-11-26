#include <esp_now.h>
#include <WiFi.h>

// ===== MOTOR PINS =====
#define ENA 14  // Left Motor speed (PWM) pin
#define ENB 32  // Right Motor speed (PWM) pin
#define IN1 27  // Left Motor direction pin 1
#define IN2 26  // Left Motor direction pin 2
#define IN3 25  // Right Motor direction pin 1
#define IN4 33  // Right Motor direction pin 2

// ===== SENSOR PINS =====
#define LEFT_IR 23
#define RIGHT_IR 22
#define LEFT_TRIG 21
#define LEFT_ECHO 19
#define RIGHT_TRIG 18
#define RIGHT_ECHO 5

// ===== AUTONOMOUS MODE SETTINGS =====
#define ENEMY_RANGE 50        // cm - enemy detection distance
#define MAX_DISTANCE 200      // cm - maximum ultrasonic range

// ===== SPEED SETTINGS (Adjustable) =====
#define ROAMING_SPEED 120     // Speed when searching (0-255) - Adjust this for slower/faster roaming
#define ATTACK_SPEED 255      // Speed when enemy detected (0-255) - Full power attack
#define TURN_SPEED 200        // Speed when turning (0-255)
#define PIVOT_SPEED 180       // Speed when pivoting away from edge (0-255)

unsigned long pushStartTime = 0;
unsigned long pushDuration = 1500;  // ms - how long to push when enemy detected
bool pushing = false;

// ===== IR SENSITIVITY SETTINGS =====
// If your IR sensors are too sensitive or not sensitive enough:
// Option 1: Adjust this delay to allow sensor readings to stabilize
#define IR_READ_DELAY 5       // ms - increase if getting false readings

// Option 2: Use multiple readings for more reliable detection
#define IR_SAMPLE_COUNT 3     // Number of samples to average (increase for less sensitivity)

// ===== CONNECTION TRACKING =====
unsigned long lastReceiveTime = 0;
const unsigned long CONNECTION_TIMEOUT = 1000;  // 1 second timeout
bool controllerConnected = false;

typedef struct struct_message {
  int16_t x;
  int16_t y;
} struct_message;

struct_message incomingData;

// ===== ESP-NOW RECEIVE CALLBACK =====
void onReceive(const esp_now_recv_info *recvInfo, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  lastReceiveTime = millis();
  
  if (!controllerConnected) {
    controllerConnected = true;
    Serial.println("\n*** CONTROLLER CONNECTED - Manual control active ***\n");
    stopMotors();  // Stop autonomous actions
    pushing = false;  // Reset autonomous state
  }
  
  Serial.printf("Joystick X: %d Y: %d\n", incomingData.x, incomingData.y);
  driveMotors(incomingData.x, incomingData.y);
}

// ===== MOTOR CONTROL FUNCTIONS (Manual Mode) =====
void driveMotors(int16_t x, int16_t y) {
  int deadzone = 200;
  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;

  int16_t leftMotor = y + x;
  int16_t rightMotor = y - x;

  leftMotor = constrain(leftMotor, -1000, 1000);
  rightMotor = constrain(rightMotor, -1000, 1000);

  int leftSpeed = (leftMotor * 255) / 1000;
  int rightSpeed = (rightMotor * 255) / 1000;

  setMotor(ENA, IN1, IN2, leftSpeed, false);
  setMotor(ENB, IN3, IN4, rightSpeed, true);
}

void setMotor(int pwmPin, int dirPin1, int dirPin2, int speed, bool invert) {
  if (invert) {
    speed = -speed;
  }

  int minSpeed = 50;
  
  if (speed > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    int pwmValue = (speed < minSpeed && speed > 0) ? minSpeed : abs(speed);
    ledcWrite(pwmPin, pwmValue);
  } else if (speed < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    int pwmValue = (abs(speed) < minSpeed && speed < 0) ? minSpeed : abs(speed);
    ledcWrite(pwmPin, pwmValue);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

// ===== AUTONOMOUS MOTOR FUNCTIONS =====
void stopMotors() {
  Serial.println("Action: STOP");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void moveForward() {
  // ROAMING SPEED - searching for enemy
  Serial.println("Action: FORWARD (Roaming)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, ROAMING_SPEED);  // Use roaming speed
  ledcWrite(ENB, ROAMING_SPEED);
}

void moveForwardFast() {
  // ATTACK SPEED - enemy detected!
  Serial.println("Action: FORWARD (ATTACKING!)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, ATTACK_SPEED);   // Use attack speed
  ledcWrite(ENB, ATTACK_SPEED);
}

void moveBackward() {
  Serial.println("Action: BACKWARD");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
}

void turnLeft() {
  Serial.println("Action: TURN LEFT (in place)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, TURN_SPEED);
  ledcWrite(ENB, TURN_SPEED);
}

void turnRight() {
  Serial.println("Action: TURN RIGHT (in place)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, TURN_SPEED);
  ledcWrite(ENB, TURN_SPEED);
}

void pivotLeft() {
  Serial.println("Action: PIVOT LEFT");
  // Right motor forward, left motor stop
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, PIVOT_SPEED);
}

void pivotRight() {
  Serial.println("Action: PIVOT RIGHT");
  // Left motor forward, right motor stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, PIVOT_SPEED);
  ledcWrite(ENB, 0);
}

void pushEnemy() {
  Serial.println("Action: PUSH (FULL POWER!)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, ATTACK_SPEED);  // Full attack speed
  ledcWrite(ENB, ATTACK_SPEED);
}

// ===== SENSOR READING FUNCTIONS =====
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return MAX_DISTANCE;  // No echo = max distance
  
  long distance = duration * 0.034 / 2;  // Convert to cm
  return distance;
}

bool detectBlackEdge(int pin) {
  // ===== IR SENSITIVITY ADJUSTMENT =====
  // This function reads IR sensors to detect black edges
  // LOW = Black surface detected (edge!)
  // HIGH = White surface (safe area)
  //
  // TO INCREASE SENSITIVITY: Decrease IR_SAMPLE_COUNT (try 1 or 2)
  // TO DECREASE SENSITIVITY: Increase IR_SAMPLE_COUNT (try 5 or 7)
  //
  // Alternatively, if you have adjustable IR sensors with a potentiometer,
  // turn the pot to adjust the trigger threshold directly on the sensor.
  
  int blackCount = 1;
  
  // Take multiple samples to reduce false positives
  for (int i = 0; i < IR_SAMPLE_COUNT; i++) {
    if (digitalRead(pin) == LOW) {
      blackCount++;
    }
    if (IR_READ_DELAY > 0) {
      delay(IR_READ_DELAY);
    }
  }
  
  // Return true if majority of samples detected black
  return blackCount > (IR_SAMPLE_COUNT / 2);
}

long getClosestDistance() {
  long leftDist = readUltrasonic(LEFT_TRIG, LEFT_ECHO);
  long rightDist = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
  
  // Return the closest reading
  return min(leftDist, rightDist);
}

// ===== AUTONOMOUS SUMOBOT MODE =====
void autonomousMode() {
  long distance = getClosestDistance();
  bool leftBlackDetected = detectBlackEdge(LEFT_IR);
  bool rightBlackDetected = detectBlackEdge(RIGHT_IR);
  
  // Debug sensor readings
  Serial.printf("AUTO | Dist:%ld cm | L_IR:%s | R_IR:%s | Speed:%s\n", 
                distance, 
                leftBlackDetected ? "BLACK!" : "White",
                rightBlackDetected ? "BLACK!" : "White",
                (distance > 0 && distance < ENEMY_RANGE) ? "ATTACK" : "ROAM");
  
  // --- ENEMY DETECTION (non-blocking push with speed boost) ---
  if (!pushing && distance > 0 && distance < ENEMY_RANGE) {
    pushing = true;
    pushStartTime = millis();
    pushEnemy();  // Uses ATTACK_SPEED
  }

  // Check if push duration completed
  if (pushing) {
    if (millis() - pushStartTime >= pushDuration) {
      stopMotors();
      pushing = false;
    }
  }

  // --- EDGE DETECTION (HIGHEST PRIORITY) ---
  if (leftBlackDetected && rightBlackDetected) {
    // BOTH EDGES DETECTED - Critical! Back up and turn around
    Serial.println("⚠️ BOTH EDGES! Backing up and turning around");
    stopMotors();
    delay(50);
    moveBackward();
    delay(400);  // Longer backup
    stopMotors();
    delay(50);
    turnRight();  // Turn 180 degrees
    delay(500);
    stopMotors();
    pushing = false;
  }
  else if (leftBlackDetected) {
    // LEFT EDGE DETECTED - Turn right to stay in ring
    Serial.println("⚠️ LEFT EDGE! Turning right");
    stopMotors();
    delay(50);
    // Quick pivot right (keeps robot compact and in ring)
    pivotRight();
    delay(200);  // Short turn to redirect
    stopMotors();
    pushing = false;
  }
  else if (rightBlackDetected) {
    // RIGHT EDGE DETECTED - Turn left to stay in ring
    Serial.println("⚠️ RIGHT EDGE! Turning left");
    stopMotors();
    delay(50);
    // Quick pivot left (keeps robot compact and in ring)
    pivotLeft();
    delay(200);  // Short turn to redirect
    stopMotors();
    pushing = false;
  }
  else if (!pushing) {
    // NO EDGE DETECTED - Check for enemy
    if (distance > 0 && distance < ENEMY_RANGE) {
      // Enemy nearby - move forward at ATTACK SPEED
      moveForwardFast();
    } else {
      // No enemy - move forward at ROAMING SPEED
      moveForward();
    }
  }
  
  delay(50);  // Sensor stability delay
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  // Sensor pins
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  // Initialize motors to stop
  stopMotors();

  // WiFi and ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1);
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("\n========================================");
  Serial.println("    SMART EDGE-AVOIDING SUMOBOT");
  Serial.println("========================================");
  Serial.println("Mode 1: Manual control via ESP-NOW");
  Serial.println("Mode 2: Autonomous sumobot (fallback)");
  Serial.println("========================================");
  Serial.println("Speed Settings:");
  Serial.printf("  Roaming: %d/255\n", ROAMING_SPEED);
  Serial.printf("  Attack:  %d/255\n", ATTACK_SPEED);
  Serial.printf("  Enemy detection range: %d cm\n", ENEMY_RANGE);
  Serial.println("========================================");
  Serial.println("IR Logic: WHITE=Safe | BLACK=Edge");
  Serial.println("Edge Response: INSTANT turn to stay in");
  Serial.println("========================================");
  Serial.println("Waiting for controller...");
  Serial.println("Will auto-start in 3 seconds\n");
  
  // Initial delay to allow controller connection
  delay(3000);
}

// ===== MAIN LOOP =====
void loop() {
  // Check if controller is still connected
  if (millis() - lastReceiveTime > CONNECTION_TIMEOUT) {
    if (controllerConnected) {
      controllerConnected = false;
      Serial.println("\n*** CONTROLLER DISCONNECTED - Autonomous mode active ***\n");
    }
    
    // Run autonomous sumobot mode
    autonomousMode();
  } else {
    // Controller connected - ESP-NOW callback handles everything
    delay(10);
  }
}