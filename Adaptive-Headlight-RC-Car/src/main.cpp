#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <math.h>



BluetoothSerial ble;
MPU6050 mpu(Wire);

// Define buzzer pin
int buzzerPin = 4;

// Define Fog Light pin
int foglightPin = 16;

// Define Rain Sensor pin
int rainsensorPin = 34;

// Define DHT sensor pin and type
#define DHTPIN 13
#define DHTTYPE DHT11  // Change this line to use DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);

// Define Servo Pin
int servoPin = 32;
Servo servo;  // Create servo object to control a servo

// Define motor control pins
int ENB = 12;
int In4 = 14;
int In3 = 27;
int In2 = 26;
int In1 = 25;
int ENA = 33;

// Define IR sensor pins
int LS = 35;
int RS = 39;

// Motor speed (0 to 255)
int speed = 200; // Initial speed set to half speed

// Define PWM channels
int pwmChannelA = 3;
int pwmChannelB = 4;

// Bluetooth command variables
char bluetoothCommand = 'S'; // 'S' for stop initially

// Time variables for non-blocking timing
unsigned long previousMillis = 0;
unsigned long previousPrintMillis = 0;
const long interval = 50;  // Interval for main loop in milliseconds
const long printInterval = 5000;  // Interval for print statements in milliseconds

// Variables for smoothing gyro readings
const int numReadings = 10;
float gyroReadings[numReadings];
float smoothedGyroZ = 0;
int readingIndex = 0;

// Flag to enable/disable rotation detection
bool rotationDetectionEnabled = false;

void moveForward();
void moveBackward();
void turnSharpLeft();
void turnSharpRight();
void stopMotors();
void controlFogLight();
void outOfTrack();
void detectRotation();
float calculateDewPoint(float temperature, float humidity);

void setup() {
  Serial.begin(115200);
  ble.begin("Group_18");
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
  } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");

  // Initialize DHT sensor
  dht.begin();

  // Set the motor control pins as outputs
  pinMode(ENB, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Set the sensor pins as inputs
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(rainsensorPin, INPUT);

  // Configure PWM channels
  ledcSetup(pwmChannelA, 5000, 8); // Channel 0, 5kHz, 8-bit resolution
  ledcAttachPin(ENA, pwmChannelA);

  ledcSetup(pwmChannelB, 5000, 8); // Channel 1, 5kHz, 8-bit resolution
  ledcAttachPin(ENB, pwmChannelB);

  // Attach the servo on pin 32 to the servo object
  servo.attach(servoPin);

  pinMode(foglightPin, OUTPUT);

  // Initialize the gyroReadings array
  for (int i = 0; i < numReadings; i++) {
    gyroReadings[i] = 0;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (rotationDetectionEnabled) {
      detectRotation();
    }
    controlFogLight();
    outOfTrack();
  }

  // Process Bluetooth commands
  if (ble.available()) {
    bluetoothCommand = ble.read();
  }

  // Execute command based on Bluetooth input
  switch (bluetoothCommand) {
    case 'U':
      moveForward();
      break;
    case 'D':
      moveBackward();
      break;
    case 'R':
      turnSharpRight();
      break;
    case 'L':
      turnSharpLeft();
      break;
    case 'H':
      rotationDetectionEnabled = !rotationDetectionEnabled;
      if (rotationDetectionEnabled) {
        Serial.println("Rotation detection enabled");
      } else {
        Serial.println("Rotation detection disabled");
        // Optionally, move servo to a default position when disabling rotation detection
        servo.write(90); // Move to center position
      }
      break;
    default:
      stopMotors();
      break;
  }
}

void controlFogLight() {
  // Read DHT sensor for temperature and humidity
  float temperature = dht.readTemperature(); // Read temperature in Celsius
  float humidity = dht.readHumidity();       // Read humidity as percentage

  // Calculate dew point
  float dewPoint = calculateDewPoint(temperature, humidity);

  // Read rain sensor
  bool isRaining = digitalRead(rainsensorPin) == LOW;

  // Turn on fog light if rain is detected or if mist (high humidity) is detected
  if (isRaining || humidity >= 100 || dewPoint >= 15) {
    digitalWrite(foglightPin, HIGH); // Turn on fog lights
  } else {
    digitalWrite(foglightPin, LOW); // Turn off fog lights
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousPrintMillis >= printInterval) {
    previousPrintMillis = currentMillis;

    // Print fog light status, humidity, temperature, and dew point values
    if (digitalRead(foglightPin) == HIGH) {
      Serial.println("Fog light ON due to rain or mist detected.");
    } else {
      Serial.println("Fog light OFF.");
    }

    Serial.print("Current Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Current Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Dew Point: ");
    Serial.print(dewPoint);
    Serial.println(" °C");

    // Print additional status messages
    if (isRaining) {
      Serial.println("Rain detected.");
    }
    if (humidity >= 70) {
      Serial.println("High humidity detected.");
    }
    if (dewPoint >= 15) {
      Serial.println("Mist detected (high dew point).");
    }
  }
}

float calculateDewPoint(float temperature, float humidity) {
  float A = 17.625;
  float B = 243.04;
  float C = log(humidity / 100.0) + (A * temperature) / (B + temperature);
  float dewPoint = (B * C) / (A - C);
  return dewPoint;
}

void detectRotation() {
  mpu.update();
  float rotationalSpeed = mpu.getGyroZ();
  
  // Shift old readings in the array
  for (int i = numReadings - 1; i > 0; i--) {
    gyroReadings[i] = gyroReadings[i - 1];
  }
  
  // Add new reading to the array
  gyroReadings[0] = rotationalSpeed;
  
  // Calculate average of recent readings
  smoothedGyroZ = 0;
  for (int i = 0; i < numReadings; i++) {
    smoothedGyroZ += gyroReadings[i];
  }
  smoothedGyroZ /= numReadings;
  
  // Map and constrain the smoothed value
  float mappedValue = map(smoothedGyroZ, -150, 150, 0, 180);
  mappedValue = constrain(mappedValue, 45, 135);

  // Smooth the servo movement
  int currentAngle = servo.read();
  int targetAngle = mappedValue;
  int step = 2;

  if (abs(currentAngle - targetAngle) > 5) { // Apply deadband of 5 degrees
    if (currentAngle < targetAngle) {
      for (int pos = currentAngle; pos <= targetAngle; pos += step) {
        servo.write(pos);
        delay(0);  // Adjust the delay for smoother movement
      }
    } else {
      for (int pos = currentAngle; pos >= targetAngle; pos -= step) {
        servo.write(pos);
        delay(0);  // Adjust the delay for smoother movement
      }
    }
  }
}

void outOfTrack() {
  if (digitalRead(LS) == HIGH || digitalRead(RS) == HIGH) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }
}

void moveForward() {
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void moveBackward() {
  digitalWrite(In1, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void turnSharpRight() {
  digitalWrite(In1, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void turnSharpLeft() {
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed); // Set speed

  digitalWrite(In3, HIGH); // Swap HIGH and LOW signals
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed); // Set speed
}

void stopMotors() {
  digitalWrite(In1, LOW); // Swap HIGH and LOW signals
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, 0); // Stop motor

  digitalWrite(In3, LOW); // Swap HIGH and LOW signals
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, 0); // Stop motor
}
