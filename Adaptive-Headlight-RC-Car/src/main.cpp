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
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Define Servo Pin
int servoPin = 32;
Servo servo;

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
int speed = 200;

// Define PWM channels
int pwmChannelA = 3;
int pwmChannelB = 4;

// Bluetooth command variables
char bluetoothCommand = 'S';

// Time variables for non-blocking timing
unsigned long previousMillis = 0;
unsigned long previousPrintMillis = 0;
const long interval = 50;
const long printInterval = 5000;

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
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");

  dht.begin();

  pinMode(ENB, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(rainsensorPin, INPUT);

  ledcSetup(pwmChannelA, 5000, 8);
  ledcAttachPin(ENA, pwmChannelA);

  ledcSetup(pwmChannelB, 5000, 8);
  ledcAttachPin(ENB, pwmChannelB);

  servo.attach(servoPin);

  pinMode(foglightPin, OUTPUT);

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

  if (ble.available()) {
    bluetoothCommand = ble.read();
  }

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
        servo.write(90);
      }
      break;
    default:
      stopMotors();
      break;
  }
}

void controlFogLight() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float dewPoint = calculateDewPoint(temperature, humidity);
  bool isRaining = digitalRead(rainsensorPin) == LOW;

  if (isRaining || humidity >= 70 || dewPoint == temperature) {
    digitalWrite(foglightPin, HIGH);
  } else {
    digitalWrite(foglightPin, LOW);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousPrintMillis >= printInterval) {
    previousPrintMillis = currentMillis;

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

    if (isRaining) {
      Serial.println("Rain detected.");
    }
    if (humidity >= 70) {
      Serial.println("High humidity detected.");
    }
    if (dewPoint >= 15) {
      Serial.println("Mist detected (high dew point).");
    }

    // Send data to Bluetooth terminal app
    String data = "Humidity: " + String(humidity) + " %, " + "Temperature: " + String(temperature) + " °C, " + "Dew Point: " + String(dewPoint) + " °C";
    ble.println(data);
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
  
  for (int i = numReadings - 1; i > 0; i--) {
    gyroReadings[i] = gyroReadings[i - 1];
  }
  
  gyroReadings[0] = rotationalSpeed;
  
  smoothedGyroZ = 0;
  for (int i = 0; i < numReadings; i++) {
    smoothedGyroZ += gyroReadings[i];
  }
  smoothedGyroZ /= numReadings;
  
  float mappedValue = map(smoothedGyroZ, -150, 150, 0, 180);
  mappedValue = constrain(mappedValue, 45, 135);

  int currentAngle = servo.read();
  int targetAngle = mappedValue;
  int step = 2;

  if (abs(currentAngle - targetAngle) > 5) {
    if (currentAngle < targetAngle) {
      for (int pos = currentAngle; pos <= targetAngle; pos += step) {
        servo.write(pos);
        delay(0);
      }
    } else {
      for (int pos = currentAngle; pos >= targetAngle; pos -= step) {
        servo.write(pos);
        delay(0);
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
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed);

  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed);
}

void moveBackward() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed);

  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed);
}

void turnSharpRight() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, speed);

  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  ledcWrite(pwmChannelB, speed);
}

void turnSharpLeft() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  ledcWrite(pwmChannelA, speed);

  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, speed);
}

void stopMotors() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  ledcWrite(pwmChannelA, 0);

  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  ledcWrite(pwmChannelB, 0);
}
