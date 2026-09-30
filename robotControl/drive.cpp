#include "BluetoothSerial.h"

// 블루투스 객체 생성
BluetoothSerial SerialBT;

// ==========================================
// 핀 설정 및 변수
// ==========================================
// M0: 왼쪽 앞
const int M0_IN1 = 27;
const int M0_IN2 = 13;

// M1: 오른쪽 앞
const int M1_IN1 = 4;
const int M1_IN2 = 2;

// M2: 왼쪽 뒤
const int M2_IN1 = 17;
const int M2_IN2 = 12;

// M3: 오른쪽 뒤
const int M3_IN1 = 15;
const int M3_IN2 = 14;

const int motorSpeed = 200;


// ==========================================
// 모터 제어 기본 함수
// ==========================================
void motor(int in1, int in2, int speed) {
  if (speed > 0) {
    analogWrite(in1, speed);
    digitalWrite(in2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    analogWrite(in2, -speed);
  }
  else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

// ==========================================
// 동작 함수
// ==========================================
void moveForward() {
  // 왼쪽
  motor(M0_IN1, M0_IN2, motorSpeed);
  motor(M2_IN1, M2_IN2, motorSpeed);
  // 오른쪽
  motor(M1_IN1, M1_IN2, -motorSpeed);
  motor(M3_IN1, M3_IN2, -motorSpeed);
}

void moveBackward() {
  motor(M0_IN1, M0_IN2, -motorSpeed);
  motor(M2_IN1, M2_IN2, -motorSpeed);

  motor(M1_IN1, M1_IN2, motorSpeed);
  motor(M3_IN1, M3_IN2, motorSpeed);
}

void moveLeft() {
  motor(M0_IN1, M0_IN2, -motorSpeed);
  motor(M2_IN1, M2_IN2, -motorSpeed);

  motor(M1_IN1, M1_IN2, -motorSpeed);
  motor(M3_IN1, M3_IN2, -motorSpeed);
}

void moveRight() {
  motor(M0_IN1, M0_IN2, motorSpeed);
  motor(M2_IN1, M2_IN2, motorSpeed);

  motor(M1_IN1, M1_IN2, motorSpeed);
  motor(M3_IN1, M3_IN2, motorSpeed);
}

void stopMotor() {
  analogWrite(M0_IN1, 0); analogWrite(M0_IN2, 0);
  analogWrite(M1_IN1, 0); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0); analogWrite(M2_IN2, 0);
  analogWrite(M3_IN1, 0); analogWrite(M3_IN2, 0);
}


// ==========================================
// 초기 설정
// ==========================================
void setup() {
  pinMode(M0_IN1, OUTPUT);
  pinMode(M0_IN2, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  stopMotor();

  Serial.begin(115200);
  
  // 블루투스 시작
  SerialBT.begin("ESP32_AED_Robot"); 
  Serial.println("Bluetooth Start! Device Name: ESP32_AED_Robot");
}


// ==========================================
// 블루투스 제어 루프
// ==========================================
void loop() {
  if (SerialBT.available()) {
    // 엔터(\n)를 칠 때까지 글자를 읽어옵니다.
    String command = SerialBT.readStringUntil('\n');
    command.trim(); // 공백, \r(개행) 제거

    Serial.print("Received: ");
    Serial.println(command);

    // 명령어 판별
    if (command == "go") {
      moveForward();
      Serial.println("Command: FORWARD");
    } 
    else if (command == "back") {
      moveBackward();
      Serial.println("Command: BACKWARD");
    } 
    else if (command == "left") {
      moveLeft();
      Serial.println("Command: LEFT");
    } 
    else if (command == "right") {
      moveRight();
      Serial.println("Command: RIGHT");
    } 
    else if (command == "stop") {
      stopMotor();
      Serial.println("Command: STOP");
    }
  }
}
