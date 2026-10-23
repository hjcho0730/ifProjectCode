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

const int motorSpeed = 130;

// === 통신 타임아웃 및 미끄러방지(제동) 설정 ===
const unsigned long TIMEOUT_MS = 550;     // 통신 끊김 판단 시간 (밀리초)
unsigned long lastReceiveTime = 0;         // 마지막으로 통신이 수신된 시간
bool isTimedOut = false;                 // 타임아웃 상태 플래그

// 로봇의 마지막 동작 상태를 관리하기 위한 열거형
enum MotionState {
  STATE_IDLE,     // 대기 또는 정지 상태
  STATE_FORWARD,  // 전진 중
  STATE_BACKWARD, // 후진 중
  STATE_LEFT,     // 좌회전 중
  STATE_RIGHT     // 우회전 중
};

MotionState lastMotion = STATE_IDLE;      // 마지막 동작 상태 기록 변수
const int BRAKE_DELAY_MS = 80;            // 관성 상쇄용 반대 구동 유지 시간 (밀리초)

// === 비동기 제동(Non-blocking Brake)을 위한 상태 및 변수 ===
enum BrakePhase {
  BRAKE_OFF,          // 제동 작동 안 함
  BRAKE_STAGE_PAUSE,  // 1단계: 정지 후 대기 (30ms)
  BRAKE_STAGE_ACTIVE  // 2단계: 반대 구동으로 관성 상쇄 (BRAKE_DELAY_MS)
};

BrakePhase brakePhase = BRAKE_OFF;
unsigned long brakeTimer = 0;
MotionState targetBrakeMotion = STATE_IDLE;


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
  motor(M0_IN1, M0_IN2, motorSpeed);
  motor(M2_IN1, M2_IN2, motorSpeed);
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

// === 관성 및 미끄러짐 방지 제동 시작 함수 (비동기 트리거) ===
void startBrake() {
  if (lastMotion == STATE_FORWARD || lastMotion == STATE_LEFT || lastMotion == STATE_RIGHT) {
    targetBrakeMotion = lastMotion;
    stopMotor();
    brakeTimer = millis();
    brakePhase = BRAKE_STAGE_PAUSE; // 1단계(30ms 대기) 진입
  } else {
    stopMotor();
    brakePhase = BRAKE_OFF;
    lastMotion = STATE_IDLE;
  }
}

// === 루프에서 매번 호출하여 비동기로 제동 단계를 처리하는 함수 ===
void updateBrake() {
  if (brakePhase == BRAKE_STAGE_PAUSE) {
    if (millis() - brakeTimer >= 30) {
      // 30ms 경과 후 관성 상쇄용 반대 구동 시작
      if (targetBrakeMotion == STATE_FORWARD) moveBackward();
      else if (targetBrakeMotion == STATE_LEFT) moveRight();
      else if (targetBrakeMotion == STATE_RIGHT) moveLeft();
      
      brakeTimer = millis();
      brakePhase = BRAKE_STAGE_ACTIVE; // 2단계(반대 구동 유지) 진입
    }
  } 
  else if (brakePhase == BRAKE_STAGE_ACTIVE) {
    if (millis() - brakeTimer >= BRAKE_DELAY_MS) {
      // 관성 상쇄 유지 시간 경과 후 최종 정지
      stopMotor();
      brakePhase = BRAKE_OFF;
      lastMotion = STATE_IDLE;
    }
  }
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

  Serial.begin(9600);
  
  // 블루투스 시작
  SerialBT.begin("ESP32_AED_Robot"); 
  Serial.println("Bluetooth Start! Device Name: ESP32_AED_Robot");

  lastReceiveTime = millis();
}


// ==========================================
// 블루투스 제어 루프
// ==========================================
void loop() {
  // 0. 비동기 제동 프로세스 업데이트 (딜레이 없이 계속 실행됨)
  updateBrake();

  // 1. 블루투스 데이터 수신 처리
  if (SerialBT.available()) {
    lastReceiveTime = millis();
    isTimedOut = false;

    // 새로운 명령이 들어오면 진행 중이던 제동 시퀀스를 즉시 취소
    brakePhase = BRAKE_OFF;

    String command = SerialBT.readStringUntil('\n');
    command.trim(); // 공백, \r(개행) 제거

    Serial.print("Received: ");
    Serial.println(command);

    // 명령어 판별
    if (command == "go") {
      stopMotor();
      moveForward();
      lastMotion = STATE_FORWARD; // 전진 상태 기록
      Serial.println("Command: FORWARD");
    } 
    else if (command == "back") {
      stopMotor();
      moveBackward();
      lastMotion = STATE_BACKWARD;
      Serial.println("Command: BACKWARD");
    } 
    else if (command == "left") {
      stopMotor();
      moveLeft();
      lastMotion = STATE_LEFT;    // 좌회전 상태 기록
      Serial.println("Command: LEFT");
    } 
    else if (command == "right") {
      stopMotor();
      moveRight();
      lastMotion = STATE_RIGHT;   // 우회전 상태 기록
      Serial.println("Command: RIGHT");
    } 
    else if (command == "stop") {
      startBrake(); // 비동기 제동 시작
      Serial.println("Command: STOP");
    }
  }

  // 2. 통신 타임아웃 체크 (일정 시간 동안 수신이 없으면 제동 시작)
  if (!isTimedOut && (millis() - lastReceiveTime > TIMEOUT_MS)) {
    if (brakePhase == BRAKE_OFF) {
      startBrake();
    }
    isTimedOut = true; 
    Serial.println("Timeout! Connection lost. Motors stopped with brake.");
  }
}
