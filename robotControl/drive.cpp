#include "BluetoothSerial.h"
#include <ESP32Servo.h>

Servo servo25;
Servo servo33;
Servo servo32;

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

const int motorSpeed = 165;

// === 통신 타임아웃 및 미끄러방지(제동) 설정 ===
const unsigned long TIMEOUT_MS = 117;     // 통신 끊김 판단 시간 (밀리초)
unsigned long lastReceiveTime = 0;        // 마지막으로 통신이 수신된 시간
bool isTimedOut = false;                // 타임아웃 상태 플래그

// 로봇의 마지막 동작 상태를 관리하기 위한 열거형
enum MotionState {
  STATE_IDLE,     // 대기 또는 정지 상태
  STATE_FORWARD,  // 전진 중
  STATE_BACKWARD, // 후진 중
  STATE_LEFT,     // 좌회전 중
  STATE_RIGHT     // 우회전 중
};

MotionState lastMotion = STATE_IDLE;      // 마지막 동작 상태 기록 변수
const int BRAKE_DELAY_MS = 57;            // 관성 상쇄용 반대 구동 유지 시간 (밀리초)

// === 비동기 제동(Non-blocking Brake)을 위한 상태 및 변수 ===
enum BrakePhase {
  BRAKE_OFF,          // 제동 작동 안 함
  BRAKE_STAGE_PAUSE,  // 1단계: 정지 후 대기 (30ms)
  BRAKE_STAGE_ACTIVE  // 2단계: 반대 구동으로 관성 상쇄 (BRAKE_DELAY_MS)
};

BrakePhase brakePhase = BRAKE_OFF;
unsigned long brakeTimer = 0;
MotionState targetBrakeMotion = STATE_IDLE;

// === 비동기 로봇팔(Non-blocking Arm)을 위한 상태 및 변수 ===
enum ArmState {
  ARM_IDLE,
  ARM_STEP_1_DOWN,
  ARM_DELAY_1,
  ARM_STEP_2_DOWN,
  ARM_DELAY_2,
  ARM_STEP_3_UP,
  ARM_DELAY_3,
  ARM_STEP_4_UP
};

ArmState armState = ARM_IDLE;
unsigned long armTimer = 0;
int armAngle25 = 155;
int armAngle32 = 50;


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
  if (brakePhase != BRAKE_OFF) {
    return;
  }

  targetBrakeMotion = lastMotion;
  stopMotor();

  if (targetBrakeMotion == STATE_FORWARD ||
      targetBrakeMotion == STATE_LEFT ||
      targetBrakeMotion == STATE_RIGHT) {
    brakeTimer = millis();
    brakePhase = BRAKE_STAGE_PAUSE;
  } else {
    brakePhase = BRAKE_OFF;
  }

  lastMotion = STATE_IDLE;
}

// === 루프에서 매번 호출하여 비동기로 제동 단계를 처리하는 함수 ===
void updateBrake() {
  unsigned long now = millis();

  switch (brakePhase) {
    case BRAKE_OFF:
      break;

    case BRAKE_STAGE_PAUSE:
      if (now - brakeTimer >= 35) {
        if (targetBrakeMotion == STATE_FORWARD) {
          moveBackward();
        }
        else if (targetBrakeMotion == STATE_LEFT) {
          moveRight();
        }
        else if (targetBrakeMotion == STATE_RIGHT) {
          moveLeft();
        }

        brakeTimer = now;
        brakePhase = BRAKE_STAGE_ACTIVE;
      }
      break;

    case BRAKE_STAGE_ACTIVE:
      if (now - brakeTimer >= BRAKE_DELAY_MS) {
        stopMotor();
        brakePhase = BRAKE_OFF;
        targetBrakeMotion = STATE_IDLE;
        lastMotion = STATE_IDLE;
      }
      break;
  }
}

// === 비동기 로봇팔 동작 업데이트 함수 (루프에서 매번 호출) ===
void updateRobotArm() {
  if (armState == ARM_IDLE) return;

  unsigned long now = millis();

  switch (armState) {
    case ARM_STEP_1_DOWN:
      if (now - armTimer >= 2) {
        armTimer = now;
        servo25.write(armAngle25);
        servo33.write(190 - armAngle25);
        armAngle25--;
        if (armAngle25 < 35) {
          armState = ARM_DELAY_1;
          armTimer = now;
        }
      }
      break;

    case ARM_DELAY_1:
      if (now - armTimer >= 300) {
        armState = ARM_STEP_2_DOWN;
        armAngle32 = 50;
        armTimer = now;
      }
      break;

    case ARM_STEP_2_DOWN:
      if (now - armTimer >= 6) {
        armTimer = now;
        servo32.write(armAngle32);
        armAngle32--;
        if (armAngle32 < 0) {
          armState = ARM_DELAY_2;
          armTimer = now;
        }
      }
      break;

    case ARM_DELAY_2:
      if (now - armTimer >= 4000) {
        armState = ARM_STEP_3_UP;
        armAngle32 = 0;
        armTimer = now;
      }
      break;

    case ARM_STEP_3_UP:
      if (now - armTimer >= 6) {
        armTimer = now;
        servo32.write(armAngle32);
        armAngle32++;
        if (armAngle32 > 50) {
          armState = ARM_DELAY_3;
          armTimer = now;
        }
      }
      break;

    case ARM_DELAY_3:
      if (now - armTimer >= 300) {
        armState = ARM_STEP_4_UP;
        armAngle25 = 35;
        armTimer = now;
      }
      break;

    case ARM_STEP_4_UP:
      if (now - armTimer >= 2) {
        armTimer = now;
        servo25.write(armAngle25);
        servo33.write(190 - armAngle25);
        armAngle25++;
        if (armAngle25 > 155) {
          armState = ARM_IDLE;
        }
      }
      break;

    default:
      break;
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

  servo25.setPeriodHertz(50);
  servo33.setPeriodHertz(50);
  servo32.setPeriodHertz(50);

  servo25.attach(25, 500, 2500);
  servo33.attach(33, 500, 2500);
  servo32.attach(32, 500, 2500);

  servo25.write(155);
  servo33.write(35);
  servo32.write(50);

  stopMotor();

  Serial.begin(9600);
  
  SerialBT.begin("ESP32_AED_Robot"); 
  Serial.println("Bluetooth Start! Device Name: ESP32_AED_Robot");

  lastReceiveTime = millis();
}

// ==========================================
// 블루투스 제어 루프
// ==========================================
void loop() {
  // 0. 비동기 제동 및 로봇팔 프로세스 업데이트 (블로킹 없이 계속 실행됨)
  updateBrake();
  updateRobotArm();

  // 1. 블루투스 데이터 수신 처리
  if (SerialBT.available()) {
    lastReceiveTime = millis();
    isTimedOut = false;

    String command = SerialBT.readStringUntil('\n');
    command.trim();

    Serial.print("Received: ");
    Serial.println(command);

    if (command == "go") {
      brakePhase = BRAKE_OFF;
      stopMotor();
      moveForward();
      lastMotion = STATE_FORWARD;
      Serial.println("Command: FORWARD");
    } 
    else if (command == "back") {
      brakePhase = BRAKE_OFF;
      stopMotor();
      moveBackward();
      lastMotion = STATE_BACKWARD;
      Serial.println("Command: BACKWARD");
    } 
    else if (command == "left") {
      brakePhase = BRAKE_OFF;
      stopMotor();
      moveLeft();
      lastMotion = STATE_LEFT;
      Serial.println("Command: LEFT");
    } 
    else if (command == "right") {
      brakePhase = BRAKE_OFF;
      stopMotor();
      moveRight();
      lastMotion = STATE_RIGHT;
      Serial.println("Command: RIGHT");
    } 
    else if (command == "stop") {
      startBrake();
      Serial.println("Command: STOP");
    }
    else if (command == "arm") {
      //버퍼 비우기
      while(SerialBT.available()) {
        SerialBT.read();
      }
      startBrake();
      // 로봇팔이 이미 동작 중이 아닐 때만 시작 트리거
      if (armState == ARM_IDLE) {
        armState = ARM_STEP_1_DOWN;
        armAngle25 = 155;
        armTimer = millis();
      }
      Serial.println("Command: ARM");
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
