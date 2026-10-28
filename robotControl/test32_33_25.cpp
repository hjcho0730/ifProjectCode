#include <ESP32Servo.h>

Servo servo25;
Servo servo33;
Servo servo32;

void setup() {
  // 서보 PWM 설정
  servo25.setPeriodHertz(50);
  servo33.setPeriodHertz(50);
  servo32.setPeriodHertz(50);

  // 핀 연결 및 펄스 폭 설정
  servo25.attach(25, 500, 2500);
  servo33.attach(33, 500, 2500);
  servo32.attach(32, 500, 2500);

  // 초기 각도 설정
  servo25.write(90);
  servo33.write(90);
  servo32.write(90);
}

void loop() {
}
