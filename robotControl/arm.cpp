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
  servo25.write(160);
  servo33.write(160);
  servo32.write(0);
}

void loop() {
  // GPIO 25, 33: 160도에서 0도로 출력
  for (int angle = 160; angle >= 0; angle--) {
    servo25.write(angle);
    servo33.write(angle);
    delay(15);
  }

  delay(500);

  // GPIO 32: 0도에서 30도로 출력
  for (int angle = 0; angle <= 30; angle++) {
    servo32.write(angle);
    delay(15);
  }

  delay(500);
}
