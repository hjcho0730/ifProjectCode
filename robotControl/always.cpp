// 이 코드는 용마제용 360도 모터의 회전을 위해 작성되었음...
#include <Servo.h>

Servo myServo;  // 서보모터 객체 생성

void setup() {
  myServo.attach(9);  // 9번 핀에 서보모터 신호선 연결
}

void loop() {
  // 한쪽 방향으로 계속 회전 (모터 종류에 따라 0 또는 180 입력)
  // 정지하려면 myServo.write(90); 을 사용하세요.
  myServo.write(180); 
}
