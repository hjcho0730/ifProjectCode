#include <Adafruit_NeoPixel.h>

// 핀 정의
#define PIN_RING1   5       // 첫 번째 원형 LED 핀
#define PIN_RING2   16      // 두 번째 원형 LED 핀
#define NUM_LEDS    16      // 각 링의 LED 개수
#define TOTAL_LEDS  32      // 두 링을 합친 총 LED 개수

#define TRIG_PIN    23      // 초음파 센서 Trig 핀
#define ECHO_PIN    19      // 초음파 센서 Echo 핀

// NeoPixel 객체 생성
Adafruit_NeoPixel ring1(NUM_LEDS, PIN_RING1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring2(NUM_LEDS, PIN_RING2, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);

  // 초음파 센서 핀 모드 설정
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // NeoPixel 초기화
  ring1.begin();
  ring2.begin();
  ring1.setBrightness(40); // 밝기 설정 (0~255)
  ring2.setBrightness(40);
  
  ring1.show();
  ring2.show();
}

void loop() {
  // 1. 초음파 거리 측정 (cm 단위)
  long distance = measureDistance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // 2. 거리에 따른 LED 켜질 개수 계산 (가까울수록 많이 켜짐)
  // 예: 5cm 이하일 때 32개 모두 켜짐, 40cm 이상일 때 모두 꺼짐 (0개)
  int litCount = map(distance, 5, 40, TOTAL_LEDS, 0);
  litCount = constrain(litCount, 0, TOTAL_LEDS); // 범위를 0~32 사이로 제한

  // 3. LED 상태 업데이트
  updateLEDs(litCount);

  delay(50); // 측정 주기 조절
}

// 초음파 센서로 거리를 측정하는 함수
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 타임아웃 30ms 설정
  if (duration == 0) return 100; // 센서 감지 실패 시 멀리 있는 것으로 처리 (100cm)

  long cm = duration * 0.034 / 2; // 음속을 이용해 cm로 변환
  return cm;
}

// 전체 LED 중 지정된 개수만큼 켜는 함수 (가까울수록 파랑 -> 초록 -> 빨강으로 색상 변화)
void updateLEDs(int count) {
  for (int i = 0; i < TOTAL_LEDS; i++) {
    uint32_t color = 0;

    if (i < count) {
      // 켜져야 하는 LED: 위치에 따라 색상 변화 (초반엔 파랑, 중간엔 초록, 끝쪽엔 빨강)
      if (i < 10) {
        color = ring1.Color(0, 0, 255);    // 파란색 (멀리 있음)
      } else if (i < 22) {
        color = ring1.Color(0, 255, 0);    // 초록색 (중간)
      } else {
        color = ring1.Color(255, 0, 0);    // 빨간색 (매우 가까움)
      }
    } else {
      // 꺼져야 하는 LED
      color = ring1.Color(0, 0, 0);
    }

    // 0~15번은 1번 링, 16~31번은 2번 링에 할당
    if (i < NUM_LEDS) {
      ring1.setPixelColor(i, color);
    } else {
      ring2.setPixelColor(i - NUM_LEDS, color);
    }
  }

  ring1.show();
  ring2.show();
}
