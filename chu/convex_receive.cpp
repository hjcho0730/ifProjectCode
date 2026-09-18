#include <WiFi.h>
#include <WiFiUdp.h>

// 1. 공유기 Wi-Fi 정보 입력
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// UDP 설정
WiFiUDP udp;
const unsigned int localUdpPort = 8888;
char packetBuffer[255];

// 2. 모터 제어 핀 설정 (ESP32 GPIO 핀 번호)
const int ENA = 14;
const int IN1 = 27;
const int IN2 = 26;

const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;

const int SPEED = 180; // 모터 속도 (0 ~ 255)

void setup() {
  Serial.begin(115200);

  // 모터 핀 출력 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  stopMotors();

  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWi-Fi 연결 완료");
  Serial.print("ESP32 IP 주소: ");
  Serial.println(WiFi.localIP()); // 이 IP 주소를 파이썬 코드 ESP32_IP에 넣으세요.

  udp.begin(localUdpPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    
    char command = packetBuffer[0];
    
    if (command == 'F') moveForward();
    else if (command == 'L') turnLeft();
    else if (command == 'R') turnRight();
    else if (command == 'S') stopMotors();
  }
}

void moveForward() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
