# 1. ifProjectCode
만약에 프로젝트 - (타임 세이버팀)의 코드 저장소입니다.

# 2. 코드 실행 시 사전 설정
## 2.1. python(노트북 실행용. current: targetDetect.py)
버전: Python 3.10.11<br>
winget install Python.Python.3.10 --version 3.10.11 --no-upgrade --force<br>
<br>
python -m pip install -r "C:\Users\user\Downloads\ifProjectCode-main\ifProjectCode-main\requirements.txt"<br>

## 2.2. c++ (esp32보드용. current: operator.cpp)
Adafruit_NeoPixel <- 이거 라이브러리 설정에서 설치하기.

# 3. 기능
## 컴퓨터
사람 인식<br>
로봇 인식<br>
바닥 인식<br>
경로 생성<br>
로봇과 통신( 이를 통해 경로를 반영시키고, 로봇팔이 작동을 한다. )<br>

## 로봇
기본적인 이동코드<br>
거리에 따라 색이 달라지는 LED 제어<br>
컴퓨터와의 통신<br>

# 4. 참고
## 4.1. 명령어
### 4.1.1 cmd
(requirements.txt 제작)<br>
pip freeze '> requirements.txt<br>
