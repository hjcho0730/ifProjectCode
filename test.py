import time
import cv2
from ultralytics import YOLO

# [Step 1] 학습된 best.pt 모델 불러오기
model = YOLO('best.pt')

# [Step 2] 웹캠 연결
cap = cv2.VideoCapture(0)

# 'fall' 감지 시간을 측정하기 위한 변수 초기화
fall_start_time = None
EMERGENCY_THRESHOLD = 5.0  # 긴급 상황 기준 시간 (초 단위)

print("웹캠을 시작합니다. 종료하려면 키보드의 'q'를 누르세요.")

while cap.isOpened():
    success, frame = cap.read()
    
    if not success:
        print("웹캠 프레임을 읽을 수 없습니다.")
        break

    # [Step 3] YOLO 모델로 감지 수행 (conf=0.5: 50% 이상 신뢰도)
    results = model(frame, conf=0.6)
    
    # 기본 바운딩 박스가 그려진 프레임 생성
    annotated_frame = results[0].plot()

    # -------------------------------------------------------------
    # [Step 4] 'fall' 클래스 감지 및 5초 지속 시간 체크 로직
    # -------------------------------------------------------------
    is_fall_detected = False

    # 감지된 객체들 검사
    for box in results[0].boxes:
        cls_id = int(box.cls[0])           # 클래스 ID
        class_name = model.names[cls_id]   # 클래스 이름 (ex: 'fall')
        
        # 인식된 클래스 이름이 'fall'인지 확인 (대소문자 구분을 방지하기 위해 lower() 적용)
        if class_name.lower() == 'fall':
            is_fall_detected = True
            break

    # 'fall'이 감지되었을 때 시간 측정
    if is_fall_detected:
        if fall_start_time is None:
            fall_start_time = time.time()  # 처음 감지된 시각 저장
        
        # 감지 지속 시간 계산
        elapsed_time = time.time() - fall_start_time

        # 5초 이상 지속되었는지 체크
        if elapsed_time >= EMERGENCY_THRESHOLD:
            # 화면 왼쪽 상단(x=30, y=60)에 빨간색 EMERGENCY 문구 출력
            cv2.putText(
                annotated_frame, 
                "EMERGENCY", 
                (30, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1.8,                  # 글자 크기
                (0, 0, 255),          # BGR 색상 (빨간색)
                4,                    # 글자 두께
                cv2.LINE_AA
            )
    else:
        # 화면에서 'fall'이 사라지면 타이머 리셋
        fall_start_time = None

    # -------------------------------------------------------------
    # [Step 5] 결과 화면 출력 및 종료 조건
    # -------------------------------------------------------------
    cv2.imshow("Roboflow AI Webcam Test", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# 자원 해제 및 창 닫기
cap.release()
cv2.destroyAllWindows()