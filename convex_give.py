import math
import socket
import cv2
import numpy as np
import mediapipe as mp


# 미디어파이프 초기화
mp_pose = mp.solutions.pose
pose_model = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# 제외할 랜드마크 번호 정의 (1~10: 얼굴, 15~22: 손 영역 전체)
EXCLUDED_LANDMARKS = set(range(1, 11)) | set(range(15, 23))

# 확장 거리 (픽셀 단위)
EXPAND_OFFSET = 200

# 웹캠 열기
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("카메라로부터 영상을 가져올 수 없습니다.")
        continue

    # 1. 전처리: 좌우 반전 및 RGB 변환
    frame = cv2.flip(frame, 1)
    h, w, c = frame.shape
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 2. 포즈 감지 수행
    results = pose_model.process(image_rgb)

    # 계산에 사용할 점들을 담을 리스트
    hull_points = []

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # 3. 몸 전체의 중심점(Center Point) 계산 (양 어깨 및 양 골반의 평균)
        # 중요: 몸통 중심을 기준으로 바깥쪽을 정의하기 위함
        try:
            center_x = (landmarks[11].x + landmarks[12].x + landmarks[23].x + landmarks[24].x) / 4 * w
            center_y = (landmarks[11].y + landmarks[12].y + landmarks[23].y + landmarks[24].y) / 4 * h
            center_point = np.array([center_x, center_y])
        except IndexError:
            # 예외 처리: 몸통 점이 안 보일 경우
            center_point = np.array([w/2, h/2])

        # 4. 대상 랜드마크 외곽으로 50픽셀 이동 및 점 수집
        for idx, lm in enumerate(landmarks):
            # 1~10, 15~22번 점은 계산에서 완전히 제외
            if idx in EXCLUDED_LANDMARKS:
                continue

            # 픽셀 좌표로 변환
            pt = np.array([lm.x * w, lm.y * h])

            # 중심점에서 각 점으로 향하는 벡터 및 정규화
            vec = pt - center_point
            norm = np.linalg.norm(vec)

            # 점이 중심과 너무 가까우면 확장 안 함 (오류 방지)
            if norm > 0.1:
                dir_vec = vec / norm
                # 바깥쪽 방향으로 50픽셀 밀어낸 새 좌표 계산
                expanded_pt = pt + dir_vec * EXPAND_OFFSET
            else:
                expanded_pt = pt

            # 컨벡스 헐 계산을 위해 OpenCV 형식(정수형 [x, y])으로 저장
            hull_points.append([int(expanded_pt[0]), int(expanded_pt[1])])

        # 5. 컨벡스 헐(Convex Hull) 계산 및 그리기
        if len(hull_points) > 3: # 최소 4개의 점이 필요
            # 점 리스트를 NumPy 배열로 변환
            pts_array = np.array(hull_points, dtype=np.int32)
            
            # 컨벡스 헐 점 인덱스 찾기
            hull_indices = cv2.convexHull(pts_array, returnPoints=True)
            
            # 결과 화면에 녹색으로 컨벡스 헐 그리기
            # points: 헐을 구성하는 점들의 배열, isClosed=True: 닫힌 다각형
            cv2.polylines(frame, [hull_indices], isClosed=True, color=(0, 255, 0), thickness=2)

    # 6. 결과 화면 출력
    cv2.imshow('Person Convex Hull (Excluded & Expanded)', frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) == ord('q'):
        break
    
    ESP32_IP = "192.168.0.50"  # ESP32 시리얼 모니터에 출력된 IP 주소
ESP32_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd):
    """ESP32로 이동 명령(F, L, R, S) 전송"""
    try:
        sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
    except Exception as e:
        print(f"통신 에러: {e}")

# 기존 아루코 마커 함수 생략... (사용자 제공 코드 동일 사용)

# 미디어파이프 포즈 및 설정
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
EXCLUDED_LANDMARKS = set(range(1, 11)) | set(range(15, 23))
EXPAND_OFFSET = 200

cap = cv2.VideoCapture(0)
matrix = None
matrix_inv = None

# 순차적 목표점 추적을 위한 변수
current_target_index = 0

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        
        suc, markPts, dImage = get_single_marker_corners_list(frame, target_id=0)
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(image_rgb)

        if suc and len(markPts) == 4:
            matrix = getMatrix4(markPts, R)
            matrix_inv = np.linalg.inv(matrix)

        hull_points = []
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # 3D/2D 변환 및 외곽 헐 포인트 계산
            try:
                center_x = (landmarks[11].x + landmarks[12].x + landmarks[23].x + landmarks[24].x) / 4 * w
                center_y = (landmarks[11].y + landmarks[12].y + landmarks[23].y + landmarks[24].y) / 4 * h
                center_point = np.array([center_x, center_y])
            except IndexError:
                center_point = np.array([w / 2, h / 2])

            for idx, lm in enumerate(landmarks):
                if idx in EXCLUDED_LANDMARKS:
                    continue

                pt = np.array([lm.x * w, lm.y * h])
                vec = pt - center_point
                norm = np.linalg.norm(vec)

                if norm > 0.1:
                    expanded_pt = pt + (vec / norm) * EXPAND_OFFSET
                else:
                    expanded_pt = pt

                hull_points.append([int(expanded_pt[0]), int(expanded_pt[1])])

            if len(hull_points) > 3:
                pts_array = np.array(hull_points, dtype=np.int32)
                hull_indices = cv2.convexHull(pts_array, returnPoints=True)
                
                # N개의 외곽 포인트 정렬 리스트
                ordered_hull_pts = [pt[0] for pt in hull_indices]
                cv2.polylines(frame, [hull_indices], isClosed=True, color=(0, 255, 0), thickness=2)

                # -------------------------------------------------------------
                # 2. 로봇 기준 최단 거리 점 선택 및 순차 이동 알고리즘
                # -------------------------------------------------------------
                robot_pos = np.array([w / 2, h])  # 로봇/카메라 하단 중앙
                
                # 최초 실행 시 가장 가까운 점의 인덱스를 탐색
                if current_target_index >= len(ordered_hull_pts):
                    distances = [np.linalg.norm(p - robot_pos) for p in ordered_hull_pts]
                    current_target_index = np.argmin(distances)

                target_pt = ordered_hull_pts[current_target_index]
                
                # 목표점 시각화 (현재 가야할 점: 빨간색, 나머지 점: 파란색)
                for i, p in enumerate(ordered_hull_pts):
                    if i == current_target_index:
                        cv2.circle(frame, tuple(p), 12, (0, 0, 255), -1)  # Target
                    else:
                        cv2.circle(frame, tuple(p), 6, (255, 0, 0), -1)

                # -------------------------------------------------------------
                # 3. 로봇 조종 신호 제어 (X축 오차 및 거리 측정)
                # -------------------------------------------------------------
                target_x = target_pt[0]
                dist_to_target = np.linalg.norm(target_pt - robot_pos)

                # 목표 지점 도달 시 (50px 이내 접근) 다음 옆 점으로 이동
                if dist_to_target < 50:
                    current_target_index = (current_target_index + 1) % len(ordered_hull_pts)
                    send_command('S')
                else:
                    # 방향 조율 (화면 중심 기준)
                    if target_x < (w / 2) - 60:
                        send_command('L')  # 좌회전
                    elif target_x > (w / 2) + 60:
                        send_command('R')  # 우회전
                    else:
                        send_command('F')  # 직진
        else:
            send_command('S')  # 사람 미인식 시 정지

        cv2.imshow('Pose Detection & ESP32 Tracking', frame)

        if cv2.waitKey(1) == ord('q'):
            send_command('S')
            break


# 자원 해제
cap.release()
pose_model.close()
cv2.destroyAllWindows()
