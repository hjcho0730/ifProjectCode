import math
import socket
import cv2
import numpy as np
import mediapipe as mp

# -------------------------------------------------------------
# 1. ESP32 네트워크 통신 설정 (ESP32의 IP로 수정)
# -------------------------------------------------------------
ESP32_IP = "192.168.0.50"  # ESP32 시리얼 모니터에 출력된 IP 주소
ESP32_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(cmd):
    """ESP32로 이동 명령(F, L, R, S) 전송"""
    try:
        sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
    except Exception as e:
        print(f"통신 에러: {e}")

# -------------------------------------------------------------
# 2. 아루코 마커 및 호모그래피 보조 함수
# -------------------------------------------------------------
MARKER_SIZE = 100.0  # 단위: mm
R = np.array([
    [0, 0],
    [MARKER_SIZE, 0],
    [MARKER_SIZE, MARKER_SIZE],
    [0, MARKER_SIZE]
], dtype=np.float32)

def get_homography_matrix(corners):
    pts_src = np.array(corners, dtype=np.float32)
    matrix, _ = cv2.findHomography(pts_src, R)
    return matrix

def get3Dpos(matrix, pt_2d):
    pt = np.array([pt_2d[0], pt_2d[1], 1.0], dtype=np.float32)
    res = np.dot(matrix, pt)
    if res[2] != 0:
        return [res[0] / res[2], res[1] / res[2]]
    return [0, 0]

def transform_to_screen(matrix_inv, pt_3d):
    pt = np.array([pt_3d[0], pt_3d[1], 1.0], dtype=np.float32)
    res = np.dot(matrix_inv, pt)
    if res[2] != 0:
        return [res[0] / res[2], res[1] / res[2]]
    return [0, 0]

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# -------------------------------------------------------------
# 3. 미디어파이프 포즈 및 설정
# -------------------------------------------------------------
mp_pose = mp.solutions.pose
pose_model = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

EXCLUDED_LANDMARKS = set(range(1, 11)) | set(range(15, 23))
REAL_WORLD_OFFSET = 300  # 단위: mm (30cm 외곽 확장)

cap = cv2.VideoCapture(0)

matrix = None
matrix_inv = None

# 순차적 점 이동 추적 인덱스 (초기값 None)
current_target_index = None

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("카메라로부터 영상을 가져올 수 없습니다.")
        continue

    # 1. 전처리: 좌우 반전 및 RGB 변환
    frame = cv2.flip(frame, 1)
    h, w, c = frame.shape
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 2. 아루코 마커 감지 및 호모그래피 행렬 계산
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None and 0 in ids:
        idx = np.where(ids == 0)[0][0]
        markPts = corners[idx][0]
        matrix = get_homography_matrix(markPts)
        if matrix is not None:
            matrix_inv = np.linalg.pinv(matrix)

    # 3. 포즈 감지 수행
    results = pose_model.process(image_rgb)
    hull_points = []

    if results.pose_landmarks:
        landmarks = results.pose_landmarks.landmark

        # 몸 전체의 중심점(Center Point) 계산
        try:
            center_x = (landmarks[11].x + landmarks[12].x + landmarks[23].x + landmarks[24].x) / 4 * w
            center_y = (landmarks[11].y + landmarks[12].y + landmarks[23].y + landmarks[24].y) / 4 * h
            center_point = np.array([center_x, center_y])
        except IndexError:
            center_point = np.array([w / 2, h / 2])

        # 4. 3D 실제 거리(300mm) 기반 외곽 확장 점 수집
        for idx, lm in enumerate(landmarks):
            if idx in EXCLUDED_LANDMARKS:
                continue

            pt = np.array([lm.x * w, lm.y * h])

            if matrix is not None and matrix_inv is not None:
                try:
                    center_3d = np.array(get3Dpos(matrix, center_point))
                    pt_3d = np.array(get3Dpos(matrix, pt))
                    
                    vec_3d = pt_3d - center_3d
                    norm_3d = np.linalg.norm(vec_3d)
                    
                    if norm_3d > 0:
                        expanded_3d = pt_3d + (vec_3d / norm_3d) * REAL_WORLD_OFFSET
                        expanded_pt = transform_to_screen(matrix_inv, expanded_3d)
                    else:
                        expanded_pt = pt
                except Exception:
                    expanded_pt = pt
            else:
                vec = pt - center_point
                norm = np.linalg.norm(vec)
                expanded_pt = pt + (vec / norm) * 100 if norm > 0.1 else pt

            hull_points.append([int(expanded_pt[0]), int(expanded_pt[1])])

        # -------------------------------------------------------------
        # 5. 컨벡스 헐 계산 및 동일 색상 표시
        # -------------------------------------------------------------
        if len(hull_points) > 3:
            pts_array = np.array(hull_points, dtype=np.int32)
            
            # 반시계방향(왼쪽 순환) 정렬
            hull_indices = cv2.convexHull(pts_array, returnPoints=True, clockwise=False)

            ordered_hull_pts = [pt[0] for pt in hull_indices]
            
            # 컨벡스 헐 외곽선 (녹색)
            cv2.polylines(frame, [hull_indices], isClosed=True, color=(0, 255, 0), thickness=2)

            # 모든 점을 동일한 크기(반지름 6) 및 단일 색상(녹색)으로 통합 그리기
            for p in ordered_hull_pts:
                cv2.circle(frame, tuple(p), 6, (0, 255, 0), -1)

            robot_pos = np.array([w / 2, h])  # 로봇/카메라 하단 중앙

            # 처음 시작 시 로봇과 가장 가까운 점의 위치(인덱스) 계산
            if current_target_index is None or current_target_index >= len(ordered_hull_pts):
                distances = [np.linalg.norm(p - robot_pos) for p in ordered_hull_pts]
                current_target_index = np.argmin(distances)

            target_pt = ordered_hull_pts[current_target_index]

            # 로봇 이동 제어
            target_x = target_pt[0]
            dist_to_target = np.linalg.norm(target_pt - robot_pos)

            # 해당 점 근처(50px)에 다다르면 순차적으로 다음 연결 점으로 변경
            if dist_to_target < 50:
                current_target_index = (current_target_index + 1) % len(ordered_hull_pts)
                send_command('S')
            else:
                if target_x < (w / 2) - 60:
                    send_command('L')  # 좌회전
                elif target_x > (w / 2) + 60:
                    send_command('R')  # 우회전
                else:
                    send_command('F')  # 직진
    else:
        send_command('S')  # 사람 미인식 시 정지

    # 화면 출력
    cv2.imshow('Convex Hull Tracking', frame)

    if cv2.waitKey(1) == ord('q'):
        send_command('S')
        break

# 자원 해제
cap.release()
pose_model.close()
cv2.destroyAllWindows()

# 자원 해제
cap.release()
pose_model.close()
cv2.destroyAllWindows()
