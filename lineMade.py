import os


def load_landmark_data(file_name="my_list.txt"):
    # 현재 작업 경로에 있는 파일의 절대 경로 생성
    current_path = os.getcwd()
    file_path = os.path.join(current_path, file_name)

    result_list = []

    # 파일 읽기
    with open(file_path, "r", encoding="utf-8") as f:
        # 빈 줄(\n\n)을 기준으로 전체 데이터를 각각의 블록으로 쪼갭니다.
        blocks = f.read().strip().split("\n\n")

    for block in blocks:
        current_dict = {}
        # 각 블록 내의 한 줄씩 처리
        for line in block.splitlines():
            if ":" in line:
                key, val = line.split(":")
                # 공백을 제거하고 숫자는 float으로 변환하여 저장
                current_dict[key.strip()] = float(val.strip())

        # 데이터가 정상적으로 들어있는 딕셔너리만 리스트에 추가
        if current_dict:
            result_list.append(current_dict)

    return result_list


# 함수 실행 및 결과 출력
loaded_list = load_landmark_data()

# 첫 2개 데이터만 샘플로 확인
print(f"총 불러온 데이터 개수: {len(loaded_list)}개")
print("첫 번째 데이터:", loaded_list[0])
print("두 번째 데이터:", loaded_list[1])

import numpy as np
import mediapipe as mp
import cv2

frame= np.zeros((720, 960, 3), dtype=np.uint8)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# 예시: loaded_landmarks는 파일에서 불러온 랜드마크 리스트라고 가정
# 구조: [ { 'x': 0.5, 'y': 0.6, 'z': ... }, ... ] 또는 [ [x, y, z], ... ]

h, w, _ = frame.shape
w *= 0.6
w= int(w)
h *= 0.6
h= int(h)

# 1. 뼈대(Connection) 먼저 그리기
for connection in mp_pose.POSE_CONNECTIONS:
    start_idx, end_idx = connection
    
    # 데이터 형식에 맞게 좌표 추출 (딕셔너리 형태일 때 예시)
    p1 = loaded_list[start_idx]
    p2 = loaded_list[end_idx]
    
    # 정규화된 좌표(0~1)라면 픽셀 값으로 변환 필요
    pt1 = (int(p1['x'] * w + 150), int(p1['y'] * h + 150))
    pt2 = (int(p2['x'] * w + 150), int(p2['y'] * h + 150))
    
    cv2.line(frame, pt1, pt2, (0, 255, 0), 2) # 초록색 선

# 2. 관절(Landmark) 점 찍기
for p in loaded_list:
    center = (int(p['x'] * w + 150), int(p['y'] * h + 150))
    cv2.circle(frame, center, 3, (0, 0, 255), -1) # 빨간색 점
    cv2.circle(frame, center, 50+50, (0, 0, 255), -1) # 빨간색 점

f= (lambda idx: (loaded_list[idx]['x'] * w + 150, loaded_list[idx]['y'] * h + 150))


d1= f(11)
d2= f(23)

d3= ((d1[0]+d2[0])//2, (d1[1]+d2[1])//2)
cv2.circle(frame, (int(d3[0]), int(d3[1])), 25, (0, 255, 0), -1) # 빨간색 점
cv2.circle(frame, (int(d1[0]), int(d1[1])), 50, (255, 0, 0), -1) # 빨간색 점
cv2.circle(frame, (int(d2[0]), int(d2[1])), 50, (255, 0, 0), -1) # 빨간색 점

import math
for i in range(360):
    x4, y4= math.cos((i*math.pi)/180) * 350, math.sin((i*math.pi)/180) * 350
    d2= (x4+d3[0], y4+d3[1])
    cv2.circle(frame, (int(d2[0]), int(d2[1])), 10, (255, 0, 0), -1) # 빨간색 점
    



while True:
    cv2.imshow('Pose Detection1', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
