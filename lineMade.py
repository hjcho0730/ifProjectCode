import cv2
import numpy as np
import matrixCal

# 1. 빈 이미지 생성 (가로 800, 세로 800, 3채널 BGR)
image_size = (800, 800, 3)
canvas = np.zeros(image_size, dtype=np.uint8)
# 배경을 흰색으로 설정하고 싶다면 아래 주석 해제
canvas[:] = (255, 255, 255)

# 2. 임의의 사람 포즈 관절 좌표 정의 (픽셀 좌표계 [x, y])
# 일반적인 뼈대 구조 (OpenPose Body25 또는 COCO 포맷 참고)
joints = {
    "Nose": (400, 150),
    "Neck": (400, 220),
    "R_Shoulder": (470, 230),
    "R_Elbow": (520, 330),
    "R_Wrist": (550, 430),
    "L_Shoulder": (330, 230),
    "L_Elbow": (280, 330),
    "L_Wrist": (250, 430),
    "Mid_Hip": (400, 450),
    "R_Hip": (440, 470),
    "R_Knee": (450, 600),
    "R_Ankle": (460, 720),
    "L_Hip": (360, 470),
    "L_Knee": (350, 600),
    "L_Ankle": (340, 720),
}

# 3. 관절들을 연결하는 막대기(뼈대) 쌍 정의
bones = [
    ("Nose", "Neck"),
    ("Neck", "R_Shoulder"),
    ("R_Shoulder", "R_Elbow"),
    ("R_Elbow", "R_Wrist"),
    ("Neck", "L_Shoulder"),
    ("L_Shoulder", "L_Elbow"),
    ("L_Elbow", "L_Wrist"),
    ("Neck", "Mid_Hip"),
    ("Mid_Hip", "R_Hip"),
    ("R_Hip", "R_Knee"),
    ("R_Knee", "R_Ankle"),
    ("Mid_Hip", "L_Hip"),
    ("L_Hip", "L_Knee"),
    ("L_Knee", "L_Ankle"),
]

# 머리 크기 표현을 위한 중심점과 반지름
head_center = (joints["Nose"][0], joints["Nose"][1] - 30)
head_radius = 25

# 4. 시각화 (OpenCV 활용)
# 색상 정의 (BGR 형식)
COLOR_BONE = (255, 0, 0)  # 파란색 계열 (막대기)
COLOR_JOINT = (0, 0, 255)  # 빨간색 계열 (관절 포인트)
COLOR_HEAD = (0, 255, 0)  # 초록색 계열 (머리)

# 4-1. 머리(원) 그리기
cv2.circle(canvas, head_center, head_radius, COLOR_HEAD, -1)

# 4-2. 막대기(뼈대) 그리기
for bone in bones:
  pt1_name, pt2_name = bone
  pt1 = joints[pt1_name]
  pt2 = joints[pt2_name]
  cv2.line(canvas, pt1, pt2, COLOR_BONE, thickness=4, lineType=cv2.LINE_AA)

# 4-3. 관절 위치(점) 그리기
for joint_name, pos in joints.items():
  cv2.circle(canvas, pos, 6, COLOR_JOINT, -1, lineType=cv2.LINE_AA)
  # 각 관절 이름 텍스트 추가 (디버깅 및 확인용)
  cv2.putText(
      canvas,
      joint_name,
      (pos[0] + 8, pos[1] - 8),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.4,
      (50, 50, 50),
      1,
      cv2.LINE_AA,
  )
outer_pts, _= matrixCal.convexHullDots(list(joints.values()))

pts = np.array(outer_pts, dtype=np.int32)
pts = pts.reshape((-1, 1, 2))

points_2d = pts.squeeze()

# 중심점(무게중심) 계산
center_x = np.mean(points_2d[:, 0])
center_y = np.mean(points_2d[:, 1])

# 중심점 기준 각도(radian) 계산 및 정렬 인덱스 추출
angles = np.arctan2(points_2d[:, 1] - center_y, points_2d[:, 0] - center_x)
sort_indices = np.argsort(angles)

# 추출한 인덱스로 pts 배열 재정렬 및 원래 차원(N, 1, 2) 복원
sorted_pts = pts[sort_indices]

cv2.polylines(canvas, [sorted_pts], isClosed=True, color=(0, 255, 0), thickness=3)


# 5. 이미지 출력 및 키 입력 대기
window_name = "Human Skeleton Simulation for Robot Navigation"
cv2.imshow(window_name, canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()