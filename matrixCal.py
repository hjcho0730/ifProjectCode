import cv2
import numpy as np
import os

# 설정값
#HSV_YELLOW = [(20, 100, 100), (40, 255, 255)] # 노란색 범위
HSV_YELLOW = [(145, 20, 100), (170, 165, 255)] # 노란색 범위
HSV_BLUE = [(100, 100, 100), (130, 255, 255)]  # 파란색 범위
KERNEL = np.ones((5, 5), np.uint8)            # 통합을 위한 커널

# 정삼각형을 그리기 위한 기준값 (중심에서 꼭짓점까지의 거리 R)
R = 60

def get_blobs(frame, lower, upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    centers = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 50:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append([cX, cY])
    return centers

def draw_dots(frame: np.ndarray, pts: list)-> np.ndarray:
    processed_frame = frame.copy()
    for i, pt in enumerate((tuple(i) for i in pts)):
        cv2.circle(processed_frame, (int(pt[0]), int(pt[0])), 7, (0, 255, 0), -1)
        cv2.putText(processed_frame, f"P{i}", (int(pt[0])+10, int(pt[0])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    return processed_frame

def convexHullDots(dots: list) -> tuple[list, list]:
    pts_arr = np.array(dots, dtype=np.float32)
    hull = cv2.convexHull(pts_arr)
    hull_coords = [tuple(pt[0]) for pt in hull]
        
    outer_pts = [p for p in dots if tuple(p) in hull_coords]
    center_pts = [p for p in dots if tuple(p) not in hull_coords]
    
    return outer_pts, center_pts

def getMatrix4(outer_pts, R):
    """
    4개의 꼭짓점(outer_pts)을 받아 정사각형 기준의 getPerspectiveTransform 행렬을 반환합니다.
    outer_pts: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] 형태의 4개 점 리스트 또는 numpy 배열
    """
    # 1. 4점의 중심점(외심/평균 중심) 계산
    pts_np = np.array(outer_pts, dtype=np.float32)
    center_pt = np.mean(pts_np, axis=0)
    
    # 2. 중심점 기준 각도순으로 점들 정렬 (시계/반시계 일관성 유지)
    angles = [np.arctan2(p[1] - center_pt[1], p[0] - center_pt[0]) for p in pts_np]
    sorted_indices = np.argsort(angles)
    sorted_outer = [outer_pts[i] for i in sorted_indices]
    
    # 3. 정렬된 순서에 맞춰 이상적인 정사각형 좌표(IDEAL_PTS) 동적 생성
    # 첫 번째 점의 중심으로부터의 거리(반지름 R)를 기준으로 정사각형 크기 설정
    #R = np.linalg.norm(sorted_outer[0] - center_pt)
    
    ideal_coords = [(0,0), (R,0), (R,R), (0,R), ]
    #for i in range(4):
        # 90도(pi/2) 간격으로 정사각형 꼭짓점 배치
    #    angle = i * (np.pi / 2)
    #    ideal_coords.append([R * np.cos(angle), R * np.sin(angle)])
    
    src_pts = np.array(sorted_outer, dtype=np.float32)
    dst_pts = np.array(ideal_coords, dtype=np.float32)
    
    # 4. 원근 변환 행렬 계산
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return matrix

def getMatrix(outer_pts, center_pt, R):
    # 1. 외부 점들을 중심점 기준 각도순으로 정렬 (일관성 유지)
    angles = [np.arctan2(p[1]-center_pt[1], p[0]-center_pt[0]) for p in outer_pts]
    sorted_indices = np.argsort(angles)
    sorted_outer = [outer_pts[i] for i in sorted_indices]
    
    # 2. 정렬된 순서에 맞춰 이상적인 좌표(IDEAL_PTS)를 동적으로 생성
    # 정삼각형(꼭짓점 3개) + 중심점(1개)
    ideal_coords = []
    for i in range(3):
        # 120도(2*pi/3) 간격으로 꼭짓점 배치
        angle = i * (2 * np.pi / 3)
        ideal_coords.append([R * np.cos(angle), R * np.sin(angle)])
    ideal_coords.append([0, 0]) # 마지막은 항상 중심
    
    src_pts = np.array(sorted_outer + [center_pt], dtype=np.float32)
    dst_pts = np.array(ideal_coords, dtype=np.float32)
    
    # 왜곡 보정 및 변환 행렬
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    return matrix

def get3Dpos(_matrix, pt): #pt
    pt2d = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    pt3d = cv2.perspectiveTransform(pt2d, _matrix)
    
    x3d, y3d = pt3d[0][0]
    return x3d, y3d

def transform_to_screen(matrix_inv: np.ndarray, plane_pt: tuple) -> tuple:
    """
    미리 계산된 역행렬(matrix_inv)과 3차원 평면상의 좌표를 받아 화면(이미지) 좌표로 변환합니다.

    Parameters:
        matrix_inv (np.ndarray): 미리 계산된 3x3 역변환 행렬 (np.linalg.inv(matrix))
        plane_pt (tuple): 변환할 평면상의 (x, y) 좌표 튜플

    Returns:
        tuple: 변환된 화면상의 (x, y) 좌표 튜플
    """
    # 1. 튜플 좌표를 cv2.perspectiveTransform에 맞는 형태(1 x 1 x 2)로 변환
    pts = np.array([[list(plane_pt)]], dtype=np.float32)

    # 2. 역변환 적용
    transformed_pts = cv2.perspectiveTransform(pts, matrix_inv)

    # 3. 결과 배열을 (x, y) 튜플 형태로 추출
    screen_pt = (float(transformed_pts[0, 0, 0]), float(transformed_pts[0, 0, 1]))

    return screen_pt

def warp_perspective_no_crop(frame, matrix):
    """
    원근 변환 시 이미지가 잘리거나 음수 좌표로 인해 사라지는 현상을 방지하는 함수.
    
    Parameters:
        frame: 입력 이미지 (numpy array)
        matrix: 3x3 원근 변환 행렬 (numpy array)
        
    Returns:
        ff: 변환 및 크기가 보정된 결과 이미지
    """
    h, w = frame.shape[:2]
    
    # 1. 원본 이미지의 네 모서리 좌표 정의
    corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    
    # 2. 변환 행렬을 적용했을 때 모서리들이 이동할 위치 계산
    warped_corners = cv2.perspectiveTransform(corners, matrix)
    
    # 3. 이동한 좌표들의 최소값과 최대값 구하기
    [x_min, y_min] = np.int32(warped_corners.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(warped_corners.max(axis=0).ravel() + 0.5)
    
    # 4. 음수 영역으로 나간 만큼 평행 이동(Translation)할 행렬 생성
    translation_dist = [-x_min if x_min < 0 else 0, -y_min if y_min < 0 else 0]
    translation_matrix = np.array([[1, 0, translation_dist[0]],
                                   [0, 1, translation_dist[1]],
                                   [0, 0, 1]], dtype=np.float32)
    
    # 5. 기존 matrix에 평행 이동 행렬 결합
    adjusted_matrix = translation_matrix.dot(matrix)
    
    # 6. 최종 출력 이미지의 크기 계산 (너비, 높이)
    output_width = x_max - x_min if x_min < 0 else max(x_max, w)
    output_height = y_max - y_min if y_min < 0 else max(y_max, h)
    
    # 7. 보정된 행렬과 크기로 warpPerspective 실행
    ff = cv2.warpPerspective(frame, adjusted_matrix, (output_width, output_height))
    
    return ff

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret: break

        yellow_pts = get_blobs(frame, HSV_YELLOW[0], HSV_YELLOW[1])
        
        # 노란색 점 시각화
        frame= draw_dots(frame, yellow_pts)

        #if len(yellow_pts) == 4:
        #    outer_pts, center_pts = convexHullDots(yellow_pts)

        #    if len(outer_pts) == 3 and len(center_pts) == 1:
        #        cv2.circle(frame, tuple(center_pts[0]), 9, (0, 0, 255), 2)
        #        center_pt = center_pts[0]
                
                
        #        matrix= getMatrix4(outer_pts, center_pt, R)
            
                
                # 파란색 물체 변환
        #        blue_pts = get_blobs(frame, HSV_BLUE[0], HSV_BLUE[1])
        #        for bp in blue_pts:
        #            x3d, y3d = get3Dpos(matrix, bp)
                    
        #            cv2.circle(frame, tuple(bp), 10, (255, 0, 0), -1)
        #           cv2.putText(frame, f"3D: ({x3d:.0f}, {y3d:.0f})", (bp[0]+10, bp[1]), 
       #                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
       
        if len(yellow_pts) == 4:
            matrix= getMatrix4(yellow_pts, R)
        
            
            # 파란색 물체 변환
            blue_pts = get_blobs(frame, HSV_BLUE[0], HSV_BLUE[1])
            for bp in blue_pts:
                x3d, y3d = get3Dpos(matrix, bp)
                
                cv2.circle(frame, tuple(bp), 10, (255, 0, 0), -1)
                cv2.putText(frame, f"3D: ({x3d:.0f}, {y3d:.0f})", (bp[0]+10, bp[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('Webcam', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()
