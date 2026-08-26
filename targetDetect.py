import cv2
import os
import mediapipe as mp

from matrixCal import *

current_path = os.getcwd()
file_path = os.path.join(current_path, "my_list.txt")

def get_single_marker_corners_list(
    image, target_id=0, dictionary_id=cv2.aruco.DICT_4X4_50
):
  """특정 ID의 아루코 마커 하나를 검출하고 4개의 모서리 픽셀 좌표를 파이썬 리스트로 반환하는 함수

  Returns:
      tuple: (success (bool), corners_list (list or None), debug_image (np.ndarray))
             corners_list는 [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] 형태의 리스트
  """
  debug_image = image.copy()

  # 1. 아루코 사전 및 검출기 설정
  aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
  parameters = cv2.aruco.DetectorParameters()
  detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

  # 2. 마커 검출
  corners, ids, rejected = detector.detectMarkers(image)

  if ids is None:
    cv2.putText(
        debug_image,
        f"Marker ID {target_id} not found!",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
    )
    return False, None, debug_image

  ids = ids.flatten()

  # 3. 타겟 ID 찾기
  target_idx = -1
  for i, marker_id in enumerate(ids):
    if marker_id == target_id:
      target_idx = i
      break

  if target_idx == -1:
    cv2.putText(
        debug_image,
        f"Marker ID {target_id} not found!",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2,
    )
    return False, None, debug_image

  # 4. 4개 모서리 추출 후 파이썬 리스트로 변환
  # corners[target_idx][0] shape: (4, 2)
  marker_corners = corners[target_idx][0]
  corners_list = marker_corners.tolist()  # numpy array를 파이썬 리스트로 변환

  # 5. 디버그용 시각화
  cv2.aruco.drawDetectedMarkers(
      debug_image, [corners[target_idx]], np.array([target_id])
  )

  for idx, pt in enumerate(marker_corners):
    pt_int = (int(pt[0]), int(pt[1]))
    cv2.circle(debug_image, pt_int, 4, (0, 255, 0), -1)
    cv2.putText(
        debug_image,
        str(idx),
        (pt_int[0] + 5, pt_int[1] - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 0, 0),
        2,
    )

  return True, corners_list, debug_image

# 미디어파이프 포즈 모델 로드
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# 웹캠을 열어 실시간으로 영상을 가져옵니다.
cap = cv2.VideoCapture(0)

matrix= None
matrix_inv= None
# 포즈 모델 사용
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("카메라로부터 영상을 가져올 수 없습니다.")
            continue
        suc, markPts, dImage = get_single_marker_corners_list(frame, target_id=0)
        #frame= cv2.flip(frame, 1)
        frame= dImage
        # BGR 이미지를 RGB로 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        h,w,_= frame.shape
        
        # 프레임을 포즈 모델에 전달하여 포즈를 처리
        results = pose.process(frame)
        
        # RGB 이미지를 다시 BGR로 변환하여 OpenCV에서 사용
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        if suc:
            frame= draw_dots(frame, markPts)
            if len(markPts) == 4:
                matrix= getMatrix4(markPts, R)
                matrix_inv = np.linalg.inv(matrix)
                
        # 포즈 랜드마크가 감지되면 랜드마크와 연결선 그리기
        blue_pts= []
        if results.pose_landmarks:
            print(type(results.pose_landmarks))
            mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=results.pose_landmarks,
                connections=mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
            )
            print()
            with open(file_path, "w", encoding="utf-8") as f:
                for item in results.pose_landmarks.landmark:
                    f.write(f"{item}\n")
            print(f"파일이 다음 경로에 저장되었습니다: {file_path}")
            f= (lambda idx: (results.pose_landmarks.landmark[idx].x * w, results.pose_landmarks.landmark[idx].y * h))
            
            
            arr= [f(11), f(23)]
            d= ((arr[0][0]+arr[1][0])/2, (arr[0][1]+arr[1][1])/2) 
            cv2.putText(frame, "right : " + str(d[0]) + " , " + str(d[1]), 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            #blue_pts.append(d)
            #blue_pts.append(arr[0])
            #blue_pts.append(arr[1])
            
            import math
            if matrix is not None:
                x1, y1 = get3Dpos(matrix, arr[0])
                x2, y2 = get3Dpos(matrix, arr[1])
                x3, y3 = (x1+x2)/2, (y1+y2)/2
                
                blue_pts.append(transform_to_screen(matrix_inv, (x1,y1)))
                blue_pts.append(transform_to_screen(matrix_inv, (x2,y2)))
                blue_pts.append(transform_to_screen(matrix_inv, (x3,y3)))
                
                for i in range(360):
                    x4, y4= math.cos((i*math.pi)/180) * 500, math.sin((i*math.pi)/180) * 500
                    d2= transform_to_screen(matrix_inv, (x4+x3, y4+y3))
                    blue_pts.append(d2)
            
        
        if matrix is not None:
            for bp in blue_pts:
                x3d, y3d = get3Dpos(matrix, bp)
                bp= tuple(map(int, bp))
                cv2.circle(frame, bp, 10, (255, 0, 0), -1)
                #cv2.putText(frame, f"3D: ({x3d:.0f}, {y3d:.0f})", (bp[0]+10, bp[1]), 
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            
            #cv2.circle(frame, d2, radius=5, color=(0, 0, 255), thickness=-1)
            
            

        # 결과 화면 출력
        cv2.imshow('Pose Detection1', frame)
        try:
            if matrix is not None:
                ff= warp_perspective_no_crop(frame, matrix)
                cv2.imshow('Pose Detection2', ff)
        except:
            print("e")
        
        # 'q'를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

# 웹캠을 닫고 모든 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()

