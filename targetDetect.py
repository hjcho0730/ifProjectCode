import cv2
import os
import mediapipe as mp

import numpy as np

from matrixCal import *
from lineMade import *

current_path = os.getcwd()
file_path = os.path.join(current_path, "my_list.txt")

# 1. 아루코 사전 및 검출기 설정
dictionary_id=cv2.aruco.DICT_4X4_50
aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
def get_single_marker_corners_list(
    image, target_id=0,
):
  """특정 ID의 아루코 마커 하나를 검출하고 4개의 모서리 픽셀 좌표를 파이썬 리스트로 반환하는 함수

  Returns:
      tuple: (success (bool), corners_list (list or None), debug_image (np.ndarray))
             corners_list는 [[x0, y0], [x1, y1], [x2, y2], [x3, y3]] 형태의 리스트
  """
  debug_image = image.copy()

  

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

matrix= None  # 2D -> 3D
matrix_inv= None  # 3D -> 2D
bodyPoints= [] #2D 스크린 좌표
route= [] #예정 경로
# 포즈 모델 사용
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        
        print(3)
        success, frame = cap.read()
        if not success:
            print("카메라로부터 영상을 가져올 수 없습니다.")
            continue
        suc, markPts, dImage = get_single_marker_corners_list(frame, target_id=0)
        frame= dImage

        h,w,_= frame.shape

        #frame= cv2.flip(frame, 1)

        # BGR 이미지를 RGB로 변환
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) # 프레임을 포즈 모델에 전달하여 포즈를 처리

        ########### 변환 행렬 계산 ###################
        if suc:
            frame= draw_dots(frame, markPts)
            if len(markPts) == 4:
                matrix= getMatrix4(markPts, R)
                try:
                    matrix_inv = np.linalg.inv(matrix)
                except Exception as e:
                    print(f"Error: {e}")
        ############################################
        print(4)

                
        # 포즈 랜드마크가 감지되면 랜드마크와 연결선 그리기
        blue_pts= []
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                image=frame,
                landmark_list=results.pose_landmarks,
                connections=mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
            )

            ####### 신체 형상 데이터 -> 파일 저장 ############
            # print()
            # with open(file_path, "w", encoding="utf-8") as f:
            #     for item in results.pose_landmarks.landmark:
            #         f.write(f"{item}\n")
            # print(f"파일이 다음 경로에 저장되었습니다: {file_path}")
            ###############포즈 모델 값 저장##################
            f= (lambda idx: (results.pose_landmarks.landmark[idx].x * w, results.pose_landmarks.landmark[idx].y * h))

            bodyPoints= [f(kkkk) for kkkk in range(33)]
            ##############################################

        '''
            로봇 감지 코드
            
            반환될 변수:
            robotDetected(bool)
            robotPos(tuple)
        '''
        
        print(5)

        if len(bodyPoints) != 0:
            ###############옆구리 계산 및 표시 ##################
            arr= [bodyPoints[11], bodyPoints[23]]
            d= (int(arr[0][0]+arr[1][0])//2, int(arr[0][1]+arr[1][1])//2)
            cv2.circle(frame, d, 10, (255, 125, 0), -1)
            ##################################################
            
            if matrix is not None and True:#robotDetected:
                tmpPList= [get3Dpos(matrix, bodyPoints[kkkk]) for kkkk in range(33)]
                ressss= get_transformed_screen_vertices(matrix, w, h)
                lx= int(abs(ressss['min_max_limits'][0]-ressss['min_max_limits'][2]))
                ly= int(abs(ressss['min_max_limits'][1]-ressss['min_max_limits'][3]))
                d1= ((tmpPList[11][0]+tmpPList[12][0])/2, (tmpPList[11][1]+tmpPList[12][1])/2)
                d2= ((tmpPList[23][0]+tmpPList[24][0])/2, (tmpPList[23][1]+tmpPList[24][1])/2)
                d3= ((d1[0]+d2[0]*2) / 3, (d1[1]+d2[1]*2) / 3)
                lx,ly= d3[0]+750,d3[1]+300
                print("start")
                route= find_robot_path(tmpPList, mp_pose.POSE_CONNECTIONS, (lx,ly), d3, 75,lx, ly, 25, 30) #robotPos
                print("end")
                #print(route)
        if len(route) != 0 and True:#robotDetected:
            pass
            # 대충 로봇 조종
        
        print(6)
        

        # 결과 화면 출력
        cv2.imshow('Pose Detection1', frame)
        try:
            if matrix is not None and matrix_inv is not None and True:
                if len(route) != 0:
                   print(len(route), 0)
                   aaRoute= [transform_to_screen(matrix_inv, pptt) for pptt in route]
                   print(len(route), 1)
                   frame = draw_robot_path(frame, aaRoute) 
                   print(len(route), 2)
                ##ff= warp_perspective_no_crop(frame, matrix)
                
                #ff = cv2.resize(ff, (w, h))
                cv2.imshow('Pose Detection2', frame)
        except Exception as e:
            print(f"Error: {e}")
        
        # 'q'를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

# 웹캠을 닫고 모든 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()


