# 카메라 번호 0, 1..
# carMovable = True
# comport 지정
# arm_use = True
camera_num=1 #카메라는 몇번 사용?
carMovable= True #자동차 쓸거임?
bluetooth_port = 'COM5' #블루투스 송신 포트는?
arm_use= False #팔 쓸거임?
targetAruco= 1 # 몇개로 판별할 거야? (지금 지원중인 것: [1, 2], 이외의 값은 1로 취급)

### x,y,n(한 변 길이)로 설정 예정. 꼭!!!!!!!!!!!! 설정하기
offset1= (130, 30, 120) 
offset2= (120, 35, 120) 
####################################################

arrived_times= 0
arrived_times_sec= 5 #도착한 후 팔을 내릴 때까지 걸릴 시간. (회전 시간 포함임)

is_arm= False
arm_time= 0
arm_wait_time= 24 # 팔의 움직임을 기다릴 시간.

import cv2, time
#import os
import mediapipe as mp
import numpy as np

from matrixCal import *
from lineMade import *

if carMovable:
    import serial

    # 아까 확인한 내 컴퓨터의 COM 번호로 꼭 바꿔주세요! (예: 'COM5')
    print("블루투스 자동차에 연결하는 중...")
    # 자동차와 블루투스 전화 연결!
    car = serial.Serial(bluetooth_port, 9600)
    time.sleep(2) # 연결될 때까지 2초 정도 기다려줍니다.

    def turn(dire):
        try:
            if dire == "CW":
                car.write(b"left\n")
            elif dire == "CCW":
                car.write(b"right\n")
        except Exception as e:
            print(f"Serial write error: {e}")
            
    def stop():
        try:
            car.write(b'stop\n')
        except Exception as e:
            print(f"Serial write error: {e}")
            
    def move():
        try:
            car.write(b'go\n')
        except Exception as e:
            print(f"Serial write error: {e}")
            
    def arm():
        try:
            car.write(b'arm\n')
        except Exception as e:
            print(f"Serial write error: {e}")

#current_path = os.getcwd()
#file_path = os.path.join(current_path, "my_list.txt")

def trimmed_mean(values):
    # 프레임이 2개 이하일 때는 이상치 제거 없이 전체 평균 반환
    if len(values) <= 2:
        return sum(values) / len(values)
    return (sum(values) - max(values) - min(values)) / (len(values) - 2)

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
    #print(marker_id)
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
cap = cv2.VideoCapture(camera_num)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

matrix= None  # 2D -> 3D
matrix_inv= None  # 3D -> 2D
bodyPoints= [] #2D 스크린 좌표
route= [] #예정 경로
# 포즈 모델 사용

planeDots= []
sceneFrame= 0 # 0 ~ 99

pos= (0,0)
d3= (0,0)

prev_route_cal_time= 0

center= (0,0)
forward= (0,0)
forwardPos= (0,0)
RobotDetected= False

fps_cal_second= 0.0

robot_detect_frame= 0
RobotNotDetectedLim= 4

is_arrived= False

fps_cal_second= time.time()-1
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        if time.time()-fps_cal_second <= 1e-5:
            print("inf")
        else:
            print(round(1/(time.time()-fps_cal_second), 2))
            if time.time()-fps_cal_second < 0.03:
                time.sleep(0.04 - (time.time()-fps_cal_second) ) # 아두이노의 통신 버퍼가 차는 것을 방지하기 위함. 통신이 너무 오래 안 가면 자동 제동이 있으니 최소한만 기다리기.
        fps_cal_second= time.time()
        
        success, frame = cap.read()
        if not success:
            print("카메라로부터 영상을 가져올 수 없습니다.")
            continue
        
        suc, markPts, dImageA = get_single_marker_corners_list(frame, target_id=0)
        
        if suc:
            planeDots.append(markPts)
            if len(planeDots) > 8:
                planeDots.pop(0)
        
        #frame= dImage

        h,w,_= frame.shape

        #frame= cv2.flip(frame, 1)

        # BGR 이미지를 RGB로 변환
        results = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) # 프레임을 포즈 모델에 전달하여 포즈를 처리

        ########### 변환 행렬 계산 ###################
        if suc:
            AverDots = [
                [
                    trimmed_mean([j[i][k] for j in planeDots])
                    for k in range(2)
                ]
                for i in range(4)
            ]            #print(markPts)
            #print(AverDots)
            frame= draw_dots(frame, AverDots)
            if len(markPts) == 4:
                matrix= getMatrix4(AverDots, R)
                try:
                    matrix_inv = np.linalg.inv(matrix)
                except Exception as e:
                    print(f"Error: {e}")
        ############################################
        
        
        if targetAruco == 2:
            suc1, markPts1, dImage = get_single_marker_corners_list(frame, target_id=1)
            suc2, markPts2, dImage = get_single_marker_corners_list(frame, target_id=2)
            
            ################## TODO!!!! 아루코 마커 2개로 감지하기... ########################
            #i think... offset 1/2 를 저장하기. -> 실행 중에는, 1,2의 좌표와 포워드를 행렬 변환 적용해 얻은 3d 좌표에 offset 만큼 더하고 빼서 중심 구하기.
            # pos, forward를 3d로 저장하고 있으니 바로 대입하면 됨 ㅇㅇ
            # 그리고, 둘 중 하나만 추출되면 하나만으로 오프셋해서 구하고, 둘다 있으면 둘다 써서 구하기. 고민 중인건, 둘 다 있을 때 둘 다 쓰면 렉이 걱정됨
            # -> 하나에 우선순위 적용해서 하나씩만 계산하게 하면 될 듯함. 이건 일단 나중에.
            # center, forward 구하지 말고, 바로 여기서(targetAruco=1 포함) pos, forwardPos로 만들어서 내보내자. ㅇㅇ
            
            RobotDetected=False
            if suc1 or suc2:
                RobotDetected= True
                
                if suc1:
                    center = np.mean(markPts1, axis=0)
                    forward= np.mean(markPts1[:2], axis=0)
                    right= np.mean(markPts1[1:3], axis=0)
                else:
                    center = np.mean(markPts2, axis=0)
                    forward= np.mean(markPts2[:2], axis=0)
                    right= np.mean(markPts2[1:3], axis=0)
                    
                cv2.circle(dImageA, tuple(int(i) for i in center), 10, (255, 125, 0), -1)
                cv2.circle(dImageA, tuple(int(i) for i in forward), 10, (255, 125, 0), -1)
                #cv2.circle(dImageA, tuple(int(i) for i in right), 10, (255, 125, 0), -1)
                    
                if matrix is not None:
                    if suc1:
                        pos1= get3Dpos(matrix, center)
                        forwardPos1= get3Dpos(matrix, forward)
                        right1= get3Dpos(matrix, right)
                        
                        wWw= (((forwardPos1[0]-pos1[0]) / offset1[2]) * 2, ((forwardPos1[1]-pos1[1]) / offset1[2]) * 2)
                        rRr= (((right1[0]-pos1[0]) / offset1[2]) * 2, ((right1[1]-pos1[1]) / offset1[2]) * 2)
                        
                        pos= (pos1[0] + rRr[0] * offset1[0] - wWw[0] * offset1[1], pos1[1] + rRr[1] * offset1[0] - wWw[1] * offset1[1])                        
                        forwardPos= (pos[0] + wWw[0] * 4, pos[1] + wWw[1] * 4)
                    else:
                        pos2= get3Dpos(matrix, center)
                        forwardPos2= get3Dpos(matrix, forward)
                        right2= get3Dpos(matrix, right)
                        
                        wWw= (((forwardPos2[0]-pos2[0]) / offset2[2]) * 2, ((forwardPos2[1]-pos2[1]) / offset2[2]) * 2)
                        rRr= (((right2[0]-pos2[0]) / offset2[2]) * 2, ((right2[1]-pos2[1]) / offset2[2]) * 2)
                        
                        pos= (pos2[0] - rRr[0] * offset2[0] - wWw[0] * offset2[1], pos2[1] - rRr[1] * offset2[0] - wWw[1] * offset2[1])                        
                        forwardPos= (pos[0] + wWw[0] * 4, pos[1] + wWw[1] * 4)

            #################################################################################
        else: # 1, etc(for except)
            suc, markPts, dImage = get_single_marker_corners_list(frame, target_id=1)
        
            RobotDetected=False
            if suc:    
                center = np.mean(markPts, axis=0)
                
                forward= np.mean(markPts[:2], axis=0)
                
                cv2.circle(dImageA, tuple(int(i) for i in center), 10, (255, 125, 0), -1)
                cv2.circle(dImageA, tuple(int(i) for i in forward), 10, (255, 125, 0), -1)
                
                RobotDetected= True
            
            
            if matrix is not None and RobotDetected:
                pos= get3Dpos(matrix, center)
                forwardPos= get3Dpos(matrix, forward)
                '''                
                ppppp= transform_to_screen(matrix_inv, pos)
                ppppp= (int(ppppp[0]), int(ppppp[1]))
                cv2.circle(dImageA, ppppp, 10, (59, 125, 0), -1)
                ppppp= transform_to_screen(matrix_inv, forwardPos)
                ppppp= (int(ppppp[0]), int(ppppp[1]))
                cv2.circle(dImageA, ppppp, 10, (0, 30, 255), -1)
                '''
                
        frame= dImageA
                
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
        
        if len(bodyPoints) != 0 and matrix is not None:
            tmpPList= [get3Dpos(matrix, bodyPoints[kkkk]) for kkkk in range(33)]
            d1= ((tmpPList[11][0]+tmpPList[12][0])/2, (tmpPList[11][1]+tmpPList[12][1])/2)
            d2= ((tmpPList[23][0]+tmpPList[24][0])/2, (tmpPList[23][1]+tmpPList[24][1])/2)
            d3= ((d1[0]*2+d2[0]) / 3, (d1[1]*2+d2[1]) / 3)
            if matrix_inv is not None:
                cv2.circle(frame, tuple(map(int, transform_to_screen(matrix_inv, d3))), 10, (255, 125, 0), -1)
            if (len(route) <= 1 or (time.time() - prev_route_cal_time) > 0.5) and RobotDetected:
                
                
                #pos= get3Dpos(matrix, center)
                
                extra_padding = {
                    frozenset({6, 8}): 65,
                    frozenset({4, 5}): 65,
                    frozenset({5, 6}): 65,
                    frozenset({0, 4}): 65,
                    frozenset({0, 1}): 65,
                    frozenset({1, 2}): 65,
                    frozenset({2, 3}): 65,
                    frozenset({3, 7}): 65,
                    frozenset({9, 10}): 65,
                }
                
                route= find_robot_path(tmpPList, mp_pose.POSE_CONNECTIONS, pos, d3, 600, padding_radius=350, extra_padding_by_connection=extra_padding) #robotPos
                #print(route)
                if len(route) >= 2:
                    prev_route_cal_time= time.time()
                
                #display_robot_path(
                #    route,
                #    points_list=tmpPList,
                #    connections=mp_pose.POSE_CONNECTIONS,
                #    save_path="robot_path.png",
                #    image_size=(12, 9),
                #)
            
        if carMovable:
            if not (matrix is not None):
                stop()
                print("Mat stop")
                is_arrived= False
            else:
                if not RobotDetected:
                    robot_detect_frame += 1
                else:
                    robot_detect_frame=0
                if robot_detect_frame > RobotNotDetectedLim:
                    stop()
                    print("Detect stop")
                    is_arrived= False
                    robot_detect_frame= RobotNotDetectedLim+1
                else:
                    if len(bodyPoints) == 0:
                        stop()
                        is_arrived= False
                        print("no body stop")
                    else:
                        #print(pos, d3)
                        
                        if arm_use and ((is_arrived and time.time()-arrived_times > arrived_times_sec) or is_arm):
                            if not is_arm:
                                is_arm= True
                                arm_time= time.time()
                                arm()
                            else:
                                if time.time()-arm_time > arm_wait_time:
                                    is_arm= False
                                    is_arrived= False
                            time.sleep(0.1)
                        else:
                            if len(route) >= 2:
                                if np.hypot(pos[0]-route[1][0], pos[1]-route[1][1]) < 50:
                                    route.pop(1)
                            
                            if len(route) <= 1: ## 도착했거나, 도착 직전
                                direction, angle= calculate_rotation(pos, forwardPos, d3)
                                if np.hypot(pos[0]-d3[0], pos[1]-d3[1]) < 780 + (30 if is_arrived else 0):
                                    print(arrived_times, is_arrived, 1000)
                                    if not is_arrived:
                                        arrived_times= time.time()
                                    is_arrived= True
                                    if angle > 15:
                                        turn(direction)
                                        print('angle-arrive', angle)
                                    else:
                                        print("arrive!!!!")
                                        stop()
                                                   
                                    print("close stop", np.hypot(pos[0]-d3[0], pos[1]-d3[1]))
                                else:
                                    is_arrived= False
                                    if angle > 15:
                                        turn(direction)
                                        print('angle-line', angle)
                                    else:
                                        move()
                                        print("line")
                            else:
                                direction, angle= calculate_rotation(pos, forwardPos, route[1])
                            
                                if angle > 15:
                                    turn(direction)
                                    print('angle', angle)
                                else:
                                    if (not is_arrived) or (is_arrived and np.hypot(pos[0]-d3[0], pos[1]-d3[1]) > 620):
                                        is_arrived= False  
                                        print('f', angle)
                                        move()
                                    else:
                                        print("arrived, but there is route... stop")
                                        stop()

        # 결과 화면 출력
        cv2.imshow('Pose Detection1', frame)
        try:
            if matrix is not None and matrix_inv is not None:
                #if len(route) != 0:
                #   aaRoute= [transform_to_screen(matrix_inv, pptt) for pptt in route]
                #   frame = draw_robot_path(frame, aaRoute) 
                warped_frame= warp_perspective_no_crop(frame, matrix)
                
                warped_frame = cv2.resize(warped_frame, (w, h))
                cv2.imshow('Pose Detection2', warped_frame)
        except Exception as e:
            print(f"Error: {e}")
        
        # 'q'를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

# 웹캠을 닫고 모든 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()

if carMovable:
    car.close()




