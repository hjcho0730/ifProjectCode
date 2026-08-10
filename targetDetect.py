import cv2
import mediapipe as mp

from matrixCal import *

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
        
        # BGR 이미지를 RGB로 변환
        frame = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        
        h,w,_= frame.shape
        
        # 프레임을 포즈 모델에 전달하여 포즈를 처리
        results = pose.process(frame)
        
        # RGB 이미지를 다시 BGR로 변환하여 OpenCV에서 사용
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        yellow_pts = get_blobs(frame, HSV_YELLOW[0], HSV_YELLOW[1])
        frame= draw_dots(frame, yellow_pts)
        if len(yellow_pts) == 4:
            outer_pts, center_pts = convexHullDots(yellow_pts)
            if len(outer_pts) == 3 and len(center_pts) == 1:
                cv2.circle(frame, tuple(center_pts[0]), 9, (0, 0, 255), 2)
                center_pt = center_pts[0]
                matrix= getMatrix(outer_pts, center_pt, R)
                matrix_inv = np.linalg.inv(matrix)
                
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
            
            f= (lambda idx: (results.pose_landmarks.landmark[idx].x * w, results.pose_landmarks.landmark[idx].y * h))
            
            
            arr= [f(11), f(23)]
            d= ((arr[0][0]+arr[1][0])/2, (arr[0][1]+arr[1][1])/2) 
            cv2.putText(frame, "right : " + str(d[0]) + " , " + str(d[1]), 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            blue_pts.append(d)
            blue_pts.append(arr[0])
            blue_pts.append(arr[1])
            
            if matrix is not None:
                x1, y1 = get3Dpos(matrix, arr[0])
                x2, y2 = get3Dpos(matrix, arr[1])
                x3, y3 = (x1+x2)/2, (y1+y2)/2
                x4, y4= -(y3-y1), (x3-x1)
                
                import math
                rrr= math.sqrt( (x4**2) + (y4**2) )
                x4 /= rrr
                y4 /= rrr
                x4 *= 50
                y4 *= 50
                
                d2= transform_to_screen(matrix_inv, (x4+x3, y4+y3))
                print(x4, y4 )
                blue_pts.append(d2)
            
        
        if matrix is not None:
            for bp in blue_pts:
                x3d, y3d = get3Dpos(matrix, bp)
                bp= tuple(map(int, bp))
                cv2.circle(frame, bp, 10, (255, 0, 0), -1)
                cv2.putText(frame, f"3D: ({x3d:.0f}, {y3d:.0f})", (bp[0]+10, bp[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            
            #cv2.circle(frame, d2, radius=5, color=(0, 0, 255), thickness=-1)
            
            
            


        # 결과 화면 출력
        cv2.imshow('Pose Detection', frame)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

# 웹캠을 닫고 모든 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()