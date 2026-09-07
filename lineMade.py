import heapq
import cv2
import numpy as np

import cv2
import numpy as np


def draw_robot_path(
    img, path, robot_start=None, points_list=None, color=(255, 255, 0), thickness=2
):
  """반환된 path 리스트를 OpenCV 이미지(img) 위에 시각화합니다."""
  vis_img = img.copy()

  if not path or len(path) < 2:
    print("표시할 경로가 없습니다.")
    return vis_img

  if robot_start == None:
    robot_start= path[0]

  # 1. 경로 선(Line) 그리기
  for i in range(len(path) - 1):
    pt1 = (int(round(path[i][0])), int(round(path[i][1])))
    pt2 = (int(round(path[i + 1][0])), int(round(path[i + 1][1])))
    cv2.line(vis_img, pt1, pt2, color, thickness)

  # 2. 경로 상의 중간 격자점 표시 (작은 흰색 점)
  for pt in path[1:-1]:
    cv2.circle(
        vis_img, (int(round(pt[0])), int(round(pt[1]))), 3, (255, 255, 255), -1
    )

  # 3. 로봇 시작 위치 표시 (파란색 원)
  start_pt = (int(round(robot_start[0])), int(round(robot_start[1])))
  cv2.circle(vis_img, start_pt, 8, (255, 0, 0), -1)
  cv2.putText(
      vis_img,
      "Robot",
      (start_pt[0] - 15, start_pt[1] + 20),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.5,
      (255, 0, 0),
      1,
  )

  # 4. 최종 목적지(심장 부근 후보점) 표시 (초록색 원)
  goal_pt = (int(round(path[-1][0])), int(round(path[-1][1])))
  cv2.circle(vis_img, goal_pt, 8, (0, 255, 0), -1)
  cv2.putText(
      vis_img,
      "Goal",
      (goal_pt[0] - 15, goal_pt[1] - 10),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.5,
      (0, 255, 0),
      1,
  )

  return vis_img
# 사용 예시:
# width, height = 600, 600
# base_img = np.zeros((height, width, 3), dtype=np.uint8) # 또는 장애물 마스크 기반 이미지
# robot_start = (43.7, 552.3)
# path = find_robot_path(points_list, connections, robot_start)
#
# result_img = draw_robot_path(base_img, path, robot_start)
# cv2.imshow("Robot Route", result_img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


def find_robot_path(
    points_list,
    connections,
    robot_start,
    width=600,
    height=600,
    grid_step=20,
    padding_radius=25,
):
  """0~32 번호의 점들과 연결 정보를 받아 회전이 적고 최단인 경로의 정점 리스트를 반환합니다."""
  obstacle_mask = np.zeros((height, width), dtype=np.uint8)

  # 1 & 3. 점과 연결 정보를 바탕으로 패딩이 적용된 장애물 영역 생성
  for p1_idx, p2_idx in connections:
    if p1_idx < len(points_list) and p2_idx < len(points_list):
      pt1 = (int(points_list[p1_idx][0]), int(points_list[p1_idx][1]))
      pt2 = (int(points_list[p2_idx][0]), int(points_list[p2_idx][1]))
      cv2.line(obstacle_mask, pt1, pt2, 255, thickness=padding_radius * 2)

  for pt in points_list:
    cv2.circle(obstacle_mask, (int(pt[0]), int(pt[1])), padding_radius, 255, -1)

  # 4. 격자형 점 생성 및 패딩 영역에 겹치는 점 제거
  valid_points = set()
  for y in range(0, height, grid_step):
    for x in range(0, width, grid_step):
      if obstacle_mask[y, x] == 0:
        valid_points.add((float(x), float(y)))

  # 실수형 로봇 시작점 추가
  sx, sy = int(round(robot_start[0])), int(round(robot_start[1]))
  if 0 <= sx < width and 0 <= sy < height and obstacle_mask[sy, sx] == 0:
    valid_points.add(robot_start)

  # 5. 심장 부근(어깨 중앙 기준) 위치 추정 및 후보점 지정
  if len(points_list) >= 13:
    # MediaPipe 기준 어깨인덱스(11, 12) 활용 혹은 중앙 위치 계산
    heart_pt = (
        (points_list[11][0] + points_list[12][0]) / 2.0,
        (points_list[11][1] + points_list[12][1]) / 2.0 + 40.0,
    )
  else:
    heart_pt = np.mean(points_list, axis=0)

  min_dist, max_dist = 50, 160
  candidates = []
  for pt in valid_points:
    if pt == robot_start:
      continue
    dist = np.linalg.norm(np.array(pt) - np.array(heart_pt))
    if min_dist <= dist <= max_dist:
      candidates.append(pt)

  if not candidates:
    return []

  # 6. 회전 가중치가 반영된 다익스트라 길찾기
  def dijkstra_smooth_path(start, goal, valid_set, step):
    pq = [(0, start[0], start[1], 0, 0)]
    best_cost = {}
    parent = {}

    moves = [
        (step, 0, 10),
        (-step, 0, 10),
        (0, step, 10),
        (0, -step, 10),
        (step, step, 14),
        (-step, step, 14),
        (step, -step, 14),
        (-step, -step, 14),
    ]

    while pq:
      cost, x, y, pdx, pdy = heapq.heappop(pq)
      curr = (x, y)

      if curr == goal:
        path = []
        state = (x, y, pdx, pdy)
        while state in parent:
          path.append((state[0], state[1]))
          state = parent[state]
        path.append(start)
        return path[::-1]

      state_key = (x, y, pdx, pdy)
      if state_key in best_cost and best_cost[state_key] < cost:
        continue

      if curr == start:
        for vx, vy in valid_set:
          if vx != start[0] or vy != start[1]:
            dist_to_v = np.hypot(vx - start[0], vy - start[1])
            if dist_to_v <= step * 1.5:
              new_cost = cost + dist_to_v
              next_state_key = (vx, vy, vx - start[0], vy - start[1])
              if (
                  next_state_key not in best_cost
                  or new_cost < best_cost[next_state_key]
              ):
                best_cost[next_state_key] = new_cost
                parent[next_state_key] = state_key
                heapq.heappush(
                    pq, (new_cost, vx, vy, vx - start[0], vy - start[1])
                )
        continue

      for dx, dy, base_cost in moves:
        nx, ny = x + dx, y + dy
        if (float(nx), float(ny)) in valid_set or (nx, ny) == goal:
          turn_penalty = (
              80
              if (pdx != 0 or pdy != 0) and (dx, dy) != (pdx, pdy)
              else 0
          )
          new_cost = cost + base_cost + turn_penalty
          next_state_key = (nx, ny, dx, dy)

          if (
              next_state_key not in best_cost
              or new_cost < best_cost[next_state_key]
          ):
            best_cost[next_state_key] = new_cost
            parent[next_state_key] = state_key
            heapq.heappush(pq, (new_cost, nx, ny, dx, dy))

    return []

  best_path = []
  min_cost_val = float('inf')

  for cand in candidates:
    path = dijkstra_smooth_path(robot_start, cand, valid_points, grid_step)
    if path:
      turns = 0
      for i in range(2, len(path)):
        dx1 = path[i - 1][0] - path[i - 2][0]
        dy1 = path[i - 1][1] - path[i - 2][1]
        dx2 = path[i][0] - path[i - 1][0]
        dy2 = path[i][1] - path[i - 1][1]
        if (dx1, dy1) != (dx2, dy2):
          turns += 1

      total_score = len(path) * 10 + turns * 80
      if total_score < min_cost_val:
        min_cost_val = total_score
        best_path = path

  return best_path

import cv2
import numpy as np

def get_transformed_screen_vertices(matrix, screen_width, screen_height):
    """
    변환 행렬과 스크린(이미지)의 가로/세로 크기를 이용해 
    변환된 좌표계에서의 스크린 4꼭짓점 위치와 최소/최대 바운딩 박스를 계산합니다.
    """
    # 스크린의 4개 모서리 좌표 (좌상, 우상, 우하, 좌하)
    screen_corners = np.array([
        [[0, 0]],
        [[screen_width, 0]],
        [[screen_width, screen_height]],
        [[0, screen_height]]
    ], dtype=np.float32)
    
    # 1. 원근 변환 적용 (스크린 좌표 -> 변환된 좌표계)
    transformed_pts = cv2.perspectiveTransform(screen_corners, matrix)
    transformed_pts = transformed_pts.reshape(-1, 2)  # shape: (4, 2)
    
    # 2. 변환된 좌표계에서의 최소/최대 범위 계산
    min_x = np.min(transformed_pts[:, 0])
    max_x = np.max(transformed_pts[:, 0])
    min_y = np.min(transformed_pts[:, 1])
    max_y = np.max(transformed_pts[:, 1])
    
    # 3. 최소/최대 지점을 기준으로 하는 바운딩 박스 꼭짓점 생성 (좌상, 우상, 우하, 좌하)
    bounding_box = np.array([
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y]
    ], dtype=np.float32)
    
    return {
        "transformed_corners": transformed_pts,  # 변환된 실제 모서리 4개 점
        "bounding_box": bounding_box,            # 최소/최대 외곽 사각형 4개 꼭짓점
        "min_max_limits": (min_x, min_y, max_x, max_y) # (min_x, min_y, max_x, max_y)
    }
