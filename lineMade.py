import heapq
import cv2
import numpy as np

import math

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


_SQRT2 = math.sqrt(2.0)


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

### 2 ver

def calculate_rotation(pos, forward_pos, target_pos):
    """
    pos: (x, y) - 현재 위치
    forward_pos: (x, y) - 현재 바라보는 방향 좌표
    target_pos: (x, y) - 이동해야 할 다음 위치 (route[1])
    
    returns: (direction, angle_degrees)
             direction: 'CW' (시계 방향) 또는 'CCW' (반시계 방향)
             angle_degrees: 회전해야 하는 각도 (0 ~ 180도)
    """
    # 1. 현재 바라보는 방향 벡터 (V_forward)
    v_f_x = forward_pos[0] - pos[0]
    v_f_y = forward_pos[1] - pos[1]
    
    # 2. 목표 지점을 향하는 벡터 (V_target)
    v_t_x = target_pos[0] - pos[0]
    v_t_y = target_pos[1] - pos[1]
    
    # 3. 벡터 크기 (Magnitude) 계산
    mag_f = math.hypot(v_f_x, v_f_y)
    mag_t = math.hypot(v_t_x, v_t_y)
    
    # 예외 처리: 위치가 같아 방향을 정의할 수 없는 경우
    if mag_f == 0 or mag_t == 0:
        return 'CW', 0.0
    
    # 4. 내적(Dot Product)을 이용한 각도 계산
    dot_product = v_f_x * v_t_x + v_f_y * v_t_y
    # 부동소수점 오차로 인해 cos_theta가 [-1.0, 1.0] 범위를 벗어나는 것 방지
    cos_theta = max(-1.0, min(1.0, dot_product / (mag_f * mag_t)))
    
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    
    # 5. 외적(Cross Product) 2D z성분을 이용한 회전 방향 결정
    # cross_product > 0: 반시계 방향(CCW)
    # cross_product < 0: 시계 방향(CW)
    cross_product = v_f_x * v_t_y - v_f_y * v_t_x
    
    if cross_product < 0:
        direction = 'CCW'   # 반시계 방향
    else:
        direction = 'CW'  # 시계 방향
        
    return direction, round(angle_deg, 2)


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
    target_pos,
    radius,
    padding_radius=25,
    extra_padding_by_connection=None,
):
    """
    패딩이 적용된 연속 공간 기반 경로 탐색.

    - 장애물 선분 주변 padding_radius만큼 안전 영역 적용
    - Visibility Graph + A* 기반 경로 탐색
    - 잠재 도착지를 출발점에서 가까운 순서로 탐색
    - 도달 가능한 가장 가까운 후보를 찾으면 조기 종료
    - 경로 스무딩을 통해 불필요한 경유점 제거
    """

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def dist2(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    def point_segment_distance(p, a, b):
        px, py = p
        ax, ay = a
        bx, by = b

        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy

        if length2 < 1e-12:
            return dist(p, a)

        t = ((px - ax) * dx + (py - ay) * dy) / length2
        t = max(0.0, min(1.0, t))

        proj = (ax + t * dx, ay + t * dy)
        return dist(p, proj)

    robot_start = tuple(robot_start)
    target_pos = tuple(target_pos)

    # 1. 장애물 및 우회 노드 생성
    obstacle_capsules = []
    nav_nodes = {robot_start}

    if extra_padding_by_connection is None:
        extra_padding_by_connection = {}

    n_points = len(points_list)

    for conn in connections:
        # connections의 원소는 frozenset({i, j})
        if len(conn) != 2:
            continue

        i, j = tuple(conn)

        if not (0 <= i < n_points and 0 <= j < n_points):
            continue

        p1 = tuple(points_list[i])
        p2 = tuple(points_list[j])

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)

        if length < 1e-6:
            continue

        # 해당 장애물에 적용할 최종 패딩
        connection_key = frozenset({i, j})
        extra_padding = extra_padding_by_connection.get(
            connection_key, 0
        )
        obstacle_padding = max(
            0, padding_radius + extra_padding
        )

        ux, uy = dx / length, dy / length

        nx = -uy * obstacle_padding
        ny = ux * obstacle_padding
        px = ux * obstacle_padding
        py = uy * obstacle_padding

        c1 = (
            p1[0] - px + nx,
            p1[1] - py + ny
        )
        c2 = (
            p2[0] + px + nx,
            p2[1] + py + ny
        )
        c3 = (
            p2[0] + px - nx,
            p2[1] + py - ny
        )
        c4 = (
            p1[0] - px - nx,
            p1[1] - py - ny
        )

        obstacle_capsules.append({
            "p1": p1,
            "p2": p2,
            "padding": obstacle_padding,
            "connection": connection_key,
        })

        nav_nodes.update([c1, c2, c3, c4])

    # 2. 안전 영역 충돌 검사
    def is_blocked(p):
        for cap in obstacle_capsules:
            if point_segment_distance(
                p, cap["p1"], cap["p2"]
            ) < cap["padding"] - 1e-5:
                return True

        return False

    # 3. 두 점 사이 직선 이동 가능 여부 검사
    def is_line_clear(a, b):
        d = dist(a, b)

        if d < 1e-6:
            return not is_blocked(a)

        # 간격이 지나치게 커지지 않도록 샘플링
        sample_spacing = max(
            1.0,
            padding_radius * 0.25
        )
        steps = max(2, math.ceil(d / sample_spacing))

        dx = (b[0] - a[0]) / steps
        dy = (b[1] - a[1]) / steps

        for k in range(steps + 1):
            chk = (
                a[0] + dx * k,
                a[1] + dy * k
            )

            if is_blocked(chk):
                return False

        return True

    # 4. 잠재 도착지 생성
    target_candidates = []

    if not is_blocked(target_pos):
        target_candidates.append(target_pos)

    num_samples = 12

    for k in range(num_samples):
        angle = 2 * math.pi * k / num_samples

        cand = (
            target_pos[0] + radius * math.cos(angle),
            target_pos[1] + radius * math.sin(angle)
        )

        if not is_blocked(cand):
            target_candidates.append(cand)

    # 후보가 없으면 원래 목표점을 마지막 대안으로 사용
    if not target_candidates:
        target_candidates = [target_pos]

    # 중복 후보 제거
    target_candidates = list(dict.fromkeys(target_candidates))

    # 5. 유효한 우회 노드만 유지
    valid_nav_nodes = [
        node for node in nav_nodes
        if not is_blocked(node)
    ]

    def solve_astar(start, goal):
        if is_blocked(start) or is_blocked(goal):
            return None, math.inf

        if is_line_clear(start, goal):
            return [start, goal], dist(start, goal)

        nodes = list(valid_nav_nodes)

        if goal not in nodes:
            nodes.append(goal)

        open_set = [
            (dist(start, goal), 0.0, start, [start])
        ]

        visited = {}

        while open_set:
            f, g, curr, path = heapq.heappop(open_set)

            if g >= visited.get(curr, math.inf):
                continue

            visited[curr] = g

            if curr == goal:
                return path, g

            for nxt in nodes:
                if nxt == curr:
                    continue

                new_g = g + dist(curr, nxt)

                if new_g >= visited.get(nxt, math.inf):
                    continue

                if not is_line_clear(curr, nxt):
                    continue

                h = dist(nxt, goal)

                heapq.heappush(
                    open_set,
                    (new_g + h, new_g, nxt, path + [nxt])
                )

        return None, math.inf

    # 6. 출발점에서 가까운 도착 후보부터 탐색
    target_candidates.sort(
        key=lambda goal: (
            dist2(robot_start, goal),
            dist2(target_pos, goal)
        )
    )

    best_path = []

    for goal in target_candidates:
        path, cost = solve_astar(robot_start, goal)

        # 첫 번째 도달 가능한 후보를 선택
        if path:
            best_path = path
            break

    if not best_path:
        return []

    # 7. 경로 스무딩
    if len(best_path) > 2:
        smoothed = [best_path[0]]
        i = 0

        while i < len(best_path) - 1:
            j = len(best_path) - 1

            while j > i + 1:
                if is_line_clear(
                    best_path[i],
                    best_path[j]
                ):
                    break

                j -= 1

            smoothed.append(best_path[j])
            i = j

        best_path = smoothed

    return best_path


import matplotlib.pyplot as plt


def display_robot_path(
    path,
    points_list=None,
    connections=None,
    width=600,
    height=600,
    padding=30,
    image_size=(10, 8),
    save_path=None,
    show_grid=True,
):
    """
    find_robot_path()의 반환 경로를 자동 크기 조정하여 표시한다.

    Args:
        path: [(x, y), ...] 형태의 경로
        points_list: 장애물 꼭짓점 목록 (선택)
        connections: 장애물 연결 목록 (선택)
        width, height: 원본 지도 크기
        padding: 이미지 가장자리 여백 (좌표 단위)
        image_size: 출력 이미지 크기 (인치)
        save_path: 저장할 파일 경로. None이면 저장하지 않음
        show_grid: 격자 표시 여부

    Returns:
        fig, ax: matplotlib Figure와 Axes
    """

    if not path:
        print("표시할 경로가 없습니다.")
        return None, None

    path = [(float(p[0]), float(p[1])) for p in path]

    if not all(
        math.isfinite(v)
        for p in path
        for v in p
    ):
        raise ValueError("경로에 유효하지 않은 좌표가 포함되어 있습니다.")

    fig, ax = plt.subplots(figsize=image_size)

    # 1. 장애물 표시
    if points_list is not None and connections is not None:
        for conn in connections:
            i, j = int(conn[0]), int(conn[1])

            if 0 <= i < len(points_list) and 0 <= j < len(points_list):
                p1 = points_list[i]
                p2 = points_list[j]

                ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    color="black",
                    linewidth=2,
                    zorder=1,
                )

    # 2. 전체 경로 표시
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]

    ax.plot(
        xs,
        ys,
        color="tab:blue",
        linewidth=2,
        marker="o",
        markersize=5,
        label="Robot path",
        zorder=3,
    )

    # 시작점
    ax.scatter(
        xs[0], ys[0],
        color="tab:green",
        s=100,
        marker="o",
        label="Start",
        zorder=4,
    )

    # 도착점
    ax.scatter(
        xs[-1], ys[-1],
        color="tab:red",
        s=100,
        marker="X",
        label="End",
        zorder=4,
    )

    # 3. 경유점 번호 표시
    for i, (x, y) in enumerate(path):
        ax.annotate(
            str(i),
            (x, y),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
        )

    # 4. 모든 경로와 장애물을 포함하는 표시 범위 계산
    all_x = xs[:]
    all_y = ys[:]

    if points_list:
        for p in points_list:
            all_x.append(float(p[0]))
            all_y.append(float(p[1]))

    min_x = min(all_x)
    max_x = max(all_x)
    min_y = min(all_y)
    max_y = max(all_y)

    # 지도 범위도 포함하되, 전체 지도가 너무 크면
    # 경로 주변을 중심으로 자동 확대한다.
    span_x = max_x - min_x
    span_y = max_y - min_y

    padding_x = max(padding, span_x * 0.05, 1.0)
    padding_y = max(padding, span_y * 0.05, 1.0)

    min_x -= padding_x
    max_x += padding_x
    min_y -= padding_y
    max_y += padding_y

    # 5. 이미지 비율에 맞춰 전체 경로가 잘리지 않도록 설정
    aspect = image_size[0] / image_size[1]

    view_w = max_x - min_x
    view_h = max_y - min_y

    if view_w / view_h < aspect:
        extra = (view_h * aspect - view_w) / 2
        min_x -= extra
        max_x += extra
    else:
        extra = (view_w / aspect - view_h) / 2
        min_y -= extra
        max_y += extra

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    # 실제 좌표의 x, y 비율을 동일하게 유지한다.
    ax.set_aspect("equal", adjustable="box")

    if show_grid:
        ax.grid(True, linestyle="--", alpha=0.3)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Robot Path Visualization")
    ax.legend()
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")


    return fig, ax
