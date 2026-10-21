import heapq
import cv2
import numpy as np

import math

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
        return 'NONE', 0.0
    
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


import heapq
import math

_SQRT2 = math.sqrt(2.0)



def find_robot_path(
    points_list,
    connections,
    robot_start,
    target_pos,
    radius,
    width=600,
    height=600,
    grid_step=20,
    padding_radius=25,
):
    """
    장애물 안전거리 기반 2D 로봇 경로 탐색.

    주요 기능:
    - 원본 장애물 선분과 padding_radius를 이용한 충돌 판정
    - 선분 전체에 대한 충돌 검사
    - 적응형 격자 기반 A*
    - 대각선 모서리 통과 방지
    - 목표 주변의 안전한 대체 위치 탐색
    - 직선 경로 우선 검사
    - 도달 가능한 부분 경로 반환

    Returns:
        list[tuple[float, float]]:
            시작점부터 경유점 및 도착점까지의 경로.
            안전한 경로를 찾지 못하면 [] 반환.
    """
    import math
    import heapq

    SQRT2 = math.sqrt(2.0)
    EPS = 1e-9

    # --------------------------------------------------
    # 1. 입력 검증
    # --------------------------------------------------
    if width <= 0 or height <= 0:
        return []

    if grid_step <= 0 or padding_radius < 0 or radius < 0:
        return []

    try:
        start = (float(robot_start[0]), float(robot_start[1]))
        target = (float(target_pos[0]), float(target_pos[1]))
        width = float(width)
        height = float(height)
        grid_step = float(grid_step)
        padding_radius = float(padding_radius)
        radius = float(radius)
    except (TypeError, ValueError, IndexError):
        return []

    if not all(math.isfinite(v) for v in (
        *start, *target, width, height, grid_step,
        padding_radius, radius
    )):
        return []

    def in_bounds(p):
        return (
            0 <= p[0] <= width
            and 0 <= p[1] <= height
        )

    if not in_bounds(start) or not in_bounds(target):
        return []

    def dist2(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    def dist(a, b):
        return math.sqrt(dist2(a, b))

    # --------------------------------------------------
    # 2. 원본 장애물 선분 구성
    # --------------------------------------------------
    # 입력 points_list는 절대 수정하지 않는다.
    segments = []

    for conn in connections:
        try:
            i, j = int(conn[0]), int(conn[1])

            if not (
                0 <= i < len(points_list)
                and 0 <= j < len(points_list)
            ):
                continue

            p1 = (
                float(points_list[i][0]),
                float(points_list[i][1]),
            )
            p2 = (
                float(points_list[j][0]),
                float(points_list[j][1]),
            )

            if not all(math.isfinite(v) for v in (*p1, *p2)):
                continue

            # 길이가 0인 선분도 점 장애물로 처리할 수 있다.
            segments.append((p1, p2))

        except (TypeError, ValueError, IndexError):
            continue

    # --------------------------------------------------
    # 3. 기하학적 충돌 판정
    # --------------------------------------------------
    def point_segment_distance(p, a, b):
        """점 p와 선분 ab 사이의 최소 거리."""
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length2 = dx * dx + dy * dy

        if length2 <= EPS:
            return dist(p, a)

        t = (
            (p[0] - a[0]) * dx
            + (p[1] - a[1]) * dy
        ) / length2

        t = max(0.0, min(1.0, t))

        projection = (
            a[0] + t * dx,
            a[1] + t * dy,
        )

        return dist(p, projection)

    def orientation(a, b, c):
        return (
            (b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0])
        )

    def segments_intersect(a, b, c, d):
        """두 선분이 교차하거나 접하는지 검사."""
        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        if (
            ((o1 > EPS and o2 < -EPS)
             or (o1 < -EPS and o2 > EPS))
            and
            ((o3 > EPS and o4 < -EPS)
             or (o3 < -EPS and o4 > EPS))
        ):
            return True

        if abs(o1) <= EPS and point_segment_distance(c, a, b) <= EPS:
            return True
        if abs(o2) <= EPS and point_segment_distance(d, a, b) <= EPS:
            return True
        if abs(o3) <= EPS and point_segment_distance(a, c, d) <= EPS:
            return True
        if abs(o4) <= EPS and point_segment_distance(b, c, d) <= EPS:
            return True

        return False

    def segment_segment_distance(a, b, c, d):
        """두 선분 사이의 최소 거리."""
        if segments_intersect(a, b, c, d):
            return 0.0

        return min(
            point_segment_distance(a, c, d),
            point_segment_distance(b, c, d),
            point_segment_distance(c, a, b),
            point_segment_distance(d, a, b),
        )

    def point_blocked(p):
        """점 주변에 안전 반경을 확보할 수 없는지 검사."""
        if not in_bounds(p):
            return True

        for a, b in segments:
            if (
                point_segment_distance(p, a, b)
                <= padding_radius + EPS
            ):
                return True

        return False

    def segment_blocked(a, b):
        """이동 선분 전체가 안전한지 검사."""
        if not in_bounds(a) or not in_bounds(b):
            return True

        for c, d in segments:
            if (
                segment_segment_distance(a, b, c, d)
                <= padding_radius + EPS
            ):
                return True

        return False

    def line_of_sight_clear(a, b):
        # 샘플링이 아닌 선분 전체의 최소 거리로 검사한다.
        return not segment_blocked(a, b)

    # 시작 위치가 이미 장애물 안전 영역 안이라면
    # 안전한 경로를 보장할 수 없으므로 실패 처리한다.
    if point_blocked(start):
        return []

    # --------------------------------------------------
    # 4. 적응형 격자 설정
    # --------------------------------------------------
    travel_span = max(dist(start, target), 1.0)

    min_cells = 25
    max_cells = 120

    estimated_cells = travel_span / grid_step

    if estimated_cells < min_cells:
        grid_step = max(
            travel_span / min_cells,
            grid_step * 0.25,
        )
    elif estimated_cells > max_cells:
        grid_step = min(
            travel_span / max_cells,
            grid_step * 8.0,
        )

    grid_step = max(grid_step, 1.0)

    # 지도 전체를 격자로 사용해 좌표 변환을 단순화한다.
    cols = int(math.floor(width / grid_step)) + 1
    rows = int(math.floor(height / grid_step)) + 1

    def grid_coord(node):
        return (
            node[0] * grid_step,
            node[1] * grid_step,
        )

    # --------------------------------------------------
    # 5. 차단 격자 캐시
    # --------------------------------------------------
    blocked_cache = {}

    def is_blocked_node(node):
        if node not in blocked_cache:
            blocked_cache[node] = point_blocked(grid_coord(node))

        return blocked_cache[node]

    def nearest_free_node(p, require_connection=False):
        """
        주변 링을 탐색해 가장 가까운 자유 격자점을 찾는다.

        require_connection=True이면 입력 좌표에서 해당
        격자점까지의 이동 구간도 안전해야 한다.
        """
        base_c = round(p[0] / grid_step)
        base_r = round(p[1] / grid_step)

        base_c = max(0, min(cols - 1, base_c))
        base_r = max(0, min(rows - 1, base_r))

        max_ring = max(cols, rows)

        for ring in range(max_ring + 1):
            best_node = None
            best_d2 = math.inf

            for dc in range(-ring, ring + 1):
                for dr in range(-ring, ring + 1):
                    if max(abs(dc), abs(dr)) != ring:
                        continue

                    c = base_c + dc
                    r = base_r + dr

                    if not (0 <= c < cols and 0 <= r < rows):
                        continue

                    node = (c, r)

                    if is_blocked_node(node):
                        continue

                    coord = grid_coord(node)

                    if (
                        require_connection
                        and segment_blocked(p, coord)
                    ):
                        continue

                    d2 = dist2(p, coord)

                    if d2 < best_d2:
                        best_d2 = d2
                        best_node = node

            if best_node is not None:
                return best_node

        return None

    start_node = nearest_free_node(
        start,
        require_connection=True,
    )

    if start_node is None:
        return []

    # --------------------------------------------------
    # 6. 8방향 이웃 및 이동 비용
    # --------------------------------------------------
    neighbor_offsets = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    def neighbors(node):
        c, r = node
        current_pos = grid_coord(node)

        for dc, dr in neighbor_offsets:
            nc, nr = c + dc, r + dr

            if not (0 <= nc < cols and 0 <= nr < rows):
                continue

            nb = (nc, nr)

            if is_blocked_node(nb):
                continue

            next_pos = grid_coord(nb)

            # 노드뿐 아니라 실제 이동 선분도 검사한다.
            if segment_blocked(current_pos, next_pos):
                continue

            # 대각선으로 두 장애물 모서리 사이를 통과하지 않는다.
            if dc != 0 and dr != 0:
                side1 = (c + dc, r)
                side2 = (c, r + dr)

                if (
                    is_blocked_node(side1)
                    or is_blocked_node(side2)
                ):
                    continue

                if (
                    segment_blocked(current_pos, grid_coord(side1))
                    or segment_blocked(current_pos, grid_coord(side2))
                ):
                    continue

            cost = grid_step * (
                SQRT2 if dc != 0 and dr != 0 else 1.0
            )

            yield nb, cost

    def heuristic(node, goal):
        dc = abs(node[0] - goal[0])
        dr = abs(node[1] - goal[1])

        lo = min(dc, dr)
        hi = max(dc, dr)

        return grid_step * (
            hi - lo + SQRT2 * lo
        )

    # --------------------------------------------------
    # 7. A* 및 부분 경로 복원
    # --------------------------------------------------
    def astar(start_n, goal_n):
        open_heap = [
            (heuristic(start_n, goal_n), 0.0, start_n)
        ]

        came_from = {}
        g_score = {start_n: 0.0}
        closed = set()

        best_node = start_n
        best_h = heuristic(start_n, goal_n)

        while open_heap:
            _, g, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            # 오래된 우선순위 큐 항목은 무시한다.
            if g > g_score.get(current, math.inf) + EPS:
                continue

            closed.add(current)

            h = heuristic(current, goal_n)

            if h < best_h:
                best_h = h
                best_node = current

            if current == goal_n:
                best_node = current
                success = True
                break

            for nb, cost in neighbors(current):
                if nb in closed:
                    continue

                tentative_g = g + cost

                if tentative_g < g_score.get(nb, math.inf):
                    g_score[nb] = tentative_g
                    came_from[nb] = current

                    heapq.heappush(
                        open_heap,
                        (
                            tentative_g + heuristic(nb, goal_n),
                            tentative_g,
                            nb,
                        ),
                    )
        else:
            success = False

        path = [best_node]
        current = best_node

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path, success

    # --------------------------------------------------
    # 8. 목표 주변의 안전한 후보점 생성
    # --------------------------------------------------
    # 기존 코드처럼 장애물 꼭짓점을 목표 후보로 삼지 않는다.
    # 목표 자체가 막혀 있다면 주변의 실제 자유 격자점을 찾는다.
    candidates = []
    seen = set()

    def add_candidate(node):
        if node is None or node in seen:
            return

        if is_blocked_node(node):
            return

        pos = grid_coord(node)

        if not in_bounds(pos):
            return

        seen.add(node)
        candidates.append(node)

    target_node = nearest_free_node(target)

    if target_node is not None:
        target_grid_pos = grid_coord(target_node)

        if line_of_sight_clear(target, target_grid_pos):
            add_candidate(target_node)

    max_radius = max(
        radius,
        padding_radius + grid_step,
        1.0,
    )

    # 목표 주변을 여러 반경과 각도로 샘플링한다.
    # 기존 코드의 최대 6회 반경 확장 정책을 유지한다.
    for expansion in range(7):
        ring_radius = max_radius * (1.6 ** expansion)

        for i in range(24):
            angle = 2.0 * math.pi * i / 24

            sample = (
                target[0] + math.cos(angle) * ring_radius,
                target[1] + math.sin(angle) * ring_radius,
            )

            if not in_bounds(sample):
                continue

            node = nearest_free_node(sample)

            if node is None:
                continue

            pos = grid_coord(node)

            if dist(pos, target) <= ring_radius + grid_step:
                add_candidate(node)

        if len(candidates) >= 8:
            break

    # 목표와 가까운 후보부터 시도한다.
    candidates.sort(key=lambda n: dist2(grid_coord(n), target))

    if not candidates:
        return []

    # --------------------------------------------------
    # 9. 후보별 직선 검사 및 A*
    # --------------------------------------------------
    best_fallback = None
    best_fallback_d2 = math.inf

    for goal_node in candidates[:8]:
        goal_pos = grid_coord(goal_node)

        # 직선 이동이 안전하다면 A*를 생략한다.
        if line_of_sight_clear(start, goal_pos):
            return [start, goal_pos] if dist2(start, goal_pos) > EPS else [start]

        # 시작 격자점과 실제 시작점 사이도 안전해야 한다.
        if segment_blocked(start, grid_coord(start_node)):
            continue

        node_path, success = astar(start_node, goal_node)

        if not node_path:
            continue

        world_path = [grid_coord(n) for n in node_path]

        # 실제 시작점에서 첫 격자점까지의 구간을 검증한다.
        if segment_blocked(start, world_path[0]):
            continue

        result = [start]

        if dist2(start, world_path[0]) > EPS:
            result.append(world_path[0])

        for p in world_path[1:]:
            if dist2(result[-1], p) > EPS:
                result.append(p)

        # 목표 좌표 자체까지 안전하게 연결할 수 있으면 추가한다.
        if (
            success
            and dist2(world_path[-1], target) > EPS
            and line_of_sight_clear(world_path[-1], target)
        ):
            result.append(target)

        # 최종 경로의 모든 구간을 다시 검사한다.
        valid = all(
            not segment_blocked(result[i], result[i + 1])
            for i in range(len(result) - 1)
        )

        if not valid:
            continue

        if success:
            return result

        # 부분 경로 중 목표에 가장 가까운 결과를 보관한다.
        end_d2 = dist2(result[-1], target)

        if end_d2 < best_fallback_d2:
            best_fallback_d2 = end_d2
            best_fallback = result

    return best_fallback if best_fallback is not None else []


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

### 2 ver

import heapq
import cv2
import numpy as np

import math

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
import heapq
import math


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



import math
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




