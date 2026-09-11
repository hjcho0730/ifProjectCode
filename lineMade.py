import heapq
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



import math


import heapq
import math


import heapq
import math

_SQRT2 = math.sqrt(2.0)


def find_robot_path(points_list, connections, robot_start, target_pos, radius,
                     width=600, height=600, grid_step=20, padding_radius=25):
    """
    장애물 마스크 + 후보군 기반 A* 로봇 경로 탐색 (최적화 버전)
    - 거리 적응형 grid_step / 탐색 영역
    - 도착 시 target_pos를 바라보도록 방향 및 위치 보정
    - 격자 단위(grid unit) 기반 계산
    - Line-of-sight shortcut, 차단 셀 캐싱, dist^2 비교, octile heuristic,
      후보 시도 횟수 상한 등으로 추가 최적화

    Returns:
        path: [robot_start, waypoint1, ..., destination]
              (실패 시 가장 가까운 지점까지의 경로, 완전 실패 시 [])
    """

    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def dist2(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    def point_segment_distance(p, a, b):
        px, py = p
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        if dx == 0 and dy == 0:
            return dist(p, a)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        proj = (ax + t * dx, ay + t * dy)
        return dist(p, proj)

    # ---------- 1. 장애물 선분 구성 ----------
    segments = []
    n_points = len(points_list)
    for conn in connections:
        i, j = conn[0], conn[1]
        if 0 <= i < n_points and 0 <= j < n_points:
            segments.append((tuple(points_list[i]), tuple(points_list[j])))

    # ---------- 2. 공간 해시(버킷) 인덱싱 ----------
    bucket_size = max(grid_step * 2, padding_radius * 2)
    seg_buckets = {}

    def bucket_keys_for_segment(a, b):
        min_x = min(a[0], b[0]) - padding_radius
        max_x = max(a[0], b[0]) + padding_radius
        min_y = min(a[1], b[1]) - padding_radius
        max_y = max(a[1], b[1]) + padding_radius
        kx0, kx1 = int(min_x // bucket_size), int(max_x // bucket_size)
        ky0, ky1 = int(min_y // bucket_size), int(max_y // bucket_size)
        return [(kx, ky) for kx in range(kx0, kx1 + 1) for ky in range(ky0, ky1 + 1)]

    for seg in segments:
        for key in bucket_keys_for_segment(*seg):
            seg_buckets.setdefault(key, []).append(seg)

    def nearby_segments(p):
        kx, ky = int(p[0] // bucket_size), int(p[1] // bucket_size)
        result = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                b = seg_buckets.get((kx + dx, ky + dy))
                if b:
                    result.extend(b)
        return result

    def raw_is_blocked(p, bounds):
        min_x, min_y, max_x, max_y = bounds
        if p[0] < min_x or p[0] > max_x or p[1] < min_y or p[1] > max_y:
            return True
        for seg in nearby_segments(p):
            if point_segment_distance(p, seg[0], seg[1]) <= padding_radius:
                return True
        return False

    def is_blocked_unbounded(p):
        if p[0] < 0 or p[0] > width or p[1] < 0 or p[1] > height:
            return True
        for seg in nearby_segments(p):
            if point_segment_distance(p, seg[0], seg[1]) <= padding_radius:
                return True
        return False

    # ---------- Step 1: Line-of-sight 체크 ----------
    def line_of_sight_clear(a, b):
        d = dist(a, b)
        if d < 1e-6:
            return True
        step_len = max(padding_radius * 0.6, 1.0)
        num_steps = min(200, max(2, int(d / step_len)))
        dx = (b[0] - a[0]) / num_steps
        dy = (b[1] - a[1]) / num_steps
        for k in range(1, num_steps + 1):
            p = (a[0] + dx * k, a[1] + dy * k)
            if is_blocked_unbounded(p):
                return False
        return True

    # ---------- 3. 후보군 선정 (반경 자동 확장 포함) ----------
    def collect_candidates(r):
        r2 = r * r
        return [tuple(p) for p in points_list if dist2(p, target_pos) <= r2]

    candidates = collect_candidates(radius)
    cur_radius = radius
    expand_tries = 0
    while not candidates and expand_tries < 6:
        cur_radius *= 1.6
        candidates = collect_candidates(cur_radius)
        expand_tries += 1
    if not candidates:
        candidates = [tuple(target_pos)]
    candidates.sort(key=lambda p: dist2(robot_start, p))  # Step 3: sqrt 없이 정렬

    # ---------- 4. 거리 적응형 grid_step 및 탐색 바운딩 박스 계산 ----------
    farthest_candidate_dist = max(dist(robot_start, c) for c in candidates)
    travel_span = max(farthest_candidate_dist, 1.0)

    min_cells = 25
    max_cells = 120

    adaptive_step = grid_step
    est_cells = travel_span / adaptive_step
    if est_cells < min_cells:
        adaptive_step = max(travel_span / min_cells, grid_step * 0.25)
    elif est_cells > max_cells:
        adaptive_step = min(travel_span / max_cells, grid_step * 8)

    grid_step = max(adaptive_step, 1.0)

    margin = max(padding_radius * 3, grid_step * 5)
    xs = [robot_start[0]] + [c[0] for c in candidates]
    ys = [robot_start[1]] + [c[1] for c in candidates]
    region_min_x = max(0, min(xs) - margin)
    region_max_x = min(width, max(xs) + margin)
    region_min_y = max(0, min(ys) - margin)
    region_max_y = min(height, max(ys) + margin)
    bounds = (region_min_x, region_min_y, region_max_x, region_max_y)

    cols = max(1, int((region_max_x - region_min_x) // grid_step) + 1)
    rows = max(1, int((region_max_y - region_min_y) // grid_step) + 1)

    # ---------- Step 2: 차단 셀 캐시 ----------
    blocked_cache = {}

    def grid_coord(node):
        return (region_min_x + node[0] * grid_step, region_min_y + node[1] * grid_step)

    def is_blocked_node(node):
        cached = blocked_cache.get(node)
        if cached is not None:
            return cached
        result = raw_is_blocked(grid_coord(node), bounds)
        blocked_cache[node] = result
        return result

    def nearest_free_node(p):
        base_c = round((p[0] - region_min_x) / grid_step)
        base_r = round((p[1] - region_min_y) / grid_step)
        max_ring = max(cols, rows)
        for ring in range(0, max_ring + 1):
            best, best_d2 = None, None
            for dc in range(-ring, ring + 1):
                for dr in range(-ring, ring + 1):
                    if max(abs(dc), abs(dr)) != ring:
                        continue
                    c, r = base_c + dc, base_r + dr
                    if 0 <= c < cols and 0 <= r < rows:
                        node = (c, r)
                        if not is_blocked_node(node):
                            coord = grid_coord(node)
                            d2 = dist2(p, coord)  # Step 3
                            if best_d2 is None or d2 < best_d2:
                                best_d2, best = d2, node
            if best is not None:
                return best
        return None

    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1),
                         (-1, -1), (-1, 1), (1, -1), (1, 1)]

    def neighbors(node):
        c, r = node
        for dc, dr in neighbor_offsets:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows:
                nb = (nc, nr)
                if not is_blocked_node(nb):  # Step 2: 캐시 사용
                    cost = (_SQRT2 if dc != 0 and dr != 0 else 1.0) * grid_step
                    yield nb, cost

    def heuristic(node, goal_node):
        # Step 4: octile distance, 격자 인덱스 기준 (grid_coord 변환 없이 저비용)
        dc = abs(node[0] - goal_node[0])
        dr = abs(node[1] - goal_node[1])
        lo, hi = (dc, dr) if dc < dr else (dr, dc)
        return (hi - lo + _SQRT2 * lo) * grid_step

    # ---------- A* ----------
    def astar(start_node, goal_node):
        open_heap = [(heuristic(start_node, goal_node), 0.0, start_node)]
        came_from = {}
        g_score = {start_node: 0.0}
        closed = set()
        best_node, best_h = start_node, heuristic(start_node, goal_node)

        while open_heap:
            f, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            h = heuristic(current, goal_node)
            if h < best_h:
                best_h, best_node = h, current

            if current == goal_node:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, True

            for nb, cost in neighbors(current):
                if nb in closed:
                    continue
                tentative_g = g + cost
                if tentative_g < g_score.get(nb, math.inf):
                    g_score[nb] = tentative_g
                    came_from[nb] = current
                    heapq.heappush(open_heap, (tentative_g + heuristic(nb, goal_node), tentative_g, nb))

        path = [best_node]
        cur = best_node
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path, False

    # ---------- 도착 시 target_pos 방향으로 전진 ----------
    def march_toward_target(from_point, to_point):
        total = dist(from_point, to_point)
        if total < 1e-6:
            return from_point
        num_steps = min(40, max(4, int(total / max(grid_step, 1.0))))
        dx = (to_point[0] - from_point[0]) / num_steps
        dy = (to_point[1] - from_point[1]) / num_steps

        last_valid = from_point
        cur = from_point
        for _ in range(num_steps):
            nxt = (cur[0] + dx, cur[1] + dy)
            if is_blocked_unbounded(nxt):
                break
            last_valid = nxt
            cur = nxt
        return last_valid

    start_node = nearest_free_node(robot_start)
    if start_node is None:
        return []

    best_fallback_path = None
    best_fallback_dist2 = None

    # Step 5: 후보 시도 횟수 상한
    max_candidate_tries = min(len(candidates), 8)

    for cand in candidates[:max_candidate_tries]:
        # Step 1: 직선 경로가 뚫려 있으면 A* 생략
        if line_of_sight_clear(robot_start, cand):
            final_point = march_toward_target(cand, tuple(target_pos))
            result = [tuple(robot_start), tuple(cand)]
            if dist2(final_point, cand) > 1e-6:
                result.append(final_point)
            return result

        goal_node = nearest_free_node(cand)
        if goal_node is None:
            continue

        node_path, success = astar(start_node, goal_node)
        world_path = [grid_coord(n) for n in node_path]

        if success:
            final_point = march_toward_target(cand, tuple(target_pos))
            result = [tuple(robot_start)] + world_path[1:-1] + [tuple(cand)]
            if dist2(final_point, cand) > 1e-6:
                result.append(final_point)
            return result

        end_d2 = dist2(world_path[-1], cand)
        if best_fallback_dist2 is None or end_d2 < best_fallback_dist2:
            best_fallback_dist2 = end_d2
            final_point = march_toward_target(world_path[-1], tuple(target_pos))
            path_candidate = [tuple(robot_start)] + world_path[1:]
            if dist2(final_point, world_path[-1]) > 1e-6:
                path_candidate.append(final_point)
            best_fallback_path = path_candidate

    return best_fallback_path if best_fallback_path else []

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

