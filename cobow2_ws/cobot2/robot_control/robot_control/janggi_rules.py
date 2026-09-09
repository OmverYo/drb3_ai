"""janggi_rules.py

장기(Janggi) 기물 이동 규칙 검증기.

robot_control 노드가 물리적으로 말을 이동시키기 "전"에, 그 이동이 실제 장기 규칙상
유효한지 검사하기 위한 모듈이다. 기물별 이동 방식(직선/캐논/나이트형/코끼리형/궁성 내
한 칸 이동/병졸 전진)은 janggi_move_rules.yaml 에서 읽어오므로, 규칙(궁성 좌표,
전진 방향, 포가 넘을 수 없는 기물 목록 등)을 바꾸고 싶으면 이 파일을 고치지 말고
YAML만 수정하면 된다.

이 모듈은 ROS와 무관하게 순수 파이썬으로 동작하므로 단위 테스트가 쉽다.

좌표 규약
---------
- (row, col) 은 robot_control.py 의 0-based 내부 좌표(row0, col0)를 그대로 사용한다.
- occupancy 는 {(row, col): {"name": "cha_red", "score": 0.9, ...}, ...} 형태의 dict.
  해당 칸에 아무 기물도 없으면 키 자체가 없다(= 빈 칸).
- 기물 이름은 반드시 "<피스타입>_<팀>" 형식이어야 한다 (예: "cha_red", "jol_green").
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover - PyYAML은 ROS2 파이썬 환경에 보통 포함됨
    raise ImportError(
        "janggi_rules.py 는 PyYAML이 필요합니다. `pip install pyyaml` 로 설치하세요."
    ) from exc


Cell = Tuple[int, int]


@dataclass
class MoveResult:
    ok: bool
    reason: str

    def __bool__(self):
        return self.ok

    # (row,col) 튜플처럼 unpack 하고 싶을 때를 대비해 iterable 도 지원.
    def __iter__(self):
        yield self.ok
        yield self.reason


@dataclass
class _Palace:
    team: str
    cells: set = field(default_factory=set)
    diagonals: List[List[Cell]] = field(default_factory=list)


class JanggiRuleEngine:
    """YAML 규칙을 읽어 before -> after 이동의 유효성을 판정한다."""

    def __init__(self, rules_path: Optional[str] = None, rules: Optional[dict] = None):
        if rules is None:
            if rules_path is None:
                raise ValueError("rules_path 또는 rules 중 하나는 반드시 주어져야 합니다.")
            with open(rules_path, "r", encoding="utf-8") as f:
                rules = yaml.safe_load(f)
        self.rules = rules

        board = rules.get("board", {})
        self.rows = int(board.get("rows", 10))
        self.cols = int(board.get("cols", 9))

        self.forward_direction: Dict[str, int] = {
            team: int(val) for team, val in rules.get("forward_direction", {}).items()
        }

        self.palaces: Dict[str, _Palace] = {}
        for team, pdef in rules.get("palace", {}).items():
            palace_rows = pdef.get("rows", [])
            palace_cols = pdef.get("cols", [])
            cells = {(r, c) for r in palace_rows for c in palace_cols}
            diagonals = [
                [tuple(pt) for pt in line] for line in pdef.get("diagonals", [])
            ]
            self.palaces[team] = _Palace(team=team, cells=cells, diagonals=diagonals)

        self.piece_rules: Dict[str, dict] = rules.get("pieces", {})

        self._handlers = {
            "line": self._validate_line,
            "cannon": self._validate_cannon,
            "horse": self._validate_horse,
            "elephant": self._validate_elephant,
            "palace_step": self._validate_palace_step,
            "soldier": self._validate_soldier,
        }

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def validate_move(
        self,
        piece_name: Optional[str],
        before: Cell,
        after: Cell,
        occupancy: Dict[Cell, dict],
    ) -> MoveResult:
        """before -> after 로 piece_name 기물을 이동시키는 것이 유효한지 검사한다.

        occupancy 에는 이동하려는 기물 자신도 before 칸에 들어있을 수 있으나(있어도
        없어도 결과에 영향 없음), 계산 과정에서 before 칸은 항상 "출발점"으로 취급되어
        경로/도착지 판정에서 제외된다.
        """
        if not piece_name:
            return MoveResult(False, "이동할 기물의 이름(클래스)을 확인할 수 없습니다.")

        piece_type, team = self._split_name(piece_name)
        if team is None:
            return MoveResult(
                False,
                f"'{piece_name}' 에서 팀 정보를 분리할 수 없습니다 "
                "(클래스명은 '<기물>_<팀>' 형식이어야 합니다).",
            )

        if before == after:
            return MoveResult(False, "출발지와 도착지가 동일합니다.")

        if not self._in_bounds(before) or not self._in_bounds(after):
            return MoveResult(False, f"좌표가 판 범위를 벗어났습니다: {before} -> {after}")

        rule = self.piece_rules.get(piece_type)
        if rule is None:
            return MoveResult(
                False, f"'{piece_type}' 기물에 대한 이동 규칙이 정의되어 있지 않습니다."
            )

        move_type = rule.get("move_type")
        handler = self._handlers.get(move_type)
        if handler is None:
            return MoveResult(False, f"알 수 없는 move_type 입니다: {move_type}")

        # occupancy 에서 출발지 자기 자신은 경로/도착지 검사에 영향을 주지 않도록 제외.
        occupancy_wo_self = {c: v for c, v in occupancy.items() if c != before}

        return handler(piece_type, team, before, after, occupancy_wo_self, rule)

    def build_occupancy_from_detections(
        self,
        detections: List[dict],
        cell_positions: Dict[Cell, "object"],
        match_radius: float,
    ) -> Dict[Cell, dict]:
        """BASE 프레임 detection 목록을 보드 칸(row, col)에 스냅해 occupancy grid로 변환한다.

        detections: [{"pos": np.array([x, y, z]), "name": str, "score": float}, ...]
        cell_positions: {(row, col): np.array([x, y, z])}  -- 각 칸의 BASE 좌표(캐시 권장)
        match_radius: 이 거리(mm) 안에 있는 detection만 해당 칸의 점유로 인정.

        같은 칸에 여러 detection이 매칭되면 score가 더 높은 쪽을 채택한다.
        """
        occupancy: Dict[Cell, dict] = {}
        if not detections or not cell_positions:
            return occupancy

        for det in detections:
            det_pos = det["pos"]
            best_cell = None
            best_dist = None
            for cell, cell_pos in cell_positions.items():
                dist = float(
                    ((det_pos[0] - cell_pos[0]) ** 2 + (det_pos[1] - cell_pos[1]) ** 2) ** 0.5
                )
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_cell = cell
            if best_cell is None or best_dist > match_radius:
                continue
            existing = occupancy.get(best_cell)
            if existing is None or det.get("score", 0.0) > existing.get("score", 0.0):
                occupancy[best_cell] = det
        return occupancy

    # ------------------------------------------------------------------
    # per move-type handlers
    # ------------------------------------------------------------------
    def _validate_line(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        segment = self._segment_between(before, after, allow_diagonal=rule.get("palace_diagonal", False))
        if segment is None:
            return MoveResult(
                False,
                f"{piece_type}: 가로/세로 직선이 아니거나(궁성 대각선 규칙 위반) 이동할 수 없습니다.",
            )
        between_cells, _distance = segment

        max_distance = rule.get("max_distance")
        if max_distance is not None and _distance > max_distance:
            return MoveResult(False, f"{piece_type}: 최대 이동 거리({max_distance}칸)를 초과했습니다.")

        for cell in between_cells:
            if cell in occupancy:
                blocker = occupancy[cell]
                return MoveResult(
                    False,
                    f"{piece_type}: 이동 경로 {cell} 에 다른 기물({blocker.get('name')})이 있어 "
                    "이동할 수 없습니다.",
                )

        dest = occupancy.get(after)
        if dest is not None:
            _, dest_team = self._split_name(dest.get("name"))
            if dest_team == team:
                return MoveResult(False, f"{piece_type}: 도착지에 같은 편 기물이 이미 있습니다.")

        return MoveResult(True, "OK")

    def _validate_cannon(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        segment = self._segment_between(before, after, allow_diagonal=rule.get("palace_diagonal", False))
        if segment is None:
            return MoveResult(
                False,
                f"{piece_type}: 가로/세로 직선이 아니거나(궁성 대각선 규칙 위반) 이동할 수 없습니다.",
            )
        between_cells, _distance = segment

        screens = [c for c in between_cells if c in occupancy]
        if len(screens) == 0:
            return MoveResult(
                False, f"{piece_type}: 포는 반드시 다른 기물을 하나 넘어야 합니다(넘을 대상 없음)."
            )
        if len(screens) > 1:
            return MoveResult(
                False, f"{piece_type}: 이동 경로에 기물이 두 개 이상 있어 넘을 수 없습니다."
            )

        screen_cell = screens[0]
        screen_piece = occupancy[screen_cell]
        screen_type, _screen_team = self._split_name(screen_piece.get("name"))
        forbidden_screens = rule.get("screen_forbidden_pieces", [])
        if screen_type in forbidden_screens:
            return MoveResult(
                False,
                f"{piece_type}: {screen_cell} 의 {screen_type} 는 넘을 수 없는 기물입니다.",
            )

        dest = occupancy.get(after)
        if dest is not None:
            dest_type, dest_team = self._split_name(dest.get("name"))
            if dest_team == team:
                return MoveResult(False, f"{piece_type}: 도착지에 같은 편 기물이 이미 있습니다.")
            forbidden_captures = rule.get("capture_forbidden_pieces", [])
            if dest_type in forbidden_captures:
                return MoveResult(False, f"{piece_type}: {dest_type} 는 포로 잡을 수 없습니다.")

        return MoveResult(True, "OK")

    def _validate_horse(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        dr, dc = after[0] - before[0], after[1] - before[1]
        valid_offsets = {
            (2, 1), (2, -1), (-2, 1), (-2, -1),
            (1, 2), (1, -2), (-1, 2), (-1, -2),
        }
        if (dr, dc) not in valid_offsets:
            return MoveResult(False, f"{piece_type}: 마(馬)의 이동 형태(직 2 + 대각 1)가 아닙니다.")

        if abs(dr) == 2:
            leg = (before[0] + (1 if dr > 0 else -1), before[1])
        else:
            leg = (before[0], before[1] + (1 if dc > 0 else -1))

        if leg in occupancy:
            blocker = occupancy[leg]
            return MoveResult(
                False,
                f"{piece_type}: 다리({leg})에 기물({blocker.get('name')})이 있어 멍군(다리 걸림)입니다.",
            )

        return self._check_destination(piece_type, team, after, occupancy)

    def _validate_elephant(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        dr, dc = after[0] - before[0], after[1] - before[1]
        valid_offsets = {
            (3, 2), (3, -2), (-3, 2), (-3, -2),
            (2, 3), (-2, 3), (2, -3), (-2, -3),
        }
        if (dr, dc) not in valid_offsets:
            return MoveResult(False, f"{piece_type}: 상(象)의 이동 형태(직 3 + 대각 2)가 아닙니다.")

        sr = 1 if dr > 0 else -1
        sc = 1 if dc > 0 else -1
        if abs(dr) == 3:
            leg1 = (before[0] + sr, before[1])
            leg2 = (before[0] + 2 * sr, before[1] + sc)
        else:
            leg1 = (before[0], before[1] + sc)
            leg2 = (before[0] + sr, before[1] + 2 * sc)

        for leg in (leg1, leg2):
            if leg in occupancy:
                blocker = occupancy[leg]
                return MoveResult(
                    False,
                    f"{piece_type}: 다리({leg})에 기물({blocker.get('name')})이 있어 멍군(다리 걸림)입니다.",
                )

        return self._check_destination(piece_type, team, after, occupancy)

    def _validate_palace_step(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        palace = self._palace_containing(before)
        if palace is None:
            return MoveResult(False, f"{piece_type}: 궁성 밖에서는 이동 규칙을 판정할 수 없습니다.")
        if after not in palace.cells:
            return MoveResult(False, f"{piece_type}: 궁성 밖으로는 이동할 수 없습니다.")

        dr, dc = after[0] - before[0], after[1] - before[1]
        if max(abs(dr), abs(dc)) != 1:
            return MoveResult(False, f"{piece_type}: 궁성 안에서 한 칸만 이동할 수 있습니다.")

        if abs(dr) == 1 and abs(dc) == 1:
            if not self._is_single_diagonal_step(palace, before, after):
                return MoveResult(
                    False, f"{piece_type}: 궁성 대각선(米자) 라인 위에서만 대각선 이동이 가능합니다."
                )

        return self._check_destination(piece_type, team, after, occupancy)

    def _validate_soldier(self, piece_type, team, before, after, occupancy, rule) -> MoveResult:
        forward = self.forward_direction.get(team)
        if forward is None:
            return MoveResult(False, f"'{team}' 진영의 전진 방향(forward_direction)이 정의되지 않았습니다.")

        dr, dc = after[0] - before[0], after[1] - before[1]
        valid = False

        if dr == 0 and abs(dc) == 1:
            valid = True  # 좌/우 한 칸
        elif dc == 0 and dr == forward:
            valid = True  # 전진 한 칸 (후진 불가)
        elif abs(dr) == 1 and abs(dc) == 1 and dr == forward:
            # 궁성 대각선 라인 위에서만 전진 대각선 이동 허용.
            palace = self._palace_containing(before) or self._palace_containing(after)
            if palace is not None and self._is_single_diagonal_step(palace, before, after):
                valid = True

        if not valid:
            return MoveResult(
                False,
                f"{piece_type}: 병/졸은 앞/좌/우 한 칸(후진 불가), "
                "궁성 안에서는 전진 대각선(米자 라인)만 이동할 수 있습니다.",
            )

        return self._check_destination(piece_type, team, after, occupancy)

    # ------------------------------------------------------------------
    # shared helpers
    # ------------------------------------------------------------------
    def _check_destination(self, piece_type, team, after, occupancy) -> MoveResult:
        dest = occupancy.get(after)
        if dest is not None:
            _, dest_team = self._split_name(dest.get("name"))
            if dest_team == team:
                return MoveResult(False, f"{piece_type}: 도착지에 같은 편 기물이 이미 있습니다.")
        return MoveResult(True, "OK")

    def _segment_between(
        self, before: Cell, after: Cell, allow_diagonal: bool
    ) -> Optional[Tuple[List[Cell], int]]:
        """직선(가로/세로) 이동이면 (사이 칸 목록, 거리)를 반환.
        allow_diagonal 이 True이고 궁성 대각선(米자) 라인 위의 이동이면 그것도 인정한다.
        둘 다 아니면 None.
        """
        dr, dc = after[0] - before[0], after[1] - before[1]

        if dr == 0 and dc == 0:
            return None

        if dr == 0 or dc == 0:
            step_r = 0 if dr == 0 else (1 if dr > 0 else -1)
            step_c = 0 if dc == 0 else (1 if dc > 0 else -1)
            distance = max(abs(dr), abs(dc))
            cells = []
            r, c = before
            for _ in range(distance - 1):
                r += step_r
                c += step_c
                cells.append((r, c))
            return cells, distance

        if allow_diagonal:
            diag = self._diagonal_segment(before, after)
            if diag is not None:
                return diag

        return None

    def _diagonal_segment(self, before: Cell, after: Cell) -> Optional[Tuple[List[Cell], int]]:
        """두 점이 같은 궁성 대각선(米자) 라인 위에 있으면 (사이 칸 목록, 거리)를 반환."""
        for palace in self.palaces.values():
            for line in palace.diagonals:
                if before in line and after in line:
                    idx1, idx2 = line.index(before), line.index(after)
                    lo, hi = sorted((idx1, idx2))
                    return line[lo + 1:hi], hi - lo
        return None

    def _is_single_diagonal_step(self, palace: _Palace, before: Cell, after: Cell) -> bool:
        for line in palace.diagonals:
            if before in line and after in line:
                idx1, idx2 = line.index(before), line.index(after)
                if abs(idx1 - idx2) == 1:
                    return True
        return False

    def _palace_containing(self, cell: Cell) -> Optional[_Palace]:
        for palace in self.palaces.values():
            if cell in palace.cells:
                return palace
        return None

    def _in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    @staticmethod
    def _split_name(name: Optional[str]) -> Tuple[str, Optional[str]]:
        if not name:
            return "", None
        if "_" not in name:
            return name, None
        piece_type, team = name.rsplit("_", 1)
        return piece_type, team


def load_rule_engine(default_relative_path: str = os.path.join("resource", "janggi_move_rules.yaml")) -> "JanggiRuleEngine":
    """환경변수 JANGGI_RULES_PATH 우선, 없으면 패키지 기준 기본 경로에서 규칙을 로드한다."""
    override = os.getenv("JANGGI_RULES_PATH")
    if override:
        return JanggiRuleEngine(rules_path=override)

    try:
        from ament_index_python.packages import get_package_share_directory
        package_path = get_package_share_directory("robot_control")
        path = os.path.join(package_path, default_relative_path)
        if os.path.exists(path):
            return JanggiRuleEngine(rules_path=path)
    except Exception:
        pass

    # 마지막 fallback: 이 파일과 같은 디렉터리에 있는 janggi_move_rules.yaml
    local_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "janggi_move_rules.yaml")
    return JanggiRuleEngine(rules_path=local_path)


if __name__ == "__main__":
    # 간단한 자체 테스트 (ROS 없이 실행 가능): python3 janggi_rules.py
    here = os.path.dirname(os.path.abspath(__file__))
    engine = JanggiRuleEngine(rules_path=os.path.join(here, "janggi_move_rules.yaml"))

    # 차(cha_green)가 (0,0) -> (0,4) 로 이동하려는데, (0,2)에 다른 기물이 있어 막힘.
    occ = {(0, 2): {"name": "jol_green", "score": 0.9}}
    result = engine.validate_move("cha_green", (0, 0), (0, 4), occ)
    print("경로 막힘 테스트:", result.ok, "-", result.reason)
    assert result.ok is False

    # 경로가 비어있으면 이동 가능.
    result = engine.validate_move("cha_green", (0, 0), (0, 4), {})
    print("직선 이동 테스트:", result.ok, "-", result.reason)
    assert result.ok is True

    # 포(po)는 스크린 없이 이동 불가.
    result = engine.validate_move("po_red", (9, 1), (9, 7), {})
    print("포 스크린 없음 테스트:", result.ok, "-", result.reason)
    assert result.ok is False

    # 포가 정확히 하나를 넘으면 이동 가능.
    occ = {(9, 4): {"name": "cha_red", "score": 0.9}}
    result = engine.validate_move("po_red", (9, 1), (9, 7), occ)
    print("포 스크린 하나 테스트:", result.ok, "-", result.reason)
    assert result.ok is True

    # 마(ma) 다리 걸림 테스트.
    occ = {(1, 0): {"name": "jol_green", "score": 0.9}}
    result = engine.validate_move("ma_green", (0, 0), (2, 1), occ)
    print("마 다리걸림 테스트:", result.ok, "-", result.reason)
    assert result.ok is False

    print("모든 자체 테스트 통과.")
