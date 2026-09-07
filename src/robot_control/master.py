#!/usr/bin/env python3
import os
import json
import uuid
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import psycopg2
from psycopg2.extras import Json


# ============================================================
# DB 설정
# ============================================================
DB_CONFIG = {
    "host": "localhost",
    "port": 5432,
    "dbname": "translation_db",
    "user": "postgres",
    "password": "postgres",
}


# ============================================================
# Master Node
# ============================================================
class JanggiMasterNode(Node):

    def __init__(self):
        super().__init__("janggi_master_node")

        # ----------------------------------------------------
        # ROS2 INPUT
        # ----------------------------------------------------
        self.gesture_sub = self.create_subscription(
            String,
            "/input/gesture",
            self.gesture_callback,
            10,
        )

        self.voice_sub = self.create_subscription(
            String,
            "/input/voice",
            self.voice_callback,
            10,
        )

        self.board_state_sub = self.create_subscription(
            String,
            "/vision/board_state",
            self.board_state_callback,
            10,
        )

        # ----------------------------------------------------
        # ROS2 OUTPUT
        # ----------------------------------------------------
        self.robot_cmd_pub = self.create_publisher(
            String,
            "/game/robot_command",
            10,
        )

        # RobotControlNode -> MasterNode
        # 예:
        # {
        #   "command_id": "...",
        #   "success": true,
        #   "message": "hover completed"
        # }
        self.robot_result_sub = self.create_subscription(
            String,
            "/game/robot_result",
            self.robot_result_callback,
            10,
        )

        # UI/디버깅용 상태 발행
        self.status_pub = self.create_publisher(
            String,
            "/game/master_status",
            10,
        )

        # ----------------------------------------------------
        # 상태 관리
        # ----------------------------------------------------
        self.session_id = str(uuid.uuid4())

        self.state = "IDLE"
        self.robot_busy = False
        self.active_robot_command_id: Optional[str] = None
        self.active_robot_command_type: Optional[str] = None

        self.selected_team = "CHO"
        self.selected_piece = ""
        self.piece_candidates: List[Dict[str, Any]] = []
        self.current_candidate_index = 0

        self.legal_destinations: List[Dict[str, int]] = []
        self.current_destination_index = 0

        self.board_state: Dict[str, Any] = {
            "pieces": []
        }

        # gesture debounce
        self.last_gesture_intent = ""
        self.last_gesture_time = 0.0
        self.gesture_debounce_sec = 0.45

        # 신뢰도 threshold
        self.min_gesture_confidence = 0.70
        self.min_voice_confidence = 0.60

        # ----------------------------------------------------
        # DB 초기화
        # ----------------------------------------------------
        self.db_ready = False
        self.init_database()

        self.get_logger().info("=" * 65)
        self.get_logger().info("Janggi Master Node + DB started")
        self.get_logger().info(f"session_id = {self.session_id}")
        self.get_logger().info("=" * 65)

        self.publish_status()

    # ========================================================
    # DB
    # ========================================================
    def get_db_connection(self):
        return psycopg2.connect(**DB_CONFIG)

    def init_database(self):
        """
        필요한 테이블이 없으면 생성.
        실제 서비스 DB 스키마가 따로 있다면 CREATE TABLE 부분만 제거하고
        INSERT/UPDATE 쿼리를 기존 테이블에 맞게 수정하면 됨.
        """
        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            cur.execute("""
                CREATE TABLE IF NOT EXISTS janggi_sessions (
                    session_id UUID PRIMARY KEY,
                    started_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    ended_at TIMESTAMPTZ,
                    status VARCHAR(32) NOT NULL DEFAULT 'RUNNING'
                )
            """)

            cur.execute("""
                CREATE TABLE IF NOT EXISTS janggi_event_log (
                    event_id BIGSERIAL PRIMARY KEY,
                    session_id UUID NOT NULL,
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

                    source VARCHAR(32) NOT NULL,
                    intent VARCHAR(64),
                    confidence DOUBLE PRECISION,

                    team VARCHAR(16),
                    piece VARCHAR(32),

                    master_state VARCHAR(64),
                    accepted BOOLEAN NOT NULL DEFAULT TRUE,

                    payload JSONB,

                    CONSTRAINT fk_event_session
                        FOREIGN KEY(session_id)
                        REFERENCES janggi_sessions(session_id)
                        ON DELETE CASCADE
                )
            """)

            cur.execute("""
                CREATE TABLE IF NOT EXISTS janggi_board_state_log (
                    board_state_id BIGSERIAL PRIMARY KEY,
                    session_id UUID NOT NULL,
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    board_state JSONB NOT NULL,

                    CONSTRAINT fk_board_session
                        FOREIGN KEY(session_id)
                        REFERENCES janggi_sessions(session_id)
                        ON DELETE CASCADE
                )
            """)

            cur.execute("""
                CREATE TABLE IF NOT EXISTS janggi_robot_command_log (
                    command_id UUID PRIMARY KEY,
                    session_id UUID NOT NULL,
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    finished_at TIMESTAMPTZ,

                    command VARCHAR(64) NOT NULL,
                    status VARCHAR(32) NOT NULL DEFAULT 'SENT',

                    source VARCHAR(32),

                    source_row INTEGER,
                    source_col INTEGER,
                    target_row INTEGER,
                    target_col INTEGER,

                    payload JSONB,
                    result_payload JSONB,

                    CONSTRAINT fk_command_session
                        FOREIGN KEY(session_id)
                        REFERENCES janggi_sessions(session_id)
                        ON DELETE CASCADE
                )
            """)

            cur.execute("""
                INSERT INTO janggi_sessions(session_id, status)
                VALUES (%s, 'RUNNING')
                ON CONFLICT (session_id) DO NOTHING
            """, (self.session_id,))

            conn.commit()
            cur.close()
            conn.close()

            self.db_ready = True
            self.get_logger().info("[DB] database initialization complete")

        except Exception as e:
            self.db_ready = False
            self.get_logger().error(f"[DB] initialization failed: {e}")
            self.get_logger().warning(
                "[DB] ROS2 통신은 계속 동작합니다. DB 연결 후 노드를 재시작하세요."
            )

    def log_input_event(
        self,
        source: str,
        data: Dict[str, Any],
        accepted: bool,
    ):
        if not self.db_ready:
            return

        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            cur.execute("""
                INSERT INTO janggi_event_log (
                    session_id,
                    source,
                    intent,
                    confidence,
                    team,
                    piece,
                    master_state,
                    accepted,
                    payload
                )
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s)
            """, (
                self.session_id,
                source,
                data.get("intent"),
                self.safe_float(data.get("confidence")),
                data.get("team"),
                data.get("piece"),
                self.state,
                accepted,
                Json(data),
            ))

            conn.commit()
            cur.close()
            conn.close()

        except Exception as e:
            self.get_logger().error(f"[DB] input log failed: {e}")

    def log_board_state(self, board_state: Dict[str, Any]):
        if not self.db_ready:
            return

        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            cur.execute("""
                INSERT INTO janggi_board_state_log (
                    session_id,
                    board_state
                )
                VALUES (%s,%s)
            """, (
                self.session_id,
                Json(board_state),
            ))

            conn.commit()
            cur.close()
            conn.close()

        except Exception as e:
            self.get_logger().error(f"[DB] board state log failed: {e}")

    def log_robot_command(
        self,
        command_id: str,
        command_data: Dict[str, Any],
    ):
        if not self.db_ready:
            return

        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            cur.execute("""
                INSERT INTO janggi_robot_command_log (
                    command_id,
                    session_id,
                    command,
                    status,
                    source,
                    source_row,
                    source_col,
                    target_row,
                    target_col,
                    payload
                )
                VALUES (%s,%s,%s,'SENT',%s,%s,%s,%s,%s,%s)
            """, (
                command_id,
                self.session_id,
                command_data.get("command"),
                command_data.get("source"),
                command_data.get("source_row"),
                command_data.get("source_col"),
                command_data.get("target_row"),
                command_data.get("target_col"),
                Json(command_data),
            ))

            conn.commit()
            cur.close()
            conn.close()

        except Exception as e:
            self.get_logger().error(f"[DB] robot command log failed: {e}")

    def update_robot_result(
        self,
        command_id: str,
        success: bool,
        result_data: Dict[str, Any],
    ):
        if not self.db_ready:
            return

        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            status = "SUCCESS" if success else "FAILED"

            cur.execute("""
                UPDATE janggi_robot_command_log
                SET
                    finished_at = NOW(),
                    status = %s,
                    result_payload = %s
                WHERE command_id = %s
            """, (
                status,
                Json(result_data),
                command_id,
            ))

            conn.commit()
            cur.close()
            conn.close()

        except Exception as e:
            self.get_logger().error(f"[DB] robot result update failed: {e}")

    def close_session(self):
        if not self.db_ready:
            return

        try:
            conn = self.get_db_connection()
            cur = conn.cursor()

            cur.execute("""
                UPDATE janggi_sessions
                SET ended_at = NOW(),
                    status = 'ENDED'
                WHERE session_id = %s
            """, (self.session_id,))

            conn.commit()
            cur.close()
            conn.close()

        except Exception as e:
            self.get_logger().error(f"[DB] session close failed: {e}")

    # ========================================================
    # ROS2 INPUT CALLBACK
    # ========================================================
    def gesture_callback(self, msg: String):
        data = self.parse_json_message(msg.data, "gesture")
        if data is None:
            return

        data["source"] = "gesture"

        intent = str(data.get("intent", "")).upper()
        confidence = self.safe_float(data.get("confidence"), 0.0)

        accepted = True

        # 같은 제스처 연속 중복 방지
        now = time.monotonic()
        if (
            intent == self.last_gesture_intent
            and now - self.last_gesture_time < self.gesture_debounce_sec
        ):
            accepted = False

        if confidence < self.min_gesture_confidence:
            accepted = False

        self.log_input_event("gesture", data, accepted)

        if not accepted:
            return

        self.last_gesture_intent = intent
        self.last_gesture_time = now

        self.get_logger().info(
            f"[GESTURE] intent={intent}, confidence={confidence:.2f}"
        )

        self.process_input("gesture", data)

    def voice_callback(self, msg: String):
        data = self.parse_json_message(msg.data, "voice")
        if data is None:
            return

        data["source"] = "voice"

        intent = str(data.get("intent", "")).upper()
        confidence = self.safe_float(data.get("confidence"), 0.0)

        accepted = confidence >= self.min_voice_confidence

        self.log_input_event("voice", data, accepted)

        if not accepted:
            self.get_logger().warning(
                f"[VOICE] low confidence: {confidence:.2f}"
            )
            return

        self.get_logger().info(
            f"[VOICE] intent={intent}, confidence={confidence:.2f}"
        )

        self.process_input("voice", data)

    def board_state_callback(self, msg: String):
        data = self.parse_json_message(msg.data, "vision")
        if data is None:
            return

        if not isinstance(data.get("pieces", []), list):
            self.get_logger().warning(
                "[VISION] board_state['pieces'] must be a list"
            )
            return

        self.board_state = data

        self.get_logger().info(
            f"[VISION] board state updated: "
            f"{len(self.board_state.get('pieces', []))} pieces"
        )

        self.log_board_state(self.board_state)

    def robot_result_callback(self, msg: String):
        data = self.parse_json_message(msg.data, "robot_result")
        if data is None:
            return

        command_id = str(data.get("command_id", ""))
        success = bool(data.get("success", False))

        if not command_id:
            self.get_logger().warning(
                "[ROBOT RESULT] command_id missing"
            )
            return

        self.update_robot_result(
            command_id=command_id,
            success=success,
            result_data=data,
        )

        # 현재 실행 중인 명령의 결과인지 확인
        if command_id != self.active_robot_command_id:
            self.get_logger().warning(
                "[ROBOT RESULT] old/unknown command result received"
            )
            return

        finished_command = self.active_robot_command_type

        self.robot_busy = False
        self.active_robot_command_id = None
        self.active_robot_command_type = None

        if success:
            self.get_logger().info(
                f"[ROBOT RESULT] success: {finished_command}"
            )

            # 실제 수 실행이 끝났으면 초기 상태로
            if finished_command in ("PICK_PLACE", "CAPTURE"):
                self.reset_selection()

        else:
            self.get_logger().error(
                f"[ROBOT RESULT] failed: {finished_command}"
            )

        self.publish_status()

    # ========================================================
    # INPUT -> MASTER STATE MACHINE
    # ========================================================
    def process_input(
        self,
        source: str,
        data: Dict[str, Any],
    ):
        intent = str(data.get("intent", "")).upper()

        # ----------------------------------------------------
        # STOP은 언제든 최우선
        # ----------------------------------------------------
        if intent == "STOP":
            self.send_robot_command(
                command="STOP",
                source=source,
                busy=False,
            )
            return

        # 로봇 움직임 중에는 새 일반 명령 무시
        if self.robot_busy:
            self.get_logger().warning(
                f"[MASTER] robot busy -> ignore {intent}"
            )
            return

        # ----------------------------------------------------
        # IDLE: 음성 등으로 말 종류 선택
        # ----------------------------------------------------
        if self.state == "IDLE":

            if intent != "SELECT_PIECE":
                self.get_logger().warning(
                    f"[MASTER] IDLE에서 처리할 수 없는 명령: {intent}"
                )
                return

            team = str(data.get("team", "CHO")).upper()
            piece = str(data.get("piece", "")).upper()

            if not piece:
                self.get_logger().warning(
                    "[MASTER] SELECT_PIECE requires piece"
                )
                return

            candidates = self.find_piece_candidates(
                team=team,
                piece=piece,
            )

            if not candidates:
                self.get_logger().warning(
                    f"[MASTER] {team} {piece} candidate not found"
                )
                return

            self.selected_team = team
            self.selected_piece = piece
            self.piece_candidates = candidates
            self.current_candidate_index = 0

            self.state = "SOURCE_BROWSING"

            self.hover_current_piece(source)
            self.publish_status()
            return

        # ----------------------------------------------------
        # SOURCE_BROWSING:
        # 같은 종류 말 중 하나를 로봇 hover로 보여줌
        # ----------------------------------------------------
        if self.state == "SOURCE_BROWSING":

            if intent == "NEXT":
                self.current_candidate_index = (
                    self.current_candidate_index + 1
                ) % len(self.piece_candidates)

                self.hover_current_piece(source)
                self.publish_status()
                return

            if intent == "PREVIOUS":
                self.current_candidate_index = (
                    self.current_candidate_index - 1
                ) % len(self.piece_candidates)

                self.hover_current_piece(source)
                self.publish_status()
                return

            if intent == "CONFIRM":
                selected = self.get_current_piece()

                if selected is None:
                    return

                # TODO:
                # 실제 장기 규칙 엔진이 추가되면 여기서 합법 수 계산
                self.legal_destinations = self.get_legal_destinations(
                    selected
                )

                if not self.legal_destinations:
                    self.get_logger().warning(
                        "[MASTER] legal destination 없음. "
                        "장기 Rule Engine 연결이 필요합니다."
                    )
                    return

                self.current_destination_index = 0
                self.state = "DESTINATION_BROWSING"

                self.hover_current_destination(source)
                self.publish_status()
                return

            if intent in ("CANCEL", "BACK"):
                self.reset_selection()
                return

        # ----------------------------------------------------
        # DESTINATION_BROWSING:
        # 합법 목적지 후보를 Robot hover로 순환
        # ----------------------------------------------------
        if self.state == "DESTINATION_BROWSING":

            if intent == "NEXT":
                self.current_destination_index = (
                    self.current_destination_index + 1
                ) % len(self.legal_destinations)

                self.hover_current_destination(source)
                self.publish_status()
                return

            if intent == "PREVIOUS":
                self.current_destination_index = (
                    self.current_destination_index - 1
                ) % len(self.legal_destinations)

                self.hover_current_destination(source)
                self.publish_status()
                return

            if intent == "CONFIRM":
                self.execute_selected_move(source)
                return

            if intent in ("CANCEL", "BACK"):
                self.state = "SOURCE_BROWSING"
                self.hover_current_piece(source)
                self.publish_status()
                return

    # ========================================================
    # GAME / BOARD HELPER
    # ========================================================
    def find_piece_candidates(
        self,
        team: str,
        piece: str,
    ) -> List[Dict[str, Any]]:

        result = []

        for item in self.board_state.get("pieces", []):
            item_team = str(item.get("team", "")).upper()
            item_piece = str(item.get("piece", "")).upper()

            if item_team == team and item_piece == piece:
                result.append(item)

        # 좌->우 / 위->아래 정렬
        result.sort(
            key=lambda x: (
                int(x.get("row", 0)),
                int(x.get("col", 0)),
            )
        )

        return result

    def get_current_piece(self) -> Optional[Dict[str, Any]]:
        if not self.piece_candidates:
            return None

        index = self.current_candidate_index % len(
            self.piece_candidates
        )

        return self.piece_candidates[index]

    def get_current_destination(self) -> Optional[Dict[str, int]]:
        if not self.legal_destinations:
            return None

        index = self.current_destination_index % len(
            self.legal_destinations
        )

        return self.legal_destinations[index]

    def get_legal_destinations(
        self,
        selected_piece: Dict[str, Any],
    ) -> List[Dict[str, int]]:
        """
        TODO: 장기 Rule Engine 연결 지점.

        현재는 Vision이 board_state에 선택 말별 legal_moves를 같이 넣는 경우만 지원.
        예:
        {
          "team": "CHO",
          "piece": "MA",
          "row": 0,
          "col": 1,
          "legal_moves": [
            {"row": 2, "col": 2},
            {"row": 2, "col": 0}
          ]
        }

        나중에는 janggi_rule_engine.py에서 현재 board_state를 바탕으로 계산하는 것을 추천.
        """
        legal_moves = selected_piece.get("legal_moves", [])

        valid_moves = []

        for move in legal_moves:
            if "row" in move and "col" in move:
                valid_moves.append({
                    "row": int(move["row"]),
                    "col": int(move["col"]),
                })

        return valid_moves

    # ========================================================
    # ROBOT COMMAND
    # ========================================================
    def hover_current_piece(self, source: str):
        piece = self.get_current_piece()

        if piece is None:
            return

        self.send_robot_command(
            command="HOVER",
            source=source,
            row=int(piece["row"]),
            col=int(piece["col"]),
            extra={
                "hover_type": "SOURCE",
                "team": self.selected_team,
                "piece": self.selected_piece,
                "candidate_index": self.current_candidate_index,
            },
        )

    def hover_current_destination(self, source: str):
        destination = self.get_current_destination()

        if destination is None:
            return

        self.send_robot_command(
            command="HOVER",
            source=source,
            row=destination["row"],
            col=destination["col"],
            extra={
                "hover_type": "DESTINATION",
                "destination_index": self.current_destination_index,
            },
        )

    def execute_selected_move(self, source: str):
        piece = self.get_current_piece()
        destination = self.get_current_destination()

        if piece is None or destination is None:
            return

        target_piece = self.find_piece_at(
            destination["row"],
            destination["col"],
        )

        command = "PICK_PLACE"

        # 목적지에 상대 말이 있으면 CAPTURE
        if target_piece is not None:
            if (
                str(target_piece.get("team", "")).upper()
                != self.selected_team
            ):
                command = "CAPTURE"
            else:
                self.get_logger().error(
                    "[MASTER] destination occupied by same team"
                )
                return

        self.state = "EXECUTING"

        self.send_robot_command(
            command=command,
            source=source,
            source_row=int(piece["row"]),
            source_col=int(piece["col"]),
            target_row=destination["row"],
            target_col=destination["col"],
            extra={
                "team": self.selected_team,
                "piece": self.selected_piece,
            },
        )

        self.publish_status()

    def find_piece_at(
        self,
        row: int,
        col: int,
    ) -> Optional[Dict[str, Any]]:
        for item in self.board_state.get("pieces", []):
            if (
                int(item.get("row", -1)) == int(row)
                and int(item.get("col", -1)) == int(col)
            ):
                return item

        return None

    def send_robot_command(
        self,
        command: str,
        source: str,
        row: Optional[int] = None,
        col: Optional[int] = None,
        source_row: Optional[int] = None,
        source_col: Optional[int] = None,
        target_row: Optional[int] = None,
        target_col: Optional[int] = None,
        extra: Optional[Dict[str, Any]] = None,
        busy: bool = True,
    ):

        command_id = str(uuid.uuid4())

        data = {
            "command_id": command_id,
            "session_id": self.session_id,
            "command": command,
            "source": source,
            "row": row,
            "col": col,
            "source_row": source_row,
            "source_col": source_col,
            "target_row": target_row,
            "target_col": target_col,
        }

        if extra:
            data.update(extra)

        msg = String()
        msg.data = json.dumps(
            data,
            ensure_ascii=False,
        )

        self.robot_cmd_pub.publish(msg)

        self.get_logger().info(
            f"[ROBOT CMD] {msg.data}"
        )

        self.log_robot_command(
            command_id=command_id,
            command_data=data,
        )

        # STOP은 별도 처리하므로 busy를 잡지 않음
        if busy:
            self.robot_busy = True
            self.active_robot_command_id = command_id
            self.active_robot_command_type = command

    # ========================================================
    # STATUS
    # ========================================================
    def publish_status(self):
        data = {
            "session_id": self.session_id,
            "state": self.state,
            "robot_busy": self.robot_busy,

            "selected_team": self.selected_team,
            "selected_piece": self.selected_piece,

            "candidate_index": self.current_candidate_index,
            "candidate_count": len(self.piece_candidates),

            "destination_index": self.current_destination_index,
            "destination_count": len(self.legal_destinations),
        }

        msg = String()
        msg.data = json.dumps(
            data,
            ensure_ascii=False,
        )

        self.status_pub.publish(msg)

    def reset_selection(self):
        self.state = "IDLE"

        self.selected_piece = ""
        self.piece_candidates = []
        self.current_candidate_index = 0

        self.legal_destinations = []
        self.current_destination_index = 0

        self.robot_busy = False
        self.active_robot_command_id = None
        self.active_robot_command_type = None

        self.get_logger().info(
            "[MASTER] reset -> IDLE"
        )

        self.publish_status()

    # ========================================================
    # UTIL
    # ========================================================
    def parse_json_message(
        self,
        raw: str,
        source_name: str,
    ) -> Optional[Dict[str, Any]]:
        try:
            data = json.loads(raw)

            if not isinstance(data, dict):
                raise ValueError(
                    "message must be JSON object"
                )

            return data

        except Exception as e:
            self.get_logger().error(
                f"[{source_name}] invalid JSON: {e}, raw={raw}"
            )
            return None

    def safe_float(
        self,
        value,
        default=None,
    ):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default


# ============================================================
# main
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    node = JanggiMasterNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close_session()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
