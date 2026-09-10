"""기존 robot_control 패키지 안에서 사용하는 JSON 생성 + Flask 전송 모듈.

robot_control.py:
    from robot_control.task_json import TaskJsonPublisher
    self.task_json = TaskJsonPublisher(self)  # self.mode 설정 후 한 번 생성
    기존 start(), finish(), fail_safely() 호출 유지.

setup.py의 console_scripts에 추가:
    'task_json = robot_control.task_json:main'

별도 터미널 또는 launch에서 실행:
    ros2 run robot_control task_json --ros-args \
        -p api_base_url:=http://192.168.0.10:5000

Flask 계약: POST /api/events, 성공 응답 JSON {"ok": true}.
기존 janggi_api/api_node 대신 이 노드 하나만 실행한다.
HTTP는 이 모듈을 import한 제어 프로세스에서는 실행하지 않는다.
ACK는 Flask가 이벤트를 정상 접수했다는 뜻이다.
전송 실패 시 ACK하지 않아 기존 publisher가 동일 JSON을 재전송한다.
로봇 프로세스 종료 중에는 pending 파일을 유지하고 재시작 후 전송한다.
"""
import json
import os
import re
import threading
import time
import uuid
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


def coordinate(text):
    match = re.fullmatch(r'\s*(\d+)\s*,\s*(\d+)\s*', str(text))
    if not match:
        raise ValueError(f'Invalid row,col: {text}')
    row, col = map(int, match.groups())
    if not (1 <= row <= 10 and 1 <= col <= 9):
        raise ValueError('Board coordinate out of range')
    return {'row': row, 'col': col}


def vision_command(text):
    match = re.fullmatch(
        r'\s*(\d+\s*,\s*\d+)\s+(?:grap|grab)\s+'
        r'(?:(\d+\s*,\s*\d+)\s+release|(?:\d+\s*,\s*\d+\s+)?bucket)\s*', text)
    if not match:
        raise ValueError('Invalid vision command')
    return coordinate(match[1]), coordinate(match[2]) if match[2] else None


class JsonStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.pending = self.root / 'pending'
        self.sent = self.root / 'sent'
        self.pending.mkdir(parents=True, exist_ok=True)
        self.sent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()

    def put(self, data):
        body = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
        with self.lock:
            name = f'{time.time_ns():020d}_{data["event_id"]}.json'
            temporary = self.pending / (name + '.tmp')
            target = self.pending / name
            with temporary.open('x', encoding='utf-8') as stream:
                stream.write(body)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        return data['event_id']

    def first(self):
        with self.lock:
            paths = sorted(self.pending.glob('*.json'))
            if not paths:
                return None
            return paths[0].read_text(encoding='utf-8')

    def ack(self, event_id):
        event_id = str(uuid.UUID(event_id))
        with self.lock:
            for path in self.pending.glob(f'*_{event_id}.json'):
                os.replace(path, self.sent / path.name)


class TaskJsonPublisher:
    def __init__(self, node):
        from std_msgs.msg import String
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.node = node
        self.String = String
        self.mode = node.mode
        self.robot_id = os.getenv('JANGGI_ROBOT_ID', 'dsr01')
        self.session_id = str(uuid.uuid4())
        self.sequence = 0
        self.active = None
        self.lock = threading.RLock()
        self.store = JsonStore(os.getenv('JANGGI_JSON_DIR',
            '~/.local/state/janggi_json'))
        self.publisher = node.create_publisher(String, '/janggi/task_json', 10)
        self.group = MutuallyExclusiveCallbackGroup()
        self.subscription = node.create_subscription(
            String, '/janggi/task_json_ack', self._on_ack, 10,
            callback_group=self.group)
        self.timer = node.create_timer(1.0, self._send_pending,
                                      callback_group=self.group)
        self._emit('mode')

    def _emit(self, kind, **fields):
        self.sequence += 1
        event = dict(event_id=str(uuid.uuid4()), robot_id=self.robot_id,
                     session_id=self.session_id, sequence=self.sequence,
                     occurred_at=datetime.now(timezone.utc).isoformat(),
                     mode=self.mode, kind=kind, **fields)
        return self.store.put(event)

    def _send_pending(self):
        try:
            body = self.store.first()
            if body is not None:
                self.publisher.publish(self.String(data=body))
        except Exception as error:
            self.node.get_logger().error(f'JSON queue blocked: {error}')

    def _on_ack(self, message):
        try:
            self.store.ack(message.data)
        except Exception as error:
            self.node.get_logger().error(f'Invalid ACK: {error}')

    def start(self, piece=None, before=None, after=None):
        with self.lock:
            if self.active is not None:
                raise RuntimeError('Another task is active')
            task = dict(task_id=str(uuid.uuid4()), piece=piece, before=before,
                        after=after, destination='board' if after else 'bucket')
            self._emit('task', status='started', error=None, **task)
            self.active = task

    def finish(self, status='completed', error=None):
        with self.lock:
            if self.active is None:
                return
            # A terminal write failure must not generate a contradictory failed event.
            task, self.active = self.active, None
            self._emit('task', status=status,
                       error=str(error)[:4000] if error is not None else None, **task)

    def fail_safely(self, error):
        try:
            self.finish('failed', error)
        except Exception as write_error:
            self.node.get_logger().error(f'Could not save failed Task: {write_error}')


def forward_json(body, api_url, token='', timeout_sec=3.0, opener=None):
    """받은 JSON 본문 그대로 HTTP POST. 성공한 경우에만 event_id 반환.

    HTTP 성공뿐 아니라 Flask의 ok=true도 확인한다. 응답 유실 시에도
    같은 event_id가 재전송되므로 Flask에서 event_id 중복 처리가 필요하다.
    """
    data = json.loads(body)
    if not isinstance(data, dict):
        raise ValueError('Task JSON must be an object')
    event_id = str(uuid.UUID(data['event_id']))
    headers = {'Content-Type': 'application/json'}
    if token:
        headers['Authorization'] = 'Bearer ' + token
    request = urllib.request.Request(
        api_url, data=body.encode('utf-8'), headers=headers, method='POST')
    if opener is None:
        opener = urllib.request.urlopen
    with opener(request, timeout=timeout_sec) as response:
        if not 200 <= response.status < 300:
            raise RuntimeError(f'Flask HTTP status: {response.status}')
        reply = json.loads(response.read())
        if not isinstance(reply, dict) or reply.get('ok') is not True:
            raise RuntimeError('Flask response must contain ok=true')
    return event_id


def main(args=None):
    """launch/ros2 run으로 실행되는 독립 HTTP 전송 노드.

    ROS imports/노드 생성을 여기서만 수행하여 robot_control이 이 파일을
    import할 때 rclpy.init()이나 별도 spin이 실행되지 않게 한다.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = Node('task_json_api')
    base_url = node.declare_parameter(
        'api_base_url',
        os.getenv('JANGGI_API_BASE_URL', 'http://127.0.0.1:5000')
    ).value.rstrip('/')
    api_url = base_url + '/api/events'
    token = os.getenv('JANGGI_API_TOKEN', '')
    ack_publisher = node.create_publisher(String, '/janggi/task_json_ack', 10)

    def on_json(message):
        try:
            event_id = forward_json(message.data, api_url, token)
        except Exception as error:
            node.get_logger().warning(
                f'Flask 전송 실패. ACK 없이 JSON 재전송 대기: {error}')
            return
        ack_publisher.publish(String(data=event_id))
        node.get_logger().info(f'Flask 전송 완료: event_id={event_id}')

    # HTTP 대기는 이 별도 프로세스에서만 발생한다.
    # TaskJsonPublisher는 ACK 전까지 원본을 보관하고 1초마다 재발행한다.
    subscription = node.create_subscription(
        String, '/janggi/task_json', on_json, 10)
    node.get_logger().info(f'Task JSON API ready: {api_url}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()