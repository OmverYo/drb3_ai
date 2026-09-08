from flask import Flask, request, jsonify, send_from_directory
from pymongo import MongoClient
from datetime import datetime
from flask_socketio import SocketIO
from db_config import MONGODB_URI
import os
import re

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
@app.route('/index.html')
def serve_index():
    return send_from_directory(app.root_path, 'index.html')

@app.route('/style.css')
@app.route('/app.js')
def serve_frontend_asset():
    if request.path == '/app.js':
        return send_from_directory(app.root_path, 'app.js')
    elif request.path == '/style.css':
        return send_from_directory(app.root_path, 'style.css')
    return send_from_directory(app.root_path, request.path.lstrip('/'))

client = MongoClient(MONGODB_URI)
db = client.janggi
boards_col = db.boards

DEMO_DOC_ID = "demo_board"

def create_initial_board():
    board = [[None for _ in range(9)] for _ in range(10)]
    board[0] = ["cha_green", "ma_green", "sang_green", "sa_green", None, "sa_green", "sang_green", "ma_green", "cha_green"]
    board[1][4] = "wang_green"
    board[2][1] = "po_green"; board[2][7] = "po_green"
    board[3][0] = "jol_green"; board[3][2] = "jol_green"; board[3][4] = "jol_green"; board[3][6] = "jol_green"; board[3][8] = "jol_green"
    board[9] = ["cha_red", "ma_red", "sang_red", "sa_red", None, "sa_red", "sang_red", "ma_red", "cha_red"]
    board[8][4] = "wang_red"
    board[7][1] = "po_red"; board[7][7] = "po_red"
    board[6][0] = "jol_red"; board[6][2] = "jol_red"; board[6][4] = "jol_red"; board[6][6] = "jol_red"; board[6][8] = "jol_red"
    return board

with app.app_context():
    if not boards_col.find_one({'doc_id': DEMO_DOC_ID}):
        boards_col.insert_one({
            'doc_id': DEMO_DOC_ID,
            'board': create_initial_board(),
            'currentTurn': 'red',
            'lastMove': '대국 시작',
            'updatedAt': datetime.now()
        })

@app.route('/api/board', methods=['GET'])
def get_board():
    board = boards_col.find_one({'doc_id': DEMO_DOC_ID}, {'_id': 0, 'appliedTaskIds': 0})
    return jsonify(board)

@app.route('/api/board', methods=['PUT'])
def update_board():
    data = request.get_json(silent=True)
    board = data.get('board') if isinstance(data, dict) else None
    if (not isinstance(board, list) or len(board) != 10
            or not all(isinstance(row, list) and len(row) == 9
                       and all(cell is None or (isinstance(cell, str) and cell in PIECE_KEYS)
                               for cell in row) for row in board)):
        return jsonify(error='board는 10행 9열의 말 클래스 배열이어야 합니다.'), 400
    # YOLO에서 보내주는 lastMove 텍스트를 받음 (없으면 기본값)
    last_move_text = data.get('lastMove', '기물 이동 감지됨')
    
    boards_col.update_one(
        {'doc_id': DEMO_DOC_ID},
        {'$set': {
            'board': data['board'],
            'boardSource': 'camera',
            'currentTurn': data.get('currentTurn', 'red'),
            'lastMove': last_move_text,
            'updatedAt': datetime.now()
        }, '$unset': {'taskControlled': '', 'appliedTaskIds': ''}}
    )
    
    # 카메라가 전달한 실제 장기판만 웹으로 전달한다.
    latest = boards_col.find_one({'doc_id': DEMO_DOC_ID},
                                 {'_id': 0, 'appliedTaskIds': 0})
    latest['updatedAt'] = latest['updatedAt'].isoformat()
    socketio.emit('board_updated', latest)
    return jsonify({'message': '카메라 보드 업데이트 완료'})

@app.route('/api/test_move', methods=['GET'])
def test_move():
    return jsonify(error='장기판은 카메라 API로만 갱신합니다.'), 409

# [Task JSON 추가] robot_control의 JSON을 events 컬렉션에 기록한다.
# task_id를 MongoDB 기본 고유키로 사용해 Task당 문서 하나만 저장한다.
from pymongo.errors import PyMongoError
import hmac

events_col = db.events

PIECE_KEYS = {f'{name}_{color}' for name in
              ('cha', 'ma', 'sang', 'sa', 'wang', 'po', 'jol')
              for color in ('red', 'green')}


def board_position(value):
    """1부터 시작하는 {row, col} 좌표. 장기판 외부 목적지는 별도 처리."""
    if not isinstance(value, dict):
        return None
    row, col = value.get('row'), value.get('col')
    if type(row) is int and type(col) is int and 1 <= row <= 10 and 1 <= col <= 9:
        return {'row': row, 'col': col}
    return None


def normalize_task(data, previous=None, camera_lookup=False):
    previous = previous or {}
    task = {name: previous.get(name) for name in ('mode', 'piece', 'before', 'after')}
    task['mode'] = data.get('mode', task['mode'])
    raw_piece = data.get('piece') or task['piece']
    # robot_control의 voice 형식: 클래스@행,열 (예: wang_green@2,5).
    match = re.fullmatch(r'([a-z]+_(?:red|green))@(\d+),(\d+)', raw_piece if isinstance(raw_piece, str) else '')
    if match:
        task['piece'] = match[1] if match[1] in PIECE_KEYS else None
        encoded_before = board_position({'row': int(match[2]), 'col': int(match[3])})
    else:
        task['piece'] = raw_piece if isinstance(raw_piece, str) and raw_piece in PIECE_KEYS else None
        encoded_before = None
    task['before'] = board_position(data.get('before')) or board_position(task['before']) or encoded_before
    if 'after' in data:
        after = data['after']
        # 현재 송신 규약에서 명시적인 after:null은 장기판 밖 bucket을 의미한다.
        # 필드 자체가 생략된 경우에는 기존 목적지를 유지한다.
        if after is None or after == 'bucket' or after == {'type': 'bucket'}:
            task['after'] = {'type': 'bucket'}
        else:
            task['after'] = board_position(after)
            if task['after'] is None:
                raise ValueError('after는 {row, col}, null 또는 {type: bucket}이어야 합니다.')
    # Task 시작 시점에만 카메라를 참조한다. 완료 시점 위치를 출발점으로 쓰지 않는다.
    need_voice_position = (task['mode'] == 'voice' and task['piece']
                           and task['before'] is None)
    need_piece_name = not task['piece'] and task['before'] is not None
    if camera_lookup and (need_voice_position or need_piece_name):
        doc = boards_col.find_one({'doc_id': DEMO_DOC_ID}) or {}
        if doc.get('boardSource') == 'camera':
            if need_voice_position:
                positions = [
                    {'row': row + 1, 'col': col + 1}
                    for row, cells in enumerate(doc['board'])
                    for col, piece in enumerate(cells) if piece == task['piece']
                ]
                # 같은 팀/종류의 말이 하나일 때만 출발 위치를 확정한다.
                if len(positions) == 1:
                    task['before'] = positions[0]
            if need_piece_name:
                pos = task['before']
                candidate = doc['board'][pos['row'] - 1][pos['col'] - 1]
                if candidate in PIECE_KEYS:
                    task['piece'] = candidate
    return task


def normalize_existing_tasks():
    """기존 voice 출발 좌표와 완료된 bucket 기록만 복원. 과거 말 이름은 추측하지 않는다."""
    query = {'$or': [
        {'piece': {'$regex': r'^[a-z]+_(red|green)@\d+,\d+$'}},
        {'status': 'completed', 'after': {'$eq': None, '$exists': True}}
    ]}
    for old in events_col.find(query):
        normalized = normalize_task(old)
        # 서버 시작 중 들어온 갱신을 덮어쓰지 않도록 읽은 상태를 확인한다.
        events_col.update_one(
            {'_id': old['_id'], 'status': old.get('status'),
             'piece': old.get('piece'), 'before': old.get('before'), 'after': old.get('after')},
            {'$set': normalized})


@app.route('/api/events', methods=['POST'])
def receive_task_event():
    token = os.getenv('JANGGI_API_TOKEN', '')
    if token and not hmac.compare_digest(
            request.headers.get('Authorization', ''), 'Bearer ' + token):
        return jsonify(ok=False, error='Unauthorized'), 401
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify(ok=False), 400
    kind = data.get('kind')
    key = data.get('task_id') if kind == 'task' else data.get('robot_id')
    if (kind not in ('task', 'mode') or not isinstance(key, str) or not key
            or not all(data.get(name) for name in ('event_id', 'occurred_at', 'mode'))):
        return jsonify(ok=False), 400
    if kind == 'task' and data.get('status') not in ('started', 'completed', 'failed'):
        return jsonify(ok=False, error='status는 started/completed/failed입니다.'), 400
    try:
        if kind == 'mode':
            states = db.robot_states
            states.update_one({'_id': key}, {'$setOnInsert': {'robot_id': key}}, upsert=True)
            states.update_one(
                {'_id': key, '$or': [{'updated_at': {'$exists': False}},
                                   {'updated_at': {'$lt': data['occurred_at']}}]},
                {'$set': {'mode': data['mode'], 'updated_at': data['occurred_at']}})
        else:
            previous = events_col.find_one({'_id': key})
            if not previous or previous.get('status') == 'started':
                task = normalize_task(data, previous, camera_lookup=data['status'] == 'started')
                events_col.update_one({'_id': key}, {'$setOnInsert': {
                    **task, 'status': 'started', 'error': None}}, upsert=True)
                if data['status'] == 'started':
                    events_col.update_one(
                        {'_id': key, 'status': 'started', 'started_at': {'$exists': False}},
                        {'$set': {**task, 'started_at': data['occurred_at']}})
                else:
                    # 중복 완료나 늦은 started가 최종 상태와 말 이름을 덮어쓰지 않는다.
                    events_col.update_one({'_id': key, 'status': 'started'}, {'$set': {
                        **task, 'status': data['status'], 'error': data.get('error'),
                        'finished_at': data['occurred_at']}})
            stored = events_col.find_one({'_id': key})
            data = {**data, **{name: stored.get(name) for name in
                    ('status', 'mode', 'piece', 'before', 'after', 'error')}}
    except ValueError as exc:
        return jsonify(ok=False, error=str(exc)), 400
    except PyMongoError:
        return jsonify(ok=False), 503
    try:
        # Task는 기록 알림만 보낸다. 보드 DB 변경/board_updated 전송은 하지 않는다.
        socketio.emit('task_event', data)
    except Exception:
        pass
    return jsonify(ok=True, event_id=data['event_id'])


normalize_existing_tasks()

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)