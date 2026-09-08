from flask import Flask, request, jsonify, send_from_directory
from pymongo import MongoClient
from datetime import datetime
from flask_socketio import SocketIO
from db_config import MONGODB_URI
import os

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
    board = boards_col.find_one({'doc_id': DEMO_DOC_ID}, {'_id': 0})
    return jsonify(board)

@app.route('/api/board', methods=['PUT'])
def update_board():
    data = request.json
    # YOLO에서 보내주는 lastMove 텍스트를 받음 (없으면 기본값)
    last_move_text = data.get('lastMove', '기물 이동 감지됨')
    
    boards_col.update_one(
        {'doc_id': DEMO_DOC_ID},
        {'$set': {
            'board': data['board'],
            'currentTurn': data.get('currentTurn', 'red'),
            'lastMove': last_move_text,
            'updatedAt': datetime.now()
        }}
    )
    
    data['lastMove'] = last_move_text
    socketio.emit('board_updated', data)
    return jsonify({'message': '보드 및 이동 기록 업데이트 완료'})

@app.route('/api/test_move', methods=['GET'])
def test_move():
    board_doc = boards_col.find_one({'doc_id': DEMO_DOC_ID})
    if not board_doc:
        return jsonify({'error': '보드가 없습니다'}), 404
    
    board = board_doc['board']
    
    # 테스트용 기물 이동 및 문자열 생성
    if board[3][4] == 'jol_green':
        board[3][4] = None
        board[4][4] = 'jol_green'
        turn = 'red'
        last_move_text = "초나라 卒(졸)을 4-5에서 5-5로 옮김"
    else:
        board[4][4] = None
        board[3][4] = 'jol_green'
        turn = 'green'  
        last_move_text = "초나라 卒(졸)을 5-5에서 4-5로 옮김"

    boards_col.update_one(
        {'doc_id': DEMO_DOC_ID},
        {'$set': {
            'board': board,
            'currentTurn': turn,
            'lastMove': last_move_text,
            'updatedAt': datetime.now()
        }}
    )
    
    data = {'board': board, 'currentTurn': turn, 'lastMove': last_move_text}
    socketio.emit('board_updated', data)
    
    return jsonify({'message': '테스트 이동 성공', 'lastMove': last_move_text})

# [Task JSON 추가] robot_control의 JSON을 events 컬렉션에 기록한다.
# task_id를 MongoDB 기본 고유키로 사용해 Task당 문서 하나만 저장한다.
from pymongo.errors import PyMongoError
import hmac

events_col = db.events


@app.route('/api/events', methods=['POST'])
def receive_task_event():
    token = os.getenv('JANGGI_API_TOKEN', '')
    if token and not hmac.compare_digest(
            request.headers.get('Authorization', ''), 'Bearer ' + token):
        return jsonify(ok=False, error='Unauthorized'), 401
    data = request.get_json(silent=True)
    # 저장에 필요한 필수 항목만 확인한다.
    if not isinstance(data, dict):
        return jsonify(ok=False), 400
    kind = data.get('kind')
    key = data.get('task_id') if kind == 'task' else data.get('robot_id')
    if (kind not in ('task', 'mode') or not isinstance(key, str) or not key
            or not all(data.get(name) for name in ('event_id', 'occurred_at', 'mode'))):
        return jsonify(ok=False), 400
    try:
        if kind == 'mode':
            # 모드는 로봇당 문서 하나로 관리한다. 오래된 재전송은 무시한다.
            states = db.robot_states
            states.update_one({'_id': key}, {'$setOnInsert': {'robot_id': key}}, upsert=True)
            states.update_one(
                {'_id': key, '$or': [{'updated_at': {'$exists': False}},
                                   {'updated_at': {'$lt': data['occurred_at']}}]},
                {'$set': {'mode': data['mode'], 'updated_at': data['occurred_at']}})
        else:
            # Task당 하나만 생성하고, 완료/실패가 오면 같은 문서를 갱신한다.
            task = {name: data.get(name) for name in
                    ('mode', 'piece', 'before', 'after')}
            task.update(status='started', error=None)
            events_col.update_one({'_id': key}, {'$setOnInsert': task}, upsert=True)
            if data['status'] == 'started':
                events_col.update_one(
                    {'_id': key, 'started_at': {'$exists': False}},
                    {'$set': {'started_at': data['occurred_at']}})
            else:
                # 재전송이나 늦게 도착한 started가 완료 상태를 되돌리지 않는다.
                events_col.update_one({'_id': key, 'status': 'started'}, {'$set': {
                    'status': data['status'], 'error': data.get('error'),
                    'finished_at': data['occurred_at']}})
    except PyMongoError:
        return jsonify(ok=False), 503
    # 웹 클라이언트는 task_event 이벤트를 구독해 기록/모드를 표시할 수 있다.
    # 기존 board_updated 및 장기판 갱신 동작은 그대로 유지한다.
    try:
        socketio.emit('task_event', data)
    except Exception:
        pass  # DB 저장은 완료됐으므로 웹 알림 실패로 재전송하지 않는다.
    return jsonify(ok=True, event_id=data['event_id'])


if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
