from flask import Flask, request, jsonify, send_from_directory
from pymongo import MongoClient
from datetime import datetime
from flask_socketio import SocketIO, emit, join_room
import uuid
from db_config import MONGODB_URI

app = Flask(__name__)

# [추가됨] SocketIO 초기화 (CORS 허용으로 외부 웹사이트 접속 허용)
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
@app.route('/index.html')
def serve_index():
    return send_from_directory(app.root_path, 'index.html')

@app.route('/style.css')
@app.route('/app.js')
def serve_frontend_asset():
    return send_from_directory(app.root_path, request.path.lstrip('/'))

# MongoDB 연결 정보는 로컬 전용 db_config.py에서 읽습니다.
client = MongoClient(MONGODB_URI)
db = client.janggi
boards_col = db.boards
moves_col = db.moves
sessions_col = db.sessions

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

# === 웹소켓 이벤트 (새로 추가된 부분) ===

@socketio.on('join')
def on_join(data):
    """
    웹사이트가 처음 열릴 때 이 방(session_id)에 접속하겠다고 서버에 알립니다.
    이 방에 들어와 있어야 서버가 업데이트 방송을 해줄 때 들을 수 있습니다.
    """
    session_id = data.get('sessionId')
    if session_id:
        join_room(session_id)
        print(f"웹사이트 클라이언트가 {session_id} 방에 접속했습니다.")

# === REST API (YOLO 및 기존 기능) ===

@app.route('/api/session', methods=['POST'])
def create_session():
    session_id = str(uuid.uuid4())[:8]
    session = {
        'sessionId': session_id,
        'status': 'playing',
        'winner': None,
        'startTime': datetime.now(),
        'endTime': None
    }
    sessions_col.insert_one(session)
    
    board_doc = {
        'sessionId': session_id,
        'board': create_initial_board(),
        'currentTurn': 'black',
        'createdAt': datetime.now(),
        'updatedAt': datetime.now()
    }
    boards_col.insert_one(board_doc)
    
    return jsonify({'sessionId': session_id, 'message': '새 게임이 시작되었습니다'})

@app.route('/api/board/<session_id>', methods=['GET'])
def get_board(session_id):
    board = boards_col.find_one({'sessionId': session_id}, {'_id': 0})
    if not board:
        return jsonify({'error': '세션을 찾을 수 없습니다'}), 404
    return jsonify(board)

@app.route('/api/board/<session_id>', methods=['PUT'])
def update_board(session_id):
    """
    YOLO가 호출하는 API입니다. DB를 업데이트 한 직후, 
    웹소켓을 통해 웹사이트로 변경된 데이터를 실시간 전송(Push)합니다.
    """
    data = request.json
    result = boards_col.update_one(
        {'sessionId': session_id},
        {'$set': {
            'board': data['board'],
            'currentTurn': data.get('currentTurn', 'red'),
            'updatedAt': datetime.now()
        }}
    )
    if result.modified_count == 0:
        return jsonify({'error': '업데이트 실패'}), 400
    
    # [핵심 로직] DB 업데이트 성공 시, 연결된 웹사이트(해당 session_id 방)로 데이터 쏘기
    # 이벤트 이름: 'board_updated'
    socketio.emit('board_updated', data, room=session_id)
    
    return jsonify({'message': '보드가 업데이트되고 웹사이트로 실시간 전송되었습니다'})

@app.route('/api/move/<session_id>', methods=['POST'])
def add_move(session_id):
    data = request.json
    last_move = moves_col.find_one(
        {'sessionId': session_id},
        sort=[('moveNumber', -1)]
    )
    move_number = (last_move['moveNumber'] + 1) if last_move else 1
    
    move = {
        'sessionId': session_id,
        'moveNumber': move_number,
        'piece': data['piece'],
        'from': data['from'],
        'to': data['to'],
        'captured': data.get('captured'),
        'timestamp': datetime.now()
    }
    moves_col.insert_one(move)
    
    # 이동 기록도 실시간으로 보여주고 싶다면 아래 주석을 푸세요.
    # socketio.emit('move_added', move_number, room=session_id)
    
    return jsonify({'message': f'{move_number}번째 이동이 기록되었습니다'})

@app.route('/api/move/<session_id>', methods=['GET'])
def get_moves(session_id):
    moves = list(moves_col.find(
        {'sessionId': session_id},
        {'_id': 0}
    ).sort('moveNumber', 1))
    return jsonify(moves)

@app.route('/api/session/<session_id>/end', methods=['PUT'])
def end_session(session_id):
    data = request.json
    sessions_col.update_one(
        {'sessionId': session_id},
        {'$set': {
            'status': 'finished',
            'winner': data.get('winner'),
            'endTime': datetime.now()
        }}
    )
    return jsonify({'message': '게임이 종료되었습니다'})

if __name__ == '__main__':
    # Flask 기본 app.run() 대신 웹소켓용 socketio.run()을 사용합니다.
    # host='0.0.0.0' 으로 두면 외부(YOLO 컴퓨터, 다른 핸드폰 등)에서도 접속 가능합니다.
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)