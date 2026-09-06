from flask import Flask, request, jsonify, send_from_directory
from pymongo import MongoClient
from datetime import datetime
from flask_socketio import SocketIO
from db_config import MONGODB_URI  # 설정 파일에서 URI 불러오기
import os

app = Flask(__name__)
# 모든 외부 접속 허용
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
@app.route('/index.html')
def serve_index():
    return send_from_directory(app.root_path, 'index.html')

@app.route('/style.css')
@app.route('/app.js')
def serve_frontend_asset():
    return send_from_directory(app.root_path, request.path.lstrip('/'))

# MongoDB 연결 (db_config.py의 변수 사용)
client = MongoClient(MONGODB_URI)
db = client.janggi
boards_col = db.boards

# 데모용 단일 문서 ID
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

# 서버 시작 시 데모 보드가 없으면 생성
with app.app_context():
    if not boards_col.find_one({'doc_id': DEMO_DOC_ID}):
        boards_col.insert_one({
            'doc_id': DEMO_DOC_ID,
            'board': create_initial_board(),
            'currentTurn': 'red',
            'updatedAt': datetime.now()
        })

@app.route('/api/board', methods=['GET'])
def get_board():
    """현재 장기판 상태 조회 (세션 없이 데모 보드 바로 리턴)"""
    board = boards_col.find_one({'doc_id': DEMO_DOC_ID}, {'_id': 0})
    return jsonify(board)

@app.route('/api/board', methods=['PUT'])
def update_board():
    """YOLO에서 호출하여 DB 업데이트 후 전체 웹사이트에 실시간 전송"""
    data = request.json
    boards_col.update_one(
        {'doc_id': DEMO_DOC_ID},
        {'$set': {
            'board': data['board'],
            'currentTurn': data.get('currentTurn', 'red'),
            'updatedAt': datetime.now()
        }}
    )
    
    # 방(Room) 구분 없이 접속한 모든 기기에 변경사항 브로드캐스트
    socketio.emit('board_updated', data)
    
    return jsonify({'message': '보드가 업데이트되고 전체 전송되었습니다'})

# === 실시간 업데이트 테스트용 API ===
@app.route('/api/test_move', methods=['GET'])
def test_move():
    """웹브라우저에서 이 주소로 접속하면 가운데 초나라 쫄이 앞뒤로 움직입니다."""
    board_doc = boards_col.find_one({'doc_id': DEMO_DOC_ID})
    if not board_doc:
        return jsonify({'error': '보드가 없습니다'}), 404
    
    board = board_doc['board']
    
    # 초나라 가운데 쫄(jol_green) 위치를 토글 (row 3 <-> row 4)
    if board[3][4] == 'jol_green':
        board[3][4] = None
        board[4][4] = 'jol_green'
        turn = 'red'
    else:
        board[4][4] = None
        board[3][4] = 'jol_green'
        turn = 'green'

    boards_col.update_one(
        {'doc_id': DEMO_DOC_ID},
        {'$set': {
            'board': board,
            'currentTurn': turn,
            'updatedAt': datetime.now()
        }}
    )
    
    data = {'board': board, 'currentTurn': turn}
    socketio.emit('board_updated', data)
    
    return jsonify({'message': '테스트 이동 성공! 웹사이트 화면이 실시간으로 변했는지 확인하세요.', 'turn': turn})

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)