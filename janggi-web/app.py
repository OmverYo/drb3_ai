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

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)