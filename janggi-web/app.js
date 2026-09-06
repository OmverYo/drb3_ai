// === [1] 설정 및 웹소켓 연결 ===
const API_BASE = '';
const socket = io();
let currentSessionId = localStorage.getItem('janggiSessionId');

// DB의 문자열 키값을 새 디자인의 텍스트와 팀 색상으로 변환하는 매핑
const PIECE_MAP = {
    'cha_red': { t: '車', team: 'han' }, 'ma_red': { t: '馬', team: 'han' },
    'sang_red': { t: '象', team: 'han' }, 'sa_red': { t: '士', team: 'han' },
    'wang_red': { t: '漢', team: 'han' }, 'po_red': { t: '包', team: 'han' },
    'jol_red': { t: '兵', team: 'han' },
    'cha_green': { t: '車', team: 'cho' }, 'ma_green': { t: '馬', team: 'cho' },
    'sang_green': { t: '象', team: 'cho' }, 'sa_green': { t: '士', team: 'cho' },
    'wang_green': { t: '楚', team: 'cho' }, 'po_green': { t: '包', team: 'cho' },
    'jol_green': { t: '卒', team: 'cho' }
};

// === [2] 보드 렌더링 (새 디자인 SVG 로직) ===
const COLS = 9, ROWS = 10, GAP = 50, OX = 40, OY = 40;
const px = c => OX + c*GAP;
const py = r => OY + r*GAP;

let pieces = [];
let selected = null;
let hoverTarget = null; 

const svgEl = document.getElementById('board');
svgEl.addEventListener('mousemove', (e) => {
  const rect = svgEl.getBoundingClientRect();
  const scaleX = 480 / rect.width;
  const scaleY = 530 / rect.height;
  const x = (e.clientX - rect.left) * scaleX;
  const y = (e.clientY - rect.top) * scaleY;
  
  let c = Math.round((x - OX) / GAP);
  let r = Math.round((y - OY) / GAP);
  
  if (c >= 0 && c < COLS && r >= 0 && r < ROWS) {
    if (!hoverTarget || hoverTarget.c !== c || hoverTarget.r !== r) {
      hoverTarget = {c, r};
      drawBoard();
    }
  } else {
    if (hoverTarget) {
      hoverTarget = null;
      drawBoard();
    }
  }
});

svgEl.addEventListener('mouseleave', () => {
  hoverTarget = null;
  drawBoard();
});

function octagonPath(cx, cy, r){
  const pts = [];
  for(let i=0;i<8;i++){
    const a = Math.PI/8 + i*(Math.PI/4);
    pts.push(`${cx + r*Math.sin(a)},${cy - r*Math.cos(a)}`);
  }
  return `M${pts.join(' L')} Z`;
}

function drawBoard(){
  const ink = '#0a0a0a';
  let parts = [];

  // 격자선 그리기
  for(let r=0;r<ROWS;r++){
    parts.push(`<line x1="${px(0)}" y1="${py(r)}" x2="${px(COLS-1)}" y2="${py(r)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);
  }
  for(let c=0;c<COLS;c++){
    parts.push(`<line x1="${px(c)}" y1="${py(0)}" x2="${px(c)}" y2="${py(ROWS-1)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);
  }
  parts.push(`<line x1="${px(3)}" y1="${py(0)}" x2="${px(5)}" y2="${py(2)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);
  parts.push(`<line x1="${px(5)}" y1="${py(0)}" x2="${px(3)}" y2="${py(2)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);
  parts.push(`<line x1="${px(3)}" y1="${py(7)}" x2="${px(5)}" y2="${py(9)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);
  parts.push(`<line x1="${px(5)}" y1="${py(7)}" x2="${px(3)}" y2="${py(9)}" stroke="${ink}" stroke-width="1" opacity="0.85"/>`);

  // 타겟 표시 렌더링
  if (selected !== null && pieces[selected]) {
    const activeC = pieces[selected].c;
    const activeR = pieces[selected].r;
    const isHan = pieces[selected].team === 'han';
    const color = isHan ? '#b23a2e' : '#2f5d55';
    parts.push(`<circle cx="${px(activeC)}" cy="${py(activeR)}" r="23" fill="${color}" fill-opacity="0.12" stroke="${color}" stroke-width="3.5" stroke-linecap="round"/>`);
  } 
  else if (hoverTarget !== null) {
    const activeC = hoverTarget.c;
    const activeR = hoverTarget.r;
    parts.push(`<circle cx="${px(activeC)}" cy="${py(activeR)}" r="20" fill="none" stroke="#0a0a0a" stroke-width="1.4" stroke-dasharray="1 5" stroke-linecap="round">
      <animateTransform attributeName="transform" type="rotate" from="0 ${px(activeC)} ${py(activeR)}" to="360 ${px(activeC)} ${py(activeR)}" dur="7s" repeatCount="indefinite"/>
    </circle>`);
  }

  // 장기알 그리기
  pieces.forEach((p,i)=>{
    const cx=px(p.c), cy=py(p.r);
    const isHan = p.team==='han';
    const isSel = selected===i;
    const textColor = isHan ? '#b23a2e' : '#2f5d55';
    parts.push(`<path d="${octagonPath(cx,cy,15)}" fill="#ffffff" stroke="${isSel ? textColor : ink}" stroke-width="${isSel?2.6:1.4}"/>`);
    parts.push(`<text class="piece-label" x="${cx}" y="${cy+5.5}" text-anchor="middle" font-size="15" fill="${textColor}">${p.t}</text>`);
  });

  svgEl.innerHTML = parts.join('');

  // 투명 히트박스 생성
  pieces.forEach((p,i)=>{
    const hit = document.createElementNS('http://www.w3.org/2000/svg','circle');
    hit.setAttribute('cx', px(p.c)); hit.setAttribute('cy', py(p.r));
    hit.setAttribute('r', 20); 
    hit.setAttribute('fill','transparent');
    hit.style.cursor='pointer';
    hit.addEventListener('click', (e)=>{ 
      e.stopPropagation(); 
      selected = (selected===i)? null : i; 
      drawBoard(); 
    });
    svgEl.appendChild(hit);
  });
}

// === [3] UI 업데이트 헬퍼 함수 ===
function setTurn(turnColor) {
  const seal = document.getElementById('turnSeal');
  const stamp = document.getElementById('turnStamp');
  const label = document.getElementById('turnLabel');
  if (turnColor === 'red') {
    seal.classList.remove('cho');
    stamp.textContent = '漢'; 
    label.textContent = '로봇 (한)';
  } else {
    seal.classList.add('cho');
    stamp.textContent = '楚'; 
    label.textContent = '사용자 (초)';
  }
}

function pushLog(text) {
  const strip = document.getElementById('logStrip');
  const el = document.createElement('span');
  el.className = 'entry';
  el.innerHTML = text;
  strip.appendChild(el);
  strip.scrollLeft = strip.scrollWidth;
}

// 2D 배열을 SVG 렌더링용 객체 배열로 변환
function parseBoardData(board2D) {
    let newPieces = [];
    for (let r = 0; r < 10; r++) {
        for (let c = 0; c < 9; c++) {
            const key = board2D[r][c];
            if (key && PIECE_MAP[key]) {
                newPieces.push({ c: c, r: r, t: PIECE_MAP[key].t, team: PIECE_MAP[key].team });
            }
        }
    }
    return newPieces;
}

// === [4] 서버/웹소켓 실시간 연동 로직 ===

function joinWebSocketRoom(sessionId) {
    socket.emit('join', { sessionId: sessionId });
    document.getElementById('displaySessionId').textContent = sessionId;
}

// 서버에서 DB 업데이트 발생 시 수신
socket.on('board_updated', function(data) {
    console.log("실시간 보드 갱신!", data);
    
    // 데이터 파싱 후 SVG 리렌더링
    pieces = parseBoardData(data.board);
    drawBoard();
    
    // 턴 및 로그 업데이트
    const turn = data.currentTurn || 'red';
    setTurn(turn);
    
    const time = new Date().toLocaleTimeString();
    pushLog(`<b>${time}</b> 실시간 서버 갱신 완료`);
});

// 초기 데이터 로드 함수
async function loadBoard() {
    if (!currentSessionId) return;
    try {
        const res = await fetch(`${API_BASE}/api/board/${currentSessionId}`);
        const data = await res.json();
        if (data && data.board) {
            pieces = parseBoardData(data.board);
            drawBoard();
            setTurn(data.currentTurn || 'red');
            pushLog('이전 게임 데이터를 불러왔습니다.');
        }
    } catch (e) {
        console.error('보드 로드 실패:', e);
    }
}

// 시작 실행
(async () => {
    if (currentSessionId) {
        joinWebSocketRoom(currentSessionId);
        await loadBoard();
    } else {
        pushLog('활성화된 세션이 없습니다. API를 통해 세션을 생성해주세요.');
        drawBoard(); // 빈 보드 생성
    }
})();