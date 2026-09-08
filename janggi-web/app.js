const API_BASE = '';
const socket = io();

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

const COLS = 9, ROWS = 10, GAP = 50, OX = 40, OY = 40;
const START_MARKERS = [
  { c: 1, r: 2 }, { c: 7, r: 2 },
  { c: 0, r: 3 }, { c: 2, r: 3 }, { c: 4, r: 3 }, { c: 6, r: 3 }, { c: 8, r: 3 },
  { c: 1, r: 7 }, { c: 7, r: 7 },
  { c: 0, r: 6 }, { c: 2, r: 6 }, { c: 4, r: 6 }, { c: 6, r: 6 }, { c: 8, r: 6 }
];
const px = c => OX + c*GAP;
const py = r => OY + r*GAP;

let pieces = [];
let selected = null;
let hoverTarget = null; 
const seenTaskEvents = new Set();
let boardEventVersion = 0;

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

function starPath(cx, cy, outerRadius, innerRadius){
  const pts = [];
  for(let i=0;i<10;i++){
    const radius = i % 2 === 0 ? outerRadius : innerRadius;
    const angle = -Math.PI / 2 + i * Math.PI / 5;
    pts.push(`${cx + radius * Math.cos(angle)},${cy + radius * Math.sin(angle)}`);
  }
  return `M${pts.join(' L')} Z`;
}

function drawBoard(){
  const ink = '#0a0a0a';
  let parts = [];

  parts.push(`<defs><clipPath id="boardBounds"><rect x="${OX}" y="${OY}" width="${(COLS - 1) * GAP}" height="${(ROWS - 1) * GAP}"/></clipPath></defs>`);

  for(let c=0;c<COLS;c++){
    parts.push(`<text x="${px(c)}" y="22" text-anchor="middle" font-size="10" font-family="Noto Sans KR, sans-serif" font-weight="700" fill="#6e6e6e">${c + 1}</text>`);
  }
  for(let r=0;r<ROWS;r++){
    parts.push(`<text x="20" y="${py(r) + 3.5}" text-anchor="middle" font-size="10" font-family="Noto Sans KR, sans-serif" font-weight="700" fill="#6e6e6e">${r + 1}</text>`);
  }

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

  START_MARKERS.forEach(({c, r}) => {
    parts.push(`<path d="${starPath(px(c), py(r), 5, 2.5)}" fill="${ink}" stroke="${ink}" stroke-width="0.5" clip-path="url(#boardBounds)"/>`);
  });

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

  pieces.forEach((p,i)=>{
    const cx=px(p.c), cy=py(p.r);
    const isHan = p.team==='han';
    const isSel = selected===i;
    const textColor = isHan ? '#b23a2e' : '#2f5d55';
    parts.push(`<path d="${octagonPath(cx,cy,15)}" fill="#ffffff" stroke="${isSel ? textColor : ink}" stroke-width="${isSel?2.6:1.4}"/>`);
    parts.push(`<text class="piece-label" x="${cx}" y="${cy+5.5}" text-anchor="middle" font-size="15" fill="${textColor}">${p.t}</text>`);
    if (isSel) {
      const labelY = Math.max(15, cy - 32);
      parts.push(`<rect x="${cx - 19}" y="${labelY - 14}" width="38" height="20" fill="#ece8c9" stroke="#6e6e6e" stroke-width="1"/>`);
      parts.push(`<text x="${cx}" y="${labelY}" text-anchor="middle" font-size="13" font-family="Noto Sans KR, sans-serif" font-weight="800" fill="${ink}">${p.r + 1}-${p.c + 1}</text>`);
    }
  });

  svgEl.innerHTML = parts.join('');

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
  el.textContent = text;
  strip.appendChild(el);
  strip.scrollLeft = strip.scrollWidth;
}

// ARM UI 업데이트 헬퍼 함수
function updateArmStatus(moveText, status = 'completed') {
    const badge = document.getElementById('armBadge');
    const target = document.getElementById('armTarget');
    const statusLine = document.getElementById('armStatusLine');

    if (moveText) {
        // 1. Task 상태 표시
        badge.textContent = status === 'started' ? 'ACTIVE' : status === 'failed' ? 'FAILED' : 'IDLE';
        statusLine.className = status === 'started' ? 'status-line moving' : 'status-line';
        
        // 2. 처음 표시된 기본 텍스트("현재 동작 없음") 지우기 및 스크롤 설정
        if (target.textContent === '현재 동작 없음') {
            target.innerHTML = '';
            target.style.maxHeight = '120px'; // 패널 최대 높이 설정
            target.style.overflowY = 'auto';  // 내용이 넘치면 스크롤 생성
        }
        
        // 3. 새로운 동작 기록 컨테이너 생성
        const moveEntry = document.createElement('div');
        moveEntry.style.paddingBottom = '6px';
        moveEntry.style.marginBottom = '6px';
        moveEntry.style.borderBottom = '1px dashed rgba(110, 110, 110, 0.3)'; // 구분선 추가
        moveEntry.textContent = moveText;
        
        // 4. 새로운 기록을 가장 위쪽(첫 번째 자식)으로 추가
        target.insertBefore(moveEntry, target.firstChild);
    }
}

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

// 장기판은 서버 상태를 렌더링하고, 동작 기록은 Task API 내용으로 표시한다.
function applyBoardUpdate(data) {
    const board = data && data.board;
    if (!Array.isArray(board) || board.length !== ROWS ||
        !board.every(row => Array.isArray(row) && row.length === COLS)) return;
    selected = null;
    pieces = parseBoardData(board);
    drawBoard();
    setTurn(data.currentTurn || 'red');
}

function positionText(value) {
    if (value && value.type === 'bucket') return '버리는 곳';
    if (!value || typeof value !== 'object') return '위치 미확인';
    if (Number.isInteger(value.row) && Number.isInteger(value.col)) {
        return `${value.col}열 ${value.row}행`;
    }
    if (Number.isInteger(value.r) && Number.isInteger(value.c)) {
        return `${value.c + 1}열 ${value.r + 1}행`;
    }
    return '위치 미확인';
}

socket.on('task_event', function(data) {
    if (!data || !['task', 'mode'].includes(data.kind)) return;
    const identity = data.task_id || data.event_id;
    const eventKey = JSON.stringify([identity, data.status]);
    if (seenTaskEvents.has(eventKey)) return;
    seenTaskEvents.add(eventKey);
    if (seenTaskEvents.size > 1000) seenTaskEvents.delete(seenTaskEvents.values().next().value);
    const mode = {voice: '음성', vision: '비전', gesture: '제스처'}[data.mode] || data.mode || '미지정';
    const time = new Date().toLocaleTimeString();
    if (data.kind === 'mode') {
        pushLog(`${time} 모드 변경: ${mode}`);
        return;
    }
    // started는 상태만 표시하고 완료/실패 기록을 한 줄씩 남긴다.
    if (data.status === 'started') {
        document.getElementById('armBadge').textContent = 'ACTIVE';
        document.getElementById('armStatusLine').className = 'status-line moving';
        return;
    }
    const names = {cha: '차', ma: '마', sang: '상', sa: '사', wang: '왕', po: '포', jol: '졸'};
    const rawPiece = typeof data.piece === 'string' ? data.piece : '';
    const [pieceKey, encodedPosition] = rawPiece.split('@');
    const piece = PIECE_MAP[pieceKey];
    const pieceName = piece ? `${piece.team === 'han' ? '한' : '초'} ${names[pieceKey.split('_')[0]]}` : '말(종류 미확인)';
    let before = data.before;
    if (!before && /^\d+,\d+$/.test(encodedPosition || '')) {
        const [row, col] = encodedPosition.split(',').map(Number);
        before = {row, col};
    }
    const isBucket = data.after === null || (data.after && data.after.type === 'bucket');
    const action = isBucket ? '버림' : '이동';
    const destination = isBucket ? '버리는 곳' : positionText(data.after);
    const result = data.status === 'failed' ? `${action} 실패` : action;
    const text = `${pieceName}: ${positionText(before)} → ${destination} · ${result}`;
    updateArmStatus(text, data.status);
    pushLog(`${time} ${text}`);
});

socket.on('board_updated', function(data) {
    boardEventVersion += 1;
    applyBoardUpdate(data);
});

// 재연결 시에도 DB의 최신 장기판 복원. 조회 도중 도착한 실시간 갱신은 유지한다.
async function loadBoard() {
    const version = boardEventVersion;
    try {
        const res = await fetch(`${API_BASE}/api/board`, {cache: 'no-store'});
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();
        if (version === boardEventVersion && data && data.board) {
            applyBoardUpdate(data);
            pushLog('최신 장기판을 불러왔습니다.');
        }
    } catch (e) {
        console.error('보드 로드 실패:', e);
    }
}

socket.on('connect', loadBoard);
if (socket.connected) loadBoard();