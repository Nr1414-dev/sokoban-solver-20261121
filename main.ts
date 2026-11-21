// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Strategy: a level is a hallway with one to four rooms off it (generator's
// construction), each room holding 3 or 4 boxes and as many goals, with a
// push-optimal count P* equal to the sum of the rooms' optima. So the level
// is decomposed into rooms (peeling articulation points off the floor graph),
// each room is solved push-optimally on its own with A* over pushes, and the
// room solutions are stitched together with player walks along the hallway.
// If decomposition fails, a whole-level push A* is the fallback.
//
// The room solver is A* over pushes with:
//   - a Hungarian-assignment heuristic on shortest-path (wall-only) distances
//     from boxes to goals (admissible, and consistent: one push moves one box
//     one cell, so the assignment cost changes by at most 1),
//   - a deadlock table from reverse pull search (a box on a cell from which no
//     goal is reachable by pulling can never be solved),
//   - 2x2 freeze deadlock pruning (a full 2x2 block of walls/boxes cannot move),
//   - a transposition table keyed on (sorted boxes, canonical player cell),
//     where the canonical player cell is the smallest cell of the player's
//     reachable region (player position within a region is irrelevant),
//   - deterministic everything: fixed direction order, fixed heap tie-breaks,
//     expansion-count budgets only (the wall clock is never read).
// Determinism: two runs on the same input write byte-identical outputs.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

const ROOM_EXPANSIONS = 200_000;
const WHOLE_EXPANSIONS = 800_000;
const BIG = 1_000_000;

interface Parsed {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall (includes out-of-row padding)
  goals: number[]; // sorted cell indices
  goalSet: Set<number>;
  boxes: number[]; // sorted cell indices
  player: number;
}

function parseLevel(text: string): Parsed {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, goalSet: new Set(goals), boxes, player };
}

// Breadth-first search over cells where `pass` is 1. Returns a mask of
// reached cells, and the move string from `from` to `to` ('' if unreachable).
function bfsWalk(
  from: number,
  to: number,
  pass: Uint8Array,
  w: number,
  h: number,
): { mask: Uint8Array; moves: string } {
  const n = w * h;
  const mask = new Uint8Array(n);
  const prev = new Int32Array(n).fill(-1);
  const prevCh = new Array<string>(n).fill('');
  const q: number[] = [from];
  mask[from] = 1;
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    if (c === to) break;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const nc = ny * w + nx;
      if (!pass[nc] || mask[nc]) continue;
      mask[nc] = 1;
      prev[nc] = c;
      prevCh[nc] = CH[d]!;
      q.push(nc);
    }
  }
  let moves = '';
  if (mask[to]) {
    for (let c = to; c !== from; c = prev[c]!) moves = prevCh[c]! + moves;
  }
  return { mask, moves };
}

// Shortest-path distances from every cell to the nearest goal, walking only
// on `floor` (walls excluded, boxes ignored). BFS from all goals at once.
function goalDistances(goals: number[], floor: Uint8Array, w: number, h: number): Int32Array {
  const n = w * h;
  const dist = new Int32Array(n).fill(-1);
  const q: number[] = [];
  for (const g of goals) {
    dist[g] = 0;
    q.push(g);
  }
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const nc = ny * w + nx;
      if (!floor[nc] || dist[nc] !== -1) continue;
      dist[nc] = dist[c]! + 1;
      q.push(nc);
    }
  }
  return dist;
}

// Deadlock table: cells from which a box can reach a goal by pulling.
// Reverse BFS from the goals: a box on cell x could have come from x-d if the
// player could stand at x-2d (both floor). Cells never reached are dead: a box
// there can never reach any goal, whatever the player does.
function liveCells(goals: number[], boxFloor: Uint8Array, w: number, h: number): Uint8Array {
  const n = w * h;
  const live = new Uint8Array(n);
  const q: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    q.push(g);
  }
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const px = cx - DX[d]!;
      const py = cy - DY[d]!;
      const bx = cx - 2 * DX[d]!;
      const by = cy - 2 * DY[d]!;
      if (px < 0 || py < 0 || px >= w || py >= h) continue;
      if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
      const prev = py * w + px;
      const beyond = by * w + bx;
      if (!boxFloor[prev] || !boxFloor[beyond] || live[prev]) continue;
      live[prev] = 1;
      q.push(prev);
    }
  }
  return live;
}

// Min-cost assignment (Hungarian, O(n^3), deterministic) of boxes to goals.
// cost[i][j] is the distance from box i to goal j; BIG means unreachable.
// Returns the minimal total, or >= BIG when no perfect matching exists.
function hungarian(cost: number[][], n: number): number {
  // 1-indexed potentials; a[i][j] = cost[i-1][j-1].
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  const p = new Int32Array(n + 1);
  const way = new Int32Array(n + 1);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0]!;
      let delta = Infinity;
      let j1 = 0;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1]![j - 1]! - u[i0]! - v[j]!;
        if (cur < minv[j]!) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j]! < delta) {
          delta = minv[j]!;
          j1 = j;
        }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          u[p[j]!] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0]!;
      p[j0] = p[j1]!;
      j0 = j1;
    } while (j0 !== 0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) {
    total += cost[p[j]! - 1]![j - 1]!;
    if (total >= BIG) return total;
  }
  return total;
}

interface PushProblem {
  w: number;
  h: number;
  boxFloor: Uint8Array; // cells a box may occupy
  walkFloor: Uint8Array; // cells the player may walk (boxFloor + entry)
  goals: number[]; // sorted
  boxes: number[]; // sorted start boxes
  player: number; // start player cell
  exit: number; // cell the player must be able to reach at the end, or -1
  maxExpansions: number;
}

interface PushSolution {
  moves: string;
  pushes: number;
  expansions: number;
}

// A* over pushes. Returns the push-optimal move string or null.
function solvePushes(prob: PushProblem): PushSolution | null {
  const { w, h, boxFloor, walkFloor, goals, maxExpansions } = prob;
  const n = w * h;
  const goalSet = new Set(goals);
  const nGoals = goals.length;
  if (prob.boxes.length !== nGoals) return null;

  // Per-goal shortest-path distances (boxes ignored).
  const distToGoal: Int32Array[] = goals.map((g) => {
    const d = new Int32Array(n).fill(-1);
    const q: number[] = [g];
    d[g] = 0;
    let head = 0;
    while (head < q.length) {
      const c = q[head++]!;
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let dd = 0; dd < 4; dd++) {
        const nx = cx + DX[dd]!;
        const ny = cy + DY[dd]!;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nc = ny * w + nx;
        if (!boxFloor[nc] || d[nc] !== -1) continue;
        d[nc] = d[c]! + 1;
        q.push(nc);
      }
    }
    return d;
  });
  const boxDist = (box: number, goal: number): number => {
    const d = distToGoal[goal]![box]!;
    return d === -1 ? BIG : d;
  };
  const heuristic = (boxes: number[]): number => {
    const cost: number[][] = [];
    for (let i = 0; i < boxes.length; i++) {
      const row: number[] = [];
      for (let j = 0; j < nGoals; j++) row.push(boxDist(boxes[i]!, j));
      cost.push(row);
    }
    return hungarian(cost, boxes.length);
  };

  const live = liveCells(goals, boxFloor, w, h);

  // 2x2 windows (top-left cell index) for freeze deadlock checks.
  const windows: number[][] = [];
  for (let y = 0; y < h - 1; y++) {
    for (let x = 0; x < w - 1; x++) {
      const a = y * w + x;
      windows.push([a, a + 1, a + w, a + w + 1]);
    }
  }
  const frozen = (boxes: number[]): boolean => {
    const set = new Set(boxes);
    for (const win of windows) {
      let allBlocked = true;
      let badBox = false;
      for (const c of win) {
        if (set.has(c)) {
          if (!goalSet.has(c)) badBox = true;
        } else if (boxFloor[c]) {
          allBlocked = false;
          break;
        }
      }
      if (allBlocked && badBox) return true;
    }
    return false;
  };

  // State storage.
  const sBoxes: number[][] = [];
  const sCanon: number[] = [];
  const sActual: number[] = [];
  const sParent: number[] = [];
  const sPushBox: number[] = [];
  const sPushDir: number[] = [];
  const sG: number[] = [];

  const keyOf = (boxes: number[], canon: number): string => boxes.join(',') + '|' + canon;
  const best = new Map<string, number>();

  // Reachable region of the player given the boxes; returns mask + canonical.
  const region = (player: number, boxes: number[]): { mask: Uint8Array; canon: number } => {
    const pass = new Uint8Array(n);
    for (let i = 0; i < n; i++) pass[i] = walkFloor[i];
    for (const b of boxes) pass[b] = 0;
    const { mask } = bfsWalk(player, -1, pass, w, h);
    let canon = -1;
    for (let i = 0; i < n; i++) {
      if (mask[i]) {
        canon = i;
        break;
      }
    }
    return { mask, canon };
  };

  const startBoxes = prob.boxes.slice().sort((a, b) => a - b);
  const startRegion = region(prob.player, startBoxes);
  const h0 = heuristic(startBoxes);
  if (h0 >= BIG || frozen(startBoxes)) return null;
  sBoxes.push(startBoxes);
  sCanon.push(startRegion.canon);
  sActual.push(prob.player);
  sParent.push(-1);
  sPushBox.push(-1);
  sPushDir.push(-1);
  sG.push(0);
  best.set(keyOf(startBoxes, startRegion.canon), 0);

  // Heap entries: [f, g, seq, state]. Order: f asc, g desc, seq asc.
  const heap: [number, number, number, number][] = [];
  let seq = 0;
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  heapPush([h0, 0, seq++, 0]);

  const isGoal = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));

  let expansions = 0;
  while (heap.length > 0 && expansions < maxExpansions) {
    const [, g, , s] = heapPop();
    const boxes = sBoxes[s]!;
    const k = keyOf(boxes, sCanon[s]!);
    if (best.get(k)! < g) continue; // stale entry
    if (isGoal(boxes)) {
      // The player must be able to walk out to the exit cell.
      if (prob.exit >= 0) {
        const reg = region(sActual[s]!, boxes);
        if (!reg.mask[prob.exit]) continue;
      }
      // Reconstruct the move string.
      const steps: { box: number; dir: number }[] = [];
      for (let t = s; t > 0; t = sParent[t]!) steps.push({ box: sPushBox[t]!, dir: sPushDir[t]! });
      steps.reverse();
      let moves = '';
      let player = prob.player;
      let curBoxes = prob.boxes.slice().sort((a, b) => a - b);
      for (const st of steps) {
        const bx = st.box % w;
        const by = (st.box / w) | 0;
        const from = st.box - (DY[st.dir]! * w + DX[st.dir]!);
        const pass = new Uint8Array(n);
        for (let i = 0; i < n; i++) pass[i] = walkFloor[i];
        for (const b of curBoxes) pass[b] = 0;
        const walk = bfsWalk(player, from, pass, w, h);
        moves += walk.moves + CH[st.dir]!;
        const at = curBoxes.indexOf(st.box);
        curBoxes[at] = st.box + (DY[st.dir]! * w + DX[st.dir]!);
        curBoxes.sort((a, b) => a - b);
        player = st.box;
      }
      // Walk out to the exit cell.
      if (prob.exit >= 0) {
        const pass = new Uint8Array(n);
        for (let i = 0; i < n; i++) pass[i] = walkFloor[i];
        for (const b of curBoxes) pass[b] = 0;
        moves += bfsWalk(player, prob.exit, pass, w, h).moves;
      }
      return { moves, pushes: g, expansions };
    }
    expansions++;
    const reg = region(sActual[s]!, boxes);
    const boxSet = new Set(boxes);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const tx = bx + DX[d]!;
        const ty = by + DY[d]!;
        if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;
        const to = ty * w + tx;
        if (!boxFloor[to] || boxSet.has(to)) continue;
        const from = b - (DY[d]! * w + DX[d]!);
        if (from < 0 || from >= n || !reg.mask[from]) continue;
        const nb = boxes.slice();
        nb[bi] = to;
        nb.sort((a, b2) => a - b2);
        if (!live[to] && !goalSet.has(to)) continue;
        if (frozen(nb)) continue;
        const hh = heuristic(nb);
        if (hh >= BIG) continue;
        const childRegion = region(b, nb);
        const ck = keyOf(nb, childRegion.canon);
        const ng = g + 1;
        const seen = best.get(ck);
        if (seen !== undefined && seen <= ng) continue;
        best.set(ck, ng);
        const t = sBoxes.length;
        sBoxes.push(nb);
        sCanon.push(childRegion.canon);
        sActual.push(b);
        sParent.push(s);
        sPushBox.push(b);
        sPushDir.push(d);
        sG.push(ng);
        heapPush([ng + hh, ng, seq++, t]);
      }
    }
  }
  return null;
}

interface Room {
  door: number;
  cells: number[]; // room cells (without the door)
  entry: number; // hallway cell adjacent to the door
}

// Decompose the level into rooms by peeling articulation points off the
// floor graph. Returns the rooms (sorted by door cell) or null.
function decompose(level: Parsed): Room[] | null {
  const { w, h, wall, boxes, goals } = level;
  const n = w * h;
  const isFloor = (c: number): boolean => !wall[c];
  const neighbors = (c: number): number[] => {
    const out: number[] = [];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const nc = ny * w + nx;
      if (isFloor(nc)) out.push(nc);
    }
    return out;
  };
  // Connected components of `cells` (a Set), flood fill from each unvisited.
  const components = (cells: Set<number>): number[][] => {
    const seen = new Set<number>();
    const comps: number[][] = [];
    for (const start of cells) {
      if (seen.has(start)) continue;
      const comp: number[] = [];
      const q: number[] = [start];
      seen.add(start);
      let head = 0;
      while (head < q.length) {
        const c = q[head++]!;
        comp.push(c);
        for (const nb of neighbors(c)) {
          if (cells.has(nb) && !seen.has(nb)) {
            seen.add(nb);
            q.push(nb);
          }
        }
      }
      comps.push(comp);
    }
    return comps;
  };

  const remaining = new Set<number>();
  for (let i = 0; i < n; i++) if (isFloor(i)) remaining.add(i);
  const rooms: Room[] = [];
  const boxSet = new Set(boxes);
  const goalSet = new Set(goals);

  for (;;) {
    // Find the smallest component (not holding the player) with an equal,
    // non-zero number of boxes and goals, split off by removing one cell.
    let bestCand: { door: number; cells: number[] } | null = null;
    for (const c of remaining) {
      if (c === level.player) continue;
      const without = new Set(remaining);
      without.delete(c);
      for (const comp of components(without)) {
        if (comp.includes(level.player)) continue;
        let nb = 0;
        let ng = 0;
        for (const cell of comp) {
          if (boxSet.has(cell)) nb++;
          if (goalSet.has(cell)) ng++;
        }
        if (nb === 0 || nb !== ng) continue;
        if (bestCand === null || comp.length < bestCand.cells.length || (comp.length === bestCand.cells.length && c < bestCand.door)) {
          bestCand = { door: c, cells: comp };
        }
      }
    }
    if (bestCand === null) break;
    // Entry: a remaining neighbor of the door (hallway side), smallest index.
    let entry = -1;
    for (const nb of neighbors(bestCand.door)) {
      if (remaining.has(nb) && !bestCand.cells.includes(nb)) {
        if (entry === -1 || nb < entry) entry = nb;
      }
    }
    if (entry === -1) return null;
    rooms.push({ door: bestCand.door, cells: bestCand.cells, entry });
    for (const cell of bestCand.cells) remaining.delete(cell);
  }

  // Every box must belong to a room.
  const inRoom = new Set<number>();
  for (const r of rooms) {
    for (const cell of r.cells) inRoom.add(cell);
    inRoom.add(r.door);
  }
  for (const b of boxes) if (!inRoom.has(b)) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Solve one level. Returns the move string ('' when unsolved).
function solveLevel(text: string): { moves: string; pushes: number } {
  const level = parseLevel(text);
  const { w, h, wall } = level;
  const n = w * h;

  const rooms = decompose(level);
  if (rooms !== null) {
    // Per-room solving, stitched with hallway walks.
    let moves = '';
    let player = level.player;
    let curBoxes = level.boxes.slice();
    let ok = true;
    for (const room of rooms) {
      // Walk from the current player position to the room's entry cell.
      const pass = new Uint8Array(n);
      for (let i = 0; i < n; i++) pass[i] = wall[i] ? 0 : 1;
      for (const b of curBoxes) pass[b] = 0;
      const walkIn = bfsWalk(player, room.entry, pass, w, h);
      if (!walkIn.mask[room.entry]) {
        ok = false;
        break;
      }
      moves += walkIn.moves;
      // Room subproblem: floor = room cells + door; walk also on the entry.
      const boxFloor = new Uint8Array(n);
      for (const c of room.cells) boxFloor[c] = 1;
      boxFloor[room.door] = 1;
      const walkFloor = boxFloor.slice();
      walkFloor[room.entry] = 1;
      const roomBoxes = curBoxes.filter((b) => boxFloor[b]).sort((a, b) => a - b);
      const roomGoals = level.goals.filter((g) => boxFloor[g]).sort((a, b) => a - b);
      const sol = solvePushes({
        w,
        h,
        boxFloor,
        walkFloor,
        goals: roomGoals,
        boxes: roomBoxes,
        player: room.entry,
        exit: room.entry,
        maxExpansions: ROOM_EXPANSIONS,
      });
      if (sol === null) {
        ok = false;
        break;
      }
      moves += sol.moves;
      // Update the box positions: replay the room solution on the room's boxes.
      const replayed = replay(roomBoxes, room.entry, sol.moves, boxFloor, walkFloor, w, h);
      if (replayed === null) {
        ok = false;
        break;
      }
      const boxMap = new Map<number, number>();
      roomBoxes.forEach((b, i) => boxMap.set(b, replayed.boxes[i]!));
      curBoxes = curBoxes.map((b) => (boxMap.has(b) ? boxMap.get(b)! : b));
      player = room.entry;
    }
    if (ok) {
      const verified = verify(text, moves);
      if (verified !== null) return verified;
    }
  }

  // Fallback: whole-level push A*.
  const floor = new Uint8Array(n);
  for (let i = 0; i < n; i++) floor[i] = wall[i] ? 0 : 1;
  const sol = solvePushes({
    w,
    h,
    boxFloor: floor,
    walkFloor: floor,
    goals: level.goals,
    boxes: level.boxes,
    player: level.player,
    exit: -1,
    maxExpansions: WHOLE_EXPANSIONS,
  });
  if (sol !== null) {
    const verified = verify(text, sol.moves);
    if (verified !== null) return verified;
  }
  return { moves: '', pushes: 0 };
}

// Replay moves on a box set from a player start; returns the final boxes and
// player, or null on an illegal step. `floor` marks walkable cells.
function replay(
  boxes0: number[],
  player0: number,
  moves: string,
  boxFloor: Uint8Array,
  walkFloor: Uint8Array,
  w: number,
  h: number,
): { boxes: number[]; player: number } | null {
  void boxFloor;
  void walkFloor;
  void h;
  const boxes = boxes0.slice();
  let player = player0;
  const set = new Set(boxes);
  for (const ch of moves) {
    const d = CH.indexOf(ch.toLowerCase());
    if (d === -1) return null;
    const next = player + DY[d]! * w + DX[d]!;
    if (set.has(next)) {
      const beyond = next + DY[d]! * w + DX[d]!;
      if (set.has(beyond)) return null;
      set.delete(next);
      set.add(beyond);
      const at = boxes.indexOf(next);
      boxes[at] = beyond;
    }
    player = next;
  }
  boxes.sort((a, b) => a - b);
  return { boxes, player };
}

// Verify a full move string on the level text; returns pushes or null.
function verify(text: string, moves: string): { moves: string; pushes: number } | null {
  const level = parseLevel(text);
  const { w } = level;
  const set = new Set(level.boxes);
  let player = level.player;
  let pushes = 0;
  for (const ch of moves) {
    if (ch === '\n' || ch === ' ' || ch === '\r' || ch === '\t') continue;
    const d = CH.indexOf(ch.toLowerCase());
    if (d === -1) return null;
    const next = player + DY[d]! * w + DX[d]!;
    if (level.wall[next]) return null;
    if (set.has(next)) {
      const beyond = next + DY[d]! * w + DX[d]!;
      if (level.wall[beyond] || set.has(beyond)) return null;
      set.delete(next);
      set.add(beyond);
      pushes++;
    }
    player = next;
  }
  for (const b of set) if (!level.goalSet.has(b)) return null;
  return { moves, pushes };
}

for (const id of readdirSync(I).sort()) {
  const text = readFileSync(join(I, id, 'level.txt'), 'utf8');
  const { moves } = solveLevel(text);
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
