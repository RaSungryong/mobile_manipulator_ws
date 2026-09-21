/* Operator UI — browser side.
 *
 * Thin on purpose: every button is one `call(method, args)` to the server
 * (UiController.api_<method>), every readout renders a state / event / ui
 * message the server pushed. No robot logic lives here, so two tabs open
 * on two computers show the same thing and either may act.
 *
 * Camera frames arrive as binary WebSocket messages (see web_server.py);
 * one frame per camera is outstanding at a time and the client acks after
 * decoding, so a slow link drops frames instead of lagging.
 */
'use strict';

const $ = (id) => document.getElementById(id);
const AXES = ['x', 'y', 'z', 'rx', 'ry', 'rz'];
const STREAM_CAMS = ['front_cam', 'side_cam', 'hand_cam'];

// ------------------------------------------------------------------
// Connection
// ------------------------------------------------------------------
let ws = null;
let callId = 0;
const pendingCalls = new Map();
let reconnectDelay = 500;

function connect() {
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
  setConn('connecting…', '');
  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { reconnectDelay = 500; setConn('connected ' + location.host, 'ok'); };
  ws.onclose = () => {
    setConn('DISCONNECTED — retrying', 'bad');
    for (const [, p] of pendingCalls) p.reject(new Error('connection lost'));
    pendingCalls.clear();
    setTimeout(connect, reconnectDelay);
    reconnectDelay = Math.min(5000, reconnectDelay * 2);
  };
  ws.onerror = () => {};
  ws.onmessage = (ev) => {
    if (ev.data instanceof ArrayBuffer) { onFrame(ev.data); return; }
    let msg;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    onMessage(msg);
  };
}

function setConn(text, cls) {
  const chip = $('chip-conn');
  chip.textContent = text;
  chip.className = 'chip conn ' + cls;
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

/** RPC to UiController.api_<method>. Resolves with the result; a failed
 *  call is logged and rejects. */
function call(method, args, opts) {
  opts = opts || {};
  return new Promise((resolve, reject) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      appendLog('[web] not connected — ' + method + ' dropped');
      reject(new Error('not connected'));
      return;
    }
    const id = ++callId;
    pendingCalls.set(id, { resolve, reject, method });
    send({ t: 'call', id, m: method, a: args || [] });
  });
}

// ------------------------------------------------------------------
// Inbound
// ------------------------------------------------------------------
const state = {};            // bridge state signals by name
let ui = {};                 // shared UI state
const taskInfos = {};
let taskListSeen = false;
let armPose = [0, 0, 0, 0, 0, 0];
const tagSeenAt = {};
let standoffSeenAt = null;

function onMessage(msg) {
  switch (msg.t) {
    case 'hello':
      $('log').textContent = '';
      for (const line of msg.log || []) appendLog(line, true);
      ui = msg.ui || {};
      renderUi(ui, true);
      for (const [k, v] of Object.entries(msg.state || {})) onState(k, v);
      break;
    case 'state': onState(msg.k, msg.v); break;
    case 'event': onEvent(msg.k, msg.v); break;
    case 'ui': applyUiPatch(msg.v); break;
    case 'log': appendLog(msg.line, true); break;
    case 'reply': {
      const p = pendingCalls.get(msg.id);
      if (!p) break;
      pendingCalls.delete(msg.id);
      if (msg.ok) p.resolve(msg.result);
      else { appendLog('[web] ' + p.method + ' error: ' + msg.error); p.reject(new Error(msg.error)); }
      break;
    }
  }
}

function onState(name, value) {
  state[name] = value;
  switch (name) {
    case 'arm_state': renderArm(value); break;
    case 'task_state': renderTask(value); break;
    case 'task_list': renderTaskList(value); break;
    case 'lift_state': renderLift(value); break;
    case 'mobile_state': renderMobile(value); break;
    case 'battery_state': renderBattery(value); break;
    case 'estop_state': renderEstop(value); break;
    case 'camera_state': $('chip-camera').textContent = 'CAM ' + value; break;
    case 'lamp_state': tint($('chip-camera'), value ? '#665500' : ''); break;
    case 'standoff_state': renderStandoff(value); break;
  }
}

function onEvent(name, value) {
  switch (name) {
    case 'tag_ids': {
      const lbl = tagLabels[value.cam];
      if (!lbl) break;
      tagSeenAt[value.cam] = performance.now();
      lbl.textContent = 'tags: ' + (value.ids.length ? value.ids.join(', ') : '—');
      break;
    }
    case 'scan_progress': renderScan(value); break;
  }
}

function applyUiPatch(patch) {
  for (const [k, v] of Object.entries(patch)) {
    if (v && typeof v === 'object' && !Array.isArray(v) && ui[k] && typeof ui[k] === 'object' && !Array.isArray(ui[k])) {
      Object.assign(ui[k], v);
    } else {
      ui[k] = v;
    }
  }
  renderUi(patch, false);
}

// ------------------------------------------------------------------
// Chips (the Qt status bar, one function per _on_* handler)
// ------------------------------------------------------------------
function tint(el, colour) {
  el.style.background = colour || '';
  el.style.color = colour ? 'white' : '';
}

function renderArm(st) {
  if (st.pose_valid) armPose = st.tcp_pose.slice();
  AXES.forEach((axis, i) => {
    $('pose-' + axis).textContent = st.pose_valid ? st.tcp_pose[i].toFixed(2) : '—';
  });
  const flag = st.busy ? 'BUSY' : String(st.state || '').toUpperCase();
  $('chip-arm').textContent = 'ARM ' + flag;
  tint($('chip-arm'), st.busy ? '#553311' : '#1b3a1b');
}

function renderTask(st) {
  const name = st.task || '—';
  const idx = st.group_index || 0, total = st.group_total || 0;
  $('chip-task').textContent = 'TASK ' + (st.state || '?') + ' ' + name + (total ? ' ' + idx + '/' + total : '');
  if ('charge_phase' in st) {
    const phase = st.charge_phase || '?';
    // The percentage comes from the LIVE /bms/state (BAT chip), not from the
    // latched /task_state — that one is republished only on changes, so the
    // two chips disagreed (2026-09-15). task_state's own figure is the fallback.
    const bat = state['battery_state'];
    const pct = (bat && bat.percentage != null) ? bat.percentage : st.battery_pct;
    const pctS = pct != null ? ' ' + Math.round(pct) + '%' : '';
    let text, colour;
    if (st.charging) { text = 'CHARGE charging' + pctS; colour = '#4a1a4a'; }
    else if (phase === 'full') { text = 'CHARGE full' + pctS; colour = '#3a3a3a'; }
    else if (phase === 'returning') { text = 'CHARGE returning' + pctS; colour = '#553311'; }
    else if (phase === 'dock_failed') { text = 'CHARGE DOCK FAILED' + pctS; colour = '#7a1f1f'; }
    else { text = 'CHARGE ' + phase + pctS; colour = ''; }
    $('chip-charge').textContent = text;
    tint($('chip-charge'), colour);
    $('lbl-charge-detail').textContent = 'phase: ' + phase + ', BMS charging: ' + (st.charging ? 'yes' : 'no') + pctS;
  }
}

function renderLift(st) {
  const h = st.height_mm;
  let text = 'LIFT ' + (h == null ? '—' : Math.round(h) + 'mm');
  if (st.homed === false) text += ' UNHOMED';
  $('chip-lift').textContent = text;
  tint($('chip-lift'), st.homed === false ? '#553311' : '');
}

function renderMobile(st) {
  let text, colour;
  if (st.emergency_stop) { text = 'BASE E-LATCHED'; colour = '#552222'; }
  else if (st.busy) { text = 'BASE ' + st.busy; colour = '#553311'; }
  else if (st.stop_requested) { text = 'BASE STOPPED'; colour = '#553311'; }
  else { text = 'BASE idle'; colour = '#1b3a1b'; }
  $('chip-mobile').textContent = text;
  tint($('chip-mobile'), colour);
  const res = st.result || null;
  $('lbl-mob-status').textContent = 'base: ' + (st.busy || 'idle') +
    ' | visible tags ' + JSON.stringify(st.visible_tags || []) +
    ' | last tag ' + st.last_known_tag +
    (res ? ' | last result: ' + res.message : '');
}

function renderBattery(st) {
  const pct = st.percentage;
  $('chip-battery').textContent = 'BAT ' + Math.round(pct) + '% ' + st.voltage.toFixed(1) + 'V';
  // 20% is navifra.low_battery_pct in robot.yaml, kept in step by hand.
  tint($('chip-battery'), pct < 20 ? '#553311' : '');
  // the CHARGE chip shows the same percentage: re-render it with the new value
  if (state['task_state']) renderTask(state['task_state']);
}

function renderEstop(active) {
  $('chip-estop').textContent = active ? 'E-STOP ACTIVE' : 'E-STOP clear';
  tint($('chip-estop'), active ? '#b02020' : '#1b3a1b');
}

function renderScan(ev) {
  const chip = $('chip-scan');
  const idx = ev.index || 0, total = ev.total || 0;
  const nOk = ev.n_ok || 0, nFail = ev.n_fail || 0;
  switch (ev.phase) {
    case 'start': chip.textContent = 'SCAN 0/' + total; tint(chip, '#553311'); break;
    case 'move':
      chip.textContent = 'SCAN ' + idx + '/' + total + ' ' + (ev.scan === false ? 'via' : 'pt') + ' ' + (ev.point_id ?? '?');
      break;
    case 'done':
      chip.textContent = 'SCAN ' + idx + '/' + total + ' ok ' + nOk + ' fail ' + nFail;
      tint(chip, nFail ? '#663300' : '#553311');
      break;
    case 'failed':
      chip.textContent = 'SCAN ' + idx + '/' + total + ' ok ' + nOk + ' fail ' + nFail;
      tint(chip, '#7a1f1f');
      break;
    case 'finished': {
      const tail = ev.cancelled ? ' (cancelled)' : '';
      chip.textContent = 'SCAN done ' + nOk + ' ok / ' + nFail + ' fail' + tail;
      tint(chip, nFail ? '#7a1f1f' : '#1b3a1b');
      break;
    }
  }
}

function standoffText(st) {
  const raw = st.raw_mm, tgt = st.target_mm;
  if (st.valid && st.standoff_mm != null) {
    const err = st.err_mm || 0;
    const hint = Math.abs(err) <= 0.2 ? 'ON TARGET' : Math.abs(err).toFixed(2) + ' mm too ' + (err > 0 ? 'close' : 'far');
    return 'standoff: ' + st.standoff_mm.toFixed(2) + ' mm   (target ' + tgt + ': ' + hint + ')   raw ' + (raw >= 0 ? '+' : '') + raw.toFixed(2);
  }
  const which = { far: 'too far', close: 'too close' }[st.side] || 'unknown side';
  return 'standoff: OUT OF RANGE (' + which + ')   raw ' + (raw >= 0 ? '+' : '') + Math.round(raw) + '   — jog Z toward the surface until a value appears';
}

function renderStandoff(st) {
  standoffSeenAt = performance.now();
  const lbl = $('lbl-standoff');
  lbl.textContent = standoffText(st);
  tint(lbl, st.valid ? '#1b3a1b' : '#553311');
}

// Age out per-camera tag lists and the standoff line when their topics stop.
setInterval(() => {
  const now = performance.now();
  for (const cam of STREAM_CAMS) {
    if (tagSeenAt[cam] != null && now - tagSeenAt[cam] > 1500) {
      tagLabels[cam].textContent = 'tags: — (no detections topic)';
      tagSeenAt[cam] = null;
    }
  }
  if (standoffSeenAt != null && now - standoffSeenAt > 1500) {
    const lbl = $('lbl-standoff');
    lbl.textContent = 'standoff: —  (no reading for >1.5 s: keyence node / arm_node?)';
    tint(lbl, '#553311');
    standoffSeenAt = null;
  }
}, 1000);

// ------------------------------------------------------------------
// Shared UI state → widgets
// ------------------------------------------------------------------
function setChecked(el, v) { if (el.checked !== !!v) el.checked = !!v; }

function renderUi(patch, full) {
  const u = ui;
  if (full || 'preview_on' in patch) setChecked($('chk-preview'), u.preview_on);
  if (full || 'preview_hz' in patch) { if (document.activeElement !== $('num-preview-hz')) $('num-preview-hz').value = u.preview_hz; }
  if (full || 'lamp_on' in patch) setChecked($('chk-lamp'), u.lamp_on);
  if (full || 'capture_enabled' in patch) $('btn-capture').disabled = !u.capture_enabled;
  if (full || 'save_dir' in patch) { if (full || !$('txt-save-dir').value) $('txt-save-dir').value = u.save_dir || ''; }
  if (full || 'ra_text' in patch) $('ra-label').textContent = u.ra_text || 'Ra —';
  if (full || 'roi' in patch || 'centre_box' in patch) { views.basler.roi = u.roi || null; views.basler.centreBox = u.centre_box || 0; views.basler.draw(); }
  if (full || 'cam_on' in patch) for (const cam of STREAM_CAMS) setChecked(camOn[cam], (u.cam_on || {})[cam] !== false);
  if (full || 'cam_overlay' in patch) for (const cam of STREAM_CAMS) setChecked(camTags[cam], (u.cam_overlay || {})[cam] !== false);
  if (full || 'standoff_inflight' in patch) $('btn-standoff').disabled = !!u.standoff_inflight;
  if (full || 'mobile_inflight' in patch) for (const b of document.querySelectorAll('button.mob')) b.disabled = !!u.mobile_inflight;
  if (full || 'lift_max_mm' in patch) $('num-lift').max = u.lift_max_mm || 343;
  if (full || 'calib' in patch) renderCalib(u.calib || {}, full);
  if (full || 'handeye' in patch) renderHandeye(u.handeye || {});
  if (full || 'basler_tip' in patch) renderBaslerTip(u.basler_tip || {}, full);
  if (full || 'plugins' in patch) renderPlugins(u.plugins || {});
}

function renderCalib(c, full) {
  const sel = $('sel-calib-plan');
  if (full || sel.options.length !== (c.plans || []).length) {
    const cur = sel.value;
    sel.innerHTML = '';
    (c.plans || []).forEach((label, i) => {
      const o = document.createElement('option');
      o.value = String(i); o.textContent = label; sel.appendChild(o);
    });
    if (cur && sel.querySelector('option[value="' + cur + '"]')) sel.value = cur;
  }
  refreshCalibPlanNote();
  const nodes = $('lbl-calib-nodes');
  if (c.online === true) { nodes.textContent = 'nodes: ONLINE'; nodes.style.color = '#2e7d32'; nodes.style.fontWeight = ''; }
  else if (c.online === false) { nodes.textContent = 'nodes: OFFLINE — run:  roslaunch path_tag_locator path_tag_locator.launch'; nodes.style.color = '#e05050'; nodes.style.fontWeight = 'bold'; }
  $('btn-calib-start').disabled = !!c.running;
  $('lbl-calib-state').textContent = c.state || 'idle';
  const k = c.counts || {};
  $('lbl-calib-counts').textContent = 'ok ' + (k.ok || 0) + '   fail ' + (k.fail || 0) + '   degraded ' + (k.degraded || 0);
  $('lbl-calib-last').textContent = c.last || '—';
}

function refreshCalibPlanNote() {
  const c = ui.calib || {};
  const i = parseInt($('sel-calib-plan').value || '0', 10);
  const kind = (c.kinds || [])[i] || '';
  $('lbl-calib-plan-note').textContent = (c.notes || {})[kind] || '';
}

function renderHandeye(h) {
  const nodes = $('lbl-handeye-nodes');
  if (h.online === true) { nodes.textContent = 'hand-eye node: ONLINE'; nodes.style.color = '#2e7d32'; }
  else if (h.online === false) { nodes.textContent = 'hand-eye node: OFFLINE — run:  roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true'; nodes.style.color = '#e05050'; }
  $('lbl-handeye-state').textContent = h.state || 'samples: —';
  $('lbl-handeye-last').textContent = h.last || '—';
}

function renderBaslerTip(b, full) {
  const dir = $('txt-bt-dir');
  if (b.dir && (full || !dir.value || document.activeElement !== dir)) dir.value = b.dir;
  $('lbl-bt-state').textContent = 'hand ' + (b.n_hand || 0) + ' · basler ' + (b.n_basler || 0) + (b.busy ? '   (working…)' : '');
  $('lbl-bt-last').textContent = b.last || '—';
  const pre = $('pre-bt-report');
  pre.textContent = b.report || '';
  pre.hidden = !b.report;
  for (const el of document.querySelectorAll('button.bt')) el.disabled = !!b.busy;
}

function baslerTipArgs() { return $('txt-bt-dir').value.trim(); }

function renderPlugins(p) {
  const sel = $('sel-plugin');
  const names = p.names || [];
  const cur = sel.value;
  if (Array.from(sel.options).map(o => o.value).join('\n') !== names.join('\n')) {
    sel.innerHTML = '';
    for (const n of names) { const o = document.createElement('option'); o.value = n; o.textContent = n; sel.appendChild(o); }
    if (names.includes(cur)) sel.value = cur;
  }
  $('btn-plugin-run').disabled = !!p.running;
  $('lbl-plugin-dir').textContent = p.dir || '';
}

// ---------- /task_list -> Task tab ----------
function esc(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// One-line summary: the small grey line under each task's NAME in the
// combo dropdown (see renderTaskDropdown), so it stays compact rather
// than wrapping across several lines per entry. The rich, multi-field
// view below (taskDetailHtml) is what a SELECTED task renders in
// #lbl-task-detail.
function taskSummary(info) {
  const mode = info.scan_mode || info.kind || '?';
  const tags = info.tags || [];
  const parts = [String(mode), 'tags ' + (tags.length ? tags.join(',') : '—')];
  if (info.points) {
    let s = info.points + ' pts';
    if (info.traverse_points) s += ' (+' + info.traverse_points + ' traverse)';
    parts.push(s);
  }
  const lift = info.lift_height_mm;
  parts.push(lift == null ? 'lift —' : 'lift ' + Number(lift) + ' mm');
  const files = info.files || [];
  if (files.length) parts.push(files.length === 1 ? files[0] : files.length + ' files');
  if (info.paired_file) parts.push('paired ' + info.paired_file);
  return parts.join(' · ');
}

// What the task actually IS, spelled out field by field (2026-09-15 — the
// dense one-liner above was unreadable for the long RRT-set names). Field
// choice mirrors task_manager._record_task_info / CLAUDE.md's RRT-dialect
// section, so a "joint" task always carries the reach/collision warning.
function taskDetailHtml(info) {
  const kind = info.kind || 'scan';
  const mode = info.scan_mode;
  const tags = info.tags || [];
  const rows = [];
  let modeLine;
  if (kind === 'system') {
    modeLine = 'system task — drives to the start/dock tag, no scan, writes no CSV';
  } else if (kind === 'move') {
    modeLine = 'move only — drives through the tags below in order, no scan';
  } else if (mode === 'pose') {
    modeLine = 'pose mode — end-effector poses (x y z rx ry rz) from the CSV, IK solved per point';
  } else if (mode === 'joint') {
    modeLine = 'joint mode — absolute joint angles (q1..q6) replayed with MoveJ';
  } else {
    modeLine = String(mode || kind);
  }
  rows.push(['tags', tags.length ? tags.join(', ') + '  (drives to each stop, in order)' : '—']);
  if (info.points) {
    let pts = info.points + ' scan point' + (info.points === 1 ? '' : 's');
    if (info.traverse_points) pts += ' <span class="dim">+ ' + info.traverse_points + ' traverse (driven through, not scanned)</span>';
    rows.push(['points', pts]);
  }
  const lift = info.lift_height_mm;
  rows.push(['lift', lift == null ? '— (left alone)' : Number(lift) + ' mm']);
  const files = info.files || [];
  if (files.length) rows.push(['source', files.map(esc).join('<br>')]);
  if (info.paired_file) {
    const why = mode === 'pose' ? 'IK seed' : mode === 'joint' ? 'world x y z for the Ra map' : 'paired file';
    rows.push(['paired', esc(info.paired_file) + '  <span class="dim">(' + why + ')</span>']);
  }
  if (info.ik_seeded_points != null) rows.push(['IK seeded', info.ik_seeded_points + ' / ' + info.points + ' points']);
  if (info.points_with_world_xyz != null) rows.push(['world xyz', info.points_with_world_xyz + ' / ' + info.points + ' points']);
  if (info.groups_filter) rows.push(['groups', info.groups_filter.join(', ') + '  <span class="dim">(explicit subset)</span>']);
  if (info.result_name) rows.push(['result file', esc(info.result_name)]);

  let html = '<div class="task-detail-mode">' + esc(modeLine) + '</div>'
    + '<table class="task-detail-table">' + rows.map(([k, v]) => '<tr><td>' + esc(k) + '</td><td>' + v + '</td></tr>').join('') + '</table>';
  if (mode === 'joint') {
    html += '<div class="task-detail-warn">⚠️ JOINT PATH REPLAY: MoveJ checks neither reach nor '
      + 'collision — only safe if the base is at the planned stop of every tag above. Verify the '
      + 'group→tag assignment before running (CLAUDE.md, scan-CSV section).</div>';
  }
  return html;
}

function renderTaskList(payload) {
  const tasks = payload.tasks || [];
  const firstTime = !taskListSeen;
  taskListSeen = true;
  for (const k of Object.keys(taskInfos)) delete taskInfos[k];
  for (const t of tasks) if (t.name) taskInfos[t.name] = t;
  const names = Object.keys(taskInfos);
  const input = $('txt-task');
  input.placeholder = names.length ? 'task name' : 'no tasks in /task_list';
  // A hand-typed name survives a republish; the placeholder does not.
  if (firstTime && !input.value.trim() && names.length) input.value = names[0];
  // If the dropdown is open (an operator browsing while a republish
  // arrives), keep it showing whatever they've typed so far rather than
  // resetting to the full list under their cursor.
  if (!$('task-dropdown').hidden) renderTaskDropdown(input.value);
  updateTaskDetail();
  appendLog('[task] /task_list: ' + names.length + ' task(s) from ' + (payload.task_dir || '?'));
}

// ---- Task combo dropdown ----
// A plain `<input list=datalist>` only suggests options that contain the
// input's CURRENT text as a substring — so once the field already holds a
// full task name (set above on first load, or left behind after a pick),
// opening it showed exactly that one self-match and every other task
// silently vanished. This dropdown is opened un-filtered on focus/click
// (every known task, always) and only narrows as the operator actively
// types afterward.
function renderTaskDropdown(filterText) {
  const box = $('task-dropdown');
  const q = (filterText || '').trim().toLowerCase();
  const names = Object.keys(taskInfos).filter((n) => !q || n.toLowerCase().includes(q));
  box.innerHTML = '';
  if (!names.length) {
    const empty = document.createElement('div');
    empty.className = 'empty';
    empty.textContent = Object.keys(taskInfos).length ? 'no match' : 'no tasks in /task_list';
    box.appendChild(empty);
    return;
  }
  for (const n of names) {
    const item = document.createElement('div');
    item.className = 'item';
    const nameEl = document.createElement('div');
    nameEl.className = 'name';
    nameEl.textContent = n;
    const sumEl = document.createElement('div');
    sumEl.className = 'summary';
    sumEl.textContent = taskSummary(taskInfos[n]);
    item.append(nameEl, sumEl);
    item.addEventListener('mousedown', (ev) => {
      // mousedown (not click) fires BEFORE the input's blur, so the pick
      // lands before the outside-click handler would otherwise close this
      // on the same interaction and drop it.
      ev.preventDefault();
      $('txt-task').value = n;
      closeTaskDropdown();
      updateTaskDetail();
    });
    box.appendChild(item);
  }
}

function openTaskDropdown() {
  renderTaskDropdown('');           // always the full list on open
  $('task-dropdown').hidden = false;
}

function closeTaskDropdown() {
  $('task-dropdown').hidden = true;
}

function updateTaskDetail() {
  const name = $('txt-task').value.trim();
  const info = taskInfos[name];
  const lbl = $('lbl-task-detail');
  if (!info) {
    lbl.textContent = !name ? '' : (taskListSeen ? name + ': not in /task_list' : name + ': task list not received yet');
    return;
  }
  lbl.innerHTML = '<div class="task-detail-name">' + esc(name) + '</div>' + taskDetailHtml(info);
}

// ------------------------------------------------------------------
// Camera views
// ------------------------------------------------------------------
class CamView {
  constructor(cam, canvas, opts) {
    this.cam = cam;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.bitmap = null;       // decoded wire frame
    this.ow = 0; this.oh = 0; // camera resolution (ROI coordinates)
    this.roiEnabled = !!(opts && opts.roi);
    this.roi = null;          // {x,y,w,h} in camera pixels
    this.centreBox = 0;
    this.drag = null;         // {x0,y0,x1,y1} in canvas px
    this.press = null;
    this.drawn = { x: 0, y: 0, w: 0, h: 0 };
    this.title = cam;
    this.onClick = null;
    this.onDoubleClick = null;
    this.onRoi = null;
    this._bindMouse();
    new ResizeObserver(() => this.draw()).observe(canvas);
  }

  setFrame(bitmap, header) {
    if (this.bitmap) this.bitmap.close();
    this.bitmap = bitmap;
    this.ow = header.ow; this.oh = header.oh;
    this.draw();
  }

  draw() {
    const c = this.canvas, ctx = this.ctx;
    const W = c.clientWidth, H = c.clientHeight;
    if (W === 0 || H === 0) return;
    if (c.width !== W || c.height !== H) { c.width = W; c.height = H; }
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);
    if (!this.bitmap) {
      ctx.fillStyle = '#888';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(this.title || 'no signal', W / 2, H / 2);
      return;
    }
    const bw = this.bitmap.width, bh = this.bitmap.height;
    const s = Math.min(W / bw, H / bh);
    const dw = Math.max(1, Math.round(bw * s)), dh = Math.max(1, Math.round(bh * s));
    const dx = Math.floor((W - dw) / 2), dy = Math.floor((H - dh) / 2);
    ctx.drawImage(this.bitmap, dx, dy, dw, dh);
    this.drawn = { x: dx, y: dy, w: dw, h: dh };
    if (this.centreBox > 0 && this.ow) {
      const side = this.centreBox;
      const r = this.toCanvas({ x: (this.ow - side) / 2, y: (this.oh - side) / 2, w: side, h: side });
      ctx.strokeStyle = '#00ff00'; ctx.lineWidth = 2; ctx.setLineDash([]);
      ctx.strokeRect(r.x, r.y, r.w, r.h);
    }
    if (this.drag) {
      const d = this.drag;
      ctx.strokeStyle = '#ffff00'; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.strokeRect(Math.min(d.x0, d.x1), Math.min(d.y0, d.y1), Math.abs(d.x1 - d.x0), Math.abs(d.y1 - d.y0));
      ctx.setLineDash([]);
    } else if (this.roi && this.ow) {
      const r = this.toCanvas(this.roi);
      ctx.strokeStyle = '#00ffff'; ctx.lineWidth = 2; ctx.setLineDash([]);
      ctx.strokeRect(r.x, r.y, r.w, r.h);
    }
    ctx.fillStyle = '#fff';
    ctx.font = '12px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(this.title + '  ' + this.ow + 'x' + this.oh, 6, 16);
  }

  toCanvas(r) {
    const d = this.drawn;
    const sx = d.w / this.ow, sy = d.h / this.oh;
    return { x: d.x + r.x * sx, y: d.y + r.y * sy, w: Math.max(1, r.w * sx), h: Math.max(1, r.h * sy) };
  }

  toImage(px, py) {
    const d = this.drawn;
    if (!this.ow || d.w <= 0) return null;
    const rx = Math.min(1, Math.max(0, (px - d.x) / d.w));
    const ry = Math.min(1, Math.max(0, (py - d.y) / d.h));
    return { x: Math.round(rx * this.ow), y: Math.round(ry * this.oh) };
  }

  _pos(ev) {
    const r = this.canvas.getBoundingClientRect();
    return { x: ev.clientX - r.left, y: ev.clientY - r.top };
  }

  _bindMouse() {
    const c = this.canvas;
    c.addEventListener('contextmenu', (ev) => ev.preventDefault());
    c.addEventListener('mousedown', (ev) => {
      const p = this._pos(ev);
      if (ev.button === 0) {
        this.press = p;
        if (this.roiEnabled) { this.drag = { x0: p.x, y0: p.y, x1: p.x, y1: p.y }; this.draw(); }
      } else if (ev.button === 2 && this.roiEnabled) {
        this.roi = null;
        if (this.onRoi) this.onRoi(null);
        this.draw();
      }
    });
    c.addEventListener('mousemove', (ev) => {
      if (this.drag) { const p = this._pos(ev); this.drag.x1 = p.x; this.drag.y1 = p.y; this.draw(); }
    });
    const finish = (ev) => {
      if (ev.button !== 0) return;
      const p = this._pos(ev);
      const isClick = this.press && Math.abs(p.x - this.press.x) + Math.abs(p.y - this.press.y) < 6;
      this.press = null;
      if (isClick) {
        // A click selects the view; it must NOT be read as a degenerate
        // ROI drag that clears the stored region.
        this.drag = null; this.draw();
        if (this.onClick) this.onClick(this);
        return;
      }
      if (!this.roiEnabled || !this.drag) return;
      const a = this.toImage(this.drag.x0, this.drag.y0), b = this.toImage(p.x, p.y);
      this.drag = null;
      if (!a || !b) { this.draw(); return; }
      const rect = { x: Math.min(a.x, b.x), y: Math.min(a.y, b.y), w: Math.abs(b.x - a.x), h: Math.abs(b.y - a.y) };
      this.roi = (rect.w > 4 && rect.h > 4) ? rect : null;
      if (this.onRoi) this.onRoi(this.roi);
      this.draw();
    };
    c.addEventListener('mouseup', finish);
    c.addEventListener('mouseleave', (ev) => { if (this.drag) finish(Object.assign({}, { button: 0, clientX: ev.clientX, clientY: ev.clientY })); });
    c.addEventListener('dblclick', (ev) => { if (ev.button === 0 && this.onDoubleClick) this.onDoubleClick(this); });
  }
}

const views = {};
const cells = {};
const tagLabels = {};
const camOn = {};
const camTags = {};
let mainCam = null;

function makeCell(cam) {
  const cell = document.createElement('div');
  cell.className = 'cell';
  const canvas = document.createElement('canvas');
  canvas.className = 'cam';
  cell.appendChild(canvas);
  const ctl = document.createElement('div');
  ctl.className = 'ctl';
  if (STREAM_CAMS.includes(cam)) {
    const on = document.createElement('input'); on.type = 'checkbox'; on.checked = true;
    on.title = '/robot_camera/' + cam + '/set_enabled — stops the detector AND the vendor stream';
    const onL = document.createElement('label'); onL.appendChild(on); onL.append(' on');
    const tags = document.createElement('input'); tags.type = 'checkbox'; tags.checked = true;
    tags.title = 'Show the detector overlay (tag ID, offset from the optical axis, rpy) instead of the raw image';
    const tagsL = document.createElement('label'); tagsL.appendChild(tags); tagsL.append(' tags');
    const lbl = document.createElement('span'); lbl.className = 'tags'; lbl.textContent = 'tags: —';
    ctl.append(onL, tagsL, lbl);
    on.addEventListener('change', () => call('set_stream_camera_enabled', [cam, on.checked]).catch(() => {}));
    tags.addEventListener('change', () => call('set_stream_source', [cam, tags.checked]).catch(() => {}));
    camOn[cam] = on; camTags[cam] = tags; tagLabels[cam] = lbl;
  } else {
    const hint = document.createElement('span'); hint.className = 'hint';
    hint.textContent = 'wrist camera — drag an ROI, right-click clears';
    ctl.appendChild(hint);
  }
  cell.appendChild(ctl);
  const view = new CamView(cam, canvas, { roi: cam === 'basler' });
  view.title = cam === 'basler' ? 'basler (wrist)' : cam;
  view.onClick = () => selectMain(cam);
  view.onDoubleClick = () => toggleMaximise(cam);
  view.onRoi = (roi) => call('set_roi', [roi]).catch(() => {});
  views[cam] = view;
  cells[cam] = cell;
  return cell;
}

function selectMain(cam) {
  if (cam === mainCam || !cells[cam]) return;
  const slot = $('main-slot'), strip = $('thumb-strip');
  slot.innerHTML = '';
  strip.innerHTML = '';
  slot.appendChild(cells[cam]);
  for (const other of Object.keys(cells)) if (other !== cam) strip.appendChild(cells[other]);
  mainCam = cam;
  $('tab-live').textContent = cam + ' (live)';
  showView('live');
  for (const v of Object.values(views)) v.draw();
  appendLog('[UI] main view: ' + cam);
}

function showView(which) {
  $('main-slot').hidden = which !== 'live';
  $('shot-page').hidden = which !== 'shot';
  $('tab-live').classList.toggle('active', which === 'live');
  $('tab-shot').classList.toggle('active', which === 'shot');
  for (const v of Object.values(views)) v.draw();
}

let maximised = false;
function toggleMaximise(cam) {
  // Hide the CONTROL PANEL as well as the strip: the Basler frame is
  // width-limited in this layout, so the control column's width is what
  // actually enlarges the image (measured on the Qt window).
  maximised = !maximised;
  $('controlpanel').classList.toggle('hidden', maximised);
  $('thumb-strip').classList.toggle('hidden', maximised);
  for (const v of Object.values(views)) v.draw();
  appendLog('[UI] ' + (maximised ? 'maximised' : 'restored') + ' view (' + cam + ') — double-click again to toggle');
}

// ---------- binary frames ----------
function onFrame(buf) {
  const dv = new DataView(buf);
  const n = dv.getUint32(0);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, n)));
  const jpeg = new Blob([new Uint8Array(buf, 4 + n)], { type: 'image/jpeg' });
  const view = views[header.cam];
  createImageBitmap(jpeg).then((bmp) => {
    if (view) view.setFrame(bmp, header); else bmp.close();
  }).catch(() => {}).finally(() => send({ t: 'ack', cam: header.cam }));
}

// ------------------------------------------------------------------
// Log
// ------------------------------------------------------------------
const LOG_MAX = 2000;
function appendLog(line, stamped) {
  const pre = $('log');
  if (!stamped) {
    const d = new Date();
    line = d.toTimeString().slice(0, 8) + '  ' + line;
  }
  const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 4;
  pre.textContent += line + '\n';
  const lines = pre.textContent.split('\n');
  if (lines.length > LOG_MAX + 1) pre.textContent = lines.slice(lines.length - LOG_MAX - 1).join('\n');
  if (atBottom) pre.scrollTop = pre.scrollHeight;
}

// ------------------------------------------------------------------
// Wiring
// ------------------------------------------------------------------
function num(id) { return parseFloat($(id).value); }

function init() {
  // Camera cells: Basler in the main slot, the tag cameras in the strip.
  makeCell('basler');
  for (const cam of STREAM_CAMS) makeCell(cam);
  views.captured = new CamView('captured', document.querySelector('#cell-captured canvas'), {});
  views.captured.title = 'captured';
  views.captured.onDoubleClick = () => toggleMaximise('captured');
  selectMain('basler');
  $('tab-live').addEventListener('click', () => showView('live'));
  $('tab-shot').addEventListener('click', () => showView('shot'));

  // Control tabs.
  for (const tab of document.querySelectorAll('#ctl-tabs .tab')) {
    tab.addEventListener('click', () => {
      for (const t of document.querySelectorAll('#ctl-tabs .tab')) t.classList.toggle('active', t === tab);
      for (const p of document.querySelectorAll('#ctl-pages .page')) p.hidden = p.dataset.page !== tab.dataset.tab;
    });
  }

  // Arm grids.
  const pg = $('pose-grid'), jg = $('jog-grid'), tg = $('target-grid');
  AXES.forEach((axis, i) => {
    const h = document.createElement('div'); h.textContent = axis + ' [' + (i < 3 ? 'mm' : 'deg') + ']'; pg.appendChild(h);
  });
  AXES.forEach((axis) => {
    const v = document.createElement('div'); v.className = 'val'; v.id = 'pose-' + axis; v.textContent = '—'; pg.appendChild(v);
  });
  AXES.forEach((axis) => { const h = document.createElement('div'); h.textContent = axis.toUpperCase(); jg.appendChild(h); });
  for (const sign of [+1, -1]) {
    AXES.forEach((axis) => {
      const b = document.createElement('button');
      b.textContent = sign > 0 ? '+' : '−';
      b.dataset.axis = axis; b.dataset.sign = String(sign);
      b.addEventListener('click', () => call('arm_jog', [axis, num('num-step') * sign, num('num-vel')]).catch(() => {}));
      jg.appendChild(b);
    });
  }
  AXES.forEach((axis) => { const h = document.createElement('div'); h.textContent = axis; tg.appendChild(h); });
  AXES.forEach((axis) => {
    const f = document.createElement('input'); f.type = 'text'; f.placeholder = '—'; f.id = 'target-' + axis; tg.appendChild(f);
  });

  // ---- system bar ----
  $('btn-stop-all').addEventListener('click', () => {
    appendLog('[UI] STOP ALL pressed');
    call('stop_all', []).catch(() => {});
  });

  // ---- Collect ----
  $('chk-preview').addEventListener('change', (ev) => call('set_preview', [ev.target.checked]).catch(() => renderUi({ preview_on: 1 }, false)));
  $('chk-lamp').addEventListener('change', (ev) => {
    call('set_lamp', [ev.target.checked]).catch(() => {});
    // The box follows /camera/lamp_state, not the click.
    setChecked($('chk-lamp'), ui.lamp_on);
  });
  $('num-preview-hz').addEventListener('change', () => call('set_preview_rate', [num('num-preview-hz')]).catch(() => {}));
  $('btn-save-dir').addEventListener('click', () => call('set_save_dir', [$('txt-save-dir').value]).catch(() => {}));
  $('btn-capture').addEventListener('click', () => {
    call('capture', [{
      prefix: $('txt-prefix').value, samples: parseInt($('num-samples').value, 10) || 1,
      led: $('chk-led').checked, save: $('chk-save').checked, infer: $('chk-infer').checked,
      roi: $('chk-roi').checked, save_dir: $('txt-save-dir').value,
    }]).then((r) => { if (r && r.ok) showView('shot'); }).catch(() => {});
  });

  // ---- Arm ----
  $('btn-standoff').addEventListener('click', () => call('arm_standoff', [num('num-standoff')]).catch(() => {}));
  $('btn-standoff-cancel').addEventListener('click', () => call('arm_cancel', []).catch(() => {}));
  $('btn-fill').addEventListener('click', () => AXES.forEach((axis, i) => { $('target-' + axis).value = armPose[i].toFixed(2); }));
  $('btn-move').addEventListener('click', () => {
    const fields = AXES.map((axis) => $('target-' + axis).value.trim());
    call('arm_move_cart', [fields, num('num-vel')]).then((r) => {
      if (r && r.ok === false && r.message) appendLog('[move_cart] ' + r.message);
    }).catch(() => {});
  });
  $('btn-arm-home').addEventListener('click', () => call('arm_home', []).catch(() => {}));
  $('btn-arm-cancel').addEventListener('click', () => call('arm_cancel', []).catch(() => {}));

  // ---- Task ----
  $('txt-task').addEventListener('input', () => {
    updateTaskDetail();
    if (!$('task-dropdown').hidden) renderTaskDropdown($('txt-task').value);
  });
  $('txt-task').addEventListener('focus', openTaskDropdown);
  $('txt-task').addEventListener('click', openTaskDropdown);
  $('txt-task').addEventListener('keydown', (ev) => { if (ev.key === 'Escape') closeTaskDropdown(); });
  document.addEventListener('mousedown', (ev) => {
    if (!$('task-combo').contains(ev.target)) closeTaskDropdown();
  });
  $('btn-task').addEventListener('click', () => call('task_command', ['TASK ' + $('txt-task').value.trim()]).catch(() => {}));
  $('btn-reload-tasks').addEventListener('click', () => call('task_command', ['RELOAD_TASKS']).catch(() => {}));
  $('btn-goto').addEventListener('click', () => call('task_command', ['GOTO ' + parseInt($('num-goto').value, 10)]).catch(() => {}));
  $('btn-charge').addEventListener('click', () => call('task_command', ['CHARGE']).catch(() => {}));
  $('btn-undock').addEventListener('click', () => call('task_command', ['UNDOCK']).catch(() => {}));
  const sendRaw = () => {
    const text = $('txt-raw').value.trim();
    if (!text) return;
    call('task_command', [text]).then(() => { $('txt-raw').value = ''; }).catch(() => {});
  };
  $('btn-raw').addEventListener('click', sendRaw);
  $('txt-raw').addEventListener('keydown', (ev) => { if (ev.key === 'Enter') sendRaw(); });
  $('btn-lift-go').addEventListener('click', () => call('lift_goto', [num('num-lift')]).catch(() => {}));
  $('btn-lift-home').addEventListener('click', () => call('lift_home', []).catch(() => {}));
  $('btn-lift-stop').addEventListener('click', () => call('lift_stop', []).catch(() => {}));

  // ---- Mobile ----
  $('btn-mob-fwd').addEventListener('click', () => call('mobile_drive', [num('num-mob-dist'), num('num-mob-v')]).catch(() => {}));
  $('btn-mob-rev').addEventListener('click', () => call('mobile_drive', [-num('num-mob-dist'), num('num-mob-v')]).catch(() => {}));
  $('btn-mob-ccw').addEventListener('click', () => call('mobile_pivot', [num('num-mob-angle'), num('num-mob-w')]).catch(() => {}));
  $('btn-mob-cw').addEventListener('click', () => call('mobile_pivot', [-num('num-mob-angle'), num('num-mob-w')]).catch(() => {}));
  $('btn-mob-stop').addEventListener('click', () => call('mobile_cancel', []).catch(() => {}));
  $('btn-mob-clear').addEventListener('click', () => call('mobile_clear_stop', []).catch(() => {}));

  // ---- Calibration ----
  $('sel-calib-plan').addEventListener('change', refreshCalibPlanNote);
  $('btn-calib-start').addEventListener('click', () => call('calib_start', [parseInt($('sel-calib-plan').value || '0', 10), $('chk-calib-dry').checked]).catch(() => {}));
  $('btn-calib-cancel').addEventListener('click', () => call('calib_cancel', []).catch(() => {}));
  $('btn-locate').addEventListener('click', () => call('calib_locate', [parseInt($('num-locate').value, 10)]).catch(() => {}));
  $('btn-he-auto').addEventListener('click', () => call('handeye_auto', []).catch(() => {}));
  $('btn-he-cancel').addEventListener('click', () => call('handeye_cancel', []).catch(() => {}));
  $('btn-he-capture').addEventListener('click', () => call('handeye_capture', []).catch(() => {}));
  $('btn-he-compute').addEventListener('click', () => call('handeye_compute', []).catch(() => {}));
  $('btn-he-load').addEventListener('click', () => call('handeye_load_latest', []).catch(() => {}));
  $('btn-he-reset').addEventListener('click', () => call('handeye_reset', []).catch(() => {}));
  $('btn-he-status').addEventListener('click', () => call('handeye_status', []).catch(() => {}));
  $('btn-bt-check').addEventListener('click', () => call('basler_tip_check', [baslerTipArgs()]).catch(() => {}));
  $('btn-bt-hand').addEventListener('click', () => call('basler_tip_capture_hand', [baslerTipArgs()]).catch(() => {}));
  $('btn-bt-basler').addEventListener('click', () => {
    const so = $('chk-bt-standoff').checked ? parseFloat($('num-bt-standoff').value) : null;
    call('basler_tip_capture_basler', [baslerTipArgs(), so]).catch(() => {});
  });
  $('btn-bt-status').addEventListener('click', () => call('basler_tip_status', [baslerTipArgs()]).catch(() => {}));
  $('btn-bt-solve').addEventListener('click', () => call('basler_tip_solve', [baslerTipArgs(), $('txt-bt-exclude').value]).catch(() => {}));
  $('btn-bt-verify').addEventListener('click', () => {
    const tag = parseInt($('num-bt-tag').value, 10);
    const so = $('chk-bt-standoff').checked ? parseFloat($('num-bt-standoff').value) : null;
    call('basler_tip_verify', [baslerTipArgs(), tag, so, $('chk-bt-design').checked]).catch(() => {});
  });

  // ---- Scripts ----
  $('btn-plugin-refresh').addEventListener('click', () => call('plugin_refresh', []).catch(() => {}));
  $('btn-plugin-run').addEventListener('click', () => { const n = $('sel-plugin').value; if (n) call('plugin_run', [n]).catch(() => {}); });
  $('btn-plugin-stop').addEventListener('click', () => call('plugin_stop', []).catch(() => {}));

  // ---- Log ----
  $('btn-clear-log').addEventListener('click', () => { $('log').textContent = ''; });

  connect();
}

document.addEventListener('DOMContentLoaded', init);
