#include "http_server.h"
#include "config.h"
#include "lora_wan.h"
#include "sensor.h"
#include "datalog.h"
#include "globals.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFi.h>

WebServer server(80);

// ── Helper: send with Connection:close ───────────────────────────────────
static void sendClose(int code, const char* type, const String& body) {
    server.sendHeader("Connection", "close");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(code, type, body);
    server.client().stop();
}

static void sendPageClose(const char* html) {
    server.sendHeader("Connection", "close");
    server.sendHeader("Cache-Control", "no-cache");
    server.setContentLength(strlen(html));
    server.send(200, "text/html; charset=utf-8", "");
    server.sendContent(html);
    server.client().stop();
}

// ═══════════════════════════════════════════════════════════════════════════
// DASHBOARD HTML — tabbed: Values | Graph
// ═══════════════════════════════════════════════════════════════════════════
static const char DASHBOARD_HTML[] = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SEN66 Monitor</title>
<style>
:root{
  --bg:#0d1117;--panel:#161b22;--border:#30363d;
  --green:#39d353;--yellow:#e3b341;--red:#f85149;
  --blue:#58a6ff;--muted:#8b949e;--text:#e6edf3;
  --mono:'Courier New',monospace;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Trebuchet MS',sans-serif;
  min-height:100vh;padding:1rem 1.2rem}
header{display:flex;justify-content:space-between;align-items:center;
  border-bottom:1px solid var(--border);padding-bottom:.8rem;margin-bottom:1rem}
h1{font-size:1.2rem;font-weight:600;letter-spacing:.04em}
h1 span{color:var(--green);font-family:var(--mono)}
#livebadge{font-size:.72rem;color:var(--muted);font-family:var(--mono)}
#livebadge.ok{color:var(--green)}
#livebadge.err{color:var(--red)}

/* ── Tabs ── */
.tabs{display:flex;gap:.5rem;margin-bottom:1rem;border-bottom:1px solid var(--border);padding-bottom:0}
.tab{padding:.45rem 1.1rem;border:1px solid transparent;border-bottom:none;
  border-radius:6px 6px 0 0;cursor:pointer;font-size:.85rem;color:var(--muted);
  background:transparent;transition:all .15s;position:relative;bottom:-1px}
.tab.active{border-color:var(--border);background:var(--panel);color:var(--text)}
.tabpanel{display:none}
.tabpanel.active{display:block}

/* ── Value cards ── */
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(190px,1fr));gap:.9rem}
.card{background:var(--panel);border:1px solid var(--border);border-radius:8px;
  padding:1rem 1.1rem;position:relative;overflow:hidden;transition:border-color .15s}
.card:hover{border-color:var(--blue)}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--accent,var(--blue))}
.label{font-size:.68rem;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);margin-bottom:.4rem}
.value{font-family:var(--mono);font-size:1.9rem;line-height:1}
.unit{font-size:.8rem;color:var(--muted);margin-left:.25em}
.note{font-size:.7rem;margin-top:.35rem;font-family:var(--mono)}
.warn{color:var(--yellow)}.bad{color:var(--red)}.good{color:var(--green)}

/* ── Graph tab ── */
.graph-controls{display:flex;flex-wrap:wrap;gap:.6rem;align-items:center;margin-bottom:1rem}
.ctrl-group{display:flex;gap:.3rem;align-items:center}
.ctrl-group label{font-size:.72rem;color:var(--muted);margin-right:.2rem}
.btn-set button{padding:.3rem .7rem;border:1px solid var(--border);border-radius:5px;
  background:transparent;color:var(--muted);font-size:.75rem;cursor:pointer;transition:all .15s}
.btn-set button.active{background:var(--blue);color:#0d1117;border-color:var(--blue)}
.ch-toggle{display:flex;align-items:center;gap:.25rem;padding:.25rem .6rem;
  border:1px solid var(--border);border-radius:5px;cursor:pointer;font-size:.75rem;
  user-select:none;transition:all .15s}
.ch-toggle input{display:none}
.ch-toggle.on{border-color:var(--ch-color,var(--blue));color:var(--ch-color,var(--blue))}
.sparkgrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(280px,1fr));gap:.9rem}
.sparkcard{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:.8rem}
.sparkcard h3{font-size:.72rem;letter-spacing:.08em;text-transform:uppercase;
  color:var(--muted);margin-bottom:.5rem}
.sparkcard svg{width:100%;height:80px;overflow:visible}

/* ── Nav + footer ── */
nav{margin-top:1.2rem;display:flex;gap:.6rem;flex-wrap:wrap}
nav a{padding:.4rem .9rem;border:1px solid var(--border);border-radius:6px;
  color:var(--blue);text-decoration:none;font-size:.82rem;transition:all .15s}
nav a:hover{background:var(--blue);color:var(--bg)}
footer{margin-top:1.2rem;font-size:.68rem;color:var(--muted);
  font-family:var(--mono);border-top:1px solid var(--border);padding-top:.8rem}
</style>
</head><body>

<header>
  <h1>SEN66 <span>MONITOR</span></h1>
  <span id="livebadge">connecting...</span>
</header>

<div class="tabs">
  <button class="tab active" onclick="showTab('values',this)">Values</button>
  <button class="tab"        onclick="showTab('graph',this)">Graph</button>
</div>

<!-- ── Values tab ── -->
<div class="tabpanel active" id="tab-values">
  <div class="grid">
    <div class="card" style="--accent:#ef6c00">
      <div class="label">Temperature</div>
      <div class="value"><span id="temp">—</span><span class="unit">°C</span></div>
    </div>
    <div class="card" style="--accent:#26a69a">
      <div class="label">Humidity</div>
      <div class="value"><span id="rh">—</span><span class="unit">%RH</span></div>
    </div>
    <div class="card" style="--accent:#2e7d32">
      <div class="label">CO₂</div>
      <div class="value"><span id="co2">—</span><span class="unit">ppm</span></div>
      <div class="note" id="co2note"></div>
    </div>
    <div class="card" style="--accent:#7b1fa2">
      <div class="label">VOC Index</div>
      <div class="value"><span id="voc">—</span></div>
      <div class="note" id="vocnote"></div>
    </div>
    <div class="card" style="--accent:#c62828">
      <div class="label">NOx Index</div>
      <div class="value"><span id="nox">—</span></div>
    </div>
    <div class="card" style="--accent:#0288d1">
      <div class="label">PM 2.5</div>
      <div class="value"><span id="pm25">—</span><span class="unit">µg/m³</span></div>
      <div class="note" id="pmnote"></div>
    </div>
    <div class="card" style="--accent:#4fc3f7">
      <div class="label">PM 1.0 / 4.0 / 10</div>
      <div class="value" style="font-size:1.05rem;padding-top:.4rem">
        <span id="pm1">—</span> / <span id="pm4">—</span> / <span id="pm10">—</span>
      </div>
      <div class="note">µg/m³</div>
    </div>
    <div class="card" style="--accent:#444466">
      <div class="label">Uptime / Log rows</div>
      <div class="value" style="font-size:1.05rem;padding-top:.4rem">
        <span id="uptime">—</span>
      </div>
      <div class="note"><span id="logrows">—</span> rows · interval <span id="intv">—</span>s</div>
    </div>
    <div class="card" style="--accent:#f0a500">
      <div class="label">LoRa · <span id="lora_mode_disp" style="color:var(--muted)">—</span></div>
      <div class="value" style="font-size:1.05rem;padding-top:.4rem">
        <span id="lora_state">—</span>
      </div>
      <div class="note">↑<span id="lora_uplinks">—</span>  RSSI <span id="lora_rssi">—</span>dBm  SNR <span id="lora_snr">—</span>dB</div>
      <div class="note" id="lora_rx" style="margin-top:.3rem;word-break:break-all"></div>
    </div>
  </div>
</div>

<!-- ── Graph tab ── -->
<div class="tabpanel" id="tab-graph">
  <div class="graph-controls">
    <div class="ctrl-group">
      <label>RANGE</label>
      <div class="btn-set" id="rangebtns">
        <button data-n="20"  onclick="setRange(this)">20</button>
        <button data-n="60"  onclick="setRange(this)" class="active">60</button>
        <button data-n="120" onclick="setRange(this)">120</button>
        <button data-n="999" onclick="setRange(this)">All</button>
      </div>
    </div>
    <div class="ctrl-group">
      <label>CHANNELS</label>
      <label class="ch-toggle on" style="--ch-color:#ef6c00" data-ch="temp">
        <input type="checkbox" checked> Temp
      </label>
      <label class="ch-toggle on" style="--ch-color:#26a69a" data-ch="rh">
        <input type="checkbox" checked> RH
      </label>
      <label class="ch-toggle on" style="--ch-color:#39d353" data-ch="co2">
        <input type="checkbox" checked> CO₂
      </label>
      <label class="ch-toggle on" style="--ch-color:#7b1fa2" data-ch="voc">
        <input type="checkbox" checked> VOC
      </label>
      <label class="ch-toggle on" style="--ch-color:#c62828" data-ch="nox">
        <input type="checkbox" checked> NOx
      </label>
      <label class="ch-toggle on" style="--ch-color:#0288d1" data-ch="pm25">
        <input type="checkbox" checked> PM2.5
      </label>
    </div>
    <button onclick="loadGraph()" style="padding:.3rem .8rem;border:1px solid var(--border);
      border-radius:5px;background:var(--panel);color:var(--blue);font-size:.78rem;cursor:pointer">
      ↻ Refresh
    </button>
  </div>
  <div class="sparkgrid" id="sparkgrid"></div>
</div>

<nav>
  <a href="/log">⬇ Download Log</a>
  <a href="/clearlog" onclick="return confirm('Clear log?')">🗑 Clear Log</a>
  <a href="/config">⚙ Config</a>
</nav>
<footer id="footer">last update: —</footer>

<script>
// ── Config ────────────────────────────────────────────────────────────────
const CH = {
  temp: {label:'Temperature', unit:'°C',   color:'#ef6c00', dec:2},
  rh:   {label:'Humidity',    unit:'%RH',  color:'#26a69a', dec:1},
  co2:  {label:'CO₂',         unit:'ppm',  color:'#39d353', dec:0},
  voc:  {label:'VOC Index',   unit:'',     color:'7b1fa2',  dec:0},
  nox:  {label:'NOx Index',   unit:'',     color:'#c62828', dec:0},
  pm25: {label:'PM 2.5',      unit:'µg/m³',color:'#0288d1', dec:1},
};
let graphN = 60;

// ── Tab switching ─────────────────────────────────────────────────────────
function showTab(id, btn) {
  document.querySelectorAll('.tabpanel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(b=>b.classList.remove('active'));
  document.getElementById('tab-'+id).classList.add('active');
  btn.classList.add('active');
  localStorage.setItem('activeTab', id);
  if (id === 'graph') loadGraph();
}

// ── Channel toggles ───────────────────────────────────────────────────────
function getActiveChannels() {
  const active = {};
  document.querySelectorAll('.ch-toggle').forEach(el => {
    active[el.dataset.ch] = el.classList.contains('on');
  });
  return active;
}

// Debounce helper — prevents hammering /history on rapid toggles
let _graphDebounce = null;
function debouncedGraph() {
  clearTimeout(_graphDebounce);
  _graphDebounce = setTimeout(loadGraph, 150);
}

document.querySelectorAll('.ch-toggle').forEach(el => {
  // preventDefault on mousedown stops browser focus/selection flash
  el.addEventListener('mousedown', e => e.preventDefault());
  el.addEventListener('click', e => {
    e.preventDefault();
    el.classList.toggle('on');
    const ch = el.dataset.ch;
    const saved = JSON.parse(localStorage.getItem('chState')||'{}');
    saved[ch] = el.classList.contains('on');
    localStorage.setItem('chState', JSON.stringify(saved));
    if (document.getElementById('tab-graph').classList.contains('active')) debouncedGraph();
  });
});

// Restore channel state from localStorage
const savedCh = JSON.parse(localStorage.getItem('chState')||'{}');
document.querySelectorAll('.ch-toggle').forEach(el => {
  const ch = el.dataset.ch;
  if (ch in savedCh) {
    el.classList.toggle('on', savedCh[ch]);
  }
});

// ── Range buttons ─────────────────────────────────────────────────────────
function setRange(btn) {
  document.querySelectorAll('#rangebtns button').forEach(b=>b.classList.remove('active'));
  btn.classList.add('active');
  graphN = parseInt(btn.dataset.n);
  localStorage.setItem('graphN', graphN);
  debouncedGraph();
}

// Restore range
const savedN = localStorage.getItem('graphN');
if (savedN) {
  graphN = parseInt(savedN);
  document.querySelectorAll('#rangebtns button').forEach(b=>{
    b.classList.toggle('active', parseInt(b.dataset.n) === graphN);
  });
}

// ── SVG sparkline ─────────────────────────────────────────────────────────
function makeSpark(values, color, w, h) {
  const pts = values.filter(v => v !== null);
  if (pts.length < 2) return '<text x="50%" y="50%" text-anchor="middle" fill="#444" font-size="11">no data</text>';

  const min = Math.min(...pts);
  const max = Math.max(...pts);
  const range = max - min || 1;
  const pad = 4;
  const W = w - pad*2, H = h - pad*2;

  const coords = values.map((v, i) => {
    const x = pad + (i / (values.length - 1)) * W;
    const y = v === null ? null : pad + H - ((v - min) / range) * H;
    return {x, y, v};
  });

  // Build path with gaps for null
  let path = '';
  let inSeg = false;
  for (const p of coords) {
    if (p.y === null) { inSeg = false; continue; }
    if (!inSeg) { path += `M${p.x.toFixed(1)},${p.y.toFixed(1)}`; inSeg = true; }
    else        { path += `L${p.x.toFixed(1)},${p.y.toFixed(1)}`; }
  }

  // Area fill (last valid point back to baseline)
  const first = coords.find(p=>p.y!==null);
  const last  = [...coords].reverse().find(p=>p.y!==null);
  let area = '';
  if (first && last) {
    area = `M${first.x.toFixed(1)},${(pad+H).toFixed(1)}` + path.substring(1) +
           `L${last.x.toFixed(1)},${(pad+H).toFixed(1)}Z`;
  }

  return `
    <defs>
      <linearGradient id="g${color.replace('#','')}" x1="0" y1="0" x2="0" y2="1">
        <stop offset="0%" stop-color="${color}" stop-opacity="0.3"/>
        <stop offset="100%" stop-color="${color}" stop-opacity="0"/>
      </linearGradient>
    </defs>
    <path d="${area}" fill="url(#g${color.replace('#','')})" stroke="none"/>
    <path d="${path}" fill="none" stroke="${color}" stroke-width="1.5" stroke-linejoin="round"/>
    <text x="${pad}" y="${h-1}" fill="#666" font-size="9">${min.toFixed(1)}</text>
    <text x="${(w/2).toFixed(0)}" y="${h-1}" fill="#666" font-size="9" text-anchor="middle">${((min+max)/2).toFixed(1)}</text>
    <text x="${w-pad}" y="${h-1}" fill="#666" font-size="9" text-anchor="end">${max.toFixed(1)}</text>`;
}

// ── Load graph data ───────────────────────────────────────────────────────
async function loadGraph() {
  const active = getActiveChannels();
  const chParam = Object.entries(active).filter(([,v])=>v).map(([k])=>k).join(',');
  if (!chParam) { document.getElementById('sparkgrid').innerHTML='<p style="color:var(--muted);font-size:.8rem">Select at least one channel.</p>'; return; }

  let data;
  try {
    const r = await fetch(`/history?n=${graphN}&ch=${chParam}`);
    if (!r.ok) throw new Error(r.status);
    data = await r.json();
  } catch(e) {
    document.getElementById('sparkgrid').innerHTML = `<p style="color:var(--red);font-size:.8rem">Error loading history: ${e.message}</p>`;
    return;
  }

  const labels = data.labels || [];
  const grid = document.getElementById('sparkgrid');
  grid.innerHTML = '';

  const W = 260, H = 80;
  for (const [ch, info] of Object.entries(CH)) {
    if (!active[ch] || !data[ch]) continue;
    const card = document.createElement('div');
    card.className = 'sparkcard';
    const latest = data[ch].filter(v=>v!==null).slice(-1)[0];
    const latestStr = latest != null ? latest.toFixed(info.dec) + (info.unit ? ' '+info.unit : '') : '—';
    const xMin = labels[0]||0, xMax = labels[labels.length-1]||0;
    const duration = xMax - xMin;
    const timeLabel = duration < 120 ? `${duration}s` : duration < 7200 ? `${(duration/60).toFixed(0)}min` : `${(duration/3600).toFixed(1)}h`;
    card.innerHTML = `
      <h3>${info.label} <span style="color:${info.color};float:right">${latestStr}</span></h3>
      <svg viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg">
        ${makeSpark(data[ch], info.color, W, H)}
      </svg>
      <div style="font-size:.65rem;color:#444;text-align:right;margin-top:2px">${labels.length} pts · ${timeLabel}</div>`;
    grid.appendChild(card);
  }
  if (!grid.children.length)
    grid.innerHTML = '<p style="color:var(--muted);font-size:.8rem">No log data yet — wait for readings to accumulate.</p>';
}

// ── Live value refresh ────────────────────────────────────────────────────
async function refreshValues() {
  try {
    const r = await fetch('/data');
    if (!r.ok) throw new Error(r.status);
    const d = await r.json();

    document.getElementById('livebadge').textContent='● LIVE';
    document.getElementById('livebadge').className='ok';

    document.getElementById('temp').textContent   = d.temp_C.toFixed(2);
    document.getElementById('rh').textContent     = d.rh.toFixed(1);
    document.getElementById('co2').textContent    = d.co2_ppm;
    document.getElementById('voc').textContent    = d.voc_idx.toFixed(0);
    document.getElementById('nox').textContent    = d.nox_idx.toFixed(0);
    document.getElementById('uptime').textContent = fmtUp(d.uptime_s);
    document.getElementById('logrows').textContent= d.log_rows;
    document.getElementById('intv').textContent        = d.interval_s;
    // LoRa card
    document.getElementById('lora_mode_disp').textContent = d.lora_mode||'disabled';
    if(d.lora_mode && d.lora_mode !== 'disabled'){
      const ls = document.getElementById('lora_state');
      ls.textContent = d.lora_state;
      ls.className   = (d.lora_state==='send_ok'||d.lora_state==='joined') ? 'good' :
                        d.lora_state==='joining' ? 'warn' : 'bad';
      document.getElementById('lora_uplinks').textContent = d.lora_uplinks;
      document.getElementById('lora_rssi').textContent    = d.lora_rssi;
      document.getElementById('lora_snr').textContent     = d.lora_snr ? d.lora_snr.toFixed(1) : '—';
      if(d.lora_last_rx && d.lora_last_rx.length > 0)
        document.getElementById('lora_rx').textContent = '← ' + d.lora_last_rx;
    } else {
      document.getElementById('lora_state').textContent = 'disabled';
    }
    document.getElementById('footer').textContent = 'last update: '+new Date().toLocaleTimeString()+' · '+d.wifi_mode+' · '+d.ip;

    if (d.pm_saturated) {
      ['pm1','pm25','pm4','pm10'].forEach(id=>document.getElementById(id).textContent='SAT');
      setNote('pmnote','⚠ sensor saturated','warn');
    } else {
      document.getElementById('pm1').textContent  = d.pm1.toFixed(1);
      document.getElementById('pm25').textContent = d.pm25.toFixed(1);
      document.getElementById('pm4').textContent  = d.pm4.toFixed(1);
      document.getElementById('pm10').textContent = d.pm10.toFixed(1);
      setNote('pmnote','','');
    }

    const c=d.co2_ppm;
    if(c<400)setNote('co2note','warming up','warn');
    else if(c<1000)setNote('co2note','good','good');
    else if(c<2000)setNote('co2note','elevated','warn');
    else setNote('co2note','poor','bad');

    const v=d.voc_idx;
    if(v<100)setNote('vocnote','good','good');
    else if(v<200)setNote('vocnote','moderate','warn');
    else setNote('vocnote','poor','bad');

  } catch(e) {
    document.getElementById('livebadge').textContent='● '+e.message;
    document.getElementById('livebadge').className='err';
  }
}

function setNote(id,txt,cls){const el=document.getElementById(id);el.textContent=txt;el.className='note '+(cls||'');}
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),ss=s%60;return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(ss).padStart(2,'0')}`;}

// ── Init ──────────────────────────────────────────────────────────────────
// Restore active tab
const savedTab = localStorage.getItem('activeTab');
if (savedTab === 'graph') {
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.tabpanel').forEach(p=>p.classList.remove('active'));
  document.querySelector('[onclick*="graph"]').classList.add('active');
  document.getElementById('tab-graph').classList.add('active');
  loadGraph();
}

refreshValues();
setInterval(refreshValues, 5000);
</script>
</body></html>
)HTML";

// ═══════════════════════════════════════════════════════════════════════════
// CONFIG PAGE HTML
// ═══════════════════════════════════════════════════════════════════════════
static const char CONFIG_HTML[] = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SEN66 Config</title>
<style>
:root{--bg:#0d1117;--panel:#161b22;--border:#30363d;
  --blue:#58a6ff;--green:#39d353;--muted:#8b949e;--text:#e6edf3}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Trebuchet MS',sans-serif;
  padding:1.5rem;max-width:480px}
h1{font-size:1.15rem;margin-bottom:1.5rem;border-bottom:1px solid var(--border);padding-bottom:.75rem}
h1 a{color:var(--blue);text-decoration:none;font-size:.8rem;float:right;font-weight:300;line-height:1.8}
label{display:block;font-size:.68rem;letter-spacing:.08em;text-transform:uppercase;
  color:var(--muted);margin-bottom:.3rem;margin-top:1rem}
input{width:100%;background:var(--panel);border:1px solid var(--border);border-radius:6px;
  padding:.55rem .8rem;color:var(--text);font-family:'Courier New',monospace;font-size:.88rem;outline:none}
input:focus{border-color:var(--blue)}
button{margin-top:1.5rem;width:100%;padding:.7rem;background:var(--blue);color:#0d1117;
  border:none;border-radius:6px;font-size:.95rem;font-weight:600;cursor:pointer}
button:hover{opacity:.85}
#msg{margin-top:.8rem;padding:.55rem;border-radius:6px;font-size:.82rem;display:none;text-align:center}
#msg.ok{background:#1a3a1a;color:var(--green);display:block}
#msg.err{background:#3a1a1a;color:#f85149;display:block}
.hint{font-size:.68rem;color:var(--muted);margin-top:.2rem}
</style>
</head><body>
<h1>Configuration <a href="/">← Dashboard</a></h1>
<form id="cfg">
  <label>WiFi SSID</label>
  <input name="wifi_ssid" id="wifi_ssid" placeholder="leave blank to stay in AP mode">
  <label>WiFi Password</label>
  <input name="wifi_pass" id="wifi_pass" type="password">
  <label>AP SSID (fallback hotspot name)</label>
  <input name="ap_ssid" id="ap_ssid">
  <label>AP Password</label>
  <input name="ap_pass" id="ap_pass" type="password">
  <label>Logging Interval (seconds)</label>
  <input name="interval_s" id="interval_s" type="number" min="1" max="3600">
  <p class="hint">How often a reading is written to the CSV log.</p>
  <label>Max Log Rows</label>
  <input name="log_max" id="log_max" type="number" min="10" max="5000">
  <p class="hint">Oldest rows removed when limit is reached (ring buffer).</p>
  <hr style="border-color:var(--border);margin:1.5rem 0">
  <h2 style="font-size:.9rem;color:var(--muted);margin-bottom:.5rem;font-weight:400">LoRa (EU868)</h2>
  <label>Mode</label>
  <select name="lora_mode" id="lora_mode" onchange="loraMode(this.value)"
    style="width:100%;background:var(--panel);border:1px solid var(--border);
    border-radius:6px;padding:.55rem .8rem;color:var(--text);font-size:.88rem;outline:none">
    <option value="disabled">Disabled</option>
    <option value="raw">Raw (peer-to-peer, no gateway needed)</option>
    <option value="lorawan">LoRaWAN OTAA (via ChirpStack gateway)</option>
  </select>
  <div id="lora_wan_fields" style="display:none">
    <label>Device EUI <span style="color:var(--muted);font-weight:300">(16 hex, from ChirpStack)</span></label>
    <input name="lora_dev_eui" id="lora_dev_eui" placeholder="0000000000000000" maxlength="16">
    <label>Application / Join EUI <span style="color:var(--muted);font-weight:300">(16 hex)</span></label>
    <input name="lora_app_eui" id="lora_app_eui" placeholder="0000000000000000" maxlength="16">
    <label>Application Key <span style="color:var(--muted);font-weight:300">(32 hex)</span></label>
    <input name="lora_app_key" id="lora_app_key" placeholder="00000000000000000000000000000000" maxlength="32">
  </div>
  <label>LoRa TX Interval (seconds)</label>
  <input name="lora_interval_s" id="lora_interval_s" type="number" min="30" max="86400">
  <p class="hint">Min 30s. EU868 duty cycle 1% — longer is better.</p>
  <button type="submit">Save &amp; Reboot</button>
</form>
<div id="msg"></div>
<script>
fetch('/configjson').then(r=>r.json()).then(d=>{
  document.getElementById('wifi_ssid').value  = d.wifi_ssid||'';
  document.getElementById('wifi_pass').value  = d.wifi_pass||'';
  document.getElementById('ap_ssid').value    = d.ap_ssid||'';
  document.getElementById('ap_pass').value    = d.ap_pass||'';
  document.getElementById('interval_s').value    = d.interval_s||10;
  document.getElementById('log_max').value       = d.log_max||1000;
  document.getElementById('lora_mode').value      = d.lora_mode||'disabled';
  loraMode(d.lora_mode||'disabled');
  document.getElementById('lora_dev_eui').value   = d.lora_dev_eui||'';
  document.getElementById('lora_app_eui').value   = d.lora_app_eui||'';
  document.getElementById('lora_app_key').value   = d.lora_app_key||'';
  document.getElementById('lora_interval_s').value= d.lora_interval_s||60;
});
function loraMode(v) {
  document.getElementById('lora_wan_fields').style.display =
    v === 'lorawan' ? 'block' : 'none';
}
document.getElementById('cfg').addEventListener('submit', async e=>{
  e.preventDefault();
  const body = new URLSearchParams(new FormData(e.target)).toString();
  const r = await fetch('/config',{method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},body});
  const msg = document.getElementById('msg');
  if(r.ok){msg.textContent='Saved! Rebooting in 2s…';msg.className='ok';}
  else    {msg.textContent='Save failed';msg.className='err';}
});
</script>
</body></html>
)HTML";

// ═══════════════════════════════════════════════════════════════════════════
// HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot()       { if(DBG_WEB) Serial.println("[web] GET /");          sendPageClose(DASHBOARD_HTML); }
void handleConfigPage() { if(DBG_WEB) Serial.println("[web] GET /config");    sendPageClose(CONFIG_HTML); }

void handleData() {
    if(DBG_WEB) Serial.println("[web] GET /data");
    JsonDocument doc;
    doc["ts_ms"]        = latest.ts_ms;
    doc["temp_C"]       = latest.temp;
    doc["rh"]           = latest.rh;
    doc["co2_ppm"]      = latest.co2;
    doc["voc_idx"]      = latest.voc;
    doc["nox_idx"]      = latest.nox;
    doc["pm1"]          = latest.pm1;
    doc["pm25"]         = latest.pm25;
    doc["pm4"]          = latest.pm4;
    doc["pm10"]         = latest.pm10;
    doc["pm_saturated"] = latest.pm_sat;
    doc["valid"]        = latest.valid;
    doc["uptime_s"]     = millis() / 1000;
    doc["log_rows"]     = logRowCount;
    doc["interval_s"]   = cfg.interval_s;
    doc["wifi_mode"]    = staMode ? "STA" : "AP";
    doc["ip"]           = staMode
                          ? WiFi.localIP().toString()
                          : WiFi.softAPIP().toString();
    doc["lora_state"]   = loraStateStr();
    doc["lora_uplinks"] = loraUplinkCount;
    doc["lora_rssi"]    = loraLastRssi;
    doc["lora_snr"]     = loraLastSnr;
    doc["lora_mode"]    = cfg.lora_mode;
    doc["lora_last_rx"] = loraLastRx;
    String out; serializeJson(doc, out);
    sendClose(200, "application/json", out);
}

void handleHistory() {
    if(DBG_WEB) Serial.printf("[web] GET /history?%s\n", server.uri().c_str());

    uint16_t n = 60;
    uint8_t  channels = 0b111111;  // all by default

    if (server.hasArg("n"))
        n = constrain(server.arg("n").toInt(), 1, 999);

    if (server.hasArg("ch")) {
        channels = 0;
        String ch = server.arg("ch");
        if (ch.indexOf("temp") >= 0)  channels |= (1<<0);
        if (ch.indexOf("rh")   >= 0)  channels |= (1<<1);
        if (ch.indexOf("co2")  >= 0)  channels |= (1<<2);
        if (ch.indexOf("voc")  >= 0)  channels |= (1<<3);
        if (ch.indexOf("nox")  >= 0)  channels |= (1<<4);
        if (ch.indexOf("pm25") >= 0)  channels |= (1<<5);
    }

    String json = getHistoryJson(n, channels);
    sendClose(200, "application/json", json);
}

void handleLog() {
    if(DBG_WEB) Serial.println("[web] GET /log");
    if (!LittleFS.exists("/log.csv")) { sendClose(404, "text/plain", "No log yet"); return; }
    File f = LittleFS.open("/log.csv", "r");
    server.sendHeader("Connection", "close");
    server.sendHeader("Content-Disposition", "attachment; filename=log.csv");
    server.streamFile(f, "text/csv");
    f.close();
    server.client().stop();
}

void handleClearLog() {
    if(DBG_WEB) Serial.println("[web] GET /clearlog");
    LittleFS.remove("/log.csv");
    logRowCount = 0;
    initLog();
    server.sendHeader("Location", "/");
    server.sendHeader("Connection", "close");
    server.send(303);
    server.client().stop();
}

void handleConfigJson() {
    if(DBG_WEB) Serial.println("[web] GET /configjson");
    JsonDocument doc;
    doc["wifi_ssid"]  = cfg.wifi_ssid;
    doc["wifi_pass"]  = cfg.wifi_pass;
    doc["ap_ssid"]    = cfg.ap_ssid;
    doc["ap_pass"]    = cfg.ap_pass;
    doc["interval_s"]      = cfg.interval_s;
    doc["log_max"]         = cfg.log_max;
    doc["lora_mode"]       = cfg.lora_mode;
    doc["lora_dev_eui"]    = cfg.lora_dev_eui;
    doc["lora_app_eui"]    = cfg.lora_app_eui;
    doc["lora_app_key"]    = cfg.lora_app_key;
    doc["lora_interval_s"] = cfg.lora_interval_s;
    String out; serializeJson(doc, out);
    sendClose(200, "application/json", out);
}

void handleConfigPost() {
    if(DBG_WEB) Serial.println("[web] POST /config");
    if (server.hasArg("wifi_ssid")) strlcpy(cfg.wifi_ssid, server.arg("wifi_ssid").c_str(), sizeof(cfg.wifi_ssid));
    if (server.hasArg("wifi_pass")) strlcpy(cfg.wifi_pass, server.arg("wifi_pass").c_str(), sizeof(cfg.wifi_pass));
    if (server.hasArg("ap_ssid"))   strlcpy(cfg.ap_ssid,   server.arg("ap_ssid").c_str(),   sizeof(cfg.ap_ssid));
    if (server.hasArg("ap_pass"))   strlcpy(cfg.ap_pass,   server.arg("ap_pass").c_str(),   sizeof(cfg.ap_pass));
    if (server.hasArg("interval_s"))   cfg.interval_s      = server.arg("interval_s").toInt();
    if (server.hasArg("log_max"))      cfg.log_max         = server.arg("log_max").toInt();
    if (server.hasArg("lora_mode"))    strlcpy(cfg.lora_mode, server.arg("lora_mode").c_str(), sizeof(cfg.lora_mode));
    if (server.hasArg("lora_dev_eui")) strlcpy(cfg.lora_dev_eui, server.arg("lora_dev_eui").c_str(), sizeof(cfg.lora_dev_eui));
    if (server.hasArg("lora_app_eui")) strlcpy(cfg.lora_app_eui, server.arg("lora_app_eui").c_str(), sizeof(cfg.lora_app_eui));
    if (server.hasArg("lora_app_key")) strlcpy(cfg.lora_app_key, server.arg("lora_app_key").c_str(), sizeof(cfg.lora_app_key));
    if (server.hasArg("lora_interval_s")) cfg.lora_interval_s = server.arg("lora_interval_s").toInt();
    if (saveConfig()) {
        sendClose(200, "text/plain", "OK");
        delay(500);
        rp2040.reboot();
    } else {
        sendClose(500, "text/plain", "Write failed");
    }
}

// ── LoRa status + log endpoint ───────────────────────────────────────────
void handleLoraStatus() {
    if(DBG_WEB) Serial.println("[web] GET /lorastatus");
    JsonDocument doc;
    doc["mode"]     = loraModeStr();
    doc["state"]    = loraStateStr();
    doc["uplinks"]  = loraUplinkCount;
    doc["rssi"]     = loraLastRssi;
    doc["snr"]      = loraLastSnr;
    doc["last_rx"]  = loraLastRx;
    doc["log"]      = serialized(loraGetLog(10));
    String out; serializeJson(doc, out);
    sendClose(200, "application/json", out);
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setupWebServer() {
    server.on("/",          HTTP_GET,  handleRoot);
    server.on("/data",      HTTP_GET,  handleData);
    server.on("/history",   HTTP_GET,  handleHistory);
    server.on("/log",       HTTP_GET,  handleLog);
    server.on("/clearlog",  HTTP_GET,  handleClearLog);
    server.on("/config",    HTTP_GET,  handleConfigPage);
    server.on("/config",    HTTP_POST, handleConfigPost);
    server.on("/configjson",HTTP_GET,  handleConfigJson);
    server.on("/lorastatus",HTTP_GET,   handleLoraStatus);
    server.onNotFound([]{
        if(DBG_WEB) Serial.printf("[web] 404: %s\n", server.uri().c_str());
        sendClose(404, "text/plain", "Not found");
    });
    server.begin();
    Serial.println("[web] Server started");
}