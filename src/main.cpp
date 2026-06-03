// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Environment Monitor — Phase 2
// Pico W  ·  Earle Philhower arduino-pico core  ·  PlatformIO
//
// WiFi:   tries STA mode first (config.json credentials, 10s timeout)
//         falls back to AP mode  (192.168.4.1)
// Web:    live dashboard · log download/clear · config editor
// Serial: CLI  →  help | status | read | stream
// Flash:  LittleFS  →  /config.json · /log.csv (ring-buffer, max rows)
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSen66.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ── NO_ERROR guard ────────────────────────────────────────────────────────
#ifdef NO_ERROR
  #undef NO_ERROR
#endif
#define NO_ERROR 0

// ── SEN66 sentinels ───────────────────────────────────────────────────────
static constexpr uint16_t UINT16_INVALID = 0xFFFF;
static constexpr int16_t  INT16_INVALID  = 0x7FFF;
static constexpr float    PM_SATURATED   = 6016.0f;  // 0xEB00 / 10

// ── Default config ────────────────────────────────────────────────────────
struct Config {
    char     wifi_ssid[64]  = "";
    char     wifi_pass[64]  = "";
    char     ap_ssid[32]    = "SEN66-Monitor";
    char     ap_pass[32]    = "sen66pass";
    uint16_t interval_s     = 10;   // logging interval in seconds
    uint16_t log_max        = 1000; // max CSV rows before wrap
};
Config cfg;

// ── Runtime state ─────────────────────────────────────────────────────────
struct Measurement {
    bool     valid      = false;
    bool     pm_sat     = false;
    float    pm1, pm25, pm4, pm10;
    float    rh, temp;
    float    voc, nox;
    uint16_t co2;
    uint32_t ts_ms;
};
Measurement latest;

bool        streamEnabled  = false;   // serial streaming toggle
bool        staMode        = false;   // true = joined existing WiFi

// Debug level flags (OR-able):  0=off  1=web  2=sensor  4=mqtt(future)
uint8_t     debugLevel     = 0;
#define DBG_WEB    (debugLevel & 1)
#define DBG_SENSOR (debugLevel & 2)
#define DBG_MQTT   (debugLevel & 4)
uint32_t    lastLogTime    = 0;
uint32_t    logRowCount    = 0;
char        errMsg[64];

SensirionI2cSen66 sen66;
WebServer         server(80);

// ═══════════════════════════════════════════════════════════════════════════
// LITTLEFS  ·  config
// ═══════════════════════════════════════════════════════════════════════════

void loadConfig() {
    if (!LittleFS.exists("/config.json")) {
        Serial.println("[cfg] No config.json — using defaults");
        return;
    }
    File f = LittleFS.open("/config.json", "r");
    if (!f) return;

    JsonDocument doc;
    if (deserializeJson(doc, f) != DeserializationError::Ok) {
        Serial.println("[cfg] Parse error — using defaults");
        f.close(); return;
    }
    f.close();

    strlcpy(cfg.wifi_ssid, doc["wifi_ssid"] | "", sizeof(cfg.wifi_ssid));
    strlcpy(cfg.wifi_pass, doc["wifi_pass"] | "", sizeof(cfg.wifi_pass));
    strlcpy(cfg.ap_ssid,   doc["ap_ssid"]   | "SEN66-Monitor", sizeof(cfg.ap_ssid));
    strlcpy(cfg.ap_pass,   doc["ap_pass"]   | "sen66pass",     sizeof(cfg.ap_pass));
    cfg.interval_s = doc["interval_s"] | 10;
    cfg.log_max    = doc["log_max"]    | 1000;
    Serial.printf("[cfg] Loaded: ssid=%s interval=%ds log_max=%d\n",
        cfg.wifi_ssid, cfg.interval_s, cfg.log_max);
}

bool saveConfig() {
    File f = LittleFS.open("/config.json", "w");
    if (!f) return false;
    JsonDocument doc;
    doc["wifi_ssid"]   = cfg.wifi_ssid;
    doc["wifi_pass"]   = cfg.wifi_pass;
    doc["ap_ssid"]     = cfg.ap_ssid;
    doc["ap_pass"]     = cfg.ap_pass;
    doc["interval_s"]  = cfg.interval_s;
    doc["log_max"]     = cfg.log_max;
    serializeJson(doc, f);
    f.close();
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// LITTLEFS  ·  CSV log
// ═══════════════════════════════════════════════════════════════════════════

static const char CSV_HEADER[] =
    "ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm\n";

void initLog() {
    if (!LittleFS.exists("/log.csv")) {
        File f = LittleFS.open("/log.csv", "w");
        if (f) { f.print(CSV_HEADER); f.close(); }
        logRowCount = 0;
        return;
    }
    // Count existing rows (for ring-buffer tracking)
    File f = LittleFS.open("/log.csv", "r");
    logRowCount = 0;
    while (f.available()) {
        if (f.read() == '\n') logRowCount++;
    }
    f.close();
    if (logRowCount > 0) logRowCount--;  // subtract header
    Serial.printf("[log] Existing log: %lu rows\n", logRowCount);
}

void appendLog(const Measurement& m) {
    if (!m.valid) return;

    // Ring-buffer: if at limit, rewrite file keeping last (log_max-1) rows
    if (logRowCount >= cfg.log_max) {
        Serial.println("[log] Ring wrap — trimming oldest entries");
        File src = LittleFS.open("/log.csv", "r");
        File tmp = LittleFS.open("/log.tmp", "w");
        if (!src || !tmp) return;

        // Skip header + oldest rows, keep newest (log_max - 1)
        uint32_t skipRows = logRowCount - (cfg.log_max - 1) + 1; // +1 for header
        uint32_t skipped  = 0;
        tmp.print(CSV_HEADER);
        while (src.available()) {
            String line = src.readStringUntil('\n');
            if (skipped < skipRows) { skipped++; continue; }
            if (line.length() > 5) { tmp.print(line); tmp.print('\n'); }
        }
        src.close(); tmp.close();
        LittleFS.remove("/log.csv");
        LittleFS.rename("/log.tmp", "/log.csv");
        logRowCount = cfg.log_max - 1;
    }

    File f = LittleFS.open("/log.csv", "a");
    if (!f) return;

    auto fmtFloat = [](float v, bool sat, char* buf, int dec) {
        if (sat) strcpy(buf, "sat");
        else     dtostrf(v, 1, dec, buf);
    };

    char s1[10], s2[10], s3[10], s4[10];
    fmtFloat(m.pm1,  m.pm_sat, s1, 1);
    fmtFloat(m.pm25, m.pm_sat, s2, 1);
    fmtFloat(m.pm4,  m.pm_sat, s3, 1);
    fmtFloat(m.pm10, m.pm_sat, s4, 1);

    f.printf("%lu,%s,%s,%s,%s,%.1f,%.2f,%.1f,%.1f,%u\n",
        m.ts_ms, s1, s2, s3, s4,
        m.rh, m.temp, m.voc, m.nox, m.co2);
    f.close();
    logRowCount++;
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI
// ═══════════════════════════════════════════════════════════════════════════

void startWiFi() {
    // Try STA first if credentials exist
    if (strlen(cfg.wifi_ssid) > 0) {
        Serial.printf("[wifi] Trying STA: %s\n", cfg.wifi_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
            delay(250); Serial.print(".");
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            staMode = true;
            Serial.printf("[wifi] Connected! IP: %s\n",
                WiFi.localIP().toString().c_str());
            return;
        }
        Serial.println("[wifi] STA failed — falling back to AP");
    }

    // AP fallback — must call softAPConfig BEFORE softAP to start DHCP server
    // Without this the Pico W AP assigns no IP to connecting clients
    WiFi.mode(WIFI_AP);
    IPAddress apIP(192, 168, 4, 1);
    IPAddress apGW(192, 168, 4, 1);
    IPAddress apSN(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apGW, apSN);
    delay(100);   // let config settle before starting AP
    WiFi.softAP(cfg.ap_ssid, cfg.ap_pass);
    delay(500);   // give DHCP server time to start
    staMode = false;
    Serial.printf("[wifi] AP mode: SSID=%s  IP=%s\n",
        cfg.ap_ssid, WiFi.softAPIP().toString().c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER — HTML strings
// ═══════════════════════════════════════════════════════════════════════════

// ── Dashboard page ────────────────────────────────────────────────────────
static const char DASHBOARD_HTML[] = R"rawhtml(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SEN66 Monitor</title>
<style>
  :root{
    --bg:#0d1117;--panel:#161b22;--border:#30363d;
    --green:#39d353;--yellow:#e3b341;--red:#f85149;
    --blue:#58a6ff;--muted:#8b949e;--text:#e6edf3;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Trebuchet MS',sans-serif;
    min-height:100vh;padding:1.5rem}
  header{display:flex;justify-content:space-between;align-items:center;
    border-bottom:1px solid var(--border);padding-bottom:1rem;margin-bottom:1.5rem}
  h1{font-size:1.3rem;font-weight:600;letter-spacing:.05em}
  h1 span{color:var(--green);font-family:'Courier New',monospace}
  #status{font-size:.75rem;color:var(--muted);font-family:'Courier New',monospace}
  #status.ok{color:var(--green)}
  #status.err{color:var(--red)}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:1rem}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:8px;
    padding:1.2rem;position:relative;overflow:hidden;transition:border-color .2s}
  .card:hover{border-color:var(--blue)}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;
    background:var(--accent,var(--blue))}
  .label{font-size:.7rem;letter-spacing:.1em;text-transform:uppercase;
    color:var(--muted);margin-bottom:.5rem}
  .value{font-family:'Courier New',monospace;font-size:2rem;
    color:var(--text);line-height:1}
  .unit{font-size:.85rem;color:var(--muted);margin-left:.3em}
  .sub{font-size:.75rem;color:var(--muted);margin-top:.4rem;
    font-family:'Courier New',monospace}
  .warn{color:var(--yellow)}
  .bad{color:var(--red)}
  .good{color:var(--green)}
  nav{margin-top:2rem;display:flex;gap:.75rem;flex-wrap:wrap}
  nav a{padding:.5rem 1rem;border:1px solid var(--border);border-radius:6px;
    color:var(--blue);text-decoration:none;font-size:.85rem;
    transition:all .15s}
  nav a:hover{background:var(--blue);color:var(--bg)}
  footer{margin-top:2rem;font-size:.7rem;color:var(--muted);
    font-family:'Courier New',monospace;border-top:1px solid var(--border);
    padding-top:1rem}
</style>
</head><body>
<header>
  <h1>SEN66 <span>MONITOR</span></h1>
  <span id="status">connecting...</span>
</header>

<div class="grid" id="grid">
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
    <div class="sub" id="co2note"></div>
  </div>
  <div class="card" style="--accent:#7b1fa2">
    <div class="label">VOC Index</div>
    <div class="value"><span id="voc">—</span></div>
    <div class="sub" id="vocnote"></div>
  </div>
  <div class="card" style="--accent:#c62828">
    <div class="label">NOx Index</div>
    <div class="value"><span id="nox">—</span></div>
  </div>
  <div class="card" style="--accent:#0288d1">
    <div class="label">PM 2.5</div>
    <div class="value"><span id="pm25">—</span><span class="unit">µg/m³</span></div>
    <div class="sub" id="pmnote"></div>
  </div>
  <div class="card" style="--accent:#4fc3f7">
    <div class="label">PM 1.0 / 4.0 / 10</div>
    <div class="value" style="font-size:1.1rem;padding-top:.5rem">
      <span id="pm1">—</span> /
      <span id="pm4">—</span> /
      <span id="pm10">—</span>
    </div>
    <div class="sub">µg/m³</div>
  </div>
  <div class="card" style="--accent:#444466">
    <div class="label">Uptime / Log rows</div>
    <div class="value" style="font-size:1.1rem;padding-top:.5rem">
      <span id="uptime">—</span>
    </div>
    <div class="sub"><span id="logrows">—</span> rows logged</div>
  </div>
</div>

<nav>
  <a href="/log">⬇ Download Log</a>
  <a href="/clearlog" onclick="return confirm('Clear log?')">🗑 Clear Log</a>
  <a href="/config">⚙ Config</a>
</nav>

<footer id="footer">last update: —</footer>

<script>
function classify(id,v,lo,hi,warn,bad){
  const el=document.getElementById(id);
  el.className='';
  if(v===null)return;
  if(v>=bad||v<0)el.className='bad';
  else if(v>=warn)el.className='warn';
  else el.className='good';
}
async function refresh(){
  try{
    const r=await fetch('/data');
    if(!r.ok)throw new Error(r.status);
    const d=await r.json();
    const st=document.getElementById('status');
    st.textContent='● LIVE';st.className='ok';

    document.getElementById('temp').textContent  = d.temp_C.toFixed(2);
    document.getElementById('rh').textContent    = d.rh.toFixed(1);
    document.getElementById('co2').textContent   = d.co2_ppm;
    document.getElementById('voc').textContent   = d.voc_idx.toFixed(0);
    document.getElementById('nox').textContent   = d.nox_idx.toFixed(0);
    document.getElementById('uptime').textContent= fmtUptime(d.uptime_s);
    document.getElementById('logrows').textContent= d.log_rows;
    document.getElementById('footer').textContent =
      'last update: '+new Date().toLocaleTimeString()+
      '  ·  interval: '+d.interval_s+'s';

    // PM
    if(d.pm_saturated){
      ['pm1','pm25','pm4','pm10'].forEach(id=>document.getElementById(id).textContent='SAT');
      document.getElementById('pmnote').textContent='⚠ sensor saturated';
      document.getElementById('pmnote').className='sub warn';
    } else {
      document.getElementById('pm1').textContent  = d.pm1.toFixed(1);
      document.getElementById('pm25').textContent = d.pm25.toFixed(1);
      document.getElementById('pm4').textContent  = d.pm4.toFixed(1);
      document.getElementById('pm10').textContent = d.pm10.toFixed(1);
      document.getElementById('pmnote').textContent='';
    }

    // CO2 annotation
    const co2note=document.getElementById('co2note');
    if(d.co2_ppm<400)       {co2note.textContent='warming up';co2note.className='sub warn';}
    else if(d.co2_ppm<1000) {co2note.textContent='good';co2note.className='sub good';}
    else if(d.co2_ppm<2000) {co2note.textContent='elevated';co2note.className='sub warn';}
    else                    {co2note.textContent='poor';co2note.className='sub bad';}

    // VOC annotation
    const vocnote=document.getElementById('vocnote');
    if(d.voc_idx<100)       {vocnote.textContent='good';vocnote.className='sub good';}
    else if(d.voc_idx<200)  {vocnote.textContent='moderate';vocnote.className='sub warn';}
    else                    {vocnote.textContent='poor';vocnote.className='sub bad';}

  }catch(e){
    const st=document.getElementById('status');
    st.textContent='● ERROR: '+e.message;st.className='err';
  }
}
function fmtUptime(s){
  const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),ss=s%60;
  return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(ss).padStart(2,'0');
}
refresh();
setInterval(refresh, 5000);
</script>
</body></html>
)rawhtml";

// ── Config page ───────────────────────────────────────────────────────────
static const char CONFIG_HTML[] = R"rawhtml(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SEN66 Config</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--border:#30363d;--blue:#58a6ff;
    --green:#39d353;--muted:#8b949e;--text:#e6edf3}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Trebuchet MS',sans-serif;
    padding:1.5rem;max-width:480px}
  h1{font-size:1.2rem;margin-bottom:1.5rem;border-bottom:1px solid var(--border);
    padding-bottom:.75rem}
  h1 a{color:var(--blue);text-decoration:none;font-size:.8rem;float:right;
    font-weight:300;line-height:1.8}
  label{display:block;font-size:.7rem;letter-spacing:.08em;text-transform:uppercase;
    color:var(--muted);margin-bottom:.3rem;margin-top:1rem}
  input{width:100%;background:var(--panel);border:1px solid var(--border);
    border-radius:6px;padding:.6rem .8rem;color:var(--text);font-family:'Courier New',monospace;
    font-size:.9rem;outline:none}
  input:focus{border-color:var(--blue)}
  button{margin-top:1.5rem;width:100%;padding:.75rem;background:var(--blue);
    color:#0d1117;border:none;border-radius:6px;font-family:'Trebuchet MS',sans-serif;
    font-size:1rem;font-weight:600;cursor:pointer}
  button:hover{opacity:.85}
  #msg{margin-top:1rem;padding:.6rem;border-radius:6px;font-size:.85rem;
    display:none;text-align:center}
  #msg.ok{background:#1a3a1a;color:var(--green);display:block}
  #msg.err{background:#3a1a1a;color:#f85149;display:block}
  .hint{font-size:.7rem;color:var(--muted);margin-top:.25rem}
</style>
</head><body>
<h1>Configuration <a href="/">← Dashboard</a></h1>
<form id="cfg">
  <label>WiFi SSID</label>
  <input name="wifi_ssid" id="wifi_ssid" placeholder="leave blank for AP mode">
  <label>WiFi Password</label>
  <input name="wifi_pass" id="wifi_pass" type="password" placeholder="••••••••">
  <label>AP SSID (fallback hotspot name)</label>
  <input name="ap_ssid" id="ap_ssid">
  <label>AP Password</label>
  <input name="ap_pass" id="ap_pass" type="password">
  <label>Logging Interval (seconds)</label>
  <input name="interval_s" id="interval_s" type="number" min="1" max="3600">
  <p class="hint">How often a reading is written to the CSV log.</p>
  <label>Max Log Rows (ring buffer)</label>
  <input name="log_max" id="log_max" type="number" min="10" max="5000">
  <p class="hint">Oldest rows are removed when this limit is reached.</p>
  <button type="submit">Save &amp; Reboot</button>
</form>
<div id="msg"></div>
<script>
// Pre-fill from /data endpoint
fetch('/configjson').then(r=>r.json()).then(d=>{
  document.getElementById('wifi_ssid').value  = d.wifi_ssid||'';
  document.getElementById('wifi_pass').value  = d.wifi_pass||'';
  document.getElementById('ap_ssid').value    = d.ap_ssid||'';
  document.getElementById('ap_pass').value    = d.ap_pass||'';
  document.getElementById('interval_s').value = d.interval_s||10;
  document.getElementById('log_max').value    = d.log_max||1000;
});
document.getElementById('cfg').addEventListener('submit',async e=>{
  e.preventDefault();
  const fd=new FormData(e.target);
  const body=new URLSearchParams(fd).toString();
  const r=await fetch('/config',{method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},body});
  const msg=document.getElementById('msg');
  if(r.ok){msg.textContent='Saved! Rebooting in 2 s…';msg.className='ok';}
  else{msg.textContent='Save failed';msg.className='err';}
});
</script>
</body></html>
)rawhtml";

// ═══════════════════════════════════════════════════════════════════════════
// WEB HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot() {
    server.sendHeader("Connection", "close");
    server.sendHeader("Cache-Control", "no-cache");
    server.setContentLength(strlen(DASHBOARD_HTML));
    server.send(200, "text/html; charset=utf-8", "");
    server.sendContent(DASHBOARD_HTML);
    server.client().stop();
}

void handleData() {
    JsonDocument doc;
    doc["ts_ms"]      = latest.ts_ms;
    doc["temp_C"]     = latest.temp;
    doc["rh"]         = latest.rh;
    doc["co2_ppm"]    = latest.co2;
    doc["voc_idx"]    = latest.voc;
    doc["nox_idx"]    = latest.nox;
    doc["pm1"]        = latest.pm1;
    doc["pm25"]       = latest.pm25;
    doc["pm4"]        = latest.pm4;
    doc["pm10"]       = latest.pm10;
    doc["pm_saturated"] = latest.pm_sat;
    doc["valid"]      = latest.valid;
    doc["uptime_s"]   = millis() / 1000;
    doc["log_rows"]   = logRowCount;
    doc["interval_s"] = cfg.interval_s;
    doc["wifi_mode"]  = staMode ? "STA" : "AP";
    doc["ip"]         = staMode
                        ? WiFi.localIP().toString()
                        : WiFi.softAPIP().toString();

    String out;
    serializeJson(doc, out);
    server.sendHeader("Connection", "close");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", out);
    server.client().stop();
}

void handleLog() {
    if (!LittleFS.exists("/log.csv")) {
        server.sendHeader("Connection", "close");
        server.send(404, "text/plain", "No log yet");
        return;
    }
    File f = LittleFS.open("/log.csv", "r");
    server.sendHeader("Connection", "close");
    server.sendHeader("Content-Disposition", "attachment; filename=log.csv");
    server.streamFile(f, "text/csv");
    f.close();
    server.client().stop();
}

void handleClearLog() {
    LittleFS.remove("/log.csv");
    logRowCount = 0;
    initLog();
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleConfigPage() {
    server.sendHeader("Connection", "close");
    server.sendHeader("Cache-Control", "no-cache");
    server.setContentLength(strlen(CONFIG_HTML));
    server.send(200, "text/html; charset=utf-8", "");
    server.sendContent(CONFIG_HTML);
    server.client().stop();
}

void handleConfigJson() {
    JsonDocument doc;
    doc["wifi_ssid"]  = cfg.wifi_ssid;
    doc["wifi_pass"]  = cfg.wifi_pass;
    doc["ap_ssid"]    = cfg.ap_ssid;
    doc["ap_pass"]    = cfg.ap_pass;
    doc["interval_s"] = cfg.interval_s;
    doc["log_max"]    = cfg.log_max;
    String out;
    serializeJson(doc, out);
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", out);
    server.client().stop();
}

void handleConfigPost() {
    if (server.hasArg("wifi_ssid")) strlcpy(cfg.wifi_ssid, server.arg("wifi_ssid").c_str(), sizeof(cfg.wifi_ssid));
    if (server.hasArg("wifi_pass")) strlcpy(cfg.wifi_pass, server.arg("wifi_pass").c_str(), sizeof(cfg.wifi_pass));
    if (server.hasArg("ap_ssid"))   strlcpy(cfg.ap_ssid,   server.arg("ap_ssid").c_str(),   sizeof(cfg.ap_ssid));
    if (server.hasArg("ap_pass"))   strlcpy(cfg.ap_pass,   server.arg("ap_pass").c_str(),   sizeof(cfg.ap_pass));
    if (server.hasArg("interval_s"))cfg.interval_s = server.arg("interval_s").toInt();
    if (server.hasArg("log_max"))   cfg.log_max    = server.arg("log_max").toInt();

    if (saveConfig()) {
        server.send(200, "text/plain", "OK");
        delay(500);
        rp2040.reboot();
    } else {
        server.send(500, "text/plain", "Write failed");
    }
}

void setupWebServer() {
    server.on("/",          HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /");
        handleRoot();
    });
    server.on("/data",      HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /data");
        handleData();
    });
    server.on("/log",       HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /log");
        handleLog();
    });
    server.on("/clearlog",  HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /clearlog");
        handleClearLog();
    });
    server.on("/config",    HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /config");
        handleConfigPage();
    });
    server.on("/config",    HTTP_POST, [](){
        if(DBG_WEB) Serial.println("[web] POST /config");
        handleConfigPost();
    });
    server.on("/configjson",HTTP_GET,  [](){
        if(DBG_WEB) Serial.println("[web] GET /configjson");
        handleConfigJson();
    });
    server.onNotFound([](){ 
        if(DBG_WEB) Serial.printf("[web] 404: %s\n", server.uri().c_str());
        server.send(404,"text/plain","Not found");
    });
    server.begin();
    Serial.println("[web] Server started");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR
// ═══════════════════════════════════════════════════════════════════════════

void printError(const char* label, int16_t err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("[ERR] %s: %s\n", label, errMsg);
}

bool readSensor(Measurement& m) {
    bool dataReady = false;
    uint8_t padding = 0;
    int16_t err = sen66.getDataReady(padding, dataReady);
    if (err != NO_ERROR) { printError("getDataReady", err); return false; }
    if (!dataReady) return false;

    uint16_t pm1Raw, pm25Raw, pm4Raw, pm10Raw;
    int16_t  rhRaw, tempRaw, vocRaw, noxRaw;
    uint16_t co2Raw;

    err = sen66.readMeasuredValuesAsIntegers(
        pm1Raw, pm25Raw, pm4Raw, pm10Raw,
        rhRaw, tempRaw, vocRaw, noxRaw, co2Raw);
    if (err != NO_ERROR) { printError("readMeasured", err); return false; }

    m.ts_ms = millis();
    m.rh    = rhRaw   / 100.0f;
    m.temp  = tempRaw / 200.0f;
    m.voc   = (vocRaw  == INT16_INVALID)  ? 0.0f : vocRaw  / 10.0f;
    m.nox   = (noxRaw  == INT16_INVALID)  ? 0.0f : noxRaw  / 10.0f;
    m.co2   = (co2Raw  == UINT16_INVALID) ? 0     : co2Raw;

    bool pmInvalid = (pm1Raw == UINT16_INVALID);
    float pm1  = pmInvalid ? 0.0f : pm1Raw  / 10.0f;
    float pm25 = pmInvalid ? 0.0f : pm25Raw / 10.0f;
    float pm4  = pmInvalid ? 0.0f : pm4Raw  / 10.0f;
    float pm10 = pmInvalid ? 0.0f : pm10Raw / 10.0f;
    m.pm_sat = (!pmInvalid) && (fabsf(pm25 - PM_SATURATED) < 1.0f);
    m.pm1  = pm1;  m.pm25 = pm25;
    m.pm4  = pm4;  m.pm10 = pm10;
    m.valid = true;
    return true;
}

// Print a human-readable one-liner (for [read] command)
void printReadable(const Measurement& m) {
    Serial.printf(
        "[read] T=%.2f°C  RH=%.1f%%  CO2=%uppm  VOC=%.0f  NOx=%.0f"
        "  PM2.5=%s  uptime=%lus\n",
        m.temp, m.rh, m.co2, m.voc, m.nox,
        m.pm_sat ? "SAT" : String(m.pm25, 1).c_str(),
        millis() / 1000
    );
}

// Print CSV line — identical format to Phase 1, compatible with sen66_monitor.py
// ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm
void printMeasurement(const Measurement& m) {
    char s1[10], s2[10], s3[10], s4[10];
    if (m.pm_sat) {
        strcpy(s1,"null"); strcpy(s2,"null");
        strcpy(s3,"null"); strcpy(s4,"null");
    } else {
        dtostrf(m.pm1,  1, 1, s1); dtostrf(m.pm25, 1, 1, s2);
        dtostrf(m.pm4,  1, 1, s3); dtostrf(m.pm10, 1, 1, s4);
    }
    Serial.printf("%lu,%s,%s,%s,%s,%.1f,%.2f,%.1f,%.1f,%u\n",
        m.ts_ms, s1, s2, s3, s4,
        m.rh, m.temp, m.voc, m.nox, m.co2);
}

// ═══════════════════════════════════════════════════════════════════════════
// SERIAL CLI
// ═══════════════════════════════════════════════════════════════════════════

void printHelp() {
    Serial.println("─────────────────────────────────────────");
    Serial.println(" SEN66 Serial CLI");
    Serial.println("  help         — this message");
    Serial.println("  status       — WiFi, IP, uptime, flash");
    Serial.println("  read         — single human-readable reading");
    Serial.println("  stream       — toggle CSV stream (Python compat)");
    Serial.println("  debug <n>    — set debug flags (OR flags below):");
    Serial.println("    0 = off    1 = web    2 = sensor    4 = mqtt");
    Serial.println("    e.g. 'debug 3' = web + sensor");
    Serial.println("─────────────────────────────────────────");
}

void printStatus() {
    FSInfo fs;
    LittleFS.info(fs);
    Serial.println("─────────────────────────────────────");
    Serial.printf(" WiFi:    %s\n", staMode ? "STA" : "AP");
    Serial.printf(" IP:      %s\n", staMode
        ? WiFi.localIP().toString().c_str()
        : "192.168.4.1");
    Serial.printf(" SSID:    %s\n", staMode ? cfg.wifi_ssid : cfg.ap_ssid);
    Serial.printf(" Uptime:  %lus\n", millis() / 1000);
    Serial.printf(" Flash:   %u / %u bytes used\n",
        fs.usedBytes, fs.totalBytes);
    Serial.printf(" Log:     %lu rows\n", logRowCount);
    Serial.printf(" Stream:  %s\n", streamEnabled ? "ON" : "OFF");
    Serial.printf(" Debug:   %u  (web=%d sensor=%d mqtt=%d)\n",
        debugLevel, (int)DBG_WEB, (int)DBG_SENSOR, (int)DBG_MQTT);
    Serial.println("─────────────────────────────────────");
}

void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();

    if      (cmd == "help")   printHelp();
    else if (cmd == "status") printStatus();
    else if (cmd == "read")   { if (latest.valid) printReadable(latest);
                                else Serial.println("[cli] No reading yet"); }
    else if (cmd == "stream") { streamEnabled = !streamEnabled;
                                Serial.printf("[cli] Stream %s — %s\n",
                                streamEnabled ? "ON" : "OFF",
                                streamEnabled
                                  ? "CSV format, compatible with sen66_monitor.py"
                                  : "stopped");
                                // Print header when enabling so Python monitor
                                // can sync immediately
                                if (streamEnabled)
                                    Serial.println("ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm");
                              }
    else if (cmd.startsWith("debug")) {
        String arg = cmd.substring(5); arg.trim();
        if (arg.length() > 0) {
            debugLevel = (uint8_t)arg.toInt();
            Serial.printf("[cli] Debug level set to %u  (web=%d sensor=%d mqtt=%d)\n",
                debugLevel, (int)DBG_WEB, (int)DBG_SENSOR, (int)DBG_MQTT);
        } else {
            Serial.printf("[cli] Debug level: %u  (web=%d sensor=%d mqtt=%d)\n",
                debugLevel, (int)DBG_WEB, (int)DBG_SENSOR, (int)DBG_MQTT);
        }
    }
    else if (cmd.length() > 0)
        Serial.printf("[cli] Unknown: '%s'  (try 'help')\n", cmd.c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP / LOOP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n═══ SEN66 Monitor Phase 2 ═══");

    // LittleFS
    if (!LittleFS.begin()) {
        Serial.println("[fs] Mount failed — formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    Serial.println("[fs] LittleFS mounted");
    loadConfig();
    initLog();

    // WiFi
    startWiFi();

    // Web server
    setupWebServer();

    // I2C + SEN66
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    sen66.begin(Wire, SEN66_I2C_ADDR_6B);

    int16_t err = sen66.deviceReset();
    if (err != NO_ERROR) printError("deviceReset", err);
    delay(1200);

    err = sen66.startContinuousMeasurement();
    if (err != NO_ERROR) printError("startMeasurement", err);

    err = sen66.startFanCleaning();
    if (err != NO_ERROR) printError("fanCleaning", err);
    Serial.println("[sen66] Fan cleaning (10s)...");
    delay(11000);
    Serial.println("[sen66] Ready");

    printHelp();
    Serial.println("[boot] Complete — type 'status' for network info");
}

void loop() {
    server.handleClient();
    handleSerial();

    // Read sensor every second regardless; log at configured interval
    static uint32_t lastReadMs = 0;
    if (millis() - lastReadMs >= 1000) {
        lastReadMs = millis();
        readSensor(latest);

        if (DBG_SENSOR && latest.valid) {
            Serial.printf("[sensor] T=%.2f RH=%.1f CO2=%u VOC=%.0f NOx=%.0f PM2.5=%s\n",
                latest.temp, latest.rh, latest.co2, latest.voc, latest.nox,
                latest.pm_sat ? "SAT" : String(latest.pm25,1).c_str());
        }

        if (streamEnabled && latest.valid) {
            printMeasurement(latest);
        }
    }

    // Log at configured interval
    if (latest.valid && (millis() - lastLogTime >= (uint32_t)cfg.interval_s * 1000)) {
        lastLogTime = millis();
        appendLog(latest);
    }
}