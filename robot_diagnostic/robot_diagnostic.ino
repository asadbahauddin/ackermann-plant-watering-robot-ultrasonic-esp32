/*
 * ============================================================================
 *  robot_diagnostic.ino  -  Firmware Diagnostic Robot Penyiram Tanaman
 * ============================================================================
 *
 *  Board   : ESP32 (Arduino-ESP32 core 2.x / 3.x)
 *  Mode    : Wi-Fi Access Point
 *              SSID     : "RobotDiag"
 *              Password : "robot1234"
 *              IP       : 192.168.4.1
 *  Server  : ESPAsyncWebServer (ESP32Async fork) @ port 80
 *  Socket  : AsyncWebSocket  path "/ws"
 *  JSON    : ArduinoJson v7 (kompatibel v6)
 *
 *  Library yang harus diinstal:
 *    - ESPAsyncWebServer  (fork: https://github.com/ESP32Async/ESPAsyncWebServer)
 *    - AsyncTCP           (fork: https://github.com/ESP32Async/AsyncTCP)
 *    - ArduinoJson        v7.x
 *
 * ----------------------------------------------------------------------------
 *  PIN MAPPING
 * ----------------------------------------------------------------------------
 *    GPIO13  -> TRIG  (dibagi ke SEMUA sensor lewat R 330 ohm)
 *    GPIO14  -> ECHO  JSN depan            (jsn_f)
 *    GPIO27  -> ECHO  HC samping kanan     (hc_sr)
 *    GPIO26  -> ECHO  HC kanan             (hc_r)
 *    GPIO25  -> ECHO  HC samping kiri      (hc_sl)
 *    GPIO33  -> ECHO  HC kiri              (hc_l)
 *    GPIO32  -> ECHO  JSN water level      (jsn_wl)
 *
 *    GPIO16  -> BTS7960 RPWM  (LEDC PWM)
 *    GPIO17  -> BTS7960 LPWM  (LEDC PWM)
 *    GPIO18  -> BTS7960 R_EN
 *    GPIO21  -> BTS7960 L_EN
 *
 *    GPIO23  -> DM556 STEP  (lewat BC547 - INVERTING)
 *    GPIO19  -> DM556 DIR   (lewat BC547 - INVERTING)
 *    GPIO22  -> DM556 EN    (lewat BC547 - INVERTING)
 *
 * ----------------------------------------------------------------------------
 *  CATATAN INVERTING BC547 (common cathode)
 * ----------------------------------------------------------------------------
 *    GPIO HIGH  -> BC547 ON  -> input driver ditarik ke LOW
 *    GPIO LOW   -> BC547 OFF -> input driver ditarik ke HIGH (pull-up 5V)
 *
 *    Jadi level LOGIS yang diterima driver = kebalikan level GPIO.
 *    Semua akses ke DM556 lewat helper drvWrite() / drvStepPulse().
 *
 * ----------------------------------------------------------------------------
 *  ARSITEKTUR TASK
 * ----------------------------------------------------------------------------
 *    - Pembacaan sensor jalan di FreeRTOS task terpisah (sensorTaskFn),
 *      di-pin ke core SENSOR_TASK_CORE. Sekuensial (1 sensor / giliran)
 *      supaya tidak ada crosstalk ultrasonik.
 *    - g_dist[] dilindungi mutex g_distMutex (ditulis task, dibaca broadcast).
 *    - loop() hanya urus stepper + broadcast + housekeeping WebSocket.
 *    - Timing loop pakai millis()/micros(); delayMicroseconds hanya untuk
 *      pulsa TRIG & pulsa STEP. Task sensor pakai vTaskDelay untuk jeda.
 * ============================================================================
 */

// PENTING: ArduinoJson HARUS di-include SEBELUM ESPAsyncWebServer.
// ESPAsyncWebServer memakai `#if __has_include(<ArduinoJson.h>)`, dan bila
// urutannya terbalik, library-detector arduino-cli gagal menambahkan
// ArduinoJson ke include-path (error "JsonDocument was not declared").
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---------------------------------------------------------------------------
//  LAPISAN KOMPATIBILITAS
//  Target utama: ArduinoJson v7 + Arduino-ESP32 core 3.x.
//  Shim di bawah membuat firmware tetap ter-compile pada:
//     - ArduinoJson v6          -> DynamicJsonDocument
//     - Arduino-ESP32 core 2.x  -> ledcSetup + ledcAttachPin + channel
// ---------------------------------------------------------------------------
#if ARDUINOJSON_VERSION_MAJOR < 7
  #define JSON_DOC(name) DynamicJsonDocument name(512)
#else
  #define JSON_DOC(name) JsonDocument name
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ===========================================================================
//  KONFIGURASI ACCESS POINT
// ===========================================================================
static const char *AP_SSID = "RobotDiag";
static const char *AP_PASS = "robot1234";
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GW(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

// ===========================================================================
//  PIN
// ===========================================================================
#define PIN_TRIG        13

#define PIN_ECHO_JSN_F  14   // JSN depan
#define PIN_ECHO_HC_SR  27   // HC samping kanan
#define PIN_ECHO_HC_R   26   // HC kanan
#define PIN_ECHO_HC_SL  25   // HC samping kiri
#define PIN_ECHO_HC_L   33   // HC kiri
#define PIN_ECHO_JSN_WL 32   // JSN water level

#define PIN_RPWM        16
#define PIN_LPWM        17
#define PIN_R_EN        18
#define PIN_L_EN        21

#define PIN_STEP        23   // INVERTING
#define PIN_DIR         19   // INVERTING
#define PIN_EN          22   // INVERTING

#define PIN_RELAY       15   // relay pompa air (active LOW)

// ===========================================================================
//  SENSOR - definisi
// ===========================================================================
#define N_SENSOR 6

// urutan pembacaan = urutan array ini
static const uint8_t ECHO_PIN[N_SENSOR] = {
  PIN_ECHO_JSN_F,   // 0
  PIN_ECHO_HC_SR,   // 1
  PIN_ECHO_HC_R,    // 2
  PIN_ECHO_HC_SL,   // 3
  PIN_ECHO_HC_L,    // 4
  PIN_ECHO_JSN_WL   // 5
};

static const char *SENSOR_KEY[N_SENSOR] = {
  "jsn_wl", "hc_sl", "hc_r", "hc_l", "hc_sr", "jsn_f"
};

// hasil terakhir tiap sensor (cm). -1 = timeout / invalid
static float g_dist[N_SENSOR] = { -1, -1, -1, -1, -1, -1 };

// --- sinkronisasi task sensor -------------------------------------------
static SemaphoreHandle_t g_distMutex  = NULL;
static TaskHandle_t      g_sensorTask = NULL;

// --- opsi debug --------------------------------------------------------
// TRIG_SELFTEST 1 -> GPIO13 dijadikan kotak ~2 Hz supaya BISA diukur
//                    multimeter (cek pin trigger hidup/mati). Sensor tidak
//                    dibaca di mode ini.
#define TRIG_SELFTEST   0
// SENSOR_DEBUG 1  -> cetak jarak tiap sensor ke Serial Monitor (115200)
#define SENSOR_DEBUG    1

// Core untuk task sensor.
//   CATATAN: pada Arduino-ESP32 stock, task WiFi/lwIP berjalan di core 0
//   (PRO_CPU) dan loop() Arduino di core 1 (APP_CPU). Kalau pembacaan
//   sensor sering "melar"/timeout karena diserobot WiFi, ganti ke 1.
#define SENSOR_TASK_CORE     0

#define SEQ_ECHO_TIMEOUT_US  25000UL     // batas tunggu echo (~4.3 m)
#define SEQ_INTER_MS         35UL        // jeda antar sensor

#define SOUND_CM_PER_US      0.01715f    // (343 m/s)/2 dalam cm/us
#define DIST_MAX_CM          500.0f

// ===========================================================================
//  MOTOR PENGGERAK (BTS7960)
// ===========================================================================
#define MOTOR_PWM_FREQ  20000
#define MOTOR_PWM_RES   8               // 0..255
#define MOTOR_CH_RPWM   0               // channel LEDC (core 2.x)
#define MOTOR_CH_LPWM   1

// --- helper LEDC lintas-versi core ---
static inline void pwmSetup(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
#else
  ledcSetup(ch, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(pin, ch);
#endif
}
static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

static int  g_motorSpeed = 0;          // -255..255  (neg = mundur)
static bool g_motorEn    = false;

// ===========================================================================
//  STEERING (DM556 stepper via BC547 inverting)
// ===========================================================================
static int  g_maxAngle    = 45;        // sudut maksimum (deg) - hasil kalibrasi
static int  g_stepsPerDeg = 10;        // steps per derajat  - hasil kalibrasi
static bool g_steerEn     = false;
static bool g_pumpOn      = false;     // status relay pompa


static long g_curSteps    = 0;         // posisi stepper sekarang (steps)
static long g_tgtSteps    = 0;         // target posisi (steps)
static int  g_tgtAngle    = 0;         // target sudut (deg)

static uint32_t g_lastStepUs = 0;
#define STEP_INTERVAL_US  2000         // jarak antar step -> kecepatan putar (2000 us = 500 step/s, aman untuk start tanpa ramp)

// ---- helper level INVERTING untuk DM556 ----
// logic = level yang DIINGINKAN di kaki driver.
static inline void drvWrite(uint8_t gpio, uint8_t logic) {
  digitalWrite(gpio, logic ? LOW : HIGH);   // dibalik oleh BC547
}

// satu pulsa STEP (driver: LOW -> HIGH -> LOW). delayMicroseconds diizinkan di sini.
static inline void drvStepPulse() {
  drvWrite(PIN_STEP, HIGH);
  delayMicroseconds(30);   // lewat BC547 + opto DM556 butuh pulsa lebih lebar dari 6 us
  drvWrite(PIN_STEP, LOW);
  delayMicroseconds(30);
}

// ===========================================================================
//  WEB SERVER / WEBSOCKET
// ===========================================================================
AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

static uint32_t g_lastBroadcast = 0;
static uint32_t g_lastWsClean   = 0;
static bool     g_forceSend     = false;
#define BROADCAST_MS  100

// ===========================================================================
//  UI HTML (PROGMEM)
// ===========================================================================
static const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="id"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Robot Diagnostic</title>
<style>
:root{
  --bg:#0e1116; --panel:#161b22; --panel2:#1c2330; --line:#2a3240;
  --tx:#e6edf3; --tx2:#9aa7b4; --acc:#3b82f6; --ok:#22c55e; --warn:#f59e0b;
  --danger:#ef4444;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--tx);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif;padding:12px;max-width:920px;margin:0 auto}
h1{font-size:16px}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--tx2);margin-bottom:10px}
header{display:flex;align-items:center;justify-content:space-between;gap:10px;
  background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px 14px;margin-bottom:12px}
.badge{display:flex;align-items:center;gap:7px;font-size:12px;color:var(--tx2)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--danger);box-shadow:0 0 0 3px rgba(239,68,68,.15)}
.dot.on{background:var(--ok);box-shadow:0 0 0 3px rgba(34,197,94,.15)}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.sensors{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px}
.sensor{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.sensor .lbl{font-size:11px;color:var(--tx2);text-transform:uppercase;letter-spacing:.04em}
.sensor .val{font-size:22px;font-weight:600;margin-top:4px}
.sensor .val small{font-size:12px;color:var(--tx2);font-weight:400}
.sensor.warn{border-color:var(--warn);background:rgba(245,158,11,.10)}
.sensor.warn .val{color:var(--warn)}
.sensor.dead .val{color:var(--tx2)}
.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.row+.row{margin-top:12px}
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:6px;border-radius:6px;background:var(--line);outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:20px;height:20px;border-radius:50%;background:var(--acc);cursor:pointer;border:2px solid #0e1116}
input[type=range]::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:var(--acc);cursor:pointer;border:2px solid #0e1116}
.readout{min-width:74px;text-align:right;font-variant-numeric:tabular-nums;font-weight:600}
button{background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:9px;padding:8px 12px;font-size:13px;cursor:pointer}
button:hover{border-color:var(--acc)}
button:active{transform:translateY(1px)}
button.primary{background:var(--acc);border-color:var(--acc);color:#fff}
button.stop{background:var(--danger);border-color:var(--danger);color:#fff}
.btns{display:flex;gap:8px;flex-wrap:wrap}
label.chk{display:flex;align-items:center;gap:7px;cursor:pointer;user-select:none;font-size:13px}
input[type=number]{background:var(--panel2);color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 9px;width:90px;font-size:13px}
.telemetry{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;font-size:12px}
.telemetry div{background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:7px 9px}
.telemetry b{display:block;font-size:15px;font-weight:600;margin-top:2px}
.proc{font-size:12.5px;color:var(--tx2);background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.proc ol{margin-left:16px}
.proc li{margin:3px 0}
.muted{color:var(--tx2);font-size:12px}
.val-display{font-size:22px;font-weight:700;text-align:center;padding:14px 0}
.btn-row{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
.btn-success{background:var(--ok);border-color:var(--ok);color:#fff}
.btn-danger{background:var(--danger);border-color:var(--danger);color:#fff}
</style></head><body>

<header>
  <h1>🌱 Robot Diagnostic</h1>
  <div class="badge"><span id="dot" class="dot"></span><span id="wsTxt">Terputus</span></div>
</header>


<div class="card">
  <h2>Sensor Jarak (realtime)</h2>
  <div class="sensors" id="sensors"></div>
  <p class="muted" style="margin-top:8px">Kuning = objek &lt; 20 cm &nbsp;·&nbsp; &ldquo;&mdash;&rdquo; = timeout / tidak terbaca</p>
</div>

<div class="card">
  <h2>Motor Penggerak</h2>
  <div class="row">
    <input type="range" id="mSlider" min="-255" max="255" step="1" value="0">
    <span class="readout" id="mVal">0</span>
  </div>
  <div class="row btns">
    <button data-sp="-255" class="stop">Mundur 100%</button>
    <button data-sp="-128">Mundur 50%</button>
    <button id="mStop" class="stop">STOP</button>
    <button data-sp="128">Maju 50%</button>
    <button data-sp="255" class="primary">Maju 100%</button>
  </div>
  <div class="row">
    <label class="chk"><input type="checkbox" id="mEn"> Motor Enable (R_EN / L_EN)</label>
  </div>
</div>

<div class="card">
  <h2>💧 Pompa Air</h2>
  <div class="val-display" id="pumpDisplay">MATI</div>
  <div class="btn-row">
    <button class="btn-success" id="pumpOn">💧 ON</button>
    <button class="btn-danger" id="pumpOff">⛔ OFF</button>
  </div>
</div>

<div class="card">
  <h2>Steering</h2>
  <div class="row">
    <input type="range" id="sSlider" min="-45" max="45" step="1" value="0">
    <span class="readout" id="sVal">0&deg;</span>
  </div>
  <div class="row btns">
    <button id="sCenter">Center</button>
    <span class="muted">Range: <span id="sRange">-45&deg; &hellip; +45&deg;</span></span>
  </div>
  <div class="row">
    <label class="chk"><input type="checkbox" id="sEn"> Steering Enable (DM556 EN)</label>
  </div>
</div>

<div class="card">
  <h2>Kalibrasi</h2>
  <div class="row btns">
    <input type="number" id="calMax" min="1" max="180" step="1" placeholder="max&deg;">
    <button id="setMax">Set Max Angle</button>
  </div>
  <div class="row btns">
    <input type="number" id="calSpd" min="1" max="1000" step="1" placeholder="steps/&deg;">
    <button id="setSpd">Set Steps / Derajat</button>
  </div>
  <div class="row btns">
    <button id="zero" class="primary">Zero Position (jadikan center)</button>
  </div>
  <div class="row">
    <div class="proc">
      <b>Prosedur kalibrasi:</b>
      <ol>
        <li>Pastikan roda kemudi bebas bergerak. Centang <i>Steering Enable</i>.</li>
        <li>Luruskan roda benar-benar lurus, lalu klik <i>Zero Position</i>.</li>
        <li>Set <i>Steps / Derajat</i>: geser slider ke sudut kecil (mis. 10&deg;),
            ukur sudut nyata, naik/turunkan nilai sampai 1&deg; perintah = 1&deg; nyata.</li>
        <li>Set <i>Max Angle</i>: naikkan perlahan sampai batas mekanis sebelum mentok,
            lalu kurangi 2&ndash;3&deg; sebagai margin aman.</li>
        <li>Uji: geser slider ke kiri &amp; kanan penuh, pastikan tidak mentok / selip.</li>
      </ol>
    </div>
  </div>
</div>

<div class="card">
  <h2>Telemetry</h2>
  <div class="telemetry">
    <div>motor_speed <b id="tSpeed">0</b></div>
    <div>steer_angle <b id="tAngle">0&deg;</b></div>
    <div>max_angle <b id="tMax">45</b></div>
    <div>steps_per_deg <b id="tSpd">10</b></div>
    <div>motor_en <b id="tMEn">false</b></div>
    <div>steer_en <b id="tSEn">false</b></div>
  </div>
</div>

<script>
const SENS = [
  ["jsn_f","JSN Depan"],
  ["hc_sr","HC Samping Kanan"],
  ["hc_r","HC Kanan"],
  ["hc_sl","HC Samping Kiri"],
  ["hc_l","HC Kiri"],
  ["jsn_wl","JSN Water Level"]
];
const grid = document.getElementById('sensors');
SENS.forEach(([k,label])=>{
  const d = document.createElement('div');
  d.className='sensor'; d.id='s_'+k;
  d.innerHTML = '<div class="lbl">'+label+'</div><div class="val">--<small> cm</small></div>';
  grid.appendChild(d);
});

let ws, curMax = 45;
const $ = id => document.getElementById(id);

function setStatus(ok){
  $('dot').classList.toggle('on', ok);
  $('wsTxt').textContent = ok ? 'Terhubung' : 'Terputus';
}
function send(obj){
  if(ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

function connect(){
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = ()=>{ setStatus(true); send({cmd:'read_sensors'}); };
  ws.onclose = ()=>{ setStatus(false); setTimeout(connect, 1000); };
  ws.onerror = ()=>{ ws.close(); };
  ws.onmessage = ev=>{
    let m; try{ m = JSON.parse(ev.data); }catch(e){ return; }
    if(m.type === 'sensor') applySensor(m);
  };
}

function applySensor(m){
  SENS.forEach(([k])=>{
    const el = $('s_'+k);
    const v  = m[k];
    const val = el.querySelector('.val');
    el.classList.remove('warn','dead');
    if(v === undefined) return;
    if(v < 0){
      val.innerHTML = '&mdash;<small> cm</small>';
      el.classList.add('dead');
    }else{
      val.innerHTML = v.toFixed(1) + '<small> cm</small>';
      if(v < 20) el.classList.add('warn');
    }
  });

  // range steering dinamis
  if(typeof m.max_angle === 'number' && m.max_angle !== curMax){
    curMax = m.max_angle;
    const s = $('sSlider');
    s.min = -curMax; s.max = curMax;
    if(+s.value >  curMax) s.value =  curMax;
    if(+s.value < -curMax) s.value = -curMax;
    $('sRange').innerHTML = '-' + curMax + '&deg; &hellip; +' + curMax + '&deg;';
  }

  $('tSpeed').textContent = m.motor_speed;
  $('tAngle').innerHTML   = m.steer_angle + '&deg;';
  $('tMax').textContent   = m.max_angle;
  $('tSpd').textContent   = m.steps_per_deg;
  $('tMEn').textContent   = m.motor_en;
  $('tSEn').textContent   = m.steer_en;
  $('mEn').checked = !!m.motor_en;
  $('sEn').checked = !!m.steer_en;
  if(!$('calMax').value) $('calMax').placeholder = m.max_angle + '°';
  if(!$('calSpd').value) $('calSpd').placeholder = m.steps_per_deg + ' steps/°';

  if(typeof m.pump !== 'undefined'){
    const pd = $('pumpDisplay');
    pd.textContent = m.pump ? 'HIDUP 💧' : 'MATI';
    pd.style.color = m.pump ? '#22c55e' : '#ef4444';
  }
}

/* ---- Motor slider (debounce 50 ms) ---- */
let mTimer = null;
$('mSlider').addEventListener('input', e=>{
  $('mVal').textContent = e.target.value;
  clearTimeout(mTimer);
  mTimer = setTimeout(()=> send({cmd:'motor_speed', value:+e.target.value}), 50);
});
document.querySelectorAll('button[data-sp]').forEach(b=>{
  b.addEventListener('click', ()=>{
    const v = +b.dataset.sp;
    $('mSlider').value = v; $('mVal').textContent = v;
    send({cmd:'motor_speed', value:v});
  });
});
$('mStop').addEventListener('click', ()=>{
  $('mSlider').value = 0; $('mVal').textContent = '0';
  send({cmd:'motor_stop'});
});
$('mEn').addEventListener('change', e=> send({cmd:'motor_enable', value:e.target.checked}));

/* ---- Pompa air ---- */
$('pumpOn').addEventListener('click', ()=> send({cmd:'pump', value:true}));
$('pumpOff').addEventListener('click', ()=> send({cmd:'pump', value:false}));

/* ---- Steering slider (debounce 50 ms) ---- */
let sTimer = null;
$('sSlider').addEventListener('input', e=>{
  $('sVal').innerHTML = e.target.value + '&deg;';
  clearTimeout(sTimer);
  sTimer = setTimeout(()=> send({cmd:'steer_angle', value:+e.target.value}), 50);
});
$('sCenter').addEventListener('click', ()=>{
  $('sSlider').value = 0; $('sVal').innerHTML = '0&deg;';
  send({cmd:'steer_center'});
});
$('sEn').addEventListener('change', e=> send({cmd:'steer_enable', value:e.target.checked}));

/* ---- Kalibrasi ---- */
$('setMax').addEventListener('click', ()=>{
  const v = parseInt($('calMax').value, 10);
  if(v > 0) send({cmd:'set_max_angle', value:v});
});
$('setSpd').addEventListener('click', ()=>{
  const v = parseInt($('calSpd').value, 10);
  if(v > 0) send({cmd:'set_steps_per_deg', value:v});
});
$('zero').addEventListener('click', ()=>{
  $('sSlider').value = 0; $('sVal').innerHTML = '0&deg;';
  send({cmd:'zero_position'});
});

connect();
</script>
</body></html>
)HTMLPAGE";

// ===========================================================================
//  SENSOR - task FreeRTOS (sekuensial, anti-crosstalk)
// ===========================================================================
#if SENSOR_DEBUG
static void sensorDebugPrint() {
  Serial.print("[SENS] ");
  for (int i = 0; i < N_SENSOR; i++) {
    Serial.print(SENSOR_KEY[i]); Serial.print('=');
    if (g_dist[i] < 0) Serial.print("--   ");
    else { Serial.print(g_dist[i], 1); Serial.print("cm "); }
  }
  Serial.println();
}
#endif

void sensorTaskFn(void *arg) {
  (void)arg;
  for (;;) {
    for (int i = 0; i < N_SENSOR; i++) {
      // --- trigger (semua modul terpicu, hanya ECHO[i] yang dibaca) ---
      digitalWrite(PIN_TRIG, LOW);
      delayMicroseconds(4);
      digitalWrite(PIN_TRIG, HIGH);
      delayMicroseconds(12);
      digitalWrite(PIN_TRIG, LOW);

      // --- baca echo (blocking di task ini saja, bukan di loop()) ---
      unsigned long dur = pulseIn(ECHO_PIN[i], HIGH, SEQ_ECHO_TIMEOUT_US);
      float d = (dur > 0) ? (float)dur * SOUND_CM_PER_US : -1.0f;
      if (d > DIST_MAX_CM || d <= 1.0f) d = -1.0f;

      // --- simpan dengan mutex ---
      xSemaphoreTake(g_distMutex, portMAX_DELAY);
      g_dist[i] = d;
      xSemaphoreGive(g_distMutex);

      // --- jeda antar sensor: beri waktu gema meluruh + yield ---
      vTaskDelay(pdMS_TO_TICKS(SEQ_INTER_MS));
    }
#if SENSOR_DEBUG
    sensorDebugPrint();
#endif
  }
}

// ===========================================================================
//  MOTOR - terapkan state ke hardware
// ===========================================================================
static void motorApply() {
  digitalWrite(PIN_R_EN, g_motorEn ? HIGH : LOW);
  digitalWrite(PIN_L_EN, g_motorEn ? HIGH : LOW);

  if (!g_motorEn || g_motorSpeed == 0) {
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    return;
  }
  if (g_motorSpeed > 0) {            // maju
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, g_motorSpeed);
  } else {                          // mundur
    pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);
    pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, -g_motorSpeed);
  }
}

// ===========================================================================
//  STEERING - hitung ulang target & terapkan EN
// ===========================================================================
static void steerRecalcTarget() {
  g_tgtAngle = constrain(g_tgtAngle, -g_maxAngle, g_maxAngle);
  g_tgtSteps = (long)g_tgtAngle * (long)g_stepsPerDeg;
}

static void steerApplyEnable() {
  // logic HIGH di kaki EN = driver aktif (lihat catatan BC547)
  // uji putar poros: polaritas ini benar (Enable OFF = poros bebas, ON = terkunci)
  drvWrite(PIN_EN, g_steerEn ? HIGH : LOW);
}

static int steerReportAngle() {
  if (g_stepsPerDeg <= 0) return 0;
  return (int)lroundf((float)g_curSteps / (float)g_stepsPerDeg);
}

// jalankan stepper (non-blocking). delayMicroseconds hanya untuk pulsa & settle DIR.
static void steerLoop() {
  if (!g_steerEn) return;
  if (g_curSteps == g_tgtSteps) return;

  uint32_t now = micros();
  if (now - g_lastStepUs < STEP_INTERVAL_US) return;
  g_lastStepUs = now;

  bool dirPos = (g_tgtSteps > g_curSteps);       // true = ke kanan (sudut naik)
  drvWrite(PIN_DIR, dirPos ? HIGH : LOW);        // dibalik: arah CW/CCW sebelumnya terbalik
  delayMicroseconds(50);                        // settle DIR sebelum step (DM556 >= 5 us + tunda BC547/opto)
  drvStepPulse();
  g_curSteps += dirPos ? 1 : -1;
}

// ===========================================================================
//  BROADCAST telemetry
// ===========================================================================
static void broadcastSensors() {
  JSON_DOC(doc);
  doc["type"] = "sensor";

  xSemaphoreTake(g_distMutex, portMAX_DELAY);
  for (int i = 0; i < N_SENSOR; i++) doc[SENSOR_KEY[i]] = g_dist[i];
  xSemaphoreGive(g_distMutex);

  doc["motor_speed"]   = g_motorSpeed;
  doc["steer_angle"]   = steerReportAngle();
  doc["max_angle"]     = g_maxAngle;
  doc["steps_per_deg"] = g_stepsPerDeg;
  doc["motor_en"]      = g_motorEn;
  doc["steer_en"]      = g_steerEn;
  doc["pump"]          = g_pumpOn;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ===========================================================================
//  HANDLE COMMAND dari client
// ===========================================================================
static void handleCommand(const uint8_t *data, size_t len) {
  JSON_DOC(doc);
  if (deserializeJson(doc, data, len)) return;

  const char *cmd = doc["cmd"];
  if (!cmd) return;

  if (!strcmp(cmd, "motor_speed")) {
    g_motorSpeed = constrain((int)(doc["value"] | 0), -255, 255);
    motorApply();

  } else if (!strcmp(cmd, "motor_stop")) {
    g_motorSpeed = 0;
    motorApply();

  } else if (!strcmp(cmd, "motor_enable")) {
    g_motorEn = doc["value"] | false;
    motorApply();

  } else if (!strcmp(cmd, "steer_angle")) {
    g_tgtAngle = (int)(doc["value"] | 0);
    steerRecalcTarget();

  } else if (!strcmp(cmd, "steer_center")) {
    g_tgtAngle = 0;
    steerRecalcTarget();

  } else if (!strcmp(cmd, "steer_enable")) {
    g_steerEn = doc["value"] | false;
    steerApplyEnable();

  } else if (!strcmp(cmd, "set_max_angle")) {
    g_maxAngle = constrain((int)(doc["value"] | g_maxAngle), 1, 180);
    steerRecalcTarget();

  } else if (!strcmp(cmd, "set_steps_per_deg")) {
    g_stepsPerDeg = constrain((int)(doc["value"] | g_stepsPerDeg), 1, 1000);
    steerRecalcTarget();

  } else if (!strcmp(cmd, "zero_position")) {
    g_curSteps = 0;
    g_tgtSteps = 0;
    g_tgtAngle = 0;

  } else if (!strcmp(cmd, "read_sensors")) {
    g_forceSend = true;

  } else if (!strcmp(cmd, "pump")) {
    g_pumpOn = doc["value"] | false;
    digitalWrite(PIN_RELAY, g_pumpOn ? LOW : HIGH);
  }
}

// ===========================================================================
//  WEBSOCKET EVENT
// ===========================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      g_forceSend = true;               // kirim state awal ke client baru
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        handleCommand(data, len);
      }
      break;
    }

    default:
      break;
  }
}

// ===========================================================================
//  SETUP
// ===========================================================================
void setup() {
  Serial.begin(115200);

  // ---- Sensor pin ----
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  for (int i = 0; i < N_SENSOR; i++) {
    pinMode(ECHO_PIN[i], INPUT);
  }

  // ---- Motor BTS7960 ----
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  digitalWrite(PIN_R_EN, LOW);
  digitalWrite(PIN_L_EN, LOW);
  pwmSetup(PIN_RPWM, MOTOR_CH_RPWM);
  pwmSetup(PIN_LPWM, MOTOR_CH_LPWM);
  pwmWrite(PIN_RPWM, MOTOR_CH_RPWM, 0);
  pwmWrite(PIN_LPWM, MOTOR_CH_LPWM, 0);

  // ---- DM556 (INVERTING via BC547) ----
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);
  drvWrite(PIN_STEP, LOW);     // idle
  drvWrite(PIN_DIR,  LOW);

  // ---- Relay pompa ----
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH); // relay OFF saat boot (active LOW)

  steerApplyEnable();          // g_steerEn = false -> driver disabled

  // ---- Access Point ----
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP \"%s\"  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // ---- WebSocket + HTTP ----
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "Not found");
  });

  // ---- Task sensor (core terpisah) ----
  g_distMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    sensorTaskFn,       // fungsi
    "sensor",           // nama
    4096,               // stack (words)
    NULL,               // arg
    1,                  // prioritas
    &g_sensorTask,      // handle
    SENSOR_TASK_CORE    // core
  );

  server.begin();

  g_lastBroadcast = millis();
  g_lastWsClean   = millis();
  g_lastStepUs    = micros();
}

// ===========================================================================
//  LOOP  (stepper + broadcast + housekeeping; sensor jalan di task)
// ===========================================================================
void loop() {
  uint32_t now = millis();

#if TRIG_SELFTEST
  // GPIO13 jadi kotak ~2 Hz -> multimeter DC harusnya baca ~1.6 V.
  digitalWrite(PIN_TRIG, (now / 250) & 1);
  return;
#endif

  steerLoop();

  if (g_forceSend || (now - g_lastBroadcast >= BROADCAST_MS)) {
    g_forceSend     = false;
    g_lastBroadcast = now;
    broadcastSensors();
  }

  if (now - g_lastWsClean >= 1000) {
    g_lastWsClean = now;
    ws.cleanupClients();
  }
}
