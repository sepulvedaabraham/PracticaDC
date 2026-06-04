# PracticaDC
Practica de diseño ESP32
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>


const char* SSID     = "Abraham";
const char* PASSWORD = "123456789";

#define DAC1_PIN   25
#define LED_STATUS  2

Adafruit_ADS1115 ads;

const float ADS_LSB   = 0.0001250f;   
const float V_DIV     = 4.3f;      
const float ACS_VREF  = 2.5f;          
const float ACS_SENS  = 0.1f;          


const int   SWEEP_POINTS = 100;
const int   SETTLE_US    = 200000;   
const float VOC_50       = 11.35f;   
const float ISC_50       = 4.3f;    

const int DAC_MAX       = 255;
const int DAC_THRESHOLD = 178;      

float kp = 0.4f;
float ki = 25.0f;
float dt = 1.0f / 860.0f;


float coef_A0 = kp + ki * dt;
float coef_A1 = -kp;

float pi_out  = 0.0f;
float err_prv = 0.0f;
int   dac_val = 0;


struct IVPoint { float V, I, P; };
IVPoint curve[SWEEP_POINTS];
int     nPts     = 0;
bool    sweeping = false;
bool    ready    = false;
String  status   = "Listo – abre esta IP en tu navegador";

WebServer server(80);


float readV();
float readI();
void  piStep(float ref, float meas);
void  setDAC(int v);
void  runSweep();
void  handleRoot();
void  handleStart();
void  handleData();
void  handleConfig();
void  handleStatus();


void setup() {
  Serial.begin(115200);
  pinMode(LED_STATUS, OUTPUT);
  dacWrite(DAC1_PIN, 0);   // MOSFET apagado al iniciar

  
  Wire.begin(21, 22);
  if (!ads.begin(0x48)) {
    Serial.println("ERROR: ADS1115 no encontrado en 0x48 – revisa cableado I2C");
    while (1) { digitalWrite(LED_STATUS, !digitalRead(LED_STATUS)); delay(200); }
  }
  ads.setGain(GAIN_ONE);               
  ads.setDataRate(RATE_ADS1115_860SPS);   
  
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, true);

  WiFi.begin(SSID, PASSWORD);
  Serial.print("Conectando a ");
  Serial.print(SSID);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 20) {
    delay(500); Serial.print("."); t++;
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_STATUS, HIGH);
    String ip = WiFi.localIP().toString();
    Serial.println("\n\n>>> Abre en tu navegador:  http://" + ip);
    Serial.println(">>> Conectado a: " + String(SSID));
    status = "http://" + ip;
  } else {
    // Sin WiFi → crear punto de acceso propio
    WiFi.softAP("SolarIV", "solar1234");
    String ip = WiFi.softAPIP().toString();
    Serial.println("\n\n>>> WiFi no disponible – modo Access Point");
    Serial.println(">>> Conectate a la red:  SolarIV  (clave: solar1234)");
    Serial.println(">>> Luego abre:          http://" + ip);
    status = "AP SolarIV | http://" + ip;
  }


  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/start",  HTTP_POST, handleStart);
  server.on("/data",   HTTP_GET,  handleData);
  server.on("/config", HTTP_POST, handleConfig);
  server.on("/status", HTTP_GET,  handleStatus);
  server.begin();
  Serial.println("Servidor web iniciado\n");
}

void loop() {
  server.handleClient();
}


float readV() {
  int16_t raw = ads.readADC_SingleEnded(0);
  return max(0.0f, raw * ADS_LSB * V_DIV);
}

float readI() {
  int16_t raw  = ads.getLastConversionResults();
  float   vout = raw * ADS_LSB;
  return max(0.0f, (vout - ACS_VREF) / ACS_SENS);
}


void piStep(float ref, float meas) {
  float e  = ref - meas;
  // Corregido: Uso de coef_A0 y coef_A1
  pi_out   = pi_out + coef_A0 * e + coef_A1 * err_prv;
  pi_out   = constrain(pi_out, 0.0f, (float)DAC_MAX);
  err_prv  = e;
}

void setDAC(int v) {
  dac_val = constrain(v, 0, DAC_MAX);
  dacWrite(DAC1_PIN, dac_val);
}

void runSweep() {
  sweeping = true;
  ready    = false;
  nPts     = 0;
  status   = "Midiendo Voc...";

  const unsigned long DT_US = 1163UL;

 
  setDAC(0);
  pi_out  = 0.0f;
  err_prv = 0.0f;
  delay(800);

  float v = readV();
  float i = readI();
  curve[0] = { v, max(0.0f, i), v * max(0.0f, i) };
  nPts = 1;
  status = "Voc=" + String(v, 2) + "V  Barriendo...";
  Serial.printf("Punto 0 (Voc): V=%.3f V  I=%.4f A\n", v, i);
  server.handleClient();


  pi_out  = DAC_THRESHOLD;
  err_prv = 0.0f;
  setDAC(DAC_THRESHOLD);
  delay(200);

  for (int p = 1; p <= SWEEP_POINTS - 2; p++) {
    float vRef = VOC_50 - (VOC_50 - 0.1f) * (float)(p - 1) / (SWEEP_POINTS - 3);

    unsigned long t0 = micros();
    while ((micros() - t0) < (unsigned long)SETTLE_US) {
      unsigned long ti = micros();
      v = readV();
      piStep(vRef, v);
      setDAC((int)pi_out);
      while ((micros() - ti) < DT_US) { yield(); }
    }
    i = readI();

    if (i > ISC_50 * 1.1f) {
      setDAC(0);
      status = "⚠ Protección Imax – detenido en punto " + String(p);
      Serial.println(status);
      break;
    }

    curve[p] = { v, i, v * i };
    nPts++;

    Serial.printf("Punto %3d: Vref=%.3f  V=%.3f  I=%.4f  P=%.3f  DAC=%d\n",
                  p, vRef, v, i, v*i, dac_val);

    status = "Punto " + String(p) + "/98 – V=" + String(v, 2) + "V";
    server.handleClient();
  }

 
  status = "Midiendo Isc...";
  server.handleClient();
  setDAC(DAC_MAX);
  delay(500);
  v = readV();
  i = readI();
  curve[SWEEP_POINTS - 1] = { max(0.0f, v), i, max(0.0f, v) * i };
  nPts++;
  Serial.printf("Punto 99 (Isc): V=%.3f V  I=%.4f A\n", v, i);

  setDAC(0);
  pi_out  = 0.0f;
  err_prv = 0.0f;

  sweeping = false;
  ready    = true;
  status   = "✓ Completo: " + String(nPts) + " pts | Voc=" +
             String(curve[0].V, 2) + "V  Isc=" +
             String(curve[SWEEP_POINTS-1].I, 3) + "A";
  Serial.println("\n" + status + "\n");
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Trazador I-V Solar</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{--bg:#f0f4ff;--card:#fff;--txt:#1e293b;--acc:#2563eb;--grn:#10b981;--bdr:#cbd5e1}
@media(prefers-color-scheme:dark){:root{--bg:#0f1117;--card:#1a1d27;--txt:#e2e8f0;--bdr:#2d3148}}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt);padding:16px;max-width:960px;margin:auto}
h1{font-size:1.25rem;font-weight:700;color:var(--acc);margin-bottom:14px}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(100px,1fr));gap:10px;margin-bottom:14px}
.kpi{background:var(--card);border:1px solid var(--bdr);border-radius:10px;padding:12px;text-align:center}
.kpi .v{font-size:1.4rem;font-weight:700;color:var(--acc)}
.kpi .l{font-size:.72rem;opacity:.6;margin-top:3px}
.row{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:12px;align-items:center}
button{padding:9px 20px;border-radius:8px;border:none;cursor:pointer;font-size:.9rem;font-weight:600}
.go{background:var(--acc);color:#fff}.go:disabled{opacity:.4;cursor:not-allowed}
.csv{background:transparent;border:1px solid var(--bdr);color:var(--txt)}
#bar{height:6px;background:var(--bdr);border-radius:3px;margin-bottom:6px;overflow:hidden}
#bar div{height:100%;width:0;background:var(--acc);transition:width .4s}
#st{font-size:.82rem;color:var(--acc);min-height:16px;margin-bottom:12px}
.charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px;margin-bottom:14px}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:12px;padding:14px}
.card h2{font-size:.85rem;opacity:.6;margin-bottom:10px}
.cfg{background:var(--card);border:1px solid var(--bdr);border-radius:12px;padding:14px}
.cfg h2{font-size:.85rem;opacity:.6;margin-bottom:10px}
.frow{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:8px}
.frow label{font-size:.82rem;opacity:.7;min-width:45px}
.frow input{width:90px;padding:5px 8px;border-radius:6px;border:1px solid var(--bdr);
            background:var(--bg);color:var(--txt);font-size:.82rem}
.sv{background:var(--grn);color:#fff;padding:7px 16px;border-radius:7px;border:none;cursor:pointer;font-size:.82rem;font-weight:600}
#cm{font-size:.78rem;color:var(--grn);margin-left:8px}
</style>
</head>
<body>
<h1>⚡ Trazador Curva I-V – Panel Solar 150W (50%)</h1>

<div class="kpis">
  <div class="kpi"><div class="v" id="kVoc">–</div><div class="l">Voc (V)</div></div>
  <div class="kpi"><div class="v" id="kIsc">–</div><div class="l">Isc (A)</div></div>
  <div class="kpi"><div class="v" id="kVmp">–</div><div class="l">Vmp (V)</div></div>
  <div class="kpi"><div class="v" id="kImp">–</div><div class="l">Imp (A)</div></div>
  <div class="kpi"><div class="v" id="kPmp">–</div><div class="l">Pmax (W)</div></div>
  <div class="kpi"><div class="v" id="kFF">–</div><div class="l">FF (%)</div></div>
</div>

<div id="bar"><div id="prog"></div></div>
<div id="st">Conectado – presiona Iniciar barrido</div>
<div class="row">
  <button class="go" id="btn" onclick="start()">▶ Iniciar barrido</button>
  <button class="csv" onclick="exportCSV()">↓ CSV</button>
</div>

<div class="charts">
  <div class="card"><h2>Curva I-V</h2><canvas id="cIV"></canvas></div>
  <div class="card"><h2>Curva P-V</h2><canvas id="cPV"></canvas></div>
</div>

<div class="cfg">
  <h2>Parámetros PI de voltaje</h2>
  <div class="frow">
    <label>Kp</label><input id="iKp" value="0.40" type="number" step="0.01">
    <label>Ki</label><input id="iKi" value="25.0" type="number" step="0.5">
  </div>
  <button class="sv" onclick="saveCfg()">Guardar</button>
  <span id="cm"></span>
</div>

<script>
const opt = (axis, lbl) => ({
  responsive:true, maintainAspectRatio:true,
  plugins:{legend:{display:false}},
  scales:{
    x:{type:'linear',title:{display:true,text:lbl.x},ticks:{maxTicksLimit:8}},
    y:{title:{display:true,text:lbl.y}}
  }
});
const IV = new Chart(document.getElementById('cIV'),{
  type:'line',
  data:{datasets:[{data:[],borderColor:'#2563eb',borderWidth:2.5,
        pointRadius:0,fill:false,tension:.35}]},
  options:opt(null,{x:'Voltaje (V)',y:'Corriente (A)'})
});
const PV = new Chart(document.getElementById('cPV'),{
  type:'line',
  data:{datasets:[{data:[],borderColor:'#f59e0b',borderWidth:2.5,
        pointRadius:0,fill:true,backgroundColor:'rgba(245,158,11,.1)',tension:.35}]},
  options:opt(null,{x:'Voltaje (V)',y:'Potencia (W)'})
});

let raw=[], polling=null;

async function start(){
  document.getElementById('btn').disabled=true;
  document.getElementById('st').textContent='Iniciando...';
  document.getElementById('prog').style.width='0%';
  await fetch('/start',{method:'POST'});
  polling = setInterval(poll, 700);
}

async function poll(){
  try{
    const r = await fetch('/status');
    const j = await r.json();
    document.getElementById('st').textContent = j.msg;
    const pct = Math.min(100, j.pts ? j.pts : 0);
    document.getElementById('prog').style.width = pct + '%';
    if(!j.sweeping){
      clearInterval(polling);
      document.getElementById('btn').disabled=false;
      document.getElementById('prog').style.width='100%';
      if(j.ready) load();
    }
  }catch(e){}
}

async function load(){
  const r = await fetch('/data');
  const j = await r.json();
  raw = j.points;
  const iv=[], pv=[];
  let pmax=0, vmp=0, imp=0;
  let voc=0, isc=0;
  raw.forEach((p,idx)=>{
    iv.push({x:+p.v.toFixed(3), y:+p.i.toFixed(3)});
    pv.push({x:+p.v.toFixed(3), y:+p.p.toFixed(3)});
    if(idx===0){voc=p.v;}
    if(idx===raw.length-1){isc=p.i;}
    if(p.p>pmax){pmax=p.p;vmp=p.v;imp=p.i;}
  });
  IV.data.datasets[0].data=iv; IV.update();
  PV.data.datasets[0].data=pv; PV.update();
  const ff = (voc>0&&isc>0) ? (pmax/(voc*isc)*100) : 0;
  document.getElementById('kVoc').textContent=voc.toFixed(2);
  document.getElementById('kIsc').textContent=isc.toFixed(3);
  document.getElementById('kVmp').textContent=vmp.toFixed(2);
  document.getElementById('kImp').textContent=imp.toFixed(3);
  document.getElementById('kPmp').textContent=pmax.toFixed(2);
  document.getElementById('kFF').textContent=ff.toFixed(1);
}

async function saveCfg(){
  const body = JSON.stringify({
    kp: parseFloat(document.getElementById('iKp').value),
    ki: parseFloat(document.getElementById('iKi').value)
  });
  await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body});
  document.getElementById('cm').textContent='✓ Guardado';
  setTimeout(()=>document.getElementById('cm').textContent='',2500);
}

function exportCSV(){
  if(!raw.length)return;
  let s='V(V),I(A),P(W)\n';
  raw.forEach(p=>s+=`${p.v.toFixed(4)},${p.i.toFixed(4)},${p.p.toFixed(4)}\n`);
  const a=document.createElement('a');
  a.href='data:text/csv;charset=utf-8,'+encodeURIComponent(s);
  a.download='curva_iv_solar.csv'; a.click();
}

fetch('/status').then(r=>r.json()).then(j=>{
  document.getElementById('st').textContent=j.msg;
  if(j.ready) load();
});
</script>
</body>
</html>
)HTML";
  server.send_P(200, "text/html", HTML);
}

void handleStart() {
  server.send(200, "application/json", "{\"ok\":true}");
  if (!sweeping) {
    delay(50); 
    runSweep();
  }
}

void handleData() {
  String out = "{\"points\":[";
  for (int i = 0; i < nPts; i++) {
    if (i) out += ",";
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"v\":%.4f,\"i\":%.4f,\"p\":%.4f}",
             curve[i].V, curve[i].I, curve[i].P);
    out += buf;
  }
  out += "]}";
  server.send(200, "application/json", out);
}

void handleConfig() {
  if (server.hasArg("plain")) {
    StaticJsonDocument<128> doc;
    if (!deserializeJson(doc, server.arg("plain"))) {
      kp = doc["kp"] | kp;
      ki = doc["ki"] | ki;
      
      coef_A0 = kp + ki * dt;
      coef_A1 = -kp;
      Serial.printf("PI actualizado: Kp=%.3f Ki=%.2f  A0=%.5f A1=%.4f\n", kp, ki, coef_A0, coef_A1);
    }
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"sweeping\":%s,\"ready\":%s,\"pts\":%d,\"dac\":%d,\"msg\":\"%s\"}",
           sweeping ? "true" : "false",
           ready    ? "true" : "false",
           nPts, dac_val,
           status.c_str());
  server.send(200, "application/json", buf);
}