#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <math.h>

// ========== PIN DEFINITIONS ==========
#define SDA_PIN     21
#define SCL_PIN     22
#define BUTTON_PIN  23

// ========== OLED ==========
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                         &Wire, -1);
MPU6050 mpu;
WebServer server(80);

// ========== WIFI ACCESS POINT ==========
const char* AP_SSID = "AIR_BLACK_BOX";
const char* AP_PASS = "flight123";

// ========== SENSOR VARIABLES ==========
int16_t ax, ay, az, gx, gy, gz;

float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;
float magA, magW;
float roll, pitch;
float freq = 0;

// ========== TIMING ==========
unsigned long previousSample = 0;
unsigned long previousDisplay = 0;
unsigned long previousLoop = 0;

const unsigned long SAMPLE_INTERVAL = 20; // 50 Hz
const unsigned long DISPLAY_INTERVAL = 100;

// ========== BUTTON ==========
int page = 0;
bool lastReading = LOW;
bool stableState = LOW;
unsigned long debounceTime = 0;
const unsigned long DEBOUNCE_MS = 40;

// ========== DASHBOARD HTML ==========
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Airplane Black Box</title>
<style>
*{box-sizing:border-box}
body{
  margin:0;background:#0b1220;color:#e8eef8;
  font-family:Arial,sans-serif;padding:16px
}
header{
  max-width:1000px;margin:auto;display:flex;
  justify-content:space-between;align-items:center;
  flex-wrap:wrap;gap:10px
}
h1{font-size:23px;margin:8px 0}
.status{
  color:#4ade80;background:#123324;padding:8px 12px;
  border-radius:20px;font-size:12px
}
main{max-width:1000px;margin:20px auto}
.section{font-size:14px;color:#94a3b8;margin:22px 0 10px}
.grid{
  display:grid;grid-template-columns:repeat(2,minmax(0,1fr));
  gap:12px
}
.card{
  background:#151f31;border:1px solid #26344b;
  padding:16px;border-radius:12px;min-width:0
}
.label{color:#a6b4c9;font-size:13px}
.value{font-size:27px;font-weight:bold;margin-top:12px;
       overflow-wrap:anywhere}
.unit{font-size:12px;color:#9caec4}
.panel{
  background:#151f31;border:1px solid #26344b;
  padding:15px;border-radius:12px;margin-top:12px
}
canvas{width:100%;height:210px;display:block}
select{
  background:#26344b;color:white;border:1px solid #526078;
  padding:8px;border-radius:6px
}
.small{color:#9caec4;font-size:12px;line-height:1.6}
@media(min-width:650px){
 .grid{grid-template-columns:repeat(3,minmax(0,1fr))}
}
</style>
</head>
<body>
<header>
  <h1>FLIGHT BLACK BOX</h1>
  <span class="status" id="status">Connecting...</span>
</header>

<main>
  <div class="small">
    ESP32 flight motion monitoring |
    Local Wi-Fi dashboard
  </div>

  <div class="section">ACCELEROMETER</div>
  <div class="grid">
    <div class="card">
      <div class="label">Acceleration X</div>
      <div class="value" id="ax">--</div>
      <div class="unit">g</div>
    </div>
    <div class="card">
      <div class="label">Acceleration Y</div>
      <div class="value" id="ay">--</div>
      <div class="unit">g</div>
    </div>
    <div class="card">
      <div class="label">Acceleration Z</div>
      <div class="value" id="az">--</div>
      <div class="unit">g</div>
    </div>
  </div>

  <div class="section">GYROSCOPE</div>
  <div class="grid">
    <div class="card">
      <div class="label">Gyroscope X</div>
      <div class="value" id="gx">--</div>
      <div class="unit">deg/s</div>
    </div>
    <div class="card">
      <div class="label">Gyroscope Y</div>
      <div class="value" id="gy">--</div>
      <div class="unit">deg/s</div>
    </div>
    <div class="card">
      <div class="label">Gyroscope Z</div>
      <div class="value" id="gz">--</div>
      <div class="unit">deg/s</div>
    </div>
  </div>

  <div class="section">FLIGHT ORIENTATION</div>
  <div class="grid">
    <div class="card">
      <div class="label">Roll</div>
      <div class="value" id="roll">--</div>
      <div class="unit">degrees</div>
    </div>
    <div class="card">
      <div class="label">Pitch</div>
      <div class="value" id="pitch">--</div>
      <div class="unit">degrees</div>
    </div>
    <div class="card">
      <div class="label">Acceleration magnitude</div>
      <div class="value" id="magA">--</div>
      <div class="unit">g</div>
    </div>
    <div class="card">
      <div class="label">Angular velocity magnitude</div>
      <div class="value" id="magW">--</div>
      <div class="unit">deg/s</div>
    </div>
    <div class="card">
      <div class="label">Sampling frequency</div>
      <div class="value" id="freq">--</div>
      <div class="unit">Hz</div>
    </div>
  </div>

  <div class="section">LIVE GRAPH</div>
  <div class="panel">
    <select id="axis" onchange="draw()">
      <option value="ax">Acceleration X</option>
      <option value="ay">Acceleration Y</option>
      <option value="az">Acceleration Z</option>
      <option value="gx">Gyroscope X</option>
      <option value="gy">Gyroscope Y</option>
      <option value="gz">Gyroscope Z</option>
      <option value="roll">Roll</option>
      <option value="pitch">Pitch</option>
    </select>
    <div class="small" id="graphLabel">
      Last 60 seconds | X: time | Y: value
    </div>
    <canvas id="chart"></canvas>
  </div>

  <div class="small" style="margin-top:16px">
    Data shown is live and is not stored on the ESP32.
    Sensor orientation depends on how the MPU6050 is mounted.
  </div>
</main>

<script>
const fields=[
 "ax","ay","az","gx","gy","gz",
 "roll","pitch","magA","magW","freq"
];
let history=[];
let lastData=null;

async function update(){
 try{
   const r=await fetch('/data',{cache:'no-store'});
   if(!r.ok) throw Error("HTTP "+r.status);
   const d=await r.json();
   lastData=d;
   fields.forEach(k=>{
     document.getElementById(k).textContent=
       Number(d[k]).toFixed(k==="freq"?1:2);
   });
   document.getElementById('status').textContent="CONNECTED";
   document.getElementById('status').style.color="#4ade80";
   history.push({t:Date.now(),...d});
   const cutoff=Date.now()-60000;
   history=history.filter(p=>p.t>=cutoff);
   draw();
 }catch(e){
   document.getElementById('status').textContent="DISCONNECTED";
   document.getElementById('status').style.color="#fb7185";
 }
}

function draw(){
 const c=document.getElementById('chart');
 const rect=c.getBoundingClientRect();
 if(!rect.width)return;
 const dpr=window.devicePixelRatio||1;
 c.width=Math.round(rect.width*dpr);
 c.height=Math.round(210*dpr);
 const ctx=c.getContext('2d');
 ctx.scale(dpr,dpr);
 const w=rect.width,h=210;
 ctx.clearRect(0,0,w,h);

 const key=document.getElementById('axis').value;
 const pts=history.filter(p=>Number.isFinite(Number(p[key])));
 if(pts.length<2){
   ctx.fillStyle="#a6b4c9";
   ctx.fillText("Waiting for sensor data...",12,30);
   return;
 }

 const values=pts.map(p=>Number(p[key]));
 let min=Math.min(0,...values),max=Math.max(0,...values);
 if(max-min<0.001){min-=1;max+=1}
 const pad=(max-min)*0.12;
 min-=pad;max+=pad;
 const left=45,right=10,top=12,bottom=24;
 const gw=w-left-right,gh=h-top-bottom;

 ctx.strokeStyle="#29364a";
 ctx.lineWidth=1;
 for(let i=0;i<=4;i++){
   let y=top+gh*i/4;
   ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();
   let val=max-(max-min)*i/4;
   ctx.fillStyle="#a6b4c9";
   ctx.font="10px Arial";
   ctx.textAlign="right";
   ctx.fillText(val.toFixed(1),left-5,y+3);
 }
 ctx.strokeStyle="#60a5fa";
 ctx.lineWidth=2;
 ctx.beginPath();
 pts.forEach((p,i)=>{
   const x=left+gw*i/(pts.length-1);
   const y=top+gh*(max-Number(p[key]))/(max-min);
   if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
 });
 ctx.stroke();
 ctx.fillStyle="#a6b4c9";
 ctx.textAlign="left";
 ctx.fillText("60s ago",left,h-5);
 ctx.textAlign="right";
 ctx.fillText("Now",w-right,h-5);
}
window.addEventListener('resize',draw);
setInterval(update,250);
update();
</script>
</body>
</html>
)rawliteral";

// ========== BUTTON ==========
void checkButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    debounceTime = millis();
  }

  if (millis() - debounceTime >= DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == HIGH) {
        page = (page + 1) % 2;
      }
    }
  }
  lastReading = reading;
}

// ========== SENSOR READING ==========
void readSensors() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  ax_g = ax / 16384.0f;
  ay_g = ay / 16384.0f;
  az_g = az / 16384.0f;

  gx_dps = gx / 131.0f;
  gy_dps = gy / 131.0f;
  gz_dps = gz / 131.0f;

  magA = sqrtf(ax_g * ax_g +
               ay_g * ay_g +
               az_g * az_g);

  magW = sqrtf(gx_dps * gx_dps +
               gy_dps * gy_dps +
               gz_dps * gz_dps);

  roll = atan2f(ay_g, az_g) * 180.0f / PI;

  pitch = atan2f(-ax_g,
           sqrtf(ay_g * ay_g + az_g * az_g))
           * 180.0f / PI;
}

// ========== FREQUENCY ==========
void calculateFrequency(unsigned long now) {
  if (previousSample != 0) {
    unsigned long dt = now - previousSample;
    if (dt > 0) {
      freq = 1000000.0f / dt;
    }
  }
  previousSample = now;
}

// ========== OLED PAGE A ==========
void showPageA() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Flight Black Box A");

  display.setCursor(0, 12);
  display.print("AP: AIR_BLACK_BOX");

  display.setCursor(0, 22);
  display.print("Hz: ");
  display.print(freq, 1);

  display.setCursor(0, 34);
  display.print("ax:");
  display.print(ax_g, 2);

  display.setCursor(64, 34);
  display.print("R:");
  display.print(roll, 1);

  display.setCursor(0, 46);
  display.print("ay:");
  display.print(ay_g, 2);

  display.setCursor(64, 46);
  display.print("P:");
  display.print(pitch, 1);

  display.display();
}

// ========== OLED PAGE B ==========
void showPageB() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Flight Black Box B");

  display.setCursor(0, 12);
  display.print("az:");
  display.print(az_g, 2);

  display.setCursor(64, 12);
  display.print("R:");
  display.print(roll, 1);

  display.setCursor(0, 23);
  display.print("gx:");
  display.print(gx_dps, 0);

  display.setCursor(64, 23);
  display.print("P:");
  display.print(pitch, 1);

  display.setCursor(0, 34);
  display.print("gy:");
  display.print(gy_dps, 0);

  display.setCursor(0, 45);
  display.print("gz:");
  display.print(gz_dps, 0);

  display.setCursor(0, 56);
  display.print("|a|:");
  display.print(magA, 2);

  display.print(" |w|:");
  display.print(magW, 0);

  display.display();
}

// ========== WEB DASHBOARD ==========
void handleRoot() {
  server.send_P(200, "text/html", webpage);
}

void handleData() {
  String json;
  json.reserve(300);

  json = "{";
  json += "\"ax\":" + String(ax_g, 3) + ",";
  json += "\"ay\":" + String(ay_g, 3) + ",";
  json += "\"az\":" + String(az_g, 3) + ",";
  json += "\"gx\":" + String(gx_dps, 2) + ",";
  json += "\"gy\":" + String(gy_dps, 2) + ",";
  json += "\"gz\":" + String(gz_dps, 2) + ",";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"magA\":" + String(magA, 3) + ",";
  json += "\"magW\":" + String(magW, 2) + ",";
  json += "\"freq\":" + String(freq, 2);
  json += "}";

  server.send(200, "application/json", json);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED initialization failed");
    while (true) delay(100);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("AIR BLACK BOX");
  display.println("Starting...");
  display.display();

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 ERROR");
    display.display();
    while (true) delay(100);
  }

  // Start ESP32 access point
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("AP startup failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP ERROR");
    display.display();
    while (true) delay(100);
  }

  // Start web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.begin();

  Serial.println("Black box ready");
  Serial.print("WiFi SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BLACK BOX READY");
  display.println("WiFi: AIR_BLACK_BOX");
  display.println("IP: 192.168.4.1");
  display.display();

  delay(1500);
}

// ========== MAIN LOOP ==========
void loop() {
  server.handleClient();
  checkButton();

  unsigned long now = millis();

  if (now - previousLoop >= SAMPLE_INTERVAL) {
    previousLoop = now;

    readSensors();
    calculateFrequency(micros());
  }

  if (now - previousDisplay >= DISPLAY_INTERVAL) {
    previousDisplay = now;

    if (page == 0) {
      showPageA();
    } else {
      showPageB();
    }
  }
}
This is the code I want to upload on GitHub how to make its file