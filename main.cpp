/*  WasteBot Phone6
 *  ===================================================
 *  WiFi: WasteBot / wastebot123 → open 192.168.4.1
 *
 *  ARM POSITIONS:
 *    Initial:      {60,  83, 180, 107,  64,  0}
 *    Intermediate: {17,  83, 180, 107,  64,  0}  gripper=17(closed)
 *    Unload:       {100,180, 109, 142,  88, 88}  base=88, gripper=100(drop)
 *
 *  ARM SEQUENCE:
 *    PICK:         0→5→4→3→2→1  then close gripper 0→17
 *    INTERMEDIATE: 1→2→3→4→5    (gripper stays 17)
 *    UNLOAD:       5→4→3→2→1→0  (base→S4→S3→S2→S1→gripper=100)
 *    RETURN:       4→3→2→1→0→5  back to initial
 *    500ms between each servo
 *
 *  PICK ANGLES by distance:
 *    ≥13cm: {113,110,175,104,147,0}
 *    12cm:  {113,110,170, 92,144,0}
 *    10-11: {113,100,170, 92,140,0}
 *    9cm:   {113, 99,170, 94,142,0}
 *    8cm:   {113, 99,170, 60,129,0}
 *    ≤7cm:  {113, 79,168, 70,130,0}
 *
 *  Motor: ENA=15 IN1=2 IN2=4 ENB=19 IN3=16 IN4=17
 *  Servo: 0=13 1=32 2=14 3=27 4=26 5=25
 *  Sonar: TRIG=5 ECHO=18
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <NewPing.h>

// ── WiFi AP ───────────────────────────────────
const char* AP_SSID = "WasteBot";
const char* AP_PASS = "wastebot123";
WebServer server(80);

// ── Motor pins ────────────────────────────────
#define ENA 15
#define ENB 19
#define IN1  2
#define IN2  4
#define IN3 16
#define IN4 17

// ── Sonar ─────────────────────────────────────
#define TRIG_PIN  5
#define ECHO_PIN 18
#define MAX_CM  400
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_CM);

// ── Thresholds ────────────────────────────────
#define TARGET_CM     2.0f  // PID drives car to 2cm, then measures real dist for arm angles
#define DEAD_BAND     1.5f
#define SLOW_CM      12.0f
#define DETECT_CM   200
#define PID_TRY_MS  5000
#define SETTLE_WAIT 5000
#define MANUAL_WAIT 3000

// ── Speeds ────────────────────────────────────
#define SPD_FULL  180
#define SPD_MIN   140
#define SPD_MAX   175
#define SPD_TURN  160

// ── PID ───────────────────────────────────────
float Kp=8.0f, Ki=0.1f, Kd=1.0f;
float pidIntegral=0, pidPrevErr=0;
unsigned long pidLastT=0;
void resetPID(){pidIntegral=0;pidPrevErr=0;pidLastT=0;}

// ── Timing ────────────────────────────────────
#define TURN_MS   150
#define ROT_MS    150
#define SETTLE_MS 300

// ── State ─────────────────────────────────────
float         lastDist   = MAX_CM;
unsigned long lastSonarT = 0;
bool          autoMode   = false;
String        autoStatus = "Idle";
bool          turnActive = false;
unsigned long turnEnd    = 0;

// ── Sonar ─────────────────────────────────────
unsigned int medianDist5() {
    unsigned int p[5];
    for(int i=0;i<5;i++){p[i]=sonar.ping_cm();delay(12);if(!p[i])p[i]=MAX_CM;}
    for(int i=0;i<4;i++) for(int j=0;j<4-i;j++) if(p[j]>p[j+1]){auto t=p[j];p[j]=p[j+1];p[j+1]=t;}
    return p[2];
}
void updateSonar(){
    if(millis()-lastSonarT<150)return;
    lastSonarT=millis();
    lastDist=(float)medianDist5();
}
float measureNow(){
    unsigned int p[5];
    for(int i=0;i<5;i++){p[i]=sonar.ping_cm();delay(30);if(!p[i])p[i]=MAX_CM;}
    for(int i=0;i<4;i++) for(int j=0;j<4-i;j++) if(p[j]>p[j+1]){auto t=p[j];p[j]=p[j+1];p[j+1]=t;}
    return (float)p[2];
}

// ── Motors ────────────────────────────────────
void mStop(){digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);analogWrite(ENA,0);analogWrite(ENB,0);}
void mFwd(int s){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENA,s);analogWrite(ENB,s);}
void mBwd(int s){digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENA,s);analogWrite(ENB,s);}
void mLeft(){digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENA,SPD_TURN);analogWrite(ENB,SPD_TURN);}
void mRight(){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENA,SPD_TURN);analogWrite(ENB,SPD_TURN);}
void handleTurnTick(){if(turnActive&&millis()>=turnEnd){mStop();turnActive=false;}}

// ── Servos ────────────────────────────────────
Servo svos[6];
int svPins[6] = {13, 32, 14, 27, 26, 25};
int curPos[6] = {60, 83,180,107, 64,  0};  // matches initPos

//                         grip  s1   s2   s3   s4  base
int initPos[6]   = { 60,  83, 150, 107,  64,   0};  // initial rest
int interPos[6]  = { 17,  83, 150, 107,  64,   0};  // intermediate (holding obj)
int unloadPos[6] = {113, 110, 180, 92,  147,  88};  // unload (base=88, gripper=100)

void getPickAngles(int dist, int out[6]) {
    int a13[6] = {113,110,180,104,147,3};  // >=13cm
    int a12[6] = {113,110,180, 92,144,3};  // 12cm
    int a10[6] = {113,100,180, 92,140,3};  // 10-11cm
    int a9[6]  = {113, 99,180, 94,142,3};  // 9cm
    int a8[6]  = {113, 99,180, 60,129,3};  // 8cm
    int a7[6]  = {113, 79,180, 70,130,3};  // 6-7cm
    int a5[6]  = {113, 79,180, 70,130,3};  // <=5cm (same as 7cm)
    int* src;
    if     (dist>=13) src=a13;
    else if(dist==12) src=a12;
    else if(dist>=10) src=a10;
    else if(dist== 9) src=a9;
    else if(dist>= 6) src=a7;
    else              src=a5;
    for(int i=0;i<6;i++) out[i]=src[i];
}

void smoothMove(int idx, int target){
    int step=(curPos[idx]<target)?1:-1;
    for(int a=curPos[idx];a!=target;a+=step){svos[idx].write(a);delay(15);}
    svos[idx].write(target);delay(60);
    curPos[idx]=target;
}

volatile bool armRunning=false;
TaskHandle_t  armHandle=NULL;
int           armDist=10;
String        armPhase="Idle";  // current arm phase for display

void armTask(void* pv){
    int pick[6];
    getPickAngles(armDist, pick);

    char buf[48];
    snprintf(buf,sizeof(buf),"ARM:Picking at %dcm",armDist);
    Serial.println(buf);

    // ══ PHASE 1: PICK ════════════════════════════
    armPhase = "1:PICK";
    // Order: gripper(0)→base(5)→S4(4)→S3(3)→S2(2)→S1(1)
    smoothMove(0, 113);        delay(500);
    smoothMove(5, pick[5]);    delay(500);
    smoothMove(4, pick[4]);    delay(500);
    smoothMove(3, pick[3]);    delay(500);
    smoothMove(2, pick[2]);    delay(500);
    smoothMove(1, pick[1]);    delay(500);
    smoothMove(0, 17);         delay(500);  // gripper close
    delay(3000);  // 3s gap before next phase

    // ══ PHASE 2: INTERMEDIATE ════════════════════
    armPhase = "2:INTERMEDIATE";
    smoothMove(4, interPos[4]); delay(500);
    smoothMove(3, interPos[3]); delay(500);
    smoothMove(2, interPos[2]); delay(500);
    smoothMove(1, interPos[1]); delay(500);
    smoothMove(5, interPos[5]); delay(500);
    delay(3000);  // 3s gap before next phase

    // ══ PHASE 3: UNLOAD ══════════════════════════
    armPhase = "3:UNLOAD";
    smoothMove(5, unloadPos[5]); delay(500);
    smoothMove(4, unloadPos[4]); delay(500);
    smoothMove(3, unloadPos[3]); delay(500);
    smoothMove(2, unloadPos[2]); delay(500);
    smoothMove(1, unloadPos[1]); delay(500);
    smoothMove(0, unloadPos[0]); delay(500);  // gripper → 100 drop
    delay(3000);  // 3s gap before next phase

    // ══ PHASE 4: RETURN TO INITIAL ═══════════════
    armPhase = "4:RETURN";
    smoothMove(4, initPos[4]);   delay(500);
    smoothMove(3, initPos[3]);   delay(500);
    smoothMove(2, initPos[2]);   delay(500);
    smoothMove(1, initPos[1]);   delay(500);
    smoothMove(0, initPos[0]);   delay(500);
    smoothMove(5, initPos[5]);   delay(500);

    armPhase="Idle";
    autoStatus="Done! Searching...";
    Serial.println("ARM:DONE");
    armRunning=false; armHandle=NULL;
    vTaskDelete(NULL);
}

void fireArm(int dist){
    if(!armRunning){armDist=dist;armRunning=true;xTaskCreatePinnedToCore(armTask,"arm",4096,NULL,1,&armHandle,0);}
}

// ── PID ───────────────────────────────────────
bool pidApproach(){
    unsigned long now=millis();
    float dt=(pidLastT==0)?0.15f:(now-pidLastT)/1000.0f;
    pidLastT=now;
    float err=lastDist-TARGET_CM;
    pidIntegral=constrain(pidIntegral+err*dt,-30.0f,30.0f);
    float deriv=(err-pidPrevErr)/dt; pidPrevErr=err;
    float out=Kp*err+Ki*pidIntegral+Kd*deriv;
    if(fabsf(err)<=DEAD_BAND){mStop();return true;}
    int spd=(int)constrain(fabsf(out),(float)SPD_MIN,(float)SPD_MAX);
    if(out>0)mFwd(spd);else mBwd(spd);
    return false;
}

// ── Autonomous FSM ────────────────────────────
enum AutoState:uint8_t{
    AS_IDLE=0,AS_SRCH_SETTLE,AS_SRCH_SCAN,AS_SRCH_ROTATE,AS_SRCH_ROT_WAIT,
    AS_APPROACH_FULL,AS_APPROACH_PID,AS_SETTLE_WAIT,AS_MEASURE,
    AS_ARM_WAIT,AS_CYCLE_SETTLE
};
AutoState autoState=AS_IDLE;
unsigned long autoTimer=0,pidStartT=0;
int rotSteps=0;

void goTo(AutoState s,const char* msg=""){autoState=s;autoTimer=0;if(strlen(msg))autoStatus=String(msg);}

void runAuto(){
    unsigned long now=millis();
    switch(autoState){
    case AS_SRCH_SETTLE:
        mStop();autoStatus="Searching...";
        if(!autoTimer)autoTimer=now+SETTLE_MS;
        if(now>=autoTimer){autoTimer=0;autoState=AS_SRCH_SCAN;}break;
    case AS_SRCH_SCAN:
        if(lastDist<=SLOW_CM){resetPID();pidStartT=now;goTo(AS_APPROACH_PID,"PID approach...");break;}
        if(lastDist<DETECT_CM){goTo(AS_APPROACH_FULL,"Approaching...");break;}
        autoStatus="Searching...";mRight();autoTimer=now+ROT_MS;autoState=AS_SRCH_ROTATE;break;
    case AS_SRCH_ROTATE:
        if(now>=autoTimer){mStop();rotSteps++;autoTimer=now+SETTLE_MS;autoState=AS_SRCH_ROT_WAIT;}break;
    case AS_SRCH_ROT_WAIT:
        if(now>=autoTimer){autoTimer=0;autoState=AS_SRCH_SCAN;}break;
    case AS_APPROACH_FULL:
        if(lastDist>=(float)DETECT_CM){mStop();goTo(AS_SRCH_SETTLE,"Object lost");break;}
        if(lastDist<=SLOW_CM){mStop();resetPID();pidStartT=now;goTo(AS_APPROACH_PID,"PID approach...");break;}
        mFwd(SPD_FULL);autoStatus="Approaching...";break;
    case AS_APPROACH_PID:{
        if(lastDist>=(float)DETECT_CM){mStop();goTo(AS_SRCH_SETTLE,"Object lost");break;}
        if(millis()-lastSonarT<150)break;  // wait fresh sonar

        bool settled=pidApproach();

        char buf[48];
        snprintf(buf,sizeof(buf),"PID: %.0fcm err:%.1f",lastDist,lastDist-TARGET_CM);
        autoStatus=String(buf);

        // PID done or timed out after PID_TRY_MS
        if(settled || (now-pidStartT>=PID_TRY_MS)){
            mStop();
            // Wait 5 seconds so car fully stops and sonar stabilises
            autoTimer = now + SETTLE_WAIT;
            goTo(AS_SETTLE_WAIT,"Stopped — waiting 5s...");
        }
        break;}

    case AS_SETTLE_WAIT:
        mStop();  // keep stopped
        if(now >= autoTimer){
            goTo(AS_MEASURE,"Measuring exact distance...");
        } else {
            char buf[30];
            int rem=(int)((autoTimer-now)/1000)+1;
            snprintf(buf,sizeof(buf),"Waiting %ds...",rem);
            autoStatus=String(buf);
        }
        break;

    case AS_MEASURE:{
        mStop();
        // Take accurate measurement — 5 sample median
        float d = measureNow();
        // Clamp to valid pick range
        int di = constrain((int)round(d), 4, 13);
        char buf[48];
        snprintf(buf,sizeof(buf),"Dist: %dcm — starting arm",di);
        autoStatus=String(buf);
        Serial.println(buf);
        // Fire arm with exact measured distance
        fireArm(di);
        autoState=AS_ARM_WAIT;
        break;}
    case AS_ARM_WAIT:
        if(!armRunning){autoTimer=now+1500;autoState=AS_CYCLE_SETTLE;}break;
    case AS_CYCLE_SETTLE:
        if(now>=autoTimer){rotSteps=0;resetPID();goTo(AS_SRCH_SETTLE,"Searching...");}break;
    default:break;
    }
}

// ── Manual arm ────────────────────────────────
bool manualArmPending=false;
unsigned long manualArmWaitEnd=0;

void startManualArm(){
    if(armRunning)return;
    manualArmPending=true;
    manualArmWaitEnd=millis()+MANUAL_WAIT;
    autoStatus="Arm in 3s...";
}
void handleManualArmTick(){
    if(!manualArmPending)return;
    if(armRunning){manualArmPending=false;return;}
    if(millis()>=manualArmWaitEnd){
        manualArmPending=false;
        float d=measureNow();int di=constrain((int)round(d),7,15);
        char buf[40];snprintf(buf,sizeof(buf),"Manual arm at %dcm",di);
        autoStatus=String(buf);Serial.println(buf);fireArm(di);
    }else{
        char buf[20];snprintf(buf,sizeof(buf),"Arm in %ds...",(int)((manualArmWaitEnd-millis())/1000)+1);
        autoStatus=String(buf);
    }
}

// ── HTML ──────────────────────────────────────
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>WasteBot</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;user-select:none;}
body{margin:0;background:#1a1a2e;color:#fff;font-family:Arial,sans-serif;display:flex;flex-direction:column;align-items:center;padding:10px;}
h1{color:#e94560;margin:8px 0 4px;font-size:20px;}
.card{background:#0f0f23;border:2px solid #333;border-radius:12px;padding:12px;width:100%;max-width:420px;margin-bottom:10px;}
.title{color:#e94560;font-size:12px;font-weight:bold;text-align:center;margin-bottom:8px;letter-spacing:1px;}
.dist{font-size:52px;font-weight:bold;font-family:monospace;text-align:center;}
.dist.ok{color:#16c79a;}.dist.slow{color:#ffb300;}.dist.stop{color:#f44336;}
.warn{font-size:12px;text-align:center;min-height:16px;color:#f44336;}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:260px;margin:0 auto 8px;}
.btn{border:none;border-radius:10px;font-size:26px;padding:18px 0;cursor:pointer;width:100%;touch-action:manipulation;}
.btn:active{opacity:.65;transform:scale(.94);}
.fwd,.bwd{background:#4CAF50;color:#fff;}
.lft,.rgt{background:#2196F3;color:#fff;}
.stp{background:#f44336;color:#fff;font-size:14px;font-weight:bold;}
.row{display:flex;gap:8px;margin-top:6px;}
.abtn{flex:1;background:#9C27B0;color:#fff;border:none;border-radius:10px;padding:13px 0;font-size:13px;font-weight:bold;cursor:pointer;touch-action:manipulation;}
.abtn:active{opacity:.65;}
.abtn.orange{background:#FF9800;}
.auto-status{text-align:center;font-size:14px;font-weight:bold;margin:4px 0 10px;color:#16c79a;min-height:22px;}
.abig{width:100%;border:none;border-radius:10px;padding:14px;font-size:14px;font-weight:bold;color:#fff;cursor:pointer;margin-bottom:6px;touch-action:manipulation;}
.abig:active{opacity:.65;}
.start{background:#4CAF50;}.stopbtn{background:#f44336;}.resetbtn{background:#FF9800;}.armbtn{background:#e94560;}
.pid-box{display:flex;gap:6px;align-items:center;margin:4px 0;font-size:12px;}
.pid-box label{width:30px;color:#aaa;}
.pid-box input{flex:1;}
.pid-box span{width:38px;text-align:right;color:#16c79a;font-family:monospace;font-size:11px;}
.srow{display:flex;align-items:center;gap:8px;margin:7px 0;}
.srow label{font-size:11px;font-weight:bold;width:82px;flex-shrink:0;}
.srow input[type=range]{flex:1;}
.srow span{width:38px;text-align:right;color:#16c79a;font-family:monospace;font-size:12px;}
.pos-info{font-size:10px;color:#555;text-align:center;margin-top:4px;}
.ph{display:flex;align-items:flex-start;gap:10px;background:#1a1a2e;border:2px solid #333;border-radius:10px;padding:10px;transition:all .3s;}
.ph.active{border-color:#16c79a;background:#0a2a1a;box-shadow:0 0 12px #16c79a55;}
.ph-num{background:#333;color:#fff;font-weight:bold;font-size:16px;width:28px;height:28px;border-radius:50%;display:flex;align-items:center;justify-content:center;flex-shrink:0;}
.ph.active .ph-num{background:#16c79a;color:#000;}
.ph-body{flex:1;}
.ph-name{font-weight:bold;font-size:14px;color:#aaa;}
.ph.active .ph-name{color:#16c79a;}
.ph-info{font-size:11px;color:#666;margin-top:2px;}
.ph.active .ph-info{color:#aaa;}
.ph-pos{font-size:10px;color:#555;font-family:monospace;margin-top:3px;}
.ph.active .ph-pos{color:#16c79a;}
.ph-arrow{text-align:center;color:#555;font-size:12px;padding:4px 0;}
</style>
</head>
<body>
<h1>&#129302; WasteBot</h1>

<div class="card">
  <div class="title">SONAR</div>
  <div class="dist ok" id="dist">-- cm</div>
  <div class="warn" id="warn"></div>
</div>

<div class="card">
  <div class="title">DRIVE CONTROLS</div>
  <div class="dpad">
    <div></div>
    <button class="btn fwd" ontouchstart="go('forward')" onmousedown="go('forward')">&#9650;</button>
    <div></div>
    <button class="btn lft" ontouchstart="go('left')" onmousedown="go('left')">&#9668;</button>
    <button class="btn stp" ontouchstart="go('stop')" onmousedown="go('stop')">STOP</button>
    <button class="btn rgt" ontouchstart="go('right')" onmousedown="go('right')">&#9658;</button>
    <div></div>
    <button class="btn bwd" ontouchstart="go('backward')" onmousedown="go('backward')">&#9660;</button>
    <div></div>
  </div>
  <div class="row">
    <button class="abtn orange" ontouchstart="go('drop')" onmousedown="go('drop')">Drop Waste</button>
    <button class="abtn orange" ontouchstart="go('reset')" onmousedown="go('reset')">Reset</button>
  </div>
</div>

<div class="card">
  <div class="title">AUTONOMOUS MODE</div>
  <div class="auto-status" id="autoStatus">&#9679; Idle</div>
  <button class="abig start" ontouchstart="go('start_auto')" onmousedown="go('start_auto')">&#9654; Start Auto</button>
  <button class="abig stopbtn" ontouchstart="go('stop_auto')" onmousedown="go('stop_auto')">&#9632; Stop Auto</button>
  <button class="abig resetbtn" ontouchstart="go('reset_auto')" onmousedown="go('reset_auto')">&#8635; Reset Search</button>
</div>

<div class="card">
  <div class="title">AUTO ARM (waits 3s, measures distance, picks angles)</div>
  <button class="abig armbtn" ontouchstart="go('arm')" onmousedown="go('arm')">&#9654; Auto Arm Cycle</button>
</div>

<div class="card">
  <div class="title">ARM CYCLE MONITOR</div>
  <div id="phases">
    <div class="ph" id="ph0">
      <div class="ph-num">1</div>
      <div class="ph-body">
        <div class="ph-name">PICK</div>
        <div class="ph-info">Grip:113&#8594;17 | Base&#8594;pick angles</div>
        <div class="ph-pos">{113, 83&#8594;pick, 180&#8594;pick, 107&#8594;pick, 64&#8594;pick, 0&#8594;pick}</div>
      </div>
    </div>
    <div class="ph-arrow">&#8595; 3s &#8595;</div>
    <div class="ph" id="ph1">
      <div class="ph-num">2</div>
      <div class="ph-body">
        <div class="ph-name">INTERMEDIATE</div>
        <div class="ph-info">Gripper stays 17 (holding object)</div>
        <div class="ph-pos">{17, 83, 180, 107, 64, 0}</div>
      </div>
    </div>
    <div class="ph-arrow">&#8595; 3s &#8595;</div>
    <div class="ph" id="ph2">
      <div class="ph-num">3</div>
      <div class="ph-body">
        <div class="ph-name">UNLOAD</div>
        <div class="ph-info">Gripper opens to 100 (drop object)</div>
        <div class="ph-pos">{100, 180, 109, 142, 88, 88}</div>
      </div>
    </div>
    <div class="ph-arrow">&#8595; 3s &#8595;</div>
    <div class="ph" id="ph3">
      <div class="ph-num">4</div>
      <div class="ph-body">
        <div class="ph-name">RETURN</div>
        <div class="ph-info">Back to initial position</div>
        <div class="ph-pos">{60, 83, 180, 107, 64, 0}</div>
      </div>
    </div>
  </div>
</div>

<div class="card">
  <div class="title">MANUAL SERVO CONTROL (6 sliders)</div>
  <div id="sliders"></div>
  <div class="row" style="margin-top:10px;">
    <button class="abig resetbtn" style="margin:0" onclick="resetToInit()">Reset to Initial</button>
    <button class="abig stopbtn" style="margin:0" onclick="resetTo90()">Reset to 90</button>
  </div>
</div>

<div class="card">
  <div class="title">PID TUNING (Target: 10cm)</div>
  <div class="pid-box"><label>Kp</label><input type="range" id="kp" min="1" max="20" step="0.5" value="8" oninput="document.getElementById('kpv').textContent=this.value"><span id="kpv">8.0</span></div>
  <div class="pid-box"><label>Ki</label><input type="range" id="ki" min="0" max="2" step="0.05" value="0.1" oninput="document.getElementById('kiv').textContent=this.value"><span id="kiv">0.1</span></div>
  <div class="pid-box"><label>Kd</label><input type="range" id="kd" min="0" max="5" step="0.1" value="1.0" oninput="document.getElementById('kdv').textContent=this.value"><span id="kdv">1.0</span></div>
  <button class="abig start" style="margin-top:6px" onclick="applyPID()">Apply PID</button>
</div>

<script>
var names=['Gripper(0)','Servo 1','Servo 2','Servo 3','Servo 4','Base(5)'];
var colors=['#FF5733','#33FF57','#3357FF','#F333FF','#FF33A1','#33FFF5'];
var initVals=[60,83,180,107,64,0];
var sEls=[], vEls=[];
var timers=[null,null,null,null,null,null];

function buildSliders(){
  var c=document.getElementById('sliders');
  for(var i=0;i<6;i++){
    (function(idx){
      var row=document.createElement('div');
      row.className='srow';
      var lbl=document.createElement('label');
      lbl.textContent=names[idx];
      lbl.style.color=colors[idx];
      var sl=document.createElement('input');
      sl.type='range';sl.min=0;sl.max=180;sl.value=initVals[idx];
      sl.style.accentColor=colors[idx];
      var val=document.createElement('span');
      val.textContent=initVals[idx]+'deg';
      vEls.push(val);
      sl.oninput=function(){
        val.textContent=this.value+'deg';
        var v=this.value;
        clearTimeout(timers[idx]);
        timers[idx]=setTimeout(function(){
          fetch('/servo?i='+idx+'&a='+v).catch(function(){});
        },80);
      };
      sEls.push(sl);
      row.appendChild(lbl);row.appendChild(sl);row.appendChild(val);
      c.appendChild(row);
    })(i);
  }
}
buildSliders();

function setSlider(i,v){
  sEls[i].value=v;
  vEls[i].textContent=v+'deg';
  clearTimeout(timers[i]);
  (function(idx,val){
    timers[idx]=setTimeout(function(){fetch('/servo?i='+idx+'&a='+val).catch(function(){});},80);
  })(i,v);
}

function resetToInit(){for(var i=0;i<6;i++)setSlider(i,initVals[i]);}
function resetTo90(){for(var i=0;i<6;i++)setSlider(i,90);}

function go(c){fetch('/cmd?c='+c).catch(function(){});}

function applyPID(){
  fetch('/pid?kp='+document.getElementById('kp').value+
        '&ki='+document.getElementById('ki').value+
        '&kd='+document.getElementById('kd').value)
  .then(function(){alert('PID updated');}).catch(function(){});
}

var phaseMap={'1:PICK':0,'2:INTERMEDIATE':1,'3:UNLOAD':2,'4:RETURN':3};
function setPhase(p){
  for(var i=0;i<4;i++) document.getElementById('ph'+i).className='ph';
  if(p && phaseMap[p]!==undefined)
    document.getElementById('ph'+phaseMap[p]).className='ph active';
}
function poll(){
  fetch('/status').then(function(r){return r.json();}).then(function(d){
    var dist=parseFloat(d.dist);
    var el=document.getElementById('dist');
    el.textContent=dist+' cm';
    el.className='dist '+(dist<=10?'stop':dist<=30?'slow':'ok');
    document.getElementById('warn').textContent=dist<=10?'AT GRAB DISTANCE (10cm)':dist<=30?'SLOW ZONE':'';
    document.getElementById('autoStatus').textContent=d.auto;
    setPhase(d.phase);
  }).catch(function(){});
  setTimeout(poll,300);
}
poll();
</script>
</body>
</html>
)rawhtml";

// ── Web routes ────────────────────────────────
void handleRoot(){server.send_P(200,"text/html",HTML);}

void handleCmd(){
    if(!server.hasArg("c")){server.send(200,"text/plain","ok");return;}
    String c=server.arg("c");
    if(c=="start_auto"){autoMode=true;turnActive=false;manualArmPending=false;resetPID();rotSteps=0;goTo(AS_SRCH_SETTLE,"Searching...");}
    else if(c=="stop_auto"||c=="reset_auto"){autoMode=false;autoState=AS_IDLE;turnActive=false;manualArmPending=false;mStop();resetPID();autoStatus="Idle";}
    else if(!autoMode){
        if(c=="forward")       mFwd(SPD_FULL);
        else if(c=="backward") mBwd(SPD_FULL);
        else if(c=="stop")    {mStop();turnActive=false;}
        else if(c=="left")    {mLeft();turnActive=true;turnEnd=millis()+TURN_MS;}
        else if(c=="right")   {mRight();turnActive=true;turnEnd=millis()+TURN_MS;}
        else if(c=="arm")      startManualArm();
        else if(c=="reset")   {mStop();turnActive=false;manualArmPending=false;autoStatus="Idle";}
    }
    server.send(200,"text/plain","ok");
}

void handleServo(){
    if(server.hasArg("i")&&server.hasArg("a")){
        int idx=server.arg("i").toInt();
        int ang=server.arg("a").toInt();
        if(idx>=0&&idx<6&&ang>=0&&ang<=180) smoothMove(idx,ang);
    }
    server.send(200,"text/plain","ok");
}

void handlePID(){
    if(server.hasArg("kp"))Kp=server.arg("kp").toFloat();
    if(server.hasArg("ki"))Ki=server.arg("ki").toFloat();
    if(server.hasArg("kd"))Kd=server.arg("kd").toFloat();
    resetPID();server.send(200,"text/plain","ok");
}

void handleStatus(){
    String j="{\"dist\":"+String((int)lastDist)+",\"auto\":\""+autoStatus+"\"}";
    server.send(200,"application/json",j);
}

// ── setup() ──────────────────────────────────
void setup(){
    Serial.begin(115200);
    for(int p:{ENA,ENB,IN1,IN2,IN3,IN4})pinMode(p,OUTPUT);
    mStop();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID,AP_PASS);
    WiFi.setSleep(false);
    delay(500);
    Serial.printf("\nWiFi:%s IP:%s\n",AP_SSID,WiFi.softAPIP().toString().c_str());

    server.on("/",       handleRoot);
    server.on("/cmd",    handleCmd);
    server.on("/servo",  handleServo);
    server.on("/pid",    handlePID);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("Open 192.168.4.1 in browser");

    ESP32PWM::allocateTimer(0);ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);ESP32PWM::allocateTimer(3);
    for(int i=0;i<6;i++){
        svos[i].setPeriodHertz(50);
        svos[i].attach(svPins[i],500,2400);
        delay(50);yield();
        svos[i].write(initPos[i]);
        curPos[i]=initPos[i];
        delay(300);yield();
    }
    Serial.println("Ready!");
}

// ── loop() ───────────────────────────────────
void loop(){
    server.handleClient();
    updateSonar();
    handleTurnTick();
    handleManualArmTick();
    if(autoMode)runAuto();
    vTaskDelay(1);
}
