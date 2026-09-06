// ============================================================
// us_test.ino - 초음파 3개만 떼어낸 시험 스케치 (Arduino Mega 2560)
//
// merge.ino 에서 초음파 측정과 노면 위험 판정만 남기고 IMU / FSR / 홀센서 /
// 모터를 전부 뺐다. merge.ino 를 올렸는데 "초음파 쪽이 아무 반응이 없다" 할
// 때, 그게 센서 문제인지 판정 게이트 문제인지 가르는 용도다.
//
// merge.ino 와 일부러 다르게 한 것 (전부 시험 편의용이다)
//   1) 보 레이트는 merge.ino 와 같은 9600 이다. 세 센서분 R 줄을 다 내면
//      9600 이 병목이 되어 센서당 재측정 간격이 45ms 에서 약 90ms 로
//      늘어나지만, US_STALE_INTERVAL_S(0.25s) 안쪽이라 판정은 그대로 돈다.
//      한 센서만 빠르게 보려면 US_PRINT_RAW_SENSOR 를 0/1/2 로 바꾼다.
//   2) 측정 상한을 1m -> 4m 로 늘렸다. merge.ino 는 노면만 보므로 1m 를
//      넘으면 전부 0(측정 실패)이 되는데, 책상 위에서 앞을 보게 두면
//      "센서가 죽은 것"과 구분이 안 된다. 여기서는 원시값을 그대로 본다.
//   3) 평지 기준거리 보정 진행 상황을 센서마다 알린다. merge.ino 는 세 개가
//      전부 끝나야 E,US_CAL 을 한 번에 내보내서, 하나만 물려 놓고 시험하면
//      아무것도 안 나온다.
//   4) 판정 게이트(노면 창, 보정)를 통과 못 한 이유를 매번 찍는다.
//   5) T 줄의 IMU / FSR / 홀센서 / 모터 자리는 진짜 값이 아니라 TEL_* 에
//      박아 둔 자리값이다. 초음파 필드(us_l/us_c/us_r/risk/hazard)만 실제
//      측정에서 나온다. 파이에서 볼 때 그 다섯 개만 믿을 것.
//
// 판정 로직 자체(기하, 중앙값 필터, 확정 프레임, 임계값)는 merge.ino 의
// TERRAIN_THRESHOLD_FROM_CALIBRATION 0 쪽과 같다. 속도는 IMU 가 없으므로
// SPEED_FIXED_MPS 고정값을 쓴다(홈 너비 참고값에만 들어간다).
//
// 출력
//   E,BOOT
//   E,US_BEAM,<센서>,<빔길이mm>,<전방주시mm>       부팅 시 3줄
//   E,US_EXPECT,<센서>,<기준거리mm>,<노면창 하한mm>,<상한mm>
//   R,<센서>,<원시mm>,<중앙값mm>,<편차mm|NA>,<에코us>,<실패원인>  매 측정
//        US_PRINT_RAW_SENSOR 로 어느 센서를 낼지 고른다 (기본 255 = 전부)
//        실패원인 0=정상 1=에코 안 올라옴(배선/전원) 2=에코 안 내려옴
//                 3=거리 범위 밖
//        편차 NA = 노면 창 밖. 판정에 들어가지 못한다.
//   E,US_CAL,<센서>,<기준거리>,<잡음>,<턱시작>,<턱확정>,<홈시작>   센서마다 1줄
//   E,US_ODD,<센서>,<실측mm>,<계산mm>   평지가 아닌 것을 보고 있다(보정 재시작)
//   H,<risk>,<hazard>,<센서>,<거리mm>,<너비mm>,<깊이mm>   위험 판정이 바뀔 때
//   T,3,<seq>,<pitch>,<slope>,<fsr1>,<fsr2>,<handle>,<belt>,<mode>,<pwm>,
//     <motor>,<us_l>,<us_c>,<us_r>,<risk>,<hazard>          500ms 주기
//        merge.ino 의 텔레메트리와 같은 17필드 포맷 3 이다. 파이의
//        app_3_12.py 가 그대로 파싱한다. 이 스케치에 없는 하드웨어(IMU /
//        FSR / 홀센서 / 모터) 자리는 아래 TEL_* 자리값이 들어간다.
//
// 확인 순서
//   아무 줄도 안 나온다        -> 보 레이트(9600) / 포트 / 보드 선택
//   R 줄의 실패원인이 계속 1   -> 그 센서의 배선(TRIG/ECHO/5V/GND)이나 전원
//   R 줄은 나오는데 편차가 NA  -> 센서가 노면을 안 보고 있다. 높이/각도를
//                                 실제 장착에 맞춰 아래 상수를 고칠 것
//   E,US_ODD 가 반복된다       -> 앞에 벽/책상이 있거나 장착이 설정과 다르다
//   E,US_CAL 이 나온 뒤        -> 여기서부터 H 줄과 유효한 T 줄이 나온다
// ============================================================

const unsigned long SERIAL_BAUD = 9600;   // merge.ino 와 같다

// 1 = 매 측정마다 R 줄을 낸다. 눈이 아프면 0.
#define US_PRINT_RAW 1

// R 줄을 낼 센서. 0=좌 1=중 2=우, 255=세 개 전부.
//
// 255 로 두면 9600 baud 가 병목이 된다. 초당 960바이트인데 R 줄 하나가 최대
// 28바이트라, 세 개를 다 내면 한 바퀴에 84바이트 = 약 88ms 가 송신에 든다.
// 송신 버퍼(64바이트)가 차면 Serial.print 가 빌 때까지 기다리므로, 센서당
// 재측정 간격이 45ms 에서 약 90ms 로 늘어난다.
//
// 느려지는 것 말고 깨지는 것은 없다. 90ms 는 US_STALE_INTERVAL_S(0.25s)
// 안쪽이라 판정 상태가 초기화되지 않고, 중앙값 필터와 확정 프레임도 그대로
// 돈다. 다만 2프레임 확정에 90ms 가 아니라 180ms 가 걸리므로 밀면서 시험할
// 때는 경보가 그만큼 늦게 뜬다. 세워 놓고 편차만 보는 데는 상관없다.
//
// 한 센서만 빠르게 보려면 0/1/2 로 바꾼다(그때는 45ms 주기 그대로).
#define US_PRINT_RAW_SENSOR 255
// 1 = 3점 중앙값 필터(merge.ino 와 같음). 0 = 원시값을 그대로 판정에 넣는다.
#define US_MEDIAN_FILTER 1

// ===================== 핀 =====================
const uint8_t US_COUNT = 3;
const uint8_t US_TRIG_PINS[US_COUNT] = { 22, 24, 26 };  // 좌, 중, 우
const uint8_t US_ECHO_PINS[US_COUNT] = { 23, 25, 27 };

// ===================== 측정값 =====================
const unsigned long US_PING_INTERVAL_MS = 15;    // 센서당 45ms
const unsigned long US_RISE_TIMEOUT_US = 3000;   // 에코 상승 대기 한계
// merge.ino 는 6000(약 1m). 여기서는 센서가 살아 있는지부터 봐야 하므로
// 4m 까지 열어 둔다. 노면 판정에 들어가는 창은 아래 비율이 따로 잡는다.
const unsigned long US_MAX_ECHO_US = 24000;
const uint16_t US_MIN_VALID_MM = 20;
const uint16_t US_MAX_VALID_MM = 4000;

// ===================== 장착 형상 =====================
// 실제로 단 높이/각도로 고칠 것. 이 값이 실제와 다르면 노면 창을 벗어나
// 판정이 영원히 시작되지 않는다.
float US_MOUNT_HEIGHT_M[US_COUNT] = { 0.16, 0.16, 0.16 };
float US_TILT_DEG[US_COUNT] = { 65.0, 45.0, 65.0 };   // 좌 / 중 / 우
const float US_MIN_TILT_SIN = 0.0175;
const float US_BEAM_HALF_ANGLE_DEG = 7.5;

// 노면으로 인정할 범위. 평지 기대값(height / sin(tilt)) 대비 비율.
const float US_GROUND_MIN_RATIO = 0.15;
const float US_GROUND_MAX_RATIO = 3.00;

// ===================== 판정값 (merge.ino 고정값 판과 동일) =====================
// 센서별 고정 임계값. 단위는 cm (merge.ino 와 같은 값).
// 노면 높이가 평지 기준에서 벗어난 양이지 측정 거리가 아니다.
//                                            좌     중     우
const float STEP_DANGER_FIXED_CM[US_COUNT] = { 16.0,  30.0,  12.0 };
const float HOLE_ENTER_FIXED_CM[US_COUNT]  = { 20.0,  50.0,  18.0 };
const float STEP_ENTER_RATIO = 0.5;       // 턱시작 = 턱확정 * 이 비율
const float TERRAIN_EXIT_FIXED_CM = 1.0;  // 노면 복귀

// 이 중 좌 턱 16cm / 중 턱 30cm / 중 홈 50cm 은 유효 측정창(좌우 26~530mm,
// 중앙 34~679mm) 밖이라 발동하지 않는다. R 줄의 편차가 실제로 어디까지
// 오르내리는지 보고 값을 맞추는 것이 이 스케치의 용도다.

const float WHEEL_WIDTH_M = 0.07;
const float HOLE_SAFE_GAP_M = 0.6 * WHEEL_WIDTH_M;

const uint8_t US_BASELINE_SAMPLES = 20;
const float US_BASELINE_MIN_RATIO = 0.6;
const float US_BASELINE_MAX_RATIO = 1.6;
const float US_BASELINE_MAX_NOISE_M = 0.015;

const uint8_t TERRAIN_CONFIRM_FRAMES = 2;
// 위험 조건 자체가 몇 프레임 연속으로 반복돼야 DANGER 를 내보내는지.
const uint8_t TERRAIN_DANGER_CONFIRM_FRAMES = 2;
const uint8_t HOLE_DANGER_VOTES =
    (TERRAIN_CONFIRM_FRAMES > TERRAIN_DANGER_CONFIRM_FRAMES)
        ? TERRAIN_CONFIRM_FRAMES : TERRAIN_DANGER_CONFIRM_FRAMES;
const float US_STALE_INTERVAL_S = 0.25;
const float US_MIN_INTERVAL_S = 0.01;
const unsigned long RISK_HOLD_MS = 1500;
const float SPEED_FIXED_MPS = 0.40;   // IMU 가 없으므로 고정값

// merge.ino 와 같은 주기.
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// ===================== 텔레메트리 자리값 =====================
// merge.ino 의 T 줄은 17필드 고정이라 IMU / FSR / 홀센서 / 모터 자리를 비울
// 수 없다(파이 파서가 len(p) != 17 이면 통째로 버린다). 이 스케치에는 그
// 하드웨어가 없으므로 아래 값을 그대로 채워 보낸다.
//
// mode 는 1 = SENSOR_FAULT 다. IMU 가 실제로 없으니 이게 정직한 값이다.
// 앱이 SENSOR_FAULT 를 경고로 띄워서 초음파 확인에 방해가 되면 5(FLAT)로
// 바꿔도 된다 -- 다만 그건 없는 IMU 가 정상이라고 말하는 셈이다.
const float   TEL_PITCH_DEG = 0.00;  // [3]  도
const uint8_t TEL_SLOPE     = 0;     // [4]  FLAT
const int     TEL_FSR1      = 0;     // [5]
const int     TEL_FSR2      = 0;     // [6]
const uint8_t TEL_HANDLE    = 0;     // [7]  놓음
const uint8_t TEL_BELT      = 0;     // [8]  풀림
const uint8_t TEL_MODE      = 1;     // [9]  SENSOR_FAULT
const uint8_t TEL_PWM       = 0;     // [10]
const uint8_t TEL_MOTOR     = 0;     // [11] COAST

// ===================== 형 =====================
enum RiskLevel : uint8_t { RISK_SAFE = 0, RISK_CAUTION = 1, RISK_DANGER = 2 };
enum HazardCause : uint8_t { HAZARD_NONE = 0, HAZARD_STEP = 1, HAZARD_HOLE = 2 };
enum TerrainState : uint8_t { TERRAIN_IDLE = 0, TERRAIN_HOLE = 1, TERRAIN_STEP = 2 };

struct DangerDetector {
  TerrainState state;
  float holeWidthM;
  float holeDepthM;
  float stepPeakM;
  uint8_t holeVotes;
  uint8_t stepVotes;
  uint8_t dangerVotes;
  unsigned long lastSampleAtMs;
  uint16_t recentMm[3];
  uint8_t recentCount;
  float pendingIntervalS;
  float baselineSumM;
  float baselineSumSqM;
  uint8_t baselineCount;
  bool calibrated;
  float noiseM;
  float stepEnterM;
  float stepDangerM;
  float holeEnterM;
  float exitM;
  RiskLevel risk;
  HazardCause hazard;
  unsigned long riskAtMs;
  float riskWidthM;
  float riskDepthM;
};

// ===================== 상태 =====================
uint8_t usIndex = 0;
unsigned long usPingStartedAtMs = 0;
uint16_t usDistanceMm[US_COUNT] = { 0, 0, 0 };
uint16_t usLastPulseUs[US_COUNT] = { 0, 0, 0 };
uint8_t usLastFail[US_COUNT] = { 0, 0, 0 };

float usSinTilt[US_COUNT];
float usExpectedFlatM[US_COUNT];
float usBeamFootprintM[US_COUNT];
float usLookAheadM[US_COUNT];
float usBaselineM[US_COUNT];
float usBaselineComputedM[US_COUNT];
float usHeightGainM[US_COUNT];

DangerDetector detectors[US_COUNT];

RiskLevel overallRisk = RISK_SAFE;
HazardCause overallHazard = HAZARD_NONE;
uint8_t overallRiskSensor = 255;
RiskLevel reportedRisk = RISK_SAFE;
HazardCause reportedHazard = HAZARD_NONE;

unsigned long lastTelemetryAt = 0;
uint8_t telemetrySeq = 0;

void resetDetector(uint8_t index);
void setupTerrainDetectors();
uint16_t pulseToMm(unsigned long pulseUs);
float terrainDeviationM(uint8_t index, uint16_t distanceMm);
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now);
void updateOverallRisk(unsigned long now);
void outputHazardLine();
void outputTelemetry();
void measureOne(uint8_t index);

void setup() {
  Serial.begin(SERIAL_BAUD);
  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(US_TRIG_PINS[i], OUTPUT);
    digitalWrite(US_TRIG_PINS[i], LOW);
    pinMode(US_ECHO_PINS[i], INPUT);
  }

  setupTerrainDetectors();

  Serial.println(F("E,BOOT"));
  for (uint8_t i = 0; i < US_COUNT; i++) {
    Serial.print(F("E,US_BEAM,"));
    Serial.print(i);
    Serial.print(',');
    Serial.print((int)(usBeamFootprintM[i] * 1000.0));
    Serial.print(',');
    Serial.println((int)(usLookAheadM[i] * 1000.0));

    // 이 센서가 판정에 넣어 주는 거리 창. R 줄의 원시값이 이 밖이면
    // 편차가 NA 로 나가고 보정이 시작되지 않는다.
    Serial.print(F("E,US_EXPECT,"));
    Serial.print(i);
    Serial.print(',');
    Serial.print((int)(usBaselineComputedM[i] * 1000.0));
    Serial.print(',');
    Serial.print((int)(usExpectedFlatM[i] * US_GROUND_MIN_RATIO * 1000.0));
    Serial.print(',');
    Serial.println((int)(usExpectedFlatM[i] * US_GROUND_MAX_RATIO * 1000.0));
  }

  unsigned long now = millis();
  usPingStartedAtMs = now;
  lastTelemetryAt = now;
  for (uint8_t i = 0; i < US_COUNT; i++) {
    detectors[i].lastSampleAtMs = now;
    detectors[i].riskAtMs = now;
  }
  Serial.println(F("E,READY"));
}

void loop() {
  unsigned long now = millis();

  if (now - usPingStartedAtMs >= US_PING_INTERVAL_MS) {
    usPingStartedAtMs = now;
    measureOne(usIndex);
    runTerrainDetector(usIndex, usDistanceMm[usIndex], millis());
    usIndex++;
    if (usIndex >= US_COUNT) usIndex = 0;
  }

  updateOverallRisk(now);

  if (overallRisk != reportedRisk || overallHazard != reportedHazard) {
    outputHazardLine();
  }

  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAt = now;
    outputTelemetry();
  }
}

// ===================== 측정 =====================
// merge.ino 와 같이 한 번에 한 센서만 쏘고 에코를 끝까지 잰다.
void measureOne(uint8_t index) {
  uint8_t trigPin = US_TRIG_PINS[index];
  uint8_t echoPin = US_ECHO_PINS[index];

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long trigAt = micros();
  while (digitalRead(echoPin) == LOW) {
    if (micros() - trigAt > US_RISE_TIMEOUT_US) {
      usDistanceMm[index] = 0;
      usLastPulseUs[index] = 0;
      usLastFail[index] = 1;   // 에코가 올라오지 않음: 배선/전원
      return;
    }
  }

  unsigned long echoStart = micros();
  while (digitalRead(echoPin) == HIGH) {
    if (micros() - echoStart > US_MAX_ECHO_US) {
      usDistanceMm[index] = 0;
      usLastPulseUs[index] = (uint16_t)US_MAX_ECHO_US;
      usLastFail[index] = 2;   // 에코가 내려오지 않음
      return;
    }
  }

  unsigned long pulseUs = micros() - echoStart;
  usLastPulseUs[index] = (pulseUs > 65000) ? 65000 : (uint16_t)pulseUs;
  usDistanceMm[index] = pulseToMm(pulseUs);
  usLastFail[index] = (usDistanceMm[index] == 0) ? 3 : 0;
}

uint16_t pulseToMm(unsigned long pulseUs) {
  if (pulseUs == 0 || pulseUs > US_MAX_ECHO_US) return 0;
  unsigned long mm = (pulseUs * 343UL) / 2000UL;
  if (mm < US_MIN_VALID_MM || mm > US_MAX_VALID_MM) return 0;
  return (uint16_t)mm;
}

// ===================== 기하 =====================
void setupTerrainDetectors() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    float s = sin(US_TILT_DEG[i] * PI / 180.0);
    if (s < US_MIN_TILT_SIN) s = US_MIN_TILT_SIN;
    usSinTilt[i] = s;
    usExpectedFlatM[i] = US_MOUNT_HEIGHT_M[i] / s;

    float nearDeg = US_TILT_DEG[i] + US_BEAM_HALF_ANGLE_DEG;
    float farDeg = US_TILT_DEG[i] - US_BEAM_HALF_ANGLE_DEG;
    float nearM = US_MOUNT_HEIGHT_M[i] / tan(nearDeg * PI / 180.0);
    if (farDeg < 1.0) {
      usBeamFootprintM[i] = 10.0;
    } else {
      usBeamFootprintM[i] = US_MOUNT_HEIGHT_M[i] / tan(farDeg * PI / 180.0) - nearM;
    }
    usLookAheadM[i] = US_MOUNT_HEIGHT_M[i] / tan(US_TILT_DEG[i] * PI / 180.0);

    float nearSin = sin(nearDeg * PI / 180.0);
    if (nearSin < US_MIN_TILT_SIN) nearSin = US_MIN_TILT_SIN;
    usBaselineM[i] = US_MOUNT_HEIGHT_M[i] / nearSin;
    usBaselineComputedM[i] = usBaselineM[i];
    usHeightGainM[i] = nearSin;

    resetDetector(i);
  }
}

void resetDetector(uint8_t index) {
  DangerDetector &d = detectors[index];
  d.state = TERRAIN_IDLE;
  d.holeWidthM = 0.0;
  d.holeDepthM = 0.0;
  d.stepPeakM = 0.0;
  d.holeVotes = 0;
  d.stepVotes = 0;
  d.dangerVotes = 0;
  d.pendingIntervalS = 0.0;
  d.recentCount = 0;
  d.recentMm[0] = d.recentMm[1] = d.recentMm[2] = 0;
  d.baselineSumM = 0.0;
  d.baselineSumSqM = 0.0;
  d.baselineCount = 0;
  d.calibrated = false;
  d.noiseM = 0.0;
  d.stepDangerM = STEP_DANGER_FIXED_CM[index] / 100.0;
  d.stepEnterM = d.stepDangerM * STEP_ENTER_RATIO;
  d.holeEnterM = HOLE_ENTER_FIXED_CM[index] / 100.0;
  d.exitM = TERRAIN_EXIT_FIXED_CM / 100.0;
  // 복귀 임계가 턱시작보다 높으면 상태에 들어가자마자 빠져나온다.
  if (d.exitM > d.stepEnterM * 0.5) d.exitM = d.stepEnterM * 0.5;
  d.lastSampleAtMs = millis();
  d.risk = RISK_SAFE;
  d.hazard = HAZARD_NONE;
  d.riskAtMs = d.lastSampleAtMs;
  d.riskWidthM = 0.0;
  d.riskDepthM = 0.0;
}

float terrainDeviationM(uint8_t index, uint16_t distanceMm) {
  if (distanceMm == 0) return NAN;
  float distanceM = distanceMm / 1000.0;
  if (distanceM < usExpectedFlatM[index] * US_GROUND_MIN_RATIO
      || distanceM > usExpectedFlatM[index] * US_GROUND_MAX_RATIO) {
    return NAN;
  }
  return (distanceM - usBaselineM[index]) * usHeightGainM[index];
}

static uint16_t medianOf3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// ===================== 판정 =====================
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now) {
  DangerDetector &d = detectors[index];

  float interval = (now - d.lastSampleAtMs) / 1000.0 + d.pendingIntervalS;
  d.lastSampleAtMs = now;
  d.pendingIntervalS = 0.0;

  if (interval > US_STALE_INTERVAL_S) {
    d.state = TERRAIN_IDLE;
    d.holeWidthM = 0.0;
    d.holeDepthM = 0.0;
    d.stepPeakM = 0.0;
    d.dangerVotes = 0;
    d.recentCount = 0;
    interval = US_STALE_INTERVAL_S;
  }
  if (interval < US_MIN_INTERVAL_S) interval = US_MIN_INTERVAL_S;

  uint16_t rawMm = distanceMm;

#if US_MEDIAN_FILTER
  if (distanceMm != 0) {
    d.recentMm[2] = d.recentMm[1];
    d.recentMm[1] = d.recentMm[0];
    d.recentMm[0] = distanceMm;
    if (d.recentCount < 3) d.recentCount++;
    if (d.recentCount >= 3) {
      distanceMm = medianOf3(d.recentMm[0], d.recentMm[1], d.recentMm[2]);
    }
  }
#endif

  float dev = terrainDeviationM(index, distanceMm);

#if US_PRINT_RAW
  if (US_PRINT_RAW_SENSOR == 255 || index == US_PRINT_RAW_SENSOR) {
    Serial.print(F("R,"));
    Serial.print(index);
    Serial.print(',');
    Serial.print(rawMm);
    Serial.print(',');
    Serial.print(distanceMm);
    Serial.print(',');
    if (isnan(dev)) Serial.print(F("NA"));
    else Serial.print((int)(dev * 1000.0));
    Serial.print(',');
    Serial.print(usLastPulseUs[index]);
    Serial.print(',');
    Serial.println(usLastFail[index]);
  }
#endif

  if (isnan(dev)) {
    d.pendingIntervalS = interval;
    return;
  }

  // 평지 기준거리 보정
  if (!d.calibrated) {
    float m = distanceMm / 1000.0;
    d.baselineSumM += m;
    d.baselineSumSqM += m * m;
    d.baselineCount++;
    if (d.baselineCount >= US_BASELINE_SAMPLES) {
      float mean = d.baselineSumM / US_BASELINE_SAMPLES;
      float var = d.baselineSumSqM / US_BASELINE_SAMPLES - mean * mean;
      if (var < 0.0) var = 0.0;

      float lo = usBaselineComputedM[index] * US_BASELINE_MIN_RATIO;
      float hi = usBaselineComputedM[index] * US_BASELINE_MAX_RATIO;
      if (mean < lo || mean > hi) {
        Serial.print(F("E,US_ODD,"));
        Serial.print(index);
        Serial.print(',');
        Serial.print((int)(mean * 1000.0));
        Serial.print(',');
        Serial.println((int)(usBaselineComputedM[index] * 1000.0));
        d.baselineSumM = 0.0;
        d.baselineSumSqM = 0.0;
        d.baselineCount = 0;
        return;
      }

      usBaselineM[index] = mean;
      float effSin = US_MOUNT_HEIGHT_M[index] / mean;
      if (effSin > 1.0) effSin = 1.0;
      if (effSin < 0.05) effSin = 0.05;
      usHeightGainM[index] = effSin;
      d.noiseM = sqrt(var) * usHeightGainM[index];
      d.calibrated = true;

      // merge.ino 는 세 센서가 전부 끝나야 한 번에 냈다. 여기서는 센서마다
      // 낸다. 하나만 물려 놓고 시험해도 이 줄이 보여야 정상이다.
      Serial.print(F("E,US_CAL,"));
      Serial.print(index);
      Serial.print(',');
      Serial.print((int)(usBaselineM[index] * 1000.0));
      Serial.print(',');
      Serial.print((int)(d.noiseM * 1000.0));
      Serial.print(',');
      Serial.print((int)(d.stepEnterM * 1000.0));
      Serial.print(',');
      Serial.print((int)(d.stepDangerM * 1000.0));
      Serial.print(',');
      Serial.println((int)(d.holeEnterM * 1000.0));

      if (d.noiseM > US_BASELINE_MAX_NOISE_M) {
        Serial.print(F("E,US_NOISY,"));
        Serial.print(index);
        Serial.print(',');
        Serial.println((int)(d.noiseM * 1000.0));
      }
    }
    return;
  }

  RiskLevel risk = RISK_SAFE;
  HazardCause hazard = HAZARD_NONE;
  float eventWidthM = 0.0;
  float eventDepthM = 0.0;

  switch (d.state) {
    case TERRAIN_IDLE:
      if (dev > d.holeEnterM) {
        if (d.holeVotes < 255) d.holeVotes++;
        d.stepVotes = 0;
      } else if (-dev > d.stepEnterM) {
        if (d.stepVotes < 255) d.stepVotes++;
        d.holeVotes = 0;
      } else {
        d.holeVotes = 0;
        d.stepVotes = 0;
      }

      if (d.holeVotes >= HOLE_DANGER_VOTES) {
        d.state = TERRAIN_HOLE;
        d.holeDepthM = dev;
        d.holeWidthM = usBeamFootprintM[index];
        risk = RISK_DANGER;
        hazard = HAZARD_HOLE;
        eventWidthM = d.holeWidthM;
        eventDepthM = d.holeDepthM;
        d.holeVotes = 0;
      } else if (d.stepVotes >= TERRAIN_CONFIRM_FRAMES) {
        d.state = TERRAIN_STEP;
        d.stepPeakM = -dev;
        d.stepVotes = 0;
      }
      break;

    case TERRAIN_HOLE:
      d.holeWidthM += SPEED_FIXED_MPS * interval;
      if (dev > d.holeDepthM) d.holeDepthM = dev;
      if (dev < d.exitM) {
        d.state = TERRAIN_IDLE;
        d.holeWidthM = 0.0;
        d.holeDepthM = 0.0;
        d.dangerVotes = 0;
      }
      break;

    case TERRAIN_STEP:
      if (-dev > d.stepPeakM) d.stepPeakM = -dev;

      // 최대값은 한 번 올라가면 내려오지 않아서 헛에코 하나로 확정된다.
      // 이번 프레임의 편차가 임계를 연속으로 넘는지로 센다.
      if (-dev > d.stepDangerM) {
        if (d.dangerVotes < 255) d.dangerVotes++;
      } else {
        d.dangerVotes = 0;
      }

      if (d.dangerVotes >= TERRAIN_DANGER_CONFIRM_FRAMES) {
        risk = RISK_DANGER;
        hazard = HAZARD_STEP;
        eventDepthM = d.stepPeakM;
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
        d.dangerVotes = 0;
      } else if (-dev < d.exitM) {
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
        d.dangerVotes = 0;
      }
      break;
  }

  if (risk > d.risk) {
    d.risk = risk;
    d.hazard = hazard;
    d.riskAtMs = now;
    d.riskWidthM = eventWidthM;
    d.riskDepthM = eventDepthM;
  } else if (risk != RISK_SAFE && risk == d.risk) {
    d.riskAtMs = now;
  }
}

void updateOverallRisk(unsigned long now) {
  RiskLevel best = RISK_SAFE;
  HazardCause bestHazard = HAZARD_NONE;
  uint8_t bestSensor = 255;

  for (uint8_t i = 0; i < US_COUNT; i++) {
    if (detectors[i].risk != RISK_SAFE
        && now - detectors[i].riskAtMs >= RISK_HOLD_MS) {
      detectors[i].risk = RISK_SAFE;
      detectors[i].hazard = HAZARD_NONE;
    }
    if (detectors[i].risk > best) {
      best = detectors[i].risk;
      bestHazard = detectors[i].hazard;
      bestSensor = i;
    }
  }

  overallRisk = best;
  overallHazard = bestHazard;
  overallRiskSensor = bestSensor;
}

void outputHazardLine() {
  uint8_t s = overallRiskSensor;
  uint16_t distanceMm = (s < US_COUNT) ? usDistanceMm[s] : 0;
  uint16_t widthMm = (s < US_COUNT) ? (uint16_t)(detectors[s].riskWidthM * 1000.0) : 0;
  uint16_t depthMm = (s < US_COUNT) ? (uint16_t)(detectors[s].riskDepthM * 1000.0) : 0;

  Serial.print(F("H,"));
  Serial.print((uint8_t)overallRisk);
  Serial.print(',');
  Serial.print((uint8_t)overallHazard);
  Serial.print(',');
  Serial.print(s);
  Serial.print(',');
  Serial.print(distanceMm);
  Serial.print(',');
  Serial.print(widthMm);
  Serial.print(',');
  Serial.println(depthMm);

  reportedRisk = overallRisk;
  reportedHazard = overallHazard;
}

// merge.ino 의 outputTelemetry 와 같은 고정 순서 17필드 CSV.
// 초음파 필드만 실제 측정에서 나오고 나머지는 TEL_* 자리값이다.
void outputTelemetry() {
  Serial.print(F("T,3,"));
  Serial.print(telemetrySeq);
  Serial.print(',');
  Serial.print(TEL_PITCH_DEG, 2);
  Serial.print(',');
  Serial.print(TEL_SLOPE);
  Serial.print(',');
  Serial.print(TEL_FSR1);
  Serial.print(',');
  Serial.print(TEL_FSR2);
  Serial.print(',');
  Serial.print(TEL_HANDLE);
  Serial.print(',');
  Serial.print(TEL_BELT);
  Serial.print(',');
  Serial.print(TEL_MODE);
  Serial.print(',');
  Serial.print(TEL_PWM);
  Serial.print(',');
  Serial.print(TEL_MOTOR);
  Serial.print(',');
  Serial.print(usDistanceMm[0]);
  Serial.print(',');
  Serial.print(usDistanceMm[1]);
  Serial.print(',');
  Serial.print(usDistanceMm[2]);
  Serial.print(',');
  Serial.print((uint8_t)overallRisk);
  Serial.print(',');
  Serial.println((uint8_t)overallHazard);

  telemetrySeq++;

  // 텔레메트리로 나간 값이 곧 앱이 아는 최신 상태다.
  reportedRisk = overallRisk;
  reportedHazard = overallHazard;
}
