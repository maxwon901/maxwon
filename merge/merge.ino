#include <Wire.h>
#include <avr/wdt.h>

// ============================================================
// merge.ino - 지능형 안전 유모차 통합 펌웨어 (Arduino Mega 2560)
//
// 통합 대상
//   stroller_control.ino : IMU + 홀 + FSR + ZS-X11H 좌/우 모터 제어
//   handle_sensor.ino    : FSR 2개(A0/A1) 손잡이 감지
//   detect.py            : HC-SR04 3개(좌/중/우) 노면 위험(턱/홈/계단) 판정
//
// 동작 우선순위
//  1) IMU 이상 / 손잡이 놓음 / 내리막 -> 즉시 전자제동
//  2) (옵션) 전방 장애물 근접          -> 즉시 전자제동
//  3) 오르막 + 손잡이 잡음             -> 경사에 따라 PWM 자동 보조
//  4) 평지 / 경사 불확실               -> 감속 후 출력 OFF(Coast)
//  5) 안전벨트 홀센서                  -> 경고 표시만, 모터에는 미개입
//  6) 노면 위험(턱/홈/계단)            -> 위험도/위험원인만 앱으로 전송
//
// 모터 드라이버: ZS-X11H V1 (3상 BLDC, 300W 허브모터)
//   제어선 3개  PWM(속도) / DIR(방향, 액티브 로우) / BRAKE(제동, 액티브 하이)
//     전진   : BRAKE=LOW, DIR=전진레벨, PWM=analogWrite(속도)
//     제동   : PWM=LOW,  BRAKE=HIGH
//     코스트 : PWM=LOW,  BRAKE=LOW
//
// 주의: BRAKE가 액티브 하이라서 MCU가 리셋되거나 죽어 있는 동안에는
// BRAKE 선이 플로팅이 된다. 기판(main_circuit)의 R3/R4는 이 선을 GND로
// 내리는 10k **풀다운**이다 - 즉 MCU가 없으면 제동은 "해제"가 기본이다.
// 풀업이었다면 전원만 들어오고 MCU가 안 뜬 상태에서 바퀴가 잠겨 유모차를
// 밀 수 없게 되므로, 밀어서 쓰는 기기에서는 풀다운이 맞다.
// 대신 주행 중 MCU가 행에 걸려도 자동 제동은 걸리지 않으니, 그 보호는
// 워치독(wdt_enable)으로 리셋을 걸어 setup()의 제동 초기화로 복귀시킨다.
//
// 주의: 전자제동은 소형 모의실험용이며 실제 유모차의 기계식 안전
// 브레이크를 대신할 수 없다.
//
// ------------------------------------------------------------
// 노면 위험 판정 (detect.py의 DangerDetector 이식)
// ------------------------------------------------------------
// 파이썬 원본은 라즈베리파이가 초음파 3개를 읽고, 아두이노가 시리얼
// (115200)로 보내주는 IMU 속도(speed)를 받아 홈의 너비를 적분했다.
// 여기서는 판정 자체가 아두이노 안으로 들어왔으므로 시리얼로 속도를
// 주고받을 필요가 없다. 속도는 같은 보드의 MPU6050에서 바로 얻는다
// (updateSpeedEstimate 참고). 따라서 원본의 115200 시리얼 링크는 없다.
//
// 센서마다 독립된 DangerDetector 상태를 갖고, 그 센서의 새 측정이
// 끝난 순간(finishPing)에 그 센서의 직전 측정과 비교해 한 프레임을
// 진행시킨다. actual_interval은 그 센서의 직전 측정으로부터의 실제
// 경과 시간이다(라운드로빈이므로 센서당 약 US_PING_INTERVAL_MS * 3).
//
// 판정 흐름(원본 predict()와 동일한 순서)
//   첫 측정 / 측정 실패      -> 값만 저장하고 SAFE
//   in_hole == false
//     |delta| < ramp        -> 경사로. SAFE
//     delta < -sudden       -> 턱(가까워짐). DANGER, STEP
//     delta > +sudden       -> 홈 진입. SAFE, hole_depth=delta,
//                              hole_width = speed * interval
//     그 밖(임계값 사이)     -> SAFE
//   in_hole == true
//     hole_width += speed * interval
//     hole_width >= safe_gap            -> DANGER, STAIR (상태 해제)
//     0.5*safe_gap <= w < safe_gap      -> CAUTION, WIDE_HOLE
//     그 밖
//       -delta > depth + tol            -> DANGER, EXIT_STEP (상태 해제)
//       -delta > depth - tol            -> 원래 높이로 복귀 (상태 해제)
//       그 밖                            -> depth = max(0, depth + delta)
//
// safe_gap = 0.6 * 앞바퀴 지름, tolerance = 0.3 * 앞바퀴 지름 도 원본 그대로다.
// 세 값 모두 실험적으로 맞춰야 하는 값이라 아래 상수로 빼 두었다.
//
// 센서 장착 높이/각도
//   초음파를 노면 쪽으로 비스듬히 달면 측정값은 빗변 거리라서 센서마다
//   기준이 다르다. 그래서 원본의 '거리' 대신 수직 낙차
//       drop = 측정거리 * sin(설치 각도)
//   를 판정에 넣는다. 평지에서 drop은 설치 높이와 같고, delta는 그대로
//   노면의 높이 변화(m)가 되어 원본 임계값(m)을 그대로 쓸 수 있다.
//   높이/각도는 US_MOUNT_HEIGHT_M[], US_TILT_DEG[]에서 센서별로 바꾼다.
//
// ------------------------------------------------------------
// 설치값과 판정 방식의 근거 (시뮬레이션)
// ------------------------------------------------------------
// 이 파일의 판정 코드를 그대로 떼어내 PC에서 돌린 시뮬레이션으로 정했다
// (sim/ 폴더). HC-SR04의 빔을 원뿔로 모델링하고, 바퀴가 자기 지름보다 좁은
// 홈을 다리처럼 건너가는 것과 경사로에서 차체가 함께 기우는 것까지 넣었다.
//
// 검출률만 세면 '바퀴가 이미 지나간 뒤에 뜬 경보'가 성공으로 잡힌다.
// 그래서 경보가 바퀴 도착보다 얼마나 앞서 뜨는지(선행거리)를 지표로 썼다.
// 높이 16cm, 0.3~0.5 m/s, 지형당 120회 기준:
//
//   각도   빔 띠   전방주시   6cm 턱      10cm 턱     15cm 단차   8cm 홈
//    45도   86mm   160mm     +107mm      +74mm       +112mm      늦음
//                            119/120     120/120     120/120     65/120
//    65도   51mm    75mm     -346mm      -265mm      +48mm       +40mm
//                             29/120      24/120     120/120    118/120
//
// 얕은 각도는 턱과 단차를 미리 잡고(0.4 m/s에서 약 0.27초), 급한 각도는
// 빔 띠가 좁아 홈을 잡는다. 한 각도로 둘 다 안 되므로 센서별로 나눴다.
//   중앙 45도 : 턱과 단차. 연석이나 계단은 가로로 길어 하나로 충분하다.
//   좌우 65도 : 홈. 홈은 국소적이라 바퀴가 지나갈 경로에서 봐야 한다.
//
// 판정 방식도 바꿨다. 원본 detect.py는 이전 프레임과의 차이(delta)를 봤는데,
// 초음파가 빔 안의 최단 거리를 돌려준다는 성질을 쓰면 평지 기준거리 하나로
// 종류를 바로 가를 수 있다. 같은 시나리오에서 (높이 12cm, 70도)
//     원본 델타 방식 : 미탐 208 / 과잉  99
//     절대 기준거리  : 미탐  74 / 과잉 173
// 6cm 턱 미탐이 112/125에서 1/125로 줄었다. 턱 판정에는 속도 추정이 전혀
// 필요 없어서, 오차가 가장 컸던 입력이 판정에서 빠진다.
//
// 남는 한계는 홈이다. 5cm 안팎의 홈은 어느 각도에서도 28~32/120에 그친다.
// 빔 띠가 45mm라 5cm 홈은 한 프레임만 보이고, 그 한 프레임은 헛에코와
// 구분할 방법이 없다. 앞바퀴 7cm에는 위험한 크기지만 이 센서의 물리적
// 한계다. 여기서는 홈을 접고 턱과 단차에 맞췄다.
//
// ------------------------------------------------------------
// 임계값 스위치 (TERRAIN_THRESHOLD_FROM_CALIBRATION)
// ------------------------------------------------------------
// 노면 판정 임계값을 어디서 얻을지 아래 스위치 한 줄로 고른다.
//     0  <- 현재 값. 아래 STEP_DANGER_FIXED_CM / HOLE_ENTER_FIXED_CM 에
//           센서별로 적은 값을 그대로 쓴다(좌/중/우 턱확정 16/30/12cm,
//           홈시작 20/50/18cm). 부팅 보정은 평지 기준거리(d0)만 잡는다.
//     1     부팅 보정에서 잡음까지 재서 임계값을 그 배수로 만들고, 빔
//           기하에서 나오는 하한(홈의 점프 상한, 경사로가 만드는 겉보기
//           턱)을 함께 씌운다.
//
// 시뮬레이션 비교 (중앙 45도, 평지·요철·경사 400회 / 턱·단차 500회)
//   0 (고정값) : 오경보 21/400,  턱·단차 미탐 12/500
//   1 (보정)   : 오경보 13/400,  턱·단차 미탐 21/500
// 고정값 쪽이 임계가 낮아 더 민감하다. 놓치는 것이 적은 대신 오경보가 는다.
//
// 시동 위치를 평지로 잡기 어렵거나 임계값을 손으로 못 박고 싶으면 0을,
// 센서 개체차·노면 재질·장착 상태를 따라가게 하고 싶으면 1을 쓴다.
// 어느 쪽이든 평지 기준거리 보정은 필요하다(E,US_CAL 이 나와야 판정이
// 시작된다).
//
// ------------------------------------------------------------
// 시리얼 텔레메트리 포맷 (라즈베리파이/앱 파싱용)
// ------------------------------------------------------------
// 고정 순서 CSV. 파이에서는 split(',') 한 번이면 끝난다.
//
//   T,3,<seq>,<pitch>,<slope>,<fsr1>,<fsr2>,<handle>,<belt>,<mode>,<pwm>,
//     <motor>,<us_l>,<us_c>,<us_r>,<risk>,<hazard>
//   0 1   2      3        4       5      6      7        8      9     10
//     11      12     13     14     15      16
//
//   [0] "T"      텔레메트리 줄 표식 (이 글자로 시작하지 않는 줄은 무시)
//   [1] 포맷 버전 (현재 3). 필드를 늘리면 반드시 올린다.
//   [2] seq      0-255 순환. 줄 유실 감지용
//   [3] pitch    도, 소수 2자리 (+ 오르막 / - 내리막)
//   [4] slope    0=FLAT 1=UP 2=DOWN 3=UNCERTAIN
//   [5] fsr1     A0 원시값 0-1023
//   [6] fsr2     A1 원시값 0-1023
//   [7] handle   0=놓음 1=잡음
//   [8] belt     0=풀림 1=체결
//   [9] mode     0=INITIALIZING 1=SENSOR_FAULT 2=HANDLE_RELEASED
//                3=DOWNHILL_BRAKE 4=UPHILL_ASSIST 5=FLAT 6=UNCERTAIN
//                7=OBSTACLE_BRAKE
//  [10] pwm      현재 출력 0-255
//  [11] motor    0=COAST 1=FORWARD 2=BRAKE
//  [12] us_l     좌 초음파 거리 mm (노면용이라 20-1000mm), 0 = 측정 실패
//  [13] us_c     중앙
//  [14] us_r     우
//  [15] risk     노면 위험도 0=SAFE 1=CAUTION 2=DANGER (세 센서 중 최대)
//  [16] hazard   위험원인 0=NONE 1=STEP(턱) 2=HOLE(홈)
//
// 속도는 홈 너비를 적분하는 데만 쓰고 밖으로 내보내지 않는다.
//
// 위험도/위험원인이 바뀌는 순간에는 500ms 주기를 기다리지 않고 즉시
// 한 줄을 더 내보낸다(앱 경보용). 텔레메트리와 같은 코드값을 쓴다:
//
//   H,<risk>,<hazard>,<sensor>,<dist_mm>,<width_mm>,<depth_mm>
//     sensor   위험을 만든 센서 0=좌 1=중 2=우, 255=없음(위험 해제)
//     dist_mm  그 순간 그 센서의 측정 거리
//     width_mm 판정 당시 누적된 홈의 너비 (턱이면 0)
//     depth_mm 홈이면 홈의 깊이, 턱이면 노면이 올라온 높이
//
// 이벤트/오류는 별도 줄로 나간다:  E,<사유>
//   E,BOOT / E,RST,<원인> / E,CAL,<남은초> / E,IMU_READY / E,READY / E,IMU_LOST
//   E,RST의 원인은 WDT / BROWNOUT / EXTERNAL / POWERON / UNKNOWN 중 하나.
//     WDT면 직전 주행에서 loop()가 멈춰 워치독이 칩을 리셋한 것이다.
//   E,IMU_FAIL,<step>,<who_am_i>,<보정샘플수>  IMU 기동 실패. step 뜻은
//     reportImuFailure() 위의 표를 볼 것 (1=응답없음 ... 8=보정샘플부족).
//   E,I2C_LINE,SDA,<ext>,<pu>,SCL,<ext>,<pu>   SDA/SCL 선 자체의 상태.
//     ext=0이면 외부 풀업 없음(모듈 미연결/전원없음), pu=0이면 GND 단락.
//   E,I2C,<주소...> 또는 E,I2C,NONE   부팅 때 실패하면 한 번 버스를 훑는다.
//   IMU가 실패해도 2초마다 다시 붙여본다. 붙으면 E,IMU_READY가 다시 나온다.
//   E,ACC,<ax>,<ay>,<az>  부팅 보정 직후의 원시 가속도. 축 매핑 확인용으로
//     평지에서 |값|이 16384에 가까운 축이 수직축(ACC_VERT)이어야 한다.
//   E,PITCH0,<도>  보정에서 잡은 장착 오프셋. 정상이면 ±10도 안쪽이다.
//   E,IMU_AXIS,<도>  그 오프셋이 30도를 넘었다. 전후축에 중력이 실려 있어
//     경사 판정이 ±90도에서 접힌다(양쪽 다 같은 방향으로 나온다).
//     ACC_FORWARD / ACC_VERT 매핑을 볼 것.
//   E,US_BEAM,<센서>,<빔길이mm>,<전방주시mm>  센서별 빔 기하 (부팅 시 3줄)
//   E,US_EXPECT,<센서>,<기준거리mm>,<노면창 하한mm>,<상한mm>  부팅 시 3줄
//                          측정값이 이 창 밖이면 보정도 판정도 시작되지 않는다
//   E,US_CAL,<센서>,<기준거리>,<잡음>,<턱시작>,<턱확정>,<홈시작>  단위 mm
//                          그 센서의 보정이 끝나는 순간 1줄. 센서마다 따로
//                          나오므로, 한 센서가 막혀도 나머지는 보인다.
//   E,US_ODD,<센서>,<실측mm>,<계산mm>  평지가 아닌 것을 보고 있다.
//                          보정을 버리고 다시 잰다. 그 센서는 이 줄이 멈추고
//                          E,US_CAL이 나올 때까지 판정에 참여하지 않는다.
//   E,US_NOISY,<센서>,<잡음mm>  보정 구간이 이미 심하게 흔들렸다. 기준거리도
//                          임계값도 믿을 게 못 되니 측정을 먼저 볼 것.
//
// US_RAW_DEBUG를 1로 두면 진단용 줄이 추가로 나간다 (기본 0).
//   R,<센서>,<원시mm>,<중앙값mm>,<편차mm>,<에코us>,<실패원인>  매 측정
//                          US_RAW_DEBUG_SENSOR 로 센서를 고른다 (255=전부)마다
//
// HUMAN_READABLE_LOG를 1로 바꾸면 '#'로 시작하는 사람이 읽는 줄이 하나 더
// 나간다(기본 0, 현장 확인용). 아래 파서는 어차피 무시한다.
//
// 파이 파서 예시:
//   SLOPE  = ("FLAT","UP","DOWN","UNCERTAIN")
//   MOTOR  = ("COAST","FORWARD","BRAKE")
//   RISK   = ("SAFE","CAUTION","DANGER")
//   HAZARD = ("NONE","STEP","HOLE")
//   def parse_tel(line):
//       p = line.strip().split(',')
//       if len(p) != 17 or p[0] != 'T' or p[1] != '3':
//           return None
//       return {"seq": int(p[2]),   "pitch": float(p[3]),
//               "slope": SLOPE[int(p[4])],
//               "fsr1": int(p[5]),  "fsr2": int(p[6]),
//               "handle": int(p[7]),"belt": int(p[8]),
//               "mode": int(p[9]),  "pwm": int(p[10]),
//               "motor": MOTOR[int(p[11])],
//               "us_l": int(p[12]), "us_c": int(p[13]), "us_r": int(p[14]),
//               "risk": RISK[int(p[15])], "hazard": HAZARD[int(p[16])]}
// ============================================================

// 0으로 바꾸면 센서 판단과 Serial 출력만 하고 모터에는 출력하지 않는다.
#define MOTOR_OUTPUT_ENABLED 1

// 1 = 양쪽 FSR을 모두 잡아야 '손잡이 잡음'. 0 = 한쪽만 잡아도 인정.
#define HANDLE_REQUIRE_BOTH 0

// 1 = 워치독 감시 켬. loop()가 1초 넘게 멈추면 칩이 리셋되고, 재부팅한
// setup()이 곧바로 BRAKE=HIGH를 걸어 세운다. 기판 R3/R4가 풀다운이라
// MCU가 죽어 있는 동안에는 하드웨어 자동 제동이 없으므로, 그 구멍을
// 메우는 것이 이 워치독이다.
//
// 실물 검증(업로드 후 한 번은 반드시): loop() 안에 임시로 `while (1);`을
// 넣어 올리고, 약 1초 뒤 시리얼에 "E,BOOT"와 "E,RST,WDT"가 다시 뜨는지 본다.
//   - 다시 뜨면 정상.
//   - "E,BOOT"만 무한 반복되면 이 보드의 부트로더가 워치독 리셋 플래그를
//     안 지우는 개체다. setup() 첫머리의 MCUSR/wdt_disable 처리로 대부분
//     막히지만, 그래도 루프가 나면 아래를 0으로 내리고 업로드해 빠져나온다.
#define WATCHDOG_ENABLED 1

// 워치독 타임아웃. loop() 최장 구간이 수 ms 수준이라 1초는 넉넉하다.
// 초음파 한 번이 최악(에코 상승 대기 3ms + 에코 한계 6ms)이라도 9ms다.
#define WATCHDOG_TIMEOUT WDTO_1S

// 1로 바꾸면 전방 장애물이 US_OBSTACLE_BRAKE_MM보다 가까울 때 자동 제동.
// 기본 0 = 초음파는 측정과 텔레메트리 전송만 한다.
// 주의: 이 세 센서는 노면을 보도록 아래로 기울여 달았다. 평지에서 나오는
// 값이 이미 US_MOUNT_HEIGHT_M / sin(US_TILT_DEG) (기본 설정에서 161mm)라
// 임계값을 그보다 확실히 낮게(기본 80mm, 평지 반사값의 절반) 잡지 않으면
// 평지에서 계속 제동이 걸린다. 앞을 보는 장애물 감지가 필요하면 수평 센서를 따로 다는 편이 맞다.
// 실차 시험 전에 임계값을 반드시 현장에서 확인하고 켤 것.
#define US_OBSTACLE_BRAKE_ENABLED 0

// 원본 detect.py는 홈 상태에서 너비 판정을 먼저 하고, 너비가 CAUTION 구간
// (0.5*safe_gap 이상)에 들어가면 그 프레임에서는 복귀 판정을 아예 건너뛴다.
// 그래서 홈이 끝나 원래 높이로 돌아와도 in_hole이 풀리지 않고, 다음 프레임에
// 너비가 safe_gap을 넘으면서 반드시 STAIR(DANGER)가 된다. 시뮬레이션에서
// 3cm 홈이 상수를 어떻게 잡아도 전부 계단으로 보고된 원인이 이것이라,
// CAUTION은 DANGER 직전 한 프레임짜리 상태로만 존재한다.
//   1 = 복귀/턱 판정을 너비 판정보다 먼저 해서, 홈이 끝나면 그 자리에서
//       지나온 너비로 SAFE/CAUTION/DANGER를 매기고 홈 상태를 닫는다.
//   0 = 원본 detect.py와 완전히 같은 순서(위 성질을 그대로 감수).
#define HOLE_EXIT_CHECK_FIRST 1

// 초음파는 노면 상태에 따라 에코가 통째로 빠지는 프레임이 섞인다.
//   1 = 직전 유효 측정값을 들고 있다가 다음 유효 측정과 비교하고, 놓친
//       시간만큼 홈 너비도 이어서 적분한다. 실패 한 번에 판정이 두 프레임
//       먹통이 되지 않는다.
//   0 = 원본 detect.py처럼 이전 값을 버린다(실패 다음 프레임도 판정 없음).
#define US_KEEP_PREV_ON_DROPOUT 1

// 초음파는 다중 반사 때문에 가끔 한 프레임만 크게 튀는 값(헛에코)을 낸다.
// 시뮬레이션에서 과잉경보의 사실상 전부가 이 한 프레임짜리 스파이크였다
// (헛에코를 0으로 두면 과잉경보 450건 -> 0건). 판정에 넣기 전에 최근 유효
// 측정 3개의 중앙값을 쓰면 고립된 스파이크가 그대로 걸러진다.
//   1 = 3점 중앙값 필터 사용. 진짜 단차는 한 프레임(약 45ms) 늦게 잡힌다.
//   0 = 원본처럼 측정값을 그대로 판정에 넣는다.
// 텔레메트리로 나가는 us_l/us_c/us_r과 장애물 제동은 원래 측정값을 쓴다.
#define US_MEDIAN_FILTER 1

// 1로 두면 위험 판정이 바뀔 때마다 사람이 읽을 수 있는 줄을 하나 더 낸다.
// 시리얼 모니터에서 눈으로 확인하는 용도라 평소에는 0으로 꺼 둔다.
// '#'로 시작해서 파이/앱 파서는 어차피 무시하지만, 9600 baud에서 한 줄에
// 약 60ms가 들기 때문에 굳이 흘려보낼 이유가 없다. 현장에서 눈으로
// 확인해야 할 때만 1로 바꿔 굽는다.
//   # 12.345s DANGER HOLE  C dist=250mm width=45mm depth=150mm
//   # 13.900s SAFE   CLEAR
#define HUMAN_READABLE_LOG 0

// 진단용. 1로 두면 US_RAW_DEBUG_SENSOR 번 센서의 측정을 매 프레임 그대로
// 내보낸다. 가만히 서 있는데 경보가 뜨거나, 거리값이 튀는 것 같을 때 쓴다.
//     R,<센서>,<원시mm>,<중앙값mm>,<편차mm>,<에코us>,<실패원인>
//   실패원인 0=정상 1=에코 안 올라옴(배선/전원) 2=에코 안 내려옴 3=범위 밖
// 편차는 평지 기준거리 대비 노면 높이 변화다(+ 홈 / - 턱). 서 있으면 이 값이
// 0 근처에 머물러야 한다. 두 값 사이를 규칙적으로 오가면 센서 간 크로스토크,
// 불규칙하게 튀면 반향이 약한 것이다.
// 0=좌 1=중 2=우, 255=세 개 전부.
// 9600 baud는 초당 960바이트인데 R 줄 하나가 최대 28바이트다. 한 센서분
// (45ms 주기)이면 초당 약 620바이트라 T 줄과 같이 나가도 여유가 있다.
// 255로 세 개를 다 내면 초당 약 1900바이트가 필요해 송신 버퍼가 막히고,
// Serial.print가 버퍼를 기다리면서 측정 주기가 45ms에서 약 100ms로 늘어난다.
// US_STALE_INTERVAL_S(0.25s) 안쪽이라 판정 자체는 계속 돌지만, 확정에
// 걸리는 시간이 두 배가 되어 경보가 그만큼 늦게 뜬다. 주행 중에는 한 센서만,
// 세워 놓고 세 개를 비교할 때만 255로 둘 것.
#define US_RAW_DEBUG 0
#define US_RAW_DEBUG_SENSOR 1

// 노면 위험 판정은 모터에 개입하지 않는다. 위험도/위험원인은 앱으로만 간다.
// 1 = IMU 가속도 적분으로 속도를 추정. 0 = SPEED_FIXED_MPS 고정값 사용.
// 홈의 너비는 속도 * 시간으로 쌓이므로, 실험 초반에 속도 추정이 불안하면
// 0으로 두고 밀고 다니는 평균 속도를 SPEED_FIXED_MPS에 넣는 편이 낫다.
#define SPEED_FROM_IMU 1

// 텔레메트리는 아두이노 -> 파이/앱 단방향이다. 속도는 이 보드의 IMU에서
// 직접 얻으므로 파이가 아두이노로 보내주는 값은 없다.
// 파이의 app_3_12.py도 같은 값이어야 한다(현재 9600).
const unsigned long SERIAL_BAUD = 9600;

// ===================== 핀 연결 =====================
// ZS-X11H V1 제어 3선 (모터당 PWM / DIR / BRAKE)
//
// !! 기판과 불일치 !!  main_circuit 기판의 J14는 실크와 회로도가 모두
// D44~D49로 되어 있다(1:D44 2:D46 3:D48 4:D45 5:D47 6:D49 7:GND).
// 여기 D5~D10은 사용자 지시로 유지한 값이다. 둘 중 하나는 고쳐야 한다.
// 참고: Mega2560의 하드웨어 PWM은 D2~D13과 D44~D46이라 어느 쪽이든
// PWM 자체는 성립한다. D5=Timer3A, D7=Timer4B를 PWM으로 쓴다.
const uint8_t LEFT_MOTOR_PWM_PIN = 5;      // 왼쪽 ZS-X11H PWM
const uint8_t LEFT_MOTOR_DIR_PIN = 6;      // 왼쪽 ZS-X11H DIR
const uint8_t LEFT_MOTOR_BRAKE_PIN = 9;    // 왼쪽 ZS-X11H BRAKE
const uint8_t RIGHT_MOTOR_PWM_PIN = 7;     // 오른쪽 ZS-X11H PWM
const uint8_t RIGHT_MOTOR_DIR_PIN = 8;     // 오른쪽 ZS-X11H DIR
const uint8_t RIGHT_MOTOR_BRAKE_PIN = 10;  // 오른쪽 ZS-X11H BRAKE
const uint8_t BELT_HALL_PIN = 28;          // SEN080603 S/OUT
const uint8_t HANDLE_FSR1_PIN = A0;        // 손잡이 FSR 1 분압 중간점
const uint8_t HANDLE_FSR2_PIN = A1;        // 손잡이 FSR 2 분압 중간점

// HC-SR04 3개. 센서 VCC/GND는 기판 +5V 레일에서, TRIG/ECHO는 메가에 직결.
// Mega는 5V 로직이라 ECHO에 전압분배가 필요 없다.
const uint8_t US_COUNT = 3;
const uint8_t US_TRIG_PINS[US_COUNT] = { 22, 24, 26 };  // 좌, 중, 우
const uint8_t US_ECHO_PINS[US_COUNT] = { 23, 25, 27 };

// ===================== IMU 설정 =====================
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SENSITIVITY = 16384.0;  // +/-2g
const float GYRO_SENSITIVITY = 131.0;     // +/-250 deg/s
const float FILTER_TIME_CONSTANT_S = 0.35;

// 유모차 앞부분을 들었는데 DOWN으로 나오면 true로 바꾼다.
// accelPitchDeg와 pitchRateDps를 함께 뒤집는다. 아래 축 매핑이 맞는데
// 부호만 반대일 때 쓰는 스위치다. '양쪽 다 같은 방향'으로 나오는 증상은
// 이걸로 고쳐지지 않는다(양쪽 다 오르막이 양쪽 다 내리막이 될 뿐이다).
const bool INVERT_PITCH_DIRECTION = false;

// ---- 센서 축 매핑 ----
// 어느 센서축이 유모차의 어느 축인지. 부호 포함.
// 평지 정지 상태에서 E,ACC의 |값|이 가장 큰 축(약 16384)이 진짜 수직축이고,
// 그 축이 ACC_VERT에 들어가야 한다.
//
// ACC_VERT를 잘못 잡으면 전후축에 중력이 실린다. 그러면 아래 식
//     atan2(forwardG, sqrt(sideG^2 + verticalG^2))
// 의 두 번째 인자가 항상 0 이상이라 결과가 -90~+90에 갇히고, 평지가 그
// 꼭짓점(±90)에 놓인다. sqrt가 부호를 지워버리기 때문에 앞으로 기울이든
// 뒤로 기울이든 각도가 같은 방향으로만 움직인다.
//   2026-09 실측 증상: 어느 쪽으로 기울여도 오르막으로 판정.
//   당시 매핑은 전후축=-Z, 수직축=X 였고, 실제로는 중력이 Z에 실려 있었다.
//   즉 전후축과 수직축이 서로 바뀌어 있었다. 아래가 바로잡은 값이다.
//
// 부호 맞추는 법 (실물에서 반드시 확인):
//   1. 평지에서 E,ACC를 보고 |값|이 큰 축을 ACC_VERT에 넣는다.
//   2. 남은 두 축 중 앞뒤로 기울일 때 값이 크게 변하는 쪽이 ACC_FORWARD,
//      나머지가 ACC_SIDE다.
//   3. 유모차 앞을 들었을 때 텔레메트리 pitch가 +로 나와야 한다.
//      -로 나오면 ACC_FORWARD의 부호를 뒤집거나
//      INVERT_PITCH_DIRECTION을 true로 한다.
//   4. 가속도와 자이로가 서로 반대로 움직이면(기울이는 도중에만 값이
//      튀었다가 되돌아온다) GYRO_PITCH만 (-gy)로 뒤집는다.
#define ACC_FORWARD   ( ax)   // 전후축: 유모차 앞이 올라가면 +
#define ACC_SIDE      ( ay)   // 좌우축
#define ACC_VERT      ( az)   // 수직축: 평지에서 중력이 실리는 축
#define GYRO_PITCH    ( gy)   // 피치 각속도. 부호가 ACC_FORWARD와 맞아야 한다

// 보정에서 잡은 장착 오프셋이 이 각도를 넘으면 축 매핑을 의심한다.
// 정상 장착이면 ±10도 안쪽이고, ±90 근처면 전후축에 중력이 실린 것이다.
const float IMU_AXIS_WARN_DEG = 30.0;

// ===================== 센서 판단값 =====================
const float FLAT_THRESHOLD_DEG = 5.0;
const float SLOPE_THRESHOLD_DEG = 8.0;
const uint8_t REQUIRED_SLOPE_COUNT = 3;

// 실제 손잡이에서 텔레메트리의 fsr1/fsr2 값을 본 뒤 조정한다.
// handle_sensor.ino는 임계값 3에 반전 논리를 썼는데, 그건 분압 배선이
// 다른 시험용 값이다. 여기서는 stroller_control.ino 쪽 기준을 따른다.
const int FSR_GRIP_ON_THRESHOLD = 800;
const int FSR_GRIP_OFF_THRESHOLD = 750;
const unsigned long BELT_DEBOUNCE_MS = 50;

// 홀센서가 '자석 감지 = 벨트 체결'로 읽히는 레벨.
// INPUT_PULLUP이라 아무것도 없으면 HIGH다. SEN080603은 자석이 붙으면
// LOW로 떨어지는 게 보통이라 원래 LOW였는데, 실제 장착 상태에서 앱의
// 벨트 표시가 반대로 나와서 2026-08-26에 HIGH로 뒤집었다.
// 앱에서 또 반대로 보이면 이 한 줄만 LOW/HIGH로 바꾸면 된다.
const uint8_t BELT_FASTENED_LEVEL = HIGH;

// ===================== 초음파 측정값 =====================
const unsigned long US_PING_INTERVAL_MS = 15;    // 센서 간 간격 -> 센서당 45ms
// 트리거 후 에코가 올라오기를 기다리는 한계. 정품 HC-SR04는 0.5ms 안에
// 올리지만 클론 중에는 더 걸리는 것이 있어 넉넉히 잡는다. 이 시간을 넘기면
// '센서 응답 없음'(거리 0)이 된다 -- 배선이나 전원이 끊겼을 때의 증상이다.
const unsigned long US_RISE_TIMEOUT_US = 3000;   // 에코 상승 대기 한계
const unsigned long US_MAX_ECHO_US = 6000;       // 약 1 m 왕복
const uint16_t US_MIN_VALID_MM = 20;             // HC-SR04 최소 측정 거리
const uint16_t US_MAX_VALID_MM = 1000;           // 노면용으로 좁힌 상한
const uint16_t US_OBSTACLE_BRAKE_MM = 80;        // 자동 제동 임계(옵션)

// ===================== 초음파 장착 형상 =====================
// 센서별로 자유롭게 바꾼다. 좌 / 중 / 우 순서.
//   US_MOUNT_HEIGHT_M : 노면에서 센서까지의 높이 (m)
//   US_TILT_DEG       : 수평면 기준 아래로 숙인 각도 (도). 90 = 정확히 아래
// 각도는 빔이 노면에 그리는 띠의 길이를 좌우한다(아래 US_BEAM_HALF_ANGLE_DEG).
// 부팅 때 E,US_BEAM,<빔길이mm>,<전방주시mm> 로 실제 값을 찍어 준다.
// 세 개를 서로 다르게 달아도 되고, 값만 여기서 고치면 된다.
// 각도가 0에 가까우면(수평) 노면을 보지 않는 셈이라 노면 판정이 무의미하다.
// 그래서 sin은 US_MIN_TILT_SIN 아래로 내려가지 않게 막는다.
float US_MOUNT_HEIGHT_M[US_COUNT] = { 0.16, 0.16, 0.16 };
// 좌우는 급하게(빔 띠 51mm, 바퀴 경로의 홈까지), 중앙은 얕게(빔 띠 86mm,
// 대신 전방 16cm를 봐서 턱/단차를 0.27초 미리 잡는다). 턱과 단차는 가로로
// 길게 이어져 있어 중앙 하나로 충분하고, 홈은 국소적이라 바퀴 경로인
// 좌우에서 봐야 한다.
float US_TILT_DEG[US_COUNT] = { 65.0, 45.0, 65.0 };
const float US_MIN_TILT_SIN = 0.0175;  // 약 1도

// HC-SR04의 유효 빔 반각. 초음파는 선이 아니라 원뿔이라 노면을 띠로 비추고,
// 거리계는 그 띠 안에서 가장 먼저 돌아온 에코(= 최단 거리)를 값으로 낸다.
// 띠의 길이 = h/tan(각도-반각) - h/tan(각도+반각) 이고, 이 길이가
//   - 감지할 수 있는 최소 홈 너비이며 (띠가 통째로 홈 안에 들어가야 보인다)
//   - 홈의 겉보기 너비를 그만큼 짧게 만든다.
// 그래서 홈에 진입할 때 이 길이를 미리 더해 준다(usBeamFootprintM).
const float US_BEAM_HALF_ANGLE_DEG = 7.5;

// 노면으로 인정할 측정값의 범위. 평지 기대값(height / sin(tilt))에 대한 비율.
// 이 밖의 값은 노면이 아니라고 보고 '측정 실패'로 처리한다(원본의 None).
const float US_GROUND_MIN_RATIO = 0.15;
const float US_GROUND_MAX_RATIO = 3.00;

// ===================== 노면 위험 판정값 =====================
// 초음파는 빔 원뿔 안에서 '가장 먼저 돌아온 에코', 즉 최단 거리를 값으로
// 낸다. 그래서 평지에서는 빔의 가장 아래쪽 광선(내려다보는 각 = 설치각 +
// 빔반각)이 만드는 값 하나로 고정된다.
//     d0 = 설치높이 / sin(설치각 + 빔반각)
// 이 기준보다 길면 노면이 내려간 것(홈), 짧으면 올라온 것(턱)이다. 원본
// detect.py처럼 이전 프레임과의 차이를 볼 필요가 없다. 거리 편차를 노면
// 높이 변화로 바꾸는 계수는 k = sin(설치각 + 빔반각)이고,
//     노면 높이 변화 = (측정거리 - d0) * k        (+ 홈 / - 턱)
//
// 종류가 갈리면 상태별 하위 판정으로 들어간다.
//   홈 상태 : 빔 띠가 통째로 들어가야 홈이 보이기 시작하는데, 이 띠가
//             safe_gap(4.2cm)보다 길다(45도에서 86mm, 65도에서 51mm).
//             즉 보이는 홈은 이미 바퀴가 빠지는 크기라 바로 DANGER다.
//             너비 적분은 앱에 참고값으로 실어 보내는 용도로만 남겼다.
//   턱 상태 : 편차의 최대값을 추적한다. 경사로에 진입할 때도 전방을 보는
//             센서에는 노면이 올라오는 것처럼 잠깐 보이는데, 그 크기는
//                 전방주시거리 * tan(경사각)
//             뿐이라(45도/16cm에서 10도 경사면 28mm) 실제 턱의 수십 mm와
//             크기로 갈린다. 최대 편차가 턱 확정 임계에 못 미친 채 노면이
//             돌아오면 경사로나 잔요철로 보고 조용히 끝낸다.
// 임계값은 고정값으로 박지 않고 부팅 보정에서 만들 수도 있다. 부팅 직후
// 평지를 굴리는 동안 기준거리와 함께 그 구간의 잡음(표준편차)을 재고,
// 임계값을 잡음의 배수로 잡는 방식이다. 센서 개체차, 노면 재질, 장착
// 상태가 달라도 따라간다.
//   시작할 때 평지에 있어야 한다는 제약이 생기는 대신, 손으로 맞춘 숫자를
//   현장마다 다시 맞출 필요가 없어진다.
// 잡음이 비정상적으로 작게(또는 크게) 잡히는 경우를 막으려고 하한과 상한을
// 함께 둔다.
// 임계값을 어디서 얻을지 고르는 스위치.
//   1 = 부팅 보정에서 잰 잡음의 배수로 만든다. 센서 개체차나 노면 재질이
//       달라도 따라가지만, 시동할 때 평지에 있어야 한다.
//   0 = 아래 고정값을 그대로 쓴다(현재 값). 보정은 기준거리만 잡는다.
//       시동 위치를 평지로 잡기 어렵거나 임계값을 손으로 못 박고 싶을 때.
// 두 판의 차이는 파일 상단 '임계값 스위치' 항목에 정리해 두었다.
#define TERRAIN_THRESHOLD_FROM_CALIBRATION 0

// 스위치가 0일 때 쓰는 고정 임계값. 센서마다 따로 잡는다.
// 단위는 cm 다. 판정 내부는 m로 돌아가고 E,US_CAL 줄은 mm로 나가므로,
// applyFixedThresholds가 100으로 나눠 옮겨 담는다.
//
// 이 값은 '노면 높이가 평지 기준에서 얼마나 벗어났는가'다. 측정 거리 자체가
// 아니라 편차이고, 부호는 턱이 위(노면이 올라옴) / 홈이 아래다.
//
// 센서마다 다른 이유는 각도가 달라서다. 중앙 45도는 전방 160mm를 보기 때문에
// 경사로에 들어가는 것도 노면이 올라오는 것으로 보이고(160mm * tan10도 = 28mm),
// 빔 띠가 86mm라 좁은 홈이 만드는 겉보기 점프도 48mm로 크다. 그래서 중앙만
// 임계가 높다. 좌우 65도는 전방 75mm / 빔 띠 51mm라 그만큼 낮게 잡는다.
//
// 턱은 시작(상태 진입)과 확정(DANGER) 두 단계인데, 여기 적는 값은 확정
// 임계다. 시작 임계는 그 절반으로 자동으로 잡는다 -- 원래 코드의 20:40
// 비율을 그대로 옮긴 것이다.
//                                            좌     중     우
const float STEP_DANGER_FIXED_CM[US_COUNT] = { 16.0,  30.0,  12.0 };
const float HOLE_ENTER_FIXED_CM[US_COUNT]  = { 20.0,  50.0,  18.0 };
const float STEP_ENTER_RATIO = 0.5;       // 턱시작 = 턱확정 * 이 비율
const float TERRAIN_EXIT_FIXED_CM = 1.0;  // 노면 복귀

// ------------------------------------------------------------
// 주의: 위 여섯 값 중 셋은 지금 설치(높이 16cm, 45/65도)에서 물리적으로
// 발동하지 않는다. 편차가 그만큼 나오려면 아래 측정 거리가 필요한데, 그
// 거리가 유효 측정창(좌우 26~530mm, 중앙 34~679mm) 밖이기 때문이다.
// 창 밖 측정은 노면이 아니라고 보고 '측정 실패'로 버려진다.
//
//   좌 턱 16cm -> 측정거리   0mm 필요   발동 불가
//   중 턱 30cm -> 측정거리  음수 필요   발동 불가
//   중 홈 50cm -> 측정거리 832mm 필요   발동 불가 (창 상한 679mm)
//   우 턱 12cm -> 측정거리  42mm        발동 가능 (노면이 센서 4cm 앞까지)
//   좌 홈 20cm -> 측정거리 378mm        발동 가능
//   우 홈 18cm -> 측정거리 356mm        발동 가능
//
// 즉 턱·단차를 맡는 중앙 센서는 경보를 한 번도 내지 않는다. 편차의 물리적
// 상한은 센서 높이(16cm) 근처이고, 중앙 홈은 창 상한이 먼저 걸린다.
// 되살리려면 둘 중 하나다.
//   - 이 값들을 편차 실측 범위(수 cm)로 낮춘다.
//   - 중앙 홈만 살리려면 US_GROUND_MAX_RATIO(3.00)와 US_MAX_VALID_MM(1000),
//     US_MAX_ECHO_US(6000)를 함께 올려 창을 832mm 위로 넓힌다. 턱 쪽은
//     창을 넓혀도 상한이 센서 높이라 살아나지 않는다.
// ------------------------------------------------------------

const float STEP_ENTER_SIGMA = 4.0;    // 턱 판정 시작 = 잡음의 몇 배
const float STEP_DANGER_SIGMA = 8.0;   // 턱 확정
const float HOLE_ENTER_SIGMA = 4.0;    // 홈 판정 시작
const float TERRAIN_EXIT_SIGMA = 2.0;  // 노면 복귀

const float STEP_ENTER_FLOOR_M = 0.010;
const float STEP_DANGER_FLOOR_M = 0.030;
const float HOLE_ENTER_FLOOR_M = 0.020;
const float TERRAIN_THRESHOLD_CEIL_M = 0.080;  // 이보다 크면 보정을 의심한다

// 홈 판정에는 기하에서 나오는 하한이 하나 더 있다. 빔 띠보다 좁은 홈은
// 측정값이 '빔 안에서 두 번째로 가까운 지점'으로 옮겨가는 만큼만 튀는데,
// 그 크기가 홈 깊이와 무관하게 아래 값으로 고정된다.
//     점프 상한 = h * ( sin(각+반각) / sin(각-반각) - 1 )
//     16cm 기준으로 45도 48mm, 65도 21mm, 85도 4mm
// 홈 진입 임계를 이 값 위에 두면, 경보가 뜬 홈은 반드시 빔 띠보다 넓다.
// 빔 띠는 이미 safe_gap(4.2cm)보다 기니까 '뜨면 확실히 위험한 홈'이 된다.
const float HOLE_ENTER_CAP_MARGIN = 1.15;

// 턱 판정에도 같은 성격의 기하 하한이 있다. 전방을 보는 센서에는 경사로에
// 진입하는 것도 노면이 올라오는 것으로 보이는데, 그 크기가
//     전방주시거리 * tan(경사각)
// 이다. 45도(전방 160mm)에서 10도 경사면 29mm라 웬만한 턱과 맞먹는다.
// 그래서 '이 각도까지의 경사로는 턱으로 보지 않는다'를 정하고, 턱 확정
// 임계를 그 위에 둔다. 값을 올리면 경사로 오경보가 줄고 낮은 턱을 놓친다.
const float RAMP_MAX_DEG = 10.0;
const float STEP_DANGER_RAMP_MARGIN = 1.2;

const float WHEEL_WIDTH_M = 0.07;    // 앞바퀴 지름 (m)

// 안전하게 지날 수 있는 홈의 너비. 원본 detect.py와 같은 비율이다.
// 지금 설치(16cm, 45/65도)에서는 빔 띠가 이미 이 값보다 길어서 판정에
// 직접 쓰이지는 않고, 앱으로 보내는 너비 값의 해석 기준으로만 남는다.
const float HOLE_SAFE_GAP_RATIO = 0.6;
const float HOLE_SAFE_GAP_M = HOLE_SAFE_GAP_RATIO * WHEEL_WIDTH_M;

// 부팅 직후 이만큼의 유효 측정을 모아 평지 기준거리 d0를 실측으로 잡는다.
// 타이어 공기압이나 장착 오차로 실제 높이가 조금 달라도 흡수된다. 평균이
// 계산값과 너무 다르면(위험 지형 위에서 켠 경우) 계산값을 그대로 쓴다.
const uint8_t US_BASELINE_SAMPLES = 20;
// 보정에서 잰 기준거리가 계산값에서 이 비율 밖으로 벗어나면 평지를 보고
// 있는 것이 아니다. 45도 센서는 앞을 보기 때문에 앞에 책상이나 벽이 있으면
// 그 거리가 잡힌다(실측에서 202mm 기대에 418mm가 나온 적이 있다).
//
// 이때 계산값으로 슬쩍 되돌리면 안 된다. 센서는 계속 책상을 보는데 기준만
// 노면 값이 되어, 편차가 영원히 크게 남고 조금만 흔들려도 턱과 홈이 번갈아
// 뜬다. 그래서 보정을 받아들이지 않고 처음부터 다시 잰다. 그 센서는 평지가
// 잡힐 때까지 판정에 참여하지 않는다.
const float US_BASELINE_MIN_RATIO = 0.6;
const float US_BASELINE_MAX_RATIO = 1.6;
const float US_BASELINE_MAX_NOISE_M = 0.015;  // 이보다 흔들리면 경고한다

// 판정 상태를 바꾸기 전에 같은 방향이 몇 프레임 연속으로 보여야 하는지.
// 발이나 사람처럼 빔에 걸쳤다 빠졌다 하는 것이 한 프레임씩 튀면서 턱과 홈을
// 번갈아 만들어내는 것을 막는다. 1로 두면 예전처럼 한 프레임에 바로 판정한다.
const uint8_t TERRAIN_CONFIRM_FRAMES = 2;

// 위험(DANGER)을 내보내기 전에 위험 조건 자체가 몇 프레임 연속으로
// 반복돼야 하는지. TERRAIN_CONFIRM_FRAMES가 '상태에 들어가는' 문턱이라면
// 이쪽은 '경보를 내는' 문턱이다.
//   턱 : 편차가 턱확정 임계를 이만큼 연속으로 넘어야 DANGER. 예전에는
//        상태에 들어간 뒤 한 프레임만 넘어도 바로 확정했다.
//   홈 : 홈 진입이 곧 위험 판정이라 진입 표를 이 값까지 세고 나서 낸다.
// 1로 두면 예전처럼 한 프레임에 바로 확정한다. 값이 튈 때 올린다.
const uint8_t TERRAIN_DANGER_CONFIRM_FRAMES = 2;

// 홈은 진입 조건과 위험 조건이 같아서, 둘 중 큰 쪽을 쓰면 두 규칙을 모두 만족한다.
const uint8_t HOLE_DANGER_VOTES =
    (TERRAIN_CONFIRM_FRAMES > TERRAIN_DANGER_CONFIRM_FRAMES)
        ? TERRAIN_CONFIRM_FRAMES : TERRAIN_DANGER_CONFIRM_FRAMES;

// 측정 주기
//   센서 하나가 다시 측정될 때까지의 시간은 US_PING_INTERVAL_MS * US_COUNT,
//   기본값에서 45ms다. 판정이 절대 기준거리 방식으로 바뀌면서 한 프레임만
//   있어도 종류가 갈리므로, 예전처럼 속도에 따라 판정이 깨지는 한계는 없다.
//   다만 위험이 보이는 구간을 몇 프레임 안에 지나가느냐는 여전히 중요하다.
//   0.4 m/s에서 한 프레임에 18mm를 이동한다.
//
//   US_MAX_ECHO_US(6ms)는 측정 간격보다 짧게 잡아 두었다. 앞 센서의 늦은
//   에코가 다음 센서의 청취 구간에 들어와도 범위 밖으로 버려지고, 잘못된
//   거리 대신 '측정 실패'가 되어 판정이 안전한 쪽으로 넘어간다.

// 한 센서의 측정이 이 시간 이상 끊기면 연속성이 깨진 것으로 보고
// 그 센서의 판정 상태를 초기화한다.
const float US_STALE_INTERVAL_S = 0.25;
const float US_MIN_INTERVAL_S = 0.01;

// 위험도는 한 프레임짜리 판정이라 그대로 두면 500ms 텔레메트리에서 놓친다.
// 마지막 위험 판정을 이 시간만큼 유지해서 앱이 확실히 받게 한다.
const unsigned long RISK_HOLD_MS = 1500;

// ===================== 속도 추정 (IMU) =====================
// 파이썬에서는 시리얼로 받던 speed를 여기서는 같은 보드의 IMU로 만든다.
// 전후 가속도에서 중력 성분을 빼고 적분하되, 적분 드리프트를 누설
// (SPEED_LEAK_PER_S)로 계속 갉아 준다. 정밀한 속도계가 아니라 홈 너비를
// 쌓는 용도의 근사값이다. 실측하며 아래 상수를 맞춘다.
// 가속도만으로는 등속 구간의 속도를 유지할 수 없다(가속도가 0이라 정보가
// 없다). 그래서 세 가지를 겹쳐 쓴다.
//   1) 느린 저역통과로 가속도 바이어스와 피치 오차를 빼낸다.
//   2) 빠른 저역통과와의 차이(진동)로 '움직이는 중'인지 판단한다.
//      절대값이 아니라 진동을 보므로 경사로에 세워 둬도 오판하지 않는다.
//   3) 적분값은 실측 평균 밀기 속도(SPEED_NOMINAL_MPS)로 수렴시킨다.
//
// 시뮬레이션 결과를 그대로 적자면, 수렴 속도(SPEED_LEAK_PER_S)를 올릴수록
// 오차가 줄었다. 즉 MPU6050 하나로는 적분이 보태 주는 정보가 사실상 없고,
// 이 추정기는 실질적으로 '이동 중이면 SPEED_NOMINAL_MPS, 서 있으면 0'에
// 가깝게 동작한다(순항 RMS 오차 약 0.15 m/s, 그 대부분이 실제 속도와
// SPEED_NOMINAL_MPS의 차이다). 정지 판정은 잘 맞아서 서 있을 때 홈 너비가
// 헛되이 쌓이지는 않는다.
//   -> 그래서 SPEED_NOMINAL_MPS를 실제로 밀어 보고 잰 평균 속도로 맞추는
//      것이 이 파일에서 가장 중요한 실측 작업이다. 홈 너비는 속도 x 시간
//      이라 속도 오차가 그대로 너비 오차가 된다.
//   -> 바퀴에 엔코더나 홀센서를 달 수 있으면 그쪽이 훨씬 정확하다.
const float GRAVITY_MPS2 = 9.80665;
const float SPEED_NOMINAL_MPS = 0.40;             // 실측 평균 밀기 속도
const float SPEED_BIAS_TC_S = 1.0;                // 바이어스 추정 시정수
const float SPEED_VIB_TC_S = 0.15;                // 진동 성분 추출 시정수
const float SPEED_MOTION_DEADBAND_MPS2 = 0.10;    // 이보다 큰 진동이면 이동 중
const float SPEED_LEAK_PER_S = 2.0;               // 평균 속도로 수렴하는 속도
const unsigned long SPEED_ZERO_HOLD_MS = 400;     // 진동이 끊기면 정지로 간주
const float SPEED_MAX_MPS = 3.0;                  // 추정 상한
const float SPEED_FIXED_MPS = 0.40;               // SPEED_FROM_IMU 0일 때 사용
const float SPEED_FALLBACK_MPS = 0.40;            // IMU 실패 시 가정 속도

// ===================== 모터 시험값 =====================
// PWM_MIN_DRIVE는 구간별 시험에서 찾은 확실히 회전하는 최소값으로 수정한다.
const uint8_t PWM_MIN_DRIVE = 120;
const uint8_t PWM_MAX_ASSIST = 200;
const float PWM_START_SLOPE_DEG = 8.0;
const float PWM_MAX_SLOPE_DEG = 15.0;
const uint8_t PWM_RISE_STEP = 2;
const uint8_t PWM_FALL_STEP = 6;

// ZS-X11H의 DIR은 액티브 로우다. 전진에 해당하는 레벨을 여기서 정한다.
// 좌/우 중 반대로 도는 쪽만 LOW/HIGH를 바꾼다.
const uint8_t LEFT_FORWARD_DIR_LEVEL = LOW;
const uint8_t RIGHT_FORWARD_DIR_LEVEL = LOW;

// 출력 모드 전환 시 드라이버가 상태를 정리할 시간(us).
const unsigned int MOTOR_TRANSITION_BLANK_US = 100;

// ===================== 실행 주기 =====================
const unsigned long IMU_INTERVAL_MS = 20;
// IMU가 죽어 있을 때 재연결을 시도하는 주기. 예전에는 부팅 때 한 번
// 실패하거나 E,IMU_LOST가 한 번 뜨면 재부팅 전까지 영영 SENSOR_FAULT였다.
const unsigned long IMU_RETRY_INTERVAL_MS = 2000;
const unsigned long INPUT_INTERVAL_MS = 20;
const unsigned long SLOPE_INTERVAL_MS = 100;
const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long TELEMETRY_INTERVAL_MS = 500;
const uint8_t INITIAL_SETUP_SECONDS = 5;

// 텔레메트리 코드값은 파이 파서와 맞물려 있다. 순서를 바꾸지 말 것.
enum SlopeState : uint8_t {
  SLOPE_FLAT = 0,
  SLOPE_UP = 1,
  SLOPE_DOWN = 2,
  SLOPE_UNCERTAIN = 3
};

enum ControlMode : uint8_t {
  MODE_INITIALIZING = 0,
  MODE_SENSOR_FAULT = 1,
  MODE_HANDLE_RELEASED = 2,
  MODE_DOWNHILL_BRAKE = 3,
  MODE_UPHILL_ASSIST = 4,
  MODE_FLAT = 5,
  MODE_UNCERTAIN = 6,
  MODE_OBSTACLE_BRAKE = 7
};

enum MotorOutput : uint8_t {
  MOTOR_COAST = 0,
  MOTOR_FORWARD = 1,
  MOTOR_BRAKE = 2
};

enum UsPhase : uint8_t {
  US_IDLE,
  US_MEASURING  // 한 번의 updateUltrasonic 안에서만 유지된다
};

// detect.py의 Risk / Hazard IntEnum과 값이 같다.
enum RiskLevel : uint8_t {
  RISK_SAFE = 0,
  RISK_CAUTION = 1,
  RISK_DANGER = 2
};

// 원본 detect.py는 STEP / WIDE_HOLE / EXIT_STEP / STAIR 네 가지였는데,
// 앱에서 쓸 구분은 결국 '노면이 올라왔는가(턱)'와 '내려갔는가(홈)' 둘이라
// 두 가지로 합쳤다.
//   STEP <- 턱, 홈에서 빠져나오며 원래 높이보다 높아진 턱
//   HOLE <- 지나갈 수 없는 넓은 홈, 계단/단차
enum HazardCause : uint8_t {
  HAZARD_NONE = 0,
  HAZARD_STEP = 1,  // 턱: 노면이 갑자기 올라옴
  HAZARD_HOLE = 2   // 홈: 노면이 갑자기 내려가고 안전 너비를 넘음
};

// 센서 하나의 노면 판정 상태.
enum TerrainState : uint8_t {
  TERRAIN_IDLE = 0,  // 평지
  TERRAIN_HOLE = 1,  // 홈을 지나는 중
  TERRAIN_STEP = 2   // 노면이 올라옴 (턱인지 경사로인지 판별 중)
};

struct DangerDetector {
  TerrainState state;
  float holeWidthM;        // 홈 상태에서 누적한 너비 (참고값)
  float holeDepthM;        // 홈의 최대 깊이
  float stepPeakM;         // 턱 상태에서 본 최대 높이
  uint8_t holeVotes;       // 같은 방향이 연속으로 보인 프레임 수
  uint8_t stepVotes;
  uint8_t dangerVotes;     // 위험 조건이 연속으로 보인 프레임 수
  unsigned long lastSampleAtMs;
  uint16_t recentMm[3];    // 중앙값 필터용 최근 유효 측정
  uint8_t recentCount;
  float pendingIntervalS;  // 측정 실패로 건너뛴 시간 (다음 유효 프레임에 합산)
  float baselineSumM;      // 부팅 직후 기준거리 실측용
  float baselineSumSqM;
  uint8_t baselineCount;
  float noiseM;            // 보정 구간에서 잰 잡음 (수직 환산)
  float stepEnterM;        // 아래 넷은 보정이 끝나면 잡음에서 만든다
  float stepDangerM;
  float holeEnterM;
  float exitM;
  RiskLevel risk;          // 마지막 판정 (RISK_HOLD_MS 동안 유지)
  HazardCause hazard;
  unsigned long riskAtMs;
  // 판정이 난 그 프레임의 값. 상태를 지우기 전에 잡아 둔다.
  // hazard가 STEP이면 riskDepthM은 홈 깊이가 아니라 턱 높이다.
  float riskWidthM;
  float riskDepthM;
};

// ===================== IMU 값 =====================
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;
float gyroYOffsetDps = 0.0;
float pitchMountOffsetDeg = 0.0;
float accelPitchDeg = 0.0;
float filteredPitchDeg = 0.0;
float pitchRateDps = 0.0;
bool imuReady = false;
uint8_t consecutiveImuFailures = 0;
bool imuCalibrated = false;      // 한 번이라도 보정에 성공했나
// 마지막 IMU 기동 실패 지점. E,IMU_FAIL,<step>으로 나간다. 0 = 실패 없음.
uint8_t imuFailStep = 0;
uint8_t imuWhoAmI = 0xFF;        // 0x75 WHO_AM_I 마지막 읽은 값
int imuCalibSamples = 0;         // 마지막 보정에서 모은 유효 샘플 수
unsigned long lastImuRetryAt = 0;

// ===================== 속도 추정 상태 =====================
float linearAccelMps2 = 0.0;
float accelBiasMps2 = 0.0;
float accelVibLpMps2 = 0.0;
float estimatedSpeedMps = 0.0;
unsigned long lastAccelActiveAtMs = 0;

// ===================== 입력/제어 상태 =====================
int hallRaw = HIGH;
bool beltFastened = false;
bool lastRawBeltFastened = false;
unsigned long hallRawChangedAt = 0;

int fsr1Value = 0;
int fsr2Value = 0;
bool grip1 = false;
bool grip2 = false;
bool handleHeld = false;

SlopeState slopeState = SLOPE_FLAT;
SlopeState previousSlopeCandidate = SLOPE_FLAT;
uint8_t slopeStableCount = 0;
ControlMode controlMode = MODE_INITIALIZING;
MotorOutput motorOutput = MOTOR_COAST;
uint8_t targetPWM = 0;
uint8_t currentPWM = 0;
uint8_t telemetrySeq = 0;

// ===================== 초음파 상태 =====================
UsPhase usPhase = US_IDLE;
uint8_t usIndex = 0;
unsigned long usPingStartedAtMs = 0;
uint16_t usDistanceMm[US_COUNT] = { 0, 0, 0 };

// 진단용. 마지막 측정의 에코 길이(us)와 실패 원인.
//   0 = 정상, 1 = 에코가 안 올라옴(배선/전원 의심), 2 = 에코가 안 내려옴,
//   3 = 거리 범위 밖
uint16_t usLastPulseUs[US_COUNT] = { 0, 0, 0 };
uint8_t usLastFail[US_COUNT] = { 0, 0, 0 };

// 설치 각도/높이에서 미리 계산해 두는 값.
float usSinTilt[US_COUNT];
float usExpectedFlatM[US_COUNT];
float usBeamFootprintM[US_COUNT];  // 빔이 노면에 그리는 띠의 길이
float usLookAheadM[US_COUNT];      // 센서 바로 아래에서 빔 중심까지의 거리
float usBaselineM[US_COUNT];       // 평지에서 나오는 기준 거리 d0 (실측)
float usBaselineComputedM[US_COUNT];  // 설치값에서 계산한 기준 거리
float usHeightGainM[US_COUNT];     // 거리 편차 -> 노면 높이 변화 계수 k
float usJumpCapM[US_COUNT];        // 빔 띠보다 좁은 홈이 만드는 편차의 상한

DangerDetector detectors[US_COUNT];

// 세 센서를 합친 결과. 앱으로 나가는 값이다.
RiskLevel overallRisk = RISK_SAFE;
HazardCause overallHazard = HAZARD_NONE;
uint8_t overallRiskSensor = 255;
RiskLevel reportedRisk = RISK_SAFE;
HazardCause reportedHazard = HAZARD_NONE;

unsigned long lastImuAt = 0;
unsigned long lastInputAt = 0;
unsigned long lastSlopeAt = 0;
unsigned long lastControlAt = 0;
unsigned long lastTelemetryAt = 0;


bool writeRegister(uint8_t reg, uint8_t value);
bool readRegisters(uint8_t startReg, uint8_t *data, uint8_t length);
bool initializeIMU();
bool readIMU();
bool calibrateIMU();
bool bringUpIMU(bool allowCalibration);
void reportImuFailure();
void reportI2cLines();
void scanI2cBus();
void calculateIMUValues();
void updateIMU(unsigned long now);
void updateSpeedEstimate(float dt, unsigned long now);
float currentSpeedMps();
void updateInputs(unsigned long now);
void updateUltrasonic(unsigned long now);
void finishPing();
uint16_t pulseToMm(unsigned long pulseUs);
bool obstacleTooClose();
void setupTerrainDetectors();
void resetDetector(uint8_t index);
void applyFixedThresholds(uint8_t index);
float terrainDeviationM(uint8_t index, uint16_t distanceMm);
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now);
void updateOverallRisk(unsigned long now);
void outputHazardLine();
void outputHumanLine();
SlopeState classifySlope(float pitchDeg);
void updateConfirmedSlope(SlopeState candidate);
void updateControlMode();
uint8_t calculateSlopePWM(float pitchDeg);
void updatePwmRamp();
void commandMotor(MotorOutput requestedOutput, uint8_t pwm);
void writeOneMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin,
                   uint8_t forwardDirLevel, MotorOutput output, uint8_t pwm);
void outputTelemetry();

void setup() {
  // ---- 워치독 부팅 함정 회피 (사용자 코드 첫 줄이어야 한다) ----
  // AVR은 워치독 리셋 후에도 WDT가 켜진 채, 그것도 최단 타임아웃으로
  // 부팅한다. 부트로더가 이걸 안 꺼주는 보드에서는 부팅을 마치기 전에
  // 또 리셋이 걸려 무한 재부팅에 빠진다. WATCHDOG_ENABLED와 무관하게
  // 항상 실행한다 - 이게 안전망이다.
  const uint8_t resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();

  Serial.begin(SERIAL_BAUD);

  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_BRAKE_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_BRAKE_PIN, OUTPUT);
  pinMode(BELT_HALL_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (uint8_t i = 0; i < US_COUNT; i++) {
    pinMode(US_TRIG_PINS[i], OUTPUT);
    digitalWrite(US_TRIG_PINS[i], LOW);
    pinMode(US_ECHO_PINS[i], INPUT);
  }

  setupTerrainDetectors();

  // 부팅/보정 중에는 모터 전자제동.
  // ZS-X11H는 BRAKE가 액티브 하이이므로 PWM을 먼저 끄고 BRAKE를 올린다.
  digitalWrite(LEFT_MOTOR_PWM_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PWM_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LEFT_FORWARD_DIR_LEVEL);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, RIGHT_FORWARD_DIR_LEVEL);
  digitalWrite(LEFT_MOTOR_BRAKE_PIN, HIGH);
  digitalWrite(RIGHT_MOTOR_BRAKE_PIN, HIGH);
  motorOutput = MOTOR_BRAKE;
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  Wire.setClock(100000);

  Serial.println(F("E,BOOT"));

  // 직전 리셋 원인. WDT면 지난 주행에서 loop()가 멈춘 것이니 로그를 볼 것.
  Serial.print(F("E,RST,"));
  if (resetFlags & _BV(WDRF)) Serial.println(F("WDT"));
  else if (resetFlags & _BV(BORF)) Serial.println(F("BROWNOUT"));
  else if (resetFlags & _BV(EXTRF)) Serial.println(F("EXTERNAL"));
  else if (resetFlags & _BV(PORF)) Serial.println(F("POWERON"));
  else Serial.println(F("UNKNOWN"));
  for (uint8_t second = 0; second < INITIAL_SETUP_SECONDS; second++) {
    Serial.print(F("E,CAL,"));
    Serial.println(INITIAL_SETUP_SECONDS - second);
    delay(1000);
  }

  if (bringUpIMU(true)) {
    imuReady = true;
    Serial.println(F("E,IMU_READY"));
    // 축 매핑 확인용. 평지에 세워 둔 상태의 원시 가속도와 장착 오프셋이다.
    // |값|이 16384에 가까운 축이 진짜 수직축이고, 그게 ACC_VERT여야 한다.
    Serial.print(F("E,ACC,"));
    Serial.print(ax);
    Serial.print(',');
    Serial.print(ay);
    Serial.print(',');
    Serial.println(az);
    Serial.print(F("E,PITCH0,"));
    Serial.println(pitchMountOffsetDeg, 2);
  } else {
    imuReady = false;
    reportImuFailure();
    reportI2cLines();  // 선 자체가 살아 있는지 (풀업/단락)
    scanI2cBus();      // 부팅 때 한 번만. 어떤 주소가 살아 있는지 보여준다.
  }

  // 센서별 빔 기하: 노면에 그리는 띠 길이와 전방 주시 거리 (mm)
  for (uint8_t i = 0; i < US_COUNT; i++) {
    Serial.print(F("E,US_BEAM,"));
    Serial.print(i);
    Serial.print(',');
    Serial.print((int)(usBeamFootprintM[i] * 1000.0));
    Serial.print(',');
    Serial.println((int)(usLookAheadM[i] * 1000.0));

    // 이 센서가 노면으로 인정하는 거리 창. 측정값이 이 밖이면 편차가
    // 계산되지 않아 보정도 판정도 시작되지 않는다. 장착 높이/각도가
    // 실제와 다를 때 그 사실이 여기서 바로 드러난다.
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
  updateInputs(now);
  controlMode = imuReady ? MODE_FLAT : MODE_SENSOR_FAULT;
  lastImuAt = now;
  lastInputAt = now;
  lastSlopeAt = now;
  lastControlAt = now;
  lastTelemetryAt = now;
  lastAccelActiveAtMs = now;
  usPingStartedAtMs = now;
  for (uint8_t i = 0; i < US_COUNT; i++) {
    detectors[i].lastSampleAtMs = now;
    detectors[i].riskAtMs = now;
  }
  // 여기서부터 감시 시작. 보정(최대 5초 + IMU 200샘플)이 끝난 뒤에 켜야
  // 한다. 보정 전에 켜면 delay(1000) 구간에서 리셋이 걸려 무한 재부팅이다.
#if WATCHDOG_ENABLED
  wdt_enable(WATCHDOG_TIMEOUT);
#endif

  Serial.println(F("E,READY"));
}

void loop() {
  // 워치독에 밥 주기. loop()가 한 바퀴를 못 돌면 여기에 못 와서 칩이 리셋된다.
#if WATCHDOG_ENABLED
  wdt_reset();
#endif

  unsigned long now = millis();

  if (imuReady && now - lastImuAt >= IMU_INTERVAL_MS) {
    updateIMU(now);
  }

  // IMU가 죽어 있으면 계속 다시 붙여본다. 붙을 때까지는 updateControlMode()가
  // MODE_SENSOR_FAULT로 잡아 두므로 모터는 제동 상태다.
  if (!imuReady && now - lastImuRetryAt >= IMU_RETRY_INTERVAL_MS) {
    lastImuRetryAt = now;
    uint8_t previousStep = imuFailStep;
    if (bringUpIMU(false)) {
      imuReady = true;
      consecutiveImuFailures = 0;
      lastImuAt = millis();
      lastSlopeAt = lastImuAt;
      Serial.println(F("E,IMU_READY"));
    } else if (imuFailStep != previousStep) {
      // 같은 이유로 계속 실패하는 동안은 조용히 있는다(9600 baud).
      reportImuFailure();
    }
  }

  if (now - lastInputAt >= INPUT_INTERVAL_MS) {
    lastInputAt = now;
    updateInputs(now);
  }

  // 초음파는 매 loop마다 상태를 진행시킨다. 블로킹은 트리거 10us뿐.
  // 한 센서의 측정이 끝나면 그 자리에서 노면 위험 판정 한 프레임이 돈다.
  updateUltrasonic(now);

  // 위험 판정 유지시간이 지났는지 확인하고 세 센서를 합친다.
  updateOverallRisk(now);

  // 위험도나 위험원인이 바뀌면 텔레메트리 주기를 기다리지 않고 바로 알린다.
  if ((overallRisk != reportedRisk || overallHazard != reportedHazard)
      && usPhase == US_IDLE) {
    outputHazardLine();
  }

  if (imuReady && now - lastSlopeAt >= SLOPE_INTERVAL_MS) {
    lastSlopeAt = now;
    updateConfirmedSlope(classifySlope(filteredPitchDeg));
  }

  if (now - lastControlAt >= CONTROL_INTERVAL_MS) {
    lastControlAt = now;
    updateControlMode();
    updatePwmRamp();

    bool immediateBrake = controlMode == MODE_SENSOR_FAULT
                          || controlMode == MODE_HANDLE_RELEASED
                          || controlMode == MODE_DOWNHILL_BRAKE
                          || controlMode == MODE_OBSTACLE_BRAKE;

    if (immediateBrake) {
      currentPWM = 0;
      commandMotor(MOTOR_BRAKE, 0);
    } else if (currentPWM > 0) {
      commandMotor(MOTOR_FORWARD, currentPWM);
    } else {
      commandMotor(MOTOR_COAST, 0);
    }

    // 내장 LED: 안전벨트 자석 감지 시 켜짐. 모터 제어에는 영향 없음.
    digitalWrite(LED_BUILTIN, beltFastened ? HIGH : LOW);
  }

  // 초음파 측정이 진행 중이 아닐 때만 보낸다. 에코 타이밍과 겹치지 않게 막는다.
  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS && usPhase == US_IDLE) {
    lastTelemetryAt = now;
    outputTelemetry();
  }
}

// ===================== IMU =====================
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t startReg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(MPU_ADDR, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (uint8_t i = 0; i < length; i++) data[i] = Wire.read();
  return true;
}

// 실패하면 imuFailStep에 어디서 걸렸는지 남긴다. 단계 의미는
// reportImuFailure()의 주석 표를 볼 것.
bool initializeIMU() {
  imuFailStep = 0;
  imuWhoAmI = 0xFF;

  // 레지스터를 쓰기 전에 응답 자체가 있는지 먼저 본다.
  // 여기서 걸리면 배선/주소/전원 문제지 설정 문제가 아니다.
  uint8_t who = 0;
  if (!readRegisters(0x75, &who, 1)) { imuFailStep = 1; return false; }
  imuWhoAmI = who;
  // MPU-6050 = 0x68. MPU-6500/9250 클론은 0x70/0x71/0x73으로도 나온다.
  // 0x00이나 0xFF면 SDA가 계속 눌려 있거나 떠 있는 것이다.
  if (who == 0x00 || who == 0xFF) { imuFailStep = 2; return false; }

  if (!writeRegister(0x6B, 0x80)) { imuFailStep = 3; return false; }  // reset
  delay(100);
  if (!writeRegister(0x6B, 0x01)) { imuFailStep = 4; return false; }  // wake, gyro clock
  if (!writeRegister(0x1A, 0x03)) { imuFailStep = 5; return false; }  // DLPF
  if (!writeRegister(0x1B, 0x00)) { imuFailStep = 6; return false; }  // gyro +/-250 dps
  if (!writeRegister(0x1C, 0x00)) { imuFailStep = 7; return false; }  // accel +/-2g
  delay(100);
  return true;
}

bool readIMU() {
  uint8_t data[14];
  if (!readRegisters(0x3B, data, 14)) return false;

  ax = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  ay = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  az = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  gx = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  gy = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  gz = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
  return true;
}

bool calibrateIMU() {
  const int CALIBRATION_SAMPLES = 200;
  long sumGy = 0;
  float sumPitch = 0.0;
  int validSamples = 0;
  int attempts = 0;

  while (validSamples < CALIBRATION_SAMPLES && attempts < 400) {
    attempts++;
    if (!readIMU()) {
      delay(5);
      continue;
    }

    float forwardG = ACC_FORWARD / ACCEL_SENSITIVITY;
    float sideG = ACC_SIDE / ACCEL_SENSITIVITY;
    float verticalG = ACC_VERT / ACCEL_SENSITIVITY;
    float rawPitch = atan2(forwardG, sqrt(sideG * sideG + verticalG * verticalG))
                     * 180.0 / PI;

    sumGy += GYRO_PITCH;
    sumPitch += rawPitch;
    validSamples++;
    delay(5);
  }

  imuCalibSamples = validSamples;
  if (validSamples < CALIBRATION_SAMPLES) { imuFailStep = 8; return false; }
  gyroYOffsetDps = (sumGy / (float)validSamples) / GYRO_SENSITIVITY;
  pitchMountOffsetDeg = sumPitch / validSamples;

  // 오프셋이 ±90 근처면 전후축에 중력이 실려 있다는 뜻이라, 위 atan2가
  // 접혀서 어느 쪽으로 기울여도 같은 부호가 나온다. 축 매핑이 틀린 것이다.
  if (fabs(pitchMountOffsetDeg) > IMU_AXIS_WARN_DEG) {
    Serial.print(F("E,IMU_AXIS,"));
    Serial.println(pitchMountOffsetDeg, 1);
  }
  return true;
}

// allowCalibration=false면 이미 구해둔 오프셋을 그대로 쓴다. 주행 중
// 재연결에서 다시 보정하면 그때의 기울기를 '수평'으로 굳혀버린다.
bool bringUpIMU(bool allowCalibration) {
  if (!initializeIMU()) return false;
  if (allowCalibration || !imuCalibrated) {
    if (!calibrateIMU()) return false;
    imuCalibrated = true;
  }
  calculateIMUValues();
  filteredPitchDeg = accelPitchDeg;
  return true;
}

// E,IMU_FAIL,<step>,<who_am_i>,<보정샘플수>
//   1 = 0x68이 ACK를 안 한다. 배선(SDA=D20/SCL=D21)/전원/AD0 주소를 볼 것
//   2 = 응답은 오는데 WHO_AM_I가 0x00 또는 0xFF. 버스가 눌렸거나 떠 있다
//   3 = 리셋(0x6B) 쓰기 실패      4 = 웨이크업(0x6B) 쓰기 실패
//   5 = DLPF(0x1A) 쓰기 실패      6 = 자이로 레인지(0x1B) 쓰기 실패
//   7 = 가속도 레인지(0x1C) 쓰기 실패
//   8 = 설정은 됐는데 보정 200샘플을 못 채웠다(간헐 통신 불량)
void reportImuFailure() {
  Serial.print(F("E,IMU_FAIL,"));
  Serial.print(imuFailStep);
  Serial.print(',');
  Serial.print(imuWhoAmI, HEX);
  Serial.print(',');
  Serial.println(imuCalibSamples);
}

// E,I2C_LINE,SDA,<ext>,<pu>,SCL,<ext>,<pu>
// TWI를 잠깐 놓고 두 선을 맨 디지털 입력으로 본다. 각 선을 한 번 LOW로
// 눌렀다 놓고 얼마나 빨리 올라오는지로 판단한다 - 뜬 핀은 기생용량 때문에
// 그냥 읽으면 직전 값이 남아 HIGH로 보이기 때문이다.
//   ext=1        : 풀업 없이도 올라옴 -> 모듈에 전원이 들어와 있고 선도 붙었다
//   ext=0, pu=1  : 외부 풀업 없음 -> 모듈 미연결이거나 모듈 VCC가 안 들어왔다
//   pu=0         : 내부 풀업으로도 안 올라옴 -> 그 선이 GND에 물려 있다
void reportI2cLines() {
  Wire.end();

  pinMode(SDA, OUTPUT);  digitalWrite(SDA, LOW);
  pinMode(SCL, OUTPUT);  digitalWrite(SCL, LOW);
  delayMicroseconds(50);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  delayMicroseconds(50);   // 4.7k 풀업이면 1us 안에 올라온다
  uint8_t sdaExt = digitalRead(SDA);
  uint8_t sclExt = digitalRead(SCL);

  pinMode(SDA, OUTPUT);  digitalWrite(SDA, LOW);
  pinMode(SCL, OUTPUT);  digitalWrite(SCL, LOW);
  delayMicroseconds(50);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(500);  // 내부 풀업은 20~50k라 훨씬 느리다
  uint8_t sdaPu = digitalRead(SDA);
  uint8_t sclPu = digitalRead(SCL);

  Serial.print(F("E,I2C_LINE,SDA,"));
  Serial.print(sdaExt);
  Serial.print(',');
  Serial.print(sdaPu);
  Serial.print(F(",SCL,"));
  Serial.print(sclExt);
  Serial.print(',');
  Serial.println(sclPu);

  Wire.begin();
  Wire.setClock(100000);
}

// E,I2C,<주소...> - 붙어 있는 슬레이브를 16진수로 나열한다.
// 아무것도 안 나오면 SDA/SCL이나 전원이 안 붙은 것이고, 0x69가 나오면
// AD0가 HIGH라 MPU_ADDR을 0x69로 바꿔야 한다.
void scanI2cBus() {
  uint8_t found = 0;
  Serial.print(F("E,I2C"));
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
#if WATCHDOG_ENABLED
    wdt_reset();
#endif
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.print(F(",0x"));
      Serial.print(addr, HEX);
    }
  }
  if (found == 0) Serial.print(F(",NONE"));
  Serial.println();
}

void calculateIMUValues() {
  float forwardG = ACC_FORWARD / ACCEL_SENSITIVITY;
  float sideG = ACC_SIDE / ACCEL_SENSITIVITY;
  float verticalG = ACC_VERT / ACCEL_SENSITIVITY;

  accelPitchDeg = atan2(forwardG, sqrt(sideG * sideG + verticalG * verticalG))
                  * 180.0 / PI - pitchMountOffsetDeg;
  pitchRateDps = (GYRO_PITCH / GYRO_SENSITIVITY) - gyroYOffsetDps;

  if (INVERT_PITCH_DIRECTION) {
    accelPitchDeg = -accelPitchDeg;
    pitchRateDps = -pitchRateDps;
  }
}

void updateIMU(unsigned long now) {
  float dt = (now - lastImuAt) / 1000.0;
  lastImuAt = now;
  if (dt < 0.001) dt = 0.001;
  if (dt > 0.100) dt = 0.100;

  if (!readIMU()) {
    if (consecutiveImuFailures < 255) consecutiveImuFailures++;
    if (consecutiveImuFailures >= 3) {
      imuReady = false;
      Serial.println(F("E,IMU_LOST"));
    }
    return;
  }

  consecutiveImuFailures = 0;
  calculateIMUValues();
  float alpha = FILTER_TIME_CONSTANT_S / (FILTER_TIME_CONSTANT_S + dt);
  filteredPitchDeg = alpha * (filteredPitchDeg + pitchRateDps * dt)
                     + (1.0 - alpha) * accelPitchDeg;

  updateSpeedEstimate(dt, now);
}

// 전후 가속도에서 중력 성분을 빼고 적분해 전진 속도를 추정한다.
// 파이썬 원본이 시리얼로 받던 speed를 대신하는 값이다.
void updateSpeedEstimate(float dt, unsigned long now) {
  float forwardG = ACC_FORWARD / ACCEL_SENSITIVITY;

  // filteredPitchDeg는 장착 오프셋을 뺀(그리고 필요하면 부호를 뒤집은) 값이라
  // 중력 성분을 구하려면 센서가 실제로 보는 기울기로 되돌려야 한다.
  float mountedPitchDeg =
      (INVERT_PITCH_DIRECTION ? -filteredPitchDeg : filteredPitchDeg)
      + pitchMountOffsetDeg;
  float gravityForwardG = sin(mountedPitchDeg * PI / 180.0);

  float rawAccel = (forwardG - gravityForwardG) * GRAVITY_MPS2;
  if (INVERT_PITCH_DIRECTION) rawAccel = -rawAccel;

  // 느린 저역통과로 남은 바이어스(가속도계 오프셋 + 피치 오차)를 추정해 뺀다.
  float beta = dt / (SPEED_BIAS_TC_S + dt);
  accelBiasMps2 += beta * (rawAccel - accelBiasMps2);
  linearAccelMps2 = rawAccel - accelBiasMps2;

  // 이동 여부는 '고주파 진동'으로만 판단한다. 바이어스 제거용 저역통과는
  // 시정수가 길어서 감속 직후 한동안 잔차가 남는데, 그걸 이동으로 오판하지
  // 않도록 빠른 저역통과를 따로 두고 그 차이(진동)만 본다.
  float gamma = dt / (SPEED_VIB_TC_S + dt);
  accelVibLpMps2 += gamma * (rawAccel - accelVibLpMps2);
  float vibration = rawAccel - accelVibLpMps2;
  if (fabs(vibration) > SPEED_MOTION_DEADBAND_MPS2) {
    lastAccelActiveAtMs = now;
  }
  if (now - lastAccelActiveAtMs >= SPEED_ZERO_HOLD_MS) {
    estimatedSpeedMps = 0.0;  // 진동이 없다 = 서 있다
    return;
  }

  estimatedSpeedMps += linearAccelMps2 * dt;
  // 등속 구간에는 가속도에 정보가 없으므로 실측 평균 속도로 수렴시킨다.
  estimatedSpeedMps +=
      (SPEED_NOMINAL_MPS - estimatedSpeedMps) * SPEED_LEAK_PER_S * dt;

  // 유모차는 앞으로만 민다고 보고 음수는 0으로 잘라낸다.
  if (estimatedSpeedMps < 0.0) estimatedSpeedMps = 0.0;
  if (estimatedSpeedMps > SPEED_MAX_MPS) estimatedSpeedMps = SPEED_MAX_MPS;
}

float currentSpeedMps() {
#if SPEED_FROM_IMU
  // IMU가 죽었으면 홈 너비를 과소평가하지 않도록 보수적으로 가정한다.
  return imuReady ? estimatedSpeedMps : SPEED_FALLBACK_MPS;
#else
  return SPEED_FIXED_MPS;
#endif
}

// ===================== Hall + FSR =====================
void updateInputs(unsigned long now) {
  hallRaw = digitalRead(BELT_HALL_PIN);
  bool rawFastened = (hallRaw == BELT_FASTENED_LEVEL);

  if (rawFastened != lastRawBeltFastened) {
    lastRawBeltFastened = rawFastened;
    hallRawChangedAt = now;
  }
  if (rawFastened != beltFastened
      && now - hallRawChangedAt >= BELT_DEBOUNCE_MS) {
    beltFastened = rawFastened;
  }

  // FSR 2개를 각각 히스테리시스로 판정한다.
  // handle_sensor.ino의 10회 평균 + delay(2)는 20ms를 통째로 잡아먹어서
  // 제어 주기를 깨므로 가져오지 않았다.
  fsr1Value = analogRead(HANDLE_FSR1_PIN);
  fsr2Value = analogRead(HANDLE_FSR2_PIN);

  if (!grip1 && fsr1Value >= FSR_GRIP_ON_THRESHOLD) grip1 = true;
  if (grip1 && fsr1Value <= FSR_GRIP_OFF_THRESHOLD) grip1 = false;
  if (!grip2 && fsr2Value >= FSR_GRIP_ON_THRESHOLD) grip2 = true;
  if (grip2 && fsr2Value <= FSR_GRIP_OFF_THRESHOLD) grip2 = false;

#if HANDLE_REQUIRE_BOTH
  handleHeld = grip1 && grip2;
#else
  handleHeld = grip1 || grip2;
#endif
}

// ===================== 초음파 =====================
// 한 번에 한 센서만 쏘고, 에코를 폴링으로 기다린다. pulseIn을 쓰면 센서당
// 최대 30ms를 블로킹해서 3개면 90ms - 20ms 제어 주기가 무너진다.
void updateUltrasonic(unsigned long now) {
  if (usPhase != US_IDLE) return;
  if (now - usPingStartedAtMs < US_PING_INTERVAL_MS) return;
  usPingStartedAtMs = now;

  uint8_t trigPin = US_TRIG_PINS[usIndex];
  uint8_t echoPin = US_ECHO_PINS[usIndex];

  usPhase = US_MEASURING;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 에코는 여기서 끝까지 잰다. 예전에는 상승/하강 대기를 loop() 여러 바퀴에
  // 나눠 폴링했는데, 그 사이에 다른 일(특히 20ms마다 도는 IMU I2C 읽기,
  // 약 1.3ms)이 끼면 하강 엣지를 늦게 보고 그 지연이 그대로 거리 오차가 됐다.
  // 45도 센서의 실제 에코가 1.18ms라 지연과 크기가 같아서, 측정값이 두 갈래로
  // 갈리고 보정이 그 가운데를 기준거리로 잡아 서 있어도 턱/홈이 번갈아 떴다.
  // 측정 간격을 늘려도 소용이 없었던 이유는 IMU 주기의 배수라 충돌 위상이
  // 고정됐기 때문이다.
  //
  // 막는 시간은 보통 1.3ms, 최악이라도 상승 대기 + 에코 한계다. 제어 주기가
  // 20ms이고 핑은 그보다 드물게 나가므로 문제되지 않는다.
  unsigned long trigAt = micros();
  while (digitalRead(echoPin) == LOW) {
    if (micros() - trigAt > US_RISE_TIMEOUT_US) {
      usDistanceMm[usIndex] = 0;  // 센서 응답 없음
      usLastPulseUs[usIndex] = 0;
      usLastFail[usIndex] = 1;
      finishPing();
      return;
    }
  }

  unsigned long echoStart = micros();
  while (digitalRead(echoPin) == HIGH) {
    if (micros() - echoStart > US_MAX_ECHO_US) {
      usDistanceMm[usIndex] = 0;  // 에코가 내려오지 않음
      usLastPulseUs[usIndex] = US_MAX_ECHO_US;
      usLastFail[usIndex] = 2;
      finishPing();
      return;
    }
  }

  unsigned long pulseUs = micros() - echoStart;
  usLastPulseUs[usIndex] = (pulseUs > 65000) ? 65000 : (uint16_t)pulseUs;
  usDistanceMm[usIndex] = pulseToMm(pulseUs);
  usLastFail[usIndex] = (usDistanceMm[usIndex] == 0) ? 3 : 0;
  finishPing();
}

void finishPing() {
  usPhase = US_IDLE;
  // 이 센서의 한 프레임이 완성됐으니 그 자리에서 노면 위험 판정을 돌린다.
  // 파이썬의 for prev, curr in zip(...) 한 바퀴에 해당한다.
  runTerrainDetector(usIndex, usDistanceMm[usIndex], millis());

  usIndex++;
  if (usIndex >= US_COUNT) usIndex = 0;
}

uint16_t pulseToMm(unsigned long pulseUs) {
  if (pulseUs == 0 || pulseUs > US_MAX_ECHO_US) return 0;
  // 음속 343 m/s = 0.343 mm/us, 왕복이므로 절반.
  unsigned long mm = (pulseUs * 343UL) / 2000UL;
  if (mm < US_MIN_VALID_MM || mm > US_MAX_VALID_MM) return 0;
  return (uint16_t)mm;
}

bool obstacleTooClose() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    // 0은 '측정 실패'라서 장애물로 보지 않는다.
    if (usDistanceMm[i] > 0 && usDistanceMm[i] <= US_OBSTACLE_BRAKE_MM) {
      return true;
    }
  }
  return false;
}

// ===================== 노면 위험 판정 (detect.py 이식) =====================
void setupTerrainDetectors() {
  for (uint8_t i = 0; i < US_COUNT; i++) {
    float s = sin(US_TILT_DEG[i] * PI / 180.0);
    if (s < US_MIN_TILT_SIN) s = US_MIN_TILT_SIN;  // 수평 장착 방어
    usSinTilt[i] = s;
    usExpectedFlatM[i] = US_MOUNT_HEIGHT_M[i] / s;  // 평지에서 나와야 할 거리

    // 빔이 노면에 그리는 띠. 각도가 얕아 빔 위쪽이 지평선을 향하면
    // (각도 <= 반각) 노면을 제대로 못 보는 장착이라 크게 잡아 둔다.
    float nearDeg = US_TILT_DEG[i] + US_BEAM_HALF_ANGLE_DEG;
    float farDeg = US_TILT_DEG[i] - US_BEAM_HALF_ANGLE_DEG;
    float nearM = US_MOUNT_HEIGHT_M[i] / tan(nearDeg * PI / 180.0);
    if (farDeg < 1.0) {
      usBeamFootprintM[i] = 10.0;
    } else {
      usBeamFootprintM[i] = US_MOUNT_HEIGHT_M[i] / tan(farDeg * PI / 180.0) - nearM;
    }
    usLookAheadM[i] = US_MOUNT_HEIGHT_M[i] / tan(US_TILT_DEG[i] * PI / 180.0);

    // 평지 기준거리와, 거리 편차를 노면 높이로 바꾸는 계수.
    // 빔의 가장 아래쪽 광선이 최단 거리를 만든다.
    float nearSin = sin(nearDeg * PI / 180.0);
    if (nearSin < US_MIN_TILT_SIN) nearSin = US_MIN_TILT_SIN;
    usBaselineM[i] = US_MOUNT_HEIGHT_M[i] / nearSin;
    usBaselineComputedM[i] = usBaselineM[i];
    usHeightGainM[i] = nearSin;

    // 빔 띠보다 좁은 홈이 만드는 편차의 상한. 빔의 가장 가까운 광선이 홈에
    // 빠지면 측정값이 그 다음으로 가까운 지점(빔 먼 쪽 평지)으로 옮겨가는데,
    // 그 크기가 홈 깊이와 무관하게 이 값으로 고정된다.
    float farSin = sin(farDeg * PI / 180.0);
    if (farDeg < 1.0 || farSin < US_MIN_TILT_SIN) {
      usJumpCapM[i] = 0.0;
    } else {
      usJumpCapM[i] = US_MOUNT_HEIGHT_M[i] * (nearSin / farSin - 1.0);
    }

    resetDetector(i);
  }
}

// 센서별 고정 임계값을 cm에서 m로 옮겨 담는다.
// 턱시작이 복귀 임계보다 낮으면 상태에 들어가자마자 빠져나와 턱을 영영 못
// 잡는다. 우측처럼 임계가 낮은 센서에서 실제로 생기므로 복귀 임계를 턱시작의
// 절반 아래로 눌러 둔다(보정 판에서 쓰던 것과 같은 규칙이다).
void applyFixedThresholds(uint8_t index) {
  DangerDetector &d = detectors[index];
  d.stepDangerM = STEP_DANGER_FIXED_CM[index] / 100.0;
  d.stepEnterM = d.stepDangerM * STEP_ENTER_RATIO;
  d.holeEnterM = HOLE_ENTER_FIXED_CM[index] / 100.0;
  d.exitM = TERRAIN_EXIT_FIXED_CM / 100.0;
  if (d.exitM > d.stepEnterM * 0.5) d.exitM = d.stepEnterM * 0.5;
}

// 판정 상태를 모두 버린다. 기준거리 실측값도 다시 잡는다.
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
  d.noiseM = 0.0;
#if TERRAIN_THRESHOLD_FROM_CALIBRATION
  // 보정이 끝나기 전에도 안전한 쪽으로 동작하도록 하한으로 채워 둔다.
  d.stepEnterM = STEP_ENTER_FLOOR_M;
  d.stepDangerM = STEP_DANGER_FLOOR_M;
  d.holeEnterM = HOLE_ENTER_FLOOR_M;
  d.exitM = STEP_ENTER_FLOOR_M * 0.5;
#else
  applyFixedThresholds(index);
#endif
  d.lastSampleAtMs = millis();
  d.risk = RISK_SAFE;
  d.hazard = HAZARD_NONE;
  d.riskAtMs = d.lastSampleAtMs;
  d.riskWidthM = 0.0;
  d.riskDepthM = 0.0;
}

// 측정 거리(mm) -> 평지 기준 대비 노면 높이 변화(m).
//   + 노면이 내려감 (홈) / - 노면이 올라옴 (턱)
// 노면으로 볼 수 없는 값이면 NAN을 돌려준다(원본의 None에 해당).
float terrainDeviationM(uint8_t index, uint16_t distanceMm) {
  if (distanceMm == 0) return NAN;

  float distanceM = distanceMm / 1000.0;
  if (distanceM < usExpectedFlatM[index] * US_GROUND_MIN_RATIO
      || distanceM > usExpectedFlatM[index] * US_GROUND_MAX_RATIO) {
    return NAN;
  }
  return (distanceM - usBaselineM[index]) * usHeightGainM[index];
}

// 세 값의 중앙값. 한 프레임짜리 헛에코를 걸러낸다.
static uint16_t medianOf3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// 센서 한 개분의 노면 판정. 절대 기준거리에서 벗어난 방향으로 종류를 먼저
// 가르고(위 판정값 주석 참고), 상태별 하위 판정으로 확정한다.
void runTerrainDetector(uint8_t index, uint16_t distanceMm, unsigned long now) {
  DangerDetector &d = detectors[index];

  float interval = (now - d.lastSampleAtMs) / 1000.0 + d.pendingIntervalS;
  d.lastSampleAtMs = now;
  d.pendingIntervalS = 0.0;

  // 측정이 오래 끊겼으면 이어서 볼 근거가 없다. 상태를 버린다.
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

  uint16_t rawMm = distanceMm;   // 진단용으로 필터 전 값을 남겨 둔다
  (void)rawMm;

#if US_MEDIAN_FILTER
  // 유효 측정만 밀어 넣고, 3개가 모이면 중앙값을 판정에 쓴다.
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

#if US_RAW_DEBUG
  if (US_RAW_DEBUG_SENSOR == 255 || index == US_RAW_DEBUG_SENSOR) {
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
    // 측정 실패. 이번 프레임은 판정하지 않는다.
#if US_KEEP_PREV_ON_DROPOUT
    d.pendingIntervalS = interval;  // 놓친 시간은 다음 유효 프레임에 넘긴다
#else
    d.state = TERRAIN_IDLE;
#endif
    // 중앙값 버퍼는 그대로 둔다. 에코가 한 번 빠진 것이지 노면이 바뀐 것이
    // 아니고, 여기서 버리면 다음 두 프레임이 필터를 거치지 않은 채 판정에
    // 들어간다. 시뮬레이션에서 오경보의 대부분이 그 경로였다(75 -> 18/400).
    return;
  }

  // 부팅 직후에는 평지를 달린다고 보고 기준거리와 잡음을 실측으로 잡는다.
  if (d.baselineCount < US_BASELINE_SAMPLES) {
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
        // 평지가 아니다. 앞에 물체가 있거나 장착이 설정과 다르다.
        // 받아들이지 말고 처음부터 다시 잰다.
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

      // 실측 기준거리에서 빔이 실제로 노면을 보는 각도를 역산한다.
      //     sin(유효각) = 설치높이 / 실측 기준거리
      // 거리 편차를 노면 높이로 바꾸는 계수가 이 각도에 걸려 있어서,
      // 장착 오차만큼 계산값을 그대로 쓰면 편차가 부풀거나 줄어든다.
      float effSin = US_MOUNT_HEIGHT_M[index] / mean;
      if (effSin > 1.0) effSin = 1.0;
      if (effSin < 0.05) effSin = 0.05;
      usHeightGainM[index] = effSin;

      // 잡음을 노면 높이로 환산한다. 임계값 산출과 별개로 기록해 둔다.
      d.noiseM = sqrt(var) * usHeightGainM[index];

#if TERRAIN_THRESHOLD_FROM_CALIBRATION
      // 임계값을 잡음의 배수로 만든다.
      d.stepEnterM = d.noiseM * STEP_ENTER_SIGMA;
      d.stepDangerM = d.noiseM * STEP_DANGER_SIGMA;
      d.holeEnterM = d.noiseM * HOLE_ENTER_SIGMA;

      if (d.stepEnterM < STEP_ENTER_FLOOR_M) d.stepEnterM = STEP_ENTER_FLOOR_M;
      if (d.stepDangerM < STEP_DANGER_FLOOR_M) d.stepDangerM = STEP_DANGER_FLOOR_M;
      if (d.holeEnterM < HOLE_ENTER_FLOOR_M) d.holeEnterM = HOLE_ENTER_FLOOR_M;

      // 경사로 진입이 만드는 겉보기 턱보다는 위에 둬야 한다.
      float rampFloor = usLookAheadM[index]
                        * tan(RAMP_MAX_DEG * PI / 180.0)
                        * STEP_DANGER_RAMP_MARGIN;
      if (d.stepDangerM < rampFloor) d.stepDangerM = rampFloor;

      // 홈은 기하 상한 위로 올려서 '뜨면 빔 띠보다 넓은 홈'을 보장한다.
      float capFloor = usJumpCapM[index] * HOLE_ENTER_CAP_MARGIN;
      if (d.holeEnterM < capFloor) d.holeEnterM = capFloor;

      if (d.stepEnterM > TERRAIN_THRESHOLD_CEIL_M) d.stepEnterM = TERRAIN_THRESHOLD_CEIL_M;
      if (d.stepDangerM > TERRAIN_THRESHOLD_CEIL_M) d.stepDangerM = TERRAIN_THRESHOLD_CEIL_M;
      if (d.holeEnterM > TERRAIN_THRESHOLD_CEIL_M) d.holeEnterM = TERRAIN_THRESHOLD_CEIL_M;
      if (d.stepDangerM < d.stepEnterM) d.stepDangerM = d.stepEnterM;

      d.exitM = d.noiseM * TERRAIN_EXIT_SIGMA;
      if (d.exitM > d.stepEnterM * 0.5) d.exitM = d.stepEnterM * 0.5;
#else
      // 센서별 고정값을 그대로 쓴다. 기준거리만 실측으로 잡는다.
      applyFixedThresholds(index);
#endif

      // 이 센서의 보정 결과를 그 자리에서 알린다 (mm).
      // 예전에는 세 센서가 전부 끝나야 loop()에서 한 번에 냈는데, 그러면
      // 하나가 배선이 빠졌거나 노면을 안 보고 있을 때 나머지 둘이 멀쩡히
      // 보정돼도 한 줄도 나오지 않았다. 센서별로 내면 어디까지 됐는지 보인다.
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

      // 보정 구간이 이미 심하게 흔들렸다면 기준거리도 임계값도 믿을 게 못
      // 된다. 측정을 먼저 봐야 하므로 알려 준다.
      if (d.noiseM > US_BASELINE_MAX_NOISE_M) {
        Serial.print(F("E,US_NOISY,"));
        Serial.print(index);
        Serial.print(',');
        Serial.println((int)(d.noiseM * 1000.0));
      }
    }
    return;  // 보정이 끝날 때까지는 판정하지 않는다
  }

  RiskLevel risk = RISK_SAFE;
  HazardCause hazard = HAZARD_NONE;
  float eventWidthM = 0.0;
  float eventDepthM = 0.0;

  switch (d.state) {
    case TERRAIN_IDLE:
      // 같은 방향이 연속으로 보여야 상태를 바꾼다. 발처럼 빔에 걸쳤다
      // 빠졌다 하는 것이 한 프레임씩 튀며 턱과 홈을 번갈아 만드는 것을 막는다.
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
        // 빔 띠가 safe_gap보다 길어서, 보이기 시작한 홈은 이미 바퀴가
        // 빠지는 크기다. 표를 여기까지 셌다는 것은 홈 조건이 그만큼 연속으로
        // 반복됐다는 뜻이므로 이 자리에서 알린다.
        d.state = TERRAIN_HOLE;
        d.holeDepthM = dev;
        d.holeWidthM = usBeamFootprintM[index];
        risk = RISK_DANGER;
        hazard = HAZARD_HOLE;
        eventWidthM = d.holeWidthM;
        eventDepthM = d.holeDepthM;
        d.holeVotes = 0;
      } else if (d.stepVotes >= TERRAIN_CONFIRM_FRAMES) {
        // 턱일 수도, 경사로 진입일 수도 있다. 크기를 보고 정한다.
        d.state = TERRAIN_STEP;
        d.stepPeakM = -dev;
        d.stepVotes = 0;
      }
      break;

    case TERRAIN_HOLE:
      d.holeWidthM += currentSpeedMps() * interval;   // 앱에 보낼 참고값
      if (dev > d.holeDepthM) d.holeDepthM = dev;
      if (dev < d.exitM) {                            // 노면 복귀
        d.state = TERRAIN_IDLE;
        d.holeWidthM = 0.0;
        d.holeDepthM = 0.0;
        d.dangerVotes = 0;
      }
      break;

    case TERRAIN_STEP:
      if (-dev > d.stepPeakM) d.stepPeakM = -dev;

      // 최대값(stepPeakM)은 한 번 올라가면 내려오지 않아서, 헛에코 한 번이
      // 임계를 넘기면 그대로 확정돼 버린다. 그래서 확정은 최대값이 아니라
      // '이번 프레임의 편차'가 임계를 연속으로 넘는지로 센다.
      if (-dev > d.stepDangerM) {
        if (d.dangerVotes < 255) d.dangerVotes++;
      } else {
        d.dangerVotes = 0;
      }

      if (d.dangerVotes >= TERRAIN_DANGER_CONFIRM_FRAMES) {
        risk = RISK_DANGER;
        hazard = HAZARD_STEP;
        eventDepthM = d.stepPeakM;                    // 턱 높이
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
        d.dangerVotes = 0;
      } else if (-dev < d.exitM) {
        // 노면이 돌아왔고 크기가 작았다 = 경사로 진입이나 잔요철
        d.state = TERRAIN_IDLE;
        d.stepPeakM = 0.0;
        d.dangerVotes = 0;
      }
      break;
  }

  // 한 프레임짜리 판정이라 그대로 두면 텔레메트리에서 놓친다.
  // 더 높은 위험이면 갱신하고, 같은 위험이면 유지시간만 늘린다.
  if (risk > d.risk) {
    d.risk = risk;
    d.hazard = hazard;
    d.riskAtMs = now;
    d.riskWidthM = eventWidthM;
    d.riskDepthM = eventDepthM;
  } else if (risk != RISK_SAFE && risk == d.risk) {
    // 같은 등급이면 유지시간만 늘리고 원인은 그대로 둔다. 빔에 걸친 물체가
    // 턱과 홈을 오갈 때 앱으로 나가는 원인이 계속 뒤집히는 것을 막는다.
    d.riskAtMs = now;
  }
}

// 세 센서 중 가장 높은 위험도를 앱으로 보낼 값으로 삼는다.
// 원본의 max(max_risk, risk, key=lambda x: x[0].value)에 해당한다.
void updateOverallRisk(unsigned long now) {
  RiskLevel best = RISK_SAFE;
  HazardCause bestHazard = HAZARD_NONE;
  uint8_t bestSensor = 255;

  for (uint8_t i = 0; i < US_COUNT; i++) {
    // 유지시간이 지난 판정은 내린다.
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

// 위험도/위험원인이 바뀐 순간 앱으로 바로 나가는 한 줄.
void outputHazardLine() {
  uint8_t s = overallRiskSensor;
  uint16_t distanceMm = (s < US_COUNT) ? usDistanceMm[s] : 0;
  // 판정 당시의 값이다. STAIR/EXIT_STEP은 판정과 동시에 홈 상태를 지우므로
  // 현재 상태를 읽으면 항상 0이 나간다.
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

#if HUMAN_READABLE_LOG
  outputHumanLine();
#endif

  reportedRisk = overallRisk;
  reportedHazard = overallHazard;
}

#if HUMAN_READABLE_LOG
// 시리얼 모니터에서 눈으로 확인하는 줄. '#'로 시작하므로 파서는 무시한다.
void outputHumanLine() {
  static const char *RISK_NAME[] = { "SAFE  ", "CAUTION", "DANGER" };
  static const char *HAZARD_NAME[] = { "CLEAR", "STEP ", "HOLE " };
  static const char SENSOR_NAME[] = { 'L', 'C', 'R' };

  Serial.print(F("# "));
  Serial.print(millis() / 1000.0, 3);
  Serial.print(F("s "));
  Serial.print(RISK_NAME[overallRisk]);
  Serial.print(' ');
  Serial.print(HAZARD_NAME[overallHazard]);

  uint8_t s = overallRiskSensor;
  if (s >= US_COUNT) {
    Serial.println();
    return;
  }

  Serial.print(' ');
  Serial.print(SENSOR_NAME[s]);
  Serial.print(F(" dist="));
  Serial.print(usDistanceMm[s]);
  if (overallHazard == HAZARD_HOLE) {
    Serial.print(F("mm width="));
    Serial.print((int)(detectors[s].riskWidthM * 1000.0));
    Serial.print(F("mm depth="));
    Serial.print((int)(detectors[s].riskDepthM * 1000.0));
    Serial.println(F("mm"));
  } else {
    Serial.print(F("mm height="));
    Serial.print((int)(detectors[s].riskDepthM * 1000.0));
    Serial.println(F("mm"));
  }
}
#endif

// ===================== 판단 + 제어 =====================
SlopeState classifySlope(float pitchDeg) {
  if (pitchDeg >= SLOPE_THRESHOLD_DEG) return SLOPE_UP;
  if (pitchDeg <= -SLOPE_THRESHOLD_DEG) return SLOPE_DOWN;
  if (fabs(pitchDeg) <= FLAT_THRESHOLD_DEG) return SLOPE_FLAT;
  return SLOPE_UNCERTAIN;
}

void updateConfirmedSlope(SlopeState candidate) {
  if (candidate == previousSlopeCandidate) {
    if (slopeStableCount < 255) slopeStableCount++;
  } else {
    previousSlopeCandidate = candidate;
    slopeStableCount = 1;
  }

  if (slopeStableCount >= REQUIRED_SLOPE_COUNT) {
    slopeState = candidate;
    slopeStableCount = REQUIRED_SLOPE_COUNT;
  }
}

void updateControlMode() {
  targetPWM = 0;

  if (!imuReady) {
    controlMode = MODE_SENSOR_FAULT;
  } else if (!handleHeld) {
    controlMode = MODE_HANDLE_RELEASED;
#if US_OBSTACLE_BRAKE_ENABLED
  } else if (obstacleTooClose()) {
    controlMode = MODE_OBSTACLE_BRAKE;
#endif
  } else if (slopeState == SLOPE_DOWN) {
    controlMode = MODE_DOWNHILL_BRAKE;
  } else if (slopeState == SLOPE_UP) {
    controlMode = MODE_UPHILL_ASSIST;
    targetPWM = calculateSlopePWM(filteredPitchDeg);
  } else if (slopeState == SLOPE_FLAT) {
    controlMode = MODE_FLAT;
  } else {
    controlMode = MODE_UNCERTAIN;
  }
}

uint8_t calculateSlopePWM(float pitchDeg) {
  if (pitchDeg <= PWM_START_SLOPE_DEG) return PWM_MIN_DRIVE;
  if (pitchDeg >= PWM_MAX_SLOPE_DEG) return PWM_MAX_ASSIST;

  float ratio = (pitchDeg - PWM_START_SLOPE_DEG)
                / (PWM_MAX_SLOPE_DEG - PWM_START_SLOPE_DEG);
  return (uint8_t)(PWM_MIN_DRIVE
                   + ratio * (PWM_MAX_ASSIST - PWM_MIN_DRIVE));
}

void updatePwmRamp() {
  if (currentPWM < targetPWM) {
    int nextPWM = currentPWM + PWM_RISE_STEP;
    currentPWM = nextPWM > targetPWM ? targetPWM : (uint8_t)nextPWM;
  } else if (currentPWM > targetPWM) {
    int nextPWM = currentPWM - PWM_FALL_STEP;
    currentPWM = nextPWM < targetPWM ? targetPWM : (uint8_t)nextPWM;
  }
}

void commandMotor(MotorOutput requestedOutput, uint8_t pwm) {
#if !MOTOR_OUTPUT_ENABLED
  requestedOutput = MOTOR_COAST;
  pwm = 0;
#endif

  // 출력 모드가 바뀔 때 PWM을 먼저 0으로 떨어뜨려 전환 충돌을 줄인다.
  // BRAKE는 여기서 건드리지 않는다. 제동 중에 한순간이라도 풀리면 안 된다.
  if (requestedOutput != motorOutput) {
    digitalWrite(LEFT_MOTOR_PWM_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_PWM_PIN, LOW);
    delayMicroseconds(MOTOR_TRANSITION_BLANK_US);
    motorOutput = requestedOutput;
  }

  writeOneMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_BRAKE_PIN,
                LEFT_FORWARD_DIR_LEVEL, requestedOutput, pwm);
  writeOneMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_BRAKE_PIN,
                RIGHT_FORWARD_DIR_LEVEL, requestedOutput, pwm);
}

void writeOneMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin,
                   uint8_t forwardDirLevel, MotorOutput output, uint8_t pwm) {
  if (output == MOTOR_FORWARD) {
    // 제동을 먼저 풀고 방향을 정한 뒤 속도를 준다.
    digitalWrite(brakePin, LOW);
    digitalWrite(dirPin, forwardDirLevel);
    analogWrite(pwmPin, pwm);
  } else if (output == MOTOR_BRAKE) {
    // 속도를 먼저 끊고 제동을 건다.
    // digitalWrite가 analogWrite로 붙어 있던 PWM 타이머를 떼어낸다.
    digitalWrite(pwmPin, LOW);
    digitalWrite(brakePin, HIGH);
  } else {
    // 코스트: 출력도 제동도 없이 자유 회전.
    digitalWrite(pwmPin, LOW);
    digitalWrite(brakePin, LOW);
  }
}

// ===================== 텔레메트리 =====================
// 고정 순서 CSV 한 줄. 파일 상단의 포맷 설명과 반드시 함께 고칠 것.
// 최악의 경우 약 64바이트라 9600 baud에서 마지막 몇 바이트가 송신 버퍼를
// 기다릴 수 있지만, 그래도 1-2ms 수준이라 제어 주기에는 영향이 없다.
void outputTelemetry() {
  Serial.print(F("T,3,"));
  Serial.print(telemetrySeq);
  Serial.print(',');
  Serial.print(filteredPitchDeg, 2);
  Serial.print(',');
  Serial.print((uint8_t)slopeState);
  Serial.print(',');
  Serial.print(fsr1Value);
  Serial.print(',');
  Serial.print(fsr2Value);
  Serial.print(',');
  Serial.print(handleHeld ? 1 : 0);
  Serial.print(',');
  Serial.print(beltFastened ? 1 : 0);
  Serial.print(',');
  Serial.print((uint8_t)controlMode);
  Serial.print(',');
  Serial.print(currentPWM);
  Serial.print(',');
  Serial.print((uint8_t)motorOutput);
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
