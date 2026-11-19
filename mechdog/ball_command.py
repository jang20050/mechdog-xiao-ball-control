# MechDog: XIAO ESP32-S3 가 무선(ESP-NOW)으로 보낸 공 색 판단으로 움직이기
#   빨간 공 → 정지,  파란 공 → 전진 (공이 사라져도 마지막 명령 유지)
#   MechDog 의 AI 비전 모듈(IIC(2) 0x53)은 그대로 연결해 둬도 됨 (이 프로그램은 사용 안 함)
#
# XIAO(xiao_ball_eye 펌웨어)는 채널 1 로 브로드캐스트. 10바이트 패킷:
#   "BEYE" + [판단 0=없음/1=빨강/2=파랑, 신뢰도%, x%, y%, 크기‰, 프레임번호]
#
# 안전
#   - XIAO 신호가 LOST_MS 동안 안 오면 → 정지 (보라 눈)
#   - 머리 초음파 STOP_CM 이내 장애물 → 멈춰 있다가 CLEAR_CM 보다 멀어지면 다시 전진 (주황 눈)
#
# 업로드: python -m mpremote connect COM7 resume cp mechdog/ball_command.py :main.py → 리셋
import time
import network
import espnow
import Hiwonder_IIC
from HW_MechDog import MechDog

CHANNEL = 1         # XIAO 와 같은 채널
XIAO_MAC = None     # 특정 XIAO 만 받으려면 b'\xe8\xf6\x0a\x8b\xe3\x40' (None = "BEYE" 패킷이면 모두)
STRIDE = 70         # 전진 보폭(mm). 클수록 빠름 (최대 100)
TRIM = 0            # 직진 회전 보정 (양수 = 왼쪽)
MIN_CONF = 20       # 이보다 낮은 신뢰도의 판단은 무시
LOST_MS = 1000      # XIAO 신호가 이만큼 없으면 정지
STOP_CM = 25        # 초음파 장애물 정지 거리 (0 = 사용 안 함)
CLEAR_CM = 35
LOOP_S = 0.05

mechdog = MechDog()
i2c1 = Hiwonder_IIC.IIC(1)
try:
    sonar = Hiwonder_IIC.I2CSonar(i2c1)
except Exception:
    sonar = None

_eye = [None]


def eyes(r, g, b):
    """눈 색이 바뀔 때만 I2C 로 보냄"""
    if sonar and _eye[0] != (r, g, b):
        try:
            sonar.setRGB(0, r, g, b)
            _eye[0] = (r, g, b)
        except Exception:
            pass


def distance_cm():
    if not sonar or not STOP_CM:
        return 0
    try:
        d = sonar.getDistance()
        return d if 0 < d < 400 else 0
    except Exception:
        return 0


# ---- 무선 준비 ----
sta = network.WLAN(network.STA_IF)
sta.active(True)
try:
    sta.disconnect()
except Exception:
    pass
try:
    sta.config(channel=CHANNEL)
except Exception as e:
    print("채널 설정 실패:", e)
esp = espnow.ESPNow()
esp.active(True)
print("MechDog MAC:", ":".join("%02X" % b for b in sta.config("mac")))


def latest_packet():
    """쌓여 있는 패킷 중 가장 최근 것 (없으면 None)"""
    last = None
    while True:
        try:
            mac, msg = esp.irecv(0)
        except Exception:
            break
        if msg is None:
            break
        if len(msg) == 10 and msg[:4] == b"BEYE" and (XIAO_MAC is None or mac == XIAO_MAC):
            last = bytes(msg[4:])
    return last


# ===================== 시작 =====================
mechdog.set_default_pose()
eyes(0, 0, 255)
time.sleep(1)

want = None            # 공이 내린 마지막 명령: None(대기) / "stop" / "walk"
moving = False         # 실제로 걷는 중인지
blocked = False
near = clear = 0
last_rx = None         # 마지막으로 패킷을 받은 시각
lost = True
print("준비 완료 - XIAO 신호를 기다립니다 (파란 공 = 전진, 빨간 공 = 정지)")


def go(walk):
    global moving
    if walk and not moving:
        mechdog.move(STRIDE, TRIM)
        moving = True
    elif not walk and moving:
        mechdog.move(0, 0)
        moving = False


while True:
    now = time.ticks_ms()
    p = latest_packet()
    if p is not None:
        last_rx = now

    # ---- XIAO 신호 끊김 ----
    if last_rx is None or time.ticks_diff(now, last_rx) > LOST_MS:
        if not lost:
            lost = True
            want = None
            go(False)
            print("XIAO 신호 없음 - 정지")
        eyes(255, 0, 255)
        time.sleep(LOOP_S)
        continue
    if lost:
        lost = False
        print("XIAO 신호 받음")

    # ---- 공 색 → 명령 ----
    if p is not None and p[1] >= MIN_CONF:
        if p[0] == 1 and want != "stop":
            want = "stop"
            print("빨간 공 (신뢰도 %d%%) → 정지" % p[1])
        elif p[0] == 2 and want != "walk":
            want = "walk"
            print("파란 공 (신뢰도 %d%%) → 전진" % p[1])

    # ---- 초음파 장애물 ----
    d = distance_cm()
    near = near + 1 if 0 < d < STOP_CM else 0
    clear = 0 if 0 < d < CLEAR_CM else clear + 1
    if not blocked and near >= 2:
        blocked = True
        print("장애물 %dcm - 정지" % d)
    elif blocked and clear >= 5:
        blocked = False
        print("장애물 사라짐")

    # ---- 동작 ----
    walk = want == "walk" and not blocked
    go(walk)
    if walk:
        eyes(0, 255, 0)     # 초록 = 전진
    elif want == "walk" and blocked:
        eyes(255, 120, 0)   # 주황 = 장애물 때문에 대기
    elif want == "stop":
        eyes(255, 0, 0)     # 빨강 = 빨간 공 정지
    else:
        eyes(0, 0, 255)     # 파랑 = 신호는 받는 중, 명령 대기

    time.sleep(LOOP_S)
