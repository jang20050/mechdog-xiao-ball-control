"""XIAO(COM6) 공 색 학습 웹 — PC 에서 실행

실행:  python pc_web/server.py [COM6] [8000]
브라우저: http://localhost:8000

- XIAO 와 USB 시리얼로 통신 (화면 JPEG, 상태, 학습 명령)
- "YOLO 자동 학습": YOLO26 이 화면에서 공(COCO sports ball)을 찾아 그 위치의 색을 XIAO 에 학습시킴
  (처음 한 번 yolo26n.pt 를 내려받음. 실패해도 클릭 학습은 그대로 사용 가능)
"""
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
HTTP_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
HERE = Path(__file__).resolve().parent
W, H = 320, 240
BALL_CLASS = 32  # COCO "sports ball"


class Xiao:
    """XIAO 시리얼 명령: 한 번에 한 명령씩 (스레드 안전)"""

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.lock = threading.Lock()

    def _open(self):
        s = serial.Serial()
        s.port = self.port
        s.baudrate = 115200
        s.timeout = 2
        s.dtr = False  # DTR/RTS 를 건드리면 XIAO 가 리셋될 수 있음
        s.rts = False
        s.open()
        self.ser = s

    def cmd(self, line):
        with self.lock:
            for attempt in range(2):
                try:
                    if self.ser is None:
                        self._open()
                    self.ser.reset_input_buffer()
                    self.ser.write(line.encode() + b"\n")
                    deadline = time.time() + 3
                    while time.time() < deadline:
                        hdr = self.ser.readline()
                        if hdr.startswith(b"@"):
                            kind, n = hdr[1:].split()
                            data = self.ser.read(int(n))
                            return kind.decode(), data
                    raise TimeoutError("XIAO 응답 없음")
                except Exception:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    if attempt:
                        raise
                    time.sleep(0.5)

    def json(self, line):
        kind, data = self.cmd(line)
        return json.loads(data.decode("utf-8"))


xiao = Xiao(PORT)


# ---------------- YOLO26 (선택) ----------------
_yolo = {"model": None, "error": None, "loading": False}


def get_yolo():
    if _yolo["model"] is None and _yolo["error"] is None:
        try:
            from ultralytics import YOLO

            _yolo["model"] = YOLO(str(HERE / "yolo26n.pt") if (HERE / "yolo26n.pt").exists() else "yolo26n.pt")
        except Exception as e:  # 인터넷이 없거나 ultralytics 문제
            _yolo["error"] = str(e)
    return _yolo["model"]


def yolo_learn(cls):
    """화면에서 가장 큰 공을 찾아 그 안의 5지점을 cls 색으로 학습"""
    import cv2
    import numpy as np

    model = get_yolo()
    if model is None:
        return {"msg": "YOLO 를 불러오지 못했습니다: %s" % _yolo["error"]}
    kind, data = xiao.cmd("F0")
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    res = model.predict(img, imgsz=320, conf=0.25, classes=[BALL_CLASS], verbose=False)[0]
    if res.boxes is None or len(res.boxes) == 0:
        return {"msg": "YOLO 가 화면에서 공을 찾지 못했습니다. 공을 더 가까이 보여 주세요", "box": None}
    xyxy = res.boxes.xyxy.cpu().numpy()
    areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
    i = int(areas.argmax())
    x0, y0, x1, y1 = xyxy[i]
    conf = float(res.boxes.conf[i])
    cx, cy, rw, rh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    pts = [(cx, cy), (cx - rw * 0.4, cy), (cx + rw * 0.4, cy), (cx, cy - rh * 0.4), (cx, cy + rh * 0.4)]
    msg = ""
    for px, py in pts:
        px, py = int(min(W - 1, max(0, px))), int(min(H - 1, max(0, py)))
        msg = xiao.json("L %d %d %d" % (cls, px, py))["msg"]
    return {
        "msg": "YOLO 공 발견 (%.0f%%) → %s" % (conf * 100, msg),
        "box": [float(x0), float(y0), float(x1), float(y1)],
    }


# ---------------- HTTP ----------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, obj, code=200):
        self.send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        num = lambda k, d=-1: int(float(q.get(k, d)))
        try:
            if u.path == "/":
                self.send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif u.path == "/frame.jpg":
                kind, data = xiao.cmd("F1" if q.get("mask") == "1" else "F0")
                if kind != "F":
                    return self.send_json(json.loads(data), 500)
                self.send(200, data, "image/jpeg")
            elif u.path == "/api/status":
                st = xiao.json("S")
                st["port"] = PORT
                st["yolo"] = "ready" if _yolo["model"] else ("error" if _yolo["error"] else "not loaded")
                self.send_json(st)
            elif u.path == "/api/learn":
                self.send_json(xiao.json("L %d %d %d" % (num("c"), num("x"), num("y"))))
            elif u.path == "/api/set":
                if "minArea" in q:
                    self.send_json(xiao.json("A %d" % num("minArea")))
                else:
                    self.send_json(xiao.json("T %d %d %d %d %d" % (num("c"), num("hc"), num("tol"), num("smin"), num("vmin"))))
            elif u.path == "/api/reset":
                self.send_json(xiao.json("R %d" % num("c")))
            elif u.path == "/api/save":
                self.send_json(xiao.json("W"))
            elif u.path == "/api/yolo":
                self.send_json(yolo_learn(num("c")))
            else:
                self.send(404, b"not found", "text/plain")
        except Exception as e:
            self.send_json({"msg": "XIAO(%s) 통신 오류: %s" % (PORT, e)}, 503)


if __name__ == "__main__":
    print("XIAO 포트: %s" % PORT)
    print("브라우저에서 http://localhost:%d 를 여세요 (Ctrl+C 로 종료)" % HTTP_PORT)
    ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
