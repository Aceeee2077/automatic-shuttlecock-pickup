"""
方案 22：Web 远程监控。

用 Python 标准库 http.server 实现（无第三方依赖）：
- GET /        ：监控页面（单文件 HTML，内嵌状态轮询 + MJPEG 显示）
- GET /video   ：MJPEG 视频流
- GET /status  ：状态 JSON（动作 / 捡球计数 / 电量 / 避障状态）

main.py 每帧调用 update(display_frame, status_dict) 写入最新帧和状态，
推流线程只在浏览器请求时编码 JPEG，不影响主循环实时性。
浏览器打开 http://<本机IP>:8000 即可远程查看。
"""

import io
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class WebMonitor:
    def __init__(self, port=8000, jpeg_quality=70):
        self.port = port
        self.jpeg_quality = jpeg_quality

        # 最新一帧（编码后的 JPEG bytes）和状态字典，主循环写入，HTTP 线程读取
        self._jpeg = None
        self._status = {}
        self._lock = threading.Lock()

        self._server = None
        self._thread = None

    # ---- 主循环调用 ----

    def update(self, frame, status):
        """写入最新显示帧（BGR numpy 数组）和状态字典。

        JPEG 编码在这里完成（主循环线程），HTTP 线程直接取编码结果，
        避免多线程同时碰 cv2。
        """
        import cv2

        ok, buf = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )

        if not ok:
            return

        with self._lock:
            self._jpeg = buf.tobytes()
            self._status = dict(status)

    def start(self):
        """启动 HTTP 服务线程（守护线程，主进程退出自动结束）。"""
        monitor = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # 不打印访问日志，避免刷屏

            def _send(self, content_type, body):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/":
                    self._send("text/html; charset=utf-8", PAGE_HTML.encode("utf-8"))
                elif self.path == "/video":
                    self._stream_video()
                elif self.path == "/status":
                    with monitor._lock:
                        body = json.dumps(monitor._status).encode("utf-8")
                    self._send("application/json", body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def _stream_video(self):
                """MJPEG 推流：multipart/x-mixed-replace，每帧 --frameboundary 分隔。"""
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frameboundary"
                )
                self.end_headers()

                try:
                    while True:
                        with monitor._lock:
                            jpeg = monitor._jpeg

                        if jpeg is None:
                            # 还没有画面时稍等再试
                            threading.Event().wait(0.2)
                            continue

                        self.wfile.write(b"--frameboundary\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")

                        # 约 15fps 的推送节奏
                        threading.Event().wait(0.066)
                except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                    pass  # 浏览器断开，正常退出

        self._server = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"Web monitor: http://0.0.0.0:{self.port}")

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None


# 监控页面：轮询 /status 刷新状态栏，/video 显示画面
PAGE_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>羽毛球捡球机器人监控</title>
<style>
  body { margin: 0; background: #111; color: #eee;
         font-family: Consolas, monospace; text-align: center; }
  h1 { font-size: 20px; padding: 12px; }
  img { max-width: 96%; border: 1px solid #333; }
  #bar { display: flex; justify-content: center; gap: 28px;
         padding: 10px; font-size: 17px; }
  .ok { color: #6f6; } .warn { color: #fd5; } .bad { color: #f55; }
</style>
</head>
<body>
<h1>羽毛球捡球机器人远程监控</h1>
<img src="/video" alt="视频流">
<div id="bar">
  <span id="mode">-</span>
  <span id="action">-</span>
  <span id="collected">-</span>
  <span id="batt">-</span>
  <span id="obstacle">-</span>
  <span id="miss">-</span>
</div>
<script>
function clsBatt(mv) {
  if (mv == null) return "ok";
  if (mv <= 10000) return "bad";
  if (mv <= 10500) return "warn";
  return "ok";
}
function clsObstacle(s) {
  if (s === "blocked" || s === "giveup") return "bad";
  return "ok";
}
async function poll() {
  try {
    const r = await fetch("/status");
    const s = await r.json();
    document.getElementById("mode").textContent = "模式: " + (s.mode || "-");
    document.getElementById("action").textContent = "动作: " + (s.action || "-");
    document.getElementById("collected").textContent =
      "已捡: " + (s.collected ?? 0) + " 个";
    const batt = document.getElementById("batt");
    if (s.battery_mv == null) {
      batt.textContent = "电量: 未知";
      batt.className = "ok";
    } else {
      batt.textContent = "电量: " + (s.battery_mv / 1000).toFixed(2) + "V";
      batt.className = clsBatt(s.battery_mv);
    }
    const obs = document.getElementById("obstacle");
    obs.textContent = "避障: " + (s.obstacle_state || "clear")
      + (s.obstacle ? "(" + s.obstacle + ")" : "");
    obs.className = clsObstacle(s.obstacle_state);
    document.getElementById("miss").textContent =
      "丢帧: " + (s.miss_count ?? 0);
  } catch (e) { /* 服务未就绪时静默重试 */ }
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>"""
