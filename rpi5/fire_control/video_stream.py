"""RPi kamera → PC UDP RTP/JPEG video akışı (low-latency + OTO LAG flash).

Normal akış: rpicam | gst → udpsink (queue yok).
OTO LAG: ayrı kısa magenta videotestsrc burst aynı UDP portuna.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import threading
from typing import Optional

_BIN_CANDIDATES = (
    "/usr/bin",
    "/usr/local/bin",
    "/bin",
)


def _find_bin(*names: str) -> Optional[str]:
    for name in names:
        found = shutil.which(name)
        if found:
            return found
        for folder in _BIN_CANDIDATES:
            path = os.path.join(folder, name)
            if os.path.isfile(path) and os.path.access(path, os.X_OK):
                return path
    return None


class VideoStreamer:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        port: int = 5000,
        enabled: bool = True,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.port = port
        self.enabled = enabled
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._host: Optional[str] = None
        path = os.environ.get("PATH", "")
        extra = ":".join(_BIN_CANDIDATES)
        if extra not in path:
            os.environ["PATH"] = f"{extra}:{path}" if path else extra
        self._rpicam = _find_bin("rpicam-vid", "libcamera-vid")
        self._gst_bin = _find_bin("gst-launch-1.0")

    @property
    def running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def request_flash(self, frames: int = 10) -> None:
        """PC OTO LAG: magenta kareleri doğrudan UDP'ye bas (insan yok)."""
        with self._lock:
            host = self._host
            port = self.port
            gst = self._gst_bin
            w, h, fps = self.width, self.height, self.fps
        if not host or not gst:
            print("[WARN] lag-flash: video yayını yok / gst yok")
            return
        n = max(4, min(30, int(frames)))
        # solid magenta (ARGB) — odada net görünür
        cmd = (
            f"{gst} -q videotestsrc num-buffers={n} pattern=solid-color "
            f"foreground-color=0xffff00ff ! "
            f"video/x-raw,width={w},height={h},framerate={fps}/1 ! "
            f"jpegenc quality=55 ! rtpjpegpay pt=26 ! "
            f"udpsink host={host} port={port} sync=false async=false"
        )
        print(f"[OK] lag-flash UDP burst → {host}:{port} ({n} kare magenta)")

        def _run() -> None:
            try:
                subprocess.run(cmd, shell=True, timeout=6, check=False)
            except Exception as exc:  # noqa: BLE001
                print(f"[WARN] lag-flash burst hata: {exc}")

        threading.Thread(target=_run, name="lag-flash", daemon=True).start()

    def start(self, host: str, port: Optional[int] = None) -> bool:
        if not self.enabled:
            return False
        host = (host or "").strip()
        if not host or host.startswith("127."):
            print(f"[WARN] Video: geçersiz hedef host={host!r}")
            return False
        if not self._rpicam or not self._gst_bin:
            print(
                "[WARN] Video: rpicam-vid/libcamera-vid veya gst-launch-1.0 yok.\n"
                "  Pi'de dene: which rpicam-vid gst-launch-1.0"
            )
            return False

        out_port = int(port or self.port)
        with self._lock:
            if (
                self._proc is not None
                and self._proc.poll() is None
                and self._host == host
                and out_port == self.port
            ):
                return True
            self._stop_locked()
            self.port = out_port
            cmd = (
                f"{self._rpicam} --width {self.width} --height {self.height} "
                f"--framerate {self.fps} --codec mjpeg --nopreview "
                f"--buffer-count 2 -t 0 -o - | "
                f"{self._gst_bin} -q fdsrc do-timestamp=false ! jpegparse ! "
                f"rtpjpegpay pt=26 ! "
                f"udpsink host={host} port={out_port} sync=false async=false "
                f"max-lateness=0 qos=true buffer-size=131072"
            )
            try:
                self._proc = subprocess.Popen(
                    cmd,
                    shell=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
            except OSError as exc:
                print(f"[WARN] Video başlatılamadı: {exc}")
                self._proc = None
                self._host = None
                return False
            self._host = host
            print(
                f"[OK] Video UDP → {host}:{out_port} "
                f"({self.width}x{self.height}@{self.fps}) low-latency"
            )
            return True

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        if self._proc is None:
            return
        proc = self._proc
        self._proc = None
        host = self._host
        self._host = None
        try:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
        except OSError:
            pass
        if host:
            print(f"[OK] Video durdu ({host})")
