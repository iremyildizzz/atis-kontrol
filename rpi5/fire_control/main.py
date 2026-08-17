"""Atış kontrol ana döngü — Raspberry Pi 5 (gokhisar JSON uyumlu)."""
from __future__ import annotations

import argparse
import signal
import time
from dataclasses import dataclass

from .engagement import distance_allows_fire, engage_range_for
from .lidar_tf02 import TF02Pro
from .optics import GS_16MM, CameraOptics
from .pid import PID, PIDGains
from .protocol import DownlinkCommand
from .tcp_server import MissionState, TcpJsonServer
from .uart_bridge import Stm32Bridge
from .video_stream import VideoStreamer


@dataclass
class Limits:
    # gokhisar servo uzayı: 0…180°; pan 90°, tilt home elevation -15° → 75°
    pan_min: float = 0.0
    pan_max: float = 180.0
    tilt_min: float = 0.0
    tilt_max: float = 180.0
    home_pan_deg: float = 90.0
    home_tilt_deg: float = 75.0
    engage_err_deg: float = 0.35
    engage_stable_s: float = 1.0  # menzilde kararlı kalma


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def iff_allows_fire(stage: int, iff: str, engage_active: bool) -> bool:
    if iff.lower() in ("dost", "friend", "friendly"):
        return False
    # gokhisar engage → PC IFF geçmiş sayılır
    if engage_active and iff.lower() in ("dusman", "düşman", "enemy", "bilinmiyor"):
        return True
    if stage >= 3 and iff.lower() not in ("dusman", "düşman", "enemy"):
        return False
    return True


def run(args: argparse.Namespace) -> None:
    limits = Limits(
        engage_stable_s=args.engage_stable,
        home_pan_deg=args.home_pan,
        home_tilt_deg=args.home_tilt,
    )
    optics: CameraOptics = GS_16MM
    state = MissionState(frame_w=args.frame_w, frame_h=args.frame_h)
    # Kamera: Pi sürekli PC'ye UDP yayınlar. Arayüz sadece dinler / kapatır;
    # Pi tarafı fire_control kapanana kadar akar.
    video = VideoStreamer(
        width=args.video_width,
        height=args.video_height,
        fps=args.video_fps,
        port=args.video_port,
        enabled=not args.no_video,
    )

    def _on_connect(peer: str) -> None:
        # --video-host verilmediyse ilk TCP istemcisinin IP'sine yayın başlat
        if video.enabled and not args.video_host and not video.running:
            video.start(peer, args.video_port)

    tcp = TcpJsonServer(
        host=args.tcp_host,
        port=args.tcp_port,
        state=state,
        on_client_connect=_on_connect,
    )
    bridge = Stm32Bridge(port=args.stm_port, baud=args.baud)

    lidar = None
    if args.lidar_port:
        try:
            lidar = TF02Pro(port=args.lidar_port, baud=args.lidar_baud)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] LiDAR açılamadı: {exc}")

    pid_x = PID(PIDGains(
        kp=args.kp, ki=args.ki, kd=args.kd, kv=args.kv,
        output_limit=args.out_limit,
    ))
    pid_y = PID(PIDGains(
        kp=args.kp, ki=args.ki, kd=args.kd, kv=args.kv,
        output_limit=args.out_limit,
    ))

    # STM telemetrisi gelmeden açı komutu yok — aksi halde home'a ani iniş olur
    pan = limits.home_pan_deg
    tilt = limits.home_tilt_deg
    angles_synced = False
    seeking_home = False
    in_range_since: float | None = None
    stop = False
    control_hz = 50.0
    control_dt = 1.0 / control_hz
    lost_hold_s = 0.5
    home_slew_dps = float(args.home_slew)  # soft home hızı (°/s)

    def _stop(*_a: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    tcp.start()
    video_ok = False
    if video.enabled and args.video_host:
        video_ok = video.start(args.video_host, args.video_port)

    print(f"[OK] TCP JSON : {args.tcp_host}:{args.tcp_port} (gokhisar uyumlu)")
    print(f"[OK] Frame merkezi: {args.frame_w}x{args.frame_h}")
    print(
        f"[OK] Home: pan={limits.home_pan_deg:.1f}° "
        f"tilt={limits.home_tilt_deg:.1f}° (UI elev {limits.home_tilt_deg - 90.0:+.0f}°)"
    )
    print(f"[OK] STM32 UART: {args.stm_port} @ {args.baud}")
    if not video.enabled:
        print("[OK] Video: kapalı (--no-video)")
    elif args.video_host:
        if video_ok:
            print(
                f"[OK] Video sürekli → {args.video_host}:{args.video_port} "
                f"({args.video_width}x{args.video_height}@{args.video_fps})"
            )
        else:
            print(
                f"[HATA] Video başlamadı (hedef {args.video_host}:{args.video_port}). "
                "rpicam-vid + gst-launch-1.0 kurulu mu?"
            )
    else:
        print(
            f"[OK] Video: ilk TCP bağlanınca peer'e UDP:{args.video_port} "
            "(sürekli; arayüz kapansa da akar)"
        )
    print(
        f"[OK] Optik GS+16mm: HFOV≈{optics.hfov_deg:.1f}° VFOV≈{optics.vfov_deg:.1f}°"
    )

    last_t = time.monotonic()
    last_status = 0.0

    try:
        while not stop:
            now = time.monotonic()
            dt = min(0.05, max(1e-3, now - last_t))
            last_t = now

            bridge.poll()
            if lidar:
                lidar.poll()

            snap = state.snapshot()
            stage = int(snap.stage)

            if snap.pid_dirty and snap.mode == "otonom" and stage >= 2:
                pid_x.set_gains(snap.pid_kp, snap.pid_ki, snap.pid_kd)
                pid_y.set_gains(snap.pid_kp, snap.pid_ki, snap.pid_kd)
                state.clear_pid_dirty()
                print(
                    f"[OK] PID: kp={pid_x.gains.kp:.3f} "
                    f"ki={pid_x.gains.ki:.3f} kd={pid_x.gains.kd:.3f}"
                )
            elif snap.pid_dirty:
                state.clear_pid_dirty()

            # STM gerçek açısını al; sonra soft-home (−15) — ani aşağı yok
            if not angles_synced:
                if bridge.last_telem is not None:
                    pan = clamp(bridge.last_telem.pan_deg, limits.pan_min, limits.pan_max)
                    tilt = clamp(bridge.last_telem.tilt_deg, limits.tilt_min, limits.tilt_max)
                    angles_synced = True
                    seeking_home = True
                    print(
                        f"[OK] Servo senkron: pan={pan:.1f} tilt={tilt:.1f} "
                        f"→ soft home {limits.home_pan_deg:.0f}/{limits.home_tilt_deg:.0f} "
                        f"(elev {limits.home_tilt_deg - 90.0:+.0f}°)"
                    )
                else:
                    time.sleep(control_dt)
                    continue

            home = False
            with state.lock:
                if state.home:
                    home = True
                    state.home = False

            if snap.estop:
                bridge.send(
                    DownlinkCommand(
                        pan_deg=pan,
                        tilt_deg=tilt,
                        enable=False,
                        arm=False,
                        fire=False,
                        safe=True,
                        heartbeat=True,
                        stage=stage,
                    )
                )
                pid_x.reset()
                pid_y.reset()
                in_range_since = None
                time.sleep(control_dt)
                continue

            if home:
                seeking_home = True
                pid_x.reset()
                pid_y.reset()

            # Soft home: elevation −15'e yavaşça git (bir anda kafası inmesin)
            if seeking_home:
                step = home_slew_dps * dt
                dpan = clamp(limits.home_pan_deg - pan, -step, step)
                dtilt = clamp(limits.home_tilt_deg - tilt, -step, step)
                pan += dpan
                tilt += dtilt
                if (
                    abs(limits.home_pan_deg - pan) < 0.4
                    and abs(limits.home_tilt_deg - tilt) < 0.4
                ):
                    pan = limits.home_pan_deg
                    tilt = limits.home_tilt_deg
                    seeking_home = False
                    print(
                        f"[OK] Soft home tamam: pan={pan:.1f} tilt={tilt:.1f} "
                        f"(UI elev {tilt - 90.0:+.0f}°)"
                    )
                state.consume_manual_delta()
                err_pan_deg = 0.0
                err_tilt_deg = 0.0
                # soft-home bitene kadar PID/manuel yok; açı komutu aşağıda gider
                pan = clamp(pan, limits.pan_min, limits.pan_max)
                tilt = clamp(tilt, limits.tilt_min, limits.tilt_max)
                bridge.send(
                    DownlinkCommand(
                        pan_deg=pan,
                        tilt_deg=tilt,
                        fire=False,
                        arm=False,
                        heartbeat=True,
                        home=False,
                        safe=False,
                        enable=True,
                        stage=stage,
                    )
                )
                time.sleep(control_dt)
                continue

            # KTR 4.3:
            #   MANUEL → klavye dx/dy (PID yok)
            #   OTONOM 2/3 → hedef merkezi + PID (klavye yok)
            err_pan_deg = 0.0
            err_tilt_deg = 0.0

            if snap.mode == "manuel":
                dpan, dtilt = state.consume_manual_delta()
                if dpan or dtilt:
                    pan += dpan
                    tilt += dtilt
                    stage = max(stage, 1)
                with state.lock:
                    if state.pan_cmd_deg is not None:
                        pan = state.pan_cmd_deg
                        state.pan_cmd_deg = None
                    if state.tilt_cmd_deg is not None:
                        tilt = state.tilt_cmd_deg
                        state.tilt_cmd_deg = None
                pid_x.reset()
                pid_y.reset()

            elif snap.mode == "otonom" and stage >= 2:
                state.consume_manual_delta()
                target_age = (
                    (now - snap.target_mono) if snap.target_mono > 0.0 else 1e9
                )
                target_fresh = target_age < 0.4
                err_pan_deg, err_tilt_deg = optics.pixel_offset_to_deg(
                    snap.err_x,
                    snap.err_y,
                    frame_w=snap.frame_w,
                    frame_h=snap.frame_h,
                )
                has_target = target_fresh and (
                    snap.track_id >= 0 or snap.class_id >= 0
                )
                if has_target or snap.engage_active:
                    sx = -1.0 if args.invert_x else 1.0
                    sy = -1.0 if args.invert_y else 1.0
                    e_pan = err_pan_deg
                    e_tilt = err_tilt_deg * float(args.tilt_gain)
                    rate_pan = sx * pid_x.step(e_pan, dt)
                    rate_tilt = sy * pid_y.step(e_tilt, dt)
                    tilt_cap = min(pid_y.gains.output_limit, args.tilt_rate_limit)
                    rate_tilt = max(-tilt_cap, min(tilt_cap, rate_tilt))
                    dpan = rate_pan * dt
                    dtilt = rate_tilt * dt
                    pan += dpan
                    tilt += dtilt
                    if now - last_status >= 0.25:
                        print(
                            f"[OTONOM] id={snap.track_id} "
                            f"err=({err_pan_deg:+.2f},{err_tilt_deg:+.2f})° "
                            f"v=({rate_pan:+.1f},{rate_tilt:+.1f})°/s "
                            f"→ pan={pan:.1f} tilt={tilt:.1f}"
                        )
                elif target_age > lost_hold_s:
                    pid_x.reset()
                    pid_y.reset()

            else:
                state.consume_manual_delta()
                pid_x.reset()
                pid_y.reset()

            pan = clamp(pan, limits.pan_min, limits.pan_max)
            tilt = clamp(tilt, limits.tilt_min, limits.tilt_max)

            dist = None
            if lidar and lidar.last is not None:
                dist = lidar.last.distance_m

            if stage >= 3:
                lidar_ok, range_reason = distance_allows_fire(
                    stage,
                    dist,
                    class_name=snap.class_name,
                    class_id=snap.class_id,
                    require_lidar_stage3=True,
                )
            else:
                lidar_ok, range_reason = True, "lidar_not_required"

            # gokhisar: menzilde N sn kararlı kal
            range_stable = True
            if stage >= 3 and snap.engage_active:
                if lidar_ok:
                    if in_range_since is None:
                        in_range_since = now
                    range_stable = (now - in_range_since) >= limits.engage_stable_s
                    if not range_stable:
                        range_reason = f"range_warming:{(now - in_range_since):.2f}/{limits.engage_stable_s}"
                else:
                    in_range_since = None
                    range_stable = False
            else:
                in_range_since = None

            allow_iff = iff_allows_fire(stage, snap.iff, snap.engage_active)
            centered = abs(err_pan_deg) <= limits.engage_err_deg and abs(err_tilt_deg) <= limits.engage_err_deg

            fire_intent = bool(snap.fire or snap.engage_active)

            if stage <= 1 and snap.mode == "manuel":
                want_fire = bool(fire_intent and snap.arm and snap.enable and allow_iff)
            elif stage == 2:
                want_fire = bool(
                    fire_intent
                    and snap.arm
                    and snap.enable
                    and (snap.locked or snap.engage_active)
                    and centered
                    and allow_iff
                )
            else:
                want_fire = bool(
                    fire_intent
                    and snap.arm
                    and snap.enable
                    and (snap.locked or snap.engage_active)
                    and centered
                    and allow_iff
                    and lidar_ok
                    and range_stable
                )

            sent = bridge.send(
                DownlinkCommand(
                    pan_deg=pan,
                    tilt_deg=tilt,
                    fire=want_fire,
                    arm=snap.arm and allow_iff,
                    heartbeat=True,
                    home=home,
                    safe=False,
                    enable=True,  # manuel/otonom: STM açı alsın
                    stage=stage,
                )
            )

            # Engage'i ancak FIRE frame gerçekten UART'a yazıldıysa kapat
            if want_fire and sent:
                state.clear_engage()
                in_range_since = None

            if now - last_status >= 0.2:
                last_status = now
                tel = bridge.last_telem
                rng = engage_range_for(snap.class_name, snap.class_id) if stage >= 3 else None
                status = {
                    "type": "status",
                    "mode": snap.mode,
                    "stage": stage,
                    "class_id": snap.class_id,
                    "class_name": snap.class_name,
                    "pan_deg": round(pan, 2),
                    "tilt_deg": round(tilt, 2),
                    "err_px": {"x": round(snap.err_x, 1), "y": round(snap.err_y, 1)},
                    "err_deg": {"pan": round(err_pan_deg, 3), "tilt": round(err_tilt_deg, 3)},
                    "frame": [snap.frame_w, snap.frame_h],
                    "locked": snap.locked,
                    "engage_active": snap.engage_active,
                    "iff": snap.iff,
                    "lidar_m": None if dist is None else round(dist, 3),
                    "engage_range_m": None if rng is None else {"min": rng[0], "max": rng[1]},
                    "range_ok": lidar_ok,
                    "range_stable": range_stable,
                    "range_reason": range_reason,
                    "want_fire": want_fire,
                    "stm": None
                    if tel is None
                    else {
                        "failsafe": tel.failsafe,
                        "armed": tel.armed,
                        "fired": tel.fired,
                        "busy": tel.busy,
                        "enabled": tel.enabled,
                    },
                }
                tcp.broadcast_status(status)

            time.sleep(control_dt)
    finally:
        try:
            bridge.send(
                DownlinkCommand(enable=False, arm=False, fire=False, safe=True, heartbeat=True),
                min_period_s=0.0,
            )
        except Exception:  # noqa: BLE001
            pass
        video.stop()
        tcp.stop()
        bridge.close()
        if lidar:
            lidar.close()
        print("[OK] kapatıldı")


def main() -> None:
    p = argparse.ArgumentParser(description="Hava savunma atış kontrol — RPi5")
    p.add_argument("--tcp-host", default="0.0.0.0")
    p.add_argument("--tcp-port", type=int, default=5005, help="gokhisar RPI_PORT")
    p.add_argument("--frame-w", type=int, default=1280, help="PC FRAME_WIDTH (cx merkezi)")
    p.add_argument("--frame-h", type=int, default=720, help="PC FRAME_HEIGHT (cy merkezi)")
    p.add_argument("--stm-port", default="/dev/ttyAMA0", help="STM32 UART")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--lidar-port", default="/dev/ttyAMA1", help="TF02-PRO; boş = kapalı")
    p.add_argument("--lidar-baud", type=int, default=115200)
    p.add_argument("--engage-stable", type=float, default=1.0, help="Aşama-3 menzil kararlılık sn")
    p.add_argument("--kp", type=float, default=0.18, help="P — oransal")
    p.add_argument("--ki", type=float, default=0.0, help="I — hareketli hedefte 0")
    p.add_argument(
        "--kd",
        type=float,
        default=0.004,
        help="D — çok küçük + kodda LPF (0.15 gibi değerler fırlatır)",
    )
    p.add_argument(
        "--kv",
        type=float,
        default=0.45,
        help="Velocity feedforward — balon hızına akıcı uyum",
    )
    p.add_argument(
        "--out-limit",
        type=float,
        default=14.0,
        help="Maksimum pan hızı (°/s)",
    )
    p.add_argument(
        "--tilt-rate-limit",
        type=float,
        default=10.0,
        help="Elevation max hız (°/s)",
    )
    p.add_argument(
        "--tilt-gain",
        type=float,
        default=0.75,
        help="Elevation hata çarpanı",
    )
    p.add_argument("--home-pan", type=float, default=90.0, help="Başlangıç / home pan (°)")
    p.add_argument(
        "--home-tilt",
        type=float,
        default=75.0,
        help="Başlangıç / home tilt (°) — UI Elevation -15 ⇒ 75",
    )
    p.add_argument(
        "--home-slew",
        type=float,
        default=12.0,
        help="Soft-home hızı (°/s) — açılışta ani iniş olmasın",
    )
    p.add_argument("--invert-x", action="store_true")
    p.add_argument("--invert-y", action="store_true")
    p.add_argument(
        "--video-host",
        default="",
        help="PC IP — kamera UDP hedefi (boşsa ilk TCP istemcisine yayın)",
    )
    p.add_argument("--video-port", type=int, default=5000, help="PC UDP video portu")
    p.add_argument("--video-width", type=int, default=1280)
    p.add_argument("--video-height", type=int, default=720)
    p.add_argument("--video-fps", type=int, default=20)
    p.add_argument("--no-video", action="store_true", help="Kamera UDP akışını kapat")
    args = p.parse_args()
    if args.lidar_port.strip() == "":
        args.lidar_port = None
    args.video_host = (args.video_host or "").strip()
    run(args)


if __name__ == "__main__":
    main()
