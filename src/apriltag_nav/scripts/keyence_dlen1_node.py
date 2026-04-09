#!/usr/bin/env python3
import socket
import threading
import time
from typing import Optional, Tuple

import rospy
from std_msgs.msg import Int32, Float32


def parse_response(resp: bytes) -> Tuple[bool, Optional[int], str]:
    text = resp.decode(errors="ignore").strip()
    if not text:
        return False, None, text
    if text.startswith("ER,"):
        return False, None, text

    parts = text.split(",")
    if len(parts) < 2:
        return False, None, text

    try:
        return True, int(parts[1].strip()), text
    except ValueError:
        import re
        m = re.search(r"[-+]?\d+", parts[1])
        if not m:
            return False, None, text
        return True, int(m.group(0)), text


class KeyenceDLEN1Client:
    """
    connect -> send -> shutdown(SHUT_WR) -> recv -> close
    (ROS2 성공 코드와 동일 동작)
    """
    def __init__(self, host: str, port: int, timeout_s: float = 0.5):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.lock = threading.Lock()

    def request_once(self, payload: bytes) -> bytes:
        with self.lock:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout_s)
            s.connect((self.host, self.port))
            try:
                s.sendall(payload)
                # nc 파이프라인처럼 "보낼 거 끝"을 명확히
                try:
                    s.shutdown(socket.SHUT_WR)
                except OSError:
                    pass

                chunks = []
                t0 = time.time()
                while True:
                    try:
                        b = s.recv(256)
                    except socket.timeout:
                        break
                    if not b:
                        break
                    chunks.append(b)
                    if b"\r" in b or b"\n" in b:
                        break
                    if (time.time() - t0) > self.timeout_s:
                        break

                return b"".join(chunks)
            finally:
                try:
                    s.close()
                except Exception:
                    pass


def main():
    rospy.init_node("keyence_dlen1_node", anonymous=False)

    # ROS2 성공 코드 기본값에 맞춤
    host = rospy.get_param("~host", "192.168.1.5")
    port = int(rospy.get_param("~port", 64000))
    rate_hz = float(rospy.get_param("~rate_hz", 60.0))
    timeout_s = float(rospy.get_param("~timeout_s", 0.5))
    scale = float(rospy.get_param("~scale", 0.001))
    offset = float(rospy.get_param("~offset", 0.0))
    warn_throttle_s = float(rospy.get_param("~warn_throttle_s", 1.0))

    # ROS2 성공 코드: use_crlf=True
    use_crlf = bool(rospy.get_param("~use_crlf", True))

    pub_raw = rospy.Publisher("keyence/raw", Int32, queue_size=1)
    pub_value = rospy.Publisher("keyence/value", Float32, queue_size=1)

    client = KeyenceDLEN1Client(host, port, timeout_s=timeout_s)

    ending = "CRLF(\\r\\n)" if use_crlf else "CR(\\r)"
    rospy.loginfo(
        f"Start Keyence DL-EN1 ROS1 node: {host}:{port}, cmd=M0, "
        f"rate={rate_hz}Hz, ending={ending}, eof_after_send=True, scale={scale}"
    )

    last_warn_t = 0.0

    def throttled_warn(msg: str):
        nonlocal last_warn_t
        now = time.time()
        if (now - last_warn_t) >= warn_throttle_s:
            rospy.logwarn(msg)
            last_warn_t = now

    if rate_hz <= 0:
        rospy.logwarn(f"[Keyence] Invalid rate_hz={rate_hz}, falling back to 1.0 Hz")
        rate_hz = 1.0
    r = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        payload = b"M0\r\n" if use_crlf else b"M0\r"

        try:
            resp = client.request_once(payload)
            ok, raw_val, raw_text = parse_response(resp)
            if not ok or raw_val is None:
                throttled_warn(f"Bad response: '{raw_text}'")
                r.sleep()
                continue

            pub_raw.publish(Int32(data=int(raw_val)))
            pub_value.publish(Float32(data=float(raw_val) * scale + offset))

        except Exception as e:
            throttled_warn(f"Comm error: {repr(e)}")

        r.sleep()


if __name__ == "__main__":
    main()