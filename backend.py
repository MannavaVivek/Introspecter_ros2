from fastapi import FastAPI
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import rclpy
from rclpy.node import Node
from pathlib import Path

import importlib
import logging
import threading
from rclpy.qos import QoSProfile

# -------------------------------
# ROS2 Node to list topics
# -------------------------------

import time
import struct
from collections import deque


def extract_ros_time_from_serialized(serialized_msg):
    buf = memoryview(serialized_msg.buffer)
    # Usually bytes [4:12] contain seconds/nanoseconds (little endian)
    sec, nsec = struct.unpack_from("<II", buf, offset=4)
    return sec * 1e9 + nsec  # nanoseconds


class RollingWindow:
    def __init__(self, max_size=100):
        self.interarrivals_us = deque(maxlen=max_size)
        self.sum_interarrival_us = 0.0

    def add(self, diff_us):
        if len(self.interarrivals_us) == self.interarrivals_us.maxlen:
            # subtract the oldest value from the running sum
            self.sum_interarrival_us -= self.interarrivals_us[0]
        self.interarrivals_us.append(diff_us)
        self.sum_interarrival_us += diff_us

    def frame_rate_hz(self):
        if not self.interarrivals_us:
            return 0.0
        mean_us = self.sum_interarrival_us / len(self.interarrivals_us)
        return 1e6 / mean_us if mean_us > 0 else 0.0


class TopicMonitor:
    def __init__(self, topic_name, expected_rate_hz=None):
        self.topic = topic_name
        self.node_window = RollingWindow()
        self.msg_window = RollingWindow()
        self.prev_node_time_us = None
        self.prev_msg_time_us = None
        self.last_update_ns = None
        self.sample_count = 0
        self.expected_rate_hz = expected_rate_hz

    def update(self, msg_timestamp_ns):
        now_us = time.time() * 1e6
        msg_us = msg_timestamp_ns / 1e3

        if self.prev_node_time_us is not None:
            diff_node_us = now_us - self.prev_node_time_us
            self.node_window.add(diff_node_us)

        if self.prev_msg_time_us is not None:
            diff_msg_us = msg_us - self.prev_msg_time_us
            self.msg_window.add(diff_msg_us)

        self.prev_node_time_us = now_us
        self.prev_msg_time_us = msg_us
        # record last seen (nanoseconds)
        try:
            self.last_update_ns = int(msg_timestamp_ns)
        except Exception:
            self.last_update_ns = time.time_ns()
        self.sample_count += 1

    def get_rate(self):
        return {
            "node_rate_hz": self.node_window.frame_rate_hz(),
            "msg_rate_hz": self.msg_window.frame_rate_hz(),
            "last_update_ns": self.last_update_ns,
            "sample_count": self.sample_count,
            "expected_rate_hz": self.expected_rate_hz,
        }


class TopicListNode(Node):
    def __init__(self):
        super().__init__("topic_list_node")
        self.monitors = {}
        # store subscriptions so we can destroy them when unmonitoring
        # use a private name that won't clash with rclpy.Node internals
        self._topic_subscriptions = {}
        self.qos = QoSProfile(depth=10)

    def add_topic_monitor(self, topic_name, topic_type=None):
        """
        Start monitoring a topic. If topic_type is None, try to discover it
        from get_topic_names_and_types().
        """
        if topic_name in self._topic_subscriptions:
            # already monitoring (subscription exists)
            return True

        # if type not provided, try to look it up
        if topic_type is None:
            topics = self.get_topic_names_and_types()
            for name, types in topics:
                if name == topic_name and types:
                    topic_type = types[0]
                    break

        if topic_type is None:
            logging.getLogger('topic_list_node').warning(
                f"Cannot determine type for topic {topic_name}; skipping monitor"
            )
            return False

        # create the TopicMonitor if it doesn't exist, or reuse existing one (might have expected_rate_hz)
        if topic_name not in self.monitors:
            self.monitors[topic_name] = TopicMonitor(topic_name)

        # resolve message class from the string type
        msg_type = None
        try:
            parts = topic_type.split('/')
            if len(parts) >= 3 and parts[1] == 'msg':
                pkg = parts[0]
                type_name = parts[2]
                module = importlib.import_module(f"{pkg}.msg")
                msg_type = getattr(module, type_name)
            else:
                pkg = parts[0]
                module = importlib.import_module(f"{pkg}.msg")
                type_name = parts[-1]
                msg_type = getattr(module, type_name)
        except Exception as e:
            logging.getLogger('topic_list_node').warning(
                f"Could not resolve message type for {topic_name}: {topic_type} ({e})"
            )

        if msg_type is None:
            # cleanup monitor
            del self.monitors[topic_name]
            return False

        sub = self.create_subscription(msg_type, topic_name, self.make_callback(topic_name), self.qos, raw=True)
        self._topic_subscriptions[topic_name] = sub
        self.get_logger().info(f"Monitoring {topic_name}")
        return True

    def make_callback(self, topic_name):
        def callback(serialized_msg):
            try:
                ts = extract_ros_time_from_serialized(serialized_msg)
            except Exception:
                ts = time.time_ns()
            self.monitors[topic_name].update(ts)

        return callback

    def remove_topic_monitor(self, topic_name):
        """Stop monitoring a topic and remove its subscription and monitor."""
        removed = False
        if topic_name in self._topic_subscriptions:
            try:
                self.destroy_subscription(self._topic_subscriptions.pop(topic_name))
                removed = True
            except Exception:
                pass
        if topic_name in self.monitors:
            try:
                del self.monitors[topic_name]
                removed = True
            except Exception:
                pass
        return removed

    def list_topics_with_rates(self):
        topics = self.get_topic_names_and_types()
        # Return current rates and monitored state; do NOT auto-create monitors here
        result = []
        for name, types in topics:
            rate = self.monitors[name].get_rate() if name in self.monitors else {}
            result.append(
                {
                    "topic": name,
                    "type": types[0] if types else "",
                    "monitored": name in self._topic_subscriptions,
                    "node_rate_hz": rate.get("node_rate_hz", 0),
                    "msg_rate_hz": rate.get("msg_rate_hz", 0),
                    "last_update_ns": rate.get("last_update_ns", None),
                    "expected_rate_hz": rate.get("expected_rate_hz", None),
                }
            )
        return result


# -------------------------------
# FastAPI setup
# -------------------------------
app = FastAPI(title="ROS2 Topic Monitor")

# Configure CORS so requests from the common local dev origins succeed.
# Allow both 127.0.0.1 and localhost (same port) to avoid the browser blocking
# requests when the frontend uses either hostname.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:8000", "http://localhost:8000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve static files (frontend)
STATIC_DIR = Path(__file__).parent / "static"
# Mount static files under /static so API routes (e.g. /api/topics) are not
# intercepted by the static file handler. Serve index.html at root explicitly.
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
def serve_index():
    return FileResponse(STATIC_DIR / "index.html")

node = None
_spin_thread = None


@app.on_event("startup")
def on_startup():
    global node
    global _spin_thread
    rclpy.init()
    node = TopicListNode()
    # start a background thread to spin the node so subscriptions receive callbacks
    _spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    _spin_thread.start()
    node.get_logger().info("ROS2 topic list node started")


@app.on_event("shutdown")
def on_shutdown():
    global node
    global _spin_thread
    if node is not None:
        try:
            # request shutdown which will stop rclpy.spin
            rclpy.shutdown()
        except Exception:
            pass
        # wait briefly for the spin thread to exit
        if _spin_thread is not None:
            _spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass


# -------------------------------
# API Endpoint
# -------------------------------
@app.get("/api/topics")
def get_topics():
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    topics = node.list_topics_with_rates()
    return JSONResponse(topics)


@app.post("/api/monitor/{topic:path}")
def start_monitor(topic: str):
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    # find topic type
    topics = node.get_topic_names_and_types()
    topic_type = None
    for name, types in topics:
        if name == topic and types:
            topic_type = types[0]
            break
    if topic_type is None:
        return JSONResponse({"error": f"Topic {topic} not found"}, status_code=404)
    try:
        started = node.add_topic_monitor(topic, topic_type)
        if started:
            return JSONResponse({"topic": topic, "monitored": True})
        else:
            logging.getLogger('topic_list_node').warning(f"add_topic_monitor returned False for {topic}")
            return JSONResponse({"error": f"Could not start monitor for {topic}"}, status_code=500)
    except Exception as e:
        logging.getLogger('topic_list_node').exception(f"Failed to start monitor for {topic}")
        return JSONResponse({"error": f"Failed to start monitor for {topic}: {e}"}, status_code=500)


@app.put("/api/expected_rate/{topic:path}")
def set_expected_rate(topic: str, expected_rate_hz: float):
    """Set the expected rate for a topic."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    # Check if topic is being monitored
    if topic in node.monitors:
        node.monitors[topic].expected_rate_hz = expected_rate_hz
        return JSONResponse({"topic": topic, "expected_rate_hz": expected_rate_hz})
    else:
        # Create a monitor entry even if not subscribed yet, to preserve the expected rate
        node.monitors[topic] = TopicMonitor(topic, expected_rate_hz)
        return JSONResponse({"topic": topic, "expected_rate_hz": expected_rate_hz})


@app.delete("/api/expected_rate/{topic:path}")
def delete_expected_rate(topic: str):
    """Remove the expected rate for a topic."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    if topic in node.monitors:
        node.monitors[topic].expected_rate_hz = None
        return JSONResponse({"topic": topic, "expected_rate_hz": None})
    else:
        return JSONResponse({"error": f"Topic {topic} not found in monitors"}, status_code=404)


@app.delete("/api/monitor/{topic:path}")
def stop_monitor(topic: str):
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    try:
        removed = node.remove_topic_monitor(topic)
        if removed:
            return JSONResponse({"topic": topic, "monitored": False})
        else:
            return JSONResponse({"error": f"Topic {topic} was not being monitored"}, status_code=404)
    except Exception as e:
        logging.getLogger('topic_list_node').exception(f"Failed to stop monitor for {topic}")
        return JSONResponse({"error": f"Failed to stop monitor for {topic}: {e}"}, status_code=500)


@app.get("/api/monitors")
def get_monitors():
    """Return internal monitor/subscription state for debugging."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    data = {
        "subscriptions": list(node._topic_subscriptions.keys()),
        "monitors": {},
    }
    for name, mon in node.monitors.items():
        r = mon.get_rate()
        data["monitors"][name] = {
            "sample_count": r.get("sample_count", 0),
            "last_update_ns": r.get("last_update_ns"),
            "node_rate_hz": r.get("node_rate_hz", 0),
            "msg_rate_hz": r.get("msg_rate_hz", 0),
        }
    return JSONResponse(data)
