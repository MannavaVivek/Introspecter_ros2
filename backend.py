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
import json
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
    def __init__(self, topic_name, expected_rate_hz=None, stale_timeout_sec=2.0):
        self.topic = topic_name
        self.node_window = RollingWindow()
        self.msg_window = RollingWindow()
        self.prev_node_time_us = None
        self.prev_msg_time_us = None
        self.last_update_ns = None
        self.sample_count = 0
        self.expected_rate_hz = expected_rate_hz
        self.stale_timeout_sec = stale_timeout_sec

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

    def is_stale(self):
        """Check if the topic data is stale (no recent updates)."""
        if self.last_update_ns is None:
            return True
        now_ns = time.time_ns()
        elapsed_sec = (now_ns - self.last_update_ns) / 1e9
        return elapsed_sec > self.stale_timeout_sec

    def get_rate(self):
        # If data is stale, return 0 Hz
        if self.is_stale():
            return {
                "node_rate_hz": 0.0,
                "msg_rate_hz": 0.0,
                "last_update_ns": self.last_update_ns,
                "sample_count": self.sample_count,
                "expected_rate_hz": self.expected_rate_hz,
            }
        
        return {
            "node_rate_hz": self.node_window.frame_rate_hz(),
            "msg_rate_hz": self.msg_window.frame_rate_hz(),
            "last_update_ns": self.last_update_ns,
            "sample_count": self.sample_count,
            "expected_rate_hz": self.expected_rate_hz,
        }


class NodeMonitor:
    def __init__(self, node_name, namespace="/"):
        self.node_name = node_name
        self.namespace = namespace
        self.full_name = f"{namespace}/{node_name}".replace("//", "/")
        self.last_seen = time.time()
        self.status = "active"  # active, MIA
        self.mia_timeout_sec = 3.0  # Consider MIA after 3 seconds
    
    def update_status(self, is_alive):
        """Update the node's status based on whether it's still alive."""
        if is_alive:
            self.last_seen = time.time()
            self.status = "active"
        else:
            elapsed = time.time() - self.last_seen
            if elapsed > self.mia_timeout_sec:
                self.status = "MIA"
    
    def get_status(self):
        """Get the current status of the node."""
        elapsed = time.time() - self.last_seen
        if elapsed > self.mia_timeout_sec:
            self.status = "MIA"
        return {
            "node_name": self.node_name,
            "namespace": self.namespace,
            "full_name": self.full_name,
            "status": self.status,
            "last_seen": self.last_seen,
        }


class TopicListNode(Node):
    def __init__(self):
        super().__init__("introspection_node")
        self.monitors = {}
        # store subscriptions so we can destroy them when unmonitoring
        # use a private name that won't clash with rclpy.Node internals
        self._topic_subscriptions = {}
        self.qos = QoSProfile(depth=10)
        
        # Node monitoring
        self.node_monitors = {}  # dict of full_name -> NodeMonitor
        self._node_check_timer = None
        self._start_node_health_check()

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
            logging.getLogger('introspection_node').warning(
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
            logging.getLogger('introspection_node').warning(
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
            # Check if topic has any active publishers
            publishers_info = self.get_publishers_info_by_topic(name)
            if not publishers_info:
                # Skip topics with no active publishers
                continue
                
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
    
    def _start_node_health_check(self):
        """Start periodic health check for monitored nodes."""
        self._node_check_timer = self.create_timer(1.0, self._check_node_health)
    
    def _check_node_health(self):
        """Periodically check if monitored nodes are still alive."""
        if not self.node_monitors:
            return
        
        # Get current list of nodes
        current_nodes = self.get_node_names_and_namespaces()
        current_node_set = set()
        for name, namespace in current_nodes:
            full_name = f"{namespace}/{name}".replace("//", "/")
            current_node_set.add(full_name)
        
        # Update status of monitored nodes
        for full_name, monitor in self.node_monitors.items():
            is_alive = full_name in current_node_set
            monitor.update_status(is_alive)
    
    def add_node_monitor(self, node_name, namespace="/"):
        """Start monitoring a node."""
        full_name = f"{namespace}/{node_name}".replace("//", "/")
        
        if full_name not in self.node_monitors:
            self.node_monitors[full_name] = NodeMonitor(node_name, namespace)
            self.get_logger().info(f"Started monitoring node {full_name}")
            return True
        return False
    
    def remove_node_monitor(self, full_name):
        """Stop monitoring a node."""
        if full_name in self.node_monitors:
            del self.node_monitors[full_name]
            self.get_logger().info(f"Stopped monitoring node {full_name}")
            return True
        return False
    
    def get_monitored_nodes(self):
        """Get status of all monitored nodes."""
        result = []
        for full_name, monitor in self.node_monitors.items():
            result.append(monitor.get_status())
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
CONFIG_FILE = Path(__file__).parent / "monitor_config.json"
# Mount static files under /static so API routes (e.g. /api/topics) are not
# intercepted by the static file handler. Serve index.html at root explicitly.
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
def serve_index():
    return FileResponse(STATIC_DIR / "index.html")

node = None
_spin_thread = None


# -------------------------------
# Configuration Management
# -------------------------------
def save_config():
    """Save the current monitoring configuration to disk."""
    global node
    if node is None:
        return
    
    config = {
        "monitored_topics": [],
        "monitored_nodes": []
    }
    
    # Save all monitored topics with their types and expected rates
    for topic_name in node._topic_subscriptions.keys():
        # Get topic type
        topics = node.get_topic_names_and_types()
        topic_type = None
        for name, types in topics:
            if name == topic_name and types:
                topic_type = types[0]
                break
        
        entry = {
            "topic": topic_name,
            "type": topic_type,
        }
        
        # Add expected rate if set
        if topic_name in node.monitors and node.monitors[topic_name].expected_rate_hz is not None:
            entry["expected_rate_hz"] = node.monitors[topic_name].expected_rate_hz
        
        config["monitored_topics"].append(entry)
    
    # Also save expected rates for topics that aren't currently monitored
    for topic_name, monitor in node.monitors.items():
        if topic_name not in node._topic_subscriptions and monitor.expected_rate_hz is not None:
            # Get topic type
            topics = node.get_topic_names_and_types()
            topic_type = None
            for name, types in topics:
                if name == topic_name and types:
                    topic_type = types[0]
                    break
            
            config["monitored_topics"].append({
                "topic": topic_name,
                "type": topic_type,
                "expected_rate_hz": monitor.expected_rate_hz,
                "monitored": False
            })
    
    # Save all monitored nodes
    for full_name, node_monitor in node.node_monitors.items():
        config["monitored_nodes"].append({
            "full_name": full_name,
            "name": node_monitor.node_name,
            "namespace": node_monitor.namespace,
        })
    
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
        logging.getLogger('introspection_node').info(f"Saved configuration to {CONFIG_FILE}")
    except Exception as e:
        logging.getLogger('introspection_node').error(f"Failed to save config: {e}")


def load_config():
    """Load and apply the monitoring configuration from disk."""
    global node
    if node is None:
        return
    
    if not CONFIG_FILE.exists():
        logging.getLogger('introspection_node').info("No config file found, starting fresh")
        return
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        
        monitored_topics = config.get("monitored_topics", [])
        monitored_nodes = config.get("monitored_nodes", [])
        logging.getLogger('introspection_node').info(f"Loading {len(monitored_topics)} topics from config")
        
        # Load monitored topics
        for entry in monitored_topics:
            topic = entry.get("topic")
            topic_type = entry.get("type")
            expected_rate = entry.get("expected_rate_hz")
            should_monitor = entry.get("monitored", True)
            
            if not topic:
                continue
            
            # Set expected rate if provided
            if expected_rate is not None:
                if topic not in node.monitors:
                    node.monitors[topic] = TopicMonitor(topic, expected_rate)
                else:
                    node.monitors[topic].expected_rate_hz = expected_rate
            
            # Start monitoring if configured to do so
            if should_monitor:
                try:
                    node.add_topic_monitor(topic, topic_type)
                except Exception as e:
                    logging.getLogger('introspection_node').warning(
                        f"Failed to restore monitoring for {topic}: {e}"
                    )
        
        # Load monitored nodes
        for entry in monitored_nodes:
            node_name = entry.get("name")
            namespace = entry.get("namespace", "/")
            
            if not node_name:
                continue
            
            try:
                node.add_node_monitor(node_name, namespace)
            except Exception as e:
                logging.getLogger('introspection_node').warning(
                    f"Failed to restore node monitoring for {node_name}: {e}"
                )
        
        logging.getLogger('introspection_node').info("Configuration loaded successfully")
    except Exception as e:
        logging.getLogger('introspection_node').error(f"Failed to load config: {e}")


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
    
    # Load saved configuration after a brief delay to allow topic discovery
    def delayed_config_load():
        time.sleep(0.5)  # Give time for initial topic discovery
        load_config()
    
    config_thread = threading.Thread(target=delayed_config_load, daemon=True)
    config_thread.start()


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
            save_config()  # Save config after starting monitor
            return JSONResponse({"topic": topic, "monitored": True})
        else:
            logging.getLogger('introspection_node').warning(f"add_topic_monitor returned False for {topic}")
            return JSONResponse({"error": f"Could not start monitor for {topic}"}, status_code=500)
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to start monitor for {topic}")
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
        save_config()  # Save config after setting expected rate
        return JSONResponse({"topic": topic, "expected_rate_hz": expected_rate_hz})
    else:
        # Create a monitor entry even if not subscribed yet, to preserve the expected rate
        node.monitors[topic] = TopicMonitor(topic, expected_rate_hz)
        save_config()  # Save config after setting expected rate
        return JSONResponse({"topic": topic, "expected_rate_hz": expected_rate_hz})


@app.delete("/api/expected_rate/{topic:path}")
def delete_expected_rate(topic: str):
    """Remove the expected rate for a topic."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    if topic in node.monitors:
        node.monitors[topic].expected_rate_hz = None
        save_config()  # Save config after deleting expected rate
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
            save_config()  # Save config after stopping monitor
            return JSONResponse({"topic": topic, "monitored": False})
        else:
            return JSONResponse({"error": f"Topic {topic} was not being monitored"}, status_code=404)
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to stop monitor for {topic}")
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


@app.get("/api/nodes")
def get_nodes():
    """Return list of ROS2 nodes with their namespaces."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        nodes_with_ns = node.get_node_names_and_namespaces()
        
        seen_full_names = set()
        result = []
        
        # Add all active nodes
        for name, namespace in nodes_with_ns:
            if name.startswith('_ros2cli'):
                continue
            
            # Clean up namespace and create full name
            clean_namespace = namespace if namespace and namespace != "/" else ""
            if clean_namespace and not clean_namespace.startswith('/'):
                clean_namespace = '/' + clean_namespace
            
            full_name = f"{clean_namespace}/{name}" if clean_namespace else f"/{name}"
            full_name = full_name.replace("//", "/")
            seen_full_names.add(full_name)
            
            # Check if node is monitored
            monitored = full_name in node.node_monitors
            status = None
            if monitored:
                node_status = node.node_monitors[full_name].get_status()
                status = node_status["status"]
            
            result.append({
                "name": name,
                "namespace": namespace,
                "full_name": full_name,
                "monitored": monitored,
                "status": status,
            })
        
        # Add monitored nodes that are MIA (not in active node list)
        for full_name, monitor in node.node_monitors.items():
            if full_name not in seen_full_names:
                node_status = monitor.get_status()
                logging.getLogger('Neura_topic_monitor').info(
                    f"Adding MIA node: {full_name}, status: {node_status['status']}"
                )
                result.append({
                    "name": monitor.node_name,
                    "namespace": monitor.namespace,
                    "full_name": full_name,
                    "monitored": True,
                    "status": node_status["status"],
                })
        
        # Sort by full name
        result.sort(key=lambda x: x["full_name"])
        
        return JSONResponse(result)
    except Exception as e:
        logging.getLogger('introspection_node').exception("Failed to get nodes")
        return JSONResponse({"error": f"Failed to get nodes: {e}"}, status_code=500)


@app.post("/api/node_monitor/{node_full_name:path}")
def start_node_monitor(node_full_name: str):
    """Start monitoring a node."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        # Parse node name and namespace
        parts = node_full_name.split('/')
        if len(parts) >= 2:
            name = parts[-1]
            namespace = '/'.join(parts[:-1]) if len(parts) > 2 else '/'
            if not namespace:
                namespace = '/'
        else:
            name = node_full_name
            namespace = '/'
        
        started = node.add_node_monitor(name, namespace)
        if started:
            save_config()  # Save config after starting node monitor
            return JSONResponse({"node": node_full_name, "monitored": True})
        else:
            return JSONResponse({"node": node_full_name, "monitored": True, "message": "Already monitoring"})
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to start node monitor for {node_full_name}")
        return JSONResponse({"error": f"Failed to start node monitor: {e}"}, status_code=500)


@app.delete("/api/node_monitor/{node_full_name:path}")
def stop_node_monitor(node_full_name: str):
    """Stop monitoring a node."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        removed = node.remove_node_monitor(node_full_name)
        if removed:
            save_config()  # Save config after stopping node monitor
            return JSONResponse({"node": node_full_name, "monitored": False})
        else:
            return JSONResponse({"error": f"Node {node_full_name} was not being monitored"}, status_code=404)
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to stop node monitor for {node_full_name}")
        return JSONResponse({"error": f"Failed to stop node monitor: {e}"}, status_code=500)


@app.get("/api/monitored_nodes")
def get_monitored_nodes():
    """Return list of monitored nodes with their status."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        result = node.get_monitored_nodes()
        return JSONResponse(result)
    except Exception as e:
        logging.getLogger('introspection_node').exception("Failed to get monitored nodes")
        return JSONResponse({"error": f"Failed to get monitored nodes: {e}"}, status_code=500)


@app.get("/api/topic_info/{topic:path}")
def get_topic_info(topic: str):
    """Get detailed information about a specific topic."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        # Get topic type
        topics = node.get_topic_names_and_types()
        topic_type = None
        for name, types in topics:
            if name == topic and types:
                topic_type = types[0]
                break
        
        # Get publishers and subscribers count
        publishers_info = node.get_publishers_info_by_topic(topic)
        subscribers_info = node.get_subscriptions_info_by_topic(topic)
        
        # Format node names - replace ros2cli with friendly name
        def format_node_name(node_name):
            if node_name.startswith('_ros2cli'):
                return 'ros2_cli_command'
            return node_name
        
        result = {
            "topic": topic,
            "type": topic_type,
            "publishers_count": len(publishers_info),
            "subscribers_count": len(subscribers_info),
            "publishers": [{"node_name": format_node_name(p.node_name), "node_namespace": p.node_namespace} for p in publishers_info],
            "subscribers": [{"node_name": format_node_name(s.node_name), "node_namespace": s.node_namespace} for s in subscribers_info],
        }
        
        # Add monitoring info if available
        if topic in node.monitors:
            rate = node.monitors[topic].get_rate()
            result.update({
                "monitored": True,
                "node_rate_hz": rate.get("node_rate_hz", 0),
                "msg_rate_hz": rate.get("msg_rate_hz", 0),
                "sample_count": rate.get("sample_count", 0),
                "expected_rate_hz": rate.get("expected_rate_hz"),
            })
        else:
            result["monitored"] = False
        
        return JSONResponse(result)
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to get info for topic {topic}")
        return JSONResponse({"error": f"Failed to get topic info: {e}"}, status_code=500)


@app.get("/api/node_info/{node_name:path}")
def get_node_info(node_name: str):
    """Get detailed information about a specific node."""
    global node
    if node is None:
        return JSONResponse({"error": "ROS2 node not initialized"}, status_code=500)
    
    try:
        # Parse node name and namespace
        # node_name could be "/namespace/name" format
        parts = node_name.split('/')
        if len(parts) >= 2:
            name = parts[-1]
            namespace = '/'.join(parts[:-1]) if len(parts) > 2 else '/'
        else:
            name = node_name
            namespace = '/'
        
        # Get topics this node publishes and subscribes to
        all_topics = node.get_topic_names_and_types()
        
        publishers = []
        subscribers = []
        
        for topic_name, types in all_topics:
            # Check publishers
            pubs_info = node.get_publishers_info_by_topic(topic_name)
            for pub in pubs_info:
                pub_ns = pub.node_namespace if pub.node_namespace else '/'
                if pub.node_name == name and pub_ns == namespace:
                    publishers.append({
                        "topic": topic_name,
                        "type": types[0] if types else ""
                    })
                    break
            
            # Check subscribers
            subs_info = node.get_subscriptions_info_by_topic(topic_name)
            for sub in subs_info:
                sub_ns = sub.node_namespace if sub.node_namespace else '/'
                if sub.node_name == name and sub_ns == namespace:
                    subscribers.append({
                        "topic": topic_name,
                        "type": types[0] if types else ""
                    })
                    break
        
        # Get services
        service_names = node.get_service_names_and_types_by_node(name, namespace)
        services = [{"name": svc_name, "type": svc_types[0] if svc_types else ""} 
                   for svc_name, svc_types in service_names]
        
        result = {
            "name": name,
            "namespace": namespace,
            "full_name": node_name,
            "publishers": publishers,
            "subscribers": subscribers,
            "services": services,
        }
        
        return JSONResponse(result)
    except Exception as e:
        logging.getLogger('introspection_node').exception(f"Failed to get info for node {node_name}")
        return JSONResponse({"error": f"Failed to get node info: {e}"}, status_code=500)
