// ======= CONFIG =======
// Set API_URL to the endpoint that returns topic listing.
// Acceptable response shapes:
// 1) ["topic1", "topic2", ...]
// 2) {"topics": ["topic1", ...]}
// 3) {"topic1": {...}, "topic2": {...}} -> uses Object.keys()
// Use a relative URL so the browser won't trigger CORS when the frontend is
// served from the same host/port (127.0.0.1 vs localhost differences).
const API_URL = "/api/topics"; // change if your backend uses another path or host

// ======= State =======
let topics = []; // array of strings
let selectedIndex = -1;
let pollTimer = null;

const listEl = document.getElementById("list");
const statusEl = document.getElementById("status");
const emptyEl = document.getElementById("empty");
const pollIntervalSelect = document.getElementById("pollInterval");
const refreshBtn = document.getElementById("refreshBtn");

// ======= Helpers =======
function setStatus(s) {
    statusEl.textContent = s;
}
function clamp(i, min, max) {
    return Math.max(min, Math.min(max, i));
}

// Parse many possible shapes of the API response into an array of topic names
async function fetchTopicsOnce() {
    setStatus("fetching…");
    try {
        const r = await fetch(API_URL, { cache: "no-store" });
        if (!r.ok) throw new Error("HTTP " + r.status);
        const data = await r.json();

        let newTopics = [];
        if (Array.isArray(data)) {
            // backend may return an array of strings OR an array of objects
            // like [{ topic, type, node_rate_hz, msg_rate_hz }, ...]
            if (data.length > 0 && typeof data[0] === "object" && data[0] !== null) {
                // keep objects intact so we preserve metadata (monitored, rates)
                newTopics = data;
            } else {
                newTopics = data;
            }
        } else if (data && Array.isArray(data.topics)) {
            newTopics = data.topics;
        } else if (data && typeof data === "object") {
            // if the backend returns an object mapping topic->value (like latest_data), take keys
            newTopics = Object.keys(data);
        } else {
            newTopics = [];
        }

        // normalize into objects: { topic, type?, monitored?, node_rate_hz?, msg_rate_hz? }
        const toObj = (t) => {
            if (typeof t === 'string') return { topic: String(t), type: '', monitored: false, node_rate_hz: 0, msg_rate_hz: 0, last_update_ns: null };
            if (t && typeof t === 'object') return {
                topic: String(t.topic || t.name || ''),
                type: t.type || '',
                monitored: Boolean(t.monitored),
                node_rate_hz: Number(t.node_rate_hz || 0),
                msg_rate_hz: Number(t.msg_rate_hz || 0),
                last_update_ns: t.last_update_ns || null,
            };
            return { topic: String(t), type: '', monitored: false, node_rate_hz: 0, msg_rate_hz: 0, last_update_ns: null };
        };

        let normalized = Array.from(new Set(newTopics.map(String))).map((s) => ({ topic: s }));
        // if newTopics was already objects, use them directly (preserving metadata)
        if (newTopics.length > 0 && typeof newTopics[0] === 'object') {
            normalized = newTopics.map(toObj);
        } else {
            normalized = newTopics.map(toObj);
        }

        // ensure unique and sort by topic name
        normalized = Array.from(
            new Map(normalized.map((o) => [o.topic, o])).values()
        ).sort((a, b) => a.topic.localeCompare(b.topic));

        // update state
    const prevTopics = topics.map(t => t.topic).join("|");
    topics = normalized;

        // keep selection anchored to the same topic name if possible
        if (selectedIndex >= 0) {
            const prevTopic = prevTopics.split("|")[selectedIndex] || null;
            let newIndex = -1;
            if (prevTopic) {
                newIndex = topics.findIndex((o) => o.topic === prevTopic);
            }
            selectedIndex =
                newIndex >= 0
                    ? newIndex
                    : topics.length
                      ? clamp(selectedIndex, 0, topics.length - 1)
                      : -1;
        }

        renderList();
        setStatus(`last: ${new Date().toLocaleTimeString()}`);
    } catch (err) {
        setStatus("error: " + (err.message || err));
        console.error("Failed to fetch topics:", err);
    }
}

// ======= UI Rendering =======
function renderList() {
    listEl.innerHTML = "";
    if (!topics.length) {
        emptyEl.textContent = "No topics found";
        listEl.appendChild(emptyEl);
        return;
    }

    topics.forEach((t, i) => {
        const item = document.createElement("div");
        item.className =
            "topic" + (i === selectedIndex ? " selected" : "") + (t.monitored ? " monitored" : "");
        item.setAttribute("role", "option");
        item.setAttribute(
            "aria-selected",
            String(i === selectedIndex),
        );
        // content
        const name = document.createElement("div");
        name.className = "name";
        name.textContent = t.topic;
        const badge = document.createElement("span");
        badge.className = "badge";
        badge.textContent = t.monitored ? "monitored" : "";

        const rate = document.createElement("div");
        rate.className = "meta rate";
        rate.textContent = t.monitored ? `${Number(t.msg_rate_hz).toFixed(1)} Hz` : "";

        const last = document.createElement("div");
        last.className = "meta last";
        if (t.last_update_ns) {
            const d = new Date(Number(t.last_update_ns) / 1e6);
            last.textContent = d.toLocaleTimeString();
        } else {
            last.textContent = "";
        }
        const meta = document.createElement("div");
        meta.className = "meta index";
        meta.textContent = `#${i + 1}`;
        item.appendChild(name);
        item.appendChild(badge);
        item.appendChild(rate);
        item.appendChild(last);
        item.appendChild(meta);

        // click -> select and log
        item.addEventListener("click", () => {
            setSelectedIndex(i, { userAction: true });
        });

        listEl.appendChild(item);
    });

    // ensure selected is visible
    ensureSelectedInView();
}

function ensureSelectedInView() {
    if (selectedIndex < 0) return;
    const children = listEl.children;
    if (!children || !children[selectedIndex]) return;
    const node = children[selectedIndex];
    node.scrollIntoView({ block: "nearest", inline: "nearest" });
}

function setSelectedIndex(i, opts = {}) {
    if (!topics.length) return;
    selectedIndex = clamp(i, 0, topics.length - 1);
    renderList();
    ensureSelectedInView();
    const topic = topics[selectedIndex];
    // for now: log; you can replace with pushing selection to backend or opening detail pane
    console.log("Selected topic:", topic);
    if (opts.userAction) {
        // small feedback
        setStatus(`selected ${topic}`);
    }
}

// ======= Keyboard handling =======
function onKeyDown(e) {
    if (
        ![
            "ArrowUp",
            "ArrowDown",
            "Enter",
            "PageUp",
            "PageDown",
            "Home",
            "End",
        ].includes(e.key)
    )
        return;
    e.preventDefault();
    if (!topics.length) return;
    // print the key pressed to console
    console.log(`Key pressed: ${e.key}`);
    if (e.key === 'Enter') {
        e.preventDefault();
        const t = topics[selectedIndex];
        if (!t) return;
        const topicEnc = encodeURIComponent(t.topic);
        // optimistic UI update
        t.monitored = !t.monitored;
        renderList();
        // toggle monitored
        if (!t.monitored) {
            // after optimistic toggle we set to unmonitored, so we must DELETE
            fetch(`/api/monitor/${topicEnc}`, { method: 'DELETE' })
                .then(() => fetchTopicsOnce())
                .catch((err) => {
                    console.error('Failed to stop monitor', err);
                    fetchTopicsOnce();
                });
        } else {
            fetch(`/api/monitor/${topicEnc}`, { method: 'POST' })
                .then(() => fetchTopicsOnce())
                .catch((err) => {
                    console.error('Failed to start monitor', err);
                    fetchTopicsOnce();
                });
        }
        return;
    }
    if (e.key === "ArrowUp") {
        setSelectedIndex(
            selectedIndex <= 0 ? 0 : selectedIndex - 1,
            { userAction: true },
        );
    } else if (e.key === "ArrowDown") {
        setSelectedIndex(
            selectedIndex >= topics.length - 1
                ? topics.length - 1
                : selectedIndex + 1,
            { userAction: true },
        );
    } else if (e.key === "PageUp") {
        const visible = Math.max(
            1,
            Math.floor(listEl.clientHeight / 48),
        );
        setSelectedIndex(selectedIndex - visible, {
            userAction: true,
        });
    } else if (e.key === "PageDown") {
        const visible = Math.max(
            1,
            Math.floor(listEl.clientHeight / 48),
        );
        setSelectedIndex(selectedIndex + visible, {
            userAction: true,
        });
    } else if (e.key === "Home") {
        setSelectedIndex(0, { userAction: true });
    } else if (e.key === "End") {
        setSelectedIndex(topics.length - 1, { userAction: true });
    }
}

// ======= Polling control =======
function startPolling() {
    stopPolling();
    const ms = Number(pollIntervalSelect.value) || 1000;
    pollTimer = setInterval(fetchTopicsOnce, ms);
    setStatus("polling every " + ms + "ms");
    // run immediate fetch
    fetchTopicsOnce();
}
function stopPolling() {
    if (pollTimer) {
        clearInterval(pollTimer);
        pollTimer = null;
    }
}

// ======= Wire up events =======
listEl.addEventListener("keydown", onKeyDown);
pollIntervalSelect.addEventListener("change", startPolling);
refreshBtn.addEventListener("click", () => fetchTopicsOnce());
// focus the list on page load so arrow keys work immediately
window.addEventListener("load", () => {
    listEl.focus();
    startPolling();
});

// also stop polling if tab hidden to reduce load (nice-to-have)
document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
        stopPolling();
        setStatus("paused (tab hidden)");
    } else startPolling();
});

// make sure clicking outside doesn't steal focus from keyboard interactions
document.body.addEventListener("click", (e) => {
    if (!listEl.contains(e.target)) return;
    // keep focus on list so arrow keys continue to work
    listEl.focus();
});
