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
let filteredTopics = []; // filtered based on search
let selectedIndex = -1;
let pollTimer = null;
let searchQuery = "";
let expectedRates = {}; // store expected rates per topic: { "topic_name": expectedHz }
let editingRateIndex = -1; // index of topic currently being edited
let isInputRendered = false; // track if input is currently rendered
let showMonitoredOnly = false; // filter to show only monitored topics

const listEl = document.getElementById("list");
const statusEl = document.getElementById("status");
const emptyEl = document.getElementById("empty");
const pollIntervalSelect = document.getElementById("pollInterval");
const refreshBtn = document.getElementById("refreshBtn");
const searchInput = document.getElementById("searchInput");
const clearSearchBtn = document.getElementById("clearSearch");
const searchToggleBtn = document.getElementById("searchToggle");
const monitoredToggleBtn = document.getElementById("monitoredToggle");
let isSearchExpanded = false;

// ======= Helpers =======
function setStatus(s) {
    statusEl.textContent = s;
}
function clamp(i, min, max) {
    return Math.max(min, Math.min(max, i));
}

// Update only the rate values without full re-render (when editing)
function updateRatesOnly() {
    const topicElements = listEl.querySelectorAll('.topic');
    topicElements.forEach((topicEl, idx) => {
        // Note: idx corresponds to the filteredTopics array index
        if (idx === editingRateIndex) return; // Skip the one being edited
        const t = filteredTopics[idx];
        if (!t) return;
        
        // Update rate display
        const rateEl = topicEl.querySelector('.meta.rate');
        if (rateEl) {
            rateEl.textContent = t.monitored ? `${Number(t.msg_rate_hz).toFixed(1)} Hz` : "";
        }
        
        // Update badge
        const badgeEl = topicEl.querySelector('.badge');
        if (badgeEl) {
            badgeEl.textContent = t.monitored ? "monitored" : "";
        }
        
        // Update expected rate color if exists
        const expectedRateEl = topicEl.querySelector('.expected-rate');
        if (expectedRateEl && !expectedRateEl.querySelector('.expected-rate-input')) {
            if (expectedRates[t.topic] !== undefined) {
                const expectedHz = expectedRates[t.topic];
                const actualHz = t.msg_rate_hz || 0;
                
                // Color coding only when monitored and has actual rate
                let colorClass = "";
                if (t.monitored && actualHz > 0) {
                    const tolerance = expectedHz * 0.1;
                    if (Math.abs(actualHz - expectedHz) <= tolerance) {
                        colorClass = "within-tolerance";
                    } else {
                        colorClass = "out-of-tolerance";
                    }
                }
                
                expectedRateEl.className = `expected-rate ${colorClass}`;
                expectedRateEl.textContent = `${expectedHz.toFixed(1)} Hz`;
            } else {
                // Hide if no expected rate set
                expectedRateEl.textContent = "";
            }
        }
        
        // Update monitored class
        if (t.monitored) {
            topicEl.classList.add('monitored');
        } else {
            topicEl.classList.remove('monitored');
        }
    });
}

// Filter topics based on search query and monitored status
function filterTopics() {
    let result = topics;
    
    // Apply search filter if search is active
    if (searchQuery) {
        result = result.filter(t => 
            t.topic.toLowerCase().includes(searchQuery.toLowerCase())
        );
    }
    // Apply monitored filter if toggle is on and search is NOT active
    else if (showMonitoredOnly) {
        result = result.filter(t => t.monitored);
    }
    
    filteredTopics = result;
    
    // Reset selection when filter changes
    if (selectedIndex >= filteredTopics.length) {
        selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    }
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
        topics = normalized;
        
        // Load expected rates from backend data
        topics.forEach(t => {
            if (t.expected_rate_hz !== null && t.expected_rate_hz !== undefined) {
                expectedRates[t.topic] = t.expected_rate_hz;
            }
        });
        
        // Apply search filter
        filterTopics();
        
        // keep selection anchored if possible
        if (selectedIndex >= 0 && filteredTopics.length > 0) {
            selectedIndex = clamp(selectedIndex, 0, filteredTopics.length - 1);
        } else if (filteredTopics.length > 0) {
            selectedIndex = 0;
        } else {
            selectedIndex = -1;
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
    // Don't re-render if input is already shown to avoid closing it
    if (isInputRendered && editingRateIndex >= 0 && editingRateIndex < filteredTopics.length) {
        // Only update rates without full re-render
        updateRatesOnly();
        return;
    }
    
    // Reset the flag since we're doing a full render
    isInputRendered = false;
    
    listEl.innerHTML = "";
    if (!filteredTopics.length) {
        emptyEl.textContent = searchQuery ? `No topics matching "${searchQuery}"` : "No topics found";
        listEl.appendChild(emptyEl);
        return;
    }

    // Add column headers
    const header = document.createElement("div");
    header.className = "topic-header";
    header.innerHTML = `
        <div class="header-cell">Topic Name</div>
        <div class="header-cell">Status</div>
        <div class="header-cell">Rate</div>
        <div class="header-cell">Expected Rate</div>
    `;
    listEl.appendChild(header);

    filteredTopics.forEach((t, i) => {
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

        // Expected rate column
        const expectedRateCell = document.createElement("div");
        expectedRateCell.className = "expected-rate";
        
        if (i === editingRateIndex) {
            // Store the previous value for cancel
            const previousValue = expectedRates[t.topic];
            
            // Show input field
            const input = document.createElement("input");
            input.type = "number";
            input.className = "expected-rate-input";
            input.placeholder = "Enter Hz";
            input.value = expectedRates[t.topic] || "";
            input.step = "0.1";
            input.min = "0";
            
            // Handle input submission
            const handleSubmit = () => {
                const val = parseFloat(input.value);
                const topicEnc = encodeURIComponent(t.topic);
                
                if (!isNaN(val) && val > 0) {
                    expectedRates[t.topic] = val;
                    // Save to backend
                    fetch(`/api/expected_rate/${topicEnc}?expected_rate_hz=${val}`, { method: 'PUT' })
                        .catch((err) => console.error('Failed to save expected rate', err));
                } else if (input.value === "") {
                    delete expectedRates[t.topic];
                    // Delete from backend
                    fetch(`/api/expected_rate/${topicEnc}`, { method: 'DELETE' })
                        .catch((err) => console.error('Failed to delete expected rate', err));
                }
                editingRateIndex = -1;
                isInputRendered = false;
                renderList();
                // Return focus to list for keyboard navigation
                setTimeout(() => listEl.focus(), 0);
            };
            
            // Handle cancel (restore previous value)
            const handleCancel = () => {
                // Restore previous value
                if (previousValue !== undefined) {
                    expectedRates[t.topic] = previousValue;
                } else {
                    delete expectedRates[t.topic];
                }
                editingRateIndex = -1;
                isInputRendered = false;
                renderList();
                // Return focus to list for keyboard navigation
                setTimeout(() => listEl.focus(), 0);
            };
            
            input.addEventListener("keydown", (e) => {
                if (e.key === "Enter") {
                    e.preventDefault();
                    e.stopPropagation();
                    handleSubmit();
                } else if (e.key === "Escape") {
                    e.preventDefault();
                    e.stopPropagation();
                    handleCancel();
                } else {
                    // Stop other key events from propagating
                    e.stopPropagation();
                }
            });
            
            // Handle blur to close input
            input.addEventListener("blur", () => {
                // Small delay to allow clicking on other UI elements
                setTimeout(() => {
                    if (editingRateIndex === i) {
                        handleCancel();
                    }
                }, 100);
            });
            
            // Prevent click from propagating
            input.addEventListener("click", (e) => {
                e.stopPropagation();
            });
            
            expectedRateCell.appendChild(input);
            
            // Mark that input is now rendered
            isInputRendered = true;
            
            // Auto-focus the input after render
            setTimeout(() => {
                input.focus();
                input.select();
            }, 50);
        } else if (expectedRates[t.topic] !== undefined) {
            // Show expected rate if set (regardless of monitoring status)
            const expectedHz = expectedRates[t.topic];
            const actualHz = t.msg_rate_hz || 0;
            
            // Check if within 10% tolerance (only if monitored and has actual rate)
            let colorClass = "";
            if (t.monitored && actualHz > 0) {
                const tolerance = expectedHz * 0.1;
                if (Math.abs(actualHz - expectedHz) <= tolerance) {
                    colorClass = "within-tolerance";
                } else {
                    colorClass = "out-of-tolerance";
                }
            }
            
            expectedRateCell.className = `expected-rate ${colorClass}`;
            expectedRateCell.textContent = `${expectedHz.toFixed(1)} Hz`;
        } else {
            // Empty cell (no expected rate set)
            expectedRateCell.textContent = "";
        }

        item.appendChild(name);
        item.appendChild(badge);
        item.appendChild(rate);
        item.appendChild(expectedRateCell);

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
    if (!filteredTopics.length) return;
    selectedIndex = clamp(i, 0, filteredTopics.length - 1);
    renderList();
    ensureSelectedInView();
    const topic = filteredTopics[selectedIndex];
    // for now: log; you can replace with pushing selection to backend or opening detail pane
    console.log("Selected topic:", topic);
    if (opts.userAction) {
        // small feedback
        setStatus(`selected ${topic.topic}`);
    }
}

// ======= Keyboard handling =======
function onKeyDown(e) {
    if (
        ![
            "ArrowUp",
            "ArrowDown",
            "Enter",
            " ",
            "PageUp",
            "PageDown",
            "Home",
            "End",
        ].includes(e.key)
    )
        return;
    e.preventDefault();
    if (!filteredTopics.length) return;
    // print the key pressed to console
    console.log(`Key pressed: ${e.key}`);
    
    // Space bar toggles monitoring
    if (e.key === ' ') {
        e.preventDefault();
        const t = filteredTopics[selectedIndex];
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
    
    // Enter key opens expected rate input
    if (e.key === 'Enter') {
        e.preventDefault();
        const t = filteredTopics[selectedIndex];
        if (!t) return;
        editingRateIndex = selectedIndex;
        renderList();
        return;
    }
    if (e.key === "ArrowUp") {
        setSelectedIndex(
            selectedIndex <= 0 ? 0 : selectedIndex - 1,
            { userAction: true },
        );
    } else if (e.key === "ArrowDown") {
        setSelectedIndex(
            selectedIndex >= filteredTopics.length - 1
                ? filteredTopics.length - 1
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
        setSelectedIndex(filteredTopics.length - 1, { userAction: true });
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

// ======= Search handling =======
function expandSearch() {
    isSearchExpanded = true;
    searchInput.classList.remove('collapsed');
    searchInput.classList.add('expanded');
    searchToggleBtn.classList.add('hidden');
    setTimeout(() => searchInput.focus(), 100);
}

function collapseSearch() {
    isSearchExpanded = false;
    searchInput.classList.remove('expanded');
    searchInput.classList.add('collapsed');
    searchToggleBtn.classList.remove('hidden');
    searchInput.value = '';
    searchQuery = '';
    clearSearchBtn.style.display = 'none';
    filterTopics();
    selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    renderList();
    listEl.focus();
}

function handleSearch() {
    searchQuery = searchInput.value;
    
    // Show/hide clear button
    if (searchQuery) {
        clearSearchBtn.style.display = 'flex';
    } else {
        clearSearchBtn.style.display = 'none';
    }
    
    // Filter and re-render
    filterTopics();
    selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    renderList();
}

function clearSearch() {
    searchInput.value = '';
    searchQuery = '';
    clearSearchBtn.style.display = 'none';
    filterTopics();
    selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    renderList();
    searchInput.focus();
}

// ======= Wire up events =======
listEl.addEventListener("keydown", onKeyDown);
pollIntervalSelect.addEventListener("change", startPolling);
refreshBtn.addEventListener("click", () => fetchTopicsOnce());

// Monitored toggle button
monitoredToggleBtn.addEventListener("click", () => {
    showMonitoredOnly = !showMonitoredOnly;
    if (showMonitoredOnly) {
        monitoredToggleBtn.classList.add('active');
    } else {
        monitoredToggleBtn.classList.remove('active');
    }
    filterTopics();
    selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    renderList();
});

// Search toggle button
searchToggleBtn.addEventListener("click", () => {
    expandSearch();
});

// Search input events
searchInput.addEventListener("input", handleSearch);
clearSearchBtn.addEventListener("click", clearSearch);

// Allow arrow keys in search input to control list navigation
searchInput.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
        e.preventDefault();
        collapseSearch();
    } else if (["ArrowUp", "ArrowDown", "Enter", " ", "PageUp", "PageDown", "Home", "End"].includes(e.key)) {
        e.preventDefault();
        onKeyDown(e);
    }
});

// Global escape handler to collapse search when expanded
document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && isSearchExpanded) {
        e.preventDefault();
        collapseSearch();
    }
});

// Click outside to collapse search
document.addEventListener("click", (e) => {
    if (isSearchExpanded) {
        const searchContainer = document.getElementById("searchContainer");
        if (!searchContainer.contains(e.target)) {
            collapseSearch();
        }
    }
});

// focus the list on page load (search is collapsed by default)
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
