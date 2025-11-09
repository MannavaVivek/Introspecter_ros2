// ======= CONFIG =======
// Set API_URL to the endpoint that returns topic listing.
// Acceptable response shapes:
// 1) ["topic1", "topic2", ...]
// 2) {"topics": ["topic1", ...]}
// 3) {"topic1": {...}, "topic2": {...}} -> uses Object.keys()
// Use a relative URL so the browser won't trigger CORS when the frontend is
// served from the same host/port (127.0.0.1 vs localhost differences).
const API_URL = "/api/topics"; // change if your backend uses another path or host
const NODES_API_URL = "/api/nodes"; // API endpoint for nodes

// ======= State =======
let currentTab = "topics"; // current active tab
let topics = []; // array of strings
let filteredTopics = []; // filtered based on search
let selectedIndex = -1;
let pollTimer = null;
let searchQuery = "";
let expectedRates = {}; // store expected rates per topic: { "topic_name": expectedHz }
let editingRateIndex = -1; // index of topic currently being edited
let isInputRendered = false; // track if input is currently rendered
let showMonitoredOnly = false; // filter to show only monitored topics
let expandedTopicIndex = -1; // index of expanded topic (-1 means none)
let topicDetailsCache = {}; // cache for topic details

// Node browser state
let nodes = []; // array of node objects
let filteredNodes = []; // filtered based on search
let selectedNodeIndex = -1;
let nodeSearchQuery = "";
let nodesPollTimer = null;
let expandedNodeIndex = -1; // index of expanded node (-1 means none)
let nodeDetailsCache = {}; // cache for node details

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

// Node browser DOM elements
const nodeListEl = document.getElementById("nodeList");
const nodeEmptyEl = document.getElementById("nodeEmpty");
const nodeSearchInput = document.getElementById("nodeSearchInput");
const nodeClearSearchBtn = document.getElementById("nodeClearSearch");
const nodeSearchToggleBtn = document.getElementById("nodeSearchToggle");
let isNodeSearchExpanded = false;

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

// Fetch detailed information about a topic
async function fetchTopicDetails(topicName, forceRefresh = false) {
    if (!forceRefresh && topicDetailsCache[topicName]) {
        return topicDetailsCache[topicName];
    }
    
    try {
        const response = await fetch(`/api/topic_info/${encodeURIComponent(topicName)}`, { cache: "no-store" });
        if (!response.ok) throw new Error("HTTP " + response.status);
        const data = await response.json();
        topicDetailsCache[topicName] = data;
        return data;
    } catch (err) {
        console.error("Failed to fetch topic details:", err);
        return null;
    }
}

// Update the expanded topic details without full re-render
function updateExpandedTopicDetails() {
    if (expandedTopicIndex < 0 || expandedTopicIndex >= filteredTopics.length) return;
    
    const t = filteredTopics[expandedTopicIndex];
    const details = topicDetailsCache[t.topic];
    if (!details) return;
    
    // Find the details element in the DOM
    const detailsEl = listEl.querySelector('.topic-details');
    if (!detailsEl) return;
    
    // Update the details content
    detailsEl.innerHTML = `
        <div class="details-section">
            <div class="details-header">Topic Information</div>
            <div class="details-grid">
                <div class="detail-item">
                    <span class="detail-label">Type:</span>
                    <span class="detail-value monospace">${details.type || 'N/A'}</span>
                </div>
                <div class="detail-item">
                    <span class="detail-label">Publishers:</span>
                    <span class="detail-value">${details.publishers_count || 0}</span>
                </div>
                <div class="detail-item">
                    <span class="detail-label">Subscribers:</span>
                    <span class="detail-value">${details.subscribers_count || 0}</span>
                </div>
                ${details.monitored ? `
                    <div class="detail-item">
                        <span class="detail-label">Sample Count:</span>
                        <span class="detail-value">${details.sample_count || 0}</span>
                    </div>
                ` : ''}
            </div>
            ${details.publishers && details.publishers.length > 0 ? `
                <div class="details-subheader">Publishing Nodes</div>
                <div class="details-list">
                    ${details.publishers.map(p => {
                        const ns = p.node_namespace && p.node_namespace !== '/' ? p.node_namespace : '';
                        return `<div class="list-item monospace">${ns}/${p.node_name}</div>`.replace('//', '/');
                    }).join('')}
                </div>
            ` : ''}
            ${details.subscribers && details.subscribers.length > 0 ? `
                <div class="details-subheader">Subscribing Nodes</div>
                <div class="details-list">
                    ${details.subscribers.map(s => {
                        const ns = s.node_namespace && s.node_namespace !== '/' ? s.node_namespace : '';
                        return `<div class="list-item monospace">${ns}/${s.node_name}</div>`.replace('//', '/');
                    }).join('')}
                </div>
            ` : ''}
        </div>
    `;
}

// Collapse expanded topic
function collapseTopicDetails() {
    if (expandedTopicIndex >= 0) {
        expandedTopicIndex = -1;
        renderList();
    }
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

        // normalize into objects: { topic, type?, monitored?, node_rate_hz?, msg_rate_hz?, expected_rate_hz? }
        const toObj = (t) => {
            if (typeof t === 'string') return { topic: String(t), type: '', monitored: false, node_rate_hz: 0, msg_rate_hz: 0, last_update_ns: null, expected_rate_hz: null };
            if (t && typeof t === 'object') return {
                topic: String(t.topic || t.name || ''),
                type: t.type || '',
                monitored: Boolean(t.monitored),
                node_rate_hz: Number(t.node_rate_hz || 0),
                msg_rate_hz: Number(t.msg_rate_hz || 0),
                last_update_ns: t.last_update_ns || null,
                expected_rate_hz: t.expected_rate_hz !== null && t.expected_rate_hz !== undefined ? Number(t.expected_rate_hz) : null,
            };
            return { topic: String(t), type: '', monitored: false, node_rate_hz: 0, msg_rate_hz: 0, last_update_ns: null, expected_rate_hz: null };
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
        
        // Clear and reload expected rates from backend data to ensure sync
        expectedRates = {};
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

        // If details are expanded, only update rates to avoid flashing
        if (expandedTopicIndex >= 0) {
            updateRatesOnly();
            // Also refresh the expanded topic details
            const expandedTopic = filteredTopics[expandedTopicIndex];
            if (expandedTopic) {
                await fetchTopicDetails(expandedTopic.topic, true); // Force refresh
                // Re-render just the details section
                updateExpandedTopicDetails();
            }
        } else {
            renderList();
        }
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
        emptyEl.textContent = searchQuery ? `No topics matching "${searchQuery}"` : "No topics being monitored";
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

        // click -> select and collapse details
        item.addEventListener("click", (e) => {
            // Don't trigger if clicking on input field
            if (e.target.classList.contains('expected-rate-input')) return;
            setSelectedIndex(i, { userAction: true });
            // Always collapse details on single click
            if (expandedTopicIndex >= 0) {
                collapseTopicDetails();
            }
        });

        // double-click -> expand details
        item.addEventListener("dblclick", async (e) => {
            e.preventDefault();
            if (expandedTopicIndex === i) {
                // Already expanded, collapse it
                collapseTopicDetails();
            } else {
                // Expand this item
                expandedTopicIndex = i;
                setSelectedIndex(i, { userAction: false });
                renderList(); // Re-render to show loading
                
                // Fetch details
                const details = await fetchTopicDetails(t.topic);
                if (details && expandedTopicIndex === i) {
                    // Still expanded, render with details
                    renderList();
                }
            }
        });

        listEl.appendChild(item);
        
        // Add expanded details section if this is the expanded item
        if (i === expandedTopicIndex) {
            const detailsRow = document.createElement("div");
            detailsRow.className = "topic-details";
            
            // Prevent clicks inside details from bubbling up
            detailsRow.addEventListener("click", (e) => {
                e.stopPropagation();
            });
            
            const details = topicDetailsCache[t.topic];
            if (details) {
                detailsRow.innerHTML = `
                    <div class="details-section">
                        <div class="details-header">Topic Information</div>
                        <div class="details-grid">
                            <div class="detail-item">
                                <span class="detail-label">Type:</span>
                                <span class="detail-value monospace">${details.type || 'N/A'}</span>
                            </div>
                            <div class="detail-item">
                                <span class="detail-label">Publishers:</span>
                                <span class="detail-value">${details.publishers_count || 0}</span>
                            </div>
                            <div class="detail-item">
                                <span class="detail-label">Subscribers:</span>
                                <span class="detail-value">${details.subscribers_count || 0}</span>
                            </div>
                            ${details.monitored ? `
                                <div class="detail-item">
                                    <span class="detail-label">Sample Count:</span>
                                    <span class="detail-value">${details.sample_count || 0}</span>
                                </div>
                            ` : ''}
                        </div>
                        ${details.publishers && details.publishers.length > 0 ? `
                            <div class="details-subheader">Publishing Nodes</div>
                            <div class="details-list">
                                ${details.publishers.map(p => {
                                    const ns = p.node_namespace && p.node_namespace !== '/' ? p.node_namespace : '';
                                    return `<div class="list-item monospace">${ns}/${p.node_name}</div>`.replace('//', '/');
                                }).join('')}
                            </div>
                        ` : ''}
                        ${details.subscribers && details.subscribers.length > 0 ? `
                            <div class="details-subheader">Subscribing Nodes</div>
                            <div class="details-list">
                                ${details.subscribers.map(s => {
                                    const ns = s.node_namespace && s.node_namespace !== '/' ? s.node_namespace : '';
                                    return `<div class="list-item monospace">${ns}/${s.node_name}</div>`.replace('//', '/');
                                }).join('')}
                            </div>
                        ` : ''}
                    </div>
                `;
            } else {
                detailsRow.innerHTML = `
                    <div class="details-section">
                        <div class="details-loading">Loading details...</div>
                    </div>
                `;
            }
            
            listEl.appendChild(detailsRow);
        }
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
    if (opts.userAction) {
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
        collapseTopicDetails();
        setSelectedIndex(
            selectedIndex <= 0 ? 0 : selectedIndex - 1,
            { userAction: true },
        );
    } else if (e.key === "ArrowDown") {
        collapseTopicDetails();
        setSelectedIndex(
            selectedIndex >= filteredTopics.length - 1
                ? filteredTopics.length - 1
                : selectedIndex + 1,
            { userAction: true },
        );
    } else if (e.key === "PageUp") {
        collapseTopicDetails();
        const visible = Math.max(
            1,
            Math.floor(listEl.clientHeight / 48),
        );
        setSelectedIndex(selectedIndex - visible, {
            userAction: true,
        });
    } else if (e.key === "PageDown") {
        collapseTopicDetails();
        const visible = Math.max(
            1,
            Math.floor(listEl.clientHeight / 48),
        );
        setSelectedIndex(selectedIndex + visible, {
            userAction: true,
        });
    } else if (e.key === "Home") {
        collapseTopicDetails();
        setSelectedIndex(0, { userAction: true });
    } else if (e.key === "End") {
        collapseTopicDetails();
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
    
    // Collapse details when searching
    collapseTopicDetails();
    
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

// ======= Tab Switching =======
function switchTab(tabName) {
    currentTab = tabName;
    
    // Update tab buttons
    document.querySelectorAll('.tab').forEach(tab => {
        if (tab.dataset.tab === tabName) {
            tab.classList.add('active');
        } else {
            tab.classList.remove('active');
        }
    });
    
    // Update tab content
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });
    
    if (tabName === 'topics') {
        document.getElementById('topicsTab').classList.add('active');
        stopNodesPolling();
        startPolling();
        listEl.focus();
    } else if (tabName === 'nodes') {
        document.getElementById('nodesTab').classList.add('active');
        stopPolling();
        startNodesPolling();
        nodeListEl.focus();
    }
}

// ======= Node Browser Functions =======
// Fetch detailed information about a node
async function fetchNodeDetails(nodeName, forceRefresh = false) {
    if (!forceRefresh && nodeDetailsCache[nodeName]) {
        return nodeDetailsCache[nodeName];
    }
    
    try {
        const response = await fetch(`/api/node_info/${encodeURIComponent(nodeName)}`, { cache: "no-store" });
        if (!response.ok) throw new Error("HTTP " + response.status);
        const data = await response.json();
        nodeDetailsCache[nodeName] = data;
        return data;
    } catch (err) {
        console.error("Failed to fetch node details:", err);
        return null;
    }
}

// Update the expanded node details without full re-render
function updateExpandedNodeDetails() {
    if (expandedNodeIndex < 0 || expandedNodeIndex >= filteredNodes.length) return;
    
    const n = filteredNodes[expandedNodeIndex];
    const details = nodeDetailsCache[n.full_name];
    if (!details) return;
    
    // Find the details element in the DOM
    const detailsEl = nodeListEl.querySelector('.node-details');
    if (!detailsEl) return;
    
    // Update the details content
    detailsEl.innerHTML = `
        <div class="details-section">
            <div class="details-header">Node Information</div>
            <div class="details-grid">
                <div class="detail-item">
                    <span class="detail-label">Full Name:</span>
                    <span class="detail-value monospace">${details.full_name || 'N/A'}</span>
                </div>
                <div class="detail-item">
                    <span class="detail-label">Publishers:</span>
                    <span class="detail-value">${details.publishers?.length || 0}</span>
                </div>
                <div class="detail-item">
                    <span class="detail-label">Subscribers:</span>
                    <span class="detail-value">${details.subscribers?.length || 0}</span>
                </div>
                <div class="detail-item">
                    <span class="detail-label">Services:</span>
                    <span class="detail-value">${details.services?.length || 0}</span>
                </div>
            </div>
            ${details.publishers && details.publishers.length > 0 ? `
                <div class="details-subheader">Published Topics</div>
                <div class="details-list">
                    ${details.publishers.map(p => `
                        <div class="list-item">
                            <span class="monospace topic-name">${p.topic}</span>
                            <span class="topic-type">${p.type}</span>
                        </div>
                    `).join('')}
                </div>
            ` : ''}
            ${details.subscribers && details.subscribers.length > 0 ? `
                <div class="details-subheader">Subscribed Topics</div>
                <div class="details-list">
                    ${details.subscribers.map(s => `
                        <div class="list-item">
                            <span class="monospace topic-name">${s.topic}</span>
                            <span class="topic-type">${s.type}</span>
                        </div>
                    `).join('')}
                </div>
            ` : ''}
            ${details.services && details.services.length > 0 ? `
                <div class="details-subheader">Services</div>
                <div class="details-list">
                    ${details.services.map(svc => `
                        <div class="list-item">
                            <span class="monospace topic-name">${svc.name}</span>
                            <span class="topic-type">${svc.type}</span>
                        </div>
                    `).join('')}
                </div>
            ` : ''}
        </div>
    `;
}

// Collapse expanded node
function collapseNodeDetails() {
    if (expandedNodeIndex >= 0) {
        expandedNodeIndex = -1;
        renderNodeList();
    }
}

function filterNodes() {
    if (nodeSearchQuery) {
        filteredNodes = nodes.filter(n => 
            n.full_name.toLowerCase().includes(nodeSearchQuery.toLowerCase()) ||
            n.name.toLowerCase().includes(nodeSearchQuery.toLowerCase())
        );
    } else {
        filteredNodes = nodes;
    }
    
    // Reset selection when filter changes
    if (selectedNodeIndex >= filteredNodes.length) {
        selectedNodeIndex = filteredNodes.length > 0 ? 0 : -1;
    }
}

async function fetchNodesOnce() {
    try {
        const r = await fetch(NODES_API_URL, { cache: "no-store" });
        if (!r.ok) throw new Error("HTTP " + r.status);
        const data = await r.json();
        
        if (Array.isArray(data)) {
            nodes = data;
        } else {
            nodes = [];
        }
        
        // Apply filter
        filterNodes();
        
        // Keep selection anchored if possible
        if (selectedNodeIndex >= 0 && filteredNodes.length > 0) {
            selectedNodeIndex = clamp(selectedNodeIndex, 0, filteredNodes.length - 1);
        } else if (filteredNodes.length > 0) {
            selectedNodeIndex = 0;
        } else {
            selectedNodeIndex = -1;
        }
        
        // If details are expanded, don't re-render to avoid flashing
        if (expandedNodeIndex >= 0) {
            // Refresh the expanded node details
            const expandedNode = filteredNodes[expandedNodeIndex];
            if (expandedNode) {
                await fetchNodeDetails(expandedNode.full_name, true); // Force refresh
                // Re-render just the details section
                updateExpandedNodeDetails();
            }
        } else {
            renderNodeList();
        }
    } catch (err) {
        console.error("Failed to fetch nodes:", err);
        nodeEmptyEl.textContent = "Error: " + (err.message || err);
        nodeListEl.innerHTML = "";
        nodeListEl.appendChild(nodeEmptyEl);
    }
}

function renderNodeList() {
    nodeListEl.innerHTML = "";
    
    if (!filteredNodes.length) {
        nodeEmptyEl.textContent = nodeSearchQuery ? `No nodes matching "${nodeSearchQuery}"` : "No nodes found";
        nodeListEl.appendChild(nodeEmptyEl);
        return;
    }
    
    // Add column headers
    const header = document.createElement("div");
    header.className = "node-header";
    header.innerHTML = `
        <div class="header-cell">Node Name</div>
        <div class="header-cell">Namespace</div>
        <div class="header-cell">Status</div>
    `;
    nodeListEl.appendChild(header);
    
    filteredNodes.forEach((n, i) => {
        const item = document.createElement("div");
        item.className = "node" + (i === selectedNodeIndex ? " selected" : "");
        item.setAttribute("role", "option");
        item.setAttribute("aria-selected", String(i === selectedNodeIndex));
        
        const name = document.createElement("div");
        name.className = "node-name";
        name.textContent = n.name;
        
        const namespace = document.createElement("div");
        namespace.className = "node-namespace";
        namespace.textContent = n.namespace;
        
        // Add status column
        const status = document.createElement("div");
        status.className = "node-status";
        
        if (n.monitored) {
            const badge = document.createElement("span");
            badge.className = "badge node-status-badge";
            if (n.status === "MIA") {
                badge.className += " status-mia";
                badge.textContent = "MIA";
            } else {
                badge.className += " status-active";
                badge.textContent = "active";
            }
            status.appendChild(badge);
        }
        
        item.appendChild(name);
        item.appendChild(namespace);
        item.appendChild(status);
        
        item.addEventListener("click", () => {
            setSelectedNodeIndex(i, { userAction: true });
            // Always collapse details on single click
            if (expandedNodeIndex >= 0) {
                collapseNodeDetails();
            }
            nodeListEl.focus();
        });
        
        // double-click -> expand details
        item.addEventListener("dblclick", async (e) => {
            e.preventDefault();
            if (expandedNodeIndex === i) {
                // Already expanded, collapse it
                collapseNodeDetails();
            } else {
                // Expand this item
                expandedNodeIndex = i;
                setSelectedNodeIndex(i, { userAction: false });
                renderNodeList(); // Re-render to show loading
                
                // Fetch details
                const details = await fetchNodeDetails(n.full_name);
                if (details && expandedNodeIndex === i) {
                    // Still expanded, render with details
                    renderNodeList();
                }
            }
        });
        
        nodeListEl.appendChild(item);
        
        // Add expanded details section if this is the expanded item
        if (i === expandedNodeIndex) {
            const detailsRow = document.createElement("div");
            detailsRow.className = "node-details";
            
            // Prevent clicks inside details from bubbling up
            detailsRow.addEventListener("click", (e) => {
                e.stopPropagation();
            });
            
            const details = nodeDetailsCache[n.full_name];
            if (details) {
                detailsRow.innerHTML = `
                    <div class="details-section">
                        <div class="details-header">Node Information</div>
                        <div class="details-grid">
                            <div class="detail-item">
                                <span class="detail-label">Full Name:</span>
                                <span class="detail-value monospace">${details.full_name || 'N/A'}</span>
                            </div>
                            <div class="detail-item">
                                <span class="detail-label">Publishers:</span>
                                <span class="detail-value">${details.publishers?.length || 0}</span>
                            </div>
                            <div class="detail-item">
                                <span class="detail-label">Subscribers:</span>
                                <span class="detail-value">${details.subscribers?.length || 0}</span>
                            </div>
                            <div class="detail-item">
                                <span class="detail-label">Services:</span>
                                <span class="detail-value">${details.services?.length || 0}</span>
                            </div>
                        </div>
                        ${details.publishers && details.publishers.length > 0 ? `
                            <div class="details-subheader">Published Topics</div>
                            <div class="details-list">
                                ${details.publishers.map(p => `
                                    <div class="list-item">
                                        <span class="monospace topic-name">${p.topic}</span>
                                        <span class="topic-type">${p.type}</span>
                                    </div>
                                `).join('')}
                            </div>
                        ` : ''}
                        ${details.subscribers && details.subscribers.length > 0 ? `
                            <div class="details-subheader">Subscribed Topics</div>
                            <div class="details-list">
                                ${details.subscribers.map(s => `
                                    <div class="list-item">
                                        <span class="monospace topic-name">${s.topic}</span>
                                        <span class="topic-type">${s.type}</span>
                                    </div>
                                `).join('')}
                            </div>
                        ` : ''}
                        ${details.services && details.services.length > 0 ? `
                            <div class="details-subheader">Services</div>
                            <div class="details-list">
                                ${details.services.map(svc => `
                                    <div class="list-item">
                                        <span class="monospace topic-name">${svc.name}</span>
                                        <span class="topic-type">${svc.type}</span>
                                    </div>
                                `).join('')}
                            </div>
                        ` : ''}
                    </div>
                `;
            } else {
                detailsRow.innerHTML = `
                    <div class="details-section">
                        <div class="details-loading">Loading details...</div>
                    </div>
                `;
            }
            
            nodeListEl.appendChild(detailsRow);
        }
    });
    
    ensureSelectedNodeInView();
}

function ensureSelectedNodeInView() {
    if (selectedNodeIndex < 0) return;
    const children = nodeListEl.children;
    if (!children || !children[selectedNodeIndex + 1]) return; // +1 to account for header
    const node = children[selectedNodeIndex + 1];
    node.scrollIntoView({ block: "nearest", inline: "nearest" });
}

function setSelectedNodeIndex(i, opts = {}) {
    if (!filteredNodes.length) return;
    selectedNodeIndex = clamp(i, 0, filteredNodes.length - 1);
    renderNodeList();
    ensureSelectedNodeInView();
    const node = filteredNodes[selectedNodeIndex];
}

function onNodeKeyDown(e) {
    if (!["ArrowUp", "ArrowDown", " ", "PageUp", "PageDown", "Home", "End"].includes(e.key)) return;
    e.preventDefault();
    if (!filteredNodes.length) return;
    // Space bar toggles node monitoring
    if (e.key === ' ') {
        e.preventDefault();
        const n = filteredNodes[selectedNodeIndex];
        if (!n) {
            return;
        }
        const nodeEnc = encodeURIComponent(n.full_name);
        
        // optimistic UI update
        n.monitored = !n.monitored;
        if (n.monitored) {
            n.status = "active";
        } else {
            n.status = null;
        }
        renderNodeList();
        
        // toggle monitored
        if (!n.monitored) {
            // after optimistic toggle we set to unmonitored, so we must DELETE
            fetch(`/api/node_monitor/${nodeEnc}`, { method: 'DELETE' })
                .then((resp) => {
                    return fetchNodesOnce();
                })
                .catch((err) => {
                    console.error('Failed to stop node monitor', err);
                    fetchNodesOnce();
                });
        } else {
            fetch(`/api/node_monitor/${nodeEnc}`, { method: 'POST' })
                .then(async (resp) => {
                    const result = await resp.json();
                    await fetchNodesOnce();
                    const refreshedNode = filteredNodes.find(nd => nd.full_name === n.full_name);
                })
                .catch((err) => {
                    console.error('Failed to start node monitor', err);
                    fetchNodesOnce();
                });
        }
        return;
    }
    
    if (e.key === "ArrowUp") {
        collapseNodeDetails();
        setSelectedNodeIndex(selectedNodeIndex <= 0 ? 0 : selectedNodeIndex - 1, { userAction: true });
    } else if (e.key === "ArrowDown") {
        collapseNodeDetails();
        setSelectedNodeIndex(
            selectedNodeIndex >= filteredNodes.length - 1 ? filteredNodes.length - 1 : selectedNodeIndex + 1,
            { userAction: true }
        );
    } else if (e.key === "PageUp") {
        collapseNodeDetails();
        const visible = Math.max(1, Math.floor(nodeListEl.clientHeight / 48));
        setSelectedNodeIndex(selectedNodeIndex - visible, { userAction: true });
    } else if (e.key === "PageDown") {
        collapseNodeDetails();
        const visible = Math.max(1, Math.floor(nodeListEl.clientHeight / 48));
        setSelectedNodeIndex(selectedNodeIndex + visible, { userAction: true });
    } else if (e.key === "Home") {
        collapseNodeDetails();
        setSelectedNodeIndex(0, { userAction: true });
    } else if (e.key === "End") {
        collapseNodeDetails();
        setSelectedNodeIndex(filteredNodes.length - 1, { userAction: true });
    }
}

function startNodesPolling() {
    stopNodesPolling();
    const ms = Number(pollIntervalSelect.value) || 1000;
    nodesPollTimer = setInterval(fetchNodesOnce, ms);
    fetchNodesOnce();
}

function stopNodesPolling() {
    if (nodesPollTimer) {
        clearInterval(nodesPollTimer);
        nodesPollTimer = null;
    }
}

function expandNodeSearch() {
    isNodeSearchExpanded = true;
    nodeSearchInput.classList.remove('collapsed');
    nodeSearchInput.classList.add('expanded');
    nodeSearchToggleBtn.classList.add('hidden');
    setTimeout(() => nodeSearchInput.focus(), 100);
}

function collapseNodeSearch() {
    isNodeSearchExpanded = false;
    nodeSearchInput.classList.remove('expanded');
    nodeSearchInput.classList.add('collapsed');
    nodeSearchToggleBtn.classList.remove('hidden');
    nodeSearchInput.value = '';
    nodeSearchQuery = '';
    nodeClearSearchBtn.style.display = 'none';
    filterNodes();
    selectedNodeIndex = filteredNodes.length > 0 ? 0 : -1;
    renderNodeList();
    nodeListEl.focus();
}

function handleNodeSearch() {
    nodeSearchQuery = nodeSearchInput.value;
    
    if (nodeSearchQuery) {
        nodeClearSearchBtn.style.display = 'flex';
    } else {
        nodeClearSearchBtn.style.display = 'none';
    }
    
    // Collapse details when searching
    collapseNodeDetails();
    
    filterNodes();
    selectedNodeIndex = filteredNodes.length > 0 ? 0 : -1;
    renderNodeList();
}

function clearNodeSearch() {
    nodeSearchInput.value = '';
    nodeSearchQuery = '';
    nodeClearSearchBtn.style.display = 'none';
    filterNodes();
    selectedNodeIndex = filteredNodes.length > 0 ? 0 : -1;
    renderNodeList();
    nodeSearchInput.focus();
}

// ======= Wire up events =======
// Tab switching
document.querySelectorAll('.tab').forEach(tab => {
    tab.addEventListener('click', () => {
        switchTab(tab.dataset.tab);
    });
});

// Topic browser events
listEl.addEventListener("keydown", onKeyDown);
pollIntervalSelect.addEventListener("change", startPolling);
refreshBtn.addEventListener("click", () => fetchTopicsOnce());

// Monitored toggle button (checkbox)
monitoredToggleBtn.addEventListener("change", () => {
    showMonitoredOnly = monitoredToggleBtn.checked;
    collapseTopicDetails(); // Collapse details when filtering
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
    if (listEl.contains(e.target)) {
        // keep focus on list so arrow keys continue to work
        listEl.focus();
    } else if (nodeListEl.contains(e.target)) {
        // keep focus on nodeListEl so arrow keys continue to work
        nodeListEl.focus();
    }
});

// ======= Node Browser Event Listeners =======
nodeListEl.addEventListener("keydown", (e) => {
    onNodeKeyDown(e);
});

// Node search toggle button
nodeSearchToggleBtn.addEventListener("click", () => {
    expandNodeSearch();
});

// Node search input events
nodeSearchInput.addEventListener("input", handleNodeSearch);
nodeClearSearchBtn.addEventListener("click", clearNodeSearch);

// Allow arrow keys in node search input to control list navigation
nodeSearchInput.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
        e.preventDefault();
        collapseNodeSearch();
    } else if (["ArrowUp", "ArrowDown", " ", "PageUp", "PageDown", "Home", "End"].includes(e.key)) {
        e.preventDefault();
        onNodeKeyDown(e);
    }
});

// Global handler to prevent space from scrolling when lists are focused
// Use capture phase to ensure we don't interfere with element handlers
document.addEventListener("keydown", (e) => {
    // Prevent space from scrolling the page when topic/node list is focused
    if (e.key === " " && currentTab === "topics" && document.activeElement === listEl) {
        e.preventDefault();
    }
    if (e.key === " " && currentTab === "nodes" && document.activeElement === nodeListEl) {
        e.preventDefault();
    }
}, false); // Use bubbling phase, not capture

// Global escape handler to collapse node search when expanded
document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && isNodeSearchExpanded) {
        e.preventDefault();
        collapseNodeSearch();
    }
});

// Click outside to collapse node search
document.addEventListener("click", (e) => {
    if (isNodeSearchExpanded) {
        const nodeSearchContainer = document.getElementById("nodeSearchContainer");
        if (!nodeSearchContainer.contains(e.target)) {
            collapseNodeSearch();
        }
    }
});
