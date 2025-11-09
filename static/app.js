//static/app.js

// ======= Configuration =======
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
let expandedTopicIndex = -1; // currently expanded topic index (-1 means none)
let topicDetailsCache = {}; // cache for topic details

// Node browser state
let nodes = []; // array of node objects
let filteredNodes = []; // filtered based on search
let selectedNodeIndex = -1;
let nodeSearchQuery = "";
let nodesPollTimer = null;
let expandedNodeIndex = -1; // index of expanded node (-1 means none)
let nodeDetailsCache = {}; // cache for node details
let showMonitoredNodesOnly = false; // filter to show only monitored nodes

// Modal state
let modalOpen = false;

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
const nodeMonitoredToggleBtn = document.getElementById("nodeMonitoredToggle");
let isNodeSearchExpanded = false;

// Modal elements (will be initialized after DOM loads)
let detailsModal = null;
let modalBody = null;

// ======= Helpers =======
function setStatus(s) {
    statusEl.textContent = s;
}
function clamp(i, min, max) {
    return Math.max(min, Math.min(max, i));
}

// Modal helper functions
function showModal(content) {
    if (!detailsModal || !modalBody) {
        detailsModal = document.getElementById("detailsModal");
        modalBody = document.getElementById("modalBody");
    }
    if (!detailsModal || !modalBody) {
        console.error("Modal elements not found in DOM. Make sure the modal HTML exists.");
        return;
    }
    modalBody.innerHTML = content;
    detailsModal.style.display = "flex";
    modalOpen = true;
    // Prevent body scrolling when modal is open
    document.body.style.overflow = "hidden";
}

function hideModal() {
    if (!detailsModal) {
        detailsModal = document.getElementById("detailsModal");
    }
    if (!detailsModal) {
        console.error("Modal element not found in DOM");
        return;
    }
    detailsModal.style.display = "none";
    modalOpen = false;
    document.body.style.overflow = "";
}

// Set up modal event listeners (called once on initialization)
function initModalHandlers() {
    // Get modal elements if not already cached
    if (!detailsModal) {
        detailsModal = document.getElementById("detailsModal");
        modalBody = document.getElementById("modalBody");
    }
    if (!detailsModal) {
        console.error("Cannot initialize modal handlers - modal not found in DOM");
        return;
    }
    
    // Close on click outside modal content
    detailsModal.addEventListener("click", (e) => {
        if (e.target === detailsModal) {
            hideModal();
        }
    });
    
    // Close on any key press
    document.addEventListener("keydown", (e) => {
        if (modalOpen) {
            e.preventDefault();
            hideModal();
        }
    });
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

// Update selection without re-rendering entire list
function updateTopicSelection(newIndex) {
    const oldIndex = selectedIndex;
    selectedIndex = newIndex;
    
    // Update classes on affected items
    const items = listEl.querySelectorAll('.topic');
    if (items[oldIndex]) {
        items[oldIndex].classList.remove('selected');
    }
    if (items[newIndex]) {
        items[newIndex].classList.add('selected');
        items[newIndex].scrollIntoView({ block: "nearest", inline: "nearest" });
    }
    
    const t = filteredTopics[newIndex];
    if (t) {
        setStatus(`selected ${t.topic}`);
    }
}

// Build topic details HTML
function buildTopicDetailsHTML(details) {
    if (!details) {
        return `<div class="details-section"><div class="details-loading">Loading details...</div></div>`;
    }
    
    return `
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

        // When modal is open, just update rates without re-rendering
        if (modalOpen) {
            updateRatesOnly();
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
            
            // Just update selection without re-rendering
            updateTopicSelection(i);
            listEl.focus();
        });

        // double-click -> show details in modal
        item.addEventListener("dblclick", async (e) => {
            e.preventDefault();
            e.stopPropagation();
            
            // Update selection
            updateTopicSelection(i);
            
            // Show modal with loading state
            showModal('<div class="details-section">Loading topic details...</div>');
            
            // Fetch details
            const details = await fetchTopicDetails(t.topic);
            if (details && modalOpen) {
                // Update the modal content with the fetched details
                if (modalBody) {
                    modalBody.innerHTML = buildTopicDetailsHTML(details);
                }
            }
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
    if (opts.userAction) {
        setStatus(`selected ${topic.topic}`);
    }
}

// ======= Keyboard handling =======

// Helper function to create expected rate input field incrementally
function startEditingRateIncrementally(index) {
    const t = filteredTopics[index];
    if (!t) return;
    
    editingRateIndex = index;
    isInputRendered = true;
    
    // Find the topic item in the DOM
    const items = listEl.querySelectorAll('.topic');
    const topicItem = items[index];
    if (!topicItem) return;
    
    // Find the expected rate cell
    const expectedRateCell = topicItem.querySelector('.expected-rate');
    if (!expectedRateCell) return;
    
    // Store the previous value for cancel
    const previousValue = expectedRates[t.topic];
    
    // Create the input field
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
        
        // Update just this cell incrementally
        updateExpectedRateCell(expectedRateCell, t);
        
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
            if (editingRateIndex === index) {
                handleCancel();
            }
        }, 100);
    });
    
    // Prevent click from propagating
    input.addEventListener("click", (e) => {
        e.stopPropagation();
    });
    
    // Replace the cell content with the input
    expectedRateCell.innerHTML = '';
    expectedRateCell.appendChild(input);
    
    // Auto-focus the input
    setTimeout(() => {
        input.focus();
        input.select();
    }, 50);
}

// Helper function to update expected rate cell display
function updateExpectedRateCell(cell, topic) {
    cell.innerHTML = '';
    
    if (expectedRates[topic.topic] !== undefined) {
        const expectedHz = expectedRates[topic.topic];
        const actualHz = topic.msg_rate_hz || 0;
        
        // Check if within 10% tolerance (only if monitored and has actual rate)
        let colorClass = "";
        if (topic.monitored && actualHz > 0) {
            const tolerance = expectedHz * 0.1;
            if (Math.abs(actualHz - expectedHz) <= tolerance) {
                colorClass = "within-tolerance";
            } else {
                colorClass = "out-of-tolerance";
            }
        }
        
        cell.className = `expected-rate ${colorClass}`;
        cell.textContent = `${expectedHz.toFixed(1)} Hz`;
    } else {
        cell.className = 'expected-rate';
        cell.textContent = "";
    }
}

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
        
        // Optimistic UI update - toggle monitored state
        const wasMonitored = t.monitored;
        t.monitored = !t.monitored;
        
        // Find the topic item in the DOM and update it incrementally
        const items = listEl.querySelectorAll('.topic');
        const currentItem = items[selectedIndex];
        if (currentItem) {
            // Update monitored class
            if (t.monitored) {
                currentItem.classList.add('monitored');
            } else {
                currentItem.classList.remove('monitored');
            }
            
            // Update badge
            const badgeEl = currentItem.querySelector('.badge');
            if (badgeEl) {
                badgeEl.textContent = t.monitored ? "monitored" : "";
            }
            
            // Update rate display if needed
            const rateEl = currentItem.querySelector('.meta.rate');
            if (rateEl) {
                rateEl.textContent = t.monitored ? `${Number(t.msg_rate_hz).toFixed(1)} Hz` : "";
            }
        }
        
        // Toggle monitored on backend
        if (!t.monitored) {
            fetch(`/api/monitor/${topicEnc}`, { method: 'DELETE' })
                .catch((err) => {
                    console.error('Failed to stop monitor', err);
                    // Revert optimistic update on error
                    t.monitored = wasMonitored;
                    fetchTopicsOnce();
                });
        } else {
            fetch(`/api/monitor/${topicEnc}`, { method: 'POST' })
                .catch((err) => {
                    console.error('Failed to start monitor', err);
                    // Revert optimistic update on error
                    t.monitored = wasMonitored;
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
        startEditingRateIncrementally(selectedIndex);
        return;
    }
    if (e.key === "ArrowUp") {
        const newIndex = selectedIndex <= 0 ? 0 : selectedIndex - 1;
        updateTopicSelection(newIndex);
    } else if (e.key === "ArrowDown") {
        const newIndex = selectedIndex >= filteredTopics.length - 1
            ? filteredTopics.length - 1
            : selectedIndex + 1;
        updateTopicSelection(newIndex);
    } else if (e.key === "PageUp") {
        const visible = Math.max(1, Math.floor(listEl.clientHeight / 48));
        const newIndex = Math.max(0, selectedIndex - visible);
        updateTopicSelection(newIndex);
    } else if (e.key === "PageDown") {
        const visible = Math.max(1, Math.floor(listEl.clientHeight / 48));
        const newIndex = Math.min(filteredTopics.length - 1, selectedIndex + visible);
        updateTopicSelection(newIndex);
    } else if (e.key === "Home") {
        updateTopicSelection(0);
    } else if (e.key === "End") {
        updateTopicSelection(filteredTopics.length - 1);
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
    // Don't clear the search query - keep the filter active
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
        // Clear node search when switching to topics
        if (nodeSearchQuery) {
            clearNodeSearch();
            collapseNodeSearch();
        }
        stopNodesPolling();
        startPolling();
        listEl.focus();
    } else if (tabName === 'nodes') {
        document.getElementById('nodesTab').classList.add('active');
        // Clear topic search when switching to nodes
        if (searchQuery) {
            clearSearch();
            collapseSearch();
        }
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


// Update node selection without re-rendering entire list
function updateNodeSelection(newIndex) {
    const oldIndex = selectedNodeIndex;
    selectedNodeIndex = newIndex;
    
    // Update classes on DOM elements
    const items = nodeListEl.querySelectorAll('.node');
    if (items[oldIndex]) {
        items[oldIndex].classList.remove('selected');
        items[oldIndex].setAttribute('aria-selected', 'false');
    }
    if (items[newIndex]) {
        items[newIndex].classList.add('selected');
        items[newIndex].setAttribute('aria-selected', 'true');
        // Scroll into view
        items[newIndex].scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    }
}

// Build node details HTML
function buildNodeDetailsHTML(details) {
    if (!details) {
        return '<div class="details-section"><div class="details-loading">Loading details...</div></div>';
    }
    
    return `
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

function filterNodes() {
    let result = nodes;
    
    // Apply search filter
    if (nodeSearchQuery) {
        result = result.filter(n => 
            n.full_name.toLowerCase().includes(nodeSearchQuery.toLowerCase()) ||
            n.name.toLowerCase().includes(nodeSearchQuery.toLowerCase())
        );
    }
    // Apply monitored filter if toggle is on and search is NOT active
    else if (showMonitoredNodesOnly) {
        result = result.filter(n => n.monitored);
    }
    
    filteredNodes = result;
    
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
        
        // When modal is open, don't re-render list
        if (!modalOpen) {
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
            // Just update selection without re-rendering
            updateNodeSelection(i);
            nodeListEl.focus();
        });
        
        // double-click -> show details in modal
        item.addEventListener("dblclick", async (e) => {
            e.preventDefault();
            e.stopPropagation();
            
            // Update selection
            updateNodeSelection(i);
            
            // Show modal with loading state
            showModal('<div class="details-section">Loading node details...</div>');
            
            // Fetch details
            const details = await fetchNodeDetails(n.full_name);
            if (details && modalOpen) {
                // Update the modal content with the fetched details
                modalBody.innerHTML = buildNodeDetailsHTML(details);
            }
        });
        
        nodeListEl.appendChild(item);
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
        
        // Optimistic UI update - toggle monitored state
        const wasMonitored = n.monitored;
        n.monitored = !n.monitored;
        if (n.monitored) {
            n.status = "active";
        } else {
            n.status = null;
        }
        
        // Find the node item in the DOM and update it incrementally
        const items = nodeListEl.querySelectorAll('.node');
        const currentItem = items[selectedNodeIndex];
        if (currentItem) {
            // Update status badge
            const statusEl = currentItem.querySelector('.node-status');
            if (statusEl) {
                statusEl.innerHTML = '';
                if (n.monitored) {
                    const badge = document.createElement("span");
                    badge.className = "badge node-status-badge status-active";
                    badge.textContent = "active";
                    statusEl.appendChild(badge);
                }
            }
        }
        
        // Toggle monitored on backend
        if (!n.monitored) {
            fetch(`/api/node_monitor/${nodeEnc}`, { method: 'DELETE' })
                .catch((err) => {
                    console.error('Failed to stop node monitor', err);
                    // Revert optimistic update on error
                    n.monitored = wasMonitored;
                    fetchNodesOnce();
                });
        } else {
            fetch(`/api/node_monitor/${nodeEnc}`, { method: 'POST' })
                .catch((err) => {
                    console.error('Failed to start node monitor', err);
                    // Revert optimistic update on error
                    n.monitored = wasMonitored;
                    fetchNodesOnce();
                });
        }
        return;
    }
    
    if (e.key === "ArrowUp") {
        const newIndex = selectedNodeIndex <= 0 ? 0 : selectedNodeIndex - 1;
        updateNodeSelection(newIndex);
    } else if (e.key === "ArrowDown") {
        const newIndex = selectedNodeIndex >= filteredNodes.length - 1
            ? filteredNodes.length - 1
            : selectedNodeIndex + 1;
        updateNodeSelection(newIndex);
    } else if (e.key === "PageUp") {
        const visible = Math.max(1, Math.floor(nodeListEl.clientHeight / 48));
        const newIndex = Math.max(0, selectedNodeIndex - visible);
        updateNodeSelection(newIndex);
    } else if (e.key === "PageDown") {
        const visible = Math.max(1, Math.floor(nodeListEl.clientHeight / 48));
        const newIndex = Math.min(filteredNodes.length - 1, selectedNodeIndex + visible);
        updateNodeSelection(newIndex);
    } else if (e.key === "Home") {
        updateNodeSelection(0);
    } else if (e.key === "End") {
        updateNodeSelection(filteredNodes.length - 1);
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
    // Don't clear the search query - keep the filter active
    nodeListEl.focus();
}

function handleNodeSearch() {
    nodeSearchQuery = nodeSearchInput.value;
    
    if (nodeSearchQuery) {
        nodeClearSearchBtn.style.display = 'flex';
    } else {
        nodeClearSearchBtn.style.display = 'none';
    }
    
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
    filterTopics();
    selectedIndex = filteredTopics.length > 0 ? 0 : -1;
    renderList();
    listEl.focus();
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
        // Escape clears the search entirely
        clearSearch();
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
        // Escape clears the search entirely
        clearSearch();
        collapseSearch();
    }
});

// Click outside to collapse search (only if search is empty)
document.addEventListener("click", (e) => {
    if (isSearchExpanded) {
        const searchContainer = document.getElementById("searchContainer");
        if (!searchContainer.contains(e.target)) {
            // Only collapse if there's no active search query
            if (!searchQuery) {
                collapseSearch();
            }
        }
    }
});

// focus the list on page load (search is collapsed by default)
window.addEventListener("load", () => {
    listEl.focus();
    startPolling();
    initModalHandlers(); // Initialize modal event handlers
    
    // Test that modal exists
    const testModal = document.getElementById("detailsModal");
    const testBody = document.getElementById("modalBody");
    console.log("Modal elements on load:", {
        modalExists: !!testModal,
        bodyExists: !!testBody,
        modalDisplay: testModal ? window.getComputedStyle(testModal).display : 'N/A'
    });
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

// Node monitored toggle button (checkbox)
nodeMonitoredToggleBtn.addEventListener("change", () => {
    showMonitoredNodesOnly = nodeMonitoredToggleBtn.checked;
    filterNodes();
    selectedNodeIndex = filteredNodes.length > 0 ? 0 : -1;
    renderNodeList();
    nodeListEl.focus();
});

// Node search input events
nodeSearchInput.addEventListener("input", handleNodeSearch);
nodeClearSearchBtn.addEventListener("click", clearNodeSearch);

// Allow arrow keys in node search input to control list navigation
nodeSearchInput.addEventListener("keydown", (e) => {
    if (e.key === "Escape") {
        e.preventDefault();
        // Escape clears the search entirely
        clearNodeSearch();
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
        // Escape clears the search entirely
        clearNodeSearch();
        collapseNodeSearch();
    }
});

// Click outside to collapse node search (only if search is empty)
document.addEventListener("click", (e) => {
    if (isNodeSearchExpanded) {
        const nodeSearchContainer = document.getElementById("nodeSearchContainer");
        if (!nodeSearchContainer.contains(e.target)) {
            // Only collapse if there's no active search query
            if (!nodeSearchQuery) {
                collapseNodeSearch();
            }
        }
    }
});
