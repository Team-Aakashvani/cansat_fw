"""
AAKASHVANI — Build CanSat 3D IMU Viewer HTML
============================================
Extracts offline Three.js libraries from cansat_viewer.html and builds
the real-time attitude-driven 3D body viewer.
"""

import os

CAD_DIR = r"C:\Users\Lenovo\Downloads\Final_Body_Cansat"
WORKSPACE_WEB_DIR = r"C:\Users\Lenovo\Desktop\Dev_Coding\Environments\Arduino_Projects\CANSAT\cansat_fw\tools\web"

os.makedirs(WORKSPACE_WEB_DIR, exist_ok=True)

# 1. Read existing offline Three.js / GLTFLoader / OrbitControls from cansat_viewer.html
orig_viewer = os.path.join(CAD_DIR, "cansat_viewer.html")
with open(orig_viewer, "r", encoding="utf-8") as f:
    content = f.read()

# Extract from start up to </head>
head_end = content.find("</head>")
if head_end == -1:
    raise ValueError("Could not find </head> in cansat_viewer.html")

offline_head_scripts = content[:head_end]

# 2. Build the body and application script for IMU Attitude Visualization
app_body_and_script = """
    <style>
        :root {
            --bg-color: #0d1117;
            --panel-bg: rgba(18, 24, 38, 0.92);
            --panel-border: rgba(56, 189, 248, 0.25);
            --accent: #00d2ff;
            --accent-glow: rgba(0, 210, 255, 0.4);
            --text: #f1f5f9;
            --text-muted: #94a3b8;
            --pitch-color: #38bdf8;
            --roll-color: #34d399;
            --yaw-color: #f59e0b;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Consolas', monospace;
            background: radial-gradient(circle at center, #111827 0%, #030712 100%);
            color: var(--text);
            overflow: hidden;
            height: 100vh;
            width: 100vw;
            user-select: none;
        }
        #canvas-container {
            width: 100%;
            height: 100%;
            display: block;
            position: absolute;
            top: 0;
            left: 0;
            z-index: 1;
        }

        /* Top Bar */
        header {
            position: absolute;
            top: 14px;
            left: 16px;
            z-index: 100;
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--panel-border);
            padding: 10px 18px;
            border-radius: 10px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.7);
            display: flex;
            align-items: center;
            gap: 14px;
        }
        header h1 {
            font-size: 15px;
            font-weight: 700;
            letter-spacing: 0.5px;
            color: #fff;
        }
        header .badge {
            background: #0284c7;
            color: #fff;
            font-size: 11px;
            font-weight: 700;
            padding: 3px 8px;
            border-radius: 6px;
            text-transform: uppercase;
        }
        .status-indicator {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            font-size: 11px;
            font-weight: 600;
            padding: 3px 8px;
            border-radius: 6px;
            background: rgba(30, 41, 59, 0.8);
            border: 1px solid #334155;
        }
        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: #ef4444;
            box-shadow: 0 0 8px #ef4444;
            transition: all 0.3s;
        }
        .status-dot.connected {
            background: #10b981;
            box-shadow: 0 0 10px #10b981;
        }

        /* Top Right: Real-time Attitude Gauges */
        #attitude-hud {
            position: absolute;
            top: 14px;
            right: 16px;
            z-index: 100;
            display: flex;
            gap: 10px;
        }
        .hud-card {
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--panel-border);
            border-radius: 10px;
            padding: 8px 16px;
            min-width: 100px;
            text-align: center;
            box-shadow: 0 8px 32px rgba(0,0,0,0.7);
        }
        .hud-title {
            font-size: 10px;
            font-weight: 700;
            color: var(--text-muted);
            letter-spacing: 1px;
            text-transform: uppercase;
        }
        .hud-value {
            font-size: 20px;
            font-weight: 800;
            font-family: 'Consolas', monospace;
            margin-top: 2px;
        }
        .val-pitch { color: var(--pitch-color); }
        .val-roll  { color: var(--roll-color); }
        .val-yaw   { color: var(--yaw-color); }

        /* Left Sidebar: Controls & Options */
        #sidebar {
            position: absolute;
            top: 72px;
            left: 16px;
            bottom: 20px;
            width: 320px;
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--panel-border);
            border-radius: 10px;
            padding: 14px;
            overflow-y: auto;
            z-index: 100;
            box-shadow: 0 8px 32px rgba(0,0,0,0.7);
            display: flex;
            flex-direction: column;
            gap: 14px;
        }

        .section-title {
            font-size: 11px;
            font-weight: 700;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--accent);
            border-bottom: 1px solid rgba(56, 189, 248, 0.2);
            padding-bottom: 4px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .control-group {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }
        .slider-row {
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .slider-label {
            font-size: 11px;
            color: var(--text-muted);
            display: flex;
            justify-content: space-between;
        }
        input[type="range"] {
            width: 100%;
            accent-color: var(--accent);
            cursor: pointer;
            height: 6px;
            background: #1e293b;
            border-radius: 3px;
        }
        .btn-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 6px;
        }
        .btn-grid-2 {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 6px;
        }
        .btn {
            background: #1e293b;
            color: var(--text);
            border: 1px solid #334155;
            padding: 6px 8px;
            border-radius: 6px;
            font-size: 11px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.15s ease;
            text-align: center;
        }
        .btn:hover {
            background: #334155;
            border-color: var(--accent);
            color: #fff;
        }
        .btn.active {
            background: #0284c7;
            border-color: #38bdf8;
            color: #fff;
        }
        .btn-action {
            background: #0ea5e9;
            color: #fff;
            border-color: #38bdf8;
        }
        .btn-action:hover {
            background: #0284c7;
        }

        /* Checkbox Rows */
        .check-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            font-size: 11px;
            color: var(--text-muted);
        }
        .check-row input[type="checkbox"] {
            accent-color: var(--accent);
            cursor: pointer;
        }

        /* Bottom Floating Stats Bar */
        #telemetry-stats {
            position: absolute;
            bottom: 14px;
            right: 16px;
            z-index: 100;
            background: var(--panel-bg);
            backdrop-filter: blur(16px);
            border: 1px solid var(--panel-border);
            padding: 8px 16px;
            border-radius: 10px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.7);
            display: flex;
            gap: 16px;
            font-size: 11px;
            font-family: 'Consolas', monospace;
        }
        #telemetry-stats span b {
            color: var(--accent);
        }

        /* Artificial Horizon overlay in lower center */
        #horizon-overlay {
            position: absolute;
            bottom: 14px;
            left: 50%;
            transform: translateX(-50%);
            z-index: 100;
            width: 140px;
            height: 140px;
            background: rgba(15, 23, 42, 0.85);
            backdrop-filter: blur(12px);
            border: 2px solid var(--panel-border);
            border-radius: 50%;
            box-shadow: 0 8px 30px rgba(0,0,0,0.8);
            overflow: hidden;
            pointer-events: none;
        }
        #horizon-canvas {
            width: 100%;
            height: 100%;
        }

        /* Loading Overlay */
        #loading {
            position: absolute;
            inset: 0;
            z-index: 999;
            background: #090d16;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            gap: 16px;
            transition: opacity 0.5s ease;
        }
        .spinner {
            width: 48px;
            height: 48px;
            border: 4px solid #1e293b;
            border-top-color: var(--accent);
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
        @keyframes spin { 100% { transform: rotate(360deg); } }
    </style>
</head>
<body>

<div id="loading">
    <div class="spinner"></div>
    <h2>Loading CanSat 3D Assembly & IMU Engine...</h2>
    <p style="color: #64748b; font-size: 12px;">CAN-7USAT REV_H Full Assembled CAD Body</p>
</div>

<header>
    <h1>CAN-7USAT Vehicle</h1>
    <span class="badge" id="flight-state-badge">IDLE (0)</span>
    <div class="status-indicator">
        <span class="status-dot" id="status-dot"></span>
        <span id="status-text">CONNECTING...</span>
    </div>
</header>

<!-- Attitude Gauges -->
<div id="attitude-hud">
    <div class="hud-card">
        <div class="hud-title">Pitch (Tilt X)</div>
        <div class="hud-value val-pitch" id="hud-pitch">+0.0°</div>
    </div>
    <div class="hud-card">
        <div class="hud-title">Roll (Bank Y)</div>
        <div class="hud-value val-roll" id="hud-roll">+0.0°</div>
    </div>
    <div class="hud-card">
        <div class="hud-title">Yaw (Heading)</div>
        <div class="hud-value val-yaw" id="hud-yaw">0.0°</div>
    </div>
</div>

<!-- Controls Sidebar -->
<div id="sidebar">
    <!-- Telemetry Source Mode -->
    <div class="control-group">
        <div class="section-title">Telemetry Mode</div>
        <div class="btn-grid-2">
            <button class="btn active" id="btn-mode-live" onclick="setMode('live')">📡 Live Serial</button>
            <button class="btn" id="btn-mode-sim" onclick="setMode('sim')">🧪 Manual Sim</button>
        </div>
    </div>

    <!-- Interactive Simulation Sliders -->
    <div class="control-group" id="sim-controls">
        <div class="section-title">Manual IMU Rotation</div>
        <div class="slider-row">
            <div class="slider-label">
                <span style="color:var(--pitch-color);">Pitch (Nose Up/Dn)</span>
                <span id="slider-pitch-val">0°</span>
            </div>
            <input type="range" id="slider-pitch" min="-90" max="90" value="0" oninput="onSimSliderChange()">
        </div>
        <div class="slider-row">
            <div class="slider-label">
                <span style="color:var(--roll-color);">Roll (Bank Angle)</span>
                <span id="slider-roll-val">0°</span>
            </div>
            <input type="range" id="slider-roll" min="-180" max="180" value="0" oninput="onSimSliderChange()">
        </div>
        <div class="slider-row">
            <div class="slider-label">
                <span style="color:var(--yaw-color);">Yaw (Heading)</span>
                <span id="slider-yaw-val">0°</span>
            </div>
            <input type="range" id="slider-yaw" min="0" max="360" value="0" oninput="onSimSliderChange()">
        </div>
        <div class="btn-grid" style="margin-top: 4px;">
            <button class="btn" onclick="setSimPreset(0, 0, 0)">Level</button>
            <button class="btn" onclick="setSimPreset(45, 0, 0)">Pitch +45°</button>
            <button class="btn" onclick="setSimPreset(0, 45, 0)">Roll +45°</button>
            <button class="btn" onclick="setSimPreset(-90, 0, 0)">Nose Down</button>
            <button class="btn" onclick="setSimPreset(0, 180, 0)">Inverted</button>
            <button class="btn" id="btn-test-wave" onclick="toggleTestWave()">Sine Wave</button>
        </div>
    </div>

    <!-- Orientation Offsets & Inversions -->
    <div class="control-group">
        <div class="section-title">Calibration & Axes</div>
        <div class="btn-grid-2">
            <button class="btn btn-action" onclick="zeroAttitude()">Zero Tare</button>
            <button class="btn" onclick="resetAttitude()">Reset Offset</button>
        </div>
        <div class="slider-row" style="margin-top: 6px;">
            <div class="slider-label">
                <span>Tracking Responsiveness</span>
                <span id="lerp-val">Fast (80%)</span>
            </div>
            <input type="range" id="lerp-slider" min="10" max="100" value="80" oninput="onLerpSliderChange(this.value)">
        </div>
        <div class="check-row">
            <span>Upright Orientation (180° Flip)</span>
            <input type="checkbox" id="chk-flip-upright" onchange="toggleUpright(this.checked)">
        </div>
        <div class="check-row">
            <span>Invert Pitch</span>
            <input type="checkbox" id="chk-inv-pitch" onchange="attitude.invertPitch = this.checked">
        </div>
        <div class="check-row">
            <span>Invert Roll</span>
            <input type="checkbox" id="chk-inv-roll" onchange="attitude.invertRoll = this.checked">
        </div>
        <div class="check-row">
            <span>Invert Yaw</span>
            <input type="checkbox" id="chk-inv-yaw" onchange="attitude.invertYaw = this.checked">
        </div>
        <div class="check-row">
            <span>Show Body Axes (XYZ)</span>
            <input type="checkbox" id="chk-show-axes" checked onchange="toggleBodyAxes(this.checked)">
        </div>
        <div class="check-row">
            <span>Show World Reference Grid</span>
            <input type="checkbox" id="chk-show-grid" checked onchange="toggleWorldGrid(this.checked)">
        </div>
    </div>

    <!-- Camera Presets -->
    <div class="control-group">
        <div class="section-title">Camera Views</div>
        <div class="btn-grid">
            <button class="btn" onclick="setView('iso')">Isometric</button>
            <button class="btn" onclick="setView('chase')">Follow</button>
            <button class="btn" onclick="setView('front')">Front</button>
            <button class="btn" onclick="setView('side')">Side</button>
            <button class="btn" onclick="setView('top')">Top</button>
            <button class="btn" onclick="setView('bottom')">Bottom</button>
        </div>
    </div>

    <!-- CAD Visualization Features -->
    <div class="control-group">
        <div class="section-title">CAD Visual Controls</div>
        <div class="slider-row">
            <div class="slider-label">
                <span>Outer Hull Opacity</span>
                <span id="opacity-val">50%</span>
            </div>
            <input type="range" id="opacity-slider" min="0" max="100" value="50">
        </div>
        <div class="slider-row">
            <div class="slider-label">
                <span>Exploded View</span>
                <span id="explode-val">0%</span>
            </div>
            <input type="range" id="explode-slider" min="0" max="100" value="0">
        </div>
    </div>
</div>

<!-- Artificial Horizon Gauge -->
<div id="horizon-overlay">
    <canvas id="horizon-canvas" width="140" height="140"></canvas>
</div>

<!-- Bottom Telemetry Bar -->
<div id="telemetry-stats">
    <span>ALT: <b id="stat-alt">0.0 m</b></span>
    <span>PRES: <b id="stat-pres">1013.2 hPa</b></span>
    <span>TEMP: <b id="stat-temp">25.0 °C</b></span>
    <span>BATT: <b id="stat-volt">7.40 V</b></span>
    <span>PKTS: <b id="stat-pkts">0</b></span>
</div>

<div id="canvas-container"></div>

<script>
    // State
    const attitude = {
        rawPitch: 0.0,
        rawRoll: 0.0,
        rawYaw: 0.0,
        offsetPitch: 0.0,
        offsetRoll: 0.0,
        offsetYaw: 0.0,
        targetPitch: 0.0,
        targetRoll: 0.0,
        targetYaw: 0.0,
        pitch: 0.0,
        roll: 0.0,
        yaw: 0.0,
        invertPitch: false,
        invertRoll: false,
        invertYaw: false,
        flip180: false,
        smoothLerp: 0.80
    };

    let telemetryMode = 'live'; // 'live' or 'sim'
    let testWaveActive = false;
    let waveTime = 0.0;

    let scene, camera, renderer, controls;
    let cansatPivotGroup = new THREE.Group(); // Centered at (0, 0, 0)
    let cansatMeshGroup = new THREE.Group();  // Offset by (0, 0, -197) to match CAD centroid
    let bodyAxesHelper;
    let worldGrid, horizonRing;
    let partsList = [];
    let initialPositions = new Map();
    let isConnected = false;
    let ws = null;
    let httpPollingInterval = null;

    // Vibrant aerospace color coding matching CAD assembly modules
    const moduleMeta = {
        'Outer_Body': { cat: '01 Outer Structure', color: '#90a4ae', explodeDir: [0, 0, 0] },
        'Bay_Bottom_Disc': { cat: '05 Electronics Bay', color: '#455a64', explodeDir: [0, 0, -1.2] },
        'Bay_Cage': { cat: '05 Electronics Bay', color: '#0288d1', explodeDir: [0, 0, -0.6] },
        'Bay_Battery_Rack': { cat: '05 Electronics Bay', color: '#f59e0b', explodeDir: [0.8, 0, -0.4] },
        'Bay_Hatch_Door': { cat: '05 Electronics Bay', color: '#374151', explodeDir: [0, 0, -1.8] },
        'Bay_Camera_Holder': { cat: '05 Electronics Bay', color: '#10b981', explodeDir: [0, -0.8, -0.8] },
        'Bay_Pitot_Mount': { cat: '05 Electronics Bay', color: '#8b5cf6', explodeDir: [0, 0.8, -0.8] },
        'Turbine_Shaft': { cat: '03 Turbine Generator', color: '#e2e8f0', explodeDir: [0, 0, 0.4] },
        'Turbine_Spacer_Lower': { cat: '03 Turbine Generator', color: '#94a3b8', explodeDir: [0, 0, 0.3] },
        'Turbine_Rotor_1': { cat: '03 Turbine Generator', color: '#ef4444', explodeDir: [0, 0, 0.5] },
        'Turbine_Spacer_Mid': { cat: '03 Turbine Generator', color: '#94a3b8', explodeDir: [0, 0, 0.6] },
        'Turbine_Rotor_2': { cat: '03 Turbine Generator', color: '#f97316', explodeDir: [0, 0, 0.7] },
        'Turbine_Spacer_Upper': { cat: '03 Turbine Generator', color: '#94a3b8', explodeDir: [0, 0, 0.8] },
        'Turbine_Shaft_Bushing': { cat: '03 Turbine Generator', color: '#94a3b8', explodeDir: [0, 0, 0.9] },
        'Drone_Frame': { cat: '02 Drone Deployment', color: '#0284c7', explodeDir: [0, 0, 1.1] },
        'Drone_Rod': { cat: '02 Drone Deployment', color: '#cbd5e1', explodeDir: [0, 0, 1.4] },
        'Drone_Latch_Plate': { cat: '02 Drone Deployment', color: '#eab308', explodeDir: [0, 0, 1.6] },
        'Drone_Top_Plate': { cat: '02 Drone Deployment', color: '#475569', explodeDir: [0, 0, 1.8] },
        'Drone_Arm_PosX': { cat: '02 Drone Deployment', color: '#06b6d4', explodeDir: [1.2, 0, 1.3] },
        'Drone_Arm_PosY': { cat: '02 Drone Deployment', color: '#06b6d4', explodeDir: [0, 1.2, 1.3] },
        'Drone_Arm_NegX': { cat: '02 Drone Deployment', color: '#06b6d4', explodeDir: [-1.2, 0, 1.3] },
        'Drone_Arm_NegY': { cat: '02 Drone Deployment', color: '#06b6d4', explodeDir: [0, -1.2, 1.3] },
        'Drone_Pin_PosX': { cat: '02 Drone Deployment', color: '#f1f5f9', explodeDir: [1.5, 0, 1.8] },
        'Drone_Pin_PosY': { cat: '02 Drone Deployment', color: '#f1f5f9', explodeDir: [0, 1.5, 1.8] },
        'Drone_Pin_NegX': { cat: '02 Drone Deployment', color: '#f1f5f9', explodeDir: [-1.5, 0, 1.8] },
        'Drone_Pin_NegY': { cat: '02 Drone Deployment', color: '#f1f5f9', explodeDir: [0, -1.5, 1.8] },
        'Parachute_Bay': { cat: '04 Parachute System', color: '#ea580c', explodeDir: [0, 0, 2.2] },
        'Parachute_Lid': { cat: '04 Parachute System', color: '#dc2626', explodeDir: [0, 0, 2.6] }
    };

    function init() {
        const container = document.getElementById('canvas-container');

        scene = new THREE.Scene();

        // Standard aerospace coordinates: Z points UP towards sky
        camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 4000);
        camera.position.set(380, -320, 220);
        camera.up.set(0, 0, 1);

        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.3;
        container.appendChild(renderer.domElement);

        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.target.set(0, 0, 0); // Focus at center of mass

        // Studio Lighting
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x1e293b, 1.3);
        scene.add(hemiLight);

        const ambLight = new THREE.AmbientLight(0xffffff, 0.7);
        scene.add(ambLight);

        const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
        keyLight.position.set(400, 300, 500);
        scene.add(keyLight);

        const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.8);
        fillLight.position.set(-400, -300, 300);
        scene.add(fillLight);

        const rimLight = new THREE.DirectionalLight(0xffedd5, 0.6);
        rimLight.position.set(0, 400, -200);
        scene.add(rimLight);

        // Inertial Ground Reference Plane / Grid
        worldGrid = new THREE.GridHelper(700, 35, 0x0284c7, 0x1e293b);
        worldGrid.rotation.x = Math.PI / 2;
        worldGrid.position.z = -220; // Just below bottom of vehicle
        scene.add(worldGrid);

        // Horizon Disc Ring (subtle aerospace attitude reference)
        const ringGeo = new THREE.RingGeometry(240, 244, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x0284c7, side: THREE.DoubleSide, transparent: true, opacity: 0.3 });
        horizonRing = new THREE.Mesh(ringGeo, ringMat);
        horizonRing.position.z = 0;
        scene.add(horizonRing);

        // Attitude Pivot Groups:
        // cansatPivotGroup: Origin is at (0, 0, 0), rotated by live IMU angles
        // cansatMeshGroup: Contains centered CAD model
        cansatPivotGroup.rotation.order = 'ZXY';
        cansatMeshGroup.rotation.set(0, 0, 0);
        cansatPivotGroup.add(cansatMeshGroup);
        scene.add(cansatPivotGroup);

        // Body-Fixed Reference Axes attached to CanSat (Red = X, Green = Y, Blue = Z)
        bodyAxesHelper = new THREE.AxesHelper(220);
        cansatPivotGroup.add(bodyAxesHelper);

        loadCanSatModel();
        setupUIEvents();
        initWebSocket();
        initArtificialHorizon();

        // Ensure clean attitude offsets on initialization
        try {
            localStorage.removeItem('cansat_offset_pitch');
            localStorage.removeItem('cansat_offset_roll');
            localStorage.removeItem('cansat_offset_yaw');
        } catch(e) {}

        // Continuous HTTP polling backup timer (runs every 300ms if ws is down)
        httpPollingInterval = setInterval(pollTelemetryFallback, 300);

        window.addEventListener('resize', onWindowResize, false);
        animate();
    }

    function loadCanSatModel() {
        const loader = new THREE.GLTFLoader();

        // Load relative binary GLB asset
        loader.load('cansat_assembly.glb', function(gltf) {
            onModelLoaded(gltf.scene);
        }, undefined, function(err) {
            console.warn("Could not load relative GLB asset; attempting fallback...", err);
            fetch('/cansat_assembly.glb')
                .then(res => res.blob())
                .then(blob => {
                    const url = URL.createObjectURL(blob);
                    loader.load(url, function(gltf) {
                        onModelLoaded(gltf.scene);
                    });
                })
                .catch(e => {
                    document.getElementById('loading').innerHTML = '<h2 style=\"color:#ef4444;\">Failed to load cansat_assembly.glb</h2><p style=\"color:#94a3b8;\">Make sure the 3D server is running or file is placed in tools/web/</p>';
                });
        });
    }

    function onModelLoaded(modelScene) {
        document.getElementById('loading').style.opacity = '0';
        setTimeout(() => document.getElementById('loading').style.display = 'none', 400);

        modelScene.traverse(function(child) {
            if (child.isMesh) {
                if (child.geometry.attributes.color) {
                    child.geometry.deleteAttribute('color');
                }
                child.geometry.computeVertexNormals();

                const name = child.name || 'Part';
                let metaKey = Object.keys(moduleMeta).find(k => name.includes(k)) || name;
                let meta = moduleMeta[metaKey] || { cat: 'Other', color: '#38bdf8', explodeDir: [0,0,0] };

                const mat = new THREE.MeshPhongMaterial({
                    color: new THREE.Color(meta.color),
                    specular: new THREE.Color(0x334155),
                    shininess: 45,
                    side: THREE.DoubleSide
                });

                if (name.includes('Outer_Body')) {
                    mat.transparent = true;
                    mat.opacity = 0.50;
                    child.isOuterBody = true;
                }

                child.material = mat;
                partsList.push({ mesh: child, name: name, meta: meta });
                initialPositions.set(child, child.position.clone());
            }
        });

        modelScene.position.set(0, 0, -197); // Align centroid at origin (0, 0, 0)
        cansatMeshGroup.add(modelScene);
    }

    // WebSocket Telemetry Connection
    function initWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = protocol + '//' + (window.location.hostname || '127.0.0.1') + ':8765';

        try {
            ws = new WebSocket(wsUrl);
            ws.onopen = function() {
                isConnected = true;
                document.getElementById('status-dot').className = 'status-dot connected';
                document.getElementById('status-text').innerText = 'LIVE TELEMETRY';
            };
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (telemetryMode === 'live') {
                        applyTelemetry(data);
                    }
                } catch (e) {}
            };
            ws.onclose = function() {
                isConnected = false;
                document.getElementById('status-dot').className = 'status-dot';
                document.getElementById('status-text').innerText = 'HTTP TELEMETRY';
                setTimeout(initWebSocket, 2000);
            };
            ws.onerror = function() {
                ws.close();
            };
        } catch (e) {}
    }

    let isPolling = false;
    function pollTelemetryFallback() {
        if (isConnected || isPolling) return; // If websocket is alive or fetch pending, skip
        isPolling = true;
        fetch('/telemetry')
            .then(res => res.json())
            .then(data => {
                isPolling = false;
                if (telemetryMode === 'live') {
                    applyTelemetry(data);
                }
                document.getElementById('status-dot').className = 'status-dot connected';
                document.getElementById('status-text').innerText = 'LIVE HTTP';
            })
            .catch(() => {
                isPolling = false;
            });
    }

    // Apply Telemetry Packet to Attitude Targets
    function applyTelemetry(data) {
        if (data.pitch !== undefined && data.pitch !== null) {
            const val = Number(data.pitch);
            if (!isNaN(val)) attitude.rawPitch = val;
        }
        if (data.roll !== undefined && data.roll !== null) {
            const val = Number(data.roll);
            if (!isNaN(val)) attitude.rawRoll = val;
        }
        if (data.yaw !== undefined && data.yaw !== null) {
            const val = Number(data.yaw);
            if (!isNaN(val)) attitude.rawYaw = val;
        }

        if (telemetryMode === 'live') {
            let dp = ((attitude.rawPitch - attitude.offsetPitch + 540) % 360) - 180;
            attitude.targetPitch = dp;

            let dr = ((attitude.rawRoll - attitude.offsetRoll + 540) % 360) - 180;
            attitude.targetRoll = dr;

            let dy = ((attitude.rawYaw - attitude.offsetYaw) % 360 + 360) % 360;
            attitude.targetYaw = dy;
        }

        // Update stats HUD
        if (data.alt !== undefined) document.getElementById('stat-alt').innerText = Number(data.alt).toFixed(1) + ' m';
        if (data.pres !== undefined) document.getElementById('stat-pres').innerText = (Number(data.pres) / 100.0).toFixed(1) + ' hPa';
        if (data.temp !== undefined) document.getElementById('stat-temp').innerText = Number(data.temp).toFixed(1) + ' °C';
        if (data.volt !== undefined) document.getElementById('stat-volt').innerText = Number(data.volt).toFixed(2) + ' V';
        if (data.pcount !== undefined) document.getElementById('stat-pkts').innerText = data.pcount;

        if (data.state_name) {
            document.getElementById('flight-state-badge').innerText = data.state_name + ' (' + (data.state || 0) + ')';
        }
    }

    // Tare / Zero Attitude (Session Offset Calibration)
    function zeroAttitude() {
        attitude.offsetPitch = attitude.rawPitch;
        attitude.offsetRoll = attitude.rawRoll;
        attitude.offsetYaw = attitude.rawYaw;

        attitude.targetPitch = 0.0;
        attitude.targetRoll = 0.0;
        attitude.targetYaw = 0.0;

        attitude.pitch = 0.0;
        attitude.roll = 0.0;
        attitude.yaw = 0.0;
    }

    function resetAttitude() {
        attitude.offsetPitch = 0.0;
        attitude.offsetRoll = 0.0;
        attitude.offsetYaw = 0.0;

        let dp = ((attitude.rawPitch + 540) % 360) - 180;
        attitude.targetPitch = dp;

        let dr = ((attitude.rawRoll + 540) % 360) - 180;
        attitude.targetRoll = dr;

        let dy = ((attitude.rawYaw) % 360 + 360) % 360;
        attitude.targetYaw = dy;
    }

    function toggleUpright(isUpright) {
        attitude.flip180 = isUpright;
    }

    function onLerpSliderChange(val) {
        const factor = parseFloat(val) / 100.0;
        attitude.smoothLerp = factor;
        const txt = factor >= 0.95 ? 'Instant (100%)' : (factor >= 0.6 ? `Fast (${val}%)` : `Smooth (${val}%)`);
        document.getElementById('lerp-val').innerText = txt;
    }

    function toggleBodyAxes(visible) {
        if (bodyAxesHelper) bodyAxesHelper.visible = visible;
    }

    function toggleWorldGrid(visible) {
        if (worldGrid) worldGrid.visible = visible;
        if (horizonRing) horizonRing.visible = visible;
    }

    // Telemetry Modes & Controls
    function setMode(mode) {
        telemetryMode = mode;
        document.getElementById('btn-mode-live').classList.toggle('active', mode === 'live');
        document.getElementById('btn-mode-sim').classList.toggle('active', mode === 'sim');
        if (mode === 'sim') {
            document.getElementById('status-dot').className = 'status-dot connected';
            document.getElementById('status-text').innerText = 'MANUAL SIM';
        } else {
            document.getElementById('status-text').innerText = isConnected ? 'LIVE TELEMETRY' : 'LIVE HTTP';
            let dp = ((attitude.rawPitch - attitude.offsetPitch + 540) % 360) - 180;
            attitude.targetPitch = dp;
            let dr = ((attitude.rawRoll - attitude.offsetRoll + 540) % 360) - 180;
            attitude.targetRoll = dr;
            let dy = ((attitude.rawYaw - attitude.offsetYaw) % 360 + 360) % 360;
            attitude.targetYaw = dy;
        }
    }

    function onSimSliderChange() {
        if (telemetryMode !== 'sim') setMode('sim');
        attitude.targetPitch = parseFloat(document.getElementById('slider-pitch').value);
        attitude.targetRoll = parseFloat(document.getElementById('slider-roll').value);
        attitude.targetYaw = parseFloat(document.getElementById('slider-yaw').value);

        document.getElementById('slider-pitch-val').innerText = attitude.targetPitch.toFixed(0) + '°';
        document.getElementById('slider-roll-val').innerText = attitude.targetRoll.toFixed(0) + '°';
        document.getElementById('slider-yaw-val').innerText = attitude.targetYaw.toFixed(0) + '°';
    }

    function setSimPreset(p, r, y) {
        if (telemetryMode !== 'sim') setMode('sim');
        testWaveActive = false;
        document.getElementById('btn-test-wave').classList.remove('active');

        document.getElementById('slider-pitch').value = p;
        document.getElementById('slider-roll').value = r;
        document.getElementById('slider-yaw').value = y;
        onSimSliderChange();
    }

    function toggleTestWave() {
        testWaveActive = !testWaveActive;
        document.getElementById('btn-test-wave').classList.toggle('active', testWaveActive);
        if (testWaveActive && telemetryMode !== 'sim') setMode('sim');
    }

    function setupUIEvents() {
        // Exploded view slider
        document.getElementById('explode-slider').addEventListener('input', function(e) {
            const factor = parseFloat(e.target.value) * 1.5;
            document.getElementById('explode-val').innerText = e.target.value + '%';
            partsList.forEach(({ mesh, meta }) => {
                const init = initialPositions.get(mesh);
                const dir = meta.explodeDir || [0,0,0];
                mesh.position.set(
                    init.x + dir[0] * factor,
                    init.y + dir[1] * factor,
                    init.z + dir[2] * factor
                );
            });
        });

        // Opacity slider
        document.getElementById('opacity-slider').addEventListener('input', function(e) {
            const val = parseFloat(e.target.value) / 100.0;
            document.getElementById('opacity-val').innerText = e.target.value + '%';
            partsList.forEach(({ mesh }) => {
                if (mesh.isOuterBody) {
                    mesh.material.opacity = val;
                    mesh.material.transparent = val < 0.99;
                }
            });
        });
    }

    function setView(view) {
        const dist = 500;
        if (view === 'iso') {
            camera.position.set(dist * 0.7, -dist * 0.7, 220);
            controls.target.set(0, 0, 0);
        } else if (view === 'front') {
            camera.position.set(0, -dist, 0);
            controls.target.set(0, 0, 0);
        } else if (view === 'side') {
            camera.position.set(dist, 0, 0);
            controls.target.set(0, 0, 0);
        } else if (view === 'top') {
            camera.position.set(0, 0, dist);
            controls.target.set(0, 0, 0);
        } else if (view === 'bottom') {
            camera.position.set(0, 0, -dist);
            controls.target.set(0, 0, 0);
        } else if (view === 'chase') {
            camera.position.set(0, -320, 180);
            controls.target.set(0, 0, 0);
        }
        controls.update();
    }

    // 2D Artificial Horizon Canvas Rendering
    function initArtificialHorizon() {
        drawHorizon(0, 0);
    }

    function drawHorizon(pitchDeg, rollDeg) {
        const canvas = document.getElementById('horizon-canvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const w = canvas.width;
        const h = canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        const radius = w / 2 - 2;

        ctx.clearRect(0, 0, w, h);

        ctx.save();
        // Clip to circular gauge
        ctx.beginPath();
        ctx.arc(cx, cy, radius, 0, Math.PI * 2);
        ctx.clip();

        // Rotate for Roll
        ctx.translate(cx, cy);
        ctx.rotate(-rollDeg * Math.PI / 180);

        // Translate for Pitch
        const pitchOffset = (pitchDeg) * 1.3;
        ctx.translate(0, pitchOffset);

        // Sky (Blue)
        ctx.fillStyle = '#0284c7';
        ctx.fillRect(-w, -h * 2, w * 2, h * 2);

        // Ground (Brown)
        ctx.fillStyle = '#78350f';
        ctx.fillRect(-w, 0, w * 2, h * 2);

        // Horizon Line
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-w, 0);
        ctx.lineTo(w, 0);
        ctx.stroke();

        // Pitch ladder lines
        ctx.font = '8px Consolas';
        ctx.fillStyle = '#ffffff';
        ctx.textAlign = 'center';
        for (let p = -60; p <= 60; p += 10) {
            if (p === 0) continue;
            const y = -p * 1.3;
            const len = (p % 20 === 0) ? 28 : 16;
            ctx.beginPath();
            ctx.moveTo(-len / 2, y);
            ctx.lineTo(len / 2, y);
            ctx.stroke();
            if (p % 20 === 0) {
                ctx.fillText(Math.abs(p).toString(), len / 2 + 10, y + 3);
                ctx.fillText(Math.abs(p).toString(), -len / 2 - 10, y + 3);
            }
        }

        ctx.restore();

        // Fixed Crosshair / Aircraft Symbol
        ctx.strokeStyle = '#facc15';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        ctx.stroke();
        ctx.moveTo(cx - 30, cy);
        ctx.lineTo(cx - 10, cy);
        ctx.moveTo(cx + 10, cy);
        ctx.lineTo(cx + 30, cy);
        ctx.stroke();
    }

    function onWindowResize() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    }

    // Main Animation / Render Loop
    function animate() {
        requestAnimationFrame(animate);

        // Automated sine wave test simulation
        if (testWaveActive) {
            waveTime += 0.03;
            attitude.targetPitch = 30.0 * Math.sin(waveTime);
            attitude.targetRoll = 45.0 * Math.cos(waveTime * 0.7);
            attitude.targetYaw = (waveTime * 25.0) % 360.0;
        }

        // Smoothly interpolate towards target angles using shortest-path circular difference
        const lerpFactor = attitude.smoothLerp;
        
        let pitchDiff = ((attitude.targetPitch - attitude.pitch + 540) % 360) - 180;
        attitude.pitch += pitchDiff * lerpFactor;

        let rollDiff = ((attitude.targetRoll - attitude.roll + 540) % 360) - 180;
        attitude.roll += rollDiff * lerpFactor;
        
        // Circular lerp for Yaw (0 to 360)
        let yawDiff = ((attitude.targetYaw - attitude.yaw + 540) % 360) - 180;
        attitude.yaw += yawDiff * lerpFactor;
        attitude.yaw = ((attitude.yaw % 360) + 360) % 360;

        let effPitch = attitude.pitch;
        let effRoll = attitude.roll;
        let effYaw = attitude.yaw;

        if (attitude.invertPitch) effPitch = -effPitch;
        if (attitude.invertRoll) effRoll = -effRoll;
        if (attitude.invertYaw) effYaw = -effYaw;

        // Apply Euler rotations to CanSat Body Pivot:
        // ZXY order: Yaw (Z), Pitch (X), Roll (Y)
        const radX = THREE.MathUtils.degToRad(effPitch + (attitude.flip180 ? 180 : 0));
        const radY = THREE.MathUtils.degToRad(effRoll);
        const radZ = THREE.MathUtils.degToRad(effYaw);

        cansatPivotGroup.rotation.set(radX, radY, radZ, 'ZXY');

        // Update HUD Readouts
        document.getElementById('hud-pitch').innerText = (effPitch >= 0 ? '+' : '') + effPitch.toFixed(1) + '°';
        document.getElementById('hud-roll').innerText = (effRoll >= 0 ? '+' : '') + effRoll.toFixed(1) + '°';
        document.getElementById('hud-yaw').innerText = effYaw.toFixed(1) + '°';

        // Update Artificial Horizon
        drawHorizon(effPitch, effRoll);

        controls.update();
        renderer.render(scene, camera);
    }

    window.onload = init;
</script>
</body>
</html>
"""

full_html = offline_head_scripts + "\n</head>\n<body>\n" + app_body_and_script

# Write to tools/web/cansat_imu_viewer.html
dest_file = os.path.join(WORKSPACE_WEB_DIR, "cansat_imu_viewer.html")
with open(dest_file, "w", encoding="utf-8") as f:
    f.write(full_html)
print(f"Generated IMU Viewer: {dest_file} ({len(full_html)} bytes)")

# Also write copy to Downloads/Final_Body_Cansat/cansat_imu_viewer.html
cad_dest_file = os.path.join(CAD_DIR, "cansat_imu_viewer.html")
with open(cad_dest_file, "w", encoding="utf-8") as f:
    f.write(full_html)
print(f"Copied to CAD directory: {cad_dest_file}")
