import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js';

const COMMANDS = [
  {
    title: 'Flight',
    items: [
      { type: 'takeoff', label: 'Takeoff' },
      { type: 'land', label: 'Land' },
      { type: 'hover', label: 'Hover' },
      { type: 'emergency_stop', label: 'E-Stop' },
      { type: 'sleep', label: 'Sleep', param: 'duration', default: '1' }
    ]
  },
  {
    title: 'Move',
    items: [
      { type: 'move_forward', label: 'Forward', param: 'dist', default: '50' },
      { type: 'move_backward', label: 'Backward', param: 'dist', default: '50' },
      { type: 'move_left', label: 'Left', param: 'dist', default: '50' },
      { type: 'move_right', label: 'Right', param: 'dist', default: '50' },
      { type: 'move_up', label: 'Up', param: 'dist', default: '20' },
      { type: 'move_down', label: 'Down', param: 'dist', default: '20' }
    ]
  },
  {
    title: 'Turn',
    items: [
      { type: 'turn_left', label: 'Turn Left', param: 'degrees', default: '90' },
      { type: 'turn_right', label: 'Turn Right', param: 'degrees', default: '90' },
      { type: 'turn_degree', label: 'Turn Deg', param: 'degrees', default: '90' }
    ]
  },
  {
    title: 'Shapes',
    items: [
      { type: 'circle', label: 'Circle', param: 'radius', default: '30' },
      { type: 'square', label: 'Square', param: 'side', default: '40' },
      { type: 'triangle', label: 'Triangle', param: 'side', default: '40' },
      { type: 'spiral', label: 'Spiral', param: 'radius', default: '30' },
      { type: 'sway', label: 'Sway', param: 'count', default: '3' }
    ]
  },
  {
    title: 'Advanced',
    items: [
      { type: 'flip', label: 'Flip', param: 'dir', default: 'forward' },
      { type: 'keep_distance', label: 'Keep Dist', param: 'dist', default: '50' },
      { type: 'led', label: 'LED', param: 'color', default: 'red' },
      { type: 'buzzer', label: 'Buzzer' }
    ]
  }
];

let engineMod = null;
let plan = [];
let obstacles = [];
let simResult = null;
let isPlaying = false;
let playIndex = 0;
let playFrame = null;
let cameraMode = 'map';

const container = document.getElementById('viewport');
const canvas = document.getElementById('sceneCanvas');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0c10);
scene.fog = new THREE.Fog(0x0b0c10, 80, 350);

const camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 1000);
camera.position.set(80, 70, 80);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 10, 0);

const ambient = new THREE.AmbientLight(0xffffff, 0.35);
scene.add(ambient);

const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
dirLight.position.set(60, 120, 40);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
scene.add(dirLight);

const rimLight = new THREE.DirectionalLight(0x8b5cf6, 0.5);
rimLight.position.set(-60, 40, -60);
scene.add(rimLight);

const grid = new THREE.GridHelper(300, 60, 0x2dd4bf, 0x262a36);
scene.add(grid);

const groundGeo = new THREE.PlaneGeometry(300, 300);
const groundMat = new THREE.MeshStandardMaterial({ color: 0x0f1117, roughness: 0.9, metalness: 0.1 });
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.05;
ground.receiveShadow = true;
scene.add(ground);

function buildDrone() {
  const root = new THREE.Group();

  const bodyGeo = new THREE.BoxGeometry(6, 3, 10);
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.4, metalness: 0.5 });
  const body = new THREE.Mesh(bodyGeo, bodyMat);
  body.castShadow = true;
  root.add(body);

  const armMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.5, metalness: 0.4 });
  const armGeoLong = new THREE.BoxGeometry(1.5, 1, 22);
  const armLong = new THREE.Mesh(armGeoLong, armMat);
  root.add(armLong);

  const armGeoShort = new THREE.BoxGeometry(22, 1, 1.5);
  const armShort = new THREE.Mesh(armGeoShort, armMat);
  root.add(armShort);

  const rotorMat = new THREE.MeshStandardMaterial({ color: 0x2dd4bf, emissive: 0x0d9488, emissiveIntensity: 0.4 });
  const rotorGeo = new THREE.CylinderGeometry(4, 4, 0.2, 32);
  const positions = [[10, 0], [-10, 0], [0, 6], [0, -6]];
  for (const [x, z] of positions) {
    const rotor = new THREE.Mesh(rotorGeo, rotorMat);
    rotor.position.set(x, 1.8, z);
    root.add(rotor);
  }

  const noseGeo = new THREE.BoxGeometry(1.5, 1, 3);
  const noseMat = new THREE.MeshStandardMaterial({ color: 0x8b5cf6, emissive: 0x6d28d9, emissiveIntensity: 0.3 });
  const nose = new THREE.Mesh(noseGeo, noseMat);
  nose.position.set(0, 0.5, 6);
  root.add(nose);

  return root;
}

const drone = buildDrone();
scene.add(drone);

const trailMat = new THREE.LineBasicMaterial({ color: 0x2dd4bf, linewidth: 2 });
const trailGeo = new THREE.BufferGeometry();
const trail = new THREE.Line(trailGeo, trailMat);
trail.frustumCulled = false;
scene.add(trail);

const obstaclesGroup = new THREE.Group();
scene.add(obstaclesGroup);

function updateTrail(points) {
  if (!points || points.length === 0) {
    trail.geometry.setFromPoints([]);
    return;
  }
  const v3s = points.map(p => new THREE.Vector3(p.x, p.y, p.z));
  trail.geometry.setFromPoints(v3s);
}

function addObstacleMesh(obs) {
  const geo = new THREE.BoxGeometry(obs.sx, obs.sy, obs.sz);
  const mat = new THREE.MeshStandardMaterial({
    color: 0xef4444,
    transparent: true,
    opacity: 0.35,
    roughness: 0.2,
    metalness: 0.1
  });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(obs.x, obs.y + obs.sy / 2, obs.z);
  mesh.receiveShadow = true;
  mesh.userData.id = obs.id;
  obstaclesGroup.add(mesh);

  const edges = new THREE.EdgesGeometry(geo);
  const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0xef4444, transparent: true, opacity: 0.7 }));
  line.position.copy(mesh.position);
  line.userData.id = obs.id;
  obstaclesGroup.add(line);
}

function removeObstacleMesh(id) {
  const toRemove = [];
  obstaclesGroup.traverse(child => {
    if (child.userData.id === id) toRemove.push(child);
  });
  for (const child of toRemove) {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
      else child.material.dispose();
    }
    obstaclesGroup.remove(child);
  }
}

function loadObstaclesFromJson(data) {
  let arr = data;
  if (data && typeof data === 'object' && Array.isArray(data.obstacles)) {
    arr = data.obstacles;
  }
  if (!Array.isArray(arr)) return;
  for (const item of arr) {
    const obs = {
      id: makeId(),
      x: parseFloat(item.x ?? item.X ?? 0),
      y: parseFloat(item.y ?? item.Y ?? 0),
      z: parseFloat(item.z ?? item.Z ?? 0),
      sx: parseFloat(item.sx ?? item.w ?? item.width ?? 10),
      sy: parseFloat(item.sy ?? item.h ?? item.height ?? 10),
      sz: parseFloat(item.sz ?? item.d ?? item.depth ?? 10)
    };
    obstacles.push(obs);
    addObstacleMesh(obs);
  }
  renderObstacles();
}

function renderCommandGroups() {
  const host = document.getElementById('commandGroups');
  host.innerHTML = '';
  for (const group of COMMANDS) {
    const gEl = document.createElement('div');
    gEl.className = 'command-group';

    const title = document.createElement('div');
    title.className = 'group-title';
    title.textContent = group.title;
    gEl.appendChild(title);

    const grid = document.createElement('div');
    grid.className = 'command-grid';

    for (const item of group.items) {
      const btn = document.createElement('button');
      btn.className = 'cmd-btn' + (item.param ? ' param' : '');
      btn.textContent = item.label;
      btn.addEventListener('click', () => addCommand(item));
      grid.appendChild(btn);
    }

    gEl.appendChild(grid);
    host.appendChild(gEl);
  }
}

function addCommand(item) {
  const entry = { type: item.type };
  if (item.param) {
    const value = prompt(`Enter ${item.param} for ${item.label}:`, item.default);
    if (value === null) return;
    entry.params = { [item.param]: value };
  }
  plan.push(entry);
  renderPlan();
}

function removeCommand(index) {
  plan.splice(index, 1);
  renderPlan();
}

function renderPlan() {
  const list = document.getElementById('planList');
  list.innerHTML = '';
  if (plan.length === 0) {
    const empty = document.createElement('li');
    empty.className = 'empty-plan';
    empty.textContent = 'No commands yet';
    list.appendChild(empty);
    return;
  }
  plan.forEach((cmd, i) => {
    const li = document.createElement('li');
    li.className = 'plan-item';

    const left = document.createElement('div');
    const name = document.createElement('span');
    name.className = 'cmd-name';
    name.textContent = cmd.type;
    left.appendChild(name);

    if (cmd.params) {
      const params = document.createElement('span');
      params.className = 'cmd-params';
      params.textContent = Object.entries(cmd.params).map(([k, v]) => `${k}=${v}`).join(', ');
      left.appendChild(params);
    }

    const remove = document.createElement('button');
    remove.className = 'remove-btn';
    remove.textContent = 'x';
    remove.addEventListener('click', () => removeCommand(i));

    li.appendChild(left);
    li.appendChild(remove);
    list.appendChild(li);
  });
}

function renderObstacles() {
  const list = document.getElementById('obstacleList');
  list.innerHTML = '';
  obstacles.forEach((obs, i) => {
    const li = document.createElement('li');
    li.className = 'obstacle-item';
    li.textContent = `Box ${i + 1}: ${obs.x},${obs.y},${obs.z} ${obs.sx}x${obs.sy}x${obs.sz}`;

    const remove = document.createElement('button');
    remove.className = 'remove-btn';
    remove.textContent = 'x';
    remove.addEventListener('click', () => {
      removeObstacleMesh(obs.id);
      obstacles.splice(i, 1);
      renderObstacles();
    });

    li.appendChild(remove);
    list.appendChild(li);
  });
}

function setStatus(state) {
  const pill = document.getElementById('statusPill');
  pill.className = 'status-pill';
  pill.textContent = state;
  if (state === 'Flying') pill.classList.add('flying');
  if (state === 'Landed') pill.classList.add('landed');
  if (state === 'Collided') pill.classList.add('collided');
}

function makeId() {
  return Math.random().toString(36).slice(2) + Date.now().toString(36);
}

function callEngineSimulate() {
  if (!engineMod) return null;
  const payload = { commands: plan };
  if (obstacles.length > 0) payload.obstacles = obstacles;
  const input = JSON.stringify(payload);
  const result = engineMod.ccall('engineSimulateC', 'string', ['string'], [input]);
  try {
    return JSON.parse(result);
  } catch {
    return { ok: false, error: result };
  }
}

function callEngineGenerateCode() {
  if (!engineMod) return '';
  const input = JSON.stringify({ commands: plan });
  return engineMod.ccall('engineGenerateCodeC', 'string', ['string'], [input]);
}

function updateTelemetry(point) {
  if (!point) {
    document.getElementById('telAlt').textContent = '0.0 m';
    document.getElementById('telSpeed').textContent = '0.0 m/s';
    document.getElementById('telBattery').textContent = '100 %';
    document.getElementById('telHeading').textContent = '0 deg';
    return;
  }
  document.getElementById('telAlt').textContent = `${(point.y || 0).toFixed(1)} m`;
  document.getElementById('telSpeed').textContent = `${(point.speed || 0).toFixed(1)} m/s`;
  document.getElementById('telBattery').textContent = `${(point.batteryPercent ?? 100).toFixed(0)} %`;
  document.getElementById('telHeading').textContent = `${(point.heading || 0).toFixed(0)} deg`;
}

function drawSpeedChart(points) {
  const chart = document.getElementById('speedChart');
  const ctx = chart.getContext('2d');
  const w = chart.width;
  const h = chart.height;
  ctx.clearRect(0, 0, w, h);

  if (!points || points.length < 2) return;

  const speeds = points.map(p => p.speed || 0);
  const max = Math.max(...speeds, 0.1);

  ctx.strokeStyle = 'rgba(45, 212, 191, 0.2)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const y = h - (i / 4) * h;
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
  }
  ctx.stroke();

  ctx.strokeStyle = '#2dd4bf';
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((p, i) => {
    const x = (i / (points.length - 1)) * w;
    const y = h - (speeds[i] / max) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.fillStyle = 'rgba(45, 212, 191, 0.15)';
  ctx.lineTo(w, h);
  ctx.lineTo(0, h);
  ctx.closePath();
  ctx.fill();
}

function stopPlayback() {
  isPlaying = false;
  if (playFrame) cancelAnimationFrame(playFrame);
  playFrame = null;
  playIndex = 0;
  setStatus('Idle');
}

function startPlayback() {
  if (plan.length === 0) return;
  stopPlayback();

  simResult = callEngineSimulate();
  if (!simResult || !simResult.ok) {
    const err = simResult && simResult.error ? simResult.error : 'Simulation failed';
    document.getElementById('codeBlock').querySelector('code').textContent = err;
    setStatus('Collided');
    return;
  }

  const code = callEngineGenerateCode();
  document.getElementById('codeBlock').querySelector('code').textContent = code || 'No code generated';

  const points = simResult.positions || [];
  updateTrail(points);
  drawSpeedChart(points);

  if (points.length === 0) return;

  if (simResult.collisions && simResult.collisions.length > 0) {
    setStatus('Collided');
  } else {
    setStatus('Flying');
  }

  isPlaying = true;
  playIndex = 0;
  const fps = 20;
  const stepMs = 1000 / fps;
  let lastTime = performance.now();

  function tick(now) {
    if (!isPlaying) return;
    const dt = now - lastTime;
    if (dt >= stepMs) {
      lastTime = now - (dt % stepMs);
      const point = points[playIndex];
      drone.position.set(point.x, point.y, point.z);
      drone.rotation.y = -(point.heading || 0) * Math.PI / 180;
      drone.rotation.z = (point.roll || 0) * Math.PI / 180;
      drone.rotation.x = (point.pitch || 0) * Math.PI / 180;
      updateTelemetry(point);
      playIndex++;
      if (playIndex >= points.length) {
        const last = points[points.length - 1];
        if (last.y < 1) setStatus('Landed');
        else setStatus('Idle');
        isPlaying = false;
        return;
      }
    }
    playFrame = requestAnimationFrame(tick);
  }

  playFrame = requestAnimationFrame(tick);
}

function onWindowResize() {
  const w = container.clientWidth;
  const h = container.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

function animate() {
  requestAnimationFrame(animate);

  if (cameraMode === 'follow' && simResult && simResult.positions && simResult.positions.length > 0 && playIndex < simResult.positions.length) {
    const p = simResult.positions[playIndex];
    controls.target.set(p.x, p.y, p.z);
    camera.position.set(p.x + 40, p.y + 35, p.z + 40);
  } else if (cameraMode === 'map') {
    controls.target.lerp(new THREE.Vector3(0, 10, 0), 0.05);
  }

  controls.update();
  renderer.render(scene, camera);
}

function init() {
  renderCommandGroups();
  renderPlan();

  document.getElementById('runBtn').addEventListener('click', startPlayback);
  document.getElementById('stopBtn').addEventListener('click', stopPlayback);
  document.getElementById('clearPlan').addEventListener('click', () => {
    plan = [];
    renderPlan();
  });

  document.getElementById('camMap').addEventListener('click', () => {
    cameraMode = 'map';
    controls.enabled = true;
    document.getElementById('camMap').classList.add('active');
    document.getElementById('camFollow').classList.remove('active');
  });

  document.getElementById('camFollow').addEventListener('click', () => {
    cameraMode = 'follow';
    controls.enabled = false;
    document.getElementById('camFollow').classList.add('active');
    document.getElementById('camMap').classList.remove('active');
  });

  document.getElementById('addObstacle').addEventListener('click', () => {
    const x = parseFloat(document.getElementById('obX').value) || 0;
    const y = parseFloat(document.getElementById('obY').value) || 0;
    const z = parseFloat(document.getElementById('obZ').value) || 0;
    const sx = parseFloat(document.getElementById('obSx').value) || 10;
    const sy = parseFloat(document.getElementById('obSy').value) || 10;
    const sz = parseFloat(document.getElementById('obSz').value) || 10;
    const obs = { id: makeId(), x, y, z, sx, sy, sz };
    obstacles.push(obs);
    addObstacleMesh(obs);
    renderObstacles();
  });

  document.getElementById('obstacleFile').addEventListener('change', e => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      try {
        const data = JSON.parse(ev.target.result);
        loadObstaclesFromJson(data);
      } catch (err) {
        alert('Invalid obstacle JSON');
      }
      e.target.value = '';
    };
    reader.readAsText(file);
  });

  document.getElementById('copyCode').addEventListener('click', () => {
    const text = document.getElementById('codeBlock').textContent;
    navigator.clipboard.writeText(text).then(() => {
      const btn = document.getElementById('copyCode');
      const old = btn.textContent;
      btn.textContent = 'Copied';
      setTimeout(() => btn.textContent = old, 1200);
    });
  });

  window.addEventListener('resize', onWindowResize);

  Engine().then(mod => {
    engineMod = mod;
  });

  animate();
}

init();
