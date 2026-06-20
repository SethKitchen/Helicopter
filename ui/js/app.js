// HELISIM Studio — the browser design-studio client.
// Renders the validated geometry/data the Rust `helisim ui` step exports.
// Three.js is vendored (./vendor); this file is the whole application.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---- palette per component group -------------------------------------------
const GROUP_COLOR = {
  blade:        0x6fd9e6,
  mast:         0xc4ccd4,
  fuselage:     0x8FA0B0,
  boom:         0x9aa3b0,
  tail_fin:     0x6f7f8e,
  hub:          0xd8a657,
  swashplate:   0xc98b3a,
  tail_rotor:   0x6fd9e6,
  landing_gear: 0x70787f,
  motor:        0xb05a4a,
  battery:      0x4a8c5a,
  esc:          0x8a7fb0,
  avionics:     0x4f93c4,
  other:        0x7e8893,
};
// fuselage renders as a translucent shell so internals (motor/battery/avionics) show.
const TRANSLUCENT = new Set(['fuselage']);
const ACCENT = 0x38e1d4;

// Short descriptions for internal components that have no BuildPart spec.
const GROUP_DESC = {
  battery:  'Li-ion pack — sized by the mission energy + 10-year life closure. Its mass dominates the CG.',
  motor:    'Brushless DC motor — sized to the hover shaft power through the ESC (see <code>helisim build</code>).',
  esc:      'Electronic speed controller — drives the motor from the pack.',
  avionics: 'Flight controller + IMU running the validated SCAS / attitude / velocity-hold loops.',
  tail_fin: 'Tail stabilizing surfaces — sized visually from rotor radius and mounted at the boom tail for yaw/pitch damping.',
};

// ---- DOM -------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const canvas = $('view');
const veil = $('veil');
const partsList = $('parts-list');
const inspTitle = $('insp-title');
const inspBody = $('insp-body');
const hud = $('hud');
const status = $('status');
const designId = $('design-id');
const moduleVeil = $('module-veil');

// ---- three setup -----------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x070809, 0.00018);

const camera = new THREE.PerspectiveCamera(42, 2, 1, 100000);
camera.position.set(1400, 900, 1700);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.07;
controls.rotateSpeed = 0.85;
controls.minDistance = 80;
controls.maxDistance = 12000;

// lighting: cool hemisphere fill + warm key + cyan rim
scene.add(new THREE.HemisphereLight(0x9fb6c4, 0x0a0c0e, 0.85));
const key = new THREE.DirectionalLight(0xffffff, 2.1);
key.position.set(1.2, 2.0, 1.4);
scene.add(key);
const rim = new THREE.DirectionalLight(ACCENT, 0.9);
rim.position.set(-1.5, 0.6, -1.2);
scene.add(rim);

// subtle ground grid
const grid = new THREE.GridHelper(8000, 40, 0x1c2329, 0x12171b);
grid.material.transparent = true;
grid.material.opacity = 0.5;
scene.add(grid);

// pivot: Rust geometry is z-up; rotate into three's y-up
const pivot = new THREE.Group();
pivot.rotation.x = -Math.PI / 2;
scene.add(pivot);

// ---- state -----------------------------------------------------------------
const state = {
  manifest: null,
  parts: [],          // { id, name, group, mesh, baseColor, centroid, explodeDir }
  selected: null,
  tab: 'model',
  assemblyStep: -1,
  center: new THREE.Vector3(),
  radius: 1000,
};
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let cgMarker = null; // CG sphere, shown on the Balance tab
let feaOverlay = null;
let cfdOverlay = null;
let assemblyTools = null;
let flightActive = false;
const keys = new Set();
const flight = {
  pos: new THREE.Vector3(),
  vel: new THREE.Vector3(),
  yaw: 0,
  pitch: 0,
  roll: 0,
  throttle: 0.52,
};
const tutorial = {
  mode: 'build',
  buildStep: 0,
  started: performance.now(),
};

const ASSEMBLY_CLIPS = [
  {
    title: 'Powertrain tray + motor',
    parts: ['fuselage', 'canopy', 'tail_boom_fairing', 'powertrain_tray', 'motor', 'esc', 'avionics'],
    text: 'Remove the canopy/access cover first. The tray, motor, ESC and avionics slide in through that opening and bolt to internal bulkhead hardpoints.',
    motion: 'access-slide',
  },
  {
    title: 'Install the mast',
    parts: ['fuselage', 'canopy', 'powertrain_tray', 'motor', 'mast'],
    text: 'With the access cover off, the mast drops through the upper/lower bearing line and couples to the motor/gearbox below. It does not pass through a closed shell.',
    motion: 'vertical',
  },
  {
    title: 'Swashplate + servos',
    parts: ['mast', 'swashplate', 'fuselage', 'canopy'],
    text: 'The swashplate slides down over the mast before the head is installed. The bore is a sliding/gimballing fit, not a press fit.',
    motion: 'vertical',
  },
  {
    title: 'Hub + blade grips',
    parts: ['mast', 'hub', 'blade_grips', 'swashplate'],
    text: 'The hub seats on the mast top, then the blade grips align with the pitch-link circle.',
    motion: 'radial',
  },
  {
    title: 'Blade root prep + installation',
    parts: ['hub', 'blade_grips', 'blade_root_fittings', 'blade'],
    text: 'Close-up of one blade root: the printed pilot is reamed to final diameter; the steel bushing/doubler stack is bonded and cured; then the reinforced root bolts into the actual grip geometry. Repeat for each blade.',
    motion: 'ream-bond',
  },
  {
    title: 'Tail boom + tail rotor',
    parts: ['fuselage', 'tail_boom', 'tail_boom_fairing', 'tail_fin', 'horizontal_stab', 'tail_rotor'],
    text: 'The boom inserts into the aft fairing; the stabilizers and anti-torque rotor attach at the tail.',
    motion: 'aft',
  },
  {
    title: 'Battery + CG check',
    parts: ['fuselage', 'canopy', 'powertrain_tray', 'battery', 'avionics', 'motor'],
    text: 'The battery slides onto the tray through the removable canopy opening, then gets strapped down. The CG marker shows the mass-balance result.',
    motion: 'slide',
    cg: true,
  },
  {
    title: 'Control direction + travel',
    parts: ['mast', 'swashplate', 'hub', 'blade_grips', 'blade', 'blade_2', 'blade_3', 'blade_4', 'blade_5'],
    text: 'Cycle collective and cyclic: the swashplate moves first, blade grips follow, and all blades change pitch together.',
    motion: 'control',
  },
  {
    title: 'Tethered spin-up',
    parts: ['fuselage', 'landing_gear', 'mast', 'hub', 'blade_grips', 'blade', 'blade_2', 'blade_3', 'blade_4', 'blade_5', 'tail_boom', 'tail_rotor'],
    text: 'Spin the rotor under restraint and watch for vibration, tracking error, and tail-rotor response.',
    motion: 'spin',
  },
  {
    title: 'Power-loss response',
    parts: ['mast', 'hub', 'blade_grips', 'blade', 'blade_2', 'blade_3', 'blade_4', 'blade_5'],
    text: 'Practice the immediate collective-drop response. The clip shows the rotor decaying while pitch unloads.',
    motion: 'collective-drop',
  },
];

function partSpecByName(name) {
  const n = name.toLowerCase();
  return (state.manifest?.parts ?? []).find((p) => p.name.toLowerCase().includes(n));
}

function partSpecByGroup(group) {
  return (state.manifest?.parts ?? []).find((p) => p.group === group);
}

function buildLines(spec, indexes) {
  if (!spec?.steps?.length) return [];
  return indexes.map((i) => spec.steps[i]).filter(Boolean);
}

function buildPartClips() {
  const blade = partSpecByName('main-rotor blades') ?? partSpecByGroup('blade');
  const root = partSpecByName('blade root fittings');
  const hub = partSpecByName('rotor hub');
  const mast = partSpecByName('rotor mast');
  const swash = partSpecByName('swashplate');
  const fuselage = partSpecByGroup('fuselage');
  const gear = partSpecByGroup('landing_gear');
  const tray = partSpecByName('powertrain tray');
  const boom = partSpecByName('tail boom');
  const tailRotor = partSpecByName('tail rotor');
  return [
    {
      title: 'Blade geometry + SLS print',
      parts: ['blade'],
      lines: buildLines(blade, [0, 1]),
      text: 'Inspect the exported blade solid and airfoil template, then order the whole blade from the SLS service. No hand shaping.',
      motion: 'blade-print',
    },
    {
      title: 'Cut 6061 root doublers',
      parts: ['blade', 'blade_root_fittings'],
      lines: buildLines(root, [0]),
      text: 'Cut and deburr the aluminium doubler plates before any bonding. The plates become the metal centrifugal load path.',
      motion: 'cut-6061',
    },
    {
      title: 'Cut steel root bushings',
      parts: ['blade', 'blade_root_fittings'],
      lines: buildLines(root, [1]),
      text: 'Cut one steel bushing per blade to the actual root thickness so the retention bolt bears on steel.',
      motion: 'bushing-cut',
    },
    {
      title: 'Ream printed pilot holes',
      parts: ['blade', 'blade_root_fittings'],
      lines: buildLines(blade, [2]).concat(buildLines(root, [2])),
      text: 'The pilot is printed into the blade root. Ream it to final bolt size; do not drill the polymer from solid.',
      motion: 'ream-only',
    },
    {
      title: 'Bond bushings + doublers',
      parts: ['blade', 'blade_root_fittings'],
      lines: buildLines(blade, [3]).concat(buildLines(root, [3])),
      text: 'Scuff, degrease, wet with structural epoxy, clamp, and let the root reinforcement fully cure.',
      motion: 'bond-only',
    },
    {
      title: 'Ream grip jaws + install root',
      parts: ['blade', 'blade_root_fittings', 'hub', 'blade_grips'],
      lines: buildLines(root, [4]).concat(buildLines(hub, [2, 3, 4])),
      text: 'The reinforced root bolts through the actual blade grip. The joint must pivot freely after the nyloc nut is snug.',
      motion: 'ream-bond',
    },
    {
      title: 'Balance matched blade set',
      parts: ['blade', 'blade_2', 'blade_3', 'blade_4', 'blade_5'],
      lines: buildLines(blade, [4]),
      text: 'Use the magnetic balancer before any spin test. Match spanwise and chordwise balance across all five finished blades.',
      motion: 'balance-blades',
    },
    {
      title: 'Machine mast, hub, and swashplate',
      parts: ['mast', 'hub', 'blade_grips', 'swashplate'],
      lines: buildLines(mast, [0, 1, 2, 3]).concat(buildLines(hub, [0, 1]), buildLines(swash, [0, 1, 2, 3, 4, 5])),
      text: 'Machine the rotating hardware and verify sliding, tilting, bearing, pitch-link and runout checks before assembly.',
      motion: 'head-build',
    },
    {
      title: 'Build fuselage shell + hardpoints',
      parts: ['fuselage', 'canopy', 'tail_boom_fairing'],
      lines: buildLines(fuselage, [0, 1, 2, 3]),
      text: 'Make the smooth shell from the exported pod, keep the upper canopy removable, and add hardpoints before closing the structure.',
      motion: 'shell-build',
    },
    {
      title: 'Build skids and tray',
      parts: ['fuselage', 'landing_gear', 'powertrain_tray', 'motor', 'esc', 'battery', 'avionics'],
      lines: buildLines(gear, [0, 1, 2, 3]).concat(buildLines(tray, [0, 1, 2, 3])),
      text: 'Cut the skid tubes/struts and powertrain tray, then bolt them into the fuselage hardpoints with the CG under the shaft.',
      motion: 'bench-build',
    },
    {
      title: 'Build tail boom + tail rotor',
      parts: ['tail_boom', 'tail_boom_fairing', 'tail_fin', 'horizontal_stab', 'tail_rotor'],
      lines: buildLines(boom, [0, 1, 2, 3]).concat(buildLines(tailRotor, [0, 1, 2, 3])),
      text: 'Cut the boom, add the tail hardware, route wiring/control, then set anti-torque thrust direction.',
      motion: 'tail-build',
    },
  ];
}

function currentTutorialClips() {
  return tutorial.mode === 'build' ? buildPartClips() : ASSEMBLY_CLIPS;
}

function currentTutorialIndex() {
  return tutorial.mode === 'build' ? tutorial.buildStep : state.assemblyStep;
}

function setCurrentTutorialIndex(i) {
  const clips = currentTutorialClips();
  const next = Math.max(0, Math.min(clips.length - 1, Math.floor(i)));
  if (tutorial.mode === 'build') tutorial.buildStep = next;
  else state.assemblyStep = next;
}

function currentTutorialClip() {
  const clips = currentTutorialClips();
  return clips[currentTutorialIndex()] ?? clips[0];
}

function tutorialLine(s) {
  return String(s).replace(/^\s*\d+\.\s*/, '');
}

// ---- load ------------------------------------------------------------------
async function boot() {
  try {
    const [geo, man] = await Promise.all([
      fetch('data/geometry.json').then((r) => r.json()),
      fetch('data/manifest.json').then((r) => r.json()),
    ]);
    state.manifest = man;
    buildModel(geo);
    buildPartsList();
    renderOverview();
    renderHud();
    setDesignId();
    buildCgMarker();
    buildFeaOverlay();
    buildCfdOverlay();
    buildAssemblyTools();
    pivot.updateMatrixWorld(true); // so frameAll maps the local centre into world space
    frameAll(false);
    veil.classList.add('gone');
    setStatus(`${state.parts.length} components loaded`);
    applyUrlState(); // deep-link / deterministic state (used by the screenshot harness)
  } catch (e) {
    $('veil').innerHTML = `<div class="veil-text">could not load design data —<br/>run <code>helisim ui</code> from the repo root</div>`;
    console.error(e);
  }
}

// Weld coincident vertices (quantised to 0.05 mm) into an indexed geometry and
// recompute averaged normals → smooth shading for curved parts.
function smoothGeometry(positions) {
  const map = new Map(), verts = [], index = [];
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    const key = `${Math.round(x * 20)}_${Math.round(y * 20)}_${Math.round(z * 20)}`;
    let id = map.get(key);
    if (id === undefined) { id = verts.length / 3; map.set(key, id); verts.push(x, y, z); }
    index.push(id);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  g.setIndex(index);
  g.computeVertexNormals();
  return g;
}

// Restore a part to its resting appearance (translucency-aware).
function restoreBase(p) {
  const m = p.mesh.material, tr = p.mesh.userData.translucent;
  p.mesh.position.set(0, 0, 0);
  p.mesh.rotation.set(0, 0, 0);
  m.color.setHex(p.baseColor);
  m.metalness = tr ? 0.1 : 0.78;
  m.roughness = tr ? 0.18 : 0.36;
  m.emissive.setHex(0x000000); m.emissiveIntensity = 0;
  m.opacity = tr ? 0.16 : 1; m.transparent = tr; m.depthWrite = !tr;
  p.mesh.visible = true;
}

function buildModel(geo) {
  const box = new THREE.Box3();
  for (const p of geo.parts) {
    // Smooth (curved) parts: weld coincident vertices + recompute averaged
    // normals. Flat (boxes/skids): keep the per-facet normals as exported.
    const g = p.smooth ? smoothGeometry(p.positions) : (() => {
      const bg = new THREE.BufferGeometry();
      bg.setAttribute('position', new THREE.Float32BufferAttribute(p.positions, 3));
      bg.setAttribute('normal', new THREE.Float32BufferAttribute(p.normals, 3));
      return bg;
    })();

    const baseColor = GROUP_COLOR[p.group] ?? GROUP_COLOR.other;
    const translucent = TRANSLUCENT.has(p.group);
    const mat = new THREE.MeshStandardMaterial({
      color: baseColor,
      metalness: translucent ? 0.1 : 0.78,
      roughness: translucent ? 0.18 : 0.36,
      emissive: 0x000000,
      transparent: translucent,
      opacity: translucent ? 0.16 : 1,
      depthWrite: !translucent,
      side: translucent ? THREE.DoubleSide : THREE.FrontSide,
    });
    const mesh = new THREE.Mesh(g, mat);
    mesh.renderOrder = translucent ? 10 : 0; // draw the shell last
    mesh.userData.partId = p.id;
    mesh.userData.translucent = translucent;
    pivot.add(mesh);

    // Vertices are absolute in pivot-local space (no per-mesh offset), so the
    // geometry bounding box is the mesh box in that space — use it for both the
    // part centroid and the assembly union, keeping every measure in one frame.
    g.computeBoundingBox();
    const c = new THREE.Vector3();
    g.boundingBox.getCenter(c);
    box.union(g.boundingBox);

    state.parts.push({
      id: p.id, name: p.name, group: p.group, mesh,
      baseColor, centroid: c, explodeDir: new THREE.Vector3(),
    });
  }
  // assembly centroid + radius (in pivot/local space)
  const center = new THREE.Vector3();
  box.getCenter(center);
  const size = new THREE.Vector3();
  box.getSize(size);
  state.center.copy(center);
  state.radius = Math.max(size.x, size.y, size.z) * 0.5 || 1000;

  // explode direction = from assembly centroid to part centroid (radial)
  for (const part of state.parts) {
    const dir = part.centroid.clone().sub(center);
    if (dir.lengthSq() < 1e-6) dir.set(0, 0, 1);
    part.explodeDir.copy(dir.normalize());
  }
}

// ---- parts list ------------------------------------------------------------
function buildPartsList() {
  partsList.innerHTML = '';
  for (const part of state.parts) {
    const li = document.createElement('li');
    li.dataset.id = part.id;
    const hex = '#' + part.baseColor.toString(16).padStart(6, '0');
    li.innerHTML =
      `<span class="dot" style="color:${hex}"></span>` +
      `<span class="nm">${esc(part.name)}</span>` +
      `<span class="ct">${part.group}</span>`;
    li.onclick = () => selectPart(part.id, true);
    partsList.appendChild(li);
  }
}

// ---- selection + framing ---------------------------------------------------
function selectPart(id, fly) {
  state.selected = id;
  for (const part of state.parts) {
    restoreBase(part);
    const on = part.id === id;
    if (on) {
      part.mesh.material.emissive.setHex(ACCENT);
      part.mesh.material.emissiveIntensity = 0.5;
      part.mesh.material.opacity = 1;
      part.mesh.material.transparent = false;
      part.mesh.material.depthWrite = true;
    } else if (id) {
      part.mesh.material.opacity = part.mesh.userData.translucent ? 0.06 : 0.16;
      part.mesh.material.transparent = true;
    }
  }
  for (const li of partsList.children) li.classList.toggle('sel', li.dataset.id === id);
  const part = state.parts.find((p) => p.id === id);
  if (part) {
    renderPartInfo(part);
    if (fly) frameMesh(part.mesh);
    setStatus(`inspecting ${part.name}`);
  }
}

function clearSelection() {
  state.selected = null;
  for (const part of state.parts) restoreBase(part);
  for (const li of partsList.children) li.classList.remove('sel');
  renderOverview();
  setStatus('ready');
}

// camera tween
let tween = null;
function flyTo(targetPos, lookAt, ms = 650) {
  const startPos = camera.position.clone();
  const startTarget = controls.target.clone();
  const t0 = performance.now();
  tween = (now) => {
    const k = Math.min(1, (now - t0) / ms);
    const e = k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2; // easeInOutQuad
    camera.position.lerpVectors(startPos, targetPos, e);
    controls.target.lerpVectors(startTarget, lookAt, e);
    if (k >= 1) tween = null;
  };
}

function frameMesh(mesh) {
  const box = new THREE.Box3().setFromObject(mesh);
  const c = new THREE.Vector3(); box.getCenter(c);
  const s = new THREE.Vector3(); box.getSize(s);
  const r = Math.max(s.x, s.y, s.z, 1) * 0.5;
  const dist = (r / Math.tan((camera.fov * Math.PI) / 360)) * 1.6 + r;
  const dir = camera.position.clone().sub(controls.target).normalize();
  flyTo(c.clone().add(dir.multiplyScalar(dist)), c);
}

function frameAll(animate = true) {
  const worldCenter = state.center.clone().applyMatrix4(pivot.matrixWorld);
  const r = state.radius;
  const dist = (r / Math.tan((camera.fov * Math.PI) / 360)) * 1.15 + r;
  const dir = new THREE.Vector3(0.7, 0.42, 0.9).normalize();
  const pos = worldCenter.clone().add(dir.multiplyScalar(dist));
  if (animate) flyTo(pos, worldCenter);
  else { camera.position.copy(pos); controls.target.copy(worldCenter); }
}

// ---- exploded view ---------------------------------------------------------
let explodeK = 0, explodeTarget = 0;
function setExplode(on) { explodeTarget = on ? 1 : 0; }
function applyExplode() {
  if (state.tab === 'assembly') return;
  if (Math.abs(explodeK - explodeTarget) < 0.001) return;
  explodeK += (explodeTarget - explodeK) * 0.12;
  const amp = state.radius * 0.9;
  for (const part of state.parts) {
    part.mesh.position.copy(part.explodeDir).multiplyScalar(explodeK * amp);
  }
}

// ---- inspector content -----------------------------------------------------
function renderOverview() {
  inspTitle.textContent = 'Design overview';
  const d = state.manifest?.design ?? {};
  const fmt = (x, p = 2) => (x?.value ?? 0).toFixed(p);
  const rat = (state.manifest?.rationale ?? []).map((r) => `<li>${esc(r)}</li>`).join('');
  const nEval = d.candidates_evaluated?.value ?? 0;
  const nFeas = d.candidates_feasible?.value ?? 0;
  inspBody.innerHTML = `
    <span class="tag">recommended</span>
    ${nEval ? `<div style="margin:8px 0 4px;font-size:12px;color:var(--ink-faint)">best of <b style="color:var(--accent)">${nEval}</b> candidates searched · ${nFeas} feasible</div>` : ''}
    <h4>Configuration</h4>
    <div class="kv">
      <span class="k">Blades</span><span class="v">${fmt(d.blades, 0)}</span>
      <span class="k">Rotor radius</span><span class="v">${fmt(d.radius_m, 3)} m</span>
      <span class="k">Tip speed</span><span class="v">${fmt(d.tip_speed_mps, 0)} m/s</span>
      <span class="k">RPM</span><span class="v">${fmt(d.rpm, 0)}</span>
      <span class="k">Solidity</span><span class="v">${fmt(d.solidity, 3)}</span>
      <span class="k">Gross mass</span><span class="v">${fmt(d.gross_mass_kg, 2)} kg</span>
    </div>
    <h4>Performance</h4>
    <div class="kv">
      <span class="k">Figure of merit</span><span class="v">${fmt(d.figure_of_merit, 3)}</span>
      <span class="k">Hover power</span><span class="v">${fmt(d.hover_shaft_power_w, 0)} W</span>
      <span class="k">Hover endurance</span><span class="v">${fmt(d.hover_endurance_min, 1)} min</span>
      <span class="k">Flare margin</span><span class="v">${fmt(d.flare_margin, 2)}</span>
      <span class="k">OASPL</span><span class="v">${fmt(d.oaspl_db, 1)} dB</span>
    </div>
    ${rat ? `<h4>Why this design</h4><ul class="steps">${rat}</ul>` : ''}
    <h4>Inspect</h4>
    <div style="color:var(--ink-faint);font-size:12px">Click any component in the model or list to see its material, dimensions, build steps and structural margin.</div>
  `;
}

function specFor(group) {
  return (state.manifest?.parts ?? []).find((p) => p.group === group);
}
function marginFor(group) {
  const map = { blade: 'blade', mast: 'mast', boom: 'boom' };
  const key = map[group];
  return (state.manifest?.structure?.items ?? []).filter((it) =>
    it.part.toLowerCase().includes(key ?? '###'));
}

function renderPartInfo(part) {
  inspTitle.textContent = part.name;
  const spec = specFor(part.group);
  const margins = marginFor(part.group);
  let html = `<span class="tag">${part.group}</span>`;
  if (spec) {
    html += `<h4>${esc(spec.name)}</h4>
      <div class="kv">
        <span class="k">Material</span><span class="v">${esc(spec.material)}</span>
        <span class="k">Source</span><span class="v">${esc(spec.source)}</span>
      </div>`;
    if (spec.dims?.length) {
      html += `<h4>Dimensions</h4><div class="kv">` +
        spec.dims.map((dm) => `<span class="k">${esc(dm.label)}</span><span class="v">${dm.mm.toFixed(1)} mm</span>`).join('') +
        `</div>`;
    }
    if (margins.length) {
      html += `<h4>Structural margin</h4>`;
      for (const m of margins) {
        const cls = m.ok ? 'ms-ok' : 'ms-bad';
        const frac = Math.max(0, Math.min(1, m.actual_mpa / Math.max(m.allowable_mpa, 1e-6)));
        html += `<div class="kv"><span class="k">${esc(m.load)}</span>
          <span class="v ${cls}">MS ${m.ms >= 0 ? '+' : ''}${m.ms.toFixed(2)}</span></div>
          <div class="bar"><i style="width:${(frac * 100).toFixed(0)}%"></i></div>
          <div style="font-size:11px;color:var(--ink-faint)">${m.actual_mpa.toFixed(1)} / ${m.allowable_mpa.toFixed(1)} MPa</div>`;
      }
    }
    if (spec.steps?.length) {
      html += `<h4>Build steps</h4><ul class="steps">` +
        spec.steps.map((s) => `<li>${esc(s)}</li>`).join('') + `</ul>`;
    }
  } else {
    const desc = GROUP_DESC[part.group];
    html += desc
      ? `<div style="margin-top:12px;line-height:1.6">${desc}</div>
         <div style="margin-top:10px;font-size:11px;color:var(--ink-faint)">Placement is representative; mass &amp; location feed the CG / mass-properties model.</div>`
      : `<div style="margin-top:12px;color:var(--ink-faint)">No build spec mapped for this component.</div>`;
  }
  inspBody.innerHTML = html;
}

// ---- HUD -------------------------------------------------------------------
function renderHud() {
  const d = state.manifest?.design ?? {};
  const m = [
    ['Blades', (d.blades?.value ?? 0).toFixed(0), ''],
    ['Radius', (d.radius_m?.value ?? 0).toFixed(2), 'm'],
    ['Gross', (d.gross_mass_kg?.value ?? 0).toFixed(2), 'kg'],
    ['FM', (d.figure_of_merit?.value ?? 0).toFixed(3), ''],
    ['Endurance', (d.hover_endurance_min?.value ?? 0).toFixed(0), 'min'],
    ['OASPL', (d.oaspl_db?.value ?? 0).toFixed(0), 'dB'],
  ];
  hud.innerHTML = m.map(([l, v, u]) =>
    `<div class="metric"><div class="v">${v}<small>${u}</small></div><div class="l">${l}</div></div>`
  ).join('');
}

// ---- FEA tab: tint by stress + numbers -------------------------------------
function applyFeaView() {
  const fea = state.manifest?.fea ?? [];
  if (feaOverlay) feaOverlay.visible = true;
  const byGroup = {};
  for (const f of fea) {
    const g = f.name.toLowerCase().includes('boom') ? 'boom' : 'blade';
    byGroup[g] = f;
  }
  const av = state.manifest?.balance?.avionics_effect;
  // stress→color: green (low) → amber → red (near allowable). Use ratio to ~200 MPa.
  for (const part of state.parts) {
    const f = byGroup[part.group];
    if (f) {
      const ratio = Math.max(0, Math.min(1, f.fe_stress_mpa / 200));
      const col = stressColor(ratio);
      part.mesh.material.color.setHex(col);
      part.mesh.material.metalness = 0.25;
      part.mesh.material.roughness = 0.6;
    } else if (part.group === 'avionics') {
      part.mesh.material.color.setHex(0x4f93c4);
      part.mesh.material.emissive.setHex(0x4f93c4);
      part.mesh.material.emissiveIntensity = 0.22;
    } else {
      part.mesh.material.color.setHex(0x2a3138);
      part.mesh.material.opacity = 0.4;
      part.mesh.material.transparent = true;
    }
  }
  inspTitle.textContent = 'FEA · beam finite element';
  inspBody.innerHTML = `<span class="tag">deflection + stress</span>
    <div style="margin:10px 0;color:var(--ink-faint);font-size:12px">Euler–Bernoulli beam FE, cross-checked against closed-form theory. Blade shows centrifugally-stiffened (spun-up) tip deflection.</div>` +
    fea.map((f) => {
      const stiff = f.tip_deflection_stiffened_mm != null
        ? ` → <span style="color:var(--good)">${f.tip_deflection_stiffened_mm.toFixed(1)} mm spun-up</span>` : '';
      return `<h4>${esc(f.name)}</h4>
        <div class="kv">
          <span class="k">Tip deflection</span><span class="v">${f.tip_deflection_mm.toFixed(1)} mm${stiff}</span>
          <span class="k">FE stress</span><span class="v">${f.fe_stress_mpa.toFixed(1)} MPa</span>
          <span class="k">Closed-form</span><span class="v">${f.closed_form_stress_mpa.toFixed(1)} MPa</span>
          <span class="k">Routes</span><span class="v ${f.routes_agree ? 'ms-ok' : 'ms-bad'}">${f.routes_agree ? 'agree' : 'mismatch'}</span>
        </div>`;
    }).join('') +
    (av ? `<h4>Avionics mount load</h4>
      <div class="kv">
        <span class="k">6g local mount load</span><span class="v">${av.fea_mount_load_n.toFixed(2)} N</span>
        <span class="k">Iyy contribution</span><span class="v">${av.pitch_inertia_delta_kg_m2.toFixed(5)} kg·m²</span>
      </div>
      <div style="font-size:12px;color:var(--ink-faint)">The avionics mass is carried as a point load for mount sizing and contributes to the same pitch inertia used by stability.</div>` : '');
}
function stressColor(r) {
  // 0→green 0x5be3a0, .5→amber 0xffb56b, 1→red 0xff5d6c
  const lerp = (a, b, t) => Math.round(a + (b - a) * t);
  let c1, c2, t;
  if (r < 0.5) { c1 = [0x5b, 0xe3, 0xa0]; c2 = [0xff, 0xb5, 0x6b]; t = r / 0.5; }
  else { c1 = [0xff, 0xb5, 0x6b]; c2 = [0xff, 0x5d, 0x6c]; t = (r - 0.5) / 0.5; }
  return (lerp(c1[0], c2[0], t) << 16) | (lerp(c1[1], c2[1], t) << 8) | lerp(c1[2], c2[2], t);
}

// ---- Optimality tab: metric bars -------------------------------------------
function applyOptimalityView() {
  inspTitle.textContent = 'Optimality';
  const d = state.manifest?.design ?? {};
  const ranked = state.manifest?.optimality?.ranked ?? [];
  const points = ranked.map((p) => {
    const x = 16 + ((p.radius_m - 0.40) / 0.40) * 218;
    const y = 142 - Math.max(0, Math.min(1, (p.endurance_min - 10) / 30)) * 118;
    const fill = p.pareto ? 'var(--accent)' : 'rgba(255,255,255,.32)';
    const r = p.rank === 1 ? 6 : (p.pareto ? 4 : 3);
    return `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${r}" fill="${fill}" opacity="${p.rank === 1 ? 1 : .78}" />`;
  }).join('');
  // normalized "goodness" bars (higher = better; flare margin vs floor 1.0)
  const rows = [
    ['Figure of merit', d.figure_of_merit?.value ?? 0, 1.0, 'higher is better'],
    ['Flare margin', d.flare_margin?.value ?? 0, 3.0, 'safety floor = 1.0'],
    ['Endurance', d.hover_endurance_min?.value ?? 0, 40, 'minutes'],
    ['Quietness', 60 - (d.oaspl_db?.value ?? 60), 60, 'lower OASPL = better'],
  ];
  inspBody.innerHTML = `<span class="tag">priority vector</span>
    <div style="margin:10px 0;color:var(--ink-faint);font-size:12px">The recommender ranks the geometry grid by safety → airtime → efficiency → noise. This design is on the Pareto front.</div>
    <svg class="plot" viewBox="0 0 250 160" role="img" aria-label="candidate optimality plot">
      <line x1="16" y1="142" x2="238" y2="142"></line>
      <line x1="16" y1="142" x2="16" y2="18"></line>
      <text x="18" y="154">radius</text>
      <text x="18" y="16">endurance</text>
      ${points}
    </svg>
    ${ranked.length ? `<div class="kv"><span class="k">Ranked samples shown</span><span class="v">${ranked.length}</span><span class="k">Best score</span><span class="v">${ranked[0].score.toFixed(3)}</span></div>` : ''}` +
    rows.map(([l, v, max, note]) => {
      const frac = Math.max(0.02, Math.min(1, v / max));
      return `<h4>${l}</h4>
        <div class="bar"><i style="width:${(frac * 100).toFixed(0)}%"></i></div>
        <div class="kv"><span class="k">${note}</span><span class="v">${v.toFixed(2)}</span></div>`;
    }).join('') +
    `<div style="margin-top:14px;color:var(--ink-faint);font-size:11.5px">Grid search is intentionally denser than the 120-corner model default; continuous optimization remains available through <code>final-report</code>.</div>`;
}

// ---- Assembly tab: step through the sequence -------------------------------
function applyAssemblyView() {
  inspTitle.textContent = 'Build tutorial';
  const clips = currentTutorialClips();
  setCurrentTutorialIndex(currentTutorialIndex());
  const step = currentTutorialIndex();
  const active = currentTutorialClip();
  const isBuild = tutorial.mode === 'build';
  const labels = isBuild ? clips.map((c) => c.title) : (state.manifest?.assembly ?? []);
  const lines = isBuild ? (active.lines?.length ? active.lines : [active.text]) : labels;
  const tag = isBuild ? `${clips.length} build operations` : `${labels.length} assembly steps`;
  inspBody.innerHTML = `<span class="tag">${tag}</span>
    <div class="tutorial-buttons">
      <button class="ghost ${isBuild ? 'active' : ''}" id="tutorial-build">Build parts</button>
      <button class="ghost ${!isBuild ? 'active' : ''}" id="tutorial-assembly">Assemble aircraft</button>
    </div>
    <ul class="steps tutorial-steps" id="asm-steps">` +
    labels.map((s, i) => `<li data-i="${i}" class="${i === step ? 'active-step' : ''}">${esc(tutorialLine(s))}</li>`).join('') + `</ul>
    <div class="panel-foot" style="border:0;padding:10px 0 0">
      <button class="ghost" id="asm-prev">‹ Prev</button>
      <span id="asm-pos" style="color:var(--ink-dim);font-size:12px">step 0 / ${labels.length}</span>
      <button class="ghost" id="asm-next">Next ›</button>
    </div>
    <h4>${esc(active.title)}</h4>
    <div class="tutorial-def">${esc(active.text)}</div>
    <ul class="steps tutorial-steps">` +
    lines.map((s) => `<li>${esc(tutorialLine(s))}</li>`).join('') + `</ul>
    <div class="tutorial-stage-text" id="tutorial-stage"></div>`;
  const setStep = (i) => {
    setCurrentTutorialIndex(i);
    tutorial.started = performance.now();
    applyAssemblyView();
  };
  $('tutorial-build').onclick = () => {
    tutorial.mode = 'build';
    tutorial.started = performance.now();
    applyAssemblyView();
  };
  $('tutorial-assembly').onclick = () => {
    tutorial.mode = 'assembly';
    if (state.assemblyStep < 0) state.assemblyStep = 0;
    tutorial.started = performance.now();
    applyAssemblyView();
  };
  const refreshList = () => {
    $('asm-pos').textContent = `${isBuild ? 'build' : 'assembly'} ${step + 1} / ${labels.length}`;
    for (const li of $('asm-steps').children) {
      const on = +li.dataset.i === step;
      li.classList.toggle('active-step', on);
      li.style.opacity = on ? '1' : '0.42';
      li.onclick = () => setStep(+li.dataset.i);
    }
  };
  $('asm-prev').onclick = () => setStep(step - 1);
  $('asm-next').onclick = () => setStep(step + 1);
  refreshList();
  if (assemblyTools) assemblyTools.visible = true;
  if (cgMarker) cgMarker.visible = !!active.cg;
  frameAssemblyClip();
  setStatus(`tutorial · ${isBuild ? 'build' : 'assembly'} ${step + 1}`);
}

// ---- Balance tab: mass properties → CG → trim attitude ---------------------
function buildCgMarker() {
  const cg = state.manifest?.balance?.cg_mm;
  if (!cg) return;
  const geo = new THREE.SphereGeometry(Math.max(state.radius * 0.04, 8), 24, 18);
  const mat = new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.92 });
  cgMarker = new THREE.Mesh(geo, mat);
  cgMarker.position.set(cg[0], cg[1], cg[2]);
  cgMarker.visible = false;
  pivot.add(cgMarker);
}

function makeTube(points, color, radius = 5, opacity = 0.55) {
  const curve = new THREE.CatmullRomCurve3(points);
  const geo = new THREE.TubeGeometry(curve, Math.max(16, points.length * 5), radius, 8, false);
  const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
  return new THREE.Mesh(geo, mat);
}

function buildFeaOverlay() {
  feaOverlay = new THREE.Group();
  feaOverlay.visible = false;
  for (const part of state.parts) {
    if (!['blade', 'boom', 'mast', 'landing_gear', 'avionics'].includes(part.group)) continue;
    const box = new THREE.Box3().setFromObject(part.mesh);
    const c = new THREE.Vector3(); box.getCenter(c);
    const s = new THREE.Vector3(); box.getSize(s);
    const r = Math.max(10, Math.min(70, Math.max(s.x, s.y, s.z) * 0.035));
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(r, Math.max(1.5, r * 0.035), 8, 36),
      new THREE.MeshBasicMaterial({ color: part.group === 'avionics' ? 0x4f93c4 : ACCENT, transparent: true, opacity: 0.55 })
    );
    ring.position.copy(c);
    ring.userData.sourcePartId = part.id;
    feaOverlay.add(ring);
  }
  pivot.add(feaOverlay);
}

function buildCfdOverlay() {
  cfdOverlay = new THREE.Group();
  cfdOverlay.visible = false;
  const r = (state.manifest?.design?.radius_m?.value ?? 0.65) * 1000;
  const downwash = state.manifest?.cfd?.downwash_mps ?? 8;
  const wakeLen = Math.max(650, downwash * 115);
  for (let i = 0; i < 18; i++) {
    const a = (i / 18) * Math.PI * 2;
    const rr = r * (0.2 + 0.78 * ((i % 6) + 1) / 6);
    const x = Math.cos(a) * rr;
    const y = Math.sin(a) * rr;
    const swirl = (i % 2 ? -1 : 1) * 0.18;
    const pts = [];
    for (let k = 0; k < 8; k++) {
      const t = k / 7;
      const aa = a + swirl * k;
      pts.push(new THREE.Vector3(Math.cos(aa) * rr * (1 - 0.18 * t), Math.sin(aa) * rr * (1 - 0.18 * t), -wakeLen * t));
    }
    cfdOverlay.add(makeTube(pts, i % 3 === 0 ? 0xffb56b : ACCENT, Math.max(2.5, r * 0.004), 0.34));
    if (i % 4 === 0) {
      const tip = new THREE.Mesh(
        new THREE.SphereGeometry(Math.max(9, r * 0.018), 18, 12),
        new THREE.MeshBasicMaterial({ color: 0xff5d6c, transparent: true, opacity: 0.48 })
      );
      tip.position.set(x, y, -wakeLen * 0.22);
      cfdOverlay.add(tip);
    }
  }
  const wake = new THREE.Mesh(
    new THREE.CylinderGeometry(r * 0.98, r * 0.55, wakeLen, 48, 1, true),
    new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.065, side: THREE.DoubleSide, depthWrite: false })
  );
  wake.position.z = -wakeLen * 0.5;
  cfdOverlay.add(wake);
  pivot.add(cfdOverlay);
}

function buildAssemblyTools() {
  assemblyTools = new THREE.Group();
  assemblyTools.visible = false;
  const toolMat = new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.9 });
  const epoxyMat = new THREE.MeshBasicMaterial({ color: 0xffb56b, transparent: true, opacity: 0.75 });

  const reamer = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, Math.max(120, state.radius * 0.18), 18), toolMat);
  reamer.userData.role = 'reamer';
  assemblyTools.add(reamer);

  const bushing = new THREE.Mesh(new THREE.CylinderGeometry(10, 10, 55, 24), toolMat);
  bushing.userData.role = 'bushing';
  assemblyTools.add(bushing);

  const epoxy = new THREE.Mesh(new THREE.TorusGeometry(36, 3, 8, 44), epoxyMat);
  epoxy.userData.role = 'epoxy';
  assemblyTools.add(epoxy);

  const arrow = makeTube(
    [new THREE.Vector3(-70, 0, 0), new THREE.Vector3(70, 0, 0)],
    ACCENT,
    3,
    0.7
  );
  arrow.userData.role = 'arrow';
  assemblyTools.add(arrow);

  pivot.add(assemblyTools);
}

function toolByRole(role) {
  return assemblyTools?.children.find((o) => o.userData.role === role);
}

function applyBalanceView() {
  inspTitle.textContent = 'Mass & balance';
  if (cgMarker) cgMarker.visible = true;
  const b = state.manifest?.balance;
  if (!b) { inspBody.innerHTML = '<div style="color:var(--ink-faint)">no balance data</div>'; return; }
  const comps = [...b.components].sort((a, z) => z.mass_kg - a.mass_kg);
  const maxm = comps[0]?.mass_kg || 1;
  const rows = comps.map((c) => {
    const nm = c.id.replace(/_/g, ' ');
    return `<div class="kv" style="margin:8px 0 2px"><span class="k">${esc(nm)}</span>
      <span class="v">${(c.mass_kg * 1000).toFixed(0)} g · x ${c.cg_mm[0].toFixed(0)} mm</span></div>
      <div class="bar"><i style="width:${(c.mass_kg / maxm * 100).toFixed(0)}%"></i></div>`;
  }).join('');
  const effect = b.converged
    ? `<div class="kv">
         <span class="k">CG offset (aft +)</span><span class="v">${(b.cg_offset_m * 1000).toFixed(1)} mm</span>
         <span class="k">Trim pitch @ CG</span><span class="v">${b.trim_pitch_deg.toFixed(2)}°</span>
         <span class="k">…CG under shaft</span><span class="v">${b.trim_pitch_centered_deg.toFixed(2)}°</span>
         <span class="k">Sensitivity</span><span class="v">${b.dpitch_dcg_deg_per_m.toFixed(0)} °/m</span>
       </div>
       <div style="margin-top:6px;font-size:12px;color:var(--ink-dim)">Moving mass aft tilts the trimmed hover <b style="color:var(--accent)">${(b.trim_pitch_deg - b.trim_pitch_centered_deg >= 0 ? 'nose-up' : 'nose-down')}</b> — the layout feeds the validated trim balance, which also shifts the stability derivatives.</div>`
    : `<div style="color:var(--ink-faint)">trim did not converge for this geometry</div>`;
  inspBody.innerHTML = `<span class="tag">${b.total_mass_kg.toFixed(2)} kg total</span>
    <h4>Centre of gravity → trim attitude</h4>
    <div style="color:var(--ink-faint);font-size:12px;margin-bottom:6px">The cyan marker is the mass-weighted CG. Its longitudinal offset feeds the trim <code>cg_offset</code> (validated in Milestone 6), tilting the trimmed hover attitude — so component placement changes trim &amp; stability.</div>
    ${effect}
    <h4>Component masses</h4>${rows}
    <div style="margin-top:10px;font-size:11px;color:var(--ink-faint)">Battery (pack energy), motor (power) and rotor-group masses are physics-based; structure carries the remainder of the gross mass — representative distribution.</div>`;
}

// ---- CFD + Flight tabs ------------------------------------------------------
function applyCfdView() {
  if (cfdOverlay) cfdOverlay.visible = true;
  for (const part of state.parts) {
    restoreBase(part);
    if (['fuselage', 'blade', 'tail_rotor', 'tail_fin'].includes(part.group)) {
      part.mesh.material.metalness = 0.25;
      part.mesh.material.roughness = 0.42;
    } else {
      part.mesh.material.opacity = part.mesh.userData.translucent ? 0.04 : 0.12;
      part.mesh.material.transparent = true;
    }
  }
  const cfd = state.manifest?.cfd ?? {};
  const av = state.manifest?.balance?.avionics_effect;
  inspTitle.textContent = 'CFD · rotor wake';
  inspBody.innerHTML = `<span class="tag">flow visualization</span>
    <div style="margin:10px 0;color:var(--ink-faint);font-size:12px">Stream tubes show rotor downwash and tip-vortex rollup. Internal avionics are hidden from the outer flow but their frontal envelope is tracked for packaging and cooling drag.</div>
    <div class="kv">
      <span class="k">Disk loading</span><span class="v">${(cfd.disk_loading_pa ?? 0).toFixed(1)} Pa</span>
      <span class="k">Tip Reynolds</span><span class="v">${((cfd.tip_re ?? 0) / 1000).toFixed(0)}k</span>
      <span class="k">Induced downwash</span><span class="v">${(cfd.downwash_mps ?? 0).toFixed(2)} m/s</span>
      <span class="k">Wake radius</span><span class="v">${(cfd.wake_radius_m ?? 0).toFixed(2)} m</span>
      ${av ? `<span class="k">Avionics frontal equiv.</span><span class="v">${(av.cfd_frontal_area_m2 * 1e4).toFixed(1)} cm²</span>` : ''}
    </div>`;
}

function applyFlightView() {
  flightActive = true;
  inspTitle.textContent = 'Flight simulator';
  inspBody.innerHTML = `<span class="tag">interactive</span>
    <div style="margin:10px 0;color:var(--ink-faint);font-size:12px">Use WASD for pitch/roll, Q/E for yaw, and R/F for collective. This is a lightweight visual simulator using the exported mass and stability data; the validated nonlinear model remains the CLI/WASM target.</div>
    <div class="kv" id="flight-readout"></div>
    <h4>Stability modes</h4>${renderModes()}`;
  setStatus('flight sim · WASD Q/E R/F');
  frameAll();
}

function renderModes() {
  const stab = state.manifest?.balance?.stability;
  if (!stab?.modes?.length) return '<div style="color:var(--ink-faint)">no modal data</div>';
  return stab.modes.map((m) => {
    const cls = m.stable ? 'ms-ok' : 'ms-bad';
    return `<div class="kv"><span class="k">${m.oscillatory ? 'oscillatory' : 'subsidence'}</span>
      <span class="v ${cls}">${m.re.toFixed(2)} ${m.im >= 0 ? '+' : '-'} ${Math.abs(m.im).toFixed(2)}i</span></div>`;
  }).join('');
}

function updateFlight(dt) {
  if (!flightActive) return;
  const rollIn = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  const pitchIn = (keys.has('KeyW') ? 1 : 0) - (keys.has('KeyS') ? 1 : 0);
  const yawIn = (keys.has('KeyE') ? 1 : 0) - (keys.has('KeyQ') ? 1 : 0);
  const colIn = (keys.has('KeyR') ? 1 : 0) - (keys.has('KeyF') ? 1 : 0);
  flight.throttle = Math.max(0.22, Math.min(0.88, flight.throttle + colIn * dt * 0.22));
  flight.roll += (rollIn * 0.55 - flight.roll * 2.2) * dt;
  flight.pitch += (pitchIn * 0.45 - flight.pitch * 2.0) * dt;
  flight.yaw += yawIn * dt * 0.9;
  const thrust = (flight.throttle - 0.52) * 820;
  const forward = new THREE.Vector3(Math.sin(flight.yaw), 0, Math.cos(flight.yaw));
  const right = new THREE.Vector3(Math.cos(flight.yaw), 0, -Math.sin(flight.yaw));
  flight.vel.addScaledVector(forward, flight.pitch * dt * 520);
  flight.vel.addScaledVector(right, flight.roll * dt * 480);
  flight.vel.y += thrust * dt;
  flight.vel.multiplyScalar(Math.pow(0.82, dt));
  flight.pos.addScaledVector(flight.vel, dt);
  flight.pos.y = Math.max(-450, Math.min(900, flight.pos.y));
  pivot.position.copy(flight.pos);
  pivot.rotation.set(-Math.PI / 2 + flight.pitch, 0, flight.yaw + flight.roll * 0.35);

  const readout = $('flight-readout');
  if (readout) {
    readout.innerHTML = `
      <span class="k">Collective</span><span class="v">${(flight.throttle * 100).toFixed(0)}%</span>
      <span class="k">Airspeed visual</span><span class="v">${(flight.vel.length() / 18).toFixed(1)} m/s</span>
      <span class="k">Altitude visual</span><span class="v">${Math.max(0, flight.pos.y / 10).toFixed(0)} m</span>
      <span class="k">Attitude</span><span class="v">${(flight.pitch * 57.3).toFixed(1)}° / ${(flight.roll * 57.3).toFixed(1)}°</span>`;
  }
}

function frameAssemblyClip() {
  const clip = currentTutorialClip();
  const selected = state.parts.filter((p) => clip.parts.includes(p.id));
  const box = new THREE.Box3();
  for (const p of selected) {
    p.mesh.updateMatrixWorld(true);
    box.union(new THREE.Box3().setFromObject(p.mesh));
  }
  if (!box.isEmpty()) {
    const c = new THREE.Vector3(); box.getCenter(c);
    const s = new THREE.Vector3(); box.getSize(s);
    const r = Math.max(s.x, s.y, s.z, 120) * 0.5;
    const dist = (r / Math.tan((camera.fov * Math.PI) / 360)) * 1.35 + r;
    const dir = new THREE.Vector3(0.85, 0.48, 0.9).normalize();
    flyTo(c.clone().applyMatrix4(pivot.matrixWorld).add(dir.multiplyScalar(dist)), c.clone().applyMatrix4(pivot.matrixWorld), 450);
  }
}

function setToolVisible(role, on) {
  const tool = toolByRole(role);
  if (tool) tool.visible = on;
  return tool;
}

function updateAssemblyClip(now) {
  if (state.tab !== 'assembly') return;
  const clip = currentTutorialClip();
  const t = ((now - tutorial.started) / 1000) % 5.6;
  const phase = t / 5.6;
  const ease = phase < 0.5 ? 2 * phase * phase : 1 - Math.pow(-2 * phase + 2, 2) / 2;
  const selected = new Set(clip.parts);
  const visibleParts = state.parts.filter((p) => selected.has(p.id));
  for (const p of state.parts) {
    const on = visibleParts.includes(p);
    p.mesh.visible = on;
    p.mesh.position.set(0, 0, 0);
    p.mesh.rotation.set(0, 0, 0);
    if (on) {
      restoreBase(p);
      p.mesh.material.opacity = p.mesh.userData.translucent ? 0.34 : 1;
      p.mesh.material.transparent = p.mesh.userData.translucent;
      p.mesh.material.emissive.setHex(p.group === 'blade' || p.id === 'swashplate' ? ACCENT : 0x000000);
      p.mesh.material.emissiveIntensity = p.group === 'blade' || p.id === 'swashplate' ? 0.12 : 0;
    }
  }

  if (assemblyTools) {
    assemblyTools.visible = true;
    for (const tool of assemblyTools.children) tool.visible = false;
  }

  const startOffset = (p) => {
    if (clip.motion === 'vertical') return new THREE.Vector3(0, 0, state.radius * 0.45);
    if (clip.motion === 'aft') return new THREE.Vector3(-state.radius * 0.55, 0, 0);
    if (clip.motion === 'slide') return new THREE.Vector3(state.radius * 0.42, 0, -state.radius * 0.12);
    if (clip.motion === 'access-slide') return new THREE.Vector3(0, state.radius * 0.44, state.radius * 0.10);
    if (clip.motion === 'cut-6061') return new THREE.Vector3(state.radius * 0.18, state.radius * 0.12, state.radius * 0.10);
    if (clip.motion === 'bushing-cut') return new THREE.Vector3(0, state.radius * 0.20, state.radius * 0.16);
    if (clip.motion === 'bond-only') return new THREE.Vector3(0, state.radius * 0.12, state.radius * 0.08);
    if (clip.motion === 'head-build') return new THREE.Vector3(0, 0, state.radius * 0.30);
    if (clip.motion === 'shell-build') return new THREE.Vector3(0, state.radius * 0.18, state.radius * 0.16);
    if (clip.motion === 'bench-build') return new THREE.Vector3(state.radius * 0.28, 0, state.radius * 0.08);
    if (clip.motion === 'tail-build') return new THREE.Vector3(-state.radius * 0.34, 0, state.radius * 0.04);
    if (clip.motion === 'radial') return p.explodeDir.clone().multiplyScalar(state.radius * 0.36);
    if (clip.motion === 'drop') return new THREE.Vector3(0, 0, state.radius * 0.34);
    return p.explodeDir.clone().multiplyScalar(state.radius * 0.28);
  };

  for (const p of visibleParts) {
    if (p.id === 'canopy' && ['access-slide', 'slide', 'vertical'].includes(clip.motion)) {
      p.mesh.position.set(0, state.radius * 0.20, state.radius * 0.30);
      p.mesh.rotation.x = -0.35;
      continue;
    }
    if (['fuselage', 'tail_boom_fairing', 'landing_gear'].includes(p.id)) continue;
    const k = ['control', 'spin', 'collective-drop', 'ream-bond', 'ream-only', 'blade-print', 'balance-blades'].includes(clip.motion) ? 1 : ease;
    p.mesh.position.copy(startOffset(p).multiplyScalar(1 - k));
  }

  let text = clip.text;
  if (clip.motion === 'access-slide') {
    text = phase < 0.22
      ? 'Remove the canopy/access cover so the fuselage opening is clear.'
      : phase < 0.72
        ? 'Slide the tray-mounted components through the opening into the internal bulkhead bay.'
        : 'Bolt tray and components to hardpoints; the cover goes back on after checks.';
  } else if (clip.motion === 'vertical') {
    text = phase < 0.60
      ? 'Lower the part along the mast/bearing axis with the access cover out of the way.'
      : clip.text;
  } else if (clip.motion === 'slide') {
    text = phase < 0.70
      ? 'Slide the battery through the canopy opening onto the exported tray geometry.'
      : 'Strap the battery down and verify the CG marker remains near the shaft axis.';
  }
  if (clip.motion === 'blade-print') {
    const blade = state.parts.find((p) => p.id === 'blade');
    if (blade) {
      blade.mesh.rotation.z = Math.sin(phase * Math.PI * 2) * 0.08;
      blade.mesh.material.emissive.setHex(ACCENT);
      blade.mesh.material.emissiveIntensity = 0.22;
    }
    text = phase < 0.45
      ? 'Check the exported airfoil and twist against the SVG/DXF before ordering.'
      : 'Order the full blade as one SLS PA-CF print so the thin airfoil is continuous.';
  } else if (clip.motion === 'ream-bond' || clip.motion === 'ream-only' || clip.motion === 'bond-only') {
    const blade = state.parts.find((p) => p.id === 'blade');
    const anchor = blade?.centroid.clone() ?? state.center.clone();
    const reamer = setToolVisible('reamer', clip.motion !== 'bond-only' && phase < 0.45);
    const bushing = setToolVisible('bushing', clip.motion !== 'ream-only' && phase >= 0.30 && phase < 0.78);
    const epoxy = setToolVisible('epoxy', clip.motion !== 'ream-only' && phase >= 0.44 && phase < 0.90);
    if (reamer) {
      reamer.position.copy(anchor).add(new THREE.Vector3(0, 0, THREE.MathUtils.lerp(state.radius * 0.25, -state.radius * 0.08, Math.min(1, phase / 0.45))));
      reamer.rotation.z += 0.2;
    }
    if (bushing) bushing.position.copy(anchor).add(new THREE.Vector3(0, 0, THREE.MathUtils.lerp(state.radius * 0.18, 0, Math.max(0, (phase - 0.30) / 0.38))));
    if (epoxy) epoxy.position.copy(anchor);
    text = clip.motion === 'ream-only'
      ? 'Ream only: guide a finishing reamer through the printed pilot and the aluminium stack to keep the hole concentric.'
      : clip.motion === 'bond-only'
        ? 'Bond only: wet the bore and doubler faces with structural epoxy, clamp, and leave it through full cure.'
        : phase < 0.45
      ? 'Ream: use a finishing reamer in the printed blade-root pilot hole to make a round final-size bore.'
      : phase < 0.72
        ? 'Bond: wet the bore and slide the steel bushing into the actual blade root.'
        : 'Install: blades seat into the exported blade-grip geometry after the bond cures.';
  } else if (clip.motion === 'cut-6061') {
    const arrow = setToolVisible('arrow', true);
    const root = state.parts.find((p) => p.id === 'blade_root_fittings');
    if (root) {
      root.mesh.material.emissive.setHex(0xffb56b);
      root.mesh.material.emissiveIntensity = 0.30;
      root.mesh.position.y += Math.sin(phase * Math.PI * 2) * state.radius * 0.018;
      if (arrow) {
        arrow.position.copy(root.centroid).add(new THREE.Vector3(0, state.radius * 0.16, 0));
        arrow.rotation.z = Math.PI / 2;
      }
    }
    text = 'Cut and deburr ten 6061 plates before bonding: two doublers per blade, one on each root face.';
  } else if (clip.motion === 'bushing-cut') {
    const bushing = setToolVisible('bushing', true);
    const blade = state.parts.find((p) => p.id === 'blade');
    const anchor = blade?.centroid.clone() ?? state.center.clone();
    if (bushing) {
      bushing.position.copy(anchor).add(new THREE.Vector3(0, state.radius * 0.16, 0));
      bushing.rotation.x = Math.PI / 2;
    }
    text = 'Cut each steel bushing to the measured root thickness; the bolt must bear on steel instead of printed polymer.';
  } else if (clip.motion === 'balance-blades') {
    const blades = visibleParts.filter((p) => p.group === 'blade');
    blades.forEach((p, i) => {
      const a = (i / Math.max(1, blades.length)) * Math.PI * 2;
      p.mesh.position.set(Math.cos(a) * state.radius * 0.26, Math.sin(a) * state.radius * 0.26, Math.sin(phase * Math.PI * 2 + i) * state.radius * 0.012);
      p.mesh.rotation.z = a + Math.sin(phase * Math.PI * 2) * 0.035;
    });
    const arrow = setToolVisible('arrow', true);
    if (arrow) {
      arrow.position.copy(state.center).add(new THREE.Vector3(0, 0, state.radius * 0.10));
      arrow.rotation.z = Math.PI / 2;
    }
    text = 'Balance the complete matched set: add tape or sand the lighter tips until every blade stays level at any angle.';
  } else if (clip.motion === 'head-build') {
    for (const p of visibleParts.filter((p) => p.id === 'swashplate')) p.mesh.position.z += Math.sin(phase * Math.PI * 2) * state.radius * 0.035;
    for (const p of visibleParts.filter((p) => p.id === 'hub' || p.id === 'blade_grips')) p.mesh.rotation.z = Math.sin(phase * Math.PI * 2) * 0.08;
    text = 'Machine, press bearings, and check free slide/tilt/runout before installing the rotor head on the airframe.';
  } else if (clip.motion === 'shell-build') {
    const canopy = visibleParts.find((p) => p.id === 'canopy');
    if (canopy) {
      canopy.mesh.position.set(0, state.radius * 0.18, state.radius * 0.26);
      canopy.mesh.rotation.x = -0.30;
    }
    text = 'Lay up the smooth pod, split the removable canopy, then add bulkheads for mast, skid, and boom loads.';
  } else if (clip.motion === 'bench-build') {
    for (const p of visibleParts.filter((p) => ['landing_gear', 'powertrain_tray', 'motor', 'esc', 'battery', 'avionics'].includes(p.id))) {
      p.mesh.position.copy(startOffset(p).multiplyScalar(1 - ease));
    }
    text = 'Bench-build the skids and tray, then bolt them to the fuselage hardpoints with the battery bay on the CG target.';
  } else if (clip.motion === 'tail-build') {
    for (const p of visibleParts.filter((p) => ['tail_boom', 'tail_rotor', 'tail_fin', 'horizontal_stab'].includes(p.id))) {
      p.mesh.position.copy(startOffset(p).multiplyScalar(1 - ease));
    }
    text = 'Build the boom/tail rotor as a subassembly, route wiring or controls, then insert it into the aft hardpoint.';
  } else if (clip.motion === 'control') {
    const q = Math.sin(phase * Math.PI * 2);
    for (const p of visibleParts.filter((p) => p.id === 'swashplate')) p.mesh.position.z += q * state.radius * 0.035;
    for (const p of visibleParts.filter((p) => p.group === 'blade')) p.mesh.rotation.y = q * 0.08;
    text = 'Control check: swashplate motion drives pitch change in every installed blade.';
  } else if (clip.motion === 'spin' || clip.motion === 'collective-drop') {
    const spin = phase * Math.PI * (clip.motion === 'spin' ? 10 : 5);
    for (const p of visibleParts.filter((p) => p.group === 'blade' || p.id === 'hub' || p.id === 'blade_grips')) p.mesh.rotation.z = spin;
    if (clip.motion === 'collective-drop') {
      for (const p of visibleParts.filter((p) => p.group === 'blade')) p.mesh.rotation.y = THREE.MathUtils.lerp(0.15, -0.08, ease);
      text = 'Power-loss response: unload the rotor by dropping collective as RPM decays.';
    } else {
      text = 'Tethered spin-up: only the rotor group is shown so vibration and tracking are easy to inspect.';
    }
  }

  const arrow = setToolVisible('arrow', !['ream-bond', 'ream-only', 'bond-only', 'cut-6061', 'bushing-cut', 'balance-blades', 'spin', 'collective-drop', 'control'].includes(clip.motion));
  if (arrow && visibleParts.length) {
    const c = visibleParts[0].centroid.clone();
    arrow.position.copy(c).add(startOffset(visibleParts[0]).multiplyScalar(0.45 * (1 - ease)));
  }

  if (cgMarker) cgMarker.visible = !!clip.cg;
  const stage = $('tutorial-stage');
  if (stage) stage.textContent = text;
}

// ---- tabs ------------------------------------------------------------------
function resetMaterials() {
  for (const part of state.parts) restoreBase(part);
  for (const part of state.parts) part.mesh.visible = true;
}

function resetFlightPose() {
  flightActive = false;
  pivot.position.set(0, 0, 0);
  pivot.rotation.set(-Math.PI / 2, 0, 0);
  flight.pos.set(0, 0, 0);
  flight.vel.set(0, 0, 0);
  flight.yaw = 0; flight.pitch = 0; flight.roll = 0; flight.throttle = 0.52;
}

function switchTab(tab) {
  state.tab = tab;
  for (const b of $('tabs').children) b.classList.toggle('active', b.dataset.tab === tab);
  if (state.tab !== 'flight') resetFlightPose();
  clearSelection();
  resetMaterials();
  moduleVeil.classList.add('hidden');
  if (cgMarker) cgMarker.visible = tab === 'balance';
  if (feaOverlay) feaOverlay.visible = tab === 'fea';
  if (cfdOverlay) cfdOverlay.visible = tab === 'cfd';
  if (assemblyTools) assemblyTools.visible = tab === 'assembly';

  if (tab === 'model') { renderOverview(); setStatus('ready'); }
  else if (tab === 'fea') { applyFeaView(); setStatus('FEA field'); }
  else if (tab === 'cfd') { applyCfdView(); setStatus('CFD flow'); }
  else if (tab === 'optimize') { applyOptimalityView(); setStatus('optimality'); }
  else if (tab === 'balance') { applyBalanceView(); setStatus('mass & balance'); frameAll(); }
  else if (tab === 'assembly') { applyAssemblyView(); }
  else if (tab === 'flight') { applyFlightView(); }
}

// ---- interaction -----------------------------------------------------------
let downXY = null;
canvas.addEventListener('pointerdown', (e) => { downXY = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', (e) => {
  if (!downXY) return;
  const moved = Math.hypot(e.clientX - downXY[0], e.clientY - downXY[1]);
  downXY = null;
  if (moved > 5 || ['assembly', 'flight', 'optimize'].includes(state.tab)) return; // a drag, not a pickable view
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects(pivot.children, false);
  if (hits.length) selectPart(hits[0].object.userData.partId, true);
  else clearSelection();
});

$('explode').addEventListener('change', (e) => setExplode(e.target.checked));
$('reset-view').addEventListener('click', () => { clearSelection(); frameAll(); });
for (const b of $('tabs').children) b.addEventListener('click', () => switchTab(b.dataset.tab));
window.addEventListener('keydown', (e) => {
  if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'KeyR', 'KeyF'].includes(e.code)) keys.add(e.code);
});
window.addEventListener('keyup', (e) => keys.delete(e.code));

// ---- URL-driven state (deep links + screenshot harness) --------------------
function applyUrlState() {
  const q = new URLSearchParams(location.search);
  const build = q.get('build');
  if (build != null) {
    const n = Number(build);
    const clips = buildPartClips();
    if (Number.isFinite(n)) tutorial.buildStep = Math.max(0, Math.min(clips.length - 1, Math.floor(n)));
    tutorial.mode = 'build';
    switchTab('assembly');
    return;
  }
  const asm = q.get('asm');
  if (asm != null) {
    const n = Number(asm);
    if (Number.isFinite(n)) state.assemblyStep = Math.max(0, Math.min(ASSEMBLY_CLIPS.length - 1, Math.floor(n)));
    tutorial.mode = 'assembly';
    switchTab('assembly');
    return;
  }
  const tab = q.get('tab');
  const step = q.get('step');
  if (step != null) {
    const n = Number(step);
    if (Number.isFinite(n)) state.assemblyStep = Math.max(0, Math.min(ASSEMBLY_CLIPS.length - 1, Math.floor(n)));
  }
  if (tab) switchTab(tab);
  if (q.get('explode') === '1') { $('explode').checked = true; setExplode(true); }
  const part = q.get('part');
  if (part && (state.tab === 'model' || !tab)) {
    const p = state.parts.find((x) => x.id === part || x.group === part);
    if (p) selectPart(p.id, true);
  }
}

// ---- helpers ---------------------------------------------------------------
function setStatus(s) { status.textContent = s; }
function setDesignId() {
  const d = state.manifest?.design ?? {};
  const b = (d.blades?.value ?? 0).toFixed(0);
  const r = (d.radius_m?.value ?? 0).toFixed(2);
  designId.textContent = `${b}-blade · R ${r} m · ${(d.gross_mass_kg?.value ?? 0).toFixed(1)} kg`;
}
function esc(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

// ---- loop / resize ---------------------------------------------------------
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
let lastNow = performance.now();
function animate(now) {
  requestAnimationFrame(animate);
  resize();
  const dt = Math.min(0.05, Math.max(0.001, (now - lastNow) / 1000));
  lastNow = now;
  if (tween) tween(now);
  updateFlight(dt);
  updateAssemblyClip(now);
  applyExplode();
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(animate);
boot();
