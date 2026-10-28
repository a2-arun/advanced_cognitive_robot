"use strict";
(function () {
  var TOKEN = new URLSearchParams(location.search).get("token") || "";
  var S = null, waypointMode = false, cloud = null, yaw = 0.5, pitch = 0.25, dragging = null, cloudTimer = null;
  var $ = function (id) { return document.getElementById(id); };

  function url(path) { return path + (TOKEN ? (path.indexOf("?") < 0 ? "?" : "&") + "token=" + encodeURIComponent(TOKEN) : ""); }
  function toast(msg, bad) {
    var t = $("captureMsg"); t.textContent = msg; t.style.color = bad ? "var(--bad)" : "var(--dim)";
  }
  function api(path, body) {
    return fetch(url(path), { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) })
      .then(function (r) { return r.json(); })
      .then(function (j) { if (j && j.ok === false && j.error) toast(j.error, true); return j; })
      .catch(function () { toast("server unreachable", true); return { ok: false }; });
  }
  function fmt(v, d, u) { return (v === null || v === undefined) ? "n/a" : Number(v).toFixed(d === undefined ? 2 : d) + (u || ""); }
  function kv(el, rows) {
    el.textContent = "";
    rows.forEach(function (r) {
      var a = document.createElement("span"); a.textContent = r[0];
      var b = document.createElement("span"); b.textContent = r[1];
      el.appendChild(a); el.appendChild(b);
    });
  }
  function chips(el, names, cur) {
    el.textContent = "";
    names.forEach(function (n) {
      var s = document.createElement("span"); s.textContent = n.replace(/_/g, " ");
      if (n === cur) s.className = "on" + (/STOP/.test(n) ? " stopped" : "");
      el.appendChild(s);
    });
  }
  function kindClass(k) { k = (k || "").toLowerCase(); return k.indexOf("sim") === 0 ? "sim" : k === "live" ? "live" : k === "recorded" ? "recorded" : ""; }

  // ---------------------------------------------------------------- streams
  function bindStream(id, name) {
    var img = $(id);
    function set() { img.src = url("/stream/" + name + ".mjpg") + (TOKEN ? "&" : "?") + "t=" + Date.now(); }
    img.onerror = function () { setTimeout(set, 1500); };
    set();
  }
  bindStream("imgObs", "obstacle"); bindStream("imgDock", "dock"); bindStream("imgDepth", "depth");

  // ---------------------------------------------------------------- polling + render
  function poll() {
    fetch(url("/api/state"), { cache: "no-store" }).then(function (r) { return r.json(); }).then(function (j) {
      if (j.ok === false) { banner("alert", j.error); return; }
      S = j; render(j);
    }).catch(function () { banner("alert", "Dashboard server unreachable - motors are stopped by the robot's own watchdog if the link drops."); });
  }
  setInterval(poll, 250); poll();

  function banner(cls, text) { var b = $("banner"); b.className = "banner " + cls; b.textContent = text; }

  function render(S) {
    var sim = S.mode === "sim", hw = S.hw, auto = S.behavior !== "off";
    $("modeSim").className = sim ? "on" : ""; $("modeHw").className = sim ? "" : "on";
    var dl = $("dataLabel");
    dl.textContent = sim ? "SIMULATED DATA" : (S.data_label === "LIVE" ? "LIVE CAMERA DATA" : S.data_label === "RECORDED" ? "RECORDED FOOTAGE" : "NO CAMERA");
    dl.className = "chip " + (sim ? "sim" : kindClass(S.data_label) || "bad");
    var lc = $("linkChip");
    if (sim) { lc.textContent = "SIM: no motor commands are sent"; lc.className = "chip sim"; }
    else if (!hw.connected) { lc.textContent = "SERIAL: not connected"; lc.className = "chip bad"; }
    else if (!hw.link_ok) { lc.textContent = "SERIAL LINK DOWN - controls disabled"; lc.className = "chip bad"; }
    else { lc.textContent = (hw.mock ? "MOCK FIRMWARE " : "ARDUINO ") + hw.port + (hw.armed ? " · ARMED" : " · disarmed"); lc.className = "chip " + (hw.armed ? "ok" : "live"); }
    $("clock").textContent = S.time + " · " + S.camera.fps + " fps";
    if (sim) banner("sim", "SIMULATION MODE — everything on this page is simulated. No physical motor command can be sent from here.");
    else if (hw.connected && !hw.link_ok) banner("alert", "HARDWARE LINK LOST — hardware commands are blocked. The Arduino stops the motors by itself.");
    else banner("hw", "HARDWARE MODE — buttons move the REAL robot (full speed, short pulses). Wheels lifted for first tests. STOP is always active.");

    $("hwCard").style.display = sim ? "none" : "";
    $("simCard").style.display = sim ? "" : "none";
    $("missionCard").style.display = sim ? "" : "none";
    $("teleopTarget").textContent = sim ? "→ simulator" : "→ real robot";
    $("teleopTarget").className = "tag " + (sim ? "sim" : "live");

    var canMove = sim ? (S.sim && S.sim.running) : S.hw_can_move;
    document.querySelectorAll("button.dir[data-dir]").forEach(function (b) { b.disabled = !canMove || auto; });
    $("pulse").min = S.cfg.min_pulse; $("pulse").max = S.cfg.max_pulse;

    // hardware card
    if (!sim) {
      kv($("hwKv"), [["Port", hw.port || "-"], ["Firmware", hw.fw || "-"], ["State", hw.state], ["Armed", hw.armed ? "YES" : "no"],
        ["Telemetry age", hw.telemetry_age_s === null ? "-" : hw.telemetry_age_s + " s"], ["FW watchdog trips", hw.firmware_timeouts],
        ["Last error", hw.last_error || "none"]]);
      var cells = $("encRow").children; for (var i = 0; i < 4; i++) cells[i].textContent = hw.enc ? hw.enc[i] : "-";
      $("btnArm").disabled = !hw.link_ok; $("btnDisarm").disabled = !hw.connected;
      $("btnConnect").disabled = hw.connected && hw.link_ok;      // allow reconnect after a dropped cable
      $("btnDisconnect").disabled = !hw.connected;
    }
    // history
    var ul = $("history"); ul.textContent = "";
    S.history.slice().reverse().forEach(function (h) {
      var li = document.createElement("li"); if (!h.ok) li.className = "bad";
      var t = document.createElement("span"); t.className = "t"; t.textContent = h.t + " ";
      li.appendChild(t); li.appendChild(document.createTextNode("[" + h.src + "] " + h.text)); ul.appendChild(li);
    });
    // obstacle
    var det = S.obstacle.det || {}, of = S.obstacle.fsm || {};
    var os = $("obsState");
    os.textContent = det.state === "OBSTACLE" ? "OBSTACLE AHEAD" : det.state === "CLEAR" ? "PATH CLEAR" : (det.state || "-").replace("_", " ");
    os.className = "big " + (det.state === "CLEAR" ? "ok" : det.state === "OBSTACLE" ? "bad" : "warn");
    kv($("obsKv"), [["Front", fmt(det.front_m, 2, " m")], ["Left", fmt(det.left_m, 2, " m")], ["Right", fmt(det.right_m, 2, " m")],
      ["Valid depth (front)", det.valid_front === undefined ? "n/a" : Math.round(det.valid_front * 100) + " %"],
      ["Stop threshold", fmt(S.cfg.stop_distance_m, 2, " m")],
      ["Decision", of.intended ? (of.intended === "L" ? "turn LEFT" : "turn RIGHT") : (of.state || "-")]]);
    chips($("avoidFsm"), ["CLEAR", "OBSTACLE_DETECTED", "SELECT_DIRECTION", "TURN", "RECHECK", "STOP_BLOCKED", "STOP_UNRELIABLE"], of.state);
    $("obsText").textContent = of.text || "";
    // docking
    var d = S.docking, ob = d.obs || {}, df = d.fsm || {};
    var ds = $("dockState");
    ds.textContent = df.state === "DOCKED" ? "DOCKED" : ob.detected ? "MARKER DETECTED" : "NO MARKER";
    ds.className = "big " + (df.state === "DOCKED" ? "ok" : ob.detected ? "ok" : "warn");
    kv($("dockKv"), [["Alignment", df.decision || "-"], ["Image offset", ob.detected ? (ob.offset >= 0 ? "+" : "") + fmt(ob.offset, 2) : "n/a"],
      ["Distance", ob.distance_m === null || ob.distance_m === undefined ? "n/a" : fmt(ob.distance_m, 2, " m") + " (" + ob.distance_src + ")"],
      ["Dock distance", fmt(d.dock_distance_m, 2, " m")], ["Marker size", fmt(d.marker_size_m * 100, 0, " cm")],
      ["Pulses used", df.actions_used === undefined ? "-" : df.actions_used]]);
    chips($("dockFsm"), ["SEARCH", "MARKER_DETECTED", "ALIGN", "APPROACH", "DOCKED"], df.state);
    $("dockBar").style.width = Math.round((df.progress || 0) * 100) + "%";
    $("dockText").textContent = (d.aruco_error ? "ArUco unavailable: " + d.aruco_error : df.text || "");
    // autonomy
    document.querySelectorAll("#behaviorSeg button").forEach(function (b) {
      b.className = b.getAttribute("data-b") === S.behavior ? "on" : "";
    });
    $("bMission").disabled = !sim;
    if (auto) $("actToggle").checked = S.act;            // server is the source of truth while a behaviour runs
    else if (!sim) $("actToggle").checked = false;       // hardware: never stay pre-armed after STOP / link loss
    $("advToggle").checked = S.advance_when_clear;
    var pill = $("autonomyStatus");
    if (S.behavior === "off") { pill.textContent = "manual teleoperation"; pill.className = "pill"; }
    else if (S.act) { pill.textContent = "ACTING: " + S.behavior.toUpperCase() + (sim ? " (simulated)" : " (REAL ROBOT)"); pill.className = "pill act"; }
    else { pill.textContent = "PERCEPTION ONLY: " + S.behavior + " — robot will not move"; pill.className = "pill"; }
    // mission / sim
    var m = S.mission;
    kv($("missionKv"), [["Stage", m.stage], ["Waypoint", m.waypoint ? m.waypoint[0] + ", " + m.waypoint[1] + " m" : "none"], ["Note", m.text]]);
    if (sim && S.sim && !S.sim.inactive) {
      $("simPause").textContent = S.sim.running ? "Pause" : "Resume";
      kv($("simKv"), [["Sim time", S.sim.sim_time + " s"], ["Pulses", S.sim.pulses], ["Event", S.sim.event],
        ["Sim encoders", S.sim.enc.join(" / ")], ["Collision", S.sim.collided ? "YES" : "no"]]);
    }
    var kc = kindClass(S.camera.kind || (sim ? "simulated" : ""));
    ["tagObs", "tagDock", "tagDepth"].forEach(function (id) { var e = $(id); e.textContent = sim ? "SIMULATED" : (S.data_label || ""); e.className = "tag " + (sim ? "sim" : kc); });
    $("tagMap").textContent = sim ? "2D simulation · not SLAM" : (S.odometry.calibrated ? "encoder dead reckoning · not SLAM" : "no localisation");
    $("tagMap").className = "tag " + (sim ? "sim" : "");
    document.querySelectorAll("[data-rec]").forEach(function (b) {
      var on = S.recording.indexOf(b.getAttribute("data-rec")) >= 0; b.className = "btn" + (on ? " rec" : ""); b.textContent = (on ? "■ stop " : "● rec ") + b.getAttribute("data-rec");
    });
    drawMap(S);
  }

  // ---------------------------------------------------------------- map
  var mapGeom = null;
  function drawMap(S) {
    var cv = $("map"), g = cv.getContext("2d"), W = cv.width, H = cv.height;
    var sim = S.mode === "sim" && S.sim && !S.sim.inactive;
    var ww = sim ? S.sim.world.w : 8, wh = sim ? S.sim.world.h : 6, pad = 20;
    var sc = Math.min((W - 2 * pad) / ww, (H - 2 * pad) / wh);
    var ox = (W - ww * sc) / 2, oy = (H - wh * sc) / 2;
    mapGeom = { sc: sc, ox: ox, oy: oy, wh: wh, ww: ww };
    function X(x) { return ox + x * sc; } function Y(y) { return oy + (wh - y) * sc; }
    g.clearRect(0, 0, W, H); g.fillStyle = "#0a0e14"; g.fillRect(0, 0, W, H);
    g.fillStyle = "#121923"; g.fillRect(X(0), Y(wh), ww * sc, wh * sc);
    g.strokeStyle = "#1f2a38"; g.lineWidth = 1;
    for (var i = 0; i <= ww; i++) { g.beginPath(); g.moveTo(X(i), Y(0)); g.lineTo(X(i), Y(wh)); g.stroke(); }
    for (var j = 0; j <= wh; j++) { g.beginPath(); g.moveTo(X(0), Y(j)); g.lineTo(X(ww), Y(j)); g.stroke(); }
    g.strokeStyle = "#5b6b80"; g.lineWidth = 3; g.strokeRect(X(0), Y(wh), ww * sc, wh * sc);
    g.font = "12px sans-serif"; g.fillStyle = "#8b98a8";
    if (!sim) {
      var o = S.odometry;
      g.fillStyle = "#8b98a8"; g.textAlign = "center";
      if (!o.calibrated) {
        g.fillText("HARDWARE: no localisation. Encoder counts are uncalibrated,", W / 2, H / 2 - 8);
        g.fillText("so no pose or map is drawn. (SLAM = future work)", W / 2, H / 2 + 10);
      } else {
        g.textAlign = "left"; g.fillText("encoder dead reckoning from start (0,0) — drifts, NOT SLAM", X(0.1), Y(wh) + 16);
        g.strokeStyle = "#ff9f43"; g.beginPath();
        o.trail.forEach(function (p, k) { var px = X(1 + p[0]), py = Y(wh / 2 + p[1]); if (k) g.lineTo(px, py); else g.moveTo(px, py); }); g.stroke();
      }
      $("mapLegend").textContent = "Hardware mode: this schematic floor is empty until odometry is calibrated.";
      return;
    }
    var s = S.sim;
    function box(b, fill, stroke) { g.fillStyle = fill; g.strokeStyle = stroke; g.lineWidth = 1.5; g.fillRect(X(b.x0), Y(b.y1), (b.x1 - b.x0) * sc, (b.y1 - b.y0) * sc); g.strokeRect(X(b.x0), Y(b.y1), (b.x1 - b.x0) * sc, (b.y1 - b.y0) * sc); }
    s.world.shelves.forEach(function (b) { box(b, "#1d4ed8aa", "#60a5fa"); });
    s.world.obstacles.forEach(function (b) { box(b, "#dc2626aa", "#f87171"); });
    // dock
    var dk = s.dock; g.fillStyle = "#fff"; g.fillRect(X(dk.x) - 7, Y(dk.y + 0.4), 7, 0.8 * sc); g.fillStyle = "#000"; g.fillRect(X(dk.x) - 5, Y(dk.y + 0.2), 4, 0.4 * sc);
    g.fillStyle = "#3fb950"; g.textAlign = "right"; g.fillText("DOCK + ArUco", X(dk.x) - 12, Y(dk.y) - 0.45 * sc);
    // trails
    function trail(pts, col, dash) { if (pts.length < 2) return; g.strokeStyle = col; g.lineWidth = 2; g.setLineDash(dash); g.beginPath(); pts.forEach(function (p, k) { if (k) g.lineTo(X(p[0]), Y(p[1])); else g.moveTo(X(p[0]), Y(p[1])); }); g.stroke(); g.setLineDash([]); }
    trail(s.trail, "#3fb6ff", []); trail(s.est_trail, "#ff9f43", [5, 4]);
    // waypoint
    if (S.mission.waypoint) { var wx = X(S.mission.waypoint[0]), wy = Y(S.mission.waypoint[1]); g.strokeStyle = "#facc15"; g.lineWidth = 2; g.beginPath(); g.arc(wx, wy, 8, 0, 7); g.moveTo(wx - 12, wy); g.lineTo(wx + 12, wy); g.moveTo(wx, wy - 12); g.lineTo(wx, wy + 12); g.stroke(); }
    // robot + FOV
    var r = s.robot, rx = X(r.x), ry = Y(r.y), a = -r.th, half = (s.hfov_deg / 2) * Math.PI / 180;
    g.fillStyle = "rgba(63,182,255,.10)"; g.beginPath(); g.moveTo(rx, ry); g.arc(rx, ry, 3 * sc, a - half, a + half); g.closePath(); g.fill();
    g.fillStyle = s.collided ? "#f85149" : (r.moving ? "#facc15" : "#3fb6ff"); g.beginPath(); g.arc(rx, ry, r.radius * sc, 0, 7); g.fill();
    g.strokeStyle = "#fff"; g.lineWidth = 2.5; g.beginPath(); g.moveTo(rx, ry); g.lineTo(rx + Math.cos(a) * r.radius * sc * 1.5, ry + Math.sin(a) * r.radius * sc * 1.5); g.stroke();
    $("mapLegend").textContent = "blue = true path · orange dashed = path estimated from noisy wheel odometry (drifts) · yellow = waypoint · shaded wedge = camera FOV";
  }
  $("map").addEventListener("click", function (ev) {
    if (!mapGeom || !S || S.mode !== "sim") return;
    var rect = $("map").getBoundingClientRect(), k = $("map").width / rect.width;
    var px = (ev.clientX - rect.left) * k, py = (ev.clientY - rect.top) * k;
    var x = (px - mapGeom.ox) / mapGeom.sc, y = mapGeom.wh - (py - mapGeom.oy) / mapGeom.sc;
    if (x < 0 || y < 0 || x > mapGeom.ww || y > mapGeom.wh) return;
    if (waypointMode) { waypointMode = false; $("btnWaypoint").textContent = "Pick waypoint on map"; api("/api/mission", { x: x, y: y }); }
    else api("/api/sim", { action: "toggle_obstacle", x: x, y: y });
  });

  // ---------------------------------------------------------------- controls
  $("pulse").addEventListener("input", function () { $("pulseVal").textContent = $("pulse").value; });
  var lastSent = 0;
  function move(dir) {
    var now = Date.now(), ms = parseInt($("pulse").value, 10);
    if (now - lastSent < Math.min(ms + 150, 600)) return;       // no pile-up of pulses
    lastSent = now;
    var btn = document.querySelector('button.dir[data-dir="' + dir + '"]');
    if (btn) { btn.classList.add("flash"); setTimeout(function () { btn.classList.remove("flash"); }, 140); }
    api("/api/move", { dir: dir, ms: ms });
  }
  document.querySelectorAll("button.dir[data-dir]").forEach(function (b) { b.addEventListener("click", function () { move(b.getAttribute("data-dir")); }); });
  function stop() { api("/api/stop"); lastSent = 0; }
  $("stopBtn").addEventListener("pointerdown", stop); $("dpadStop").addEventListener("pointerdown", stop);
  var KEYS = { w: "F", arrowup: "F", s: "B", arrowdown: "B", a: "L", arrowleft: "L", d: "R", arrowright: "R" };
  document.addEventListener("keydown", function (e) {
    if (/INPUT|SELECT|TEXTAREA/.test(e.target.tagName) && e.key !== "Escape") return;
    var k = e.key.toLowerCase();
    if (k === " " || k === "escape") { e.preventDefault(); stop(); return; }
    if (KEYS[k] && !e.repeat) { e.preventDefault(); move(KEYS[k]); }
  });
  document.querySelectorAll("#modeSim,#modeHw").forEach(function (b) {
    b.addEventListener("click", function () { api("/api/mode", { mode: b.getAttribute("data-mode") }); });
  });
  $("btnConnect").addEventListener("click", function () { toast("connecting (about 3 s)..."); api("/api/connect", { port: $("port").value.trim() || "auto" }).then(function (j) { if (j.ok) toast("connected"); }); });
  $("btnDisconnect").addEventListener("click", function () { api("/api/disconnect"); });
  $("btnArm").addEventListener("click", function () {
    if (confirm("ARM motors? Keep the wheels lifted (or the area clear) and a hand on the battery switch.")) api("/api/arm");
  });
  $("btnDisarm").addEventListener("click", function () { api("/api/disarm"); });

  function applyBehavior(b) {
    var act = $("actToggle").checked && b !== "off";
    if (act && S && S.mode === "hardware" && !confirm("The REAL robot will move by itself in short pulses.\nWheels lifted or area clear? Hand on the power switch?")) { $("actToggle").checked = false; act = false; }
    api("/api/behavior", { behavior: b, act: act, advance: $("advToggle").checked });
  }
  document.querySelectorAll("#behaviorSeg button").forEach(function (b) { b.addEventListener("click", function () { applyBehavior(b.getAttribute("data-b")); }); });
  $("actToggle").addEventListener("change", function () { if (S && S.behavior !== "off") applyBehavior(S.behavior); });
  $("advToggle").addEventListener("change", function () { if (S) api("/api/behavior", { behavior: S.behavior, act: S.act, advance: $("advToggle").checked }); });
  $("simPause").addEventListener("click", function () { api("/api/sim", { action: S && S.sim && S.sim.running ? "pause" : "resume" }); });
  $("simStep").addEventListener("click", function () { api("/api/sim", { action: "step" }); });
  $("simReset").addEventListener("click", function () { api("/api/sim", { action: "reset" }); });
  $("btnGoDock").addEventListener("click", function () { api("/api/mission", {}); });
  $("btnWaypoint").addEventListener("click", function () { waypointMode = !waypointMode; $("btnWaypoint").textContent = waypointMode ? "Click the map…" : "Pick waypoint on map"; });
  $("btnShot").addEventListener("click", function () { api("/api/snapshot").then(function (j) { if (j.ok) toast("saved " + j.files.length + " files in results/screenshots"); }); });
  document.querySelectorAll("[data-rec]").forEach(function (b) {
    b.addEventListener("click", function () {
      var n = b.getAttribute("data-rec"), on = S && S.recording.indexOf(n) >= 0;
      api("/api/record", { action: on ? "stop" : "start", stream: n }).then(function (j) { if (j.ok && j.path) toast("recording to " + j.path); });
    });
  });
  $("btnCam").addEventListener("click", function () { api("/api/source", { source: $("camSel").value }); });

  // ---------------------------------------------------------------- point cloud
  function getCloud(save) {
    if (save) { api("/api/pointcloud/save").then(function (j) { if (j.ok) toast("saved " + j.saved); }); return; }
    fetch(url("/api/pointcloud"), { cache: "no-store" }).then(function (r) { return r.json(); }).then(function (j) {
      if (!j.ok) { cloud = null; $("cloudNote").textContent = j.error; drawCloud(); return; }
      cloud = j; $("cloudNote").textContent = j.n + " points · " + j.note;
      var t = $("tagCloud"); t.textContent = j.kind; t.className = "tag " + kindClass(j.kind); drawCloud();
    });
  }
  $("btnCloud").addEventListener("click", function () { getCloud(false); });
  $("btnCloudSave").addEventListener("click", function () { getCloud(true); });
  $("cloudLive").addEventListener("change", function () {
    if (cloudTimer) { clearInterval(cloudTimer); cloudTimer = null; }
    if ($("cloudLive").checked) cloudTimer = setInterval(function () { getCloud(false); }, 1000);
  });
  var cc = $("cloud");
  cc.addEventListener("pointerdown", function (e) { dragging = [e.clientX, e.clientY]; cc.setPointerCapture(e.pointerId); });
  cc.addEventListener("pointerup", function () { dragging = null; });
  cc.addEventListener("pointermove", function (e) {
    if (!dragging) return; yaw += (e.clientX - dragging[0]) * 0.01; pitch = Math.max(-1.3, Math.min(1.3, pitch + (e.clientY - dragging[1]) * 0.01));
    dragging = [e.clientX, e.clientY]; drawCloud();
  });
  function drawCloud() {
    var g = cc.getContext("2d"), W = cc.width, H = cc.height;
    g.fillStyle = "#05080c"; g.fillRect(0, 0, W, H);
    if (!cloud) { g.fillStyle = "#8b98a8"; g.font = "13px sans-serif"; g.textAlign = "center"; g.fillText("no point cloud captured (needs a depth frame)", W / 2, H / 2); return; }
    var cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch), f = 380, zc = 2.0;
    var P = cloud.points, C = cloud.colors, out = [];
    for (var i = 0; i < P.length; i++) {
      var x = P[i][0], y = P[i][1], z = P[i][2] - zc;
      var x1 = x * cy + z * sy, z1 = -x * sy + z * cy;
      var y2 = y * cp - z1 * sp, z2 = y * sp + z1 * cp + zc + 1.5;
      if (z2 < 0.2) continue;
      out.push([W / 2 + f * x1 / z2, H / 2 + f * y2 / z2, z2, i]);
    }
    out.sort(function (a, b) { return b[2] - a[2]; });
    for (var k = 0; k < out.length; k++) {
      var p = out[k], c = C && C.length ? C[p[3]] : null;
      g.fillStyle = c ? "rgb(" + c[0] + "," + c[1] + "," + c[2] + ")" : "hsl(" + Math.max(0, 240 - P[p[3]][2] * 60) + ",80%,55%)";
      g.fillRect(p[0], p[1], 2.5, 2.5);
    }
    g.strokeStyle = "#3fb6ff"; g.lineWidth = 2;
    [[1, 0, 0, "#f85149"], [0, 1, 0, "#3fb950"], [0, 0, 1, "#3fb6ff"]].forEach(function (ax) {
      var x = ax[0] * .3, y = ax[1] * .3, z = ax[2] * .3 - zc;
      var x1 = x * cy + z * sy, z1 = -x * sy + z * cy, y2 = y * cp - z1 * sp, z2 = y * sp + z1 * cp + zc + 1.5;
      var ox = -0 * cy + (-zc) * sy, oz = (-zc) * cy, oy = 0 - oz * sp, oz2 = oz * cp + zc + 1.5;
      g.strokeStyle = ax[3]; g.beginPath(); g.moveTo(W / 2 + f * ox / oz2, H / 2 + f * oy / oz2); g.lineTo(W / 2 + f * x1 / z2, H / 2 + f * y2 / z2); g.stroke();
    });
    g.fillStyle = "#8b98a8"; g.font = "11px sans-serif"; g.textAlign = "left"; g.fillText("camera at origin · red=x(right) green=y(down) blue=z(forward)", 8, H - 8);
  }
  drawCloud();
})();
