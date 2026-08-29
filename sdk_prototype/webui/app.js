const state = {
  motors: [],
  selected: new Set(),
  firstLoad: true,
};

const rows = document.getElementById("motorRows");
const selectAll = document.getElementById("selectAll");
const connection = document.getElementById("connection");
const connectionText = document.getElementById("connectionText");
const subtitle = document.getElementById("subtitle");
const summary = document.getElementById("summary");
const events = document.getElementById("events");

function num(id) {
  return Number(document.getElementById(id).value);
}

function fmt(value, digits = 3) {
  if (!Number.isFinite(value)) return "-";
  return value.toFixed(digits);
}

function age(stamp) {
  if (!stamp) return "-";
  const seconds = Date.now() / 1000 - stamp;
  if (seconds < 0) return "0.0s";
  if (seconds < 60) return `${seconds.toFixed(1)}s`;
  return `${Math.floor(seconds / 60)}m`;
}

function selectedJoints() {
  return Array.from(state.selected);
}

function logEvent(text, isError = false) {
  const item = document.createElement("li");
  item.textContent = `${new Date().toLocaleTimeString()} ${text}`;
  if (isError) item.style.color = "#9b1c18";
  events.prepend(item);
  while (events.children.length > 30) events.lastElementChild.remove();
}

function setConnection(ok, detail) {
  connection.classList.toggle("ok", ok);
  connectionText.textContent = ok ? "Connected" : "Disconnected";
  subtitle.textContent = detail || "";
}

function render(data) {
  state.motors = data.motors || [];
  if (state.firstLoad) {
    state.selected = new Set(state.motors.map((motor) => motor.joint_name));
    state.firstLoad = false;
  }

  setConnection(data.connected, data.connected ? data.grpc_addr : data.error);
  const online = state.motors.filter((motor) => motor.online).length;
  summary.textContent = `${online}/${state.motors.length} online`;

  rows.replaceChildren(
    ...state.motors.map((motor) => {
      const tr = document.createElement("tr");
      tr.className = motor.online ? "" : "offline";
      tr.innerHTML = `
        <td><input type="checkbox" data-joint="${motor.joint_name}"></td>
        <td>${motor.joint_name}</td>
        <td class="num">${motor.motor_id ?? "-"}</td>
        <td>${motor.model ?? "-"}</td>
        <td class="num">${fmt(motor.position_rad)}</td>
        <td class="num">${fmt(motor.velocity_radps)}</td>
        <td class="num">${fmt(motor.effort_nm)}</td>
        <td class="num">${fmt(motor.temperature_c, 1)}</td>
        <td class="num">${age(motor.stamp_unix_s)}</td>
      `;
      const checkbox = tr.querySelector("input");
      checkbox.checked = state.selected.has(motor.joint_name);
      checkbox.addEventListener("change", () => {
        if (checkbox.checked) state.selected.add(motor.joint_name);
        else state.selected.delete(motor.joint_name);
        syncSelectAll();
      });
      return tr;
    })
  );
  syncSelectAll();
}

function syncSelectAll() {
  selectAll.checked = state.motors.length > 0 && state.selected.size === state.motors.length;
  selectAll.indeterminate = state.selected.size > 0 && state.selected.size < state.motors.length;
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    render(await response.json());
  } catch (error) {
    setConnection(false, error.message);
  }
}

async function command(action, params = {}) {
  const joints = selectedJoints();
  if (!joints.length) {
    logEvent("select at least one motor", true);
    return;
  }
  try {
    const response = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action, joints, params }),
    });
    const result = await response.json();
    logEvent(`${action}: ${result.message}`, !result.accepted);
    await refresh();
  } catch (error) {
    logEvent(`${action}: ${error.message}`, true);
  }
}

function paramsFor(action) {
  if (action === "set_position") {
    return { position_rad: num("positionRad"), velocity_radps: num("velocityRadps"), kp: num("kp"), kd: num("kd") };
  }
  if (action === "set_velocity") {
    return { velocity_radps: num("velocityRadps") };
  }
  if (action === "set_mit") {
    return {
      position_rad: num("positionRad"),
      velocity_radps: num("velocityRadps"),
      torque_nm: num("torqueNm"),
      kp: num("kp"),
      kd: num("kd"),
    };
  }
  if (action === "zero_position") {
    return { position_rad: 0, velocity_radps: num("velocityRadps"), kp: num("kp"), kd: num("kd") };
  }
  if (action === "goto") return { angle_deg: num("gotoDeg") };
  if (action === "step") return { delta_deg: num("stepDeg") };
  if (action === "sine") return { amp_deg: num("sineAmpDeg"), freq_hz: num("sineHz"), duration_s: num("sineDuration") };
  if (action === "set_kp") return { kp: num("kp") };
  if (action === "set_kd") return { kd: num("kd") };
  return {};
}

document.querySelectorAll("button[data-action]").forEach((button) => {
  button.addEventListener("click", () => {
    const action = button.dataset.action;
    const actualAction = action === "zero_position" ? "set_position" : action;
    command(actualAction, paramsFor(action));
  });
});

document.getElementById("refreshBtn").addEventListener("click", refresh);

selectAll.addEventListener("change", () => {
  state.selected = selectAll.checked
    ? new Set(state.motors.map((motor) => motor.joint_name))
    : new Set();
  render({ connected: connection.classList.contains("ok"), motors: state.motors, grpc_addr: subtitle.textContent });
});

refresh();
setInterval(refresh, 250);
