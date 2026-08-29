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

function fmtDeg(rad) {
  if (!Number.isFinite(rad)) return "-";
  return (rad * 180 / Math.PI).toFixed(2);
}

function logEvent(text, isError = false) {
  const item = document.createElement("li");
  item.textContent = `${new Date().toLocaleTimeString()} ${text}`;
  if (isError) item.style.color = "#9b1c18";
  events.prepend(item);
  while (events.children.length > 30) events.lastElementChild.remove();
}

function selectedIds() {
  return Array.from(state.selected);
}

function setConnection(data) {
  connection.classList.toggle("ok", data.connected);
  connectionText.textContent = data.connected ? "Connected" : "Disconnected";
  const safety = data.safety_tripped ? ` | safety: ${data.safety_reason}` : "";
  subtitle.textContent = `${data.channel} @ ${data.bitrate}${safety}`;
}

function render(data) {
  state.motors = data.motors || [];
  if (state.firstLoad) {
    state.selected = new Set(state.motors.map((motor) => motor.id));
    state.firstLoad = false;
  }

  setConnection(data);
  const enabled = state.motors.filter((motor) => motor.enabled).length;
  summary.textContent = `${enabled}/${state.motors.length} enabled, selected ${state.selected.size}`;

  rows.replaceChildren(
    ...state.motors.map((motor) => {
      const tr = document.createElement("tr");
      tr.className = motor.enabled ? "" : "disabled";
      tr.innerHTML = `
        <td><input type="checkbox" data-id="${motor.id}"></td>
        <td class="num">${motor.id}</td>
        <td>${motor.model}</td>
        <td>${motor.temp_state}${motor.excitation === "sine" ? " sine" : ""}</td>
        <td class="num">${fmt(motor.position_deg, 2)}</td>
        <td class="num">${fmtDeg(motor.commanded_target_rad)}</td>
        <td class="num">${fmt(motor.velocity_radps)}</td>
        <td class="num">${fmt(motor.torque_nm)}</td>
        <td class="num">${fmt(motor.temperature_c, 1)}</td>
        <td class="num">${fmt(motor.kp, 1)}</td>
        <td class="num">${fmt(motor.kd, 2)}</td>
        <td>${motor.direction === -1 ? "INV" : "NOR"}</td>
        <td class="num">${fmt(motor.loop_dt_ms, 1)}</td>
        <td class="error" title="${motor.last_error || ""}">${motor.last_error || ""}</td>
      `;
      const checkbox = tr.querySelector("input");
      checkbox.checked = state.selected.has(motor.id);
      checkbox.addEventListener("change", () => {
        if (checkbox.checked) state.selected.add(motor.id);
        else state.selected.delete(motor.id);
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
    connection.classList.remove("ok");
    connectionText.textContent = "Server unavailable";
    subtitle.textContent = error.message;
  }
}

async function command(action, params = {}) {
  const motorIds = selectedIds();
  if (!motorIds.length && !["connect", "disconnect"].includes(action)) {
    logEvent("select at least one motor", true);
    return;
  }
  try {
    const response = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action, motor_ids: motorIds, params }),
    });
    const result = await response.json();
    logEvent(`${action}: ${result.message}`, !result.accepted);
    await refresh();
  } catch (error) {
    logEvent(`${action}: ${error.message}`, true);
  }
}

function paramsFor(action) {
  if (action === "goto") return { angle_deg: num("gotoDeg") };
  if (action === "step") return { delta_deg: num("stepDeg") };
  if (action === "sine") return { amp_deg: num("sineAmpDeg"), freq_hz: num("sineHz"), duration_s: num("sineDuration") };
  if (action === "set_kp") return { kp: num("kp") };
  if (action === "set_kd") return { kd: num("kd") };
  return {};
}

document.querySelectorAll("button[data-action]").forEach((button) => {
  button.addEventListener("click", () => command(button.dataset.action, paramsFor(button.dataset.action)));
});

document.getElementById("refreshBtn").addEventListener("click", refresh);

selectAll.addEventListener("change", () => {
  state.selected = selectAll.checked
    ? new Set(state.motors.map((motor) => motor.id))
    : new Set();
  render({
    motors: state.motors,
    connected: connection.classList.contains("ok"),
    channel: subtitle.textContent,
    bitrate: "",
  });
});

refresh();
setInterval(refresh, 250);
