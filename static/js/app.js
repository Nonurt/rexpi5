/*  app.js  –  REX-Pi5 UI helpers
    =============================================== */

/* ——— DOM erişim referansları ——— */
const sliders      = document.getElementById("sliders");
const trackBtn     = document.getElementById("trackBtn");
const autostepBtn  = document.getElementById("autostepBtn");
const gammaBtn     = document.getElementById("gammaBtn");
const histBtn      = document.getElementById("histBtn");
const facesaveBtn  = document.getElementById("facesaveBtn");
const detSel       = document.getElementById("detSel");
const targetSel    = document.getElementById("targetSel");
const refreshTgt   = document.getElementById("refreshTgt");
const st           = document.getElementById("st");
const stv          = document.getElementById("stv");
const raw          = document.getElementById("raw");
const sendRaw      = document.getElementById("sendRaw");

/* ——— basit API çağrısı ——— */
const api = (p, opt = undefined) =>
  fetch(p.startsWith("/") ? p : "/" + p, opt);

/* ——— toggle (ON / OFF) ——— */
function toggle(btn, path) {
  const on = !btn.classList.contains("on");
  btn.classList.toggle("on", on);
  btn.textContent = btn.textContent.replace(on ? "OFF" : "ON", on ? "ON" : "OFF");
  api(path + "?v=" + (on ? 1 : 0));
}

/* ——— hedef ID listesini doldur ——— */
async function loadTargets() {
  const j = await api("/targets").then((r) => r.json());
  targetSel.innerHTML =
    '<option value="">None</option>' +
    j.ids
      .map(
        (id) =>
          `<option ${id === j.current ? "selected" : ""}>${id}</option>`
      )
      .join("");
}

/* ——— DOM hazır olduğunda ——— */
document.addEventListener("DOMContentLoaded", () => {
  /* doğrudan komut düğmeleri (gait / lean ...) */
  document
    .querySelectorAll("button[data-cmd]")
    .forEach((b) => (b.onclick = () => api("/" + b.dataset.cmd)));

  /* kamera-işleme toggles */
  trackBtn.onclick     = () => toggle(trackBtn, "/track");
  autostepBtn.onclick  = () => toggle(autostepBtn, "/autostep");
  gammaBtn.onclick     = () => toggle(gammaBtn, "/gamma");
  histBtn.onclick      = () => toggle(histBtn, "/hist");
  facesaveBtn.onclick  = () => toggle(facesaveBtn, "/facesave");

  /* detector modu seçimi */
  detSel.onchange = () => api("/detmode?m=" + detSel.value);

  /* stance slider */
  st.oninput = (e) => {
    stv.textContent = e.target.value;
    api("/cfg", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ stance_height: +e.target.value }),
    });
  };

  /* manual servo metin kutusu */
  sendRaw.onclick = () => {
    const t = raw.value.trim();
    if (t) api("/servo?cmd=" + encodeURIComponent(t));
  };

  /* 8 adet servo slider’ı oluştur */
  if (sliders) {
    for (let i = 0; i < 8; i++) {
      const d = document.createElement("div");
      d.innerHTML = `<label>#${i} <span id="v${i}">90</span></label>
                     <input type="range" min="0" max="180" value="90">`;
      const inp = d.querySelector("input"),
            sp = d.querySelector("span");
      inp.oninput = () => {
        api("/servo?cmd=" + i + ":" + inp.value);
        sp.textContent = inp.value;
      };
      sliders.appendChild(d);
    }
  }

  /* tüm servo kanallarını başlangıçta 90°'ye al */
  for (let i = 0; i < 8; i++) {
    api("/servo?cmd=" + i + ":90");
  }

  /* hedef listesi & olayları */
  loadTargets();
  refreshTgt.onclick = loadTargets;
  targetSel.onchange = () => api("/target?id=" + targetSel.value);

  /* ilk stance cfg’sini al */
  api("/cfg")
    .then((r) => r.json())
    .then((j) => {
      st.value = j.stance_height || 60;
      stv.textContent = st.value;
    });
});
