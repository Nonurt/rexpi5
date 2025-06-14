function sendCommand(endpoint) {
  fetch(endpoint)
    .then(res => res.text())
    .then(data => console.log("Command sent:", data))
    .catch(err => console.error("Error:", err));
}

function toggleTracking() {
  fetch("/api/toggle_tracking", {
    method: "POST"
  })
  .then(res => res.json())
  .then(data => {
    alert("Tracking: " + (data.tracking ? "Enabled" : "Disabled"));
  });
}