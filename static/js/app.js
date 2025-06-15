// Global variables
let cameraSettings = {
    deadZoneRadius: 60,
    stepSize: 3,
    smoothDelay: 0.05
};

// YENİ KAMERA AYARLARI FUNKTIONSYONLARI
function updateCameraSettings() {
    const deadZoneRadius = parseInt(document.getElementById('deadZoneRadius').value);
    const stepSize = parseInt(document.getElementById('cameraStepSize').value);
    const smoothDelayValue = parseInt(document.getElementById('smoothDelay').value);
    const smoothDelay = smoothDelayValue / 1000; // Convert to seconds

    // Update display values
    document.getElementById('deadZoneValue').textContent = deadZoneRadius + 'px';
    document.getElementById('stepSizeValue').textContent = stepSize + '°';
    document.getElementById('smoothDelayValue').textContent = smoothDelay.toFixed(3) + 's';

    // Send to server
    fetch('/api/camera_settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            dead_zone_radius: deadZoneRadius,
            step_size: stepSize,
            smooth_delay: smoothDelay
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            cameraSettings = data.settings;
            showMessage('Camera settings updated!', 'success');
        } else {
            showMessage('Settings update failed: ' + data.message, 'error');
        }
    })
    .catch(error => {
        showMessage('Settings error: ' + error, 'error');
    });
}

function cameraPreset(preset) {
    let panAngle, tiltAngle;

    switch(preset) {
        case 'home':
            panAngle = 90;
            tiltAngle = 90;
            break;
        case 'scan':
            // Scanning movement
            scanCamera();
            return;
        case 'low':
            panAngle = 90;
            tiltAngle = 120;
            break;
        case 'high':
            panAngle = 90;
            tiltAngle = 60;
            break;
    }

    fetch('/api/camera_control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            action: 'direct',
            pan_angle: panAngle,
            tilt_angle: tiltAngle
        })
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function scanCamera() {
    showMessage('Camera scanning mode activated!', 'info');

    const scanSequence = [
        {pan: 60, tilt: 90},
        {pan: 90, tilt: 60},
        {pan: 120, tilt: 90},
        {pan: 90, tilt: 120},
        {pan: 90, tilt: 90}  // Return to center
    ];

    let step = 0;
    const scanInterval = setInterval(() => {
        if (step >= scanSequence.length) {
            clearInterval(scanInterval);
            showMessage('Camera scan completed!', 'success');
            return;
        }

        const position = scanSequence[step];
        fetch('/api/camera_control', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                action: 'direct',
                pan_angle: position.pan,
                tilt_angle: position.tilt
            })
        });

        step++;
    }, 2000);
}

// EXISTING FUNCTIONS (updated with camera info)
function startCamera() {
    fetch('/api/start_camera', {method: 'POST'})
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function toggleTracking() {
    fetch('/api/toggle_tracking', {method: 'POST'})
    .then(response => response.json())
    .then(data => {
        showMessage(data.message, 'success');
        document.getElementById('trackingBtn').classList.toggle('active', data.tracking);
    });
}

function toggleCameraTracking() {
    fetch('/api/toggle_camera_tracking', {method: 'POST'})
    .then(response => response.json())
    .then(data => {
        showMessage(data.message, 'success');
        document.getElementById('cameraTrackingBtn').classList.toggle('active', data.camera_tracking);

        // Update tracking mode display
        document.getElementById('trackingMode').textContent = data.camera_tracking ? 'AUTO' : 'MANUAL';
    });
}

function setPowerMode() {
    const mode = document.getElementById('powerMode').value;
    fetch('/api/set_power_mode', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({mode: mode})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function manualMove(action) {
    fetch('/api/manual_move', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function cameraControl(action) {
    fetch('/api/camera_control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function advancedMove(action) {
    fetch('/api/advanced_move', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function adaptiveWalk() {
    const distance = document.getElementById('walkDistance').value;
    const direction = document.getElementById('walkDirection').value;

    fetch('/api/advanced_move', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            action: 'adaptive_walk',
            direction: direction,
            distance: parseInt(distance)
        })
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function multiStep() {
    const steps = document.getElementById('stepCount').value;
    const direction = document.getElementById('walkDirection').value;

    fetch('/api/advanced_move', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            action: 'multi_step',
            direction: direction,
            steps: parseInt(steps)
        })
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function updateROI() {
    const stopDistance = document.getElementById('stopDistance').value;
    const minDistance = document.getElementById('minDistance').value;
    const enabled = document.getElementById('roiEnabled').checked;

    document.getElementById('stopDistanceValue').textContent = stopDistance;
    document.getElementById('minDistanceValue').textContent = minDistance;

    fetch('/api/roi_settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            stop_distance: parseInt(stopDistance),
            min_distance: parseInt(minDistance),
            enabled: enabled
        })
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, 'success'));
}

function emergencyStop() {
    fetch('/api/stop_all', {method: 'POST'})
    .then(response => response.json())
    .then(data => {
        showMessage(data.message, 'error');
        // Reset all toggle buttons
        document.querySelectorAll('.toggle-btn').forEach(btn => btn.classList.remove('active'));
        document.getElementById('trackingMode').textContent = 'MANUAL';
    });
}

function updateStatus() {
    fetch('/api/status')
    .then(response => response.json())
    .then(data => {
        document.getElementById('trackingStatus').textContent = data.tracking_enabled ? 'ON' : 'OFF';
        document.getElementById('trackingStatus').className = 'status-value ' + (data.tracking_enabled ? 'active' : 'inactive');

        document.getElementById('cameraTrackingStatus').textContent = data.camera_tracking ? 'ON' : 'OFF';
        document.getElementById('cameraTrackingStatus').className = 'status-value ' + (data.camera_tracking ? 'active' : 'inactive');

        document.getElementById('humanStatus').textContent = data.human_detected ? 'YES' : 'NO';
        document.getElementById('humanStatus').className = 'status-value ' + (data.human_detected ? 'active' : 'inactive');

        document.getElementById('distanceStatus').textContent = data.human_distance + ' cm';
        document.getElementById('roiStatus').textContent = data.roi_zone.toUpperCase();
        document.getElementById('powerStatus').textContent = data.power_mode.toUpperCase();

        document.getElementById('walkingStatus').textContent = data.walking ? 'YES' : 'NO';
        document.getElementById('walkingStatus').className = 'status-value ' + (data.walking ? 'active' : 'inactive');

        // Update camera position display
        if (data.camera_position) {
            document.getElementById('panPosition').textContent = data.camera_position.pan + '°';
            document.getElementById('tiltPosition').textContent = data.camera_position.tilt + '°';
        }

        document.getElementById('trackingMode').textContent = data.camera_tracking ? 'AUTO' : 'MANUAL';
    })
    .catch(error => console.log('Status update error:', error));
}

function showMessage(text, type = 'success') {
    const message = document.createElement('div');
    message.className = `message ${type}`;
    message.textContent = text;
    document.body.appendChild(message);

    setTimeout(() => message.classList.add('show'), 100);
    setTimeout(() => {
        message.classList.remove('show');
        setTimeout(() => document.body.removeChild(message), 300);
    }, 3000);
}

// Initialize camera settings on page load
window.addEventListener('load', function() {
    updateCameraSettings();
});

// Update status every 2 seconds
setInterval(updateStatus, 2000);

// === REX QUAD STYLE FUNCTIONS ===
function rexMovement(action) {
    fetch('/api/rex_movement', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, data.status === 'success' ? 'success' : 'error'));
}

function updateRexStance() {
    const height = document.getElementById('rexStanceHeight').value;
    document.getElementById('rexStanceValue').textContent = height + '°';

    fetch('/api/rex_stance_control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({height: parseInt(height)})
    })
    .then(response => response.json())
    .then(data => showMessage(data.message, 'success'));
}

let rexModeEnabled = false;
function toggleRexMode() {
    rexModeEnabled = !rexModeEnabled;
    const btn = document.getElementById('rexModeBtn');

    if (rexModeEnabled) {
        btn.classList.add('active');
        btn.textContent = '🤖 REX Mode ON';
        showMessage('REX Quad mode activated! All movements now use ESP32 REX algorithms!', 'success');

        // Override existing movement functions to use REX
        window.originalManualMove = window.manualMove;
        window.manualMove = function(action) {
            const rexActions = {
                'forward': 'rex_forward',
                'left': 'rex_left',
                'right': 'rex_right',
                'back': 'rex_backward',
                'stance': 'rex_stabilize'
            };

            if (rexActions[action]) {
                rexMovement(rexActions[action]);
                showMessage(`REX Mode: Executing ${action} with 8-phase algorithm`, 'info');
            } else {
                window.originalManualMove(action);
            }
        };
    } else {
        btn.classList.remove('active');
        btn.textContent = '🤖 Enable REX Mode';
        showMessage('REX Quad mode deactivated - Back to normal spider movements', 'info');

        // Restore original movement functions
        if (window.originalManualMove) {
            window.manualMove = window.originalManualMove;
        }
    }
}

function rexDemo() {
    showMessage('Starting REX Quad demo sequence...', 'info');

    const sequence = [
        {action: 'rex_stabilize', msg: 'REX Stabilizing...'},
        {action: 'rex_lean_left', msg: 'REX Leaning Left...'},
        {action: 'rex_lean_right', msg: 'REX Leaning Right...'},
        {action: 'rex_lean_forward', msg: 'REX Leaning Forward...'},
        {action: 'rex_lean_back', msg: 'REX Leaning Back...'},
        {action: 'rex_forward', msg: 'REX Forward Gait...'},
        {action: 'rex_left', msg: 'REX Left Turn...'},
        {action: 'rex_right', msg: 'REX Right Turn...'},
        {action: 'rex_backward', msg: 'REX Backward Gait...'},
        {action: 'rex_stabilize', msg: 'REX Demo Complete!'}
    ];

    let step = 0;
    const demoInterval = setInterval(() => {
        if (step >= sequence.length) {
            clearInterval(demoInterval);
            showMessage('REX Quad demo completed! All ESP32 algorithms tested!', 'success');
            return;
        }

        const current = sequence[step];
        rexMovement(current.action);
        showMessage(current.msg, 'info');
        step++;
    }, 4000); // 4 saniye aralık
}


function toggleFeature(endpoint) {
    fetch(endpoint, { method: 'POST' })
        .then(response => response.json())
        .then(data => {
            console.log('Success:', data);
            // Butonun durumunu belirten bir mesaj gösterelim
            const featureName = endpoint.includes('gamma') ? 'Auto Gamma' : 'Histogram Equalization';
            const status = Object.values(data)[1] ? 'enabled' : 'disabled'; // Gelen yanıta göre durumu al
            showMessage(`${featureName} ${status}.`, 'success');
        })
        .catch((error) => {
            console.error('Error:', error);
            showMessage(`Error toggling feature: ${error}`, 'error');
        });
}
// app.js dosyanızın sonuna ekleyin

// --- LED KONTROL FONKSİYONLARI ---

// Sunucuya sürekli istek göndermemek için debounce (gecikme) mekanizması
let ledBrightnessTimeout;

function setLedMode(mode) {
    // Butonların aktif durumunu güncelle
    document.getElementById('ledBtnOff').classList.toggle('active', mode === 'off');
    document.getElementById('ledBtnAuto').classList.toggle('active', mode === 'auto');
    document.getElementById('ledBtnManual').classList.toggle('active', mode === 'manual');

    // Manuel parlaklık kontrolünü sadece manuel modda göster
    const manualControl = document.getElementById('manualLedControl');
    manualControl.style.display = (mode === 'manual') ? 'flex' : 'none';

    // Sunucuya modu gönder
    fetch('/set_led_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: mode })
    })
    .then(response => response.json())
    .then(data => {
        if (data.status === 'success') {
            showMessage(data.message, 'success');
            // Eğer moda manuel olarak geçildiyse, mevcut slider değeriyle parlaklığı ayarla
            if (mode === 'manual') {
                setLedBrightness();
            }
        } else {
            showMessage(data.message, 'error');
        }
    });
}

function setLedBrightness() {
    const brightnessSlider = document.getElementById('ledBrightness');
    const brightnessValue = parseInt(brightnessSlider.value);

    // Görünen değeri güncelle
    document.getElementById('ledBrightnessValue').textContent = brightnessValue + '%';

    // Sunucuyu aşırı yüklememek için debounce kullan
    clearTimeout(ledBrightnessTimeout);
    ledBrightnessTimeout = setTimeout(() => {
        fetch('/set_led_brightness', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ brightness: brightnessValue })
        })
        .then(response => response.json())
        .then(data => {
            if (data.status !== 'success') {
                showMessage(data.message, 'error');
            }
        });
    }, 250); // Kullanıcı kaydırmayı bıraktıktan 250ms sonra gönder
}

// Initial status update
updateStatus();