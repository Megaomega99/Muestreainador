// ===== CONFIGURACIÓN GLOBAL =====
const API_BASE = window.location.origin;
let ws = null;
let wsData = null;
let currentFilterData = null;
let isStreamingActive = false;

// Buffers para datos en tiempo real (ventana deslizante de 0.3 segundos a 500 Hz = 150 puntos)
const MAX_POINTS = 150;
const WINDOW_TIME = 0.3; // segundos
let dataBuffers = {
    time: [],
    voltage: [],
    filtered: [],
    magnitude: [],
    phase: [],
    pulseTime: [],      // Timestamps of detected pulses
    pulseVoltage: []    // Voltage values at pulse detection
};

// Variable para detectar flancos ascendentes de pulsos del Arduino
let wasInPulse = false;

// ===== ELEMENTOS DEL DOM =====
const elements = {
    // Serial connection
    portSelect: document.getElementById('port-select'),
    refreshPortsBtn: document.getElementById('refresh-ports'),
    connectBtn: document.getElementById('connect-btn'),
    connectionStatus: document.getElementById('connection-status'),

    // Filter parameters
    centerFreq: document.getElementById('center-freq'),
    centerFreqInput: document.getElementById('center-freq-input'),
    bandwidth: document.getElementById('bandwidth'),
    bandwidthInput: document.getElementById('bandwidth-input'),
    qFactor: document.getElementById('q-factor'),
    loadDefaultsBtn: document.getElementById('load-defaults'),

    // Detection parameters
    magnitudeThreshold: document.getElementById('magnitude-threshold'),
    magnitudeThresholdInput: document.getElementById('magnitude-threshold-input'),
    phaseThreshold: document.getElementById('phase-threshold'),
    phaseThresholdInput: document.getElementById('phase-threshold-input'),
    predictionDelay: document.getElementById('prediction-delay'),
    predictionDelayInput: document.getElementById('prediction-delay-input'),

    // Action buttons
    calculateBtn: document.getElementById('calculate-btn'),
    sendBtn: document.getElementById('send-btn'),
    resetArduinoBtn: document.getElementById('reset-arduino-btn'),

    // Display elements
    coeffB0: document.getElementById('coeff-b0'),
    coeffB1: document.getElementById('coeff-b1'),
    coeffB2: document.getElementById('coeff-b2'),
    coeffA1: document.getElementById('coeff-a1'),
    coeffA2: document.getElementById('coeff-a2'),

    delayIIR: document.getElementById('delay-iir'),
    delayHilbert: document.getElementById('delay-hilbert'),
    delayTotal: document.getElementById('delay-total'),
    delayMs: document.getElementById('delay-ms'),
    delayRecommended: document.getElementById('delay-recommended'),

    console: document.getElementById('console'),
    clearConsoleBtn: document.getElementById('clear-console'),
    magnitudePlot: document.getElementById('magnitude-plot'),
    phasePlot: document.getElementById('phase-plot'),

    // Real-time visualization (4 separate plots)
    voltagePlot: document.getElementById('voltage-plot'),
    filteredPlot: document.getElementById('filtered-plot'),
    magnitudeStreamPlot: document.getElementById('magnitude-stream-plot'),
    phaseStreamPlot: document.getElementById('phase-stream-plot'),
    toggleStreamBtn: document.getElementById('toggle-stream')
};

// ===== INICIALIZACIÓN =====
document.addEventListener('DOMContentLoaded', () => {
    initializeWebSocket();
    setupEventListeners();
    updateRangeValues();
    refreshPorts();
    initializePlot();
    initializeRealtimePlots();
    logMessage('Interfaz web iniciada', 'info');
});

// ===== WEBSOCKET =====
function initializeWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket conectado');
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        handleWebSocketMessage(data);
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    ws.onclose = () => {
        console.log('WebSocket desconectado. Reconectando...');
        setTimeout(initializeWebSocket, 3000);
    };
}

function handleWebSocketMessage(data) {
    if (data.type === 'log') {
        logMessage(data.message, data.level);
    }
}

// ===== EVENT LISTENERS =====
function setupEventListeners() {
    // Serial connection
    elements.refreshPortsBtn.addEventListener('click', refreshPorts);
    elements.connectBtn.addEventListener('click', toggleConnection);

    // Filter parameters - Range inputs sync with number inputs
    elements.centerFreq.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        elements.centerFreqInput.value = value;
        updateQFactor();
    });

    elements.centerFreqInput.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 1 && value <= 500) {
            elements.centerFreq.value = value;
            updateQFactor();
        }
    });

    elements.bandwidth.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        elements.bandwidthInput.value = value;
        updateQFactor();
    });

    elements.bandwidthInput.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 1 && value <= 100) {
            elements.bandwidth.value = value;
            updateQFactor();
        }
    });

    // Detection parameters - Range inputs sync with number inputs
    elements.magnitudeThreshold.addEventListener('input', (e) => {
        elements.magnitudeThresholdInput.value = parseFloat(e.target.value).toFixed(3);
        updateThresholdLines();
    });

    elements.magnitudeThresholdInput.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 0.001 && value <= 1.0) {
            elements.magnitudeThreshold.value = value;
            updateThresholdLines();
        }
    });

    elements.phaseThreshold.addEventListener('input', (e) => {
        elements.phaseThresholdInput.value = parseFloat(e.target.value).toFixed(1);
        updateThresholdLines();
    });

    elements.phaseThresholdInput.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        if (!isNaN(value) && value >= 0.1 && value <= 180) {
            elements.phaseThreshold.value = value;
            updateThresholdLines();
        }
    });

    elements.predictionDelay.addEventListener('input', (e) => {
        elements.predictionDelayInput.value = e.target.value;
    });

    elements.predictionDelayInput.addEventListener('input', (e) => {
        const value = parseInt(e.target.value);
        if (!isNaN(value) && value >= 0 && value <= 500) {
            elements.predictionDelay.value = value;
        }
    });

    // Buttons
    elements.loadDefaultsBtn.addEventListener('click', loadDefaultValues);
    elements.calculateBtn.addEventListener('click', calculateFilter);
    elements.sendBtn.addEventListener('click', sendToArduino);
    elements.resetArduinoBtn.addEventListener('click', resetArduino);
    elements.clearConsoleBtn.addEventListener('click', clearConsole);
    elements.toggleStreamBtn.addEventListener('click', toggleDataStream);
}

// ===== FUNCIONES DE UTILIDAD =====
function updateRangeValues() {
    // Sync range sliders with number inputs
    elements.centerFreqInput.value = parseFloat(elements.centerFreq.value);
    elements.bandwidthInput.value = parseFloat(elements.bandwidth.value);
    elements.magnitudeThresholdInput.value = elements.magnitudeThreshold.value;
    elements.phaseThresholdInput.value = elements.phaseThreshold.value;
    elements.predictionDelayInput.value = elements.predictionDelay.value;
    updateQFactor();
}

function updateQFactor() {
    const freq = parseFloat(elements.centerFreq.value);
    const bw = parseFloat(elements.bandwidth.value);
    const q = bw > 0 ? freq / bw : 0;
    elements.qFactor.textContent = q.toFixed(3);
}

function updateThresholdLines() {
    // Update magnitude plot threshold line
    const magThreshold = parseFloat(elements.magnitudeThreshold.value);
    const magThresholdVolts = magThreshold * (5.0 / 1.024);  // Convertir normalizado (0-1) a voltios
    const magnitudeUpdate = {
        'shapes[0].y0': magThresholdVolts,
        'shapes[0].y1': magThresholdVolts,
        'annotations[0].y': magThresholdVolts
    };

    const magnitudePlot = document.getElementById('magnitude-stream-plot');
    if (magnitudePlot && magnitudePlot.data) {
        Plotly.relayout('magnitude-stream-plot', magnitudeUpdate);
    }

    // Update phase plot threshold lines
    const phaseThreshold = parseFloat(elements.phaseThreshold.value);
    const upperThreshold = 180 - phaseThreshold;
    const lowerThreshold = -(180 - phaseThreshold);

    const phaseUpdate = {
        'shapes[0].y0': upperThreshold,
        'shapes[0].y1': upperThreshold,
        'shapes[1].y0': lowerThreshold,
        'shapes[1].y1': lowerThreshold,
        'annotations[0].y': upperThreshold,
        'annotations[1].y': lowerThreshold
    };

    const phasePlot = document.getElementById('phase-stream-plot');
    if (phasePlot && phasePlot.data) {
        Plotly.relayout('phase-stream-plot', phaseUpdate);
    }
}

function getCurrentParameters() {
    // Convert physical units to Arduino internal format
    const magnitudeNormalized = parseFloat(elements.magnitudeThreshold.value);
    const phaseDegrees = parseFloat(elements.phaseThreshold.value);

    // Magnitude: normalized (0-1) -> integer threshold
    // El CORDIC produce magnitude en unidades ADC (0-512 típico para señal centrada)
    // magnitudeNormalized de 0.05 = 5% del rango -> 512 * 0.05 = 25.6 ADC units
    // Arduino hace: MAGNITUDE_THRESHOLD = valor << 15 para comparar con envelope
    const magnitude_threshold = Math.round(magnitudeNormalized * 512);  // Scale to ADC units

    // Phase: degrees from ±180° -> Q15 phase threshold
    // Phase threshold is checked as: phase > (32768 - threshold) or phase < (-32768 + threshold)
    // So if user wants 15°, threshold = 15° * (32768/180) = 2730
    const phase_q15 = Math.round(phaseDegrees * (32768 / 180));

    return {
        center_freq: parseFloat(elements.centerFreq.value),
        bandwidth: parseFloat(elements.bandwidth.value),
        magnitude_threshold: magnitude_threshold,
        phase_threshold: phase_q15,
        prediction_delay: parseInt(elements.predictionDelay.value)
    };
}

// ===== SERIAL PORT FUNCTIONS =====
async function refreshPorts() {
    try {
        const response = await fetch(`${API_BASE}/api/ports`);
        const data = await response.json();

        if (data.success) {
            elements.portSelect.innerHTML = '<option value="">Seleccione un puerto...</option>';
            data.ports.forEach(port => {
                const option = document.createElement('option');
                option.value = port.device;
                option.textContent = `${port.device} - ${port.description}`;
                elements.portSelect.appendChild(option);
            });

            logMessage(`${data.ports.length} puerto(s) encontrado(s)`, 'info');
        }
    } catch (error) {
        logMessage(`Error al actualizar puertos: ${error.message}`, 'error');
    }
}

async function toggleConnection() {
    const isConnected = elements.connectBtn.textContent.trim() === 'Desconectar';

    if (isConnected) {
        await disconnect();
    } else {
        await connect();
    }
}

async function connect() {
    const port = elements.portSelect.value;

    if (!port) {
        logMessage('Seleccione un puerto serial', 'warning');
        return;
    }

    try {
        const response = await fetch(`${API_BASE}/api/connect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ port })
        });

        const data = await response.json();

        if (data.success) {
            updateConnectionStatus(true);
            logMessage(data.message, 'success');
        } else {
            logMessage(data.message, 'error');
        }
    } catch (error) {
        logMessage(`Error de conexión: ${error.message}`, 'error');
    }
}

async function disconnect() {
    try {
        const response = await fetch(`${API_BASE}/api/disconnect`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            updateConnectionStatus(false);
            logMessage(data.message, 'info');
        }
    } catch (error) {
        logMessage(`Error al desconectar: ${error.message}`, 'error');
    }
}

function updateConnectionStatus(connected) {
    if (connected) {
        elements.connectionStatus.classList.remove('status-disconnected');
        elements.connectionStatus.classList.add('status-connected');
        elements.connectionStatus.querySelector('.status-text').textContent = 'Conectado';
        elements.connectBtn.innerHTML = '<span class="btn-text">Desconectar</span>';
        elements.sendBtn.disabled = false;
        elements.resetArduinoBtn.disabled = false;
        elements.toggleStreamBtn.disabled = false;
    } else {
        elements.connectionStatus.classList.remove('status-connected');
        elements.connectionStatus.classList.add('status-disconnected');
        elements.connectionStatus.querySelector('.status-text').textContent = 'Desconectado';
        elements.connectBtn.innerHTML = '<span class="btn-text">Conectar</span>';
        elements.sendBtn.disabled = true;
        elements.resetArduinoBtn.disabled = true;
        elements.toggleStreamBtn.disabled = true;

        // Detener stream si estaba activo
        if (isStreamingActive) {
            stopDataStream();
        }
    }
}

// ===== FILTER FUNCTIONS =====
async function loadDefaultValues() {
    try {
        const response = await fetch(`${API_BASE}/api/default-parameters`);
        const data = await response.json();

        elements.centerFreq.value = data.center_freq;
        elements.bandwidth.value = data.bandwidth;
        elements.magnitudeThreshold.value = data.magnitude_threshold;
        elements.phaseThreshold.value = data.phase_threshold;
        elements.predictionDelay.value = data.prediction_delay;

        updateRangeValues();
        logMessage('Valores por defecto cargados (21 Hz, BW=9 Hz)', 'success');
    } catch (error) {
        logMessage(`Error al cargar valores por defecto: ${error.message}`, 'error');
    }
}

async function calculateFilter() {
    const params = getCurrentParameters();

    try {
        elements.calculateBtn.disabled = true;
        elements.calculateBtn.innerHTML = '<div class="spinner"></div><span class="btn-text">Calculando...</span>';

        const response = await fetch(`${API_BASE}/api/calculate-filter`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(params)
        });

        const data = await response.json();

        if (data.success) {
            currentFilterData = data;
            displayFilterResults(data);
            updatePlot(data.plot_data);
            logMessage('Filtro calculado exitosamente', 'success');
        } else {
            logMessage('Error al calcular filtro', 'error');
        }
    } catch (error) {
        logMessage(`Error: ${error.message}`, 'error');
    } finally {
        elements.calculateBtn.disabled = false;
        elements.calculateBtn.innerHTML = '<span class="icon">🧮</span><span class="btn-text">Calcular Coeficientes</span>';
    }
}

function displayFilterResults(data) {
    // Coeficientes IIR
    elements.coeffB0.textContent = data.iir.b0_q15;
    elements.coeffB1.textContent = data.iir.b1_q15;
    elements.coeffB2.textContent = data.iir.b2_q15;
    elements.coeffA1.textContent = data.iir.a1_q15;
    elements.coeffA2.textContent = data.iir.a2_q15;

    // Información de retrasos
    elements.delayIIR.textContent = `${data.delay.iir_delay_samples.toFixed(2)} muestras`;
    elements.delayHilbert.textContent = `${data.delay.hilbert_delay_samples.toFixed(2)} muestras`;
    elements.delayTotal.textContent = `${data.delay.total_delay_samples.toFixed(2)} muestras`;
    elements.delayMs.textContent = `${data.delay.total_delay_ms.toFixed(2)} ms`;
    elements.delayRecommended.textContent = `${data.delay.recommended_prediction_delay} muestras`;

    // Log de coeficientes
    logMessage('Coeficientes IIR Q15:', 'info');
    logMessage(`  b0_q15 = ${data.iir.b0_q15}`, 'info');
    logMessage(`  b1_q15 = ${data.iir.b1_q15}`, 'info');
    logMessage(`  b2_q15 = ${data.iir.b2_q15}`, 'info');
    logMessage(`  a1_q15 = ${data.iir.a1_q15}`, 'info');
    logMessage(`  a2_q15 = ${data.iir.a2_q15}`, 'info');
    logMessage(`Retraso recomendado: ${data.delay.recommended_prediction_delay} muestras`, 'info');
}

async function sendToArduino() {
    const params = getCurrentParameters();

    try {
        elements.sendBtn.disabled = true;
        elements.sendBtn.innerHTML = '<div class="spinner"></div><span class="btn-text">Enviando...</span>';

        const response = await fetch(`${API_BASE}/api/send-to-arduino`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(params)
        });

        const data = await response.json();

        if (data.success) {
            logMessage(data.message, 'success');
        } else {
            logMessage(data.message, 'error');
        }
    } catch (error) {
        logMessage(`Error: ${error.message}`, 'error');
    } finally {
        elements.sendBtn.disabled = false;
        elements.sendBtn.innerHTML = '<span class="icon">📤</span><span class="btn-text">Enviar a Arduino</span>';
    }
}

async function resetArduino() {
    if (!confirm('¿Desea restaurar los valores por defecto (21 Hz) en el Arduino?')) {
        return;
    }

    try {
        elements.resetArduinoBtn.disabled = true;
        elements.resetArduinoBtn.innerHTML = '<div class="spinner"></div><span class="btn-text">Restaurando...</span>';

        const response = await fetch(`${API_BASE}/api/reset-arduino`, {
            method: 'POST'
        });

        const data = await response.json();

        if (data.success) {
            logMessage(data.message, 'success');
        } else {
            logMessage(data.message, 'error');
        }
    } catch (error) {
        logMessage(`Error: ${error.message}`, 'error');
    } finally {
        elements.resetArduinoBtn.disabled = false;
        elements.resetArduinoBtn.innerHTML = '<span class="icon">⚠️</span><span class="btn-text">Restaurar Arduino</span>';
    }
}

// ===== PLOTTING FUNCTIONS =====
function initializePlot() {
    // Magnitude plot
    const magnitudeLayout = {
        title: {
            text: 'Magnitud',
            font: { color: '#f1f5f9', size: 16 }
        },
        xaxis: {
            title: 'Frecuencia (Hz)',
            gridcolor: '#334155',
            color: '#cbd5e1',
            range: [0, 100]
        },
        yaxis: {
            title: 'Magnitud (dB)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        paper_bgcolor: '#1e293b',
        plot_bgcolor: '#334155',
        font: { color: '#cbd5e1' },
        margin: { t: 50, r: 40, b: 50, l: 60 }
    };

    const magnitudeData = [{
        x: [],
        y: [],
        type: 'scatter',
        mode: 'lines',
        line: {
            color: '#667eea',
            width: 2
        },
        name: 'Magnitud'
    }];

    Plotly.newPlot('magnitude-plot', magnitudeData, magnitudeLayout, {
        responsive: true,
        displayModeBar: false
    });

    // Phase plot
    const phaseLayout = {
        title: {
            text: 'Fase',
            font: { color: '#f1f5f9', size: 16 }
        },
        xaxis: {
            title: 'Frecuencia (Hz)',
            gridcolor: '#334155',
            color: '#cbd5e1',
            range: [0, 100]
        },
        yaxis: {
            title: 'Fase (grados)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        paper_bgcolor: '#1e293b',
        plot_bgcolor: '#334155',
        font: { color: '#cbd5e1' },
        margin: { t: 50, r: 40, b: 50, l: 60 }
    };

    const phaseData = [{
        x: [],
        y: [],
        type: 'scatter',
        mode: 'lines',
        line: {
            color: '#f59e0b',
            width: 2
        },
        name: 'Fase'
    }];

    Plotly.newPlot('phase-plot', phaseData, phaseLayout, {
        responsive: true,
        displayModeBar: false
    });
}

function updatePlot(plotData) {
    const centerFreq = parseFloat(elements.centerFreq.value);

    // Update magnitude plot
    const magnitudeData = [{
        x: plotData.frequency,
        y: plotData.magnitude_db,
        type: 'scatter',
        mode: 'lines',
        line: {
            color: '#667eea',
            width: 2
        },
        name: 'Magnitud'
    }];

    const magnitudeLayout = {
        title: {
            text: 'Magnitud',
            font: { color: '#f1f5f9', size: 16 }
        },
        xaxis: {
            title: 'Frecuencia (Hz)',
            gridcolor: '#334155',
            color: '#cbd5e1',
            range: [0, 100]
        },
        yaxis: {
            title: 'Magnitud (dB)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        paper_bgcolor: '#1e293b',
        plot_bgcolor: '#334155',
        font: { color: '#cbd5e1' },
        margin: { t: 50, r: 40, b: 50, l: 60 },
        shapes: [{
            type: 'line',
            x0: centerFreq,
            y0: 0,
            x1: centerFreq,
            y1: 1,
            yref: 'paper',
            line: {
                color: '#f56565',
                width: 2,
                dash: 'dash'
            }
        }],
        annotations: [{
            x: centerFreq,
            y: 0.95,
            yref: 'paper',
            text: `Centro: ${centerFreq} Hz`,
            showarrow: false,
            font: { color: '#f56565', size: 12 },
            bgcolor: '#1e293b',
            bordercolor: '#f56565',
            borderwidth: 1,
            borderpad: 4
        }]
    };

    Plotly.react('magnitude-plot', magnitudeData, magnitudeLayout, {
        responsive: true,
        displayModeBar: false
    });

    // Update phase plot
    const phaseData = [{
        x: plotData.frequency,
        y: plotData.phase_degrees,
        type: 'scatter',
        mode: 'lines',
        line: {
            color: '#f59e0b',
            width: 2
        },
        name: 'Fase'
    }];

    const phaseLayout = {
        title: {
            text: 'Fase',
            font: { color: '#f1f5f9', size: 16 }
        },
        xaxis: {
            title: 'Frecuencia (Hz)',
            gridcolor: '#334155',
            color: '#cbd5e1',
            range: [0, 100]
        },
        yaxis: {
            title: 'Fase (grados)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        paper_bgcolor: '#1e293b',
        plot_bgcolor: '#334155',
        font: { color: '#cbd5e1' },
        margin: { t: 50, r: 40, b: 50, l: 60 },
        shapes: [{
            type: 'line',
            x0: centerFreq,
            y0: 0,
            x1: centerFreq,
            y1: 1,
            yref: 'paper',
            line: {
                color: '#f56565',
                width: 2,
                dash: 'dash'
            }
        }],
        annotations: [{
            x: centerFreq,
            y: 0.95,
            yref: 'paper',
            text: `Centro: ${centerFreq} Hz`,
            showarrow: false,
            font: { color: '#f56565', size: 12 },
            bgcolor: '#1e293b',
            bordercolor: '#f56565',
            borderwidth: 1,
            borderpad: 4
        }]
    };

    Plotly.react('phase-plot', phaseData, phaseLayout, {
        responsive: true,
        displayModeBar: false
    });
}

// ===== CONSOLE FUNCTIONS =====
function logMessage(message, level = 'info') {
    const timestamp = new Date().toLocaleTimeString('es-ES', {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });

    const line = document.createElement('div');
    line.className = `console-line log-${level}`;
    line.innerHTML = `<span class="console-timestamp">[${timestamp}]</span> ${message}`;

    elements.console.appendChild(line);
    elements.console.scrollTop = elements.console.scrollHeight;

    // Limitar el número de líneas
    while (elements.console.children.length > 100) {
        elements.console.removeChild(elements.console.firstChild);
    }
}

function clearConsole() {
    elements.console.innerHTML = '';
    logMessage('Consola limpiada', 'info');
}

// ===== REAL-TIME VISUALIZATION FUNCTIONS =====

function initializeRealtimePlots() {
    const commonLayout = {
        xaxis: {
            title: 'Tiempo (s)',
            gridcolor: '#334155',
            color: '#cbd5e1',
            range: [0, WINDOW_TIME]
        },
        paper_bgcolor: '#1e293b',
        plot_bgcolor: '#334155',
        font: { color: '#cbd5e1' },
        margin: { t: 30, r: 40, b: 50, l: 60 },
        showlegend: false
    };

    // Plot 1: Voltage (with pulse markers)
    const voltageLayout = {
        ...commonLayout,
        yaxis: {
            title: 'Voltaje (V)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        showlegend: true,
        legend: {
            x: 0.02,
            y: 0.98,
            bgcolor: 'rgba(30, 41, 59, 0.8)',
            bordercolor: '#667eea',
            borderwidth: 1
        }
    };

    const voltageData = [
        {
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Voltaje',
            line: { color: '#48bb78', width: 1.5 }
        },
        {
            x: [],
            y: [],
            type: 'scatter',
            mode: 'markers',
            name: 'Pulsos detectados',
            marker: {
                color: '#ef4444',
                size: 10,
                symbol: 'diamond'
            }
        }
    ];

    Plotly.newPlot('voltage-plot', voltageData, voltageLayout, {
        responsive: true,
        displayModeBar: false
    });

    // Plot 2: Filtered signal
    const filteredLayout = {
        ...commonLayout,
        yaxis: {
            title: 'Señal Filtrada',
            gridcolor: '#334155',
            color: '#cbd5e1'
        }
    };

    const filteredData = [
        {
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Filtrada',
            line: { color: '#667eea', width: 1.5 }
        }
    ];

    Plotly.newPlot('filtered-plot', filteredData, filteredLayout, {
        responsive: true,
        displayModeBar: false
    });

    // Plot 3: Magnitude
    const magnitudeLayout = {
        ...commonLayout,
        yaxis: {
            title: 'Magnitud (Voltios)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        shapes: [{
            type: 'line',
            x0: 0,
            x1: 1,
            xref: 'paper',
            y0: parseFloat(elements.magnitudeThreshold.value) * (5.0 / 1.024),  // Convertir normalizado a voltios
            y1: parseFloat(elements.magnitudeThreshold.value) * (5.0 / 1.024),
            line: {
                color: '#ef4444',
                width: 2,
                dash: 'dash'
            }
        }],
        annotations: [{
            x: 0.98,
            xref: 'paper',
            y: parseFloat(elements.magnitudeThreshold.value) * (5.0 / 1.024),
            text: 'Umbral',
            showarrow: false,
            font: { color: '#ef4444', size: 10 },
            xanchor: 'right',
            bgcolor: 'rgba(30, 41, 59, 0.8)',
            bordercolor: '#ef4444',
            borderwidth: 1,
            borderpad: 2
        }]
    };

    const magnitudeData = [
        {
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Magnitud',
            line: { color: '#f093fb', width: 2 }
        }
    ];

    Plotly.newPlot('magnitude-stream-plot', magnitudeData, magnitudeLayout, {
        responsive: true,
        displayModeBar: false
    });

    // Plot 4: Phase
    const phaseThresholdDegrees = parseFloat(elements.phaseThreshold.value);
    const phaseLayout = {
        ...commonLayout,
        yaxis: {
            title: 'Fase (grados)',
            gridcolor: '#334155',
            color: '#cbd5e1'
        },
        shapes: [
            {
                type: 'line',
                x0: 0,
                x1: 1,
                xref: 'paper',
                y0: 180 - phaseThresholdDegrees,
                y1: 180 - phaseThresholdDegrees,
                line: {
                    color: '#ef4444',
                    width: 2,
                    dash: 'dash'
                }
            },
            {
                type: 'line',
                x0: 0,
                x1: 1,
                xref: 'paper',
                y0: -(180 - phaseThresholdDegrees),
                y1: -(180 - phaseThresholdDegrees),
                line: {
                    color: '#ef4444',
                    width: 2,
                    dash: 'dash'
                }
            }
        ],
        annotations: [
            {
                x: 0.98,
                xref: 'paper',
                y: 180 - phaseThresholdDegrees,
                text: 'Umbral',
                showarrow: false,
                font: { color: '#ef4444', size: 10 },
                xanchor: 'right',
                bgcolor: 'rgba(30, 41, 59, 0.8)',
                bordercolor: '#ef4444',
                borderwidth: 1,
                borderpad: 2
            },
            {
                x: 0.98,
                xref: 'paper',
                y: -(180 - phaseThresholdDegrees),
                text: 'Umbral',
                showarrow: false,
                font: { color: '#ef4444', size: 10 },
                xanchor: 'right',
                bgcolor: 'rgba(30, 41, 59, 0.8)',
                bordercolor: '#ef4444',
                borderwidth: 1,
                borderpad: 2
            }
        ]
    };

    const phaseData = [
        {
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines',
            name: 'Fase',
            line: { color: '#f59e0b', width: 1.5 }
        }
    ];

    Plotly.newPlot('phase-stream-plot', phaseData, phaseLayout, {
        responsive: true,
        displayModeBar: false
    });
}

function toggleDataStream() {
    if (!isStreamingActive) {
        startDataStream();
    } else {
        stopDataStream();
    }
}

function startDataStream() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/data`;

    wsData = new WebSocket(wsUrl);
    let startTime = null;

    wsData.onopen = () => {
        isStreamingActive = true;
        elements.toggleStreamBtn.innerHTML = '<span class="icon">⏸️</span><span class="btn-text">Detener Stream</span>';
        elements.toggleStreamBtn.classList.remove('btn-primary');
        elements.toggleStreamBtn.classList.add('btn-warning');
        logMessage('Stream de datos iniciado (200 Hz)', 'success');

        // Inicializar gráficas
        initializeRealtimePlots();

        // Reset buffers
        dataBuffers.time = [];
        dataBuffers.voltage = [];
        dataBuffers.filtered = [];
        dataBuffers.magnitude = [];
        dataBuffers.phase = [];
        dataBuffers.pulseTime = [];
        dataBuffers.pulseVoltage = [];

        // Reset pulse edge detection
        wasInPulse = false;
    };

    wsData.onmessage = (event) => {
        const msg = JSON.parse(event.data);

        if (msg.type === 'data') {
            // Establecer tiempo inicial
            if (startTime === null) {
                startTime = msg.timestamp;
            }

            const relativeTime = msg.timestamp - startTime;

            // Agregar a buffers
            dataBuffers.time.push(relativeTime);
            dataBuffers.voltage.push(msg.voltage);
            dataBuffers.filtered.push(msg.filtered);
            dataBuffers.magnitude.push(msg.magnitude);
            dataBuffers.phase.push(msg.phase_degrees);

            // Detectar pulsos usando el flag del Arduino (sincronización real)
            // El Arduino envía pulse_active=true cuando hay un pulso físico activo
            if (msg.pulse_active && !wasInPulse) {
                // Flanco ascendente del pulso - agregar marcador
                dataBuffers.pulseTime.push(relativeTime);
                dataBuffers.pulseVoltage.push(msg.voltage);
            }

            // Actualizar estado para siguiente iteración
            wasInPulse = msg.pulse_active;

            // Mantener solo los últimos MAX_POINTS
            if (dataBuffers.time.length > MAX_POINTS) {
                dataBuffers.time.shift();
                dataBuffers.voltage.shift();
                dataBuffers.filtered.shift();
                dataBuffers.magnitude.shift();
                dataBuffers.phase.shift();
            }

            // Limpiar pulsos antiguos (fuera de la ventana de tiempo)
            const windowStart = relativeTime - WINDOW_TIME;
            while (dataBuffers.pulseTime.length > 0 && dataBuffers.pulseTime[0] < windowStart) {
                dataBuffers.pulseTime.shift();
                dataBuffers.pulseVoltage.shift();
            }

            // Actualizar gráfica (throttled - cada 50ms aprox)
            updateRealtimePlot();
        }
    };

    wsData.onerror = (error) => {
        console.error('WebSocket data error:', error);
        logMessage('Error en el stream de datos', 'error');
        stopDataStream();
    };

    wsData.onclose = () => {
        if (isStreamingActive) {
            logMessage('Stream de datos cerrado', 'warning');
            stopDataStream();
        }
    };
}

function stopDataStream() {
    if (wsData) {
        wsData.close();
        wsData = null;
    }

    isStreamingActive = false;
    elements.toggleStreamBtn.innerHTML = '<span class="icon">▶️</span><span class="btn-text">Iniciar Stream</span>';
    elements.toggleStreamBtn.classList.remove('btn-warning');
    elements.toggleStreamBtn.classList.add('btn-primary');
    logMessage('Stream de datos detenido', 'info');
}

let lastUpdateTime = 0;
function updateRealtimePlot() {
    const now = Date.now();
    if (now - lastUpdateTime < 50) return; // Actualizar máximo cada 50ms
    lastUpdateTime = now;

    if (dataBuffers.time.length === 0) return;

    const maxTime = Math.max(...dataBuffers.time);
    const minTime = Math.max(0, maxTime - WINDOW_TIME); // Ventana configurable
    const xAxisRange = [minTime, maxTime];

    // Update voltage plot (with pulse markers)
    Plotly.update('voltage-plot',
        {
            x: [dataBuffers.time, dataBuffers.pulseTime],
            y: [dataBuffers.voltage, dataBuffers.pulseVoltage]
        },
        {
            'xaxis.range': xAxisRange
        }
    );

    // Update filtered plot
    Plotly.update('filtered-plot',
        {
            x: [dataBuffers.time],
            y: [dataBuffers.filtered]
        },
        {
            'xaxis.range': xAxisRange
        }
    );

    // Update magnitude plot
    Plotly.update('magnitude-stream-plot',
        {
            x: [dataBuffers.time],
            y: [dataBuffers.magnitude]
        },
        {
            'xaxis.range': xAxisRange
        }
    );

    // Update phase plot
    Plotly.update('phase-stream-plot',
        {
            x: [dataBuffers.time],
            y: [dataBuffers.phase]
        },
        {
            'xaxis.range': xAxisRange
        }
    );
}
