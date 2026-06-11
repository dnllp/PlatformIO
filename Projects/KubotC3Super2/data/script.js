/**
 * script.js — Dashboard Kubot C3 Pro
 * ====================================
 * Gestiona la conexión WebSocket con el ESP32 y actualiza la UI en tiempo real.
 *
 * MEJORAS RESPECTO A LA VERSIÓN ANTERIOR:
 *   1. Reconexión automática con backoff (no pierde la sesión si el robot reinicia)
 *   2. Indicador visual de estado de conexión (verde/rojo/amarillo)
 *   3. Animación de brújula sincronizada con el Yaw del BNO055
 *   4. Gauge circular animado para el PWM
 *   5. Barras RGB animadas con valores numéricos
 *   6. Log de eventos con timestamp
 *   7. Reloj en tiempo real en el footer
 *   8. Manejo de errores JSON y datos faltantes
 */

// ─────────────────────────────────────────────────────────────────────────────
// CONFIGURACIÓN
// ─────────────────────────────────────────────────────────────────────────────

/** URL del WebSocket — usa el mismo host que sirvió la página (el ESP32) */
const GATEWAY = `ws://${window.location.hostname}/ws`;

/** Tiempo entre intentos de reconexión (ms). Aumenta con cada fallo. */
const RECONNECT_BASE_MS  = 2000;
const RECONNECT_MAX_MS   = 15000;

// ─────────────────────────────────────────────────────────────────────────────
// ESTADO INTERNO
// ─────────────────────────────────────────────────────────────────────────────

let websocket       = null;
let reconnectDelay  = RECONNECT_BASE_MS;
let reconnectTimer  = null;

// Circunferencia del gauge SVG (2π × r=50 ≈ 314.16; arco útil = 251.2 para 80%)
const GAUGE_CIRC = 251.2;

// ─────────────────────────────────────────────────────────────────────────────
// INICIALIZACIÓN
// ─────────────────────────────────────────────────────────────────────────────

window.addEventListener('load', () => {
    iniciarWebSocket();
    iniciarReloj();
});

// ─────────────────────────────────────────────────────────────────────────────
// WEBSOCKET
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Crea la conexión WebSocket y asigna los manejadores de evento.
 * Si la conexión falla o se cierra, programa un reintento automático.
 */
function iniciarWebSocket() {
    // Cancelar cualquier timer pendiente antes de intentar de nuevo
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }

    log('Intentando conectar al robot...', 'info');
    setWsStatus('connecting');

    try {
        websocket = new WebSocket(GATEWAY);
    } catch (e) {
        log(`Error al crear WebSocket: ${e.message}`, 'error');
        programarReconexion();
        return;
    }

    // ── Conexión establecida ──
    websocket.onopen = () => {
        reconnectDelay = RECONNECT_BASE_MS; // Resetear backoff
        setWsStatus('connected');
        log('Conexión WebSocket establecida con el robot.', 'ok');
    };

    // ── Mensaje recibido desde el ESP32 ──
    websocket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            actualizarUI(data);
        } catch (e) {
            log(`JSON inválido recibido: ${event.data}`, 'error');
        }
    };

    // ── Conexión cerrada (normal o inesperada) ──
    websocket.onclose = (event) => {
        setWsStatus('disconnected');
        const razon = event.wasClean
            ? `Cierre limpio (código ${event.code})`
            : 'Conexión interrumpida';
        log(`WebSocket cerrado — ${razon}. Reintentando en ${reconnectDelay / 1000}s...`, 'warn');
        programarReconexion();
    };

    // ── Error de red ──
    websocket.onerror = () => {
        // onerror siempre va seguido de onclose, así que no programamos
        // reconexión aquí para evitar duplicados.
        log('Error de red en WebSocket.', 'error');
    };
}

/**
 * Programa un reintento de conexión con backoff exponencial.
 * Duplica el delay en cada fallo hasta RECONNECT_MAX_MS.
 */
function programarReconexion() {
    reconnectTimer = setTimeout(() => {
        iniciarWebSocket();
    }, reconnectDelay);

    // Backoff: 2s → 4s → 8s → ... → 15s máximo
    reconnectDelay = Math.min(reconnectDelay * 2, RECONNECT_MAX_MS);
}

/**
 * Envía un comando de texto al ESP32 por WebSocket.
 * Verifica que la conexión esté abierta antes de enviar.
 * @param {string} cmd - Comando a enviar (e.g., "calibrar", "stop")
 */
function enviarComando(cmd) {
    if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(cmd);
        log(`Comando enviado: "${cmd}"`, 'info');
    } else {
        log('No se puede enviar: sin conexión con el robot.', 'warn');
    }
}

/** Alias para el botón de calibración en el HTML */
function calibrar() {
    enviarComando('calibrar');
}

// ─────────────────────────────────────────────────────────────────────────────
// ACTUALIZACIÓN DE LA INTERFAZ
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Recibe el objeto JSON parseado del ESP32 y actualiza todos los widgets.
 * Usa valores por defecto (0 / "---") si un campo no llega para evitar NaN.
 *
 * Campos esperados del ESP32:
 *   { yaw, r, g, b, lux, colorName, pwm }
 */
function actualizarUI(data) {
    const yaw       = data.yaw       ?? 0;
    const r         = data.r         ?? 0;
    const g         = data.g         ?? 0;
    const b         = data.b         ?? 0;
    const colorName = data.colorName ?? '---';
    const pwm       = data.pwm       ?? 0;

    // ── 1. Orientación / Brújula ──────────────────────────────────────────
    document.getElementById('yaw').textContent = yaw.toFixed(1);
    // La aguja de la brújula rota en sentido contrario al heading
    // (si el robot apunta 90°E, la aguja apunta al Norte relativo)
    document.getElementById('compass-needle').style.transform =
        `translate(-50%, -100%) rotate(${yaw}deg)`;

    // ── 2. Color ──────────────────────────────────────────────────────────
    const colorBox = document.getElementById('color-indicator');
    colorBox.style.backgroundColor = `rgb(${r}, ${g}, ${b})`;

    // Elegir texto oscuro o claro según luminosidad del color detectado
    const lum = 0.299 * r + 0.587 * g + 0.114 * b;
    colorBox.style.color = lum > 128 ? '#111' : '#fff';

    document.getElementById('color-name').textContent = colorName;

    // Chip de color cambia de clase según el color detectado
    const chip = document.getElementById('color-chip');
    chip.className = 'chip'; // reset
    if (colorName.includes('ROJO'))  chip.classList.add('chip-red');
    if (colorName.includes('VERDE')) chip.classList.add('chip-green');

    // Barras RGB (r/g/b vienen en 0–255 desde el ESP32)
    actualizarBarra('bar-r', 'val-r', r);
    actualizarBarra('bar-g', 'val-g', g);
    actualizarBarra('bar-b', 'val-b', b);

    // ── 3. PWM / Gauge ────────────────────────────────────────────────────
    document.getElementById('pwm').textContent = pwm;

    // El arco del gauge ocupa (pwm/255) de la circunferencia
    const offset = GAUGE_CIRC - (pwm / 255) * GAUGE_CIRC;
    document.getElementById('gauge-arc').style.strokeDashoffset = offset;

    // Cambiar color del gauge según velocidad
    const arc = document.getElementById('gauge-arc');
    if (pwm === 0)       arc.style.stroke = 'var(--color-stop)';
    else if (pwm < 100)  arc.style.stroke = 'var(--color-slow)';
    else                 arc.style.stroke = 'var(--color-fast)';
}

/**
 * Actualiza una barra de progreso RGB y su etiqueta numérica.
 * @param {string} barId  - ID del elemento .bar-fill
 * @param {string} valId  - ID del span de valor
 * @param {number} value  - Valor 0–255
 */
function actualizarBarra(barId, valId, value) {
    const pct = Math.round((value / 255) * 100);
    document.getElementById(barId).style.width = `${pct}%`;
    document.getElementById(valId).textContent  = value;
}

// ─────────────────────────────────────────────────────────────────────────────
// ESTADO DE CONEXIÓN (badge en el header)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Actualiza el badge de estado WebSocket en el header.
 * @param {'connecting'|'connected'|'disconnected'} estado
 */
function setWsStatus(estado) {
    const dot   = document.getElementById('ws-dot');
    const label = document.getElementById('ws-label');
    const badge = document.getElementById('ws-badge');

    dot.className   = 'ws-dot';       // reset clases
    badge.className = 'ws-badge';     // reset clases

    switch (estado) {
        case 'connected':
            dot.classList.add('dot-on');
            label.textContent = 'Conectado';
            break;
        case 'connecting':
            dot.classList.add('dot-wait');
            label.textContent = 'Conectando...';
            break;
        case 'disconnected':
        default:
            dot.classList.add('dot-off');
            label.textContent = 'Sin conexión';
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LOG DE EVENTOS
// ─────────────────────────────────────────────────────────────────────────────

const MAX_LOG_LINES = 40; // Número máximo de líneas antes de hacer scroll

/**
 * Agrega una línea al log de eventos con timestamp.
 * @param {string} mensaje
 * @param {'info'|'ok'|'warn'|'error'} tipo
 */
function log(mensaje, tipo = 'info') {
    const box = document.getElementById('log-box');
    if (!box) return;

    const now   = new Date();
    const ts    = now.toLocaleTimeString('es-MX', { hour12: false });
    const linea = document.createElement('div');
    linea.className = `log-line log-${tipo}`;
    linea.innerHTML = `<span class="log-ts">${ts}</span> ${mensaje}`;
    box.appendChild(linea);

    // Mantener scroll al fondo automáticamente
    box.scrollTop = box.scrollHeight;

    // Limitar número de líneas para no acumular memoria en el navegador
    while (box.children.length > MAX_LOG_LINES) {
        box.removeChild(box.firstChild);
    }
}

/** Vacía el log de eventos */
function limpiarLog() {
    const box = document.getElementById('log-box');
    if (box) box.innerHTML = '';
    log('Log limpiado.', 'info');
}

// ─────────────────────────────────────────────────────────────────────────────
// RELOJ EN TIEMPO REAL
// ─────────────────────────────────────────────────────────────────────────────

function iniciarReloj() {
    const el = document.getElementById('clock');
    if (!el) return;

    const tick = () => {
        el.textContent = new Date().toLocaleTimeString('es-MX', { hour12: false });
    };
    tick();
    setInterval(tick, 1000);
}
