var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

window.addEventListener('load', onLoad);

function onLoad(event) {
    initWebSocket();
}

function initWebSocket() {
    websocket = new WebSocket(gateway);
    websocket.onmessage = onMessage;
}

function onMessage(event) {
    var data = JSON.parse(event.data);
    
    // Actualizar Yaw (BNO055)
    document.getElementById('yaw').innerHTML = data.yaw.toFixed(1);
    
    // Actualizar Color
    var colorBox = document.getElementById('color-indicator');
    colorBox.style.backgroundColor = `rgb(${data.r}, ${data.g}, ${data.b})`;
    document.getElementById('color-name').innerHTML = data.colorName;
    
    // Actualizar PWM
    document.getElementById('pwm').innerHTML = data.pwm;
}

function calibrar() {
    websocket.send("calibrar"); // Envía orden al ESP32 para calibrar blanco
}