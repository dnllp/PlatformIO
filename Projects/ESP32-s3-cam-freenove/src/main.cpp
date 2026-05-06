/*
 * ============================================================
 *  Freenove ESP32-S3 WROOM — Streaming MJPEG por WiFi
 * ============================================================
 *
 * Qué hace este programa:
 *   1. Inicializa el sensor de cámara OV2640 con el pinout
 *      específico de la Freenove ESP32-S3 WROOM
 *   2. Enciende el LED flash integrado mientras corre el stream
 *   3. Se conecta a una red WiFi
 *   4. Levanta un servidor HTTP en el puerto 80
 *   5. Sirve una página web en "/"  con el video en vivo
 *   6. Transmite el video en "/stream" usando MJPEG
 *
 * Diferencias clave vs otras ESP32-S3-CAM:
 *   - El pinout del OV2640 es exclusivo de esta placa (ver abajo)
 *   - Tiene LED flash en GPIO 2 (controlable por software)
 *   - La PSRAM es tipo OPI (Octal SPI), más rápida que la QPI
 *   - Tiene botón BOOT en GPIO 0 y botón de usuario en GPIO 1
 *
 * Cómo verlo:
 *   Abre un navegador en la misma red WiFi y ve a:
 *   http://<IP que aparece en el monitor serial>
 * ============================================================
 */

#include "Arduino.h"
#include "esp_camera.h"      // Driver oficial Espressif para OV2640
#include "WiFi.h"            // Conexión a redes WiFi
#include "esp_http_server.h" // Servidor HTTP liviano del SDK ESP-IDF


// ─────────────────────────────────────────────────────────────────
//  CONFIGURACIÓN WiFi
// ─────────────────────────────────────────────────────────────────
const char* SSID     = "UAM-ROBOTICA";
const char* PASSWORD = "m4nt32024uat";


// ─────────────────────────────────────────────────────────────────
//  LED FLASH INTEGRADO
//
//  La Freenove ESP32-S3 WROOM tiene un LED blanco de alto brillo
//  en GPIO 2, orientado hacia la misma dirección que la cámara.
//  Se puede usar como flash para mejorar la iluminación.
//  HIGH = encendido, LOW = apagado.
// ─────────────────────────────────────────────────────────────────
#define LED_FLASH_PIN 2


// ─────────────────────────────────────────────────────────────────
//  PINOUT — Freenove ESP32-S3 WROOM
//
//  ¡IMPORTANTE! Este pinout es EXCLUSIVO de la Freenove WROOM.
//  No uses el pinout de la AI-Thinker, Goouuu u otras placas
//  porque el OV2640 no funcionará o se dañará el hardware.
//
//  La cámara OV2640 usa el bus paralelo DVP (Digital Video Port):
//    D0–D7  : 8 líneas de datos de imagen (píxeles en paralelo)
//    XCLK   : reloj que genera el ESP32 para la cámara (20 MHz)
//    PCLK   : reloj de píxel que la cámara genera para sincronizar
//    VSYNC  : pulso que indica inicio/fin de un frame completo
//    HREF   : pulso que indica inicio/fin de una línea horizontal
//    SIOD   : datos I2C (SDA) para configurar registros del OV2640
//    SIOC   : reloj I2C (SCL) para configurar registros del OV2640
//    PWDN   : "power down" — apaga el sensor (-1 = no disponible)
//    RESET  : reinicia el sensor (-1 = no disponible en esta placa)
// ─────────────────────────────────────────────────────────────────
#define CAM_PIN_PWDN    -1   // No disponible en la Freenove WROOM
#define CAM_PIN_RESET   -1   // No disponible en la Freenove WROOM
#define CAM_PIN_XCLK    15   // Reloj maestro hacia la cámara
#define CAM_PIN_SIOD    4    // I2C datos (SDA) — bus SCCB del OV2640
#define CAM_PIN_SIOC    5    // I2C reloj (SCL) — bus SCCB del OV2640
#define CAM_PIN_D7      16   // Bus de datos bit 7 (más significativo)
#define CAM_PIN_D6      17   // Bus de datos bit 6
#define CAM_PIN_D5      18   // Bus de datos bit 5
#define CAM_PIN_D4      12   // Bus de datos bit 4
#define CAM_PIN_D3      10   // Bus de datos bit 3
#define CAM_PIN_D2      8    // Bus de datos bit 2
#define CAM_PIN_D1      9    // Bus de datos bit 1
#define CAM_PIN_D0      11   // Bus de datos bit 0 (menos significativo)
#define CAM_PIN_VSYNC   6    // Sincronización vertical (por frame)
#define CAM_PIN_HREF    7    // Sincronización horizontal (por línea)
#define CAM_PIN_PCLK    13   // Reloj de píxel generado por la cámara


// ─────────────────────────────────────────────────────────────────
//  FUNCIÓN: initCamera()
//
//  Configura e inicializa el driver de la cámara OV2640.
//  Usa PSRAM si está disponible para mayor resolución y fluidez.
//  Retorna true si tuvo éxito, false si ocurrió algún error.
// ─────────────────────────────────────────────────────────────────
bool initCamera() {

    // camera_config_t agrupa toda la configuración del hardware:
    // pines, frecuencias, formato de imagen y tamaño de frame.
    camera_config_t config;

    // --- Generador de reloj LEDC ---
    // El periférico LEDC (LED Control) del ESP32 genera la señal
    // XCLK que la cámara necesita para operar. Si tu proyecto
    // usa otros canales LEDC (para PWM, por ejemplo), asegúrate
    // de no usar el canal 0 y el timer 0 para otra cosa.
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    // --- Pines de datos del bus DVP ---
    config.pin_d0 = CAM_PIN_D0;
    config.pin_d1 = CAM_PIN_D1;
    config.pin_d2 = CAM_PIN_D2;
    config.pin_d3 = CAM_PIN_D3;
    config.pin_d4 = CAM_PIN_D4;
    config.pin_d5 = CAM_PIN_D5;
    config.pin_d6 = CAM_PIN_D6;
    config.pin_d7 = CAM_PIN_D7;

    // --- Pines de control y sincronización ---
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD; // SDA del bus I2C/SCCB
    config.pin_sccb_scl = CAM_PIN_SIOC; // SCL del bus I2C/SCCB
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;

    // --- Frecuencia del reloj XCLK ---
    // 20 MHz es el valor óptimo para el OV2640 según su datasheet.
    // Frecuencias más altas pueden aumentar los FPS pero generan
    // ruido eléctrico; más bajas reducen la calidad de imagen.
    config.xclk_freq_hz = 20000000; // 20 MHz

    // --- Formato de pixel: JPEG comprimido ---
    // El OV2640 puede comprimir internamente cada frame a JPEG
    // antes de enviarlo al ESP32 por el bus DVP. Esto reduce
    // enormemente los datos a procesar y transmitir por WiFi.
    // Para visión artificial usa PIXFORMAT_RGB565 o YUV422,
    // pero para streaming PIXFORMAT_JPEG es siempre la mejor opción.
    config.pixel_format = PIXFORMAT_JPEG;

    // --- Resolución y buffers según disponibilidad de PSRAM ---
    // La Freenove WROOM tiene 8 MB de PSRAM tipo OPI (Octal SPI),
    // que es más rápida que la PSRAM estándar de otras placas.
    // Con ella podemos usar resoluciones altas y doble buffer.
    //
    // Doble buffer (fb_count = 2):
    //   Mientras el buffer A se envía por WiFi, la cámara ya está
    //   llenando el buffer B con el siguiente frame. Esto aumenta
    //   los FPS y evita que el stream se congele esperando la red.
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_UXGA;  // 1600x1200 — máxima del OV2640
        config.jpeg_quality = 10;              // 0=mejor calidad, 63=peor
        config.fb_count     = 2;              // Doble buffer para mayor fluidez
        config.fb_location  = CAMERA_FB_IN_PSRAM; // Guardar frames en PSRAM
    } else {
        // Sin PSRAM: resolución baja en RAM interna (solo 512 KB disponibles)
        config.frame_size   = FRAMESIZE_SVGA;  // 800x600
        config.jpeg_quality = 12;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM; // Guardar en RAM interna
    }

    // --- Inicializar el driver de cámara ---
    // Internamente configura el DMA para transferir datos del bus DVP
    // a memoria, programa los registros del OV2640 vía I2C,
    // y arranca el periférico LEDC para generar el XCLK.
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error al iniciar camara: 0x%x\n", err);
        // Error 0x20005 (ESP_ERR_NOT_FOUND): cámara no detectada en I2C
        //   → Verifica los pines SIOD y SIOC
        // Error 0x20001 (ESP_ERR_NO_MEM): sin memoria suficiente
        //   → Verifica que PSRAM esté habilitada en platformio.ini
        return false;
    }

    // --- Ajustes del sensor post-inicialización ---
    // sensor_t da acceso a los controles de imagen del OV2640
    // a través de funciones de alto nivel (sin escribir registros directamente)
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        // Voltear imagen verticalmente si la cámara está montada al revés
        // 0 = orientación normal, 1 = volteado
        s->set_vflip(s, 0);

        // Voltear imagen horizontalmente (efecto espejo)
        s->set_hmirror(s, 0);

        // Brillo: -2 a +2 (0 = neutral)
        s->set_brightness(s, 0);

        // Contraste: -2 a +2 (0 = neutral)
        s->set_contrast(s, 0);

        // Saturación de color: -2 a +2 (0 = neutral)
        s->set_saturation(s, 0);

        // Balance de blancos automático: 1 = activado
        s->set_whitebal(s, 1);

        // Ganancia automática (Auto Gain Control): 1 = activado
        // Ajusta la sensibilidad según la iluminación disponible
        s->set_gain_ctrl(s, 1);

        // Exposición automática (AEC): 1 = activado
        // Ajusta el tiempo de exposición según la iluminación
        s->set_exposure_ctrl(s, 1);
    }

    return true;
}


// ─────────────────────────────────────────────────────────────────
//  HANDLER: página principal  "/"
//
//  Responde con una página HTML que muestra el stream de video.
//  También incluye un botón para controlar el LED flash.
// ─────────────────────────────────────────────────────────────────
static esp_err_t indexHandler(httpd_req_t* req) {

    // Página HTML embebida en el firmware.
    // Incluye un botón de flash que llama a "/flash" vía fetch()
    // sin recargar la página, usando la API fetch de JavaScript.
    const char* html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Freenove ESP32-S3 Cam</title>"
        "<style>"
        "  *{box-sizing:border-box;margin:0;padding:0}"
        "  body{background:#111;color:#eee;font-family:sans-serif;"
        "       display:flex;flex-direction:column;align-items:center;"
        "       justify-content:center;min-height:100vh;gap:16px;padding:16px}"
        "  h2{font-size:1.2rem;font-weight:500}"
        "  img{max-width:100%;border:2px solid #333;border-radius:8px}"
        "  .controls{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}"
        "  button{padding:8px 20px;border:none;border-radius:6px;"
        "         background:#444;color:#eee;cursor:pointer;font-size:14px}"
        "  button:hover{background:#555}"
        "  button.active{background:#e6a817;color:#111}"
        "  select{padding:8px;border-radius:6px;background:#333;"
        "         color:#eee;border:1px solid #555;font-size:14px}"
        "</style></head><body>"
        "<h2>Freenove ESP32-S3 WROOM</h2>"
        // Imagen del stream — el navegador hace una petición GET a /stream
        "<img id='cam' src='/stream'>"
        "<div class='controls'>"
        "  <button id='flash-btn' onclick='toggleFlash()'>Flash OFF</button>"
        "  <select onchange='setRes(this.value)'>"
        "    <option value='vga'>VGA 640x480</option>"
        "    <option value='svga'>SVGA 800x600</option>"
        "    <option value='uxga' selected>UXGA 1600x1200</option>"
        "  </select>"
        "</div>"
        "<script>"
        // toggleFlash(): llama a /flash para cambiar el estado del LED
        "let flashOn=false;"
        "function toggleFlash(){"
        "  flashOn=!flashOn;"
        "  fetch('/flash?state='+flashOn);"   // Petición al servidor
        "  let b=document.getElementById('flash-btn');"
        "  b.textContent='Flash '+(flashOn?'ON':'OFF');"
        "  b.classList.toggle('active',flashOn)"
        "}"
        // setRes(): cambia la resolución recargando el stream
        "function setRes(v){"
        "  let img=document.getElementById('cam');"
        "  img.src='';"                        // Detener stream actual
        "  fetch('/res?size='+v).then(()=>{"
        "    img.src='/stream?t='+Date.now()"  // Recargar stream
        "  })"
        "}"
        "</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}


// ─────────────────────────────────────────────────────────────────
//  HANDLER: control del LED flash  "/flash?state=true|false"
//
//  Recibe el parámetro "state" en la URL y enciende o apaga
//  el LED flash en GPIO 2 según corresponda.
// ─────────────────────────────────────────────────────────────────
static esp_err_t flashHandler(httpd_req_t* req) {

    // Leer la URL completa de la petición para extraer el parámetro
    char query[32];
    char state[8];

    // httpd_req_get_url_query_str() extrae la parte "?state=true"
    // de la URL de la petición
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {

        // httpd_query_key_value() extrae el valor de "state" del query string
        if (httpd_query_key_value(query, "state", state, sizeof(state)) == ESP_OK) {

            // Comparar el valor recibido y actuar en consecuencia
            bool on = (strcmp(state, "true") == 0);
            digitalWrite(LED_FLASH_PIN, on ? HIGH : LOW);
            Serial.printf("Flash LED: %s\n", on ? "ON" : "OFF");
        }
    }

    // Responder con "OK" para que el fetch() de JavaScript no falle
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", 2);
}


// ─────────────────────────────────────────────────────────────────
//  HANDLER: cambio de resolución  "/res?size=vga|svga|uxga"
//
//  Cambia la resolución de la cámara en tiempo de ejecución
//  sin necesidad de reiniciar el ESP32.
// ─────────────────────────────────────────────────────────────────
static esp_err_t resHandler(httpd_req_t* req) {

    char query[32];
    char size[8];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "size", size, sizeof(size)) == ESP_OK) {

            sensor_t* s = esp_camera_sensor_get();
            if (s) {
                // Mapear el nombre de resolución al enum del driver
                if      (strcmp(size, "vga")  == 0) s->set_framesize(s, FRAMESIZE_VGA);  // 640x480
                else if (strcmp(size, "svga") == 0) s->set_framesize(s, FRAMESIZE_SVGA); // 800x600
                else if (strcmp(size, "uxga") == 0) s->set_framesize(s, FRAMESIZE_UXGA); // 1600x1200

                Serial.printf("Resolucion cambiada a: %s\n", size);
            }
        }
    }

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", 2);
}


// ─────────────────────────────────────────────────────────────────
//  CONSTANTES PARA EL STREAM MJPEG
//
//  MJPEG funciona como una respuesta HTTP que nunca termina.
//  El servidor envía frames JPEG separados por un "boundary"
//  (texto delimitador). El navegador actualiza la imagen
//  cada vez que recibe un nuevo frame, creando el efecto de video.
//
//  Estructura de cada "parte" del multipart:
//
//    --frame_boundary\r\n
//    Content-Type: image/jpeg\r\n
//    Content-Length: <bytes>\r\n
//    \r\n
//    [datos binarios JPEG]
//    --frame_boundary\r\n
//    ... (se repite indefinidamente)
// ─────────────────────────────────────────────────────────────────
#define PART_BOUNDARY "frame_boundary"

static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char* STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";

// Cabecera de cada frame; %u se reemplaza con el tamaño real en bytes
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


// ─────────────────────────────────────────────────────────────────
//  HANDLER: stream MJPEG  "/stream"
//
//  Función que corre en bucle infinito mientras haya un cliente
//  conectado. Captura frames y los envía uno tras otro.
//  Solo termina si el cliente cierra la conexión o hay un error.
// ─────────────────────────────────────────────────────────────────
static esp_err_t streamHandler(httpd_req_t* req) {

    esp_err_t    res = ESP_OK;
    camera_fb_t* fb  = nullptr; // Apunta al buffer del frame actual
    char         part_buf[64];  // Para construir la cabecera MIME de cada frame
    uint32_t     frame_count = 0; // Contador de frames enviados (para debug)

    // Establecer el Content-Type de la respuesta como MJPEG multipart
    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    // Permitir acceso desde cualquier origen (CORS)
    // Útil si el stream se consume desde otra página o dominio
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Deshabilitar el timeout del servidor para esta conexión.
    // Las conexiones de stream son muy largas por diseño; sin esto
    // el servidor HTTP las corta después de unos segundos.
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    Serial.println("Cliente conectado al stream");

    // ── Bucle principal de streaming ─────────────────────────────
    while (true) {

        // --- Capturar frame de la cámara ---
        // esp_camera_fb_get() bloquea hasta que el sensor tiene
        // un frame completo listo en memoria.
        // Con doble buffer (fb_count=2) la espera es mínima porque
        // el segundo buffer ya estaba llenándose en paralelo.
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Error al capturar frame — reintentando...");
            delay(10); // Pequeña pausa antes de reintentar
            continue;  // Saltar al siguiente ciclo del while
        }

        // --- Enviar separador de frame ---
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                    strlen(STREAM_BOUNDARY));

        // --- Enviar cabecera MIME del frame ---
        if (res == ESP_OK) {
            // Construir la cabecera con el tamaño exacto del frame JPEG
            size_t hlen = snprintf(part_buf, sizeof(part_buf),
                                   STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }

        // --- Enviar datos binarios del frame JPEG ---
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req,
                                        (const char*)fb->buf, fb->len);
        }

        // --- Liberar el buffer del frame ---
        // OBLIGATORIO: si no se llama, el driver no puede reusar el buffer
        // y después de 2 frames (con doble buffer) la cámara se traba.
        esp_camera_fb_return(fb);
        fb = nullptr;

        frame_count++;

        // Imprimir estadísticas cada 100 frames (para debug)
        if (frame_count % 100 == 0) {
            Serial.printf("Frames enviados: %u | Heap libre: %u bytes\n",
                          frame_count, esp_get_free_heap_size());
        }

        // Si hubo error de red (cliente desconectado), salir del bucle
        if (res != ESP_OK) {
            Serial.println("Cliente desconectado del stream");
            break;
        }
    }

    return res;
}


// ─────────────────────────────────────────────────────────────────
//  FUNCIÓN: startServer()
//
//  Inicia el servidor HTTP y registra todos los endpoints:
//    GET /        → página HTML con el visor
//    GET /stream  → stream MJPEG de video en vivo
//    GET /flash   → controla el LED flash integrado
//    GET /res     → cambia la resolución de la cámara
// ─────────────────────────────────────────────────────────────────
httpd_handle_t startServer() {

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = 80;  // Puerto HTTP estándar
    config.max_uri_handlers = 8;   // Número máximo de rutas registradas

    httpd_handle_t server = nullptr;
    if (httpd_start(&server, &config) != ESP_OK) {
        Serial.println("Error al iniciar servidor HTTP");
        return nullptr;
    }

    // Registrar endpoint: página principal
    httpd_uri_t indexUri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = indexHandler,
        .user_ctx = nullptr
    };

    // Registrar endpoint: stream de video
    httpd_uri_t streamUri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = streamHandler,
        .user_ctx = nullptr
    };

    // Registrar endpoint: control del LED flash
    httpd_uri_t flashUri = {
        .uri      = "/flash",
        .method   = HTTP_GET,
        .handler  = flashHandler,
        .user_ctx = nullptr
    };

    // Registrar endpoint: cambio de resolución
    httpd_uri_t resUri = {
        .uri      = "/res",
        .method   = HTTP_GET,
        .handler  = resHandler,
        .user_ctx = nullptr
    };

    httpd_register_uri_handler(server, &indexUri);
    httpd_register_uri_handler(server, &streamUri);
    httpd_register_uri_handler(server, &flashUri);
    httpd_register_uri_handler(server, &resUri);

    return server;
}


// ─────────────────────────────────────────────────────────────────
//  SETUP — Se ejecuta una sola vez al encender o reiniciar
// ─────────────────────────────────────────────────────────────────
void setup() {

    // Iniciar Serial a 115200 baudios
    // Debe coincidir con monitor_speed en platformio.ini
    Serial.begin(115200);

    // Esperar hasta 3 segundos a que el USB CDC esté listo.
    // La Freenove usa USB nativo del ESP32-S3 (no un chip CH340/CP2102),
    // por lo que el puerto serial tarda un momento en aparecer en el SO.
    unsigned long t = millis();
    while (!Serial && millis() - t < 3000);

    Serial.println("\n=== Freenove ESP32-S3 WROOM CAM ===");

    // --- Configurar LED flash como salida y apagarlo ---
    pinMode(LED_FLASH_PIN, OUTPUT);
    digitalWrite(LED_FLASH_PIN, LOW); // Asegurarse de que inicie apagado
    Serial.println("[OK] LED flash configurado (GPIO 2)");

    // --- Inicializar la cámara OV2640 ---
    if (!initCamera()) {
        Serial.println("FALLO: La camara no pudo iniciarse.");
        Serial.println("  → Verifica que el modulo de camara este bien conectado");
        Serial.println("  → Revisa que los pines en el codigo coincidan con tu placa");
        Serial.println("Reiniciando en 5 segundos...");
        delay(5000);
        ESP.restart(); // Reinicio por software
    }
    Serial.println("[OK] Camara OV2640 inicializada");
    Serial.printf("     PSRAM disponible : %s\n", psramFound() ? "SI" : "NO");
    if (psramFound()) {
        Serial.printf("     Tamano PSRAM    : %u bytes\n", ESP.getPsramSize());
        Serial.println("     Resolucion      : UXGA 1600x1200");
    } else {
        Serial.println("     Resolucion      : SVGA 800x600 (sin PSRAM)");
    }

    // --- Conectar a WiFi ---
    Serial.printf("\nConectando a WiFi '%s'", SSID);
    WiFi.mode(WIFI_STA); // Modo station (cliente), no punto de acceso
    WiFi.begin(SSID, PASSWORD);

    // Esperar conexión mostrando progreso en el serial
    uint8_t intentos = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        intentos++;

        // Si tarda más de 30 segundos (60 intentos), reiniciar
        if (intentos > 60) {
            Serial.println("\nTiempo de espera agotado. Reiniciando...");
            ESP.restart();
        }
    }

    Serial.println("\n[OK] WiFi conectado");
    Serial.printf("     Red         : %s\n", WiFi.SSID().c_str());
    Serial.printf("     Intensidad  : %d dBm\n", WiFi.RSSI());
    Serial.printf("     IP asignada : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("     MAC         : %s\n", WiFi.macAddress().c_str());

    // --- Iniciar el servidor HTTP ---
    if (!startServer()) {
        Serial.println("FALLO: El servidor HTTP no pudo iniciarse.");
        delay(5000);
        ESP.restart();
    }
    Serial.println("[OK] Servidor HTTP iniciado en puerto 80");

    // --- Resumen de endpoints disponibles ---
    String ip = WiFi.localIP().toString();
    Serial.println("\n─────────────────────────────────────────");
    Serial.println("  ENDPOINTS DISPONIBLES:");
    Serial.printf("  Visor de camara  →  http://%s\n",        ip.c_str());
    Serial.printf("  Stream directo   →  http://%s/stream\n", ip.c_str());
    Serial.printf("  Flash ON         →  http://%s/flash?state=true\n",  ip.c_str());
    Serial.printf("  Flash OFF        →  http://%s/flash?state=false\n", ip.c_str());
    Serial.printf("  Cambiar res.     →  http://%s/res?size=vga\n", ip.c_str());
    Serial.println("─────────────────────────────────────────\n");

    // Parpadeo del LED flash para indicar que el sistema está listo
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_FLASH_PIN, HIGH);
        delay(100);
        digitalWrite(LED_FLASH_PIN, LOW);
        delay(100);
    }
}


// ─────────────────────────────────────────────────────────────────
//  LOOP — Se ejecuta repetidamente tras el setup()
//
//  El servidor HTTP y el driver de cámara corren en sus propios
//  tasks del FreeRTOS (sistema operativo del ESP32), así que el
//  loop solo necesita supervisar el estado general del sistema.
// ─────────────────────────────────────────────────────────────────
void loop() {

    // --- Supervisión del WiFi ---
    // Si la conexión se cae (router reiniciado, señal débil, etc.),
    // intentar reconectar automáticamente sin reiniciar el ESP32.
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado. Reconectando...");
        WiFi.disconnect();
        WiFi.begin(SSID, PASSWORD);

        // Esperar hasta 10 segundos para reconectar
        uint8_t intentos = 0;
        while (WiFi.status() != WL_CONNECTED && intentos < 20) {
            delay(500);
            Serial.print(".");
            intentos++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nReconectado. IP: %s\n",
                          WiFi.localIP().toString().c_str());
        } else {
            Serial.println("\nNo se pudo reconectar. Reiniciando...");
            delay(1000);
            ESP.restart();
        }
    }

    // Ceder tiempo al sistema operativo FreeRTOS cada segundo.
    // delay() permite que otros tasks (servidor HTTP, cámara) corran.
    delay(1000);
}