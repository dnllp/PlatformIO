#include "Arduino.h"
#include "esp_camera.h"
#include "WiFi.h"
#include "esp_http_server.h"

// ─── Credenciales WiFi ────────────────────────────────────────────
const char* SSID     = "UAM-ROBOTICA";
const char* PASSWORD = "m4nt32024uat";

// ─── Pinout ESP32-S3-CAM Goouuu ──────────────────────────────────
// Ajusta si tu módulo tiene diferente asignación
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     15
#define CAM_PIN_SIOD     4
#define CAM_PIN_SIOC     5
#define CAM_PIN_D7       16
#define CAM_PIN_D6       17
#define CAM_PIN_D5       18
#define CAM_PIN_D4       12
#define CAM_PIN_D3       10
#define CAM_PIN_D2       8
#define CAM_PIN_D1       9
#define CAM_PIN_D0       11
#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK     13

// ─── Inicializar cámara ───────────────────────────────────────────
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_pclk     = CAM_PIN_PCLK;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Usa PSRAM si está disponible para mayor resolución
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_VGA;   // 640x480
        config.jpeg_quality = 10;              // 0-63, menor = mejor
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;  // 320x240
        config.jpeg_quality = 12;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error al iniciar cámara: 0x%x\n", err);
        return false;
    }
    return true;
}

// ─── Handler: página principal ────────────────────────────────────
static esp_err_t indexHandler(httpd_req_t* req) {
    const char* html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>ESP32-S3 Cam</title>"
        "<style>body{margin:0;background:#111;display:flex;"
        "flex-direction:column;align-items:center;justify-content:center;"
        "min-height:100vh;font-family:sans-serif;color:#eee;}"
        "img{max-width:100%;border:2px solid #333;border-radius:8px;}"
        "h2{margin-bottom:12px;}</style></head><body>"
        "<h2>ESP32-S3-CAM Goouuu</h2>"
        "<img src='/stream' />"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// ─── Handler: stream MJPEG ────────────────────────────────────────
#define PART_BOUNDARY "frame_boundary"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t* req) {
    esp_err_t res = ESP_OK;
    camera_fb_t* fb = nullptr;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Error al capturar frame");
            res = ESP_FAIL;
            break;
        }

        // Enviar boundary
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                    strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, sizeof(part_buf),
                                   STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req,
                                        (const char*)fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);
        fb = nullptr;

        if (res != ESP_OK) break;
    }
    return res;
}

// ─── Iniciar servidor HTTP ────────────────────────────────────────
httpd_handle_t startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = nullptr;
    if (httpd_start(&server, &config) != ESP_OK) {
        Serial.println("Error al iniciar servidor HTTP");
        return nullptr;
    }

    httpd_uri_t indexUri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = indexHandler,
        .user_ctx = nullptr
    };
    httpd_uri_t streamUri = {
        .uri     = "/stream",
        .method  = HTTP_GET,
        .handler = streamHandler,
        .user_ctx = nullptr
    };

    httpd_register_uri_handler(server, &indexUri);
    httpd_register_uri_handler(server, &streamUri);

    return server;
}

// ─── Setup ────────────────────────────────────────────────────────
void setup() {

    delay(3000);
    Serial.begin(115200);
    Serial.println("\nIniciando ESP32-S3-CAM...");

    if (!initCamera()) {
        Serial.println("Fallo en la cámara. Reiniciando en 5s...");
        delay(5000);
        ESP.restart();
    }
    Serial.println("Cámara OK");

    WiFi.begin(SSID, PASSWORD);
    Serial.print("Conectando a WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    startServer();
    Serial.println("Servidor HTTP iniciado");
    Serial.println("Abre en tu navegador: http://" + WiFi.localIP().toString());
}

void loop() {
    delay(1000);
}