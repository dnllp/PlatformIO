#include <Arduino.h>
#include <SPI.h>

SPIClass spi(VSPI);

#define NSS_PIN   5
#define RST_PIN   14
#define DIO1_PIN  26
#define BUSY_PIN  27
#define RXEN_PIN  32
#define TXEN_PIN  33

// ── SPI helpers ─────────────────────────────────────────────────────
void waitBusy(uint32_t timeout = 5000) {
    uint32_t t = millis();
    while (digitalRead(BUSY_PIN)) {
        if (millis() - t > timeout) { Serial.println("[!] waitBusy timeout"); return; }
    }
}

void sx_cmd(uint8_t cmd, uint8_t* data = nullptr, uint8_t len = 0) {
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(cmd);
    for (uint8_t i = 0; i < len; i++) spi.transfer(data[i]);
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);
}

void sx_read(uint8_t cmd, uint8_t* result, uint8_t rlen,
             uint8_t* params = nullptr, uint8_t plen = 0) {
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(cmd);
    for (uint8_t i = 0; i < plen; i++) spi.transfer(params[i]);
    for (uint8_t i = 0; i < rlen; i++) result[i] = spi.transfer(0x00);
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);
}

void writeReg(uint16_t addr, uint8_t val) {
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(0x0D);
    spi.transfer((addr >> 8) & 0xFF);
    spi.transfer(addr & 0xFF);
    spi.transfer(val);
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);
}

uint16_t getIrqStatus() {
    uint8_t buf[3];
    sx_read(0x12, buf, 3);
    return ((uint16_t)buf[1] << 8) | buf[2];
}

void clearIrq() {
    uint8_t p[2] = {0xFF, 0xFF};
    sx_cmd(0x02, p, 2);
}

// ── Init ─────────────────────────────────────────────────────────────
void sx1262_init() {
    digitalWrite(RST_PIN, LOW); delay(20);
    digitalWrite(RST_PIN, HIGH); delay(50);
    waitBusy(3000);

    // Standby RC
    uint8_t p[8];
    p[0] = 0x00; sx_cmd(0x80, p, 1); delay(10);

    // PacketType = LoRa
    p[0] = 0x01; sx_cmd(0x8A, p, 1); delay(5);

    // Frecuencia 915 MHz
    uint32_t frf = (uint32_t)(915000000.0 * (1 << 25) / 32000000.0);
    p[0]=(frf>>24)&0xFF; p[1]=(frf>>16)&0xFF;
    p[2]=(frf>> 8)&0xFF; p[3]=(frf    )&0xFF;
    sx_cmd(0x86, p, 4); delay(5);

    // TxParams: 14 dBm, ramp 200us
    p[0] = 14; p[1] = 0x04; sx_cmd(0x8E, p, 2);

    // ModulationParams: SF7, BW125, CR4/5
    p[0]=0x07; p[1]=0x04; p[2]=0x01; p[3]=0x00;
    sx_cmd(0x8B, p, 4);

    // PacketParams: preamble=8, header explícito, payload=255, CRC, IQ normal
    uint8_t pp[6] = {0x00, 0x08, 0x00, 0xFF, 0x01, 0x00};
    sx_cmd(0x8C, pp, 6);

    // SyncWord 0xAB → 0xA4 / 0xB4 en registros 0x0740-0x0741
    writeReg(0x0740, 0xA4);
    writeReg(0x0741, 0xB4);

    // *** CONFIGURAR IRQ ***
    // SetDioIrqParams: TxDone=bit0, RxDone=bit1, Timeout=bit9
    // Mask IRQ: 0x0201 → TxDone + Timeout en DIO1
    // p = [irqMask(2), dio1Mask(2), dio2Mask(2), dio3Mask(2)]
    uint8_t irqp[8] = {
        0x02, 0x01,   // IRQ mask: TxDone | Timeout
        0x02, 0x01,   // DIO1 mask: TxDone | Timeout
        0x00, 0x00,   // DIO2 mask: nada
        0x00, 0x00    // DIO3 mask: nada
    };
    sx_cmd(0x08, irqp, 8);

    clearIrq();
    Serial.println("[SX1262] Init OK ✅");
}

// ── TX ───────────────────────────────────────────────────────────────
void sx1262_transmit(const String& msg) {
    uint8_t p[4];
    uint8_t len = msg.length();

    // Standby antes de cargar buffer
    p[0] = 0x00; sx_cmd(0x80, p, 1); delay(5);

    clearIrq();

    // Escribir payload
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(0x0E); spi.transfer(0x00);
    for (uint8_t i = 0; i < len; i++) spi.transfer((uint8_t)msg[i]);
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);

    // PacketParams con tamaño real
    uint8_t pp[6] = {0x00, 0x08, 0x00, len, 0x01, 0x00};
    sx_cmd(0x8C, pp, 6);

    // TXEN alto justo antes de SetTx
    digitalWrite(RXEN_PIN, LOW);
    digitalWrite(TXEN_PIN, HIGH);
    delayMicroseconds(100);

    // SetTx con timeout 3s
    uint32_t to = 192000;
    p[0]=(to>>16)&0xFF; p[1]=(to>>8)&0xFF; p[2]=to&0xFF;
    sx_cmd(0x83, p, 3);

    // Esperar por polling de IRQ (no depende de DIO1 físico)
    Serial.print("   Esperando IRQ");
    uint32_t t = millis();
    uint16_t irq = 0;
    while (millis() - t < 5000) {
        irq = getIrqStatus();
        if (irq & 0x0001) { // TxDone
            Serial.println("\n   TxDone ✅");
            clearIrq();
            digitalWrite(TXEN_PIN, LOW);
            return;
        }
        if (irq & 0x0200) { // Timeout
            Serial.println("\n   [!] Timeout IRQ del modulo");
            break;
        }
        Serial.print(".");
        delay(50);
    }

    // Debug: mostrar IRQ y status
    Serial.print("\n   IRQ Status: 0x"); Serial.println(irq, HEX);
    uint8_t status[2];
    sx_read(0xC0, status, 2);
    Serial.print("   Chip Status: 0x"); Serial.print(status[0], HEX);
    Serial.print(" 0x"); Serial.println(status[1], HEX);

    uint8_t mode = (status[1] >> 4) & 0x07;
    Serial.print("   Modo: ");
    switch(mode) {
        case 2: Serial.println("STDBY_RC"); break;
        case 5: Serial.println("RX"); break;
        case 6: Serial.println("TX ← sigue en TX, señal no sale"); break;
        default: Serial.println(mode); break;
    }

    clearIrq();
    digitalWrite(TXEN_PIN, LOW);
}

int counter = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(NSS_PIN,  OUTPUT); digitalWrite(NSS_PIN, HIGH);
    pinMode(RST_PIN,  OUTPUT); digitalWrite(RST_PIN, HIGH);
    pinMode(BUSY_PIN, INPUT);
    pinMode(DIO1_PIN, INPUT);
    pinMode(RXEN_PIN, OUTPUT); digitalWrite(RXEN_PIN, LOW);
    pinMode(TXEN_PIN, OUTPUT); digitalWrite(TXEN_PIN, LOW);

    spi.begin(18, 19, 23, NSS_PIN);
    spi.setFrequency(2000000);

    sx1262_init();
}

void loop() {
    String msg = "Hola LoRa #" + String(counter++);
    Serial.print("[TX] " + msg + "\n");
    sx1262_transmit(msg);
    delay(3000);
}