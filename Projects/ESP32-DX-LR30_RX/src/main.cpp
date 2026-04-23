#include <Arduino.h>
#include <SPI.h>

SPIClass spi(VSPI);

#define NSS_PIN   5
#define RST_PIN   14
#define DIO1_PIN  26
#define BUSY_PIN  27
#define RXEN_PIN  32
#define TXEN_PIN  33

// ── SPI helpers ──────────────────────────────────────────────────────
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

uint8_t readReg(uint16_t addr) {
    uint8_t val;
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(0x1D);
    spi.transfer((addr >> 8) & 0xFF);
    spi.transfer(addr & 0xFF);
    spi.transfer(0x00); // NOP
    val = spi.transfer(0x00);
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);
    return val;
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

    uint8_t p[8];

    // Standby RC
    p[0] = 0x00; sx_cmd(0x80, p, 1); delay(10);

    // PacketType = LoRa
    p[0] = 0x01; sx_cmd(0x8A, p, 1); delay(5);

    // Frecuencia 915 MHz
    uint32_t frf = (uint32_t)(915000000.0 * (1 << 25) / 32000000.0);
    p[0]=(frf>>24)&0xFF; p[1]=(frf>>16)&0xFF;
    p[2]=(frf>> 8)&0xFF; p[3]=(frf    )&0xFF;
    sx_cmd(0x86, p, 4); delay(5);

    // ModulationParams: SF7, BW125, CR4/5  ← igual que TX
    p[0]=0x07; p[1]=0x04; p[2]=0x01; p[3]=0x00;
    sx_cmd(0x8B, p, 4);

    // PacketParams: preamble=8, header explícito, payload=255, CRC, IQ normal
    uint8_t pp[6] = {0x00, 0x08, 0x00, 0xFF, 0x01, 0x00};
    sx_cmd(0x8C, pp, 6);

    // SyncWord 0xAB  ← igual que TX
    writeReg(0x0740, 0xA4);
    writeReg(0x0741, 0xB4);

    // IRQ: RxDone=bit1, CrcErr=bit6, Timeout=bit9 → en DIO1
    uint8_t irqp[8] = {0x02, 0x42, 0x02, 0x42, 0x00, 0x00, 0x00, 0x00};
    sx_cmd(0x08, irqp, 8);

    clearIrq();
    Serial.println("[SX1262] Init RX OK ✅");
}

// ── Poner en modo RX continuo ────────────────────────────────────────
void sx1262_startRx() {
    uint8_t p[4];

    // Standby
    p[0] = 0x00; sx_cmd(0x80, p, 1); delay(5);

    // Habilitar RX
    digitalWrite(TXEN_PIN, LOW);
    digitalWrite(RXEN_PIN, HIGH);
    delayMicroseconds(100);

    // SetRx: 0xFFFFFF = modo continuo (sin timeout)
    p[0] = 0xFF; p[1] = 0xFF; p[2] = 0xFF;
    sx_cmd(0x82, p, 3);
}

// ── Leer paquete recibido ────────────────────────────────────────────
String sx1262_readPacket(int8_t* rssi_out, int8_t* snr_out) {
    // GetRxBufferStatus → offset y tamaño del payload
    uint8_t rxbuf[3];
    sx_read(0x13, rxbuf, 3);
    uint8_t payloadLen    = rxbuf[1];
    uint8_t bufferOffset  = rxbuf[2];

    // ReadBuffer
    waitBusy();
    digitalWrite(NSS_PIN, LOW); delayMicroseconds(50);
    spi.transfer(0x1E);
    spi.transfer(bufferOffset);
    spi.transfer(0x00); // NOP status
    String msg = "";
    for (uint8_t i = 0; i < payloadLen; i++) {
        msg += (char)spi.transfer(0x00);
    }
    delayMicroseconds(50); digitalWrite(NSS_PIN, HIGH); delayMicroseconds(100);

    // GetPacketStatus → RSSI y SNR
    uint8_t pktStatus[4];
    sx_read(0x14, pktStatus, 4);
    if (rssi_out) *rssi_out = -(int8_t)(pktStatus[1] / 2);
    if (snr_out)  *snr_out  =  (int8_t)(pktStatus[2] / 4);

    return msg;
}

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
    sx1262_startRx();
    Serial.println("[RX] Escuchando...");
}

void loop() {
    uint16_t irq = getIrqStatus();

    if (irq & 0x0002) {          // RxDone
        clearIrq();

        if (irq & 0x0040) {      // CRC error
            Serial.println("[RX] ❌ Error CRC");
        } else {
            int8_t rssi, snr;
            String msg = sx1262_readPacket(&rssi, &snr);
            Serial.print("[RX] ✅ Mensaje: ");
            Serial.println(msg);
            Serial.print("     RSSI: "); Serial.print(rssi);
            Serial.print(" dBm | SNR: "); Serial.print(snr);
            Serial.println(" dB");
        }

        // Volver a RX continuo
        sx1262_startRx();
    }

    delay(10);
}