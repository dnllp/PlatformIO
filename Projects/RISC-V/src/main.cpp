#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
}

void loop() {
    uint32_t val1 = 15;
    uint32_t resultado;

    // Usando la instrucción 'add' de RISC-V
    // Sumamos val1 con el registro x0 (que es 0) y guardamos en resultado
    asm volatile (
        "add %0, %1, x0"  // rd = rs1 + x0
        : "=r" (resultado) // Salida
        : "r" (val1)       // Entrada
    );

    Serial.printf("Valor original: %d | Resultado tras 'add rd, rs1, x0': %d\n", val1, resultado);
    
    // Demostración de desplazamiento de bits (Logical Shift Right)
    uint32_t shift_res;
    asm volatile (
        "srli %0, %1, 1"  // Shift Right Logical Immediate (divide por 2)
        : "=r" (shift_res)
        : "r" (val1)
    );
    
    Serial.printf("Desplazamiento a la derecha (div 2): %d\n", shift_res);
    
    delay(3000);
}