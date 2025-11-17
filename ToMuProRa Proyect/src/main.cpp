#include <Arduino.h>

// ===== CONFIGURACIÓN DE PINES =====
const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;
const int PULSE_OUTPUT_PIN = 10;

// ===== VARIABLES DE SISTEMA =====
volatile uint16_t rawSample = 0;
volatile bool newSampleReady = false;
bool inPulse = false;
#define Q15_SHIFT 15

// ===== FUNCIONES AUXILIARES Q15 =====
// Saturación segura de Q30 a Q15
inline int32_t saturate_q30_to_q15(int64_t value) {
  int64_t temp = value >> Q15_SHIFT;
  if (temp > 2147483647L) return 2147483647L;
  if (temp < -2147483648L) return -2147483648L;
  return (int32_t)temp;
}

// ===== VALORES POR DEFECTO (21 Hz ± 9 Hz, Q=2.333) =====
const int32_t DEFAULT_b0_q15 = 456;
const int32_t DEFAULT_b1_q15 = 0;
const int32_t DEFAULT_b2_q15 = -456;
const int32_t DEFAULT_a1_q15 = -64482;
const int32_t DEFAULT_a2_q15 = 31855;
const int32_t DEFAULT_MAGNITUDE_THRESHOLD = 50L;
const int16_t DEFAULT_PHASE_THRESHOLD = 2730;
const uint16_t DEFAULT_PREDICTION_DELAY = 75;

// ===== FILTRO IIR PASA-BANDA (VARIABLES MODIFICABLES) =====
int32_t b0_q15 = DEFAULT_b0_q15;
int32_t b1_q15 = DEFAULT_b1_q15;
int32_t b2_q15 = DEFAULT_b2_q15;
int32_t a1_q15 = DEFAULT_a1_q15;
int32_t a2_q15 = DEFAULT_a2_q15;

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

// ===== FILTRO DE HILBERT FIR =====
#define HILBERT_TAPS 15

// Coeficientes por defecto (21 Hz)
const int16_t DEFAULT_hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

// Coeficientes modificables
int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

// ===== PARÁMETROS DE DETECCIÓN (MODIFICABLES) =====
int32_t MAGNITUDE_THRESHOLD = DEFAULT_MAGNITUDE_THRESHOLD << Q15_SHIFT;
int16_t PHASE_THRESHOLD = DEFAULT_PHASE_THRESHOLD;
uint16_t PREDICTION_DELAY_SAMPLES = DEFAULT_PREDICTION_DELAY;

// ===== CONTROL DE PULSOS =====
volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 10;  // 10ms para mejor visibilidad en osciloscopio
uint16_t refractoryCountdown = 0;

// ===== COLA DE PULSOS =====
#define PULSE_QUEUE_SIZE 4
uint16_t pulseQueue[PULSE_QUEUE_SIZE];
uint8_t pulseQueueHead = 0;
uint8_t pulseQueueTail = 0;
uint8_t pulseQueueCount = 0;

// ===== COMUNICACIÓN SERIAL NO BLOQUEANTE =====
#define SERIAL_BUFFER_SIZE 128
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialIndex = 0;
bool commandReady = false;

// ===== TRANSMISIÓN DE DATOS =====
// Frecuencia de muestreo ADC: 2000 Hz
// Frecuencia de transmisión serial deseada: 500 Hz
// Decimación = 2000 / 500 = 4
// Uso de ancho de banda: ~65% (7500 bytes/s de 11520 bytes/s @ 115200 baudios)
const uint8_t SERIAL_DECIMATION = 4;

// ===== CONFIGURACIÓN DE ADC (2000 Hz) =====
void setupADC() {
  ADCSRA &= ~0x07;
  ADCSRA |= 0x04;
  ADMUX = 0;
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADEN);
}

// ===== CONFIGURACIÓN DE TIMER2 (DISPARO DE ADC A 2000 Hz) =====
void setupTimer2() {
  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 124;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

// ===== CONFIGURACIÓN DE PWM (SALIDA DE SEÑAL FILTRADA) =====
void setupPWM() {
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  OCR1A = 0;
}

// ===== INTERRUPCIONES =====
ISR(TIMER2_COMPA_vect) {
  ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
  rawSample = ADC;
  newSampleReady = true;
}

// ===== ALGORITMO CORDIC (CÁLCULO DE MAGNITUD Y FASE) =====
const int16_t cordic_angles[16] = {
  16384, 9672, 5110, 2594, 1302, 652, 326, 163,
  81, 41, 20, 10, 5, 3, 1, 1
};

const int32_t CORDIC_GAIN = 19898;

struct CordicResult {
  int32_t magnitude;
  int16_t phase;
};

CordicResult fastCORDIC(int32_t real, int32_t imag) {
  CordicResult result = {0, 0};
  if (real == 0 && imag == 0) return result;

  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;
  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;

  uint8_t shift = 0;
  if (max_val > (1L << 20)) {
    shift = 5;
    real >>= shift;
    imag >>= shift;
  }

  int16_t angle = 0;
  int32_t x = real;
  int32_t y = imag;

  if (x < 0) {
    x = -x;
    y = -y;
    angle = (y >= 0) ? 32768 : -32768;
  }

  for (uint8_t i = 0; i < 16; i++) {
    int32_t x_new, y_new;
    if (y < 0) {
      x_new = x - (y >> i);
      y_new = y + (x >> i);
      angle -= cordic_angles[i];
    } else {
      x_new = x + (y >> i);
      y_new = y - (x >> i);
      angle += cordic_angles[i];
    }
    x = x_new;
    y = y_new;
  }

  result.magnitude = (x * CORDIC_GAIN) >> Q15_SHIFT;
  result.magnitude <<= shift;
  result.phase = angle;
  return result;
}

// ===== FUNCIONES DE COMUNICACIÓN SERIAL =====
void resetFilterStates() {
  // Resetear estados del filtro IIR
  noInterrupts();
  x1_q15 = 0;
  x2_q15 = 0;
  y1_q15 = 0;
  y2_q15 = 0;

  // Resetear buffer de Hilbert
  for (uint8_t i = 0; i < HILBERT_TAPS; i++) {
    hilbert_buffer[i] = 0;
  }
  hilbert_index = 0;
  interrupts();
}

void restoreDefaultParameters() {
  noInterrupts();
  b0_q15 = DEFAULT_b0_q15;
  b1_q15 = DEFAULT_b1_q15;
  b2_q15 = DEFAULT_b2_q15;
  a1_q15 = DEFAULT_a1_q15;
  a2_q15 = DEFAULT_a2_q15;

  // Restaurar coeficientes de Hilbert
  for (uint8_t i = 0; i < HILBERT_TAPS; i++) {
    hilbert_coeffs[i] = DEFAULT_hilbert_coeffs[i];
  }

  MAGNITUDE_THRESHOLD = DEFAULT_MAGNITUDE_THRESHOLD << Q15_SHIFT;
  PHASE_THRESHOLD = DEFAULT_PHASE_THRESHOLD;
  PREDICTION_DELAY_SAMPLES = DEFAULT_PREDICTION_DELAY;
  interrupts();

  resetFilterStates();
  Serial.println("OK:DEFAULT_RESTORED");
}

void processCommand(char* cmd) {
  // Eliminar espacios al inicio y final
  while (*cmd == ' ' || *cmd == '\t') cmd++;

  uint8_t len = strlen(cmd);
  while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\t')) {
    cmd[--len] = '\0';
  }

  // Debug: mostrar comando recibido
  Serial.print("DEBUG:CMD_RECEIVED:");
  Serial.println(cmd);

  // Comando: RESET - Restaurar valores por defecto
  if (strcmp(cmd, "RESET") == 0) {
    restoreDefaultParameters();
    return;
  }

  // Comando: GET_STATUS - Obtener parámetros actuales
  if (strcmp(cmd, "GET_STATUS") == 0) {
    Serial.print("STATUS:");
    Serial.print(b0_q15); Serial.print(",");
    Serial.print(b1_q15); Serial.print(",");
    Serial.print(b2_q15); Serial.print(",");
    Serial.print(a1_q15); Serial.print(",");
    Serial.print(a2_q15); Serial.print(",");
    Serial.print(MAGNITUDE_THRESHOLD >> Q15_SHIFT); Serial.print(",");
    Serial.print(PHASE_THRESHOLD); Serial.print(",");
    Serial.println(PREDICTION_DELAY_SAMPLES);
    return;
  }

  // Comando: SET_FILTER:b0,b1,b2,a1,a2
  if (strncmp(cmd, "SET_FILTER:", 11) == 0) {
    char* params = cmd + 11;
    int32_t temp_b0, temp_b1, temp_b2, temp_a1, temp_a2;

    int parsed = sscanf(params, "%ld,%ld,%ld,%ld,%ld", &temp_b0, &temp_b1, &temp_b2, &temp_a1, &temp_a2);
    if (parsed == 5) {
      noInterrupts();
      b0_q15 = temp_b0;
      b1_q15 = temp_b1;
      b2_q15 = temp_b2;
      a1_q15 = temp_a1;
      a2_q15 = temp_a2;
      interrupts();

      resetFilterStates();
      Serial.println("OK:FILTER_SET");
    } else {
      Serial.print("ERROR:INVALID_FILTER_PARAMS:");
      Serial.println(parsed);
    }
    return;
  }

  // Comando: SET_HILBERT:h0,h1,h2,...,h14
  if (strncmp(cmd, "SET_HILBERT:", 12) == 0) {
    char* params = cmd + 12;
    int temp_hilbert[HILBERT_TAPS];

    // Parsear 15 coeficientes (usar %d porque int16_t es int en Arduino)
    int parsed = sscanf(params, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                       &temp_hilbert[0], &temp_hilbert[1], &temp_hilbert[2], &temp_hilbert[3],
                       &temp_hilbert[4], &temp_hilbert[5], &temp_hilbert[6], &temp_hilbert[7],
                       &temp_hilbert[8], &temp_hilbert[9], &temp_hilbert[10], &temp_hilbert[11],
                       &temp_hilbert[12], &temp_hilbert[13], &temp_hilbert[14]);

    if (parsed == HILBERT_TAPS) {
      noInterrupts();
      for (uint8_t i = 0; i < HILBERT_TAPS; i++) {
        hilbert_coeffs[i] = (int16_t)temp_hilbert[i];
      }
      interrupts();

      resetFilterStates();
      Serial.println("OK:HILBERT_SET");
    } else {
      Serial.print("ERROR:INVALID_HILBERT_PARAMS:");
      Serial.println(parsed);
    }
    return;
  }

  // Comando: SET_ALL:b0,b1,b2,a1,a2,mag,phase,delay
  if (strncmp(cmd, "SET_ALL:", 8) == 0) {
    char* params = cmd + 8;
    int32_t temp_b0, temp_b1, temp_b2, temp_a1, temp_a2, temp_mag;
    int16_t temp_phase;
    uint16_t temp_delay;

    int parsed = sscanf(params, "%ld,%ld,%ld,%ld,%ld,%ld,%d,%u",
               &temp_b0, &temp_b1, &temp_b2, &temp_a1, &temp_a2,
               &temp_mag, &temp_phase, &temp_delay);

    if (parsed == 8) {
      noInterrupts();
      b0_q15 = temp_b0;
      b1_q15 = temp_b1;
      b2_q15 = temp_b2;
      a1_q15 = temp_a1;
      a2_q15 = temp_a2;
      MAGNITUDE_THRESHOLD = temp_mag << Q15_SHIFT;
      PHASE_THRESHOLD = temp_phase;
      PREDICTION_DELAY_SAMPLES = temp_delay;
      interrupts();

      resetFilterStates();
      Serial.println("OK:ALL_SET");
    } else {
      Serial.print("ERROR:INVALID_ALL_PARAMS:");
      Serial.println(parsed);
    }
    return;
  }

  Serial.print("ERROR:UNKNOWN_COMMAND:");
  Serial.println(cmd);
}

void checkSerialCommand() {
  // Lectura no bloqueante de comandos seriales
  // Procesar hasta 50 bytes por llamada para evitar bloqueos
  uint8_t bytesProcessed = 0;
  const uint8_t MAX_BYTES_PER_CALL = 50;

  while (Serial.available() > 0 && !commandReady && bytesProcessed < MAX_BYTES_PER_CALL) {
    char c = Serial.read();
    bytesProcessed++;

    if (c == '\n' || c == '\r') {
      if (serialIndex > 0) {
        serialBuffer[serialIndex] = '\0';
        commandReady = true;
        break;  // Salir del while para procesar el comando
      }
      // Si serialIndex == 0, ignorar \n o \r vacíos
    } else if (c >= 32 && c <= 126) {  // Solo caracteres ASCII imprimibles
      if (serialIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[serialIndex++] = c;
      } else {
        // Buffer lleno, reiniciar
        serialIndex = 0;
        Serial.println("ERROR:BUFFER_OVERFLOW");
      }
    }
    // Ignorar otros caracteres de control (incluyendo bytes binarios)
  }

  // Procesar comando si está listo
  if (commandReady) {
    processCommand(serialBuffer);
    serialIndex = 0;
    commandReady = false;
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("READY");
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PULSE_OUTPUT_PIN, OUTPUT);
  digitalWrite(PULSE_OUTPUT_PIN, LOW);
  setupADC();
  setupPWM();
  setupTimer2();
}

// ===== LOOP PRINCIPAL =====
void loop() {
  // Verificar comandos seriales (no bloqueante)
  checkSerialCommand();

  // Control de pulso de salida
  if (triggerPulse) {
    digitalWrite(PULSE_OUTPUT_PIN, HIGH);
    pulseStartTime = millis();
    triggerPulse = false;
    inPulse = true;
  }

  if (inPulse) {
    if (millis() - pulseStartTime >= PULSE_DURATION) {
      digitalWrite(PULSE_OUTPUT_PIN, LOW);
      inPulse = false;
    }
  }

  // ===== PROCESAMIENTO DE SEÑAL =====
  if (newSampleReady) {
    noInterrupts();
    newSampleReady = false;
    uint16_t adcValue = rawSample;
    interrupts();

    // Convertir a Q15 (centrado en cero)
    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    // Filtro IIR pasa-banda biquad
    // Ecuación: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    // Los coeficientes a1 y a2 ya vienen con signo correcto desde Python
    int64_t numerator = ((int64_t)b0_q15 * x) + ((int64_t)b1_q15 * x1_q15) + ((int64_t)b2_q15 * x2_q15);
    int64_t denominator = ((int64_t)a1_q15 * y1_q15) + ((int64_t)a2_q15 * y2_q15);

    // Normalizar y calcular salida con saturación (a1, a2 ya incluyen el signo negativo)
    int32_t y_filtered = saturate_q30_to_q15(numerator - denominator);

    // Actualizar estados del filtro
    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y_filtered;

    // Filtro de Hilbert (transformada de 90 grados para obtener componente imaginaria)
    hilbert_buffer[hilbert_index] = y_filtered;
    hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;

    // Calcular salida del filtro Hilbert (FIR)
    int64_t hilbert_output = 0;
    uint8_t buf_idx = hilbert_index;
    for (int i = 0; i < HILBERT_TAPS; i++) {
      hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[buf_idx]);
      buf_idx = (buf_idx + 1) % HILBERT_TAPS;
    }
    int32_t imag_part = saturate_q30_to_q15(hilbert_output);

    // Calcular magnitud (envolvente) y fase usando CORDIC
    // real_part = señal filtrada, imag_part = transformada de Hilbert
    int32_t real_part = y_filtered;
    CordicResult cordic = fastCORDIC(real_part, imag_part);

    // La magnitud de CORDIC ya debe ser positiva
    int32_t envelope = cordic.magnitude;
    int16_t phase = cordic.phase;

    // Detección de picos
    bool isPeak = (envelope > MAGNITUDE_THRESHOLD) &&
                  ((phase > (32768 - PHASE_THRESHOLD)) || (phase < (-32768 + PHASE_THRESHOLD)));

    static bool wasAboveThreshold = false;

    // Gestión de cola de pulsos
    for (uint8_t i = 0; i < pulseQueueCount; i++) {
      uint8_t idx = (pulseQueueTail + i) % PULSE_QUEUE_SIZE;
      if (pulseQueue[idx] > 0) pulseQueue[idx]--;
    }

    if (pulseQueueCount > 0 && pulseQueue[pulseQueueTail] == 0) {
      triggerPulse = true;
      pulseQueueTail = (pulseQueueTail + 1) % PULSE_QUEUE_SIZE;
      pulseQueueCount--;
    }

    // Periodo refractario
    if (refractoryCountdown > 0) refractoryCountdown--;

    // Detección de flanco ascendente
    if (isPeak && !wasAboveThreshold && refractoryCountdown == 0) {
      if (pulseQueueCount < PULSE_QUEUE_SIZE) {
        pulseQueue[pulseQueueHead] = PREDICTION_DELAY_SAMPLES;
        pulseQueueHead = (pulseQueueHead + 1) % PULSE_QUEUE_SIZE;
        pulseQueueCount++;
      }
      refractoryCountdown = 20;
    }

    wasAboveThreshold = isPeak;

    // Salida PWM (envolvente de la señal)
    // La envolvente está en formato Q15, normalizarla a rango 0-255 para PWM de 8 bits
    // Escalar para que sea visible: envelope está en Q15, dividir por 2^15 y luego amplificar
    int32_t envelope_scaled = envelope >> Q15_SHIFT;  // Convertir de Q15 a entero

    // Amplificar la envolvente para PWM (ajustar ganancia según sea necesario)
    int32_t pwm_value = envelope_scaled * 2;  // Factor de escala ajustable

    // Limitar a rango válido de PWM (0-255)
    if (pwm_value < 0) pwm_value = 0;
    if (pwm_value > 255) pwm_value = 255;

    OCR1A = (uint8_t)pwm_value;

    // ===== TRANSMISIÓN DE DATOS PARA VISUALIZACIÓN (500 Hz) =====
    // Decimación configurable para alcanzar frecuencia deseada
    static uint8_t viz_counter = 0;
    if (++viz_counter >= SERIAL_DECIMATION) {
      viz_counter = 0;

      // Protocolo binario: 0xAA 0x55 | ADC(2) | Filtered(4) | Envelope(4) | Phase(2) | PulseFlag(1) | CRC(1)
      // Total: 16 bytes por paquete

      // Start bytes (sincronización)
      Serial.write(0xAA);
      Serial.write(0x55);

      // ADC raw (0-1023, 10 bits en uint16_t)
      Serial.write((uint8_t)(adcValue & 0xFF));
      Serial.write((uint8_t)((adcValue >> 8) & 0xFF));

      // Señal filtrada (Q15 format, int32_t)
      Serial.write((uint8_t)(y_filtered & 0xFF));
      Serial.write((uint8_t)((y_filtered >> 8) & 0xFF));
      Serial.write((uint8_t)((y_filtered >> 16) & 0xFF));
      Serial.write((uint8_t)((y_filtered >> 24) & 0xFF));

      // Envolvente/Magnitud: convertir de entero a Q15 format para consistencia
      // envelope está en unidades ADC (0-512), convertir a Q15: envelope << 15
      int32_t envelope_q15 = envelope << Q15_SHIFT;
      Serial.write((uint8_t)(envelope_q15 & 0xFF));
      Serial.write((uint8_t)((envelope_q15 >> 8) & 0xFF));
      Serial.write((uint8_t)((envelope_q15 >> 16) & 0xFF));
      Serial.write((uint8_t)((envelope_q15 >> 24) & 0xFF));

      // Fase (Q15 format, int16_t, rango -32768 a 32767)
      Serial.write((uint8_t)(phase & 0xFF));
      Serial.write((uint8_t)((phase >> 8) & 0xFF));

      // Flag de pulso (1 = pulso activo, 0 = sin pulso)
      uint8_t pulseFlag = inPulse ? 1 : 0;
      Serial.write(pulseFlag);

      // Checksum simple (XOR de todos los datos)
      uint8_t checksum = (uint8_t)(adcValue & 0xFF) ^ (uint8_t)((adcValue >> 8) & 0xFF);
      checksum ^= (uint8_t)(y_filtered & 0xFF) ^ (uint8_t)((y_filtered >> 8) & 0xFF);
      checksum ^= (uint8_t)((y_filtered >> 16) & 0xFF) ^ (uint8_t)((y_filtered >> 24) & 0xFF);
      checksum ^= (uint8_t)(envelope_q15 & 0xFF) ^ (uint8_t)((envelope_q15 >> 8) & 0xFF);
      checksum ^= (uint8_t)((envelope_q15 >> 16) & 0xFF) ^ (uint8_t)((envelope_q15 >> 24) & 0xFF);
      checksum ^= (uint8_t)(phase & 0xFF) ^ (uint8_t)((phase >> 8) & 0xFF);
      checksum ^= pulseFlag;
      Serial.write(checksum);
    }
  }
}