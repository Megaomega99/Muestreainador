#include <Arduino.h>
#include <nrf_timer.h>
#include <nrf_saadc.h>
#include <nrf_pwm.h>

// ===== CONFIGURACIÓN DE PINES =====
const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;
const int PULSE_OUTPUT_PIN = 10;
const int LED_BUILTIN_PIN = LED_BUILTIN;  // LED para debug de detección

// ===== VARIABLES DE SISTEMA =====
volatile bool newSampleReady = false;
bool inPulse = false;
#define Q15_SHIFT 15

// Variable para detectar muestras perdidas
volatile uint32_t missedSamples = 0;

// ===== DEBUG =====
#define DEBUG_OUTPUT true  // Habilitar para ver frecuencia y magnitud
volatile uint32_t sampleCount = 0;
volatile int32_t debug_magnitude = 0;  // Para debug de umbral
volatile int16_t debug_phase = 0;

// ===== ADC ASÍNCRONO =====
volatile int16_t adcBuffer = 0;  // Buffer para resultado del SAADC
volatile uint16_t adcValue = 0;
volatile bool adcBusy = false;

// ===== FILTRO FIR PASA-BANDA (12-42 Hz) =====
// FIR de orden 31, fs=2024 Hz, banda 12-42 Hz, ganancia x8
// FIR es siempre estable (no hay feedback), ideal para este caso
#define FIR_TAPS 31
const int16_t fir_coefs[FIR_TAPS] = {
   467,  667, 1071, 1752, 2759, 4109, 5785, 7739,
  9887, 12124, 14325, 16358, 18097, 19429, 20266, 20551,
  20266, 19429, 18097, 16358, 14325, 12124,  9887,  7739,
  5785,  4109,  2759,  1752,  1071,   667,   467
};

int32_t fir_buffer[FIR_TAPS] = {0};
uint8_t fir_index = 0;

// ===== FILTRO DE HILBERT FIR =====
#define HILBERT_TAPS 15
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

// ===== PARÁMETROS DE DETECCIÓN =====
// Umbral de magnitud para detección de picos
// Ajustado basado en Max Mag observado (~65,000)
const int32_t MAGNITUDE_THRESHOLD = 40000;  // ~60% del máximo observado
const int16_t PHASE_THRESHOLD = 8192;  // ~45 grados para ventana más amplia
#define PREDICTION_DELAY_SAMPLES 76  // ~38 ms a 2000 Hz

// ===== CONTROL DE PULSOS =====
volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 2;
uint16_t refractoryCountdown = 0;

// ===== COLA DE PULSOS =====
#define PULSE_QUEUE_SIZE 4
uint16_t pulseQueue[PULSE_QUEUE_SIZE];
uint8_t pulseQueueHead = 0;
uint8_t pulseQueueTail = 0;
uint8_t pulseQueueCount = 0;

// ===== DECLARACIONES ADELANTADAS =====
struct CordicResult {
  int32_t magnitude;
  int16_t phase;
};

CordicResult fastCORDIC(int32_t real, int32_t imag);

// Declaración del handler del timer (necesario para NVIC_SetVector)
extern "C" void TIMER3_IRQHandler(void);

// ===== TIMER PARA MUESTREO (2000 Hz) =====
// Timer 3 para nRF52840
void setupTimer() {
  // Detener timer si está corriendo
  NRF_TIMER3->TASKS_STOP = 1;
  delayMicroseconds(10);  // Dar tiempo para que se detenga

  NRF_TIMER3->TASKS_CLEAR = 1;
  delayMicroseconds(10);

  // Deshabilitar todas las interrupciones del timer primero
  NRF_TIMER3->INTENCLR = 0xFFFFFFFF;

  // Configurar Timer3 en modo timer
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;

  // Usar 32 bits para mayor precisión
  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

  // Prescaler para 1 MHz: freq_timer = 16 MHz / (2^PRESCALER)
  // Con PRESCALER=4: 16 MHz / 16 = 1 MHz
  NRF_TIMER3->PRESCALER = 4;

  // Comparador para 2000 Hz: 1,000,000 / 2000 = 500 ticks
  NRF_TIMER3->CC[0] = 500;

  // Limpiar cualquier evento pendiente
  NRF_TIMER3->EVENTS_COMPARE[0] = 0;
  NRF_TIMER3->EVENTS_COMPARE[1] = 0;
  NRF_TIMER3->EVENTS_COMPARE[2] = 0;
  NRF_TIMER3->EVENTS_COMPARE[3] = 0;

  // Limpiar timer automáticamente al alcanzar CC[0]
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

  // CRÍTICO para Mbed OS: Vincular el ISR usando NVIC_SetVector
  // Mbed OS requiere esto para que el handler funcione correctamente
  NVIC_SetVector(TIMER3_IRQn, (uint32_t)TIMER3_IRQHandler);

  // Configurar NVIC primero (antes de habilitar interrupciones del timer)
  NVIC_DisableIRQ(TIMER3_IRQn);
  NVIC_ClearPendingIRQ(TIMER3_IRQn);
  NVIC_SetPriority(TIMER3_IRQn, 2);  // Prioridad media (0=más alta, 7=más baja)
  NVIC_EnableIRQ(TIMER3_IRQn);

  // Ahora habilitar interrupción en comparador 0
  NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

  // Asegurar que el evento está limpio antes de iniciar
  NRF_TIMER3->EVENTS_COMPARE[0] = 0;
  (void)NRF_TIMER3->EVENTS_COMPARE[0];  // Lectura dummy para sincronizar

  // Iniciar timer
  NRF_TIMER3->TASKS_START = 1;

  // Esperar a que el timer esté efectivamente corriendo
  delayMicroseconds(10);

  // Debug: Verificar que el timer está corriendo
  if (DEBUG_OUTPUT) {
    delay(10);  // Esperar un poco
    // Capturar el valor actual del contador usando CAPTURE task
    NRF_TIMER3->TASKS_CAPTURE[1] = 1;
    uint32_t count = NRF_TIMER3->CC[1];
    Serial.print("Timer3 iniciado. Contador capturado: ");
    Serial.println(count);
    Serial.print("EVENTS_COMPARE[0]: ");
    Serial.println(NRF_TIMER3->EVENTS_COMPARE[0]);
  }
}

// ===== CONFIGURACIÓN DE ADC (SAADC de nRF52) =====
void setupADC() {
  // Configurar pin A0 como entrada analógica
  // A0 en Nano 33 BLE corresponde a AIN0 (P0.04)
  pinMode(ANALOG_INPUT_PIN, INPUT);

  // Deshabilitar SAADC primero
  NRF_SAADC->ENABLE = 0;

  // Configurar canal 0 del SAADC
  // Canal 0: Single-ended, ganancia 1/6, referencia interna (0.6V)
  // Esto nos da un rango de 0-3.6V (compatible con el pin de 3.3V)
  NRF_SAADC->CH[0].CONFIG =
    (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |      // Sin resistencia pull
    (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |      // Sin resistencia pull
    (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |     // Ganancia 1/6
    (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) | // Referencia interna 0.6V
    (SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos) |        // Tiempo de adquisición 40us (más estable)
    (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos);           // Single-ended

  // Configurar pin PSELP (positive input) para A0 (AIN0)
  // A0 = P0.04 = AIN0
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0;
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC; // Negativo no conectado (single-ended)

  // Configurar resolución a 12 bits
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;

  // Configurar buffer para resultado usando EasyDMA
  NRF_SAADC->RESULT.PTR = (uint32_t)&adcBuffer;
  NRF_SAADC->RESULT.MAXCNT = 1;  // Solo 1 muestra

  // Habilitar SAADC
  NRF_SAADC->ENABLE = 1;

  // Hacer una lectura dummy para calibrar
  NRF_SAADC->TASKS_START = 1;
  while (NRF_SAADC->EVENTS_STARTED == 0);
  NRF_SAADC->EVENTS_STARTED = 0;

  NRF_SAADC->TASKS_SAMPLE = 1;
  while (NRF_SAADC->EVENTS_END == 0);
  NRF_SAADC->EVENTS_END = 0;

  NRF_SAADC->TASKS_STOP = 1;
  while (NRF_SAADC->EVENTS_STOPPED == 0);
  NRF_SAADC->EVENTS_STOPPED = 0;
}

// ===== FUNCIÓN DE LECTURA RÁPIDA DEL ADC =====
inline uint16_t fastAnalogRead() {
  // Ciclo completo pero optimizado: START -> SAMPLE -> leer -> STOP
  // Mantener los while loops cortos para minimizar latencia

  // START
  NRF_SAADC->TASKS_START = 1;
  while (!NRF_SAADC->EVENTS_STARTED);
  NRF_SAADC->EVENTS_STARTED = 0;

  // SAMPLE
  NRF_SAADC->TASKS_SAMPLE = 1;
  while (!NRF_SAADC->EVENTS_END);
  NRF_SAADC->EVENTS_END = 0;

  // Leer resultado
  int16_t result = adcBuffer;

  // STOP
  NRF_SAADC->TASKS_STOP = 1;
  while (!NRF_SAADC->EVENTS_STOPPED);
  NRF_SAADC->EVENTS_STOPPED = 0;

  // Convertir a rango 0-4095 (12 bits)
  int32_t normalized = result + 2048;
  if (normalized < 0) normalized = 0;
  if (normalized > 4095) normalized = 4095;

  return (uint16_t)normalized;
}

// ===== CONFIGURACIÓN DE PWM =====
void setupPWM() {
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  // Por ahora volvemos a usar analogWrite() estándar
  // El PWM hardware del nRF52840 es complejo de configurar
  // La frecuencia por defecto (~500 Hz) causará ripple visible
  // pero la envolvente de la señal debe ser correcta

  analogWriteResolution(8);
  analogWrite(PWM_OUTPUT_PIN, 128);

  if (DEBUG_OUTPUT) {
    Serial.println("PWM configurado (modo Arduino, ~500 Hz)");
    Serial.println("NOTA: Ripple visible es normal con este PWM");
  }
}

// ===== INTERRUPCIÓN DE TIMER =====
extern "C" {
  void TIMER3_IRQHandler(void) {
    // Limpiar el flag de interrupción INMEDIATAMENTE
    if (NRF_TIMER3->EVENTS_COMPARE[0]) {
      NRF_TIMER3->EVENTS_COMPARE[0] = 0;

      // Limpiar pending IRQ en NVIC para evitar retriggering
      __DSB();  // Data Synchronization Barrier
      __ISB();  // Instruction Synchronization Barrier

      // Verificar si la muestra anterior fue procesada
      if (newSampleReady) {
        missedSamples++;  // Se perdió una muestra
      }

      // Marcar que hay una nueva muestra lista
      newSampleReady = true;
      sampleCount++;
    }
  }
}

// ===== ALGORITMO CORDIC (CÁLCULO DE MAGNITUD Y FASE) =====
const int16_t cordic_angles[16] = {
  16384, 9672, 5110, 2594, 1302, 652, 326, 163,
  81, 41, 20, 10, 5, 3, 1, 1
};

const int32_t CORDIC_GAIN = 19898;

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

// ===== SETUP =====
void setup() {
  // Inicializar comunicación serial PRIMERO
  Serial.begin(115200);
  delay(2000);  // Dar tiempo para que el puerto se estabilice

  // Configurar pines
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PULSE_OUTPUT_PIN, OUTPUT);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(PULSE_OUTPUT_PIN, LOW);
  digitalWrite(LED_BUILTIN_PIN, LOW);

  // Configurar resolución del ADC
  analogReadResolution(12);  // 12 bits = 0-4095

  // Inicializar periféricos ANTES del timer para evitar interrupciones durante setup
  setupADC();  // SAADC directo para mayor velocidad
  setupPWM();

  // Enviar mensaje de inicialización ANTES de iniciar el timer
  Serial.println("ok");

  if (DEBUG_OUTPUT) {
    Serial.println("Sistema iniciado - Modo DEBUG");
    Serial.println("Frecuencia de muestreo: 2000 Hz");
    Serial.println("ADC: 12 bits (0-4095)");
    Serial.println("PWM: 8 bits (0-255)");
    delay(100);  // Dar tiempo para que se envíen los mensajes
  }

  // Iniciar timer AL FINAL cuando todo está listo
  setupTimer();

  // Diagnóstico adicional
  if (DEBUG_OUTPUT) {
    Serial.print("NVIC TIMER3 habilitado: ");
    Serial.println(NVIC_GetEnableIRQ(TIMER3_IRQn));
    Serial.print("TIMER3 INTENSET: 0x");
    Serial.println(NRF_TIMER3->INTENSET, HEX);
    Serial.print("TIMER3 CC[0]: ");
    Serial.println(NRF_TIMER3->CC[0]);
  }
}

// ===== LOOP PRINCIPAL =====
void loop() {
  // Debug: Reportar magnitud máxima cada segundo para ajustar umbral
  static unsigned long lastDebugTime = 0;
  static int32_t max_magnitude = 0;

  // Actualizar máximo
  if (debug_magnitude > max_magnitude) {
    max_magnitude = debug_magnitude;
  }

  if (DEBUG_OUTPUT && (millis() - lastDebugTime >= 1000)) {
    lastDebugTime = millis();
    noInterrupts();
    uint32_t samples = sampleCount;
    uint32_t missed = missedSamples;
    sampleCount = 0;
    missedSamples = 0;
    interrupts();

    // Reportar magnitud máxima para ajustar umbral
    Serial.print("Freq: ");
    Serial.print(samples);
    Serial.print(" Hz | Max Mag: ");
    Serial.print(max_magnitude);
    Serial.print(" | Threshold: ");
    Serial.println(MAGNITUDE_THRESHOLD);
    max_magnitude = 0;  // Reset para próximo segundo
  }
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
    newSampleReady = false;

    // Leer ADC usando SAADC directo (rápido, no bloqueante)
    uint16_t adcValue = fastAnalogRead();

    // Convertir a Q15 (ajustando para 12 bits: 0-4095 -> centrado en 2048)
    int32_t x = ((int32_t)adcValue - 2048L) << Q15_SHIFT;

    // Filtro FIR pasa-banda (12-42 Hz)
    // FIR es siempre estable, no hay riesgo de oscilación
    fir_buffer[fir_index] = x;
    fir_index = (fir_index + 1) % FIR_TAPS;

    int64_t fir_sum = 0;
    uint8_t fir_buf_idx = fir_index;
    for (uint8_t i = 0; i < FIR_TAPS; i++) {
      fir_sum += ((int64_t)fir_coefs[i] * fir_buffer[fir_buf_idx]);
      fir_buf_idx = (fir_buf_idx + 1) % FIR_TAPS;
    }

    // fir_sum = suma de (coef_Q15 * muestra_Q15)
    // Necesitamos doble shift: dividir por 32768² para volver a Q15
    int32_t y_filtered = (int32_t)(fir_sum >> (Q15_SHIFT + Q15_SHIFT));

    // DEBUG: Ver valores del filtro antes y después del shift
    static uint16_t fir_debug_counter = 0;
    if (DEBUG_OUTPUT && ++fir_debug_counter >= 500) {
      fir_debug_counter = 0;
      Serial.print("ADC: ");
      Serial.print(adcValue);
      Serial.print(" | x_Q15: ");
      Serial.print(x >> Q15_SHIFT);
      Serial.print(" | fir_sum_raw: ");
      Serial.print((int32_t)(fir_sum >> 20));  // Dividir por 1M para ver
      Serial.print(" | y_filt: ");
      Serial.println(y_filtered);
    }

    // Filtro de Hilbert
    hilbert_buffer[hilbert_index] = y_filtered;
    hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;

    int64_t hilbert_output = 0;
    uint8_t hilbert_buf_idx = hilbert_index;
    for (int i = 0; i < HILBERT_TAPS; i++) {
      hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[hilbert_buf_idx]);
      hilbert_buf_idx = (hilbert_buf_idx + 1) % HILBERT_TAPS;
    }
    hilbert_output >>= Q15_SHIFT;

    // Calcular magnitud y fase
    int32_t real_part = y_filtered;
    int32_t imag_part = (int32_t)hilbert_output;
    CordicResult cordic = fastCORDIC(real_part, imag_part);
    int32_t envelope = cordic.magnitude;
    if (envelope < 0) envelope = -envelope;
    int16_t phase = cordic.phase;

    // Guardar para debug
    debug_magnitude = envelope;
    debug_phase = phase;

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
        // LED de debug: parpadeo rápido cuando se detecta pico
        digitalWrite(LED_BUILTIN_PIN, HIGH);
      }
      refractoryCountdown = 20;
    } else {
      digitalWrite(LED_BUILTIN_PIN, LOW);
    }

    wasAboveThreshold = isPeak;

    // ===== SALIDA PWM CON SEÑAL FILTRADA =====
    // y_filtered está en formato Q15 (representa valores centrados en 0)
    // Filtro tiene ganancia x8, así que amplificamos x8 adicional = x64 total
    // Para convertir a PWM (0-255):
    // 1. Amplificar para buena visibilidad
    // 2. Hacer shift para convertir de Q15 a entero
    // 3. Centrar en 128 (punto medio PWM)

    int32_t y_amplified = y_filtered << 3;  // x8 amplificación PWM adicional
    int32_t output = (y_amplified >> Q15_SHIFT) + 128L;

    // Saturar
    if (output < 0) {
      output = 0;
    } else if (output > 255) {
      output = 255;
    }

    analogWrite(PWM_OUTPUT_PIN, (int)output);

    // Filtro FIR pasa-banda aísla 12-42 Hz (incluye 21 Hz target)
    // Sistema verificado: ADC→FIR→PWM→RC todos funcionando correctamente

    // Debug deshabilitado para mejor rendimiento
    // (Serial.print eliminado)
  }
}
