#include <Arduino.h>

const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;
const int PULSE_OUTPUT_PIN = 10;

volatile uint16_t rawSample = 0;
volatile bool newSampleReady = false;

#define Q15_SHIFT 15

const int32_t b0_q15 = 900;
const int32_t b1_q15 = 0;
const int32_t b2_q15 = -900;
const int32_t a1_q15 = -63597;
const int32_t a2_q15 = 30967;

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

#define HILBERT_TAPS 15
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

const int32_t MAGNITUDE_THRESHOLD = 70L << Q15_SHIFT;
const int16_t PHASE_THRESHOLD = 5000;

// ==============================================================================
// COMPENSACIÓN DEL RETARDO DEL SISTEMA - PREDICCIÓN DEL SIGUIENTE PICO
// ==============================================================================
// El sistema introduce un retardo debido a los filtros:
//   - Filtro IIR pasa-banda: ~13 muestras (6.5 ms)
//   - Filtro Hilbert (FIR):   7 muestras (3.5 ms)
//   - TOTAL:                 ~20 muestras (10 ms)
//
// Problema: Cuando detectamos fase=0°, el pico original ya pasó hace 10ms.
//
// Solución: PREDECIR el SIGUIENTE pico
//   - Período de 21 Hz = 47.6 ms
//   - Cuando detectamos fase=0°, sabemos que el pico ocurrió hace 10ms
//   - El SIGUIENTE pico ocurrirá en: 47.6ms - 10ms = 37.6ms ≈ 38ms
//   - Esperamos 38ms y disparamos el pulso sincronizado!
//
// Timeline con PREDICCIÓN:
//   t=0ms    : Pico REAL #1 en señal de entrada
//   t=10ms   : Detectamos fase=0° (pico filtrado) → INICIAMOS COUNTDOWN
//   t=48ms   : Pico REAL #2 en señal de entrada → ¡PULSO SINCRONIZADO!
//
#define PERIOD_21HZ_MS 48           // Período de 21 Hz (1000/21 ≈ 47.6ms)
#define FILTER_DELAY_MS 10          // Retardo de filtros IIR + Hilbert

// ==============================================================================
// AJUSTE DE FASE DEL PULSO (en milisegundos)
// ==============================================================================
// Ajusta cuándo se dispara el pulso respecto al pico de la señal:
//   PHASE_ADJUST_MS = 0   → Pulso en el PICO (máximo)
//   PHASE_ADJUST_MS > 0   → Pulso DESPUÉS del pico (retraso)
//   PHASE_ADJUST_MS < 0   → Pulso ANTES del pico (adelanto)
//
// Tabla de referencia rápida:
//   Desfase  |  PHASE_ADJUST_MS  |  Momento del pulso
//   ---------|-------------------|-------------------
//      0°    |        0          |  Pico máximo (por defecto)
//     45°    |        6          |  Bajada después del pico
//     90°    |       12          |  Cruce por cero descendente
//    135°    |       18          |  Acercándose al valle
//    180°    |       24          |  Valle mínimo
//    -45°    |       -6          |  Subida antes del pico
//    -90°    |      -12          |  Cruce por cero ascendente
//
// Conversión: 1 ms ≈ 7.5° a 21 Hz (período = 48 ms)
//
#define PHASE_ADJUST_MS 0  // ← MODIFICA ESTE VALOR (rango: -48 a +48 ms)

#define PREDICTION_DELAY_MS (PERIOD_21HZ_MS - FILTER_DELAY_MS + PHASE_ADJUST_MS)
#define PREDICTION_DELAY_SAMPLES (PREDICTION_DELAY_MS * 2)      // Muestras a 2000Hz

// Estructura para predicción del siguiente pico
struct PeakPredictor {
  uint8_t countdown;
  bool active;
};

PeakPredictor peakPredictor = {0, false};

volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 5;

void setupADC() {
  ADCSRA &= ~0x07;
  ADCSRA |= 0x04;
  ADMUX = 0;
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADIE);
}

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

ISR(TIMER2_COMPA_vect) {
  ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
  rawSample = ADC;
  newSampleReady = true;
}

int32_t fastMagnitude(int32_t real, int32_t imag) {
  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;
  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;
  int32_t min_val = abs_real < abs_imag ? abs_real : abs_imag;
  return max_val + ((min_val * 13107) >> Q15_SHIFT);
}

int16_t fastPhase(int32_t real, int32_t imag) {
  if (real == 0 && imag == 0) return 0;

  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;
  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;

  if (max_val > (1L << 20)) {
    real >>= 5;
    imag >>= 5;
    abs_real = real >= 0 ? real : -real;
    abs_imag = imag >= 0 ? imag : -imag;
  }

  int32_t phase;
  if (abs_real > abs_imag) {
    if (real == 0) return 0;
    int32_t ratio = (imag << Q15_SHIFT) / real;
    phase = (ratio * 10430) >> Q15_SHIFT;
    if (real < 0) {
      phase = (imag >= 0) ? (32768 + phase) : (-32768 + phase);
    }
  } else {
    if (imag == 0) return (real >= 0) ? 0 : 32767;
    int32_t ratio = (real << Q15_SHIFT) / imag;
    phase = 16384 - ((ratio * 10430) >> Q15_SHIFT);
    if (imag < 0) phase = -phase;
  }

  return (int16_t)phase;
}

void setup() {
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PULSE_OUTPUT_PIN, OUTPUT);
  digitalWrite(PULSE_OUTPUT_PIN, LOW);
  setupADC();
  setupPWM();
  setupTimer2();
}

void loop() {
  if (triggerPulse) {
    digitalWrite(PULSE_OUTPUT_PIN, HIGH);
    pulseStartTime = millis();
    triggerPulse = false;
  }

  if (digitalRead(PULSE_OUTPUT_PIN) == HIGH) {
    if (millis() - pulseStartTime >= PULSE_DURATION) {
      digitalWrite(PULSE_OUTPUT_PIN, LOW);
    }
  }

  if (newSampleReady) {
    noInterrupts();
    newSampleReady = false;
    uint16_t adcValue = rawSample;
    interrupts();

    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    int64_t num = ((int64_t)b0_q15 * x) +
                  ((int64_t)b1_q15 * x1_q15) +
                  ((int64_t)b2_q15 * x2_q15);
    num >>= Q15_SHIFT;

    int64_t den = ((int64_t)a1_q15 * y1_q15) +
                  ((int64_t)a2_q15 * y2_q15);
    den >>= Q15_SHIFT;

    int32_t y_filtered = (int32_t)(num - den);

    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y_filtered;

    hilbert_buffer[hilbert_index] = y_filtered;
    hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;

    int64_t hilbert_output = 0;
    uint8_t buf_idx = hilbert_index;
    for (int i = 0; i < HILBERT_TAPS; i++) {
      hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[buf_idx]);
      buf_idx = (buf_idx + 1) % HILBERT_TAPS;
    }
    hilbert_output >>= Q15_SHIFT;

    int32_t real_part = y_filtered;
    int32_t imag_part = (int32_t)hilbert_output;

    int32_t envelope = fastMagnitude(real_part, imag_part);
    int16_t phase = fastPhase(real_part, imag_part);

    // Detectar pico en señal filtrada (fase cercana a 0°)
    bool isPeak = (envelope > MAGNITUDE_THRESHOLD) &&
                  (phase > -PHASE_THRESHOLD && phase < PHASE_THRESHOLD);

    // VERSIÓN SIMPLE: Disparar inmediatamente cuando se detecta pico
    static bool wasAboveThreshold = false;
    if (isPeak && !wasAboveThreshold) {
      triggerPulse = true;
    }
    wasAboveThreshold = (envelope > MAGNITUDE_THRESHOLD);

    int32_t y_amplified = (y_filtered * 7) >> 1;
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;

    if (output < 0) output = 0;
    if (output > 1023) output = 1023;

    OCR1A = output >> 2;
  }
}