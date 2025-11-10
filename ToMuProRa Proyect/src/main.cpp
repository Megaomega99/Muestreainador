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

// ===== FILTRO IIR PASA-BANDA (21 Hz ± 9 Hz, Q=2.333) =====
const int32_t b0_q15 = 456;
const int32_t b1_q15 = 0;
const int32_t b2_q15 = -456;
const int32_t a1_q15 = -64482;
const int32_t a2_q15 = 31855;

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

// ===== FILTRO DE HILBERT FIR =====
#define HILBERT_TAPS 15
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

// ===== PARÁMETROS DE DETECCIÓN =====
const int32_t MAGNITUDE_THRESHOLD = 50L << Q15_SHIFT;
const int16_t PHASE_THRESHOLD = 2730;
#define PERIOD_21HZ_MS 48
#define FILTER_DELAY_MS 10
#define PHASE_ADJUST_MS -1
#define PREDICTION_DELAY_MS (PERIOD_21HZ_MS - FILTER_DELAY_MS)
#define PREDICTION_DELAY_SAMPLES (PREDICTION_DELAY_MS * 2)

// ===== CONTROL DE PULSOS =====
volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 2;
uint16_t pulseCount = 0;
uint16_t peakDetectionCount = 0;
uint16_t refractoryCountdown = 0;

// ===== COLA DE PULSOS =====
#define PULSE_QUEUE_SIZE 4
uint16_t pulseQueue[PULSE_QUEUE_SIZE];
uint8_t pulseQueueHead = 0;
uint8_t pulseQueueTail = 0;
uint8_t pulseQueueCount = 0;

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

// ===== SETUP =====
void setup() {
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

    // Convertir a Q15
    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    // Filtro IIR pasa-banda
    int64_t num = ((int64_t)b0_q15 * x) + ((int64_t)b1_q15 * x1_q15) + ((int64_t)b2_q15 * x2_q15);
    num >>= Q15_SHIFT;
    int64_t den = ((int64_t)a1_q15 * y1_q15) + ((int64_t)a2_q15 * y2_q15);
    den >>= Q15_SHIFT;
    int32_t y_filtered = (int32_t)(num - den);

    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y_filtered;

    // Filtro de Hilbert
    hilbert_buffer[hilbert_index] = y_filtered;
    hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;

    int64_t hilbert_output = 0;
    uint8_t buf_idx = hilbert_index;
    for (int i = 0; i < HILBERT_TAPS; i++) {
      hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[buf_idx]);
      buf_idx = (buf_idx + 1) % HILBERT_TAPS;
    }
    hilbert_output >>= Q15_SHIFT;

    // Calcular magnitud y fase
    int32_t real_part = y_filtered;
    int32_t imag_part = (int32_t)hilbert_output;
    CordicResult cordic = fastCORDIC(real_part, imag_part);
    int32_t envelope = cordic.magnitude;
    if (envelope < 0) envelope = -envelope;
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
      pulseCount++;
      pulseQueueTail = (pulseQueueTail + 1) % PULSE_QUEUE_SIZE;
      pulseQueueCount--;
    }

    // Periodo refractario
    if (refractoryCountdown > 0) refractoryCountdown--;

    // Detección de flanco ascendente
    if (isPeak && !wasAboveThreshold && refractoryCountdown == 0) {
      peakDetectionCount++;
      if (pulseQueueCount < PULSE_QUEUE_SIZE) {
        pulseQueue[pulseQueueHead] = PREDICTION_DELAY_SAMPLES;
        pulseQueueHead = (pulseQueueHead + 1) % PULSE_QUEUE_SIZE;
        pulseQueueCount++;
      }
      refractoryCountdown = 20;
    }

    wasAboveThreshold = isPeak;

    // Salida PWM
    int32_t y_amplified = (y_filtered * 7) >> 1;
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;
    if (output < 0) output = 0;
    if (output > 1023) output = 1023;
    OCR1A = output >> 2;
  }
}
