#include <Arduino.h>

const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;
const int PULSE_OUTPUT_PIN = 10;

volatile uint16_t rawSample = 0;
volatile bool newSampleReady = false;

#define Q15_SHIFT 15

// Coeficientes del filtro IIR pasa-banda (centrado en 21 Hz)
const int32_t b0_q15 = 900;      // 0.027477
const int32_t b1_q15 = 0;        // 0.000000
const int32_t b2_q15 = -900;     // -0.027477
const int32_t a1_q15 = -63597;   // -1.940814
const int32_t a2_q15 = 30967;    // 0.945045

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

// Filtro de Hilbert FIR (15 taps, optimizado para ~21Hz)
// Coeficientes en Q15 para desfase de 90 grados
#define HILBERT_TAPS 15
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328,   // -0.01
  0,
  -984,   // -0.03
  0,
  -2621,  // -0.08
  0,
  -6554,  // -0.20
  0,
  6554,   // 0.20
  0,
  2621,   // 0.08
  0,
  984,    // 0.03
  0,
  328     // 0.01
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

// Umbral de detección (ajustable)
const int32_t MAGNITUDE_THRESHOLD = 70L << Q15_SHIFT;

// Variable para controlar el pulso de salida
volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 5; // ms

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

// Función para calcular la magnitud aproximada (evita sqrt)
// Usa la aproximación: mag ≈ max(|I|, |Q|) + 0.4*min(|I|, |Q|)
int32_t fastMagnitude(int32_t real, int32_t imag) {
  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;

  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;
  int32_t min_val = abs_real < abs_imag ? abs_real : abs_imag;

  return max_val + ((min_val * 13107) >> Q15_SHIFT); // 0.4 ≈ 13107/32768
}

// Función para calcular la fase usando atan2 aproximado
// Retorna fase en el rango [-32768, 32767] representando [-π, π]
int16_t fastPhase(int32_t real, int32_t imag) {
  if (real == 0 && imag == 0) return 0;

  // Normalizar para evitar overflow
  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;
  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;

  if (max_val > (1L << 20)) {
    real >>= 5;
    imag >>= 5;
    abs_real = real >= 0 ? real : -real;
    abs_imag = imag >= 0 ? imag : -imag;
  }

  // Aproximación de atan2 usando series
  int32_t phase;
  if (abs_real > abs_imag) {
    if (real == 0) return 0;  // Protección contra división por cero
    int32_t ratio = (imag << Q15_SHIFT) / real;
    phase = (ratio * 10430) >> Q15_SHIFT; // atan(x) ≈ 0.318*x para x pequeño
    if (real < 0) {
      phase = (imag >= 0) ? (32768 + phase) : (-32768 + phase);
    }
  } else {
    if (imag == 0) return (real >= 0) ? 0 : 32767;  // Protección contra división por cero
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
  // Manejo del pulso de salida
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

    // Aplicar filtro IIR pasa-banda Direct Form I en Q15
    // x[n] centrado y escalado a Q15
    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    // Calcular numerador: b0*x[n] + b1*x[n-1] + b2*x[n-2]
    // Multiplicación Q15 x Q15 = Q30, luego >> 15 = Q15
    int64_t num = ((int64_t)b0_q15 * x) +
                  ((int64_t)b1_q15 * x1_q15) +
                  ((int64_t)b2_q15 * x2_q15);
    num = num >> Q15_SHIFT;  // Escalar de Q30 a Q15

    // Calcular denominador: -a1*y[n-1] - a2*y[n-2]
    int64_t den = ((int64_t)a1_q15 * y1_q15) +
                  ((int64_t)a2_q15 * y2_q15);
    den = den >> Q15_SHIFT;  // Escalar de Q30 a Q15

    // y[n] = numerador - denominador
    int32_t y_filtered = (int32_t)(num - den);

    // Actualizar estados del filtro
    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y_filtered;

    // Aplicar Transformada de Hilbert (FIR)
    hilbert_buffer[hilbert_index] = y_filtered;
    hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;

    int64_t hilbert_output = 0;
    uint8_t buf_idx = hilbert_index;
    for (int i = 0; i < HILBERT_TAPS; i++) {
      hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[buf_idx]);
      buf_idx = (buf_idx + 1) % HILBERT_TAPS;
    }
    hilbert_output = hilbert_output >> Q15_SHIFT;  // Escalar de Q30 a Q15

    // Señal analítica compleja: z = y_filtered + j*hilbert_output
    int32_t real_part = y_filtered;      // Componente I (In-phase)
    int32_t imag_part = hilbert_output;  // Componente Q (Quadrature)

    // Calcular envolvente (magnitud)
    int32_t envelope = fastMagnitude(real_part, imag_part);

    // Calcular fase instantánea
    int16_t phase = fastPhase(real_part, imag_part);

    // Detección de pico: magnitud supera umbral y fase cerca del máximo (0° o ±180°)
    // Consideramos pico cuando phase está cerca de 0 (máximo positivo)
    bool isPeak = (envelope > MAGNITUDE_THRESHOLD) && (phase > -5000 && phase < 5000);

    static bool wasAboveThreshold = false;
    if (isPeak && !wasAboveThreshold) {
      triggerPulse = true;
    }
    wasAboveThreshold = (envelope > MAGNITUDE_THRESHOLD);

    // Salida PWM (señal filtrada para monitoreo)
    int32_t y_amplified = (y_filtered * 7) >> 1;
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;

    if (output < 0) output = 0;
    if (output > 1023) output = 1023;

    OCR1A = output >> 2;
  }
}