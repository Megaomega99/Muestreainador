#include <Arduino.h>

const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;
const int PULSE_OUTPUT_PIN = 10;

volatile uint16_t rawSample = 0;
volatile bool newSampleReady = false;
bool inPulse = false;
#define Q15_SHIFT 15

// Filtro pasa-banda IIR - 21 Hz ± 9 Hz (Q=2.333)
// Rango: 12 Hz - 30 Hz
const int32_t b0_q15 = 456;
const int32_t b1_q15 = 0;
const int32_t b2_q15 = -456;
const int32_t a1_q15 = -64482;
const int32_t a2_q15 = 31855;

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

#define HILBERT_TAPS 15
// Filtro de Hilbert FIR optimizado para 12-30 Hz
// Diseñado con firwin2 para ganancia constante en banda de paso
// Coeficientes en Q15, antisimétricos (centro = 0)
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};

int32_t hilbert_buffer[HILBERT_TAPS] = {0};
uint8_t hilbert_index = 0;

// Umbral de magnitud para detección de picos
// Ajustar experimentalmente según amplitud de señal de 21 Hz
const int32_t MAGNITUDE_THRESHOLD = 40L << Q15_SHIFT;
// Umbral de fase más estricto: ±10° en Q15 (±1820 en int16)
// Esto asegura que solo detectamos cerca del pico real
const int16_t PHASE_THRESHOLD = 1820;  // ~10° (antes: 5000 = 27.5°)

#define PERIOD_21HZ_MS 48
// Retardo calculado del sistema de filtros (ver Fitros.ipynb):
// - Filtro IIR (Q=2.333): ~13 muestras = 6.5 ms
// - Filtro Hilbert FIR: 7 muestras = 3.5 ms
// - TOTAL: ~20 muestras = 10 ms
#define FILTER_DELAY_MS 10

// Ajuste fino de fase del pulso (en ms)
// Valores positivos: pulso más tarde (después del pico)
// Valores negativos: pulso más temprano (antes del pico)
// Ajustar este valor si el pulso no coincide exactamente con el pico
#define PHASE_ADJUST_MS -1

// Cálculo de cuándo llegará el siguiente pico
// Detectamos pico en señal filtrada → predecimos siguiente pico original
#define PREDICTION_DELAY_MS (PERIOD_21HZ_MS - FILTER_DELAY_MS + PHASE_ADJUST_MS)
#define PREDICTION_DELAY_SAMPLES (PREDICTION_DELAY_MS * 2)

struct PeakPredictor {
  uint16_t countdown;  // Cambiado a uint16_t para soportar delays mayores
  bool active;
};

PeakPredictor peakPredictor = {0, false};

volatile bool triggerPulse = false;
unsigned long pulseStartTime = 0;
const unsigned long PULSE_DURATION = 2;

// Configura ADC a 2000 Hz con interrupciones
void setupADC() {
  ADCSRA &= ~0x07;
  ADCSRA |= 0x04;
  ADMUX = 0;
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADEN);  // Habilitar el ADC
}

// Configura Timer2 para disparar ADC a 2000 Hz
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

// Configura PWM en pin 9 para salida de señal filtrada
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

// Tabla de ángulos CORDIC precalculados (en formato Q15)
const int16_t cordic_angles[16] = {
  16384, 9672, 5110, 2594, 1302, 652, 326, 163,
  81, 41, 20, 10, 5, 3, 1, 1
};

// Factor de escala CORDIC: 1/K ≈ 0.6072529350 en Q15
const int32_t CORDIC_GAIN = 19898;

// Estructura para resultado CORDIC (magnitud y fase)
struct CordicResult {
  int32_t magnitude;
  int16_t phase;
};

// Calcula magnitud y fase simultáneamente usando CORDIC (vectoring mode)
CordicResult fastCORDIC(int32_t real, int32_t imag) {
  CordicResult result = {0, 0};

  if (real == 0 && imag == 0) return result;

  // Normalizar para evitar overflow
  int32_t abs_real = real >= 0 ? real : -real;
  int32_t abs_imag = imag >= 0 ? imag : -imag;
  int32_t max_val = abs_real > abs_imag ? abs_real : abs_imag;

  uint8_t shift = 0;
  if (max_val > (1L << 20)) {
    shift = 5;
    real >>= shift;
    imag >>= shift;
  }

  // Determinar cuadrante y ajustar
  int16_t angle = 0;
  int32_t x = real;
  int32_t y = imag;

  // Rotar al primer cuadrante
  if (x < 0) {
    x = -x;
    y = -y;
    angle = (y >= 0) ? 32768 : -32768;  // ±180°
  }

  // Algoritmo CORDIC vectoring mode
  for (uint8_t i = 0; i < 16; i++) {
    int32_t x_new, y_new;

    if (y < 0) {
      // Rotar en sentido horario
      x_new = x - (y >> i);
      y_new = y + (x >> i);
      angle -= cordic_angles[i];
    } else {
      // Rotar en sentido antihorario
      x_new = x + (y >> i);
      y_new = y - (x >> i);
      angle += cordic_angles[i];
    }

    x = x_new;
    y = y_new;
  }

  // La magnitud es el valor final de x (compensado por ganancia CORDIC)
  // x_final = magnitud * K, donde K ≈ 1.646760258
  // magnitud = x_final / K = x_final * (1/K)
  result.magnitude = (x * CORDIC_GAIN) >> Q15_SHIFT;
  result.magnitude <<= shift;  // Restaurar escala original
  result.phase = angle;

  return result;
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
    inPulse = true;
  }

  if (inPulse) {
    if (millis() - pulseStartTime >= PULSE_DURATION) {
      digitalWrite(PULSE_OUTPUT_PIN, LOW);
      inPulse = false;
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

    // Calcular magnitud y fase simultáneamente con CORDIC
    CordicResult cordic = fastCORDIC(real_part, imag_part);
    int32_t envelope = cordic.magnitude;
    int16_t phase = cordic.phase;

    // Detectar pico en fase ±180° (inversión por filtro de Hilbert)
    // En lugar de detectar fase=0°, detectamos cuando |fase| está cerca de 180°
    bool isPeak = (envelope > MAGNITUDE_THRESHOLD) &&
                  ((phase > (32768 - PHASE_THRESHOLD)) || (phase < (-32768 + PHASE_THRESHOLD)));

    static bool wasAboveThreshold = false;  // Cambio: inicializar en false

    if (isPeak && !wasAboveThreshold && !peakPredictor.active) {
      peakPredictor.countdown = PREDICTION_DELAY_SAMPLES;
      peakPredictor.active = true;
    }
    wasAboveThreshold = isPeak;  // Cambio: usar isPeak en lugar de solo magnitud

    if (peakPredictor.active) {
      if (peakPredictor.countdown > 0) {
        peakPredictor.countdown--;
      } else {
        triggerPulse = true;
        peakPredictor.active = false;
      }
    }

    // Salida PWM: señal filtrada amplificada
    int32_t y_amplified = (y_filtered * 7) >> 1;
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;

    if (output < 0) output = 0;
    if (output > 1023) output = 1023;

    OCR1A = output >> 2;
  }
}
