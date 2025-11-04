#include <Arduino.h>

// Configuración de pines
const int ANALOG_INPUT_PIN = A0;  // Pin de entrada analógica
const int PWM_OUTPUT_PIN = 9;     // Pin de salida PWM (Timer1 - OC1A)

// Variables para el muestreo
volatile uint16_t rawSample = 0;
volatile uint16_t analogValue = 0;
volatile uint16_t filteredValue = 0;
volatile bool newSampleReady = false;

// ============================================================================
// FILTRO IIR EN PUNTO FIJO - Optimizado para ejecución en tiempo real
// ============================================================================
// Coeficientes originales en punto flotante:
// b0 = 0.027477, b1 = 0.0, b2 = -0.027477
// a1 = -1.940814, a2 = 0.945045

// Conversión a punto fijo Q15 (rango: -1.0 a +0.99997)
// Factor de escala: 2^15 = 32768
#define Q15_SHIFT 15
#define Q15_SCALE 32768L

// Coeficientes en punto fijo Q15
const int16_t b0_q15 = 900;      // 0.027477 * 32768 ≈ 900
const int16_t b1_q15 = 0;        // 0.0
const int16_t b2_q15 = -900;     // -0.027477 * 32768 ≈ -900
const int16_t a1_q15 = -31677;   // -1.940814 * 32768 / 2 ≈ -31677 (escalado para evitar overflow)
const int16_t a2_q15 = 15467;    // 0.945045 * 32768 / 2 ≈ 15467 (escalado para evitar overflow)

// Estados del filtro en punto fijo Q15 (usando int32_t para acumulación)
int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

// Configuración del ADC para lectura rápida con interrupción
void setupADC() {
  // Configurar el ADC para lectura más rápida
  // Prescaler de 16 para frecuencia de ADC de 1 MHz (16MHz/16)
  ADCSRA &= ~0x07;  // Limpiar bits de prescaler
  ADCSRA |= 0x04;   // Establecer prescaler a 16

  ADMUX = 0;               // AREF, canal A0
  ADMUX |= (1 << REFS0);   // Referencia AVCC

  // Habilitar interrupción de ADC
  ADCSRA |= (1 << ADIE);   // ADC Interrupt Enable
}

// Configuración del Timer2 para interrupciones a exactamente 2 KHz
void setupTimer2() {
  // Deshabilitar interrupciones temporalmente
  noInterrupts();

  // Configurar Timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // Configurar para modo CTC (Clear Timer on Compare Match)
  // Frecuencia = 16MHz / (prescaler * (OCR2A + 1))
  // Para 2 KHz: OCR2A = 16000000 / (64 * 2000) - 1 = 124
  OCR2A = 124;  // Valor de comparación para 2 KHz

  TCCR2A |= (1 << WGM21);   // Modo CTC
  TCCR2B |= (1 << CS22);    // Prescaler = 64
  TIMSK2 |= (1 << OCIE2A);  // Habilitar interrupción por comparación

  // Habilitar interrupciones
  interrupts();
}

// Configuración del PWM en Timer1 (Pin 9) a frecuencia fija ~31 kHz
void setupPWM() {
  // Configurar pin 9 como salida
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  // Timer1 en modo Fast PWM de 8 bits
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= (1 << COM1A1);  // Non-inverting mode en OC1A (pin 9)
  TCCR1A |= (1 << WGM10);   // Fast PWM 8-bit
  TCCR1B |= (1 << WGM12);   // Fast PWM 8-bit
  TCCR1B |= (1 << CS10);    // No prescaler, frecuencia PWM ~31 kHz

  OCR1A = 0;  // Iniciar en 0
}

// Interrupción del Timer2 - Se ejecuta cada 500 µs (exactamente 2 KHz)
// SOLO INICIAR CONVERSIÓN ADC - Sin bloqueos
ISR(TIMER2_COMPA_vect) {
  ADCSRA |= (1 << ADSC);  // Solo iniciar ADC (~104 µs hasta completar)
}

// Interrupción del ADC - Se ejecuta cuando termina la conversión (~104 µs después)
// SOLO CAPTURAR MUESTRA - Sin procesamiento
ISR(ADC_vect) {
  rawSample = ADC;        // Solo capturar valor (0-1023)
  newSampleReady = true;  // Señalizar que hay muestra disponible
}

void setup() {
  // Inicializar comunicación serial
  Serial.begin(115200);

  // Configurar pines
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  // Configurar ADC para lectura rápida
  setupADC();

  // Configurar PWM en Timer1 (Pin 9) a ~31 kHz
  setupPWM();

  // Configurar Timer2 para interrupciones exactamente a 2 KHz
  setupTimer2();
}

void loop() {
  // PROCESAMIENTO DISTRIBUIDO - Solo cuando hay muestra lista
  if (newSampleReady) {
    newSampleReady = false;

    // Leer muestra capturada por ISR(ADC_vect)
    uint16_t adcValue = rawSample;

    // ========================================================================
    // AQUÍ PROCESAR FILTRO IIR - En contexto del loop, sin bloquear ISRs
    // ========================================================================

    // Convertir a valor centrado en 0 y escalar a Q15
    // Entrada: 0-1023 → -512 a +511
    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    // Filtro IIR Direct Form I en punto fijo Q15
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]

    // Calcular términos del numerador (feedforward)
    int32_t num = ((int32_t)b0_q15 * (x >> Q15_SHIFT)) +
                  ((int32_t)b1_q15 * (x1_q15 >> Q15_SHIFT)) +
                  ((int32_t)b2_q15 * (x2_q15 >> Q15_SHIFT));

    // Calcular términos del denominador (feedback) - con escalado adicional
    int32_t den = ((int32_t)a1_q15 * (y1_q15 >> (Q15_SHIFT - 1))) +
                  ((int32_t)a2_q15 * (y2_q15 >> (Q15_SHIFT - 1)));

    // Salida del filtro
    int32_t y = num - den;

    // Actualizar estados del filtro
    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y;

    // Amplificar para mejorar la visibilidad (ganancia × 3.5)
    // Multiplicar por 3.5 = multiplicar por 7 y dividir por 2
    int32_t y_amplified = (y * 7) >> 1;

    // Convertir de Q15 a rango 0-1023
    // Descalar de Q15 y re-centrar en 512
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;

    // Saturar a rango válido para PWM (0-1023)
    if (output < 0) output = 0;
    if (output > 1023) output = 1023;

    // ========================================================================
    // AQUÍ ACTUALIZAR PWM - También en contexto del loop
    // ========================================================================
    OCR1A = output >> 2;  // Convertir 0-1023 a 0-255 para PWM

    // Guardar valores para transmisión serial
    analogValue = adcValue;
    filteredValue = (uint16_t)output;

    // ========================================================================
    // TRANSMISIÓN SERIAL - Sin bloquear el procesamiento crítico
    // ========================================================================
    static unsigned long lastPrint = 0;
    static uint16_t sampleCounter = 0;

    sampleCounter++;

    // Enviar cada 10 muestras (200 Hz)
    if (sampleCounter >= 10 || (millis() - lastPrint >= 50)) {
      // Calcular voltaje de entrada (0-5V)
      float inputVoltage = (analogValue * 5.0) / 1023.0;

      // Calcular voltaje de salida filtrada (0-5V)
      float outputVoltage = (filteredValue * 5.0) / 1023.0;

      // Formato para Serial Plotter
      Serial.print("Entrada_V:");
      Serial.print(inputVoltage, 3);
      Serial.print(" ");
      Serial.print("Salida_V:");
      Serial.println(outputVoltage, 3);

      lastPrint = millis();
      sampleCounter = 0;
    }
  }
}