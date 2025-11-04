#include <Arduino.h>

// Configuración de pines
const int ANALOG_INPUT_PIN = A0;  // Pin de entrada analógica
const int PWM_OUTPUT_PIN = 9;     // Pin de salida PWM (Timer1 - OC1A)

// Variables para el muestreo
volatile uint16_t analogValue = 0;
volatile uint16_t filteredValue = 0;
volatile bool newSampleReady = false;
// Coeficientes del filtro IIR en punto flotante (para prueba)
const float b0_f = 0.027477;
const float b1_f = 0.000000;
const float b2_f = -0.027477;
const float a1_f = -1.940814;
const float a2_f = 0.945045;

// Estados del filtro en punto flotante
float x1_f = 0.0;
float x2_f = 0.0;
float y1_f = 0.0;
float y2_f = 0.0;

// Configuración del ADC para lectura rápida
void setupADC() {
  // Configurar el ADC para lectura más rápida
  // Prescaler de 16 para frecuencia de ADC de 1 MHz (16MHz/16)
  ADCSRA &= ~0x07;  // Limpiar bits de prescaler
  ADCSRA |= 0x04;   // Establecer prescaler a 16

  ADMUX = 0;               // AREF, canal A0
  ADMUX |= (1 << REFS0);   // Referencia AVCC
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
ISR(TIMER2_COMPA_vect) {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  
  // Leer ADC (0-1023)
  int16_t adcValue = ADC;

  // Convertir a valor centrado en 0 (rango ±512)
  float x = (float)(adcValue - 512);

  // Filtro IIR Direct Form I en punto flotante
  // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
  float y = b0_f * x + b1_f * x1_f + b2_f * x2_f - a1_f * y1_f - a2_f * y2_f;

  // Actualizar estados
  x2_f = x1_f;
  x1_f = x;
  y2_f = y1_f;
  y1_f = y;

  // Amplificar para mejorar la visibilidad sin saturar
  // Ganancia del filtro es ~0.137, amplificar por 3-4 para evitar saturación
  float y_amplified = y * 3.5;

  // Re-centrar en 512 y saturar para PWM (0-1023)
  int16_t output = (int16_t)(y_amplified + 512.0);
  output = constrain(output, 0, 1023);

  OCR1A = output >> 2;

  analogValue = ADC;
  filteredValue = output;
  newSampleReady = true;
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
  if (newSampleReady) {
    newSampleReady = false;

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