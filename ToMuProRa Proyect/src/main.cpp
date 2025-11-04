#include <Arduino.h>

// Configuración de pines
const int ANALOG_INPUT_PIN = A0;  // Pin de entrada analógica
const int PWM_OUTPUT_PIN = 9;     // Pin de salida PWM (Timer1 - OC1A)

// Variables para el muestreo
volatile uint16_t analogValue = 0;
volatile bool newSampleReady = false;

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
  // Iniciar conversión ADC
  ADCSRA |= (1 << ADSC);

  // Esperar a que termine la conversión (toma ~13 µs con prescaler 16)
  while (ADCSRA & (1 << ADSC));

  // Leer valor del ADC (0-1023)
  analogValue = ADC;
  newSampleReady = true;

  // Convertir de 10 bits (0-1023) a 8 bits (0-255) para PWM
  uint8_t pwmValue = analogValue >> 2;

  // Actualizar salida PWM directamente en el registro
  OCR1A = pwmValue;
}

void setup() {
  // Inicializar comunicación serial para debug
  Serial.begin(115200);
  Serial.println("Sistema de adquisición a 2 KHz con PWM iniciado");

  // Configurar pines
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  // Configurar ADC para lectura rápida
  setupADC();

  // Configurar PWM en Timer1 (Pin 9) a ~31 kHz
  setupPWM();

  // Configurar Timer2 para interrupciones exactamente a 2 KHz
  setupTimer2();

  Serial.println("Configuración completa");
  Serial.println("Pin entrada: A0");
  Serial.println("Pin salida PWM: D9");
  Serial.println("Frecuencia PWM: ~31 kHz");
  Serial.println("Frecuencia de muestreo: 2.000 KHz (exacto)");
}

void loop() {
  // Enviar datos en formato compatible con Serial Plotter
  // El Serial Plotter grafica valores separados por espacios/tabs
  if (newSampleReady) {
    newSampleReady = false;

    // Reducir frecuencia de impresión para no saturar el serial
    // A 2 KHz sería 2000 muestras/seg, muy rápido para el plotter
    static unsigned long lastPrint = 0;
    static uint16_t sampleCounter = 0;

    sampleCounter++;

    // Enviar cada 10 muestras (200 Hz) o cada 50ms
    if (sampleCounter >= 10 || (millis() - lastPrint >= 50)) {
      // Formato para Serial Plotter: etiqueta:valor
      // Todas las señales escaladas al mismo rango para comparación visual

      // Calcular voltaje de entrada (0-5V)
      float inputVoltage = (analogValue * 5.0) / 1023.0;

      // Calcular voltaje de salida PWM (0-5V equivalente)
      uint8_t pwmValue = analogValue >> 2;  // 0-255
      float outputVoltage = (pwmValue * 5.0) / 255.0;

      // Enviar ambas señales en voltios para comparación directa
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