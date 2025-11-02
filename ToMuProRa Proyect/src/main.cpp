#include <Arduino.h>

// Configuración de pines
const int ANALOG_INPUT_PIN = A0;  // Pin de entrada analógica
const int PWM_OUTPUT_PIN = 9;     // Pin de salida PWM (Timer1)

// Variables para el muestreo
volatile uint16_t analogValue = 0;
volatile bool newSampleReady = false;

// Configuración del Timer1 para 2 KHz (500 microsegundos)
void setupTimer1() {
  // Deshabilitar interrupciones
  noInterrupts();

  // Configurar Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Configurar para modo CTC (Clear Timer on Compare Match)
  // Frecuencia = 16MHz / (prescaler * (OCR1A + 1))
  // Para 2 KHz: OCR1A = 16000000 / (8 * 2000) - 1 = 999
  OCR1A = 999;  // Valor de comparación para 2 KHz

  TCCR1B |= (1 << WGM12);   // Modo CTC
  TCCR1B |= (1 << CS11);    // Prescaler = 8
  TIMSK1 |= (1 << OCIE1A);  // Habilitar interrupción por comparación

  // Habilitar interrupciones
  interrupts();
}

// Configuración del ADC para lectura rápida
void setupADC() {
  // Configurar el ADC para lectura más rápida
  // Prescaler de 16 (en lugar de 128 por defecto)
  // Esto da una frecuencia de ADC de 1 MHz (16MHz/16)
  ADCSRA &= ~0x07;  // Limpiar bits de prescaler
  ADCSRA |= 0x04;   // Establecer prescaler a 16
}

// Configuración del PWM en Timer1 (Pin 9)
void setupPWM() {
  // Timer1 ya está configurado, solo ajustamos para PWM en pin 9
  // Modo Fast PWM de 8 bits
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  TCCR1A |= (1 << COM1A1);  // Non-inverting mode en OC1A (pin 9)
  TCCR1A |= (1 << WGM10);   // Fast PWM 8-bit
}

// Interrupción del Timer1 - Se ejecuta cada 500 µs (2 KHz)
ISR(TIMER1_COMPA_vect) {
  // Leer valor analógico (0-1023)
  analogValue = analogRead(ANALOG_INPUT_PIN);
  newSampleReady = true;

  // Convertir de 10 bits (0-1023) a 8 bits (0-255) para PWM
  uint8_t pwmValue = analogValue >> 2;

  // Escribir valor a la salida PWM
  analogWrite(PWM_OUTPUT_PIN, pwmValue);
}

void setup() {
  // Inicializar comunicación serial para debug
  Serial.begin(115200);
  Serial.println("Sistema de adquisición a 2 KHz iniciado");

  // Configurar pines
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  // Configurar ADC para lectura rápida
  setupADC();

  // Configurar Timer1 para interrupciones a 2 KHz
  setupTimer1();

  Serial.println("Configuración completa");
  Serial.println("Pin entrada: A0");
  Serial.println("Pin salida PWM: D9");
  Serial.println("Frecuencia de muestreo: 2 KHz");
}

void loop() {
  // Opcional: Enviar datos por serial para monitoreo
  // NOTA: Esto puede afectar el timing si se envía demasiada información
  if (newSampleReady) {
    newSampleReady = false;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {  // Imprimir cada 1 segundo
      Serial.print("Valor analógico: ");
      Serial.print(analogValue);
      Serial.print(" (");
      Serial.print((analogValue * 5.0) / 1023.0);
      Serial.println(" V)");
      lastPrint = millis();
    }
  }
}