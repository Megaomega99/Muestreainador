#include <Arduino.h>

const int ANALOG_INPUT_PIN = A0;
const int PWM_OUTPUT_PIN = 9;

volatile uint16_t rawSample = 0;
volatile bool newSampleReady = false;

#define Q15_SHIFT 15

const int16_t b0_q15 = 900;
const int16_t b1_q15 = 0;
const int16_t b2_q15 = -900;
const int16_t a1_q15 = -31677;
const int16_t a2_q15 = 15467;

int32_t x1_q15 = 0;
int32_t x2_q15 = 0;
int32_t y1_q15 = 0;
int32_t y2_q15 = 0;

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

void setup() {
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  setupADC();
  setupPWM();
  setupTimer2();
}

void loop() {
  if (newSampleReady) {
    newSampleReady = false;
    uint16_t adcValue = rawSample;

    int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;

    int32_t num = ((int32_t)b0_q15 * (x >> Q15_SHIFT)) +
                  ((int32_t)b1_q15 * (x1_q15 >> Q15_SHIFT)) +
                  ((int32_t)b2_q15 * (x2_q15 >> Q15_SHIFT));

    int32_t den = ((int32_t)a1_q15 * (y1_q15 >> (Q15_SHIFT - 1))) +
                  ((int32_t)a2_q15 * (y2_q15 >> (Q15_SHIFT - 1)));

    int32_t y = num - den;

    x2_q15 = x1_q15;
    x1_q15 = x;
    y2_q15 = y1_q15;
    y1_q15 = y;

    int32_t y_amplified = (y * 7) >> 1;
    int32_t output = (y_amplified >> Q15_SHIFT) + 512L;

    if (output < 0) output = 0;
    if (output > 1023) output = 1023;

    OCR1A = output >> 2;
  }
}