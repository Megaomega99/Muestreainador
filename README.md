# Detector de Picos de Señal de 21 Hz con Arduino

## ¿Qué hace este proyecto?

Este sistema detecta los picos (puntos más altos) de una señal de 21 Hz y genera pulsos digitales perfectamente sincronizados con esos picos. Es como tener un "detector automático" que identifica exactamente cuándo la señal alcanza su máximo valor.

## ¿Cómo funciona? (Explicación simple)

Imagina que tienes una ola en el mar que sube y baja 21 veces por segundo. Este sistema:
1. **Mide la ola** constantemente (2000 veces por segundo)
2. **Filtra el ruido** para quedarse solo con la ola de 21 Hz
3. **Detecta cuándo la ola está en su punto más alto**
4. **Genera un pulso eléctrico** justo en ese momento

## Hardware necesario

- **Arduino Uno** (o compatible con ATmega328P)
- **Pin A0**: Entrada de la señal analógica (0-5V)
- **Pin 9**: Salida PWM con la señal filtrada (para visualización)
- **Pin 10**: Salida digital con los pulsos sincronizados

## Conceptos clave para entender el código

### 1. ¿Qué es el formato Q15?

Los números decimales normales (como 3.14159) ocupan mucha memoria. El formato Q15 es una forma eficiente de representar números decimales usando números enteros:
- Se multiplica el número por 32768 (2^15)
- Ejemplo: 0.5 en Q15 = 0.5 × 32768 = 16384
- Para recuperar el valor original, se divide por 32768 (shift derecho de 15 bits)

**¿Por qué se usa?** Arduino es mucho más rápido haciendo operaciones con enteros que con decimales.

### 2. ¿Qué es un filtro IIR?

IIR significa "Infinite Impulse Response" (Respuesta Infinita al Impulso). Es como un colador que deja pasar solo ciertas frecuencias:
- Imagina un colador que solo deja pasar pelotas de un tamaño específico
- En nuestro caso, solo deja pasar señales entre 12 Hz y 30 Hz
- Rechaza todo lo demás (ruido, señales de otras frecuencias)

**Fórmula básica:** La señal filtrada depende de:
- La señal actual y las dos anteriores (memoria del pasado)
- Los dos valores filtrados anteriores (feedback)

### 3. ¿Qué es la transformada de Hilbert?

La transformada de Hilbert es una operación matemática que crea una versión "desplazada 90 grados" de la señal original:
- Si tienes una ola, la transformada de Hilbert es la misma ola pero movida un cuarto de ciclo
- **¿Para qué sirve?** Combinando la señal original y su transformada de Hilbert, podemos calcular:
  - **Magnitud (amplitud)**: Qué tan "grande" es la señal en ese momento
  - **Fase (ángulo)**: En qué punto del ciclo está la señal

Piensa en las manecillas de un reloj:
- La señal original es la manecilla de minutos
- La transformada de Hilbert es la manecilla de horas
- Juntas te dicen la hora exacta (fase) y qué tan lejos están del centro (magnitud)

### 4. ¿Qué es CORDIC?

CORDIC es un algoritmo inteligente para calcular magnitud y ángulo (fase) sin usar operaciones costosas como raíz cuadrada o arcotangente:
- En lugar de calcular √(x² + y²), hace rotaciones sucesivas
- Es como encontrar un punto en el mapa rotando y acercándose poco a poco
- **Ventaja:** Solo usa sumas, restas y desplazamientos de bits (muy rápido en Arduino)

### 5. ¿Qué es un periodo refractario?

Imagina que tienes un botón que, después de presionarlo, se bloquea por 1 segundo:
- El periodo refractario hace lo mismo con la detección de picos
- Después de detectar un pico, el sistema se "duerme" por 10ms (20 muestras)
- **¿Por qué?** Para evitar detectar el mismo pico varias veces

### 6. ¿Qué es la cola de pulsos (Queue)?

Una cola es como una fila en el supermercado:
- El primero que entra es el primero que sale (FIFO: First In, First Out)
- Cada pico detectado se pone en la fila con su "tiempo de espera"
- Cuando el tiempo llega a cero, se genera el pulso
- **¿Por qué una cola?** Permite tener varios pulsos "en espera" simultáneamente

## Explicación línea por línea del código

### Configuración de pines (líneas 3-6)
```cpp
const int ANALOG_INPUT_PIN = A0;    // Pin A0 lee la señal analógica
const int PWM_OUTPUT_PIN = 9;       // Pin 9 saca la señal filtrada (PWM)
const int PULSE_OUTPUT_PIN = 10;    // Pin 10 genera los pulsos
```

### Variables del sistema (líneas 8-12)
```cpp
volatile uint16_t rawSample = 0;        // Último valor leído del ADC (0-1023)
volatile bool newSampleReady = false;   // Bandera: "hay nueva muestra lista"
bool inPulse = false;                   // Estado: "estamos generando un pulso"
#define Q15_SHIFT 15                    // Cantidad de bits para formato Q15
```
- `volatile`: Le dice al compilador "esta variable puede cambiar en cualquier momento" (interrupciones)
- `uint16_t`: Número entero sin signo de 16 bits (0 a 65535)

### Filtro IIR (líneas 14-24)
```cpp
const int32_t b0_q15 = 456;      // Coeficiente b0 del filtro (numerador)
const int32_t b1_q15 = 0;        // Coeficiente b1 del filtro (numerador)
const int32_t b2_q15 = -456;     // Coeficiente b2 del filtro (numerador)
const int32_t a1_q15 = -64482;   // Coeficiente a1 del filtro (denominador)
const int32_t a2_q15 = 31855;    // Coeficiente a2 del filtro (denominador)
```
**Fórmula del filtro IIR:**
```
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
```
Donde:
- `x[n]` = muestra actual de entrada
- `x[n-1]`, `x[n-2]` = dos muestras anteriores de entrada
- `y[n]` = salida filtrada actual
- `y[n-1]`, `y[n-2]` = dos salidas filtradas anteriores

Los coeficientes están calculados para crear un filtro pasa-banda centrado en 21 Hz.

```cpp
int32_t x1_q15 = 0;    // Memoria: x[n-1] (muestra anterior)
int32_t x2_q15 = 0;    // Memoria: x[n-2] (muestra de hace 2 pasos)
int32_t y1_q15 = 0;    // Memoria: y[n-1] (salida filtrada anterior)
int32_t y2_q15 = 0;    // Memoria: y[n-2] (salida filtrada de hace 2 pasos)
```

### Filtro de Hilbert (líneas 26-33)
```cpp
#define HILBERT_TAPS 15    // El filtro usa 15 coeficientes
const int16_t hilbert_coeffs[HILBERT_TAPS] = {
  -328, 0, -984, 0, -2621, 0, -6554, 0, 6554, 0, 2621, 0, 984, 0, 328
};
```
- **TAPS:** Número de coeficientes del filtro FIR
- Los coeficientes son antisimétricos (el del medio es 0)
- Están en formato Q15

```cpp
int32_t hilbert_buffer[HILBERT_TAPS] = {0};   // Buffer circular con últimas 15 muestras
uint8_t hilbert_index = 0;                     // Índice actual en el buffer circular
```
**Buffer circular:** Como una cinta transportadora infinita que guarda las últimas 15 muestras.

### Parámetros de detección (líneas 35-42)
```cpp
const int32_t MAGNITUDE_THRESHOLD = 50L << Q15_SHIFT;
```
- `50L << Q15_SHIFT` = 50 × 32768 = 1,638,400
- **Umbral de magnitud:** La señal debe ser más fuerte que este valor para considerarla como pico
- Se puede ajustar según la amplitud de tu señal

```cpp
const int16_t PHASE_THRESHOLD = 2730;
```
- En formato Q15, 32768 representa 180°
- 2730 representa aproximadamente 15°
- **Umbral de fase:** Permite una tolerancia de ±15° alrededor de 180°

```cpp
#define PERIOD_21HZ_MS 48              // Periodo de 21 Hz = 1000ms / 21 ≈ 48ms
#define FILTER_DELAY_MS 10             // Retardo combinado de los filtros
#define PHASE_ADJUST_MS -1             // Ajuste fino de sincronización
#define PREDICTION_DELAY_MS (PERIOD_21HZ_MS - FILTER_DELAY_MS)  // = 38ms
#define PREDICTION_DELAY_SAMPLES (PREDICTION_DELAY_MS * 2)       // = 76 muestras
```
**Lógica del delay:**
- Cuando detectamos un pico en la señal filtrada, ese pico está retrasado 10ms respecto al original
- Para generar el pulso en el SIGUIENTE pico original, esperamos: 48ms - 10ms = 38ms
- A 2000 Hz, 38ms = 76 muestras

### Control de pulsos (líneas 44-50)
```cpp
volatile bool triggerPulse = false;     // Bandera: "debe generarse un pulso"
unsigned long pulseStartTime = 0;       // Momento en que empezó el pulso actual
const unsigned long PULSE_DURATION = 2; // Duración del pulso: 2ms
uint16_t pulseCount = 0;                // Contador total de pulsos generados
uint16_t peakDetectionCount = 0;        // Contador total de picos detectados
uint16_t refractoryCountdown = 0;       // Tiempo restante de periodo refractario
```

### Cola de pulsos (líneas 52-57)
```cpp
#define PULSE_QUEUE_SIZE 4              // La cola puede contener hasta 4 pulsos
uint16_t pulseQueue[PULSE_QUEUE_SIZE];  // Array con los countdowns de cada pulso
uint8_t pulseQueueHead = 0;             // Índice donde se escribe el siguiente pulso
uint8_t pulseQueueTail = 0;             // Índice del pulso más antiguo (próximo a salir)
uint8_t pulseQueueCount = 0;            // Cantidad de pulsos actualmente en la cola
```
**Funcionamiento:**
- `Head` avanza cuando añadimos un pulso
- `Tail` avanza cuando generamos un pulso
- Si `Head == Tail`, la cola está vacía

### Configuración del ADC (líneas 59-67)
```cpp
void setupADC() {
  ADCSRA &= ~0x07;              // Limpiar bits de prescaler
  ADCSRA |= 0x04;               // Prescaler = 16 (1 MHz ADC clock)
  ADMUX = 0;                    // Limpiar ADMUX
  ADMUX |= (1 << REFS0);        // Voltaje de referencia = AVCC (5V)
  ADCSRA |= (1 << ADIE);        // Habilitar interrupción del ADC
  ADCSRA |= (1 << ADEN);        // Habilitar el ADC
}
```
**¿Por qué estos valores?**
- Prescaler 16: 16 MHz / 16 = 1 MHz (velocidad óptima del ADC)
- REFS0: Usa 5V como referencia (mide de 0 a 5V)
- ADIE: Llama a una función cuando termina la conversión
- ADEN: Enciende el ADC

### Configuración del Timer2 (líneas 69-79)
```cpp
void setupTimer2() {
  noInterrupts();               // Desactivar interrupciones temporalmente
  TCCR2A = 0;                   // Limpiar registro de control A
  TCCR2B = 0;                   // Limpiar registro de control B
  TCNT2 = 0;                    // Contador en 0
  OCR2A = 124;                  // Valor de comparación
  TCCR2A |= (1 << WGM21);       // Modo CTC (Clear Timer on Compare)
  TCCR2B |= (1 << CS22);        // Prescaler = 64
  TIMSK2 |= (1 << OCIE2A);      // Habilitar interrupción de comparación
  interrupts();                 // Reactivar interrupciones
}
```
**Cálculo de frecuencia:**
```
Frecuencia = 16,000,000 Hz / (Prescaler × (OCR2A + 1))
           = 16,000,000 / (64 × 125)
           = 2000 Hz
```
**Resultado:** El Timer2 genera una interrupción cada 0.5ms (2000 veces por segundo).

### Configuración del PWM (líneas 81-91)
```cpp
void setupPWM() {
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  TCCR1A = 0;                   // Limpiar registro
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1);      // PWM no invertido en pin 9
  TCCR1A |= (1 << WGM10);       // Modo Fast PWM de 8 bits
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);        // Sin prescaler (máxima frecuencia)
  OCR1A = 0;                    // Duty cycle inicial = 0%
}
```
**PWM de 8 bits:**
- `OCR1A` puede ir de 0 a 255
- 0 = siempre apagado (0V)
- 255 = siempre encendido (5V)
- 128 = 50% del tiempo encendido (2.5V promedio)

### Interrupciones (líneas 94-102)
```cpp
ISR(TIMER2_COMPA_vect) {
  ADCSRA |= (1 << ADSC);        // Iniciar conversión del ADC
}
```
**Esta interrupción se ejecuta 2000 veces por segundo** y dispara una nueva lectura del ADC.

```cpp
ISR(ADC_vect) {
  rawSample = ADC;              // Guardar el valor leído (0-1023)
  newSampleReady = true;        // Avisar que hay nueva muestra
}
```
**Esta interrupción se ejecuta cuando el ADC termina de leer**, guardando el resultado.

### Algoritmo CORDIC (líneas 104-161)
```cpp
const int16_t cordic_angles[16] = {
  16384, 9672, 5110, 2594, 1302, 652, 326, 163,
  81, 41, 20, 10, 5, 3, 1, 1
};
```
Ángulos precalculados para rotaciones CORDIC:
- 16384 en Q15 = 45°
- 9672 en Q15 ≈ 26.57°
- Y así sucesivamente, cada uno la mitad del anterior

```cpp
const int32_t CORDIC_GAIN = 19898;
```
- CORDIC introduce un factor de escala K ≈ 1.6468
- Para compensar, multiplicamos por 1/K ≈ 0.6073
- 0.6073 en Q15 = 19898

```cpp
struct CordicResult {
  int32_t magnitude;    // Amplitud de la señal
  int16_t phase;        // Fase en formato Q15 (-32768 a 32767 = -180° a 180°)
};
```

**Función CORDIC simplificada:**
1. **Normalización** (líneas 121-130): Evita overflow dividiendo si el número es muy grande
2. **Rotación al primer cuadrante** (líneas 136-140): Facilita el cálculo
3. **Iteraciones CORDIC** (líneas 142-155): 16 rotaciones sucesivas que "persiguen" el ángulo
4. **Resultado** (líneas 157-160): Magnitud y fase calculados

### Setup (líneas 163-173)
```cpp
void setup() {
  Serial.begin(115200);                      // Comunicación serial a 115200 baudios
  pinMode(ANALOG_INPUT_PIN, INPUT);          // A0 como entrada
  pinMode(PWM_OUTPUT_PIN, OUTPUT);           // Pin 9 como salida
  pinMode(PULSE_OUTPUT_PIN, OUTPUT);         // Pin 10 como salida
  digitalWrite(PULSE_OUTPUT_PIN, LOW);       // Iniciar pulso en bajo (0V)
  setupADC();                                // Configurar ADC
  setupPWM();                                // Configurar PWM
  setupTimer2();                             // Configurar Timer (esto inicia el muestreo)
}
```

### Loop principal (líneas 175-205)

#### Control de pulso (líneas 180-193)
```cpp
if (triggerPulse) {
  digitalWrite(PULSE_OUTPUT_PIN, HIGH);      // Pulso a 5V
  pulseStartTime = millis();                 // Guardar momento de inicio
  triggerPulse = false;                      // Limpiar bandera
  inPulse = true;                            // Marcar que estamos en pulso
}

if (inPulse) {
  if (millis() - pulseStartTime >= PULSE_DURATION) {  // Han pasado 2ms?
    digitalWrite(PULSE_OUTPUT_PIN, LOW);     // Pulso a 0V
    inPulse = false;                         // Terminar pulso
  }
}
```
**Funcionamiento:**
- Cuando `triggerPulse` es `true`, sube el pin a 5V
- Espera 2ms
- Baja el pin a 0V

#### Reporte de estadísticas (líneas 196-205)
```cpp
static uint16_t lastPeakCount = 0;
if (millis() - lastReportMillis >= 1000) {   // Cada segundo
  Serial.print("Pulsos/seg: ");
  Serial.print(pulseCount - lastPulseCount); // Pulsos en este segundo
  Serial.print(" Hz | Picos detectados: ");
  Serial.println(peakDetectionCount - lastPeakCount); // Picos en este segundo
  lastPulseCount = pulseCount;
  lastPeakCount = peakDetectionCount;
  lastReportMillis = millis();
}
```
Imprime estadísticas cada segundo por el puerto serial.

### Procesamiento de señal (líneas 207-291)

#### Lectura de muestra (líneas 208-215)
```cpp
if (newSampleReady) {
  noInterrupts();                    // Pausar interrupciones
  newSampleReady = false;            // Limpiar bandera
  uint16_t adcValue = rawSample;     // Copiar valor del ADC
  interrupts();                      // Reanudar interrupciones

  int32_t x = ((int32_t)adcValue - 512L) << Q15_SHIFT;
}
```
**Línea clave:**
- `adcValue` va de 0 a 1023 (centro en 512)
- Restamos 512 para centrar en 0 (rango: -512 a 511)
- Multiplicamos por 32768 (shift izquierdo 15) para formato Q15

#### Filtro IIR (líneas 217-227)
```cpp
int64_t num = ((int64_t)b0_q15 * x) + ((int64_t)b1_q15 * x1_q15) + ((int64_t)b2_q15 * x2_q15);
num >>= Q15_SHIFT;
int64_t den = ((int64_t)a1_q15 * y1_q15) + ((int64_t)a2_q15 * y2_q15);
den >>= Q15_SHIFT;
int32_t y_filtered = (int32_t)(num - den);
```
**Implementación de:**
```
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
```

```cpp
x2_q15 = x1_q15;    // x[n-2] = x[n-1]
x1_q15 = x;         // x[n-1] = x[n]
y2_q15 = y1_q15;    // y[n-2] = y[n-1]
y1_q15 = y_filtered; // y[n-1] = y[n]
```
Actualizar memoria del filtro para la próxima iteración.

#### Filtro de Hilbert (líneas 229-239)
```cpp
hilbert_buffer[hilbert_index] = y_filtered;             // Guardar muestra
hilbert_index = (hilbert_index + 1) % HILBERT_TAPS;     // Avanzar índice circular
```
**Buffer circular:** Cuando llegamos al final (índice 15), volvemos al principio (índice 0).

```cpp
int64_t hilbert_output = 0;
uint8_t buf_idx = hilbert_index;
for (int i = 0; i < HILBERT_TAPS; i++) {
  hilbert_output += ((int64_t)hilbert_coeffs[i] * hilbert_buffer[buf_idx]);
  buf_idx = (buf_idx + 1) % HILBERT_TAPS;
}
hilbert_output >>= Q15_SHIFT;
```
**Convolución FIR:** Multiplica cada coeficiente por su muestra correspondiente y suma todo.

#### Cálculo de magnitud y fase (líneas 241-247)
```cpp
int32_t real_part = y_filtered;              // Parte real = señal filtrada
int32_t imag_part = (int32_t)hilbert_output; // Parte imaginaria = transformada de Hilbert
CordicResult cordic = fastCORDIC(real_part, imag_part);
int32_t envelope = cordic.magnitude;
if (envelope < 0) envelope = -envelope;      // Asegurar que magnitud sea positiva
int16_t phase = cordic.phase;
```

#### Detección de picos (líneas 249-253)
```cpp
bool isPeak = (envelope > MAGNITUDE_THRESHOLD) &&
              ((phase > (32768 - PHASE_THRESHOLD)) || (phase < (-32768 + PHASE_THRESHOLD)));
```
**Dos condiciones:**
1. Magnitud mayor que umbral (señal suficientemente fuerte)
2. Fase cerca de ±180° (±15°), lo que indica un pico en la señal filtrada

**¿Por qué ±180°?**
El filtro de Hilbert invierte la fase, entonces los picos aparecen en ±180° en lugar de 0°.

#### Gestión de cola de pulsos (líneas 255-266)
```cpp
for (uint8_t i = 0; i < pulseQueueCount; i++) {
  uint8_t idx = (pulseQueueTail + i) % PULSE_QUEUE_SIZE;
  if (pulseQueue[idx] > 0) pulseQueue[idx]--;   // Decrementar countdown
}
```
Cada muestra (cada 0.5ms), decrementamos el countdown de todos los pulsos en la cola.

```cpp
if (pulseQueueCount > 0 && pulseQueue[pulseQueueTail] == 0) {
  triggerPulse = true;                 // Activar generación de pulso
  pulseCount++;                        // Incrementar contador
  pulseQueueTail = (pulseQueueTail + 1) % PULSE_QUEUE_SIZE;  // Avanzar cola
  pulseQueueCount--;                   // Un pulso menos en la cola
}
```
Cuando el countdown del pulso más antiguo llega a 0, se genera el pulso y se remueve de la cola.

#### Periodo refractario (líneas 268-269)
```cpp
if (refractoryCountdown > 0) refractoryCountdown--;
```
Decrementar el periodo refractario cada muestra.

#### Detección de flanco ascendente (líneas 271-280)
```cpp
if (isPeak && !wasAboveThreshold && refractoryCountdown == 0) {
  peakDetectionCount++;
  if (pulseQueueCount < PULSE_QUEUE_SIZE) {
    pulseQueue[pulseQueueHead] = PREDICTION_DELAY_SAMPLES;  // 76 muestras = 38ms
    pulseQueueHead = (pulseQueueHead + 1) % PULSE_QUEUE_SIZE;
    pulseQueueCount++;
  }
  refractoryCountdown = 20;  // Bloquear por 10ms
}

wasAboveThreshold = isPeak;
```
**Lógica:**
1. Si hay pico Y no había pico antes (`!wasAboveThreshold`) Y no estamos en refractario
2. Incrementar contador de picos
3. Añadir pulso a la cola con countdown de 76 muestras (38ms)
4. Activar periodo refractario de 20 muestras (10ms)

#### Salida PWM (líneas 284-289)
```cpp
int32_t y_amplified = (y_filtered * 7) >> 1;     // Amplificar x3.5
int32_t output = (y_amplified >> Q15_SHIFT) + 512L;  // Convertir a 0-1023
if (output < 0) output = 0;
if (output > 1023) output = 1023;
OCR1A = output >> 2;                             // Convertir a 0-255 (8 bits)
```
**Pasos:**
1. Amplificar la señal filtrada por 3.5 (para mejor visualización)
2. Convertir de Q15 a rango 0-1023
3. Limitar a rango válido
4. Convertir a 8 bits (dividir por 4) para el PWM

## Diagrama de flujo del algoritmo

```
┌─────────────────────────┐
│   Timer2 (2000 Hz)      │
│   Dispara ADC           │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   ADC lee pin A0        │
│   (0-1023)              │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   Filtro IIR            │
│   (21 Hz ± 9 Hz)        │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   Filtro Hilbert        │
│   (Desfase 90°)         │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   CORDIC                │
│   Calcula magnitud      │
│   y fase                │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   ¿Es pico?             │
│   (magnitud > umbral    │
│    Y fase ≈ ±180°)      │
└───────────┬─────────────┘
            │
         Sí │
            ▼
┌─────────────────────────┐
│   Añadir a cola         │
│   countdown = 38ms      │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│   Cola decrementa       │
│   countdowns            │
└───────────┬─────────────┘
            │
     countdown=0?
            │
         Sí │
            ▼
┌─────────────────────────┐
│   Generar pulso         │
│   en pin 10             │
│   (duración: 2ms)       │
└─────────────────────────┘
```

## Ajustes y calibración

### Ajustar el umbral de magnitud
Si estás detectando demasiados o muy pocos picos:
```cpp
const int32_t MAGNITUDE_THRESHOLD = 50L << Q15_SHIFT;  // Cambiar el valor 50
```
- **Más alto:** Detecta solo señales muy fuertes (menos picos)
- **Más bajo:** Detecta señales débiles (más picos, posible ruido)

### Ajustar el umbral de fase
Si la detección es inestable:
```cpp
const int16_t PHASE_THRESHOLD = 2730;  // ~15° de tolerancia
```
- **Más alto:** Más tolerante (±20°, ±25°, etc.)
- **Más bajo:** Más estricto (±10°, ±5°, etc.)

### Ajustar la sincronización del pulso
Si los pulsos no coinciden exactamente con los picos:
```cpp
#define PHASE_ADJUST_MS -1  // Ajuste fino en milisegundos
```
- **Negativo:** Pulso más temprano
- **Positivo:** Pulso más tarde

## Troubleshooting

### No se detectan picos
1. Verificar que la señal en A0 esté entre 0-5V
2. Revisar `MAGNITUDE_THRESHOLD` (probablemente muy alto)
3. Verificar conexiones de hardware

### Se detectan demasiados picos
1. Reducir `MAGNITUDE_THRESHOLD`
2. Aumentar `refractoryCountdown` (línea 279)
3. Verificar que la señal no tenga mucho ruido

### Pulsos desincronizados
1. Ajustar `PHASE_ADJUST_MS`
2. Verificar `FILTER_DELAY_MS` (puede variar según implementación)
3. Revisar que la frecuencia de muestreo sea exactamente 2000 Hz

## Recursos adicionales

- **Formato Q15:** [Fixed-Point Arithmetic](https://en.wikipedia.org/wiki/Q_(number_format))
- **CORDIC:** [CORDIC Algorithm](https://en.wikipedia.org/wiki/CORDIC)
- **Transformada de Hilbert:** [Hilbert Transform](https://en.wikipedia.org/wiki/Hilbert_transform)
- **Filtros IIR:** [IIR Filter Design](https://www.dspguide.com/)

## Licencia

Este proyecto es de código abierto. Siéntete libre de usarlo y modificarlo.
