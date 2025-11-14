#include <Arduino.h>
#include <ESP32Encoder.h>

// ----------- Pines TB6612 (Motor A) -----------
#define AIN1 18
#define AIN2 19
#define PWMA 5
#define STBY 21

// ----------- Pines Encoder -----------
#define ENC_A 34
#define ENC_B 35

// ----------- Parámetros del encoder -----------
const float CPR = 275.0;          // cuentas por vuelta (tu valor medido)
const unsigned long Ts = 100;     // periodo de muestreo en ms (100 ms = 10 Hz)

// ----------- Setpoint de velocidad (counts/s) -----------
const float setpoint = 5000.0;    // en counts/s

// ----------- Ganancias PID (ajústalas si quieres) -----------
float Kp = 0.072;
float Ki = 0.030;
float Kd =  0.0015;

// ----------- Variables PID -----------
float error = 0.0;
float prevError = 0.0;
float integral = 0.0;

// ----------- Variables globales -----------
ESP32Encoder encoder;
long lastCount = 0;
unsigned long lastTime = 0;

// ---------- Función para controlar el motor (solo hacia adelante) ----------
void setMotor(int pwm) {
  // Solo controlamos velocidad adelante
  if (pwm < 0) pwm = 0;       // nada de negativo
  if (pwm > 255) pwm = 255;   // saturación máxima

  digitalWrite(AIN1, HIGH);   // Dirección fija: adelante
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}

void setup() {
  Serial.begin(115200);

  // Pines TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);      // activa el driver

  // Encoder
  encoder.attachFullQuad(ENC_A, ENC_B);
  encoder.clearCount();

  lastCount = encoder.getCount();
  lastTime  = millis();

  Serial.println("SP vel error (todos en counts/s)");
}

void loop() {

  unsigned long now = millis();
  if (now - lastTime >= Ts) {
    // -------- 1. Medición de velocidad --------
    long countNow = encoder.getCount();
    long delta = countNow - lastCount;
    lastCount = countNow;
    lastTime = now;

    // counts/s
    float countsPerSec = (float)delta * (1000.0 / Ts);

    // -------- 2. Error --------
    error = setpoint - countsPerSec;

    float dt = (float)Ts / 1000.0;   // en segundos

    // -------- 3. PID clásico --------
    integral += error * dt;

    // Anti-windup simple (limita el integral)
    if (integral > 5000.0)  integral = 5000.0;
    if (integral < -5000.0) integral = -5000.0;

    float derivative = (error - prevError) / dt;
    prevError = error;

    float u = Kp * error + Ki * integral + Kd * derivative;

    // -------- 4. Convertir a PWM y aplicar límites --------
    int pwmOut = (int)u;

    // Zona muerta mínima para que el motor se mueva
    if (pwmOut > 0 && pwmOut < 15) pwmOut = 15;

    // Solo 0–255 (sin reversa en este PID)
    pwmOut = constrain(pwmOut, 0, 255);

    // -------- 5. Aplicar al motor --------
    setMotor(pwmOut);

    // -------- 6. Salida para plotear: SP vel error (en counts/s) --------
    Serial.print(setpoint);       // SP
    Serial.print(" ");
    Serial.print(countsPerSec);   // velocidad medida
    Serial.print(" ");
    Serial.println(error);        // error
  }
}
