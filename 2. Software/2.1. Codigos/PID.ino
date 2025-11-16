#include <Arduino.h>
#include <ESP32Encoder.h>

/* ===== Motor 1: TB6612 Canal A ===== */
#define AIN1 18
#define AIN2 19
#define PWMA 5

/* ===== Motor 2: TB6612 Canal B ===== */
#define BIN1 16         // IN1 motor 2
#define BIN2 17         // IN2 motor 2
#define PWMB 4          // PWM motor 2

/* ===== STANDBY (común a los dos motores) ===== */
#define STBY 21

/* ===== Encoders ===== */
// Motor 1
#define ENC1_A 34
#define ENC1_B 35
// Motor 2
#define ENC2_A 32
#define ENC2_B 33

/* ===== Parámetros generales ===== */
const float CPR = 275.0;          // cuentas por vuelta
const unsigned long Ts = 100;     // periodo de muestreo en ms (100 ms = 10 Hz)

/* ===== Setpoints (mismo para los 2 motores) ===== */
const float setpoint1 = 5000.0;   // counts/s motor 1
const float setpoint2 = 5000.0;   // counts/s motor 2

/* ===== PID Motor 1 ===== */
float Kp1 = 0.075;
float Ki1 = 0.0;
float Kd1 = 0.0;

float error1 = 0.0;
float prevError1 = 0.0;
float integral1 = 0.0;

/* ===== PID Motor 2 ===== */
float Kp2 = 0.075;
float Ki2 = 0.030;
float Kd2 = 0.0;

float error2 = 0.0;
float prevError2 = 0.0;
float integral2 = 0.0;

/* ===== Encoders ===== */
ESP32Encoder encoder1;
ESP32Encoder encoder2;

long lastCount1 = 0;
long lastCount2 = 0;
unsigned long lastTime = 0;

/* ===== Funciones de motor (solo hacia adelante) ===== */
void setMotor1(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  digitalWrite(AIN1, HIGH);  // adelante
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}

void setMotor2(int pwm) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  digitalWrite(BIN1, HIGH);  // adelante
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}

void setup() {
  Serial.begin(115200);

  // Pines TB6612
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // habilita driver

  // Encoders
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  encoder2.attachFullQuad(ENC2_A, ENC2_B);
  encoder1.clearCount();
  encoder2.clearCount();

  lastCount1 = encoder1.getCount();
  lastCount2 = encoder2.getCount();
  lastTime   = millis();

  Serial.println("SP1 vel1 SP2 vel2");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= Ts) {
    float dt = (float)Ts / 1000.0;

    /* ===== 1. Medición de velocidades ===== */
    long countNow1 = encoder1.getCount();
    long delta1    = countNow1 - lastCount1;
    lastCount1     = countNow1;
    float vel1     = (float)delta1 * (1000.0 / Ts);  // counts/s

    long countNow2 = encoder2.getCount();
    long delta2    = countNow2 - lastCount2;
    lastCount2     = countNow2;

    // Si todavía tu segundo encoder mide al revés, aquí podrías poner:
    // float vel2 = -(float)delta2 * (1000.0 / Ts);
    float vel2     = (float)delta2 * (1000.0 / Ts);  // counts/s

    /* ===== 2. Error de cada motor ===== */
    error1 = setpoint1 - vel1;
    error2 = setpoint2 - vel2;

    /* ===== 3. PID Motor 1 ===== */
    integral1 += error1 * dt;
    // anti-windup simple
    if (integral1 > 5000.0)  integral1 = 5000.0;
    if (integral1 < -5000.0) integral1 = -5000.0;

    float derivative1 = (error1 - prevError1) / dt;
    prevError1 = error1;

    float u1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;
    int pwm1 = (int)u1;

    // zona muerta mínima
    if (pwm1 > 0 && pwm1 < 15) pwm1 = 15;
    pwm1 = constrain(pwm1, 0, 255);

    /* ===== 4. PID Motor 2 ===== */
    integral2 += error2 * dt;
    if (integral2 > 5000.0)  integral2 = 5000.0;
    if (integral2 < -5000.0) integral2 = -5000.0;

    float derivative2 = (error2 - prevError2) / dt;
    prevError2 = error2;

    float u2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * derivative2;
    int pwm2 = (int)u2;

    if (pwm2 > 0 && pwm2 < 15) pwm2 = 15;
    pwm2 = constrain(pwm2, 0, 255);

    /* ===== 5. Aplicar PWM a los motores ===== */
    setMotor1(pwm1);

    // Si quieres probar solo el motor 1, puedes descomentar esta línea:
    pwm2 = 0;
    setMotor2(pwm2);

    /* ===== 6. Imprimir para plotear SOLO SP y velocidad ===== */
    Serial.print(setpoint1);   // SP1
    Serial.print(" ");
    Serial.print(vel1);        // vel1
    Serial.print(" ");
    Serial.print(setpoint2);   // SP2
    Serial.print(" ");
    Serial.println(vel2);      // vel2

    lastTime = now;
  }
}
