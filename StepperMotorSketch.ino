#include <AccelStepper.h>
#include <HCSR04.h>

#define STEP_PIN 9
#define DIR_PIN 10
#define ECHO_PIN 5
#define TRIG_PIN 6

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
HCSR04 sensor(TRIG_PIN, ECHO_PIN);

const float STEPS_PER_DEGREE = 2.22; // adjust for your microstepping

// --- PID gains (start conservative) ---
float Kp = 7.0;   // proportional
float Ki = 0;   // integral
float Kd = 0;   // derivative

// --- Control state ---
double prevReading = 0;
double integral = 0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  digitalWrite(4, LOW);

  stepper.setMaxSpeed(800);     // steps/s
  stepper.setAcceleration(600); // steps/s^2
  stepper.setCurrentPosition(0); // assume beam level at startup
}

void moveToDegree(float angle) {
  long target = -angle * STEPS_PER_DEGREE;  
  stepper.moveTo(target);
}

void loop() {
  unsigned long now = millis();

  if (now - prevTime >= 30) {
    double dt = (now - prevTime) / 1000.0;
    prevTime = now;

    double reading = sensor.dist();
    double error = 10.0 - reading;

    // SIMPLE P-only control
    double control = Kp * error;  
    control = constrain(control, -60, 60);

    moveToDegree(control);

    Serial.print("Dist: "); Serial.print(reading);
    Serial.print("  Err: "); Serial.print(error);
    Serial.print("  Angle: "); Serial.println(control);
  }

  stepper.run();
}



// void loop() {

// //   for (int i = 0; i < 200; i++) {
// //     stepper.moveTo(i * 10);
// //     stepper.runToPosition();
// //     Serial.println(i*10);
// // }

// // for (int i = 199; i >= 0; i--) {
// //     stepper.moveTo(i * 10);
// //     stepper.runToPosition();
// //     Serial.println(i*10);
// // }

//   unsigned long now = millis();

//   // Run control loop every 30 ms
//   if (now - prevTime >= 30) {
//     double dt = (now - prevTime) / 1000.0; // convert ms to s
//     prevTime = now;

//     double reading = sensor.dist();
//     double error = 10.0 - reading;               // desired = 10 cm
//     double rate  = (reading - prevReading) / dt; // cm/s

//     // --- Integral term (accumulate error) ---
//     integral += error * dt;
//     // prevent integral windup (limit stored value)
//     integral = constrain(integral, -20, 20);

//     // --- PID control law ---
//     double control = Kp * error + Ki * integral - Kd * rate;
//     control = constrain(control, -60, 60); // degrees limit

//     // --- Output ---
//     moveToDegree(control);

//     // Debug print
//     Serial.print("Dist: "); Serial.print(reading);
//     Serial.print("  Err: "); Serial.print(error);
//     Serial.print("  dErr: "); Serial.print(rate);
//     Serial.print("  Int: "); Serial.print(integral);
//     Serial.print("  Angle: "); Serial.println(control);

//     prevReading = reading;
//   }

//   stepper.run();
// }
