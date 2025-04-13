// Define encoder pins
const int Channel_A = 2;  // Channel A connected to D2
const int Channel_B = 4;  // Channel B connected to D3

// Motor control pins
const int Speed = 5;       // Motor speed control pin (PWM)
const int Moin1 = A4;       // Motor direction pin 1
const int Moin2 = A1;       // Motor direction pin 2

// Encoder counter
volatile long encoderCount = 0;
// Encoder pulse per revolution (ppr)
// This value should be set according to the encoder model used
//!!!!!!!!!!!!
const int ppr = 102400;  // 每圈脉冲数，根据编码器型号设定
int lastEncoded = 0;          // Previous encoder signal

static float setpoint = 0;

unsigned long startTime = 0;
unsigned long endTime = 0;
bool motorActive = false;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  pinMode(Channel_A, INPUT_PULLUP);
  pinMode(Channel_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Channel_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Channel_B), handleEncoderB, CHANGE);

  pinMode(Moin1, OUTPUT);
  pinMode(Moin2, OUTPUT);
  pinMode(Speed, OUTPUT);

  // Set initial motor speed to 128 (50% PWM duty cycle)
  analogWrite(Speed, 128);
}

void loop() {
  static long lastCount = 0;
  noInterrupts();
  long count = encoderCount;
  interrupts();

  // Calculate the angle based on the encoder count
  float angle = (float)(count % ppr) * 360.0 / ppr;
  if (angle < 0) angle += 360.0;

  // printing the angle and count

  Serial.print("Count:%");
  Serial.print(count);
  Serial.print(" -> Angle: ");
  Serial.println(angle);
  lastCount = count;

  //expected angle
  setpoint = 180;

  //run PID controller
  pidController(setpoint, angle);
  Serial.print("setpoint: ");
  Serial.println(setpoint);
  endTime = millis();
  unsigned long duration = endTime - startTime;
  Serial.print("time running: (ms): ");
  Serial.println(duration);
  delay(10);
}

// --- PID control ---
void pidController(float setpoint, float angle){
  // === PID Parameters ===
  float kp = 21;
  float ki = 0.7;
  float kd = 6;

  // PID variables
  // These variables are static to retain their values between function calls
  static float error = 0.0;
  static float lastError = 0.0;
  static float integral = 0.0;
  static float derivative = 0.0;
  static int outputPWM = 0;
  error = setpoint - angle;

  Serial.print("error: ");
  Serial.println(error);
  integral += error;
  derivative = error - lastError;
  float controlSignal = kp * error + ki * integral + kd * derivative;
  controlSignal = constrain(controlSignal, -255, 255); // 限制控制信号范围
  lastError = error;

  //Serial.print("Controlling Signal: ");
  //Serial.println(controlSignal);

  // --- Determine direction ---
  if (controlSignal > 0) {
    digitalWrite(Moin1, HIGH);
    digitalWrite(Moin2, LOW);
  } else if(controlSignal < 0){
    digitalWrite(Moin1, LOW);
    digitalWrite(Moin2, HIGH);
  }
  // --- Apply PWM ---
  outputPWM = constrain(abs(controlSignal), 0, 255);
  Serial.print("outputPWM: ");
  Serial.println(outputPWM);
  analogWrite(Speed, outputPWM);

  // --- Stop the motor if error is small ---
  if (abs(error) < 18){
    analogWrite(Speed, 100);
    if (abs(error) < 8){
      analogWrite(Speed, 50);
      if(abs(error) < 0.3){
        analogWrite(Speed, 0);
        digitalWrite(Moin1, LOW);
        digitalWrite(Moin2, LOW);
      }
    }
  }
  return;
}
// --- Interrupt Service Routines (ISR) for Encoder ---
// These functions are called when the encoder signals change
void handleEncoderA() {
  bool A = digitalRead(Channel_A);
  bool B = digitalRead(Channel_B);
  if (A == B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void handleEncoderB() {
  bool A = digitalRead(Channel_A);
  bool B = digitalRead(Channel_B);
  if (A != B) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}