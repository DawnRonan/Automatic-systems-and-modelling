#include <Keypad.h>

// 定义行和列的引脚
const byte ROW_NUM    = 4; // 行数
const byte COLUMN_NUM = 4; // 列数

// 定义按键矩阵
char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// 定义每行和每列的引脚
byte pin_rows[ROW_NUM]    = {9, 8, 7, 6};  // 行引脚连接到 D6, D7, D8, D9
byte pin_column[COLUMN_NUM] = {13, 12, 11, 10}; // 列引脚连接到 D2, D3, D4, D5

// 创建 Keypad 对象
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

// Define encoder pins
const int Channel_A = 2;  // Channel A connected to D2
const int Channel_B = 3;  // Channel B connected to D3

// Motor control pins
const int Speed = 5;       // Motor speed control pin (PWM)
const int Moin1 = A4;       // Motor direction pin 1
const int Moin2 = A1;       // Motor direction pin 2

// Encoder counter
volatile long encoderCount = 0;
const int ppr = 2048;  // 每圈脉冲数，根据编码器型号设定
int lastEncoded = 0;          // Previous encoder signal

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
  analogWrite(Speed, 12);
}

void loop() {
  static long lastCount = 0;
  noInterrupts();
  long count = encoderCount;
  interrupts();

  // 计算角度（取模限制在 0~360°）
  float angle = (float)(count % ppr) * 360.0 / ppr;
  if (angle < 0) angle += 360.0;
  float setpoint = 0;
  // 打印角度
  if (count != lastCount) {
    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" -> Angle: ");
    Serial.println(angle);
    lastCount = count;
  }

  if (key) {  // 如果按键被按下
    Serial.print("按下的键是: ");
    Serial.println(key);  // 打印按下的键
    //Input a number, rotote the moter a specific angle
    //key = 1; //red ball, 0degree
    //key = 2; //green ball, 90degree
    //key = 3; //blue ball, 180degree
    //key = 4; //yellow ball, 270degree
    if(key == '1') {  // 如果按下的键是 '1'
      setpoint = 0;
      if(angle<=180 && angle>0) {  // 如果当前角度在 0° 到 180° 之间
        rotateDirtecion(-1)
      }else if(angle>180) {  // 如果当前角度在 180° 到 360° 之间
        rotateDirtecion(1)
      }else if(angle==0) {  // 如果当前角度为 0°
        rotateDirtecion(0)
      }
    } else if(key == '2') {  // 如果按下的键是 '2'
      setpoint = 90;
      if(angle<=270 && angle>90) {  // 如果当前角度在 0° 到 180° 之间
        rotateDirtecion(-1)
      }else if(angle>270 || angle<90) {  // 如果当前角度在 180° 到 360° 之间
        rotateDirtecion(1)
      }else if(angle==90) {  // 如果当前角度为 0°
        rotateDirtecion(0)
      }

    } else if (key == '3') {  // 如果按下的键是 '3'
      setpoint = 180;
      if(angle<360 && angle>180) {  // 如果当前角度在 0° 到 180° 之间
        rotateDirtecion(-1)
      }else if(angle<180) {  // 如果当前角度在 180° 到 360° 之间
        rotateDirtecion(1)
      }else if(angle==180) {  // 如果当前角度为 0°
        rotateDirtecion(0)
      }

    } else if (key == '4') {  // 如果按下的键是 '4'
      setpoint = 270;
      if(angle<=90 || angle>270) {  // 如果当前角度在 0° 到 180° 之间
        rotateDirtecion(-1)
      }else if(angle<270 && angle>90) {  // 如果当前角度在 180° 到 360° 之间
        rotateDirtecion(1)
      }else if(angle==270) {  // 如果当前角度为 0°
        rotateDirtecion(0)
      }

    } else {
      Serial.println("Invalid input!");  // 无效输入
      rotateDirtecion(0)
    }
    delay(1000);  // 等待 1 秒
}

void rotateDirtecion(int dir) {
  if (dir == -1) {
    // Clockwise rotation
    digitalWrite(Moin1, HIGH);
    digitalWrite(Moin2, LOW);
  } else if (dir == 1) {
    // Counterclockwise rotation
    digitalWrite(Moin1, LOW);
    digitalWrite(Moin2, HIGH);
  } else {
    // Stop motor
    digitalWrite(Moin1, LOW);
    digitalWrite(Moin2, LOW);
  }
}
// --- PID control ---
void pidController(float setpoint, float angle){
  // === PID Parameters ===
  float kp = 2.0;
  float ki = 0.0;
  float kd = 0.5;
  float error = 0.0, lastError = 0.0, integral = 0.0, derivative = 0.0;
  int outputPWM = 0;
  error = setpoint - angle;
  integral += error;
  derivative = error - lastError;
  float controlSignal = kp * error + ki * integral + kd * derivative;
  lastError = error;
  // --- Determine direction ---
  if (controlSignal > 0) {
    digitalWrite(Moin1, HIGH);
    digitalWrite(Moin2, LOW);
  } else {
    digitalWrite(Moin1, LOW);
    digitalWrite(Moin2, HIGH);
  }
  // --- Apply PWM ---
  outputPWM = constrain(abs(controlSignal), 0, 255);
  analogWrite(D5, outputPWM);
  delay(20);  // PID sampling interval
}

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

