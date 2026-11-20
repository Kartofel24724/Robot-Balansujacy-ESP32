#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <BLEGamepadClient.h>
#include "FastAccelStepper.h"

#define ENA 12

#define STEP_R 25
#define DIR_R 26

#define STEP_L 27
#define DIR_L 14

MPU6050 mpu(Wire);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motorL = NULL;
FastAccelStepper *motorR = NULL;

XboxController Xcontroller;

portMUX_TYPE inputMux = portMUX_INITIALIZER_UNLOCKED;

float targetSpeed;
float steering;
float trim = -7.5;

float Kp = 5700;
float Kd = 600;
float Kv = 0.0012;
float Kiv = 0.00006f;

float speed;
float speedFilter = 0;
float cmd = 0;  
float targetAngle;       
float trimAdj = 0; 

const float SPD_COEF = 7000;
const float STR_COEF = 1500;

const float RAMP = 15000;
const float MAX_ANGLE = 50;
const float MAX_SPEED = 20000;

unsigned long prevTime;

struct ControllerInput{
    float x = 0;
    float y = 0;
    bool lb = 0;
    bool rb = 0;
    bool a = 0;
    bool b = 0;
    bool connected = false;
    uint32_t lastUpdate = 0;
};

ControllerInput controller;

void initSteppers(int accel) {

  engine.init();

  motorL = engine.stepperConnectToPin(STEP_L);
  motorR = engine.stepperConnectToPin(STEP_R);

  if (motorR && motorL) {
    motorL->setDirectionPin(DIR_L);
    motorR->setDirectionPin(DIR_R);

    motorL->setAcceleration(accel);
    motorR->setAcceleration(accel);
  }
}

const uint32_t MIN_HZ = 50;  

void setMotor(FastAccelStepper* m, float v, bool invert) {
  bool fwd = (v >= 0) != invert;
  uint32_t hz = (uint32_t)fabsf(v);
  if (hz < MIN_HZ) hz = MIN_HZ;
  m->setSpeedInHz(hz);
  if (fwd) m->runForward(); else m->runBackward();
}

void driveMotors(float sL, float sR) {
  setMotor(motorL, sL, true);   
  setMotor(motorR, sR, false);
}

float deadzone(float x, float deadzone){
    if (abs(x) < deadzone) return 0.0f;

    return copysignf((abs(x) - deadzone) / (1.0f - deadzone), x);
}

void bluetoothTask(void *parameter){

    Xcontroller.begin();

    float X, Y;
    bool LB, RB, A, B;

    for (;;){
        XboxControlsState s = {};

        if (Xcontroller.isConnected()){

            Xcontroller.read(&s);

            X = deadzone(s.leftStickX, 0.08f);
            Y = deadzone(s.leftStickY, 0.08f);
            LB = s.leftBumper;
            RB = s.rightBumper;
            A = s.buttonA;
            B = s.buttonB;

        }
        else{
            X = 0;
            Y = 0;
            LB = 0;
            RB = 0;
            A = 0;
            B = 0;
        }

        portENTER_CRITICAL(&inputMux);

        controller.x = X;
        controller.y = Y;
        controller.lb = LB;
        controller.rb = RB;
        controller.a = A;
        controller.b = B;
        controller.connected = Xcontroller.isConnected();
        controller.lastUpdate = millis();

        portEXIT_CRITICAL(&inputMux);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void balanceTask(void *parameter){

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t loopT = pdMS_TO_TICKS(4);
  int64_t prevTime = esp_timer_get_time();
  static bool fallen = true;
  static bool armed = false;

  ControllerInput input;

  for(;;){
    int64_t now = esp_timer_get_time();

    float dt = (now - prevTime) / 1000000.0f;
    dt = fminf(dt, 0.02f);
    prevTime = now;

    portENTER_CRITICAL(&inputMux);

    input = controller;

    portEXIT_CRITICAL(&inputMux);

    mpu.update();

    float angle = mpu.getAngleX();

    float err = targetAngle - angle;

    if (input.connected && millis() - input.lastUpdate < 500){
        targetSpeed = SPD_COEF * input.y;
        steering = STR_COEF * input.x;
        if (input.lb) trim += 0.01;
        if (input.rb) trim -= 0.01;
        if (input.a) armed = true;
        if (input.b) armed = false;
    }
    else{
        targetSpeed = 0;
        steering = 0;
    }
  
    if (fabsf(angle) > MAX_ANGLE) fallen = true;
    if (!armed) fallen = true;
    if (fallen) {
        digitalWrite(ENA, HIGH);
        speed = 0; 
        speedFilter = 0; 
        cmd = 0; 
        trimAdj = 0;
        if (fabsf(angle - trim) < 3 && armed) { fallen = false; digitalWrite(ENA, LOW); }
        vTaskDelayUntil(&lastWake, loopT);
        continue;
    }

    float accel = Kp * err - Kd * mpu.getGyroX();
    speed += accel * dt;

    float vR = motorR->getCurrentSpeedInMilliHz() / 1000.0f;
    float vL = -motorL->getCurrentSpeedInMilliHz() / 1000.0f;
    float vAct = 0.5f * (vL + vR);

    speedFilter += 0.025f * (vAct - speedFilter);
    float vErr = speedFilter - cmd;
    cmd += constrain(targetSpeed - cmd, -RAMP * dt, RAMP * dt);
    trimAdj += Kiv * vErr * dt;
    targetAngle = trim + trimAdj + constrain(Kv * vErr, -6.0f, 6.0f);

    int16_t speedL = constrain(speed + steering, -MAX_SPEED, MAX_SPEED);
    int16_t speedR = constrain(speed - steering, -MAX_SPEED, MAX_SPEED);
    driveMotors(speedL, speedR);

    vTaskDelayUntil(&lastWake, loopT);
  }
  
}

void setup() {
    Serial.begin(115200);

    Wire.begin(21,22);

    pinMode(ENA, OUTPUT);
    digitalWrite(ENA, HIGH);

    byte status = mpu.begin();
    if (status != 0) {while (true) delay(100);}

    mpu.setFilterGyroCoef(0.99);

    delay(3000);
    mpu.calcOffsets(true,false);
    for (int i = 0; i < 400; i++) { mpu.update(); delay(4); } 

    initSteppers(80000);

    xTaskCreatePinnedToCore(
        balanceTask,
        "balance",
        8192,
        NULL,
        3,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        bluetoothTask,
        "bluetooth",
        4096,
        NULL,
        1,
        NULL,
        0
    );

}

void loop() {

}