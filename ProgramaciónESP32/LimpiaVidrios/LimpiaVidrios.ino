#include <Bluepad32.h>

#define MOTOR_B_IN2 18  // GPIO for MOTOR_B_IN2
#define MOTOR_B_IN1 17  // GPIO for MOTOR_B_IN1
#define MOTOR_A_IN2 16  // GPIO for MOTOR_A_IN2
#define MOTOR_A_IN1 15  // GPIO for MOTOR_A_IN1

#define FACTORA 1.0  // GPIO for MOTOR_A_IN2
#define FACTORB 0.855  // GPIO for MOTOR_A_IN2



#define PWM_FREQ 20000  // Frecuencia de 20 kHz para eliminar el ruido audible
#define PWM_RESOLUTION 8  // Resolución de 8 bits
#define PWM_CHANNEL_A_IN1 0
#define PWM_CHANNEL_A_IN2 1
#define PWM_CHANNEL_B_IN1 2
#define PWM_CHANNEL_B_IN2 3

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller connected, index=%d\n", i);
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller disconnected, index=%d\n", i);
      break;
    }
  }
}

void processControllerInput(ControllerPtr ctl) {
  uint8_t dpad = ctl->dpad();

  if (dpad & DPAD_UP) {  // Forward
    forward(255);
  } else if (dpad & DPAD_DOWN) {  // Backward
    backward(255);
  } else if (dpad & DPAD_LEFT) {  // Left
    left(255);
  } else if (dpad & DPAD_RIGHT) {  // Right
    right(255);
  } else {
    motorOFF();
  }
}

void setup() {
  // Initialize motor control pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Setup PWM channels
  ledcSetup(PWM_CHANNEL_A_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_A_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B_IN2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_A_IN1, PWM_CHANNEL_A_IN1);
  ledcAttachPin(MOTOR_A_IN2, PWM_CHANNEL_A_IN2);
  ledcAttachPin(MOTOR_B_IN1, PWM_CHANNEL_B_IN1);
  ledcAttachPin(MOTOR_B_IN2, PWM_CHANNEL_B_IN2);

  // Start serial communication
  Serial.begin(115200);

  // Turn off motor initially
  motorOFF();
  
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  // Fetch controller data
  BP32.update();

  // Process controller input
  for (auto myController : myControllers) {
    if (myController && myController->isConnected()) {
      processControllerInput(myController);
    }
  }
}

void motorOFF() {
  ledcWrite(PWM_CHANNEL_A_IN1, 0);
  ledcWrite(PWM_CHANNEL_A_IN2, 0);
  ledcWrite(PWM_CHANNEL_B_IN1, 0);
  ledcWrite(PWM_CHANNEL_B_IN2, 0);
}

void forward(int speed) {
  int correctedSpeedA = speed * FACTORA;  // No corrección para motor A
  int correctedSpeedB = speed * FACTORB;  // Corrección del 90% para motor B
  ledcWrite(PWM_CHANNEL_A_IN1, correctedSpeedA);
  ledcWrite(PWM_CHANNEL_A_IN2, 0);
  ledcWrite(PWM_CHANNEL_B_IN1, correctedSpeedB);
  ledcWrite(PWM_CHANNEL_B_IN2, 0);
  Serial.println("Moving forward");
}

void backward(int speed) {
  int correctedSpeedA = speed * FACTORA;
  int correctedSpeedB = speed * FACTORB;
  ledcWrite(PWM_CHANNEL_A_IN1, 0);
  ledcWrite(PWM_CHANNEL_A_IN2, correctedSpeedA);
  ledcWrite(PWM_CHANNEL_B_IN1, 0);
  ledcWrite(PWM_CHANNEL_B_IN2, correctedSpeedB);
  Serial.println("Moving backward");
}

void left(int speed) {
  int correctedSpeedA = speed * FACTORA;
  int correctedSpeedB = speed * FACTORB;
  ledcWrite(PWM_CHANNEL_A_IN1, correctedSpeedA);
  ledcWrite(PWM_CHANNEL_A_IN2, 0);
  ledcWrite(PWM_CHANNEL_B_IN1, 0);
  ledcWrite(PWM_CHANNEL_B_IN2, correctedSpeedB);
  Serial.println("Turning left");
}

void right(int speed) {
  int correctedSpeedA = speed * FACTORA;
  int correctedSpeedB = speed * FACTORB;
  ledcWrite(PWM_CHANNEL_A_IN1, 0);
  ledcWrite(PWM_CHANNEL_A_IN2, correctedSpeedA);
  ledcWrite(PWM_CHANNEL_B_IN1, correctedSpeedB);
  ledcWrite(PWM_CHANNEL_B_IN2, 0);
  Serial.println("Turning right");
}

