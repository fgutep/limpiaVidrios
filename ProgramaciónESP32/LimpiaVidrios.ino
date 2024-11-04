#include <Bluepad32.h>
#include "Motores.h"

// Definición de pines de los motores
#define MOTOR_A_IN1 5
#define MOTOR_A_IN2 18
#define MOTOR_B_IN1 17
#define MOTOR_B_IN2 6
#define EEP 12

// Instancias globales
Motores motor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2, EEP);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d\n",  // Agregamos una coma aquí
        ctl->index(),         // Controller Index
        ctl->dpad(),          // D-pad
        ctl->buttons(),       // bitmask of pressed buttons
        ctl->axisX(),         // (-511 - 512) left X Axis
        ctl->axisY()          // (-511 - 512) left Y axis
    );
}


void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    //flecha a la derecha
    if (ctl->dpad() == 0x04) {
        motor.turnRight();
        delay(100);
    }
    //flecha a la izquierda
    if (ctl->dpad() == 0x08) {
        motor.turnLeft();
        delay(100);
    }
    //flechaabajo
    if (ctl->dpad() == 0x02) {
        motor.moveBackward();
        delay(100);
    }
    //flechaarriba
    if (ctl->dpad() == 0x01) {
        motor.moveForward();
        delay(100);
    }
     // Verificación de inactividad en dpad, buttons, axisX y axisY
    if (ctl->dpad() == 0x00 && ctl->buttons() == 0x0000) {
        motor.lockMotors();
    }
    //Definicion actividad segun el movimiento del joystick
    int x = ctl->axisX();
    int y = ctl->axisY();

    //if (x >= -8 && x <= 8 && y >= -8 && y <= 8) {
      // Se discrimina el rango de -8 a 8 de x,y dado que el control cuenta con un pequeño error de fabricacion
      // donde a pesar no estar siendo manipulado muestra un movimiento leve en estos ejes
    //    motor.lockMotors();
    //} else {
        // Movimiento según el valor de `x` y `y`
        //if (y > 8) {
            //motor.moveForward();
        //} else if (y < -8) {
            //motor.moveBackward();
        //}
        // (x > 8) {
            //motor.turnRight();
        //} else if (x < -8) {
            //motor.turnLeft();
        //}
    //}


    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } 
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated){
    processControllers();
    }
  // Procesa comandos de teclado
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'w':
        motor.moveForward();
        delay(100);
        break;
      case 's':
        motor.moveBackward();
        delay(100);
        break;
      case 'a':
        motor.turnLeft();
        delay(100);
        break;
      case 'd':
        motor.turnRight();
        delay(100);
        break;
      default:
        motor.lockMotors();
        Serial.println("Comando inválido. Usa W (adelante), S (atrás), A (izquierda), D (derecha)");
        break;
    }
  }

  delay(150);  // Pequeño retardo para evitar sobrecarga
}

