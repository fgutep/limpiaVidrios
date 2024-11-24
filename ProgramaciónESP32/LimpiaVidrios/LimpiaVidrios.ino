#include "Wire.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "Bluepad32.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_Sensor.h>

#define SDA_PIN 14  // SDA pin
#define SCL_PIN 13  // SCL pin

#define MOTOR_B_IN2 18  // GPIO for MOTOR_B_IN2
#define MOTOR_B_IN1 17  // GPIO for MOTOR_B_IN1
#define MOTOR_A_IN2 16  // GPIO for MOTOR_A_IN2
#define MOTOR_A_IN1 15  // GPIO for MOTOR_A_IN1

#define VOLTAGE_SENSOR_PIN 4  // GPIO for Voltage Sensor
#define PWM_FREQ 20000  // Frecuencia de 20 kHz para eliminar el ruido audible
#define PWM_RESOLUTION 8  // Resolución de 8 bits
#define PWM_CHANNEL_A_IN1 0
#define PWM_CHANNEL_A_IN2 1
#define PWM_CHANNEL_B_IN1 2
#define PWM_CHANNEL_B_IN2 3

const float voltage_offset = 3.3;

const char* ap_ssid = "ESP32_Limpiavidrios";  // Nombre de la red creada por el ESP32
const char* ap_password = "12345678";         // Contraseña para acceder a la red

enum State {
    IDLE,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STOP
};

State currentState = IDLE;

AsyncWebServer server(80);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Adafruit_MPU6050 mpu;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float roll = 0.0, pitch = 0.0, yaw = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float voltage = 0.0;
int cleanLevel = 0;  // Valor futuro para el nivel de limpieza (en porcentaje)
bool isControllerConnected = false;
float CORRECTION_FACTOR_B = 0.855; // Factor de corrección inicial para el Motor B
bool mpuInitialized = false;
bool tcsInitialized = false;

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            isControllerConnected = true;
            Serial.printf("Controller connected, index=%d\n", i);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            isControllerConnected = false;
            Serial.printf("Controller disconnected, index=%d\n", i);
            break;
        }
    }
}

void processControllerInput(ControllerPtr ctl) {
    uint8_t dpad = ctl->dpad();
    uint16_t buttons = ctl->buttons();

    // Verificar si se presiona el botón B (valor 0x0002) para incrementar el factor de corrección
    if (buttons & 0x0002) {
        CORRECTION_FACTOR_B += 0.001;  // Incrementar el factor de corrección en 0.001
        if (CORRECTION_FACTOR_B >= 1.0) CORRECTION_FACTOR_B = 1.0;
        Serial.printf("Correction factor increased: %.3f\n", CORRECTION_FACTOR_B);
    }

    // Verificar si se presiona el botón A (valor 0x0001) para disminuir el factor de corrección
    if (buttons & 0x0001) {
        CORRECTION_FACTOR_B -= 0.001;  // Disminuir el factor de corrección en 0.001
        if (CORRECTION_FACTOR_B <= 0.0) CORRECTION_FACTOR_B = 0.0;  // Asegurar que el factor no sea negativo
        Serial.printf("Correction factor decreased: %.3f\n", CORRECTION_FACTOR_B);
    }

    if (dpad & DPAD_UP) {
        currentState = FORWARD;
    } else if (dpad & DPAD_DOWN) {
        currentState = BACKWARD;
    } else if (dpad & DPAD_LEFT) {
        currentState = TURN_LEFT;
    } else if (dpad & DPAD_RIGHT) {
        currentState = TURN_RIGHT;
    } else {
        currentState = STOP;
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);  // Inicializar I2C con pines definidos

    // Intentar inicializar MPU6050
    if (mpu.begin()) {
        mpuInitialized = true;
        Serial.println("MPU6050 Found!");
    } else {
        Serial.println("Failed to find MPU6050 chip");
    }

    // Intentar inicializar TCS34725 (Sensor de Color)
    if (tcs.begin()) {
        tcsInitialized = true;
        Serial.println("TCS34725 Found!");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
    }

    // Inicializar controladores Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Inicializar pines de los motores
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);

    // Configurar el pin del sensor de voltaje
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);  // Asegurarse de que el pin de voltaje está configurado como entrada

    // Configuración PWM
    ledcSetup(PWM_CHANNEL_A_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_A_IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(MOTOR_A_IN1, PWM_CHANNEL_A_IN1);
    ledcAttachPin(MOTOR_A_IN2, PWM_CHANNEL_A_IN2);
    ledcAttachPin(MOTOR_B_IN1, PWM_CHANNEL_B_IN1);
    ledcAttachPin(MOTOR_B_IN2, PWM_CHANNEL_B_IN2);

    motorOFF();

    // Configurar el ESP32 en modo Access Point
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point IP: ");
    Serial.println(IP);

    // Configurar el servidor web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{";

        // Leer valores actuales de los sensores si están inicializados
        if (mpuInitialized) {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
            pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;
            accelX = a.acceleration.x;
            accelY = a.acceleration.y;
            accelZ = a.acceleration.z;

            json += "\"mpu\": {";
            json += "\"roll\": \"" + String(roll, 2) + "\",";
            json += "\"pitch\": \"" + String(pitch, 2) + "\",";
            json += "\"accelX\": \"" + String(accelX, 2) + "\",";
            json += "\"accelY\": \"" + String(accelY, 2) + "\",";
            json += "\"accelZ\": \"" + String(accelZ, 2) + "\"";
            json += "},";
        }

        if (tcsInitialized) {
            uint16_t r, g_color, b, c;
            tcs.getRawData(&r, &g_color, &b, &c);

            json += "\"tcs\": {";
            json += "\"red\": \"" + String(r) + "\",";
            json += "\"green\": \"" + String(g_color) + "\",";
            json += "\"blue\": \"" + String(b) + "\",";
            json += "\"clear\": \"" + String(c) + "\"";
            json += "},";
        }

        int volt = analogRead(VOLTAGE_SENSOR_PIN); // Leer el voltaje
        voltage = map(volt, 0, 4095, 0, 25) + voltage_offset;
        voltage /= 100.0;
        Serial.print("Voltage: ");
        Serial.println(voltage);

        json += "\"voltage\": {";
        json += "\"value\": \"" + String(voltage, 2) + "\"";
        json += "},";

        json += "\"controller\": {";
        json += "\"connected\": \"" + String(isControllerConnected ? "true" : "false") + "\",";
        json += "\"correction_factor\": \"" + String(CORRECTION_FACTOR_B, 3) + "\"";
        json += "}";

        json += "}";

        request->send(200, "application/json", json);
    });

    server.begin();
}

void loop() {
    // Actualizar datos del controlador
    BP32.update();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            processControllerInput(myControllers[i]);
        }
    }

    // Máquina de estados
    switch (currentState) {
        case IDLE:
            motorOFF();
            break;
        case FORWARD:
            forward(255);
            break;
        case BACKWARD:
            backward(255);
            break;
        case TURN_LEFT:
            left(255);
            break;
        case TURN_RIGHT:
            right(255);
            break;
        case STOP:
            motorOFF();
            currentState = IDLE;  // Regresar a IDLE después de detenerse
            break;
    }

    delay(100);  // Pequeño retardo para evitar saturar la CPU
}

void motorOFF() {
    // Apagar ambos motores
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
}

void forward(int speed) {
    int adjustedSpeedB = speed * CORRECTION_FACTOR_B;  // Ajustar la velocidad del Motor B según el factor de corrección
    // Configurar los motores para avanzar hacia adelante
    ledcWrite(PWM_CHANNEL_A_IN1, speed);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, adjustedSpeedB);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    Serial.println("Moving forward");
}

void backward(int speed) {
    int adjustedSpeedB = speed * CORRECTION_FACTOR_B;  // Ajustar la velocidad del Motor B según el factor de corrección
    // Configurar los motores para retroceder
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, speed);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, adjustedSpeedB);
    Serial.println("Moving backward");
}

void left(int speed) {
    int adjustedSpeedB = speed * CORRECTION_FACTOR_B;  // Ajustar la velocidad del Motor B según el factor de corrección
    // Configurar los motores para girar a la izquierda
    ledcWrite(PWM_CHANNEL_A_IN1, speed);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, adjustedSpeedB);
    Serial.println("Turning left");
}

void right(int speed) {
    int adjustedSpeedB = speed * CORRECTION_FACTOR_B;  // Ajustar la velocidad del Motor B según el factor de corrección
    // Configurar los motores para girar a la derecha
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, speed);
    ledcWrite(PWM_CHANNEL_B_IN1, adjustedSpeedB);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    Serial.println("Turning right");
}
