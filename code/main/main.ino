#include <Arduino.h>

class Motor {
  public:
    uint8_t _m1,_m2, _p_pwm;
    int16_t pwm = 0;
    Motor(uint8_t m1, uint8_t m2, uint8_t p_pwm) {
      _m1 = m1;
      _m2 = m2;
      _p_pwm = p_pwm;
      pinMode(_m1, OUTPUT);
      pinMode(_m2, OUTPUT);
      pinMode(_p_pwm, OUTPUT);
    }
    void run(int16_t val) {
      pwm = constrain(val, -255, 255);
      analogWrite(_p_pwm, abs(pwm));
      if (pwm>0){
        digitalWrite(_m1, 0);
        digitalWrite(_m2, 1);
      }
      else if (pwm<0){
        digitalWrite(_m1, 1);
        digitalWrite(_m2, 0);
      }
      else {
        digitalWrite(_m1, 0);
        digitalWrite(_m2, 0);
      }
    }
};

volatile uint32_t t_controlTask = 0;
volatile uint32_t t_telemetryTask = 0;

Motor motorL(PB4,PB5,PB9);
Motor motorR(PB6,PB7,PB8);

const int SENSOR_PINS[9] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0};
uint16_t sensor_raw[9];
uint8_t sensor_digital[9];
uint16_t sensor_threshold[9];
uint16_t sensor_min[9] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
uint16_t sensor_max[9] = {0,    0,    0,    0,    0,    0,    0,    0,    0   };
const int32_t WEIGHTS[9] = {-3000, -1500, -700, -300, 0, 300, 700, 1500, 3000};

const int BASE_SPEED = 50;
const int R = 50;
float Kp = 0.06;
float Ki = 0.0;
float Kd = 0;

int32_t last_error = 0;
int32_t integral = 0;

volatile bool run = 0;

// BUTTON
const uint8_t BUTTON_PIN = PB13; 
unsigned long pressStartTime = 0;
bool isButtonDown = false;
bool longPressHandled = false;
const unsigned long DEBOUNCE_DELAY = 50; 
const unsigned long LONG_PRESS_TIME = 5000;

uint8_t obstacle_state = 0; 
uint8_t OBSTACLE_BTN_PIN = PA8;
// Biến chống dội cho nút nhấn
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; 
bool lastButtonState = HIGH;

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  for (int i = 0; i < 9; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PC13, OUTPUT);
  pinMode(OBSTACLE_BTN_PIN, INPUT_PULLUP);
  delay(1000);
  
}

void loop() {
  handleButton();
  checkObstacle();
  controlTask();
  telemetryTask();

}