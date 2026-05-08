#include <Arduino.h>
// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
//constexpr uint8_t PIN_SENSOR_LEFT   = A1;
constexpr uint8_t PIN_SENSOR  = A2;
constexpr uint8_t PIN_PWM_LEFT      = D10;
constexpr uint8_t PIN_PWM_RIGHT     = D9;

// ---------------------------------------------------------------------------
// ADC configuration
// ---------------------------------------------------------------------------
constexpr int ADC_BITS = 12;
constexpr int ADC_MAX  = (1 << ADC_BITS) - 1;
// ---------------------------------------------------------------------------
// PWM configuration
// ---------------------------------------------------------------------------
constexpr int PWM_FREQ_HZ   = 5000;
constexpr int PWM_BITS      = 8;
constexpr int PWM_MAX_DUTY  = (1 << PWM_BITS) - 1;
constexpr int PWM_MIN_DUTY  = 0;
// ---------------------------------------------------------------------------
// Motor speed limits
// ---------------------------------------------------------------------------
constexpr int BASE_DUTY  = 80;
constexpr int MAX_DUTY   = 100;
// ---------------------------------------------------------------------------
// Moving average filter
// ---------------------------------------------------------------------------
constexpr uint8_t FILTER_SIZE = 8;
struct MovingAvg {
    int32_t buf[FILTER_SIZE] = {};
    int64_t sum              = 0;
    uint8_t index            = 0;
    bool    full             = false;
    float update(int32_t sample) {
        sum       -= buf[index];
        buf[index] = sample;
        sum       += sample;
        index      = (index + 1) % FILTER_SIZE;
        if (index == 0) full = true;
        uint8_t n  = full ? FILTER_SIZE : (index == 0 ? FILTER_SIZE : index);
        return static_cast<float>(sum) / n;
    }
    void seed(int32_t value) {
        for (uint8_t i = 0; i < FILTER_SIZE; i++) buf[i] = value;
        sum   = static_cast<int64_t>(value) * FILTER_SIZE;
        index = 0;
        full  = true;
    }
};
MovingAvg filter;
// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
constexpr uint16_t CALIBRATION_SAMPLES  = 200;
constexpr uint16_t CALIBRATION_DELAY_US = 500;
float cal  = ADC_MAX / 2.0f;
void calibrate() {
    Serial.println("Calibrating - centre mouse over wire, then press RESET.");
    delay(2000);
    int64_t sum = 0;
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += analogRead(PIN_SENSOR);
        delayMicroseconds(CALIBRATION_DELAY_US);
    }
    cal  = static_cast<float>(sum) / CALIBRATION_SAMPLES;
    
    if (cal  < 10.0f) cal  = ADC_MAX / 2.0f;
    filter.seed(static_cast<int32_t>(cal));
}
// ---------------------------------------------------------------------------
// Signal-loss threshold
// ---------------------------------------------------------------------------
constexpr float LOW_SIGNAL_THRESHOLD = 80.0f;
// ---------------------------------------------------------------------------
// PID parameters
// ---------------------------------------------------------------------------
constexpr float KP = 80.0f;
constexpr float KI =  2.0f;
constexpr float KD = 15.0f;
constexpr float INTEGRAL_MAX =  2.0f;
constexpr float INTEGRAL_MIN = -2.0f;
// ---------------------------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------------------------
constexpr unsigned long LOOP_PERIOD_MS = 10;
// ---------------------------------------------------------------------------
// PID state
// ---------------------------------------------------------------------------
float pidIntegral    = 0.0f;
float pidPrevError   = 0.0f;
float lastCorrection = 0.0f;
unsigned long lastLoopTime = 0;
// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
inline int clampDuty(int v) {
    if (v < PWM_MIN_DUTY) return PWM_MIN_DUTY;
    if (v > MAX_DUTY)     return MAX_DUTY;
    return v;
}
void setMotorDuty(uint8_t pin, int duty) {
    analogWrite(pin, clampDuty(duty));
}
// ---------------------------------------------------------------------------
// PID update
// ---------------------------------------------------------------------------
float pidUpdate(float error, float dt) {
    float pTerm = KP * error;
    pidIntegral = constrain(pidIntegral + error * dt, INTEGRAL_MIN, INTEGRAL_MAX);
    float iTerm = KI * pidIntegral;
    float dTerm = (dt > 0.0f) ? KD * (error - pidPrevError) / dt : 0.0f;
    pidPrevError = error;
    return pTerm + iTerm + dTerm;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_PWM_LEFT,  OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);
  analogWrite(PIN_PWM_LEFT,  0);
  analogWrite(PIN_PWM_RIGHT, 0);
  calibrate();
  lastLoopTime = millis();
}
float errorChange = 0;
float pastError = 0;
float error = 0;
int loopCount = 0;
int Correction = 0;
int32_t raw  = analogRead(PIN_SENSOR);
int32_t origin  = filter.update(raw)/cal;
void loop() {
  // put your main code here, to run repeatedly:
    unsigned long now     = millis();
    unsigned long elapsed = now - lastLoopTime;
    if (elapsed < LOOP_PERIOD_MS) return;
    float dt     = elapsed * 0.001f;
    lastLoopTime = now;
    
    
    
    if (loopCount == 0){
        raw  = analogRead(PIN_SENSOR);
        float filtPast  = filter.update(raw);
        float filtCurrent = filter.update(raw);

        bool signalOk = (filtCurrent  > LOW_SIGNAL_THRESHOLD);
        int dutyLeft, dutyRight;
        if (signalOk) {
            float normPast  = filtPast / cal;
            float normCurrent  = filtCurrent / cal;
            error = origin - normCurrent;
            errorChange = normPast - normCurrent;
            float correction = pidUpdate(error, dt);
            lastCorrection   = correction;
            dutyLeft  = clampDuty(BASE_DUTY + static_cast<int>(correction));
            dutyRight = clampDuty(BASE_DUTY - static_cast<int>(correction));
            setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
            setMotorDuty(PIN_PWM_RIGHT, dutyRight);
            
            Correction = 0;

        } else {
            pidIntegral = 0.0f;
            if (errorChange > 0){
                Correction =+1;
            }
            if (Correction < 4){
                dutyLeft  = clampDuty(BASE_DUTY - static_cast<int>(lastCorrection));
                dutyRight = clampDuty(BASE_DUTY + static_cast<int>(lastCorrection));
                setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
                setMotorDuty(PIN_PWM_RIGHT, dutyRight);
            }else{
                dutyLeft  = clampDuty(BASE_DUTY + static_cast<int>(lastCorrection));
                dutyRight = clampDuty(BASE_DUTY - static_cast<int>(lastCorrection));
                setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
                setMotorDuty(PIN_PWM_RIGHT, dutyRight);
            }
            
            

            
        }

    }else{

        float filtPast  = filter.update(raw);
        raw  = analogRead(PIN_SENSOR);
        float filtCurrent = filter.update(raw);

        bool signalOk = (filtCurrent  > LOW_SIGNAL_THRESHOLD);
        int dutyLeft, dutyRight;
        if (signalOk) {
            float normPast  = filtPast / cal;
            float normCurrent  = filtCurrent / cal;
            error = origin - normCurrent;
            errorChange = normPast - normCurrent;
            float correction = pidUpdate(error, dt);
            lastCorrection   = correction;
            dutyLeft  = clampDuty(BASE_DUTY + static_cast<int>(correction));
            dutyRight = clampDuty(BASE_DUTY - static_cast<int>(correction));
            setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
            setMotorDuty(PIN_PWM_RIGHT, dutyRight);
        } else {
            pidIntegral = 0.0f;
            if (errorChange > 0){
                Correction =+1;
            }
            if (Correction < 4){
                dutyLeft  = clampDuty(BASE_DUTY - static_cast<int>(lastCorrection));
                dutyRight = clampDuty(BASE_DUTY + static_cast<int>(lastCorrection));
                setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
                setMotorDuty(PIN_PWM_RIGHT, dutyRight);
            }else{
                dutyLeft  = clampDuty(BASE_DUTY + static_cast<int>(lastCorrection));
                dutyRight = clampDuty(BASE_DUTY - static_cast<int>(lastCorrection));
                setMotorDuty(PIN_PWM_LEFT,  dutyLeft);
                setMotorDuty(PIN_PWM_RIGHT, dutyRight);
            }
        }
 
    }
    loopCount =1;
}
