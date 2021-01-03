#include <Arduino.h>
#include <Wire.h>

#define USE_SERVO_EASING
#define SERVO_COUNT 7

#ifdef USE_SERVO_EASING

#include <ServoEasing.h>

#define FIRST_PCA9685_EXPANDER_ADDRESS 0x40

bool checkI2CConnection(uint8_t aI2CAddress);
void getAndAttachServosToPCA9685Expander(uint8_t aPCA9685I2CAddress, uint8_t nNumServos);

#else

#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x41);

#endif

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const uint8_t digitToSegmentPositions [11][7] = { //for each number, position of every segment
    {1,1,1,0,1,1,1}, // zero - segment number from top lo low, left to right
    {0,0,1,0,0,1,0}, // one
    {1,0,1,1,1,0,1}, // two
    {1,0,1,1,0,1,1}, // three
    {0,1,1,1,0,1,0}, // four
    {1,1,0,1,0,1,1}, // five
    {1,1,0,1,1,1,1}, // six
    {1,0,1,0,0,1,0}, // seven
    {1,1,1,1,1,1,1}, // eight
    {1,1,1,1,0,1,1}, // nine
    {0,0,0,0,0,0,0}  // null (all down)
    };

const uint16_t servoPulse [2] [2] {{400, 210},{210, 450}}; // pulse of servos at low positions and high position

const uint8_t servoReverse [4] [7] = { // one identifies wservos that work reverse, zero is normal direction
    {0,0,1,0,1,0,0},
    //{0,1,0,0,1,0,0},
    {1,0,0,1,0,1,1},
    {1,1,0,1,0,1,1},
    {1,1,0,1,0,1,1}
    };

void showSegment(uint8_t index, uint8_t segment, uint8_t segmentPosition) {
    uint8_t servoNum = index*8 + segment;
    // temporary code to limit number of servos for testing
    if (servoNum >= SERVO_COUNT) { return; }
    uint16_t pulse = servoPulse[servoReverse[index][segment]][segmentPosition];
    // TODO: finetune?

    // TODO: handle second controller? Won't need if we use easing library instead!
#ifdef USE_SERVO_EASING
    sServoArray[servoNum]->startEaseTo(sServoArray[servoNum]->MicrosecondsOrUnitsToDegree(pulse));
#else
    servoDriver.setPWM(servoNum, 0, pulse);
#endif
}

void showDigit(uint8_t index, uint8_t digit, uint16_t segmentDelay) {
    for (uint8_t segment = 0; segment < 7; segment++) {
        uint8_t segmentPosition = digitToSegmentPositions[digit][segment];
        showSegment(index, segment, segmentPosition);
        delay(segmentDelay);
    }
}

void doCount(uint8_t index, uint16_t segmentDelay) {
    for (uint8_t digit = 0 ; digit < 10; digit++) {
        showDigit(0, digit, 0);
        delay(1000);
    }
}

#ifdef USE_SERVO_EASING

/*
 * Check if I2C communication is possible. If not, we will wait forever at endTransmission.
 * 0x40 is default PCA9685 address
 * @return true if error happened, i.e. device is not attached at this address.
 */
bool checkI2CConnection(uint8_t aI2CAddress) {
    bool tRetValue = false;
    Serial.print(F("Try to communicate with I2C device at address=0x"));
    Serial.println(aI2CAddress, HEX);
    Serial.flush();

    Wire.beginTransmission(aI2CAddress);
    uint8_t tWireReturnCode = Wire.endTransmission(true);
    if (tWireReturnCode == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("Error code="));
        Serial.print(tWireReturnCode);
        Serial.print(F(". Communication with I2C was successful, but found no"));
        tRetValue = true;
    }
    Serial.print(F(" I2C device attached at address: 0x"));
    Serial.println(aI2CAddress, HEX);
    return tRetValue;
}

/*
 * Get the ServoEasing objects for the PCA9685 expander
 * The attach() function inserts them in the sServoArray[] array.
 */
void getAndAttachServosToPCA9685Expander(uint8_t aPCA9685I2CAddress, uint8_t nNumServos) {
    ServoEasing * tServoEasingObjectPtr;

    Serial.print(F("Get ServoEasing objects and attach servos to PCA9685 expander at address=0x"));
    Serial.println(aPCA9685I2CAddress, HEX);
    // TODO: check for nNumServos <= PCA9685_MAX_CHANNELS?
    for (uint8_t i = 0; i < nNumServos; ++i) {
#if defined(ARDUINO_SAM_DUE)
        tServoEasingObjectPtr= new ServoEasing(aPCA9685I2CAddress, &Wire1);
#else
        tServoEasingObjectPtr = new ServoEasing(aPCA9685I2CAddress, &Wire);
#endif
        if (tServoEasingObjectPtr->attach(i) == INVALID_SERVO) {
            Serial.print(F("Address=0x"));
            Serial.print(aPCA9685I2CAddress, HEX);
            Serial.print(F(" i="));
            Serial.print(i);
            Serial.println(
                    F(
                            " Error attaching servo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));

        }
    }
}

#endif

void setup() {
    Serial.begin(9600);
    Serial.println("Starting up!");

#ifdef LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
#endif

#ifdef USE_SERVO_EASING
    Wire.begin();
    checkI2CConnection(FIRST_PCA9685_EXPANDER_ADDRESS);
    getAndAttachServosToPCA9685Expander(FIRST_PCA9685_EXPANDER_ADDRESS, SERVO_COUNT);

    setSpeedForAllServos(90/1);
    writeAllServos(sServoArray[0]->MicrosecondsOrUnitsToDegree(servoPulse[0][0]));
    showDigit(0, 8, 0);
#else
    servoDriver.begin();
    //servoDriver.setOscillatorFrequency(27000000);
    servoDriver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    showDigit(0, 8, 0);
#endif
}

bool needToShowInstructions = true;
void loop() {
    if (needToShowInstructions) {
        Serial.println("Type a number, or F for off or R to count 0 to 9");
        needToShowInstructions = false;
    }

    if (Serial.available() > 0) {
        needToShowInstructions = true;
        char input = Serial.read();
        if (input == -1) {
        }

        if (input == 'R' || input == 'r') {
            doCount(0, 0);
        } else if (input == 'F' || input == 'f') {
            showDigit(0, 10, 0);
        } else if (input >= '0' && input <= '9') {
            showDigit(0, input-'0', 0);
        }
    }
}
