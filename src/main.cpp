#include <Arduino.h>

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x41);

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
    {1,1,1,1,0,1,1},  // nine
    {0,0,0,0,0,0,0}  // null (all down)
    };

const uint16_t servoPulse [2] [2] {{440, 210},{210, 450}}; // pulse of servos at low positions and high position

const uint8_t servoReverse [4] [7] = { // one identifies wservos that work reverse, zero is normal direction
    {0,0,1,0,1,0,0},
    //{0,1,0,0,1,0,0},
    {1,0,0,1,0,1,1},
    {1,1,0,1,0,1,1},
    {1,1,0,1,0,1,1}
    };

void showSegment(uint8_t index, uint8_t segment, uint8_t segmentPosition) {
    uint8_t servoNum = index*8 + segment;
    uint16_t pulse = servoPulse[servoReverse[index][segment]][segmentPosition];
    // TODO: finetune?

    // TODO: handle second controller? Won't need if we use easing library instead!
    servoDriver.setPWM(servoNum, 0, pulse);
}

void showDigit(uint8_t index, uint8_t digit, uint16_t segmentDelay) {
    for (uint8_t segment = 0; segment < 7; segment++) {
        uint8_t segmentPosition = digitToSegmentPositions[digit][segment];
        showSegment(index, segment, segmentPosition);
        delay(segmentDelay);
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Starting up!");

    pinMode(LED_BUILTIN, OUTPUT);

    //pinMode(D3, OUTPUT);
    //digitalWrite(D3, LOW);

    servoDriver.begin();
    //servoDriver.setOscillatorFrequency(27000000);
    servoDriver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

#if 0

void loop() {
    Serial.println("Setting digit 0 to 8");
    showDigit(0, 8, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10000);

    for (uint8_t segment = 0; segment < 7; segment++) {
        Serial.print("Removing digit 0 segment ");
        Serial.println('A' + segment);
        showSegment(0, segment, 0);
        delay(10000);
    }
}

#else

void loop() {
    for (uint8_t num = 0 ; num < 10; num++) {
        Serial.print("Setting digit 0 to ");
        Serial.println(num);
        showDigit(0, num, 0);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
    }

    Serial.println("Setting digit 0 to NULL");
    showDigit(0, 10, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
 }

#endif

#if 0

#include <ServoEasing.h>

#define FIRST_PCA9685_EXPANDER_ADDRESS 0x40

bool checkI2CConnection(uint8_t aI2CAddress);
void getAndAttachServosToPCA9685Expander(uint8_t aPCA9685I2CAddress, uint8_t nNumServos);

void setup() {
    Serial.begin(9600);
    Serial.println("Starting up!");

    pinMode(LED_BUILTIN, OUTPUT);

    Wire.begin();
    checkI2CConnection(FIRST_PCA9685_EXPANDER_ADDRESS);
    getAndAttachServosToPCA9685Expander(FIRST_PCA9685_EXPANDER_ADDRESS, 1);

    writeAllServos(0);
    setSpeedForAllServos(20);
}

void loop() {
    Serial.println("Hello world!");
    digitalWrite(LED_BUILTIN, LOW);
    sServoArray[0]->startEaseTo(90);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    sServoArray[0]->startEaseTo(0);
    delay(3000);
}

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