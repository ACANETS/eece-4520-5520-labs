
#include <MPU6050.h>

// digital pin connected to joystick switch output (used as 
// accelerometer simulation)
const int sw_pin = 2;

// Analog pin connected to joystick X output
const int kJoystickXPin = 0;

// Analog pin connected to joystick Y output
const int kJoystickYPin = 1;

// Digital pin connected to active buzzer
const int kBuzzerPin = 13;


// Maximum value that the joystick can output in either X or Y direction
const float kMaxJoystickValue = 1023;

// The median value that the joystick rests at in both X and Y direction
const float kJoystickNeutralValue = kMaxJoystickValue / 2;

// The value that, when added to or subtracted from the joystick
// neutral value, the X and Y value must exceed to be counted as
// directional input
const float kJoystickDeadzoneThreshold = 50;

// The gyroscope sensitivity threshold for the MPU6050 module
const int kGyroThresholdSensitivity = 2;

// The time step the gyroscope is being read at in seconds
const float kGyroTimeStep = 0.01;

// The threshold value that the gyroscope pitch and roll must exceed
// in either the positive or negative direction to be counted as 
// directional input
const float kGyroDirectionThreshold = 25;

// The threshold delta that the accelerometer must exceed in any
// direction to be counted as a 'shake'
const float kShakeThreshold = 3.0;

// The amount of time in milliseconds that the buzzer should beep after
// an apple is eaten
const unsigned int kBuzzerBeepTime = 1000;

// The millisecond period at which the buzzer should be on or off
const unsigned int kBuzzerDelay = 75;

// The amount of time in milliseconds that should pass before the
// 'shake' character can be resent
const unsigned int kResendPeriod = 100;

const int kAccelXOffset = -3043;    // Accelerometer calibration x offset 
const int kAccelYOffset = 1803;     // Accelerometer calibration y offset 
const int kAccelZOffset = 502;      // Accelerometer calibration z offset 
const int kGyroXOffset = 335;       // Gyroscope calibration x offset 
const int kGyroYOffset = -82;       // Gyroscope calibration y offset 
const int kGyroZOffset = -26;       // Gyroscope calibration z offset 

const char kUp = 'w';       // Character sent to game for up direction
const char kDown = 's';     // Character sent to game for down direction
const char kLeft = 'a';     // Character sent to game for left direction
const char kRight = 'd';    // Character sent to game for right direction
const char kShaken = 'g';   // Character sent to game for double points

// Enumeration class representing the current direction that the
// snake actor in the game is going
enum class ECurrentDirection {
    STATIONARY = 0, // Snake is not moving
    UP = kUp,       // Snake is going in the upwards direction
    LEFT = kLeft,   // Snake is going in the leftwards direction
    DOWN = kDown,   // Snake is going in the downwards direction
    RIGHT = kRight  // Snake is going in the rightwards direction
};

unsigned int timer = 0;         // Time that the loop starts at
unsigned int resend_timer = 0;  // Time that shaken char is sent
unsigned int buzzer_start_time = 0; // Time buzzer was period is activated
unsigned int last_beep = 0;         // Last time buzzer was toggled on

// Value indicating whether the game is ready to receive the next
// direction value
bool ready_for_next_direction = true;
bool is_shaken = false;     // Value indicating controller is shaken
bool buzzer_on = false;     // Value indicating buzzer should be on or off
bool buzzer_toggle = false; // Switch buzzer on or off

int x = 512;    // Current x value read in from joystick
int y = 512;    // Current y value read in from joystick
int sw = 1;     // Joystick push button value (0 is pressed)
float roll = 0.0;   // Cumulative roll (tilt left or right) value
float pitch = 0.0;  // Cumulative pitch (tilt forward or backward) value
Vector gyro_normalized; // Normalized gyro values from MPU6050 module

int incomingByte = 0;           // Byte sent from game over serial
char controller_direction = 0;  // Direction value sent by controller

// The current direction that the snake actor is heading
ECurrentDirection current_direction = ECurrentDirection::STATIONARY;

// Gyroscope/Accelerometer library object
MPU6050 mpu;


/**
 * @brief Function to determine the directional value from sensor output
 * 
 * @param x_val  The value representing the x-axis point
 * @param y_val  The value representing the y-axis point
 * @param center_x  The value representing the center of the x-axis
 * @param center_y  The value representing the center of the y-axis
 * @param threshold  The threshold plus or minus the center of an axis
                    that a value must cross to be counted
 * @return char representing a directional input for the game
 */
char CheckDirection(float x_val, float y_val, float center_x, float center_y, 
                    float threshold){

    x_val -= center_x;  // Normalize x value from the center of the axis
    y_val -= center_y;  // Normalize y value from the center of the axis

    char new_direction = 0; // The direction value to be returned

     // Calculate the negative x only once
    float neg_x_val = -1 * x_val;  

    // Calculate negative threshold only once
    float neg_threshold = -1 * threshold;

    // Calculate boolean value that y value is not equal to positive
    // or negative x only once
    bool y_not_equal_x = ((y_val != x_val) && (y_val != neg_x_val));

    //              y-axis
    // (y = -x) *     ^     *  (y = x)
    //           .    |    .
    //            .  UP   . 
    //             .  |  . 
    //              . | . 
    //               .|.  
    //      <--LEFT---+---RIGHT-> x-axis
    //               .|.   
    //              . | .  
    //             .  |  .  
    //            . DOWN  . 
    //           .    |    .
    //          *     v     * 
    //
    // The logic below checks against the coordinate system above,
    // assigning one of the directional values of up, left, down, right
    // so long as the y and/or x value is past the threshold from the
    // center of the coordinate system
    if((x_val <= threshold) && (x_val >= neg_threshold)){
        if(y_val > threshold) new_direction = kUp;
        else if(y_val < neg_threshold) new_direction = kDown;
    }
    else if(x_val < neg_threshold){
        if(y_val > neg_x_val) new_direction = kUp;
        else if (y_val < x_val) new_direction = kDown;
        else if (y_not_equal_x) new_direction = kLeft;
    }
    else if(x_val > threshold){
        if(y_val > x_val) new_direction = kUp;
        else if(y_val < neg_x_val) new_direction = kDown;
        else if (y_not_equal_x) new_direction = kRight;
    }

    return new_direction;
}

/**
 * @brief Function to send a directional value to the game so long as 
 *        the direction value doesn't match the current direction the
 *        snake actor is heading
 * 
 * @param dir_to_send  Directional value as a character that should be
 *                    sent
 */
void SendDirection(char dir_to_send){
    if(dir_to_send != static_cast<char>(ECurrentDirection::STATIONARY)){
        if(dir_to_send != static_cast<char>(current_direction)){
            // Send direction character to game
            Serial.println(dir_to_send);

            // Indicate that the game isn't ready for another character
            // to be sent yet
            ready_for_next_direction = false;
        }
    }
}


/**
 * @brief Function to check whether the MPU6050 module is being shaken
 * 
 * @return true if the accelerometer indicates it is being shaken
 * @return false if the accelerometer doesn't determine shaking
 */
bool CheckShaking(){
    bool shaking = false;
    if(!is_shaken){
        if(millis() - resend_timer > kResendPeriod){
            resend_timer = millis();
            shaking = true;
        }
    }
    return shaking;
}

/**
 * @brief The setup function runs at the initialization of the arduino
 *        (either power on or when the reset button is pressed)
 * 
 */
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    
    // Set up digital pins
    pinMode(sw_pin, INPUT_PULLUP);
    pinMode(kBuzzerPin, OUTPUT);

    // Wait until accelerometer/gyroscope model is set up
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
        delay(500);
    }

    // Calibrate gyro while it's at rest
    mpu.calibrateGyro();

    // Set threshold sensitivity
    mpu.setThreshold(kGyroThresholdSensitivity);

    // Set gyro/accelerometer calibration offsets
    mpu.setAccelOffsetX(kAccelXOffset);
    mpu.setAccelOffsetY(kAccelYOffset);
    mpu.setAccelOffsetZ(kAccelZOffset);
    mpu.setGyroOffsetX(kGyroXOffset);
    mpu.setGyroOffsetY(kGyroYOffset);
    mpu.setGyroOffsetZ(kGyroZOffset);
}

/**
 * @brief The main loop function that runs forever after the startup
 *        function
 * 
 */
void loop() {

    timer = millis();   // Get start time of the loop
    controller_direction = 0;   // Clear the controller direction

    // read from the Serial port:
    if (Serial.available() > 0) {

        // read the incoming byte:
        incomingByte = Serial.read();

        // Print out the incoming byte
        Serial.println(static_cast<char>(incomingByte));

        // Determine what information the game is sending
        switch(incomingByte){
            case 'E': {     // The apple is eaten
                    is_shaken = false;
                    buzzer_on = true;
                    buzzer_start_time = millis();
                } break;
            case 'R': {     // The game is starting from the beginning
                    current_direction = ECurrentDirection::STATIONARY;
                    ready_for_next_direction = true;
                } break;
            case 'G': {     // The game received the 'shaken' value
                    is_shaken = true;
                } break;
            case 'W': {     // The game received the 'Up' value
                    current_direction = ECurrentDirection::UP;
                    ready_for_next_direction = true;
                } break;
            case 'A': {     // The game received the 'Left' value
                    current_direction = ECurrentDirection::LEFT;
                    ready_for_next_direction = true;
                } break;
            case 'S': {     // The game received the 'Down' value
                    current_direction = ECurrentDirection::DOWN;
                    ready_for_next_direction = true;
                } break;
            case 'D': {     // The game received the 'Up' value
                    current_direction = ECurrentDirection::RIGHT;
                    ready_for_next_direction = true;
                } break;
        }
    }

    // Read joystick button (being used as accelerometer shaking stand in)
    sw = digitalRead(sw_pin);

    // Only check joystick and gyro if the game is ready for the
    // next direction
    if(ready_for_next_direction){
        // Read joystick first - if no input, read gyro

        x = analogRead(kJoystickXPin);  // Get joystick x value
        y = analogRead(kJoystickYPin);  // Get joystick y value

        // 0 on joystick is up, so invert y so that CheckDirection returns
        // correct value
        controller_direction = CheckDirection(x,
                                            kMaxJoystickValue - y, 
                                            kJoystickNeutralValue, 
                                            kJoystickNeutralValue, 
                                            kJoystickDeadzoneThreshold);

        // If the joystick did not output a direction, check the gyro
        if(controller_direction == static_cast<char>(
            ECurrentDirection::STATIONARY)){

            // Read normalized values from gyro
            gyro_normalized = mpu.readNormalizeGyro();

            // Calculate roll and pitch
            roll += gyro_normalized.XAxis*kGyroTimeStep;
            pitch += gyro_normalized.YAxis*kGyroTimeStep;
            
            // Check gyro directional values
            controller_direction = CheckDirection(roll, pitch, 0, 
                0, kGyroDirectionThreshold);
        }

        // Send whatever direction value was determined
        SendDirection(controller_direction);
    }
    
    // Joystick button pressed, simulate accelerometer being shaken
    if(sw == 0){

        // Check to see if accelerometer module is being shaken, if so
        // send the 'shaken' character to the game
        if(CheckShaking()){
            Serial.println(kShaken);
        }

    }

    // If the game indicates an apple was eaten, turn the buzzer on
    // for the Buzzer Beep Time constant
    if(buzzer_on){

        // Check to see if toggle time has been reached
        if(millis() - last_beep > kBuzzerDelay){
            // Toggle buzzer on/off
            digitalWrite(kBuzzerPin,(buzzer_toggle)?HIGH:LOW);

            // Save buzzer toggle time
            last_beep = millis();

            // Switch toggle value
            buzzer_toggle = !buzzer_toggle;
        }
        
        // Check if Buzzer Beep Time constant has been exceeded
        if(millis() - buzzer_start_time  > kBuzzerBeepTime){
            buzzer_on = false;      // Reset buzzer on value
            buzzer_toggle = false;  // Reset buzzer toggle value

            // Make sure buzzer is off
            digitalWrite(kBuzzerPin,LOW);
        }
    }

    // Delay code until next gyro reading should be taken
    delay((kGyroTimeStep * 1000) - (millis() - timer));
}
