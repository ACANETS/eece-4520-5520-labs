
const int sw_pin = 2;   // digital pin connected to joystick switch output
const int x_pin = 0;    // analog pin connected to X output
const int y_pin = 1;    // analog pin connected to Y output
const int buzzer_pin = 13;

const unsigned int kMaxJoystickValue = 1023;
const unsigned int kJoystickNeutralValue = kMaxJoystickValue / 2;
const unsigned int kJoystickDeadzoneTolerance = 50;

const char kUp = 'w';
const char kDown = 's';
const char kLeft = 'a';
const char kRight = 'd';

const unsigned int kBuzzerBeepTime = 1000;
const unsigned int kBuzzerDelay = 75;

int incomingByte = 0;
int x = 512;
int y = 512;
int sw = 1;

char direction = 0;
bool buzzer_on = false;
unsigned int buzzer_start_time = 0;
unsigned int last_beep = 0;
bool buzzer_toggle = false;

char CheckDirection(unsigned int x_val, unsigned int y_val){
    char joystick_direction = 0;

    //  Joystick map sectors are determined by the middle joystick
    //  value plus or minus the deadzone tolerance, making the middle
    //  sector (sector 5) a complete deadzone. Other deadzones are on
    //  the diagonals that are between up, down, left, and right.
    //
    // Joystick map sectors:
    // (0, MAX) *-----------* (MAX, MAX)
    //          | 7 | 8 | 9 |
    //          |---*---*---|
    //          | 4 | 5 | 6 |
    //          |---*---*---|
    //          | 1 | 2 | 3 |
    // (0, 0)   *-----------* (MAX, 0)
    //
    // NOTE - Sector 5 will always be a dead zone
    //
    // (0, MAX) *-----------* (MAX, MAX)
    //          |\   down  /|
    //          | \       / |
    //          |  \     /  |
    //          | l \   / r |
    //          | e  \ /  i |
    //          | f   X   g |
    //          | t  / \  h |
    //          |   /   \ t |
    //          |  /     \  |
    //          | /       \ |
    //          |/   up    \|
    // (0, 0)   *-----------* (MAX, 0)
    if(x_val <= (kJoystickNeutralValue - kJoystickDeadzoneTolerance)){
        // Sector 1
        if(y_val <= (kJoystickNeutralValue - kJoystickDeadzoneTolerance)){
            if(x_val > y_val) joystick_direction = kUp;
            else if (x_val < y_val) joystick_direction = kLeft;
        }
        
        // Sector 4
        else if(y_val > (kJoystickNeutralValue - kJoystickDeadzoneTolerance)
            && y_val < (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
            joystick_direction = kLeft;
        }

        // Sector 7
        else if(y_val >= (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
            if(x_val > (kMaxJoystickValue - y_val)) joystick_direction = kDown;
            else if (x_val < (kMaxJoystickValue - y_val)) joystick_direction = kLeft;
        }
    }
    else if(x_val > (kJoystickNeutralValue - kJoystickDeadzoneTolerance)
            && x_val < (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
        // Sector 2
        if(y_val <= (kJoystickNeutralValue - kJoystickDeadzoneTolerance)){
            joystick_direction = kUp;
        }

        // Sector 8
        else if(y_val >= (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
            joystick_direction = kDown;
        }
    }
    else if(x_val >= (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
        // Sector 3
        if(y_val <= (kJoystickNeutralValue - kJoystickDeadzoneTolerance)){
            if((kMaxJoystickValue - x_val) > y_val) joystick_direction = kUp;
            else if ((kMaxJoystickValue - x_val) < y_val) joystick_direction = kRight;
        }
        
        // Sector 6
        else if(y_val > (kJoystickNeutralValue - kJoystickDeadzoneTolerance)
            && y_val < (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
            joystick_direction = kRight;
        }

        // Sector 9
        else if(y_val >= (kJoystickNeutralValue + kJoystickDeadzoneTolerance)){
            if(x_val > y_val) joystick_direction = kDown;
            else if (x_val < y_val) joystick_direction = kRight;
        }
    }

    return joystick_direction;
}

// the setup routine runs once when you press reset:
void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    pinMode(sw_pin, INPUT_PULLUP);
    pinMode(buzzer_pin, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() {
    // read from the Serial port:
    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        Serial.println(incomingByte);

        if(incomingByte == 'E') {
            buzzer_on = true;
            buzzer_start_time = millis();
        }
    }

    x = analogRead(x_pin);
    y = analogRead(y_pin);
    sw = digitalRead(sw_pin);

    direction = CheckDirection(x, y);
    if(direction != 0) Serial.println(direction);

    if(sw == 0){
        Serial.println('g');
    }

    if(buzzer_on){
        if(millis() - last_beep > kBuzzerDelay){
            digitalWrite(buzzer_pin,(buzzer_toggle)?HIGH:LOW);
            last_beep = millis();
            buzzer_toggle = !buzzer_toggle;
        }
        
        if(millis() - buzzer_start_time  > kBuzzerBeepTime){
            buzzer_on = false;
            buzzer_toggle = false;
            digitalWrite(buzzer_pin,LOW);
        }
    }

    delay(250);
}
