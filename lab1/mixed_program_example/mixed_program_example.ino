// Example of Using Mixed C and Assembly language on Arduino
// references 
// [1] https://youtu.be/j-qs-gJhxfs?si=RS08j62B1jNVmDRR
// [2] https://msoe.us/taylor/tutorial/ce2810/candasm

extern "C" // function prototypes external to C sketch
{
  void start();
  void led(byte);
  uint8_t shift_add_in_assembly(uint8_t i, uint8_t j);
}

uint8_t c;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  start();
  c = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i=2, j=4, s;
  led(1);
  delay(200);
  led(0);
  delay(200);
  if (c==0) {
    // call the assembly function by passing in two arguments
    // and then retrieving one return value
    s = shift_add_in_assembly(i, j);
    // send to serial port to check
    Serial.print(s);
    // increase c so that this block is executed only once
    c++;
  }
}
