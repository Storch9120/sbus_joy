// Declare switch pins
#define A 6
#define B 5
#define C 4
#define D 3
#define SW 7
// Declare joystick pins
#define X A0
#define Y A1
// Declare thumbwheel pin
#define op_pin A2

struct dataPin {
uint8_t opval;
uint8_t xval;
uint8_t yval;
uint8_t switchA;
uint8_t switchB;
uint8_t switchC;
uint8_t switchD;
uint8_t switchSW;
} data;

// serialize the struct into bytes
void serializeStruct(const dataPin& data, byte* buffer) {
  buffer[0] = 0xFF;
  memcpy(buffer + 1, &data.opval, sizeof(uint8_t));
  memcpy(buffer + 2, &data.xval, sizeof(uint8_t));
  memcpy(buffer + 3, &data.yval, sizeof(uint8_t));
  memcpy(buffer + 4, &data.switchA, sizeof(uint8_t));
  memcpy(buffer + 5, &data.switchB, sizeof(uint8_t));
  memcpy(buffer + 6, &data.switchC, sizeof(uint8_t));
  memcpy(buffer + 7, &data.switchD, sizeof(uint8_t));
  memcpy(buffer + 8, &data.switchSW, sizeof(uint8_t));
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(op_pin, INPUT);
  pinMode(X, INPUT);
  pinMode(Y, INPUT);
  pinMode(A, INPUT_PULLUP); // Initialize internal pull-up resistor to avoid floating
  pinMode(B, INPUT_PULLUP);
  pinMode(C, INPUT_PULLUP);
  pinMode(D, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
}


// int flag = 0;

void loop() {
  // put your main code here, to run repeatedly:

  data.opval = (map(analogRead(op_pin), 0, 1023, 0, 254));
  data.xval = map(analogRead(X), 0, 1023, 0, 254);
  data.yval = map(analogRead(Y), 0, 1023, 0, 254);
  data.switchA = (digitalRead(A) == LOW);
  data.switchB = (digitalRead(B) == LOW);
  data.switchC = (digitalRead(C) == LOW);
  data.switchD = (digitalRead(D) == LOW);
  data.switchSW = (digitalRead(SW) == LOW);

  // byte buffer[1+3*sizeof(uint8_t)+5*sizeof(uint8_t)];
  byte buffer[1+8*sizeof(uint8_t)];
  serializeStruct(data, buffer);
  Serial.write(buffer, sizeof(buffer));
  Serial.println();

  delay(5);

}
