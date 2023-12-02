// Define pin numbers that are connected to the MCU

#define SELECT_PINS_BEGIN 22
#define SELECT_PINS_END 37
#define NUMBER_OF_MUXES 4
#define OUTPUT_BUFFER_SIZE 8

// define Analog inputs in use
const int muxesOutputPins[] = {A0, A1, A2, A3};

// outputBuffer to send data over serial
unsigned short outputBuffer[OUTPUT_BUFFER_SIZE];


void setup() {

  Serial.begin(9600);

  // Initialize the analog pins as outputs
  for (int i = 0 ; i < NUMBER_OF_MUXES ; i++) {
    pinMode(muxesOutputPins[i], INPUT);
  }

  // Initialize the digital selector pins and set them initially to zero
  for (int pinNumber = SELECT_PINS_BEGIN ; pinNumber <= SELECT_PINS_END ; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }

}

void loop() {

  // Assure that these pins are low in the beginning
  // each Pin corresponds to the highest (MSB) selection pin of the MUX => s3
  digitalWrite(25, 0);
  digitalWrite(29, 0);
  digitalWrite(33, 0);
  digitalWrite(37, 0);

  // three nested loops to cycle through every possibility of a 3bit number
  for (unsigned short i = 0 ; i <= 1 ; i++) {
    digitalWrite(24, i);
    digitalWrite(28, i);
    digitalWrite(32, i);
    digitalWrite(36, i);
    for (unsigned short j = 0 ; j <= 1 ; j++) {
      digitalWrite(23, j);
      digitalWrite(27, j);
      digitalWrite(31, j);
      digitalWrite(35, j);
      for (unsigned short k = 0 ; k <= 1 ; k++) {
        digitalWrite(22, k);
        digitalWrite(26, k);
        digitalWrite(30, k);
        digitalWrite(34, k);

        // Read
        outputBuffer[0] = analogRead(muxesOutputPins[0]);
        outputBuffer[1] = analogRead(muxesOutputPins[1]);
        outputBuffer[2] = analogRead(muxesOutputPins[2]);
        outputBuffer[3] = analogRead(muxesOutputPins[3]);
        outputBuffer[4] = 0;
        outputBuffer[5] = i;
        outputBuffer[6] = j;
        outputBuffer[7] = k;  

        Serial.print(outputBuffer[0]);
        Serial.print(" ");
        Serial.print(outputBuffer[1]);
        Serial.print(" ");
        Serial.print(outputBuffer[2]);
        Serial.print(" ");
        Serial.print(outputBuffer[3]);
        Serial.print(" ");
        Serial.print(outputBuffer[4]);
        Serial.print(" ");
        Serial.print(outputBuffer[5]);
        Serial.print(" ");
        Serial.print(outputBuffer[6]);
        Serial.print(" ");
        Serial.println(outputBuffer[7]);

      }
    }
  }

  // Read remaining 2 pins on every MUX that are not covered by the for loops
  // First set the pins to high or low that are not going to change for reading the remaining 2 pins
  digitalWrite(25, 1);
  digitalWrite(29, 1);
  digitalWrite(33, 1);
  digitalWrite(37, 1);

  digitalWrite(23, 0);
  digitalWrite(27, 0);
  digitalWrite(31, 0);
  digitalWrite(35, 0);

  digitalWrite(24, 0);
  digitalWrite(28, 0);
  digitalWrite(32, 0);
  digitalWrite(36, 0);

  // Actually Reading the remaining two pins
  for (unsigned short k = 0 ; k <= 1 ; k++) {
        digitalWrite(22, k);
        digitalWrite(26, k);
        digitalWrite(30, k);
        digitalWrite(34, k);

        outputBuffer[0] = analogRead(muxesOutputPins[0]);
        outputBuffer[1] = analogRead(muxesOutputPins[1]);
        outputBuffer[2] = analogRead(muxesOutputPins[2]);
        outputBuffer[3] = analogRead(muxesOutputPins[3]);
        outputBuffer[4] = 1;
        outputBuffer[5] = 0;
        outputBuffer[6] = 0;
        outputBuffer[7] = k;

        Serial.print(outputBuffer[0]);
        Serial.print(" ");
        Serial.print(outputBuffer[1]);
        Serial.print(" ");
        Serial.print(outputBuffer[2]);
        Serial.print(" ");
        Serial.print(outputBuffer[3]);
        Serial.print(" ");
        Serial.print(outputBuffer[4]);
        Serial.print(" ");
        Serial.print(outputBuffer[5]);
        Serial.print(" ");
        Serial.print(outputBuffer[6]);
        Serial.print(" ");
        Serial.println(outputBuffer[7]);

      }

  
}
