#define ERROR 1000000.0
#define DEGREES_PER_CM 81.85

#define STEPS_PER_ROTATION 200
#define STEPS_PER_DEGREE (STEPS_PER_ROTATION/360.0)

// All inaccurate, need to measure
#define SLIDER_MAX_HEIGHT 30.0
#define BUILD_PLATE_RADIUS 10.0
#define ARM_LENGTH 23.0 // they are actually 23
#define SLIDER_DISTANCE_FROM_CENTER 14.0

#define MAX_INSTRUCTION_COUNT 115

struct Vector3 {
  float x, y, z;
  
  Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
  
  Vector3 operator+(const Vector3& other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
  }
  
  Vector3 operator-(const Vector3& other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
  }
};

// Trick to use less IRAM
ICACHE_FLASH_ATTR float flash_cos(float angle) {
  return cos(angle);
}

ICACHE_FLASH_ATTR float flash_sin(float angle) {
  return sin(angle);
}

ICACHE_FLASH_ATTR float flash_sqrt(float value) {
  return sqrt(value);
}

class Stepper {
private:
  float rotation = 0;
  int stepCount = 0;
  int targetStepCount = 0;
  long previousStepMillis = 0;
  int currentDirection = 0;
  byte directionPin;
  byte stepPin;

  bool stepToggle = false;
public:
  // less speed makes the motor go faster 
  int speed = 5;
  // true when the motor is done executing an instruction
  bool isDone = false;

  Stepper(byte _directionPin, byte _stepPin) {
    pinMode(_directionPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    stepPin = _stepPin;
    directionPin = _directionPin;
    previousStepMillis = millis();
  };
  
  void rotate(float degrees) {
    if (degrees > 0) {
      digitalWrite(directionPin, LOW);
      currentDirection = 1;
    } else {
      digitalWrite(directionPin, HIGH);
      currentDirection = -1;
    }
    rotation = fmod(rotation + degrees + 360.0, 360.0);
    targetStepCount = stepCount + (int)(degrees * STEPS_PER_DEGREE);
    // Serial.print("target step:");
    // Serial.println(targetStepCount);
    isDone = false;
  }
  
  void step(long currentMillis) {
    if (stepCount == targetStepCount) {
      isDone = true;
      return;
    }
    if (currentMillis - previousStepMillis >= speed) {
      previousStepMillis = currentMillis;
      stepCount = stepCount + currentDirection;
      if (stepToggle) digitalWrite(stepPin, HIGH);
      else digitalWrite(stepPin, LOW);
      stepToggle = !stepToggle;
    }
  }

  void shutdown() {
    pinMode(stepPin, INPUT);
    pinMode(directionPin, INPUT);
    Serial.println("Shutdown");
  }

  float getRotation() {
    return rotation;
  }
};

class PositionedObject {
private:
  Vector3 position;
public:
  float getX() { return position.x; }
  float getY() { return position.y; }
  float getZ() { return position.z; }
  void  setX(float x) { position.x = x; }
  void  setY(float y) { position.y = y; }
  void  setZ(float z) { position.z = z; }

  Vector3 getPosition() { return position; }
  void setPosition(Vector3 p) { position = p; }
};


class Slider : PositionedObject {
private:
  Stepper motor;

public:
  // returns true when slider has reached its target position
  bool update(long currentMillis) {
    motor.step(currentMillis);
    return motor.isDone;
  }

  void shutdownMotor() {
    motor.shutdown();
  }

  void move(float distance, float speed) {
    if (distance == ERROR) return;
    setY(getY() + distance);
    motor.speed = speed;
    motor.rotate(distance * DEGREES_PER_CM);
  }

  float findHeightChange(PositionedObject* head) {
    float deltaX = head->getX() - getX();
    float deltaZ = head->getZ() - getZ();
    float horizontalDistance = flash_sqrt(deltaX*deltaX + deltaZ*deltaZ);
    
    // Check if position is reachable
    if (horizontalDistance > ARM_LENGTH) {
        Serial.println("Position unreachable - too far horizontally");
        return ERROR;
    }
    
    float heightSquared = ARM_LENGTH*ARM_LENGTH - horizontalDistance*horizontalDistance;
    float armVerticalLength = flash_sqrt(heightSquared);
    
    // Try both solutions and pick the valid one
    float heightUpper = head->getY() + armVerticalLength;
    float heightLower = head->getY() - armVerticalLength;

    // Serial.print("Upper: ");
    // Serial.print(heightUpper);
    // Serial.print("  Lower: ");
    // Serial.print(heightLower);
    // Serial.print("  Horizontal Distance: ");
    // Serial.println(horizontalDistance);
    
    // Use the solution that's within bounds
    float height;
    if (heightUpper >= 0 && heightUpper <= SLIDER_MAX_HEIGHT) {
        height = heightUpper;
    } else if (heightLower >= 0 && heightLower <= SLIDER_MAX_HEIGHT) {
        height = heightLower;
    } else {
        return ERROR;
    }
    
    // Serial.print("Height:");
    // Serial.println(height);
    return height - getY();
  }
  
  Slider(float angle, PositionedObject* head, byte directionPin, byte stepPin) : motor(directionPin, stepPin) {
    // First set y position to 0 so that it can be used in "findHeightChange"
    // Serial.print("Angle: ");
    // Serial.println(angle);
    setPosition(Vector3(SLIDER_DISTANCE_FROM_CENTER * flash_cos(angle), 0.0, SLIDER_DISTANCE_FROM_CENTER * flash_sin(angle)));
    setY(findHeightChange(head));
    // Serial.println(getY());
  };

};

class PrintHead : public PositionedObject {
public:
  PrintHead() {
    setPosition(Vector3(0.0, 8.0, 0.0));
  }
  void move(Vector3 target, float speed, Slider& slider1, Slider& slider2, Slider& slider3) {
    setPosition(target);
    slider1.move(slider1.findHeightChange(this), speed);
    slider2.move(slider2.findHeightChange(this), speed);
    slider3.move(slider3.findHeightChange(this), speed);
  }
};


class Instruction : public PositionedObject {
private:
  int speed;

public:
  Instruction() : speed(0.0) {
    setPosition({0, 0, 0});
  }
  Instruction(int s, Vector3 p) : speed(s) {
    setPosition(p);
  }

  int getSpeed() {
    return speed;
  }
};

class Printer {
private:
  PrintHead head = PrintHead();
  // const int dirPins[] = {12, 16, 4};
  // const int stepPins[] = {13, 5, 14};
  Slider slider1 = Slider(0*TWO_PI/3, &head, 12, 13);
  Slider slider2 = Slider(1*TWO_PI/3, &head, 16, 5);
  Slider slider3 = Slider(2*TWO_PI/3, &head, 4, 14);
  Instruction instructions[MAX_INSTRUCTION_COUNT];
  int instructionCount = 0;
public:
  // true if it has finished all instructions
  bool finished = false;

  Printer() {}
  
  void update(long currentMillis) {
    bool slider1Done = slider1.update(currentMillis);
    bool slider2Done = slider2.update(currentMillis);
    bool slider3Done = slider3.update(currentMillis);
    if (slider1Done && slider2Done && slider3Done) {
      Serial.println("Instruction complete!");
      if (instructionCount > 0) {
        Serial.println("Starting next one...");
        delay(50);
        executeNextInstruction();
      } else {
        finished = true;
        slider1.shutdownMotor();
        slider2.shutdownMotor();
        slider3.shutdownMotor();
      }
    }
  }

  void executeNextInstruction() {
    Instruction instruction = instructions[instructionCount - 1];
    head.move(instruction.getPosition(), instruction.getSpeed(), slider1, slider2, slider3);
    instructionCount--;
    Serial.print(instruction.getPosition().x);
    Serial.print(", ");
    Serial.print(instruction.getPosition().y);
    Serial.print(", ");
    Serial.println(instruction.getPosition().z);
  }

  void addInstruction(int speed, Vector3 endPosition) {
    instructions[instructionCount] = Instruction(speed, endPosition);
    instructionCount++;
  }
  
  void reverseInstructions() {
    for (int i = 0; i < instructionCount / 2; i++) {
      // Swap elements from start and end, working toward middle
      Instruction temp = instructions[i];
      instructions[i] = instructions[instructionCount - 1 - i];
      instructions[instructionCount - 1 - i] = temp;
    }
  }

  Instruction getLastInstruction() {
    if (instructionCount == 0) return Instruction();
    return instructions[instructionCount - 1];
  }
};

Printer printer;

void addCircleInstructions(int N, float radius, float centerX, float centerZ, float height, int speed) {
    for (int i = 0; i < N; i++) {
        float angle = (6.28318531 * i) / N;
        float x = centerX + radius * flash_cos(angle);
        float z = centerZ + radius * flash_sin(angle);
        printer.addInstruction(speed, Vector3(x, height, z));
    }
}

void addLinearInstructions(int N, float x1, float y1, float z1, float x2, float y2, float z2, int speed) {
    for (int i = 0; i < N; i++) {
        float t = (float)i / (N - 1);
        
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        float z = z1 + t * (z2 - z1);
        printer.addInstruction(speed, Vector3(x, y, z));
    }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  delay(1000); // Give it extra time
  Serial.println("Starting...");

  printer = Printer();

  // the first param is speed, and less speed makes it go faster
  // printer.addInstruction(2, {5.0, 8.0, 5.0});
  // printer.addInstruction(2, {3.0, 10.0, 3.0});
  // printer.addInstruction(2, {-4.0, 9.0, 1.0});
  // printer.addInstruction(2, {2.0, 8.0, -3.0});
  // printer.addInstruction(2, {-3.0, 12.0, -3.0});
  for (int i = 5; i < 8; i++) {
    addCircleInstructions(i+3, (float)i, 0.0, 0.0, 9.0 + i/6.0, 1);
    Vector3 prevPoint = printer.getLastInstruction().getPosition();
    // smoothly go to next circle
    if (i != 7) {
      addLinearInstructions(5, prevPoint.x, prevPoint.y, prevPoint.z, i + 1.0, 9.0 + (i+1)/6.0, 0.0,    4);
    } else {
      addLinearInstructions(5, prevPoint.x, prevPoint.y, prevPoint.z, 0.0, 8.0, 0.0,     4);
    }
  }


  // go back to start
  printer.addInstruction(2, {0.0, 8.0, 0.0});
  // reverse, otherwise they will happen in the wrong order
  printer.reverseInstructions();
}

void loop() {
  if (!printer.finished) {
    long currentMillis = millis();
    printer.update(currentMillis);
  }
}
