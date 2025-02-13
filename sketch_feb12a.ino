#include <BasicLinearAlgebra.h>
using namespace BLA;

// All inaccurate, need to measure
#define SLIDER_MAX_HEIGHT 24.0
#define BUILD_PLATE_RADIUS 10
#define ARM_LENGTH 24
#define SLIDER_DISTANCE_FROM_CENTER 12

#define MAX_INSTRUCTION_COUNT 50

typedef BLA::Matrix<3,1,float> Vector3;

class Stepper {
private:
  int rotation = 0;

public:
  Stepper() {};
  void rotate(float degrees) {
    // TODO: rotate the actual stepper motor

  }
  int getRotation() {
    return rotation;
  }
};

class PositionedObject {
private:
  Vector3 position;
public:
  float getX() { return position(0); }
  float getY() { return position(1); }
  float getZ() { return position(2); }
  float setX(float x) { position(0) = x; }
  float setY(float y) { position(1) = y; }
  float setZ(float z) { position(2) = z; }

  Vector3 getPosition() { return position; }
  void setPosition(Vector3 p) { position = p; }
};


class Slider : PositionedObject {
private:
  Stepper motor = Stepper();

public:
  void move(float distance, float speed) {
    setY(getY() + distance);
    motor.rotate(10.0); // TODO: move "distance" at "speed"
  }

  float findHeightChange(PositionedObject* head) {
    // This equation is found by rearranging the distance equation to solve for the height of the slider
    float deltaX = head->getX() - getX();
    float deltaZ = head->getZ() - getZ();
    float squareRooted = pow(ARM_LENGTH, 2) - pow(deltaX, 2) - pow(deltaZ, 2);
    // TODO: what do I return when this function encounters an error? Can't use throw, and the function might return -1 or 0
    // Could use a large value such as 1000, which will never be returned ordinarily
    if (squareRooted < 0) return what;
    float height = head->getY() + sqrt(squareRooted);
    if (height < 0 || height > SLIDER_MAX_HEIGHT) return what;
    return height - getY();
  }
  
  Slider(float angle, PositionedObject* head) {
    // First set y position to 0 so that it can be used in "findHeightChange"
    // TODO: Should cos and sin by switched?
    setPosition({SLIDER_DISTANCE_FROM_CENTER * sin(angle), 0.0, SLIDER_DISTANCE_FROM_CENTER * cos(angle)});
    setY(findHeightChange(head));
  }

};

class PrintHead : public PositionedObject {
public:
  PrintHead() {
    setPosition({0.0, 0.0, 0.0});
  }
  void move(Vector3 delta, float speed, Slider slider1, Slider slider2, Slider slider3) {
    setPosition(getPosition() + delta);
    slider1.move(slider1.findHeightChange(this), speed);
    slider2.move(slider2.findHeightChange(this), speed);
    slider3.move(slider3.findHeightChange(this), speed);
  }
};


class Instruction : public PositionedObject {
private:
  float speed;

public:
  Instruction() : speed(0.0) {
    setPosition({0, 0, 0});
  }
  Instruction(float s, Vector3 p) : speed(s) {
    setPosition(p);
  }

  float getSpeed() {
    return speed;
  }
};

class Printer {
private:
  PrintHead head = PrintHead();
  Slider slider1 = Slider(0*TWO_PI/3, &head);
  Slider slider2 = Slider(1*TWO_PI/3, &head);
  Slider slider3 = Slider(2*TWO_PI/3, &head);
  Instruction instructions[MAX_INSTRUCTION_COUNT];
  int instructionCount = 0;
public:
  Printer() {}
  void executeAllInstructions() {
    while (instructionCount > 0) {
      executeNextInstruction();
    }
  }
  void executeNextInstruction() {
    Instruction instruction = instructions[instructionCount - 1];
    head.move(instruction.getPosition() - head.getPosition(), instruction.getSpeed(), slider1, slider2, slider3);
    instructionCount--;
  }

  void addInstruction(float speed, Vector3 endPosition) {
    instructions[instructionCount] = Instruction(speed, endPosition);
    instructionCount++;
  }
};

void setup() {
  Printer printer = Printer();
  printer.addInstruction(2.0, {5.0, 5.0, 5.0});
  printer.executeAllInstructions();
}

void loop() {
  // put your main code here, to run repeatedly:

}
