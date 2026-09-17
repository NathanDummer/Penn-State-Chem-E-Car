/*
  Hexapod Tripod Gait Walk Cycle
  ------------------------------
  Six-legged walker (AT-TE style), 2 servos per leg:
    - "hip"  servo swings the leg forward/backward
    - "knee" servo lifts the leg up/down

  Uses a TRIPOD GAIT: legs are split into two groups of three.
  While one group is lifted and swinging forward (recovery stroke),
  the other group stays planted and pushes the body forward
  (stance/power stroke). The groups then swap.

  Leg layout (viewed from above):

        FRONT
     LF        RF
      \        /
   LM--+------+--RM
      /        \
     LB        RB
        BACK

  Group A = LF, RM, LB   (diagonal-ish trio #1)
  Group B = RF, LM, RB   (diagonal-ish trio #2)

  The walk function is NON-BLOCKING (uses millis(), not delay()),
  so you can call it every loop() iteration and still read sensors,
  check for stop commands, etc. in the same loop.
*/

#include <Servo.h>

// ---------------------------------------------------------------
// CONFIGURATION - edit these to match your robot
// ---------------------------------------------------------------

const uint8_t NUM_LEGS = 6;

// Leg index order: 0=LF, 1=LM, 2=LB, 3=RF, 4=RM, 5=RB
enum LegIndex { LF = 0, LM = 1, LB = 2, RF = 3, RM = 4, RB = 5 };

// --- Servo pins: EDIT to match your wiring ---
const uint8_t hipPin[NUM_LEGS]  = { 2,  3,  4,  5,  6,  7  }; // swing servos
const uint8_t kneePin[NUM_LEGS] = { 8,  9, 10, 11, 12, 13  }; // lift servos

// Left-side legs are usually mounted mirrored to right-side legs,
// so a "swing forward" command needs the opposite angle direction.
// Set true for legs whose hip servo is mounted mirrored.
const bool mirrorHip[NUM_LEGS] = { true, true, true, false, false, false };

// --- Tuning angles (degrees). Adjust for your servo horns/linkages ---
const int HIP_NEUTRAL  = 90;   // leg pointing straight out from body
const int HIP_SWING    = 25;   // how far forward/back the hip swings (+/-)
const int KNEE_DOWN    = 90;   // leg planted on ground
const int KNEE_UP      = 55;   // leg lifted for clearance (smaller = higher, tune this)

// --- Timing ---
unsigned int phaseDuration = 200; // ms per phase; smaller = faster gait

// ---------------------------------------------------------------
// INTERNAL STATE
// ---------------------------------------------------------------

Servo hipServo[NUM_LEGS];
Servo kneeServo[NUM_LEGS];

const uint8_t groupA[3] = { LF, RM, LB };
const uint8_t groupB[3] = { RF, LM, RB };

// Walk cycle has 4 phases:
// 0: Group A lifts + swings forward   | Group B stays down, swings backward (pushes body)
// 1: Group A lowers (plants down)     | Group B continues stance
// 2: Group B lifts + swings forward   | Group A stays down, swings backward (pushes body)
// 3: Group B lowers (plants down)     | Group A continues stance
uint8_t walkPhase = 0;
unsigned long phaseStartTime = 0;
bool walkingEnabled = false;

// ---------------------------------------------------------------
// LOW-LEVEL HELPERS
// ---------------------------------------------------------------

// Write a hip angle, accounting for mirrored legs.
// direction: +1 = swing forward, -1 = swing backward, 0 = neutral
void setHip(uint8_t leg, int direction) {
  int angle = HIP_NEUTRAL + direction * HIP_SWING * (mirrorHip[leg] ? -1 : 1);
  hipServo[leg].write(angle);
}

void setKnee(uint8_t leg, bool up) {
  kneeServo[leg].write(up ? KNEE_UP : KNEE_DOWN);
}

// ---------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------

void setup() {
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    hipServo[i].attach(hipPin[i]);
    kneeServo[i].attach(kneePin[i]);
  }

  // Start in a neutral standing stance
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    setHip(i, 0);
    setKnee(i, false); // all feet down
  }

  delay(500); // let servos settle before moving

  phaseStartTime = millis();
  walkingEnabled = true; // set false to freeze in place
}

// ---------------------------------------------------------------
// WALK CYCLE FUNCTION - call this every loop()
// ---------------------------------------------------------------

void updateWalk() {
  if (!walkingEnabled) return;

  unsigned long now = millis();
  if (now - phaseStartTime < phaseDuration) return; // not time to advance yet

  phaseStartTime = now;

  switch (walkPhase) {

    case 0:
      // Group A: lift + swing forward (recovery)
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupA[i], true);
        setHip(groupA[i], +1);
      }
      // Group B: stays planted, swings backward (drives body forward)
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupB[i], false);
        setHip(groupB[i], -1);
      }
      break;

    case 1:
      // Group A: plant down at forward position
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupA[i], false);
      }
      break;

    case 2:
      // Group B: lift + swing forward (recovery)
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupB[i], true);
        setHip(groupB[i], +1);
      }
      // Group A: stays planted, swings backward (drives body forward)
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupA[i], false);
        setHip(groupA[i], -1);
      }
      break;

    case 3:
      // Group B: plant down at forward position
      for (uint8_t i = 0; i < 3; i++) {
        setKnee(groupB[i], false);
      }
      break;
  }

  walkPhase = (walkPhase + 1) % 4;
}

// ---------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------

void loop() {
  updateWalk();

  // Add other non-blocking logic here (sensors, comms, obstacle
  // detection, etc.) — updateWalk() never calls delay(), so this
  // loop stays responsive.
}
