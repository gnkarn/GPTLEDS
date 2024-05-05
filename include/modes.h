  // Auto Pilot modes
    // ----------------

#ifndef MODES_H
#define MODES_H

enum Number : int {
  MANUAL = 0,
  CIRCLE = 1,
  STABILIZE = 2,
  TRAINING = 3,
  ACRO = 4,
  FLY_BY_WIRE_A = 5,
  FLY_BY_WIRE_B = 6,
  CRUISE = 7,
  AUTOTUNE = 8,
  AUTO = 10,
  RTL = 11,
  LOITER = 12,
  TAKEOFF = 13,
  AVOID_ADSB = 14,
  GUIDED = 15,
  INITIALISING = 16,
#if HAL_QUADPLANE_ENABLED
  QSTABILIZE = 17,
  QHOVER = 18,
  QLOITER = 19,
  QLAND = 20,
  QRTL = 21,
#if QAUTOTUNE_ENABLED
  QAUTOTUNE = 22,
#endif
  QACRO = 23,
#endif
  THERMAL = 24,
#if HAL_QUADPLANE_ENABLED
  LOITER_ALT_QLAND = 25,
#endif
  };

#endif // MODES_H