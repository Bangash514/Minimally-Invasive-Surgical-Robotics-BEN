"""
Lab 16: System Architecture — State Machine
Volume 1, Chapter 16

What this lab does:
- Clinical workflow state machine
- Valid/invalid transition testing
- Safety manager simulation
- Integration with earlier modules

Author: BEN (班班)
Date: May 2026
"""

from enum import Enum


class SystemState(Enum):
    STARTUP          = "startup"
    CALIBRATION      = "calibration"
    PATIENT_SETUP    = "patient_setup"
    MRI_LOADING      = "mri_loading"
    PROBE_INSERTION  = "probe_insertion"
    REGISTRATION     = "registration"
    BIOPSY_PLANNING  = "biopsy_planning"
    BIOPSY_EXECUTION = "biopsy_execution"
    NEEDLE_RETRACT   = "needle_retract"
    PROCEDURE_END    = "procedure_end"
    EMERGENCY_STOP   = "emergency_stop"


class ClinicalStateMachine:
    VALID = {
        SystemState.STARTUP:
            [SystemState.CALIBRATION],
        SystemState.CALIBRATION:
            [SystemState.PATIENT_SETUP],
        SystemState.PATIENT_SETUP:
            [SystemState.MRI_LOADING],
        SystemState.MRI_LOADING:
            [SystemState.PROBE_INSERTION],
        SystemState.PROBE_INSERTION:
            [SystemState.REGISTRATION],
        SystemState.REGISTRATION:
            [SystemState.BIOPSY_PLANNING],
        SystemState.BIOPSY_PLANNING:
            [SystemState.BIOPSY_EXECUTION],
        SystemState.BIOPSY_EXECUTION:
            [SystemState.NEEDLE_RETRACT],
        SystemState.NEEDLE_RETRACT:
            [SystemState.BIOPSY_PLANNING,
             SystemState.PROCEDURE_END],
    }

    def __init__(self):
        self.state   = SystemState.STARTUP
        self.history = []

    def transition(self, new_state):
        if new_state == SystemState.EMERGENCY_STOP:
            self.history.append(self.state)
            self.state = new_state
            return True, "Emergency stop activated"

        valid = self.VALID.get(self.state, [])
        if new_state not in valid:
            return False, (f"Invalid: "
                           f"{self.state.value} → "
                           f"{new_state.value}")

        self.history.append(self.state)
        self.state = new_state
        return True, f"→ {new_state.value}"


# Test state machine
print("=" * 55)
print("Lab 16: Clinical State Machine")
print("=" * 55)

sm = ClinicalStateMachine()
print(f"\nInitial state: {sm.state.value}")

# Valid workflow
print("\nValid clinical workflow:")
workflow = [
    SystemState.CALIBRATION,
    SystemState.PATIENT_SETUP,
    SystemState.MRI_LOADING,
    SystemState.PROBE_INSERTION,
    SystemState.REGISTRATION,
    SystemState.BIOPSY_PLANNING,
    SystemState.BIOPSY_EXECUTION,
    SystemState.NEEDLE_RETRACT,
    SystemState.BIOPSY_PLANNING,
    SystemState.BIOPSY_EXECUTION,
    SystemState.NEEDLE_RETRACT,
    SystemState.PROCEDURE_END,
]

for next_state in workflow:
    ok, msg = sm.transition(next_state)
    status = "✓" if ok else "✗"
    print(f"  {status} {msg}")

# Invalid transition test
print("\nInvalid transition test:")
sm2 = ClinicalStateMachine()
ok, msg = sm2.transition(
    SystemState.BIOPSY_EXECUTION
)
print(f"  STARTUP → BIOPSY_EXECUTION: "
      f"{'✓' if ok else '✗'} ({msg})")

# Emergency stop from any state
print("\nEmergency stop test:")
sm3 = ClinicalStateMachine()
sm3.transition(SystemState.CALIBRATION)
sm3.transition(SystemState.PATIENT_SETUP)
ok, msg = sm3.transition(SystemState.EMERGENCY_STOP)
print(f"  From {sm3.history[-1].value}: "
      f"{'✓' if ok else '✗'} ({msg})")
print(f"  Current state: {sm3.state.value}")
