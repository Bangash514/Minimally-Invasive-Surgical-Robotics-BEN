// BEN 2026-11-01
// Change aim: C++ version of lab 16: clinical workflow state machine with an emergency stop.
// File: state_machine.cpp
#include <iostream>
#include <map>
#include <string>
#include <vector>

enum class State {
    Startup, Calibration, PatientSetup, MriLoading, ProbeInsertion, Registration,
    BiopsyPlanning, BiopsyExecution, NeedleRetract, ProcedureEnd, EmergencyStop
};

// BEN 2026-11-01
// Change aim: Readable name of a state.
// File: state_machine.cpp
// Function: name
static std::string name(State s)
{
    switch (s) {
        case State::Startup: return "startup";
        case State::Calibration: return "calibration";
        case State::PatientSetup: return "patient_setup";
        case State::MriLoading: return "mri_loading";
        case State::ProbeInsertion: return "probe_insertion";
        case State::Registration: return "registration";
        case State::BiopsyPlanning: return "biopsy_planning";
        case State::BiopsyExecution: return "biopsy_execution";
        case State::NeedleRetract: return "needle_retract";
        case State::ProcedureEnd: return "procedure_end";
        case State::EmergencyStop: return "emergency_stop";
    }
    return "?";
}

// BEN 2026-11-01
// Change aim: State machine that only allows the planned workflow; emergency stop works from any state.
// File: state_machine.cpp
// Function: ClinicalStateMachine
class ClinicalStateMachine {
public:
    ClinicalStateMachine()
    {
        valid_[State::Startup] = {State::Calibration};
        valid_[State::Calibration] = {State::PatientSetup};
        valid_[State::PatientSetup] = {State::MriLoading};
        valid_[State::MriLoading] = {State::ProbeInsertion};
        valid_[State::ProbeInsertion] = {State::Registration};
        valid_[State::Registration] = {State::BiopsyPlanning};
        valid_[State::BiopsyPlanning] = {State::BiopsyExecution};
        valid_[State::BiopsyExecution] = {State::NeedleRetract};
        valid_[State::NeedleRetract] = {State::BiopsyPlanning, State::ProcedureEnd};
    }

    // BEN 2026-11-01
    // Change aim: Try a transition; return true and a message if it is allowed.
    // File: state_machine.cpp
    // Function: ClinicalStateMachine::transition
    bool transition(State next, std::string& message)
    {
        if (next == State::EmergencyStop) {
            history_.push_back(state_);
            state_ = next;
            message = "Emergency stop activated";
            return true;
        }
        const std::vector<State>& allowed = valid_[state_];
        for (State s : allowed)
            if (s == next) {
                history_.push_back(state_);
                state_ = next;
                message = "-> " + name(next);
                return true;
            }
        message = "Invalid: " + name(state_) + " -> " + name(next);
        return false;
    }
    State state() const { return state_; }
    State lastState() const { return history_.back(); }

private:
    State state_ = State::Startup;
    std::vector<State> history_;
    std::map<State, std::vector<State>> valid_;
};

// BEN 2026-11-01
// Change aim: Run a full valid workflow, an invalid jump and an emergency stop.
// File: state_machine.cpp
// Function: main
int main()
{
    std::string msg;
    ClinicalStateMachine sm;
    std::cout << "Initial state: " << name(sm.state()) << "\n\nValid clinical workflow:\n";
    const std::vector<State> workflow = {
        State::Calibration, State::PatientSetup, State::MriLoading, State::ProbeInsertion,
        State::Registration, State::BiopsyPlanning, State::BiopsyExecution, State::NeedleRetract,
        State::BiopsyPlanning, State::BiopsyExecution, State::NeedleRetract, State::ProcedureEnd};
    int accepted = 0;
    for (State s : workflow) {
        const bool ok = sm.transition(s, msg);
        accepted += ok ? 1 : 0;
        std::cout << "  " << (ok ? "OK  " : "FAIL") << " " << msg << "\n";
    }
    std::cout << "  accepted " << accepted << " of " << workflow.size() << " transitions\n";

    ClinicalStateMachine sm2;
    const bool ok2 = sm2.transition(State::BiopsyExecution, msg);
    std::cout << "\nInvalid transition test: startup -> biopsy_execution: " << (ok2 ? "OK" : "FAIL")
              << " (" << msg << ")\n";

    ClinicalStateMachine sm3;
    sm3.transition(State::Calibration, msg);
    sm3.transition(State::PatientSetup, msg);
    const bool ok3 = sm3.transition(State::EmergencyStop, msg);
    std::cout << "\nEmergency stop test: from " << name(sm3.lastState()) << ": " << (ok3 ? "OK" : "FAIL")
              << " (" << msg << ")\nCurrent state: " << name(sm3.state()) << "\n";
    return 0;
}
