// Behavior interface and state/behavior enums
#ifndef BEHAVIOR_H
#define BEHAVIOR_H

class FireFighter; // forward declaration (defined elsewhere)

namespace BehaviorNS {

// High-level robot states (behaviour-based control)
enum class StateName : uint8_t {
    INITIALISE,
    SEARCH,
    EXTINGUISH,
    STOPPED,
};

// Sub-behaviours used while in SEARCH
enum class SearchBehaviour : uint8_t {
    FIND_FIRE,
    AVOID,
    MOVE_TO_FIRE,
};

// Generic behaviour interface
class Behaviour {
public:
    virtual ~Behaviour() {}

    // Called once when the behaviour becomes relevant/started
    virtual void begin(FireFighter &ff) = 0;

    // Called repeatedly while the behaviour is active
    virtual void update(FireFighter &ff) = 0;

    // Whether the behaviour currently wants control / is active
    virtual bool active() const = 0;

    // Human-readable name for debugging
    virtual const char* name() const = 0;
};

// Simple manager interface to run behaviours for a given high-level state.
// Implementations should select and run one or more Behaviour instances
// appropriate for the current `StateName`.
class BehaviourManager {
public:
    virtual ~BehaviourManager() {}

    // Called when the robot enters a new high-level state
    virtual void onStateEnter(StateName state, FireFighter &ff) = 0;

    // Called every loop to let the manager update active behaviours
    virtual void update(StateName state, FireFighter &ff) = 0;
};

} // namespace BehaviorNS

#endif // BEHAVIOR_H
