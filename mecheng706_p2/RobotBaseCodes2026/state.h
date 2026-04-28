#ifndef STATE_H
#define STATE_H

class FireFighter;
struct StateResult;

// Generic transition payload between states. Add fields as needed for
// specific state transitions (e.g. bearing to a detected fire).
struct StateData {
    float param;     // generic float parameter (e.g. bearing, distance)
    bool flag1;
    bool flag2;
};

class State
{
public:
    enum Name
    {
        INITIALISING,
        SEARCH,        // wander/explore looking for fire
        AVOID,         // avoid an obstacle / wall in the path
        APPROACH,      // move into the 20cm extinguishing radius of a detected fire
        EXTINGUISH,    // run fan up to 10s to put out the fire
        STOPPED,

        // leave last, gives access to length of states
        NUM_STATES,
    };

    State(Name name, FireFighter *firefighter) : name_(name), firefighter_(firefighter) {};
    virtual ~State() {};

    inline Name getState() const { return name_; }

    virtual void begin() = 0;
    virtual void begin(StateData data) {};
    virtual void end() = 0;
    virtual void poll() {};

protected:
    Name name_;
    FireFighter *firefighter_;
};

#endif // STATE_H
