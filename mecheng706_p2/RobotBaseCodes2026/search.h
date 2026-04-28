#ifndef SEARCH_H
#define SEARCH_H

#include "state.h"
#include "pid.h"

// SEARCH: wander the table looking for fires (phototransistor signal).
// Transition to APPROACH on fire detection, AVOID on obstacle detection.
class Search : public State {
  public:
    Search(FireFighter* firefighter) : State(State::SEARCH, firefighter) {};
    ~Search() {};

    void begin() override;
    void end() override;
    void poll() override;
};


#endif // SEARCH_H
