#ifndef SEARCH_H
#define SEARCH_H

#include "state.h"
#include "tracking.h"

// SEARCH owns the behaviour controller that blends find-fire, avoid, and
// move-towards-fire behaviours without leaving the SEARCH state.
class Search : public State {
  public:
    Search(FireFighter* firefighter) : State(State::SEARCH, firefighter), tracking_(firefighter) {};
    ~Search() {};

    void begin() override;
    void end() override;
    void poll() override;

  private:
    Tracking tracking_;
};


#endif // SEARCH_H
