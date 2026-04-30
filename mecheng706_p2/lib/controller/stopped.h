#ifndef STOPPED_H
#define STOPPED_H

#include "state.h"

class Stopped : public State {
  public:
    Stopped(FireFighter* firefighter) : State(State::STOPPED, firefighter), last_millis_(0) {};
    ~Stopped() {};

    void begin() override;
    void end() override;
    void poll() override;

  private:
    unsigned long last_millis_;
};


#endif // STOPPED_H
