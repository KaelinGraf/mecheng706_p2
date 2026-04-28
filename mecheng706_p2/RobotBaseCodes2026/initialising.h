#ifndef INITIALISING_H
#define INITIALISING_H

#include "state.h"

class Initialising : public State {
  public:
    Initialising(FireFighter* firefighter) : State(State::INITIALISING, firefighter), count_(0), last_millis_(0) {};
    ~Initialising() {};

    void begin() override;
    void end() override;
    void poll() override;

  private:
    int count_;
    unsigned long last_millis_;
};


#endif // INITIALISING_H
