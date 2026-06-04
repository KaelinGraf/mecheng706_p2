#include "Arduino.h"
#include "search.h"
#include "firefighter.h"

void Search::begin() {
    tracking_.begin();
}

void Search::end() {
    tracking_.end();
}

void Search::poll() {
    
    tracking_.poll();
}
