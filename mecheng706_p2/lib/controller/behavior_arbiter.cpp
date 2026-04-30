#include "behavior_arbiter.h"

BehaviorArbiter::BehaviorArbiter() : _last_winner(nullptr) {
    for (uint8_t i = 0; i < MAX_LAYERS; ++i) _layers[i] = nullptr;
}

void BehaviorArbiter::install(uint8_t idx, IBehavior* behavior) {
    if (idx >= MAX_LAYERS) return;
    _layers[idx] = behavior;
}

BehaviorOutput BehaviorArbiter::tick(const WorldView& w) {
    for (uint8_t i = 0; i < MAX_LAYERS; ++i) {
        IBehavior* b = _layers[i];
        if (!b) continue;
        BehaviorOutput out = b->tick(w);
        if (out.active) {
            if (b != _last_winner) {
                if (_last_winner) _last_winner->on_deselect();
                b->on_select();
                _last_winner = b;
            }
            return out;
        }
    }
    // Everyone abstained: deselect any prior winner and return idle.
    if (_last_winner) {
        _last_winner->on_deselect();
        _last_winner = nullptr;
    }
    return BehaviorOutput();
}
