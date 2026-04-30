#include "photo_array.h"

PhotoArray::PhotoArray() : _count(0) {
    for (uint8_t i = 0; i < MAX_PHOTOS; ++i) {
        _photos[i] = nullptr;
        _angles[i] = 0.0f;
    }
}

void PhotoArray::add(SFH300FA* photo, float angle_rad) {
    if (_count >= MAX_PHOTOS || !photo) return;
    _photos[_count] = photo;
    _angles[_count] = angle_rad;
    ++_count;
}

void PhotoArray::update() {
    for (uint8_t i = 0; i < _count; ++i) {
        if (_photos[i]) _photos[i]->readSensor();
    }
}

float PhotoArray::max_intensity() const {
    float vmax = 0.0f;
    for (uint8_t i = 0; i < _count; ++i) {
        if (!_photos[i]) continue;
        float v = _photos[i]->getFilteredV();
        if (v > vmax) vmax = v;
    }
    return vmax;
}

bool PhotoArray::any_above(float threshold) const {
    for (uint8_t i = 0; i < _count; ++i) {
        if (_photos[i] && _photos[i]->getFilteredV() >= threshold) return true;
    }
    return false;
}

float PhotoArray::bearing_error() const {
    if (_count < 2) return 0.0f;

    // Subtract per-array ambient (minimum filtered reading) so an evenly
    // lit scene gives zero error rather than a bias toward whatever side
    // happens to have a higher noise floor.
    float ambient = _photos[0] ? _photos[0]->getFilteredV() : 0.0f;
    for (uint8_t i = 1; i < _count; ++i) {
        if (!_photos[i]) continue;
        float v = _photos[i]->getFilteredV();
        if (v < ambient) ambient = v;
    }

    float num = 0.0f;
    float den = 0.0f;
    for (uint8_t i = 0; i < _count; ++i) {
        if (!_photos[i]) continue;
        float w = _photos[i]->getFilteredV() - ambient;
        if (w < 0.0f) w = 0.0f;
        num += w * _angles[i];
        den += w;
    }
    if (den < 1e-6f) return 0.0f;
    return num / den;
}
