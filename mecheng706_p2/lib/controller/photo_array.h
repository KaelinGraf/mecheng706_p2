#ifndef PHOTO_ARRAY_H
#define PHOTO_ARRAY_H

#include <stdint.h>
#include "sfh300fa.h"

// Boom-mounted phototransistor array on the panning turret. Holds 2-4
// SFH300FA instances, each with a fixed angular offset from the boom
// centerline (right-positive convention to match the body yaw sign).
//
// On each update() the array refreshes every photodiode's filtered
// reading, then exposes:
//   * max_intensity()     - the brightest single photodiode reading
//                           (volts), used as a "fire seen" gate.
//   * bearing_error()     - intensity-weighted angular centroid of the
//                           array relative to the boom centerline (rad).
//                           Drives the turret's pan-servo P controller.
//   * any_above(thresh)   - true if any photodiode is above threshold.
//
// The bearing error formula is the soft centroid:
//
//     err = sum(V_i * theta_i) / sum(V_i)
//
// where V_i is the photodiode's filtered voltage with the per-array
// ambient (the minimum reading) subtracted, and theta_i is its mounting
// angle. With two symmetric photodiodes at +/- theta this collapses to
// the classic differential-photodiode steering formula.
class PhotoArray {
public:
    static const uint8_t MAX_PHOTOS = 4;

    PhotoArray();

    // Add a photodiode at a known angular offset (rad) from the boom
    // centerline. Up to MAX_PHOTOS may be added; further calls are
    // silently ignored.
    void add(SFH300FA* photo, float angle_rad);

    // Refresh every attached photodiode's filtered reading. Cheap; the
    // turret controller calls this every tick.
    void update();

    // True when at least 2 photodiodes have been added.
    inline bool ready() const { return _count >= 2; }

    inline uint8_t count() const { return _count; }

    float max_intensity() const;
    bool  any_above(float threshold) const;

    // Soft-centroid bearing error in rad relative to the boom centerline.
    // Returns 0 when all photodiodes are at the ambient floor.
    float bearing_error() const;

private:
    SFH300FA* _photos[MAX_PHOTOS];
    float     _angles[MAX_PHOTOS];
    uint8_t   _count;
};

#endif
