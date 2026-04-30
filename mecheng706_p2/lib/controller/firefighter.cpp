#include "firefighter.h"
#include "initialising.h"
#include "search.h"
#include "avoid.h"
#include "approach.h"
#include "extinguish.h"
#include "stopped.h"


FireFighter::FireFighter(const HardwareContext& hw) : _hw(hw) {
    // Initialise the controller-side wrappers around the HAL primitives.
    // Each sensor/motor class is HAL-clean and gets its hardware backend
    // via the context.
    _gyro            = new Gyroscope(_hw.gyro_source, _hw.clock);
    _motors          = new driveMotors(_hw.motor_output);
    _front_left_ir   = new LongRangeIR (_hw.analog_input, front_left_ir_pin);
    _front_right_ir  = new LongRangeIR (_hw.analog_input, front_right_ir_pin);
    _rear_left_ir    = new ShortRangeIR(_hw.analog_input, rear_left_ir_pin);
    _rear_right_ir   = new ShortRangeIR(_hw.analog_input, rear_right_ir_pin);
    _ultrasonic      = new Ultrasonic  (_hw.ultrasonic_source, _hw.clock, MAX_DIST);
    _motors->attatchAll();

    _photo_front     = new Phototransistor(_hw.analog_input, photo_front_pin);
    _photo_right     = new Phototransistor(_hw.analog_input, photo_right_pin);
    _photo_rear      = new Phototransistor(_hw.analog_input, photo_rear_pin);
    _photo_left      = new Phototransistor(_hw.analog_input, photo_left_pin);
    _fire_bank       = new FireBank(_photo_front, _photo_right, _photo_rear, _photo_left);

    _fan             = new Fan(_hw.fan_output, _hw.clock);
    _fan->begin();

    // Initialise all states up-front to avoid runtime allocation.
    states_[State::INITIALISING] = new Initialising(this);
    states_[State::SEARCH]       = new Search(this);
    states_[State::AVOID]        = new Avoid(this);
    states_[State::APPROACH]     = new Approach(this);
    states_[State::EXTINGUISH]   = new Extinguish(this);
    states_[State::STOPPED]      = new Stopped(this);

    // Begin initial state AFTER all hardware and states are ready.
    current_state_ = states_[State::INITIALISING];
    current_state_->begin();
}

FireFighter::~FireFighter() {
    for (int i = 0; i < State::NUM_STATES; ++i) {
        delete states_[i];
    }
    delete _fire_bank;
    delete _photo_front; delete _photo_right; delete _photo_rear; delete _photo_left;
    delete _fan;
    delete _ultrasonic;
    delete _front_left_ir; delete _front_right_ir;
    delete _rear_left_ir;  delete _rear_right_ir;
    delete _motors;
    delete _gyro;
}


bool FireFighter::switchState(State::Name newState, StateData data) {
    // 1. End the current state.
    if (current_state_) {
        current_state_->end();
    }

    if (!is_battery_voltage_OK()) {
        // Fall back to initialising on undervoltage.
        current_state_ = states_[State::INITIALISING];
        return false;
    }

    // 2. Look up the new state.
    current_state_ = states_[newState];

    // 3. Begin the new state, with optional data payload.
    if (current_state_) {
        if (data.param != -1.0f) {
            current_state_->begin(data);
            return true;
        } else {
            current_state_->begin();
            return true;
        }
    }

    return false;
}

void FireFighter::pollState() {
    _gyro->readSensor();
    // Refresh the four phototransistor EWMAs every loop so any state's poll()
    // can synchronously query "is there a fire?" without paying the ADC cost
    // itself. Cheap (~4 analogReads) compared with one ultrasonic ping.
    if (_fire_bank) _fire_bank->update();
    if (current_state_) {
        current_state_->poll();
    }
}

void FireFighter::testSensors() {
    print("us med: "); println(_ultrasonic->getAvg());
    print("front left:");
    println(_front_left_ir->readSensor());
    print("front right:");
    println(_front_right_ir->readSensor());
    print("rear left:");
    println(_rear_left_ir->readSensor());
    print("rear right:");
    println(_rear_right_ir->readSensor());
    println();
    println();
}

void FireFighter::timeStepData() {
    if (_hw.clock) print(_hw.clock->now_ms());
    print(",");

    print(_front_left_ir->mapping_reading_);
    print(",");

    print(_front_right_ir->mapping_reading_);
    print(",");

    print(_rear_left_ir->mapping_reading_);
    print(",");

    print(_rear_right_ir->mapping_reading_);
    print(",");

    print(_ultrasonic->mapping_reading_);
    println(";");
}

bool FireFighter::is_battery_voltage_OK() {
    static unsigned char Low_voltage_counter = 0;

    if (!_hw.analog_input) return true;

    // Convert the HAL voltage reading back into an ADC-count equivalent so
    // the original calibrated thresholds (717..877) still apply unchanged.
    // Discharged ~3.5V => raw 717 ; full ~4.25V => raw ~860.
    float voltage = _hw.analog_input->read_voltage(battery_sense_pin);
    int raw_lipo = (int)(voltage * (float)ADC_RANGE / V_ADC);
    int Lipo_level_cal = ((raw_lipo - 717) * 100) / 143;

    if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
        Low_voltage_counter = 0;
        return true;
    } else {
        if (Low_voltage_counter < 255) Low_voltage_counter++;
        return Low_voltage_counter <= 5;
    }
}
