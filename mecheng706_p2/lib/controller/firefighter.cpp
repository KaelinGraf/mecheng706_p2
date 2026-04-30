#include "firefighter.h"
#include "world_view.h"


FireFighter::FireFighter(const HardwareContext& hw) : _hw(hw) {
    // Sensors and motion layer.
    _gyro            = new Gyroscope(_hw.gyro_source, _hw.clock);
    _imu             = new IMU(_hw.imu_source, _hw.clock);
    _motors          = new driveMotors(_hw.motor_output);
    _front_left_ir   = new LongRangeIR (_hw.analog_input, front_left_ir_pin);
    _front_right_ir  = new LongRangeIR (_hw.analog_input, front_right_ir_pin);
    _rear_left_ir    = new ShortRangeIR(_hw.analog_input, rear_left_ir_pin);
    _rear_right_ir   = new ShortRangeIR(_hw.analog_input, rear_right_ir_pin);
    _ultrasonic      = new Ultrasonic  (_hw.ultrasonic_source, _hw.clock, MAX_DIST);
    _motors->attatchAll();
    _motion          = new MotionController(_motors, _hw.clock, _imu);

    _fan             = new Fan(_hw.fan_output, _hw.clock);
    _fan->begin();

    // Turret subsystem: panning boom carrying the fan + IR photodiode array.
    _turret_photo_left  = new SFH300FA(_hw.analog_input, turret_photo_left_pin);
    _turret_photo_right = new SFH300FA(_hw.analog_input, turret_photo_right_pin);
    _turret_photos      = new PhotoArray();
    _turret_photos->add(_turret_photo_left,  TURRET_PHOTO_LEFT_RAD);
    _turret_photos->add(_turret_photo_right, TURRET_PHOTO_RIGHT_RAD);
    _pan_servo = new PanServo(_hw.motor_output,
                              TURRET_SERVO_CHANNEL,
                              turret_servo_pin,
                              TURRET_PAN_MIN_RAD,
                              TURRET_PAN_MAX_RAD);
    _turret = new TurretController(_pan_servo, _turret_photos, _imu, _hw.clock);
    _turret->start();

    // Behavior arbiter + 5 layers, top priority first.
    _b_mission_complete = new MissionCompleteBehavior();
    _b_avoid_obstacle   = new AvoidObstacleBehavior();
    _b_extinguish_fire  = new ExtinguishFireBehavior();
    _b_move_to_fire     = new MoveToFireBehavior();
    _b_find_fire        = new FindFireBehavior();
    _arbiter            = new BehaviorArbiter();
    _arbiter->install(0, _b_mission_complete);
    _arbiter->install(1, _b_avoid_obstacle);
    _arbiter->install(2, _b_extinguish_fire);
    _arbiter->install(3, _b_move_to_fire);
    _arbiter->install(4, _b_find_fire);
}

FireFighter::~FireFighter() {
    delete _arbiter;
    delete _b_mission_complete;
    delete _b_avoid_obstacle;
    delete _b_extinguish_fire;
    delete _b_move_to_fire;
    delete _b_find_fire;
    delete _turret;
    delete _pan_servo;
    delete _turret_photos;
    delete _turret_photo_left; delete _turret_photo_right;
    delete _fan;
    delete _ultrasonic;
    delete _front_left_ir; delete _front_right_ir;
    delete _rear_left_ir;  delete _rear_right_ir;
    delete _motion;
    delete _motors;
    delete _imu;
    delete _gyro;
}

void FireFighter::tick() {
    if (_imu)  _imu->update();      // drain BNO events, refresh yaw/omega/accel
    _gyro->readSensor();             // legacy omega integrator (still useful)
    if (_turret) _turret->tick();

    // Build a single WorldView snapshot for this iteration.
    WorldView w;
    w.us_cm          = _ultrasonic     ? _ultrasonic->getAvg()     : -1.0f;
    w.front_left_ir  = _front_left_ir  ? _front_left_ir->getAvg()  : -1.0f;
    w.front_right_ir = _front_right_ir ? _front_right_ir->getAvg() : -1.0f;
    w.rear_left_ir   = _rear_left_ir   ? _rear_left_ir->getAvg()   : -1.0f;
    w.rear_right_ir  = _rear_right_ir  ? _rear_right_ir->getAvg()  : -1.0f;
    if (_turret) w.turret = _turret->snapshot();
    w.body_yaw_rad   = _imu ? _imu->yaw()     : 0.0f;
    w.body_omega_z   = _imu ? _imu->omega_z() : 0.0f;
    w.fires_extinguished = fires_extinguished_;
    w.now_ms         = _hw.clock ? _hw.clock->now_ms() : 0UL;

    const BehaviorOutput out = _arbiter->tick(w);

    // Apply winning output to actuators.
    if (_turret) _turret->set_hold(out.request_turret_hold);
    if (_fan) {
        if (out.fan_on) _fan->on();
        else            _fan->off();
    }
    if (out.fires_extinguished_increment) noteFireExtinguished();
    if (_motion) _motion->apply(out.motion);
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
    print(_front_left_ir->mapping_reading_); print(",");
    print(_front_right_ir->mapping_reading_); print(",");
    print(_rear_left_ir->mapping_reading_); print(",");
    print(_rear_right_ir->mapping_reading_); print(",");
    print(_ultrasonic->mapping_reading_); println(";");
}

bool FireFighter::is_battery_voltage_OK() {
    static unsigned char Low_voltage_counter = 0;
    if (!_hw.analog_input) return true;
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
