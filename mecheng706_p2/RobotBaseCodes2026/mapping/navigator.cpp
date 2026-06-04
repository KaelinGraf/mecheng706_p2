#include "navigator.hpp"

#include <math.h>

namespace {
float clampf(float value, float lo, float hi) {
	if (value < lo) return lo;
	if (value > hi) return hi;
	return value;
}

float wrapAngle(float angle) {
	while (angle > PI) angle -= TWO_PI;
	while (angle < -PI) angle += TWO_PI;
	return angle;
}

bool cellOccupiedAhead(const OccupancyGrid* grid, const Pose2D& robotPose,
					   float forward_cm, float lateral_cm) {
	Pose2D probe = robotPose.chain(Pose2D(forward_cm, lateral_cm, 0.0f));
	OccupancyGrid::idx_xy cell = grid->worldToIndex(probe.x, probe.y);
	if (cell.x < 0 || cell.x >= WIN_N || cell.y < 0 || cell.y >= WIN_N) {
		return true;
	}
	return grid->isCellOccupied(cell.x, cell.y);
}
}

void Navigator::navigate(Pose2D targetPose) {
	(void)targetPose;
}

Pose2D Navigator::suggestCommand() const {
	Pose2D command(0.0f, 0.0f, 0.0f);
	if (!_dead_reckoner || !_local_grid || !_fire_localiser) {
		return command;
	}

	const Pose2D robotPose = _dead_reckoner->getPose();
	const Pose2D firePose = _fire_localiser->solvePose();
	const float dx = firePose.x - robotPose.x;
	const float dy = firePose.y - robotPose.y;
	const float distance = sqrtf(dx * dx + dy * dy);
	const float targetHeading = atan2f(dy, dx);
	const float headingError = wrapAngle(targetHeading - robotPose.th);

	if (_closeToFire) {
		command.x = APPROACH_FORWARD_SPEED;
		command.y = 0.0f;
		command.th = clampf(APPROACH_TURN_GAIN * headingError, -APPROACH_MAX_TURN, APPROACH_MAX_TURN);
		if (distance < EXTINGUISH_RANGE_CM) {
			command.x = 0.0f;
		}
		return command;
	}

	const bool frontBlocked = cellOccupiedAhead(_local_grid, robotPose, 15.0f, 0.0f);
	const bool leftBlocked  = cellOccupiedAhead(_local_grid, robotPose, 12.0f, 10.0f)
						   || cellOccupiedAhead(_local_grid, robotPose, 0.0f, 10.0f);
	const bool rightBlocked = cellOccupiedAhead(_local_grid, robotPose, 12.0f, -10.0f)
						   || cellOccupiedAhead(_local_grid, robotPose, 0.0f, -10.0f);

	command.x = SEARCH_SPEED;
	command.y = 0.0f;
	command.th = clampf(SEARCH_TURN_SPEED * headingError, -AVOID_ROTATE_SPEED, AVOID_ROTATE_SPEED);

	if (frontBlocked) {
		if (!leftBlocked && rightBlocked) {
			command.th = AVOID_ROTATE_SPEED;
			command.x = AVOID_SPEED;
		} else if (!rightBlocked && leftBlocked) {
			command.th = -AVOID_ROTATE_SPEED;
			command.x = AVOID_SPEED;
		} else if (!leftBlocked && !rightBlocked) {
			command.th = (headingError >= 0.0f) ? AVOID_ROTATE_SPEED : -AVOID_ROTATE_SPEED;
			command.x = AVOID_SPEED * 0.8f;
		} else {
			command.th = AVOID_ROTATE_SPEED;
			command.x = -AVOID_SPEED;
		}
	} else {
		if (leftBlocked && !rightBlocked) {
			command.th -= AVOID_ROTATE_SPEED * 0.5f;
		} else if (rightBlocked && !leftBlocked) {
			command.th += AVOID_ROTATE_SPEED * 0.5f;
		}
	}

	return command;
}
