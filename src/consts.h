#ifndef CONSTS_H
#define CONSTS_H

#include <cinttypes>

/**
 * max allowed velocity, in MPH
 */
constexpr double MAX_VELOCITY = 49.5;

/*
 * lane width, in meters
 */
constexpr double LANE_WIDTH = 4.0;

/**
 * safe distance for keeping lane, in meters
 */
constexpr double SAFE_DIST = 30.0;

/**
 * distance indicating too close, in meters. this is determined by doing some tries.
 */
constexpr double DIST_TOO_CLOSE = 10.0;


// sensor fusion data indices.
 constexpr std::size_t SENSOR_FUSION_ID = 0;
constexpr std::size_t SENSOR_FUSION_X  = 1;
constexpr std::size_t SENSOR_FUSION_Y  = 2;
constexpr std::size_t SENSOR_FUSION_VX = 3;
constexpr std::size_t SENSOR_FUSION_VY = 4;
constexpr std::size_t SENSOR_FUSION_S  = 5;
constexpr std::size_t SENSOR_FUSION_D  = 6;

#endif // !CONSTS_H
