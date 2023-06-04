//
// Created by xiang on 2022/1/4.
//

#ifndef SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
#define SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H

#include "common/gnss.h"

namespace sad {

/**
 * Calculate UTM pose and 6-DOF pose corresponding to GNSS readings in this
 * book.
 * @param gnss_reading Input GNSS reading
 * @param antenna_pos Installation position
 * @param antenna_angle Installation angle
 * @param map_origin Map origin. If specified, subtract the origin coordinates
 * from the UTM position.
 * @return True if successful, False otherwise
 */
bool ConvertGps2UTM(GNSS& gnss_reading, const Vec2d& antenna_pos,
                    const double& antenna_angle,
                    const Vec3d& map_origin = Vec3d::Zero());

/**
 * Convert only the translation part of the latitude and longitude without
 * extrinsic parameters and angle processing.
 * @param gnss_reading
 * @return True if successful, False otherwise
 */
bool ConvertGps2UTMOnlyTrans(GNSS& gnss_reading);

/**
 * Convert latitude and longitude to UTM.
 * NOTE: The latitude and longitude units are in degrees.
 * @param latlon
 * @param utm_coor
 * @return True if successful, False otherwise
 */
bool LatLon2UTM(const Vec2d& latlon, UTMCoordinate& utm_coor);

/**
 * Convert UTM coordinates to latitude and longitude.
 * @param utm_coor
 * @param latlon
 * @return True if successful, False otherwise
 */
bool UTM2LatLon(const UTMCoordinate& utm_coor, Vec2d& latlon);

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_UTM_CONVERT_H
