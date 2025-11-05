// Copyright (c) 2012, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Author: Chad Rockey
 */

#ifndef DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCAN_HPP_
#define DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCAN_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "depthimage_to_laserscan/DepthImageToLaserScan_export.h"
#include "depthimage_to_laserscan/depth_traits.hpp"
#if __has_include("image_geometry/pinhole_camera_model.hpp")
#include "image_geometry/pinhole_camera_model.hpp"
#else
// This header was deprecated as of https://github.com/ros-perception/vision_opencv/pull/448
// (for Iron), and will be completely removed for J-Turtle.  However, we still need it in
// Humble, since the .hpp doesn't exist there.
#include "image_geometry/pinhole_camera_model.h"
#endif
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace depthimage_to_laserscan
{
class DEPTHIMAGETOLASERSCAN_EXPORT DepthImageToLaserScan final
{
public:
  /**
   * Constructor.
   *
   * @param scan_time The value to use for outgoing sensor_msgs::msg::LaserScan.  In sensor_msgs::msg::LaserScan,
   *                  scan_time is defined as "time between scans [seconds]".  This value is not easily calculated
   *                  from consecutive messages, and is thus left to the user to set correctly.
   * @param range_min The minimum range for the sensor_msgs::msg::LaserScan.  This is used to determine how close
   *                  of a value to allow through when multiple radii correspond to the same angular increment.
   * @param range_max The maximum range for the sensor_msgs::msg::LaserScan.  This is used to set the output message.
   * @param scan_height The number of rows (pixels) to use in the output.  This will provide scan_height number of
   *                    radii for each angular increment.  The output scan will output the closest radius that is
   *                    still not smaller than range_min.  This can be used to vertically compress obstacles into
   *                    a single LaserScan.
   * @param scan_offset Center position of the LaserScan. A value of 0.0 corresponds to the top row of the image
   *                    while 1.0 corresponds to the bottom row of the image.
   * @param frame_id The output frame_id for the LaserScan.  This will probably NOT be the same frame_id as the
   *                 depth image.  Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while
   *                 the camera uses 'camera_depth_optical_frame'.
   *
   */
  explicit DepthImageToLaserScan(
    float scan_time, float range_min, float range_max, int scan_height, float scan_offset,
    const std::string & frame_id);

  ~DepthImageToLaserScan();

  /**
   * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
   *
   * This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
   * a sensor_msgs::msg::LaserScan as accurately as possible.  To do this, it requires the synchronized Image/CameraInfo
   * pair associated with the image.
   *
   * @param depth_msg UInt16 or Float32 encoded depth image.
   * @param info_msg CameraInfo associated with depth_msg
   * @return sensor_msgs::msg::LaserScan::SharedPtr for the center row(s) of the depth image.
   *
   */
  sensor_msgs::msg::LaserScan::UniquePtr convert_msg(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

private:
  /**
   * Computes euclidean length of a cv::Point3d (as a ray from origin)
   *
   * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
   *
   * @param ray The ray for which the magnitude is desired.
   * @return Returns the magnitude of the ray.
   *
   */
  double magnitude_of_ray(const cv::Point3d & ray) const;

  /**
   * Computes the angle between two cv::Point3d
   *
   * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
   * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
   *
   * @param ray1 The first ray
   * @param ray2 The second ray
   * @return The angle between the two rays (in radians)
   *
   */
  double angle_between_rays(const cv::Point3d & ray1, const cv::Point3d & ray2) const;

  /**
   * Determines whether or not new_value should replace old_value in the LaserScan.
   *
   * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
   * new_value is 'more ideal' (currently shorter range) than old_value.
   *
   * @param new_value The current calculated range.
   * @param old_value The current range in the output LaserScan.
   * @param range_min The minimum acceptable range for the output LaserScan.
   * @param range_max The maximum acceptable range for the output LaserScan.
   * @return If true, insert new_value into the output LaserScan.
   *
   */
  bool use_point(
    const float new_value, const float old_value, const float range_min,
    const float range_max) const;

  /**
   * Converts the depth image to a laserscan using the DepthTraits to assist.
   *
   * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
   * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
   * a specific angular measurement, then the shortest range is used.
   *
   * @param depth_msg The UInt16 or Float32 encoded depth message.
   * @param cam_model The image_geometry camera model for this image.
   * @param scan_msg The output LaserScan.
   * @param scan_height The number of vertical pixels to feed into each angular_measurement.
   * @param scan_offset Height ratio of the image where the center of the scan line should be (0.0=top row, 1.0=bottom row).
   *
   */
  template<typename T>
  void convert(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const image_geometry::PinholeCameraModel & cam_model,
    const sensor_msgs::msg::LaserScan::UniquePtr & scan_msg,
    const int & scan_height,
    const float & scan_offset) const
  {
    // Use correct principal point from calibration
    const float center_x = cam_model.cx();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters(T(1));
    const float constant_x = unit_scaling / cam_model.fx();

    // Initialize lookup tables for this image configuration
    initializeLookupTables(depth_msg->width, center_x, constant_x, scan_msg);

    const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
    const int row_step = depth_msg->step / sizeof(T);

    const int offset = static_cast<int>((cam_model.cy() * 2 * scan_offset) -
      static_cast<double>(scan_height) / 2.0);
    depth_row += offset * row_step;  // Offset to center of image

    // Pre-cache commonly used values
    const double range_min = scan_msg->range_min;
    const double range_max = scan_msg->range_max;
    const double angle_min = scan_msg->angle_min;
    const double angle_increment = scan_msg->angle_increment;
    const uint32_t ranges_size = scan_msg->ranges.size();

    // Check if we need to handle distortion
    const bool has_distortion = (cam_model.cameraInfo().distortion_model == "rational_polynomial" && 
                                  cam_model.cameraInfo().d.size() >= 8);

    for (int v = offset; v < offset + scan_height_; v++, depth_row += row_step) {
      for (uint32_t u = 0; u < depth_msg->width; u++) {  // Loop over each pixel in row
        const T depth = depth_row[u];

        if (!depthimage_to_laserscan::DepthTraits<T>::valid(depth)) { // Not NaN or Inf
            continue;  // Skip invalid depths
        }

        double r;
        int index;

        if (has_distortion) {
            // Handle distorted case (slower path, but necessary for accuracy)
            const auto& k = cam_model.cameraInfo().k;
            const double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);

            // Create the camera matrix (static to avoid re-creation)
            static const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                  k[0], k[1], k[2],
                  k[3], k[4], k[5],
                  k[6], k[7], k[8]);
            
            // Undistort point
            std::vector<cv::Point2d> distorted_points{cv::Point2d(u, v)};
            std::vector<cv::Point2d> undistorted_points;
            cv::undistortPoints(distorted_points, undistorted_points, 
                  cameraMatrix, cam_model.distortionCoeffs());
            
            const double x = undistorted_points[0].x * z;
            const double th = -fastAtan2(x, z);
            r = z;  // Use z directly for distorted case
            index = static_cast<int>((th - angle_min) / angle_increment);
        } else {
            // Optimized path for common undistorted case
            const double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);
            const double x = x_factor_lookup_[u] * depth;
            
            // Use pre-calculated index from lookup table
            index = index_lookup_[u];
            
            // Calculate actual distance - optimized sqrt calculation
            r = std::sqrt(x * x + z * z);
        }

        // Bounds checking for index
        if (index >= 0 && index < static_cast<int>(ranges_size)) {
          // Determine if this point should be used
          if (use_point(r, scan_msg->ranges[index], range_min, range_max)) {
            scan_msg->ranges[index] = r;
          }
        }
      }
    }
  }

  ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
  image_geometry::PinholeCameraModel cam_model_;

  float scan_time_;  ///< Stores the time between scans.
  float range_min_;  ///< Stores the current minimum range to use.
  float range_max_;  ///< Stores the current maximum range to use.
  int scan_height_;  ///< Number of pixel rows to use when producing a laserscan from an area.
  ///< Height ratio of the image where the center of the scan line should be.
  // (0.0=top row, 1.0=bottom row).
  float scan_offset_;
  ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.
  std::string output_frame_id_;

  // Lookup tables for optimization
  mutable std::vector<double> angle_lookup_;  ///< Pre-computed angles for each pixel column
  mutable std::vector<int> index_lookup_;  ///< Pre-computed scan indices for each pixel column
  mutable std::vector<double> x_factor_lookup_;  ///< Pre-computed x factors for each pixel column
  mutable bool lookup_tables_initialized_;  ///< Whether lookup tables have been initialized
  mutable uint32_t cached_image_width_;  ///< Cached image width for lookup table validation
  mutable double cached_center_x_;  ///< Cached center_x for lookup table validation
  mutable double cached_constant_x_;  ///< Cached constant_x for lookup table validation

  /**
   * Fast atan2 approximation using polynomial approximation.
   * Significantly faster than std::atan2 with acceptable accuracy for laser scan generation.
   *
   * @param y The y component
   * @param x The x component
   * @return The angle in radians
   */
  inline double fastAtan2(double y, double x) const
  {
    if (x == 0.0) {
      return (y > 0.0) ? M_PI_2 : -M_PI_2;
    }
    
    const double ratio = y / x;
    const double abs_ratio = std::abs(ratio);
    
    // Polynomial approximation for atan(ratio)
    // Using Chebyshev approximation for better accuracy
    double result;
    if (abs_ratio <= 1.0) {
      const double ratio2 = ratio * ratio;
      result = ratio * (0.9999993329 - 0.3332985605 * ratio2 + 
                       0.1996058543 * ratio2 * ratio2 - 
                       0.1390853351 * ratio2 * ratio2 * ratio2);
    } else {
      const double inv_ratio = 1.0 / ratio;
      const double inv_ratio2 = inv_ratio * inv_ratio;
      result = M_PI_2 - inv_ratio * (0.9999993329 - 0.3332985605 * inv_ratio2 + 
                                    0.1996058543 * inv_ratio2 * inv_ratio2 - 
                                    0.1390853351 * inv_ratio2 * inv_ratio2 * inv_ratio2);
      if (ratio < 0.0) result = -result;
    }
    
    // Adjust for quadrant
    if (x < 0.0) {
      result = (y >= 0.0) ? result + M_PI : result - M_PI;
    }
    
    return result;
  }

  /**
   * Initialize lookup tables for angle and index calculations.
   * This is called once per unique image configuration to pre-compute
   * trigonometric values and avoid repeated calculations.
   *
   * @param image_width Width of the depth image
   * @param center_x Principal point x coordinate
   * @param constant_x Pre-computed constant for x calculation
   * @param scan_msg The laser scan message with angle information
   */
  void initializeLookupTables(
    uint32_t image_width, 
    double center_x, 
    double constant_x,
    const sensor_msgs::msg::LaserScan::UniquePtr & scan_msg) const
  {
    // Check if tables are already initialized for these parameters
    if (lookup_tables_initialized_ && 
        cached_image_width_ == image_width &&
        cached_center_x_ == center_x &&
        cached_constant_x_ == constant_x) {
      return;
    }

    // Resize lookup tables
    angle_lookup_.resize(image_width);
    index_lookup_.resize(image_width);
    x_factor_lookup_.resize(image_width);

    const double angle_increment = scan_msg->angle_increment;
    const double angle_min = scan_msg->angle_min;

    // Pre-calculate angle and index for each column
    for (uint32_t u = 0; u < image_width; ++u) {
      const double u_offset = static_cast<double>(u) - center_x;
      x_factor_lookup_[u] = u_offset * constant_x;
      
      // Use fast atan approximation
      angle_lookup_[u] = -fastAtan2(u_offset * constant_x, 1.0);
      
      // Pre-calculate the index for this column
      index_lookup_[u] = static_cast<int>((angle_lookup_[u] - angle_min) / angle_increment);
    }

    // Cache parameters
    cached_image_width_ = image_width;
    cached_center_x_ = center_x;
    cached_constant_x_ = constant_x;
    lookup_tables_initialized_ = true;
  }
};
}  // namespace depthimage_to_laserscan

#endif  // DEPTHIMAGE_TO_LASERSCAN__DEPTHIMAGETOLASERSCAN_HPP_
