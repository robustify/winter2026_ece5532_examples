#include <eigen3/Eigen/Dense>
#include <iostream>  // For cout
#include <gps_tools/conversions.h> // UTM conversion functions
#include "enu.hpp" // ECEF and ENU conversion functions


int main(int argc, char** argv)
{
  double ref_lat_deg = 42.6713972;
  double ref_lon_deg = -83.2156750;
  double ref_alt = 281.0;

  double current_lat_deg = 42.6708444;
  double current_lon_deg = -83.2141333;
  double current_alt = 278.0;

  double ref_lat = DEG2RAD * ref_lat_deg;
  double ref_lon = DEG2RAD * ref_lon_deg;
  double current_lat = DEG2RAD * current_lat_deg;
  double current_lon = DEG2RAD * current_lon_deg;

  Eigen::Vector3d ref_ecef;
  Eigen::Vector3d current_ecef;
  ref_ecef = llh_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt);
  current_ecef = llh_to_ecef(current_lat_deg, current_lon_deg, current_alt);

  std::cout << std::fixed << "Reference ECEF coordinates:\n" << ref_ecef << "\n\n";
  std::cout << std::fixed << "Current ECEF coordinates:\n" << current_ecef << "\n\n";

  Eigen::Matrix3d enu_rot_mat;
  enu_rot_mat << -sin(ref_lon),                 cos(ref_lon),                0,
                 -sin(ref_lat) * cos(ref_lon), -sin(ref_lat) * sin(ref_lon), cos(ref_lat),
                  cos(ref_lat) * cos(ref_lon),  cos(ref_lat) * sin(ref_lon), sin(ref_lat);

  Eigen::Vector3d current_enu;
  current_enu = enu_rot_mat * (current_ecef - ref_ecef);

  std::cout << "Current ENU coordinates:\n" << current_enu << "\n\n";

  Eigen::Vector3d ref_utm;
  Eigen::Vector3d current_utm;
  std::string utm_zone;
  gps_tools::LLtoUTM(ref_lat_deg, ref_lon_deg, ref_utm.y(), ref_utm.x(), utm_zone);
  gps_tools::LLtoUTM(current_lat_deg, current_lon_deg, current_utm.y(), current_utm.x(), utm_zone);
  ref_utm.z() = ref_alt;
  current_utm.z() = current_alt;

  std::cout << "UTM zone: " << utm_zone << "\n";
  std::cout << std::fixed << "Reference UTM coordinates:\n" << ref_utm << "\n\n";
  std::cout << std::fixed << "Current UTM coordinates:\n" << current_utm << "\n\n";

  Eigen::Vector3d relative_utm;
  relative_utm = current_utm - ref_utm;
  std::cout << std::fixed << "Relative UTM position:\n" << relative_utm << "\n\n";

  // Calculate distance between current and reference point using both ENU and UTM
  std::cout << "ENU distance: " << current_enu.norm() << "  UTM distance: " << relative_utm.norm() << "\n";

  // Compare empirically-measured convergence angle with the theoretical convergence angle
  double angle = acos(current_enu.dot(relative_utm) / current_enu.norm() / relative_utm.norm());
  std::cout << "Empirical convergence angle: " << angle * 180.0 / M_PI << " deg\n";

  double zone_17_lon = -81.0 * M_PI / 180.0;
  double conv_angle = atan(tan(current_lon - zone_17_lon) * sin(current_lat));
  std::cout << "Theoretical convergence angle: " << conv_angle * 180.0 / M_PI << " deg\n";
}
