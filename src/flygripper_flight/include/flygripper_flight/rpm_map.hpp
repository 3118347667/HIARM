#pragma once
#include <ros/ros.h>
#include <queue>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>

#include "flygripper_common/type/base.hpp"

namespace flygripper_flight{
using namespace flygripper_common;
class RpmMap {
 public:
  RpmMap():
      map_type_(MapType::NONE){
    resetMap();
  };
  ~RpmMap() = default;

  bool valid() const{
    return (map_type_ == MapType::WITH_VOLT_LOOKUP_TABLE &&
        std::isfinite(voltage_min_) && std::isfinite(voltage_max_) &&
        std::isfinite(rpm_min_) && std::isfinite(rpm_max_) &&
        lookup_table_.allFinite()) ||
        (map_type_ == MapType::WITH_VOLT_CURVE_FIT && coeff_with_volt_.allFinite()) ||
        (map_type_ == MapType::WITHOUT_VOLT_CURVE_FIT && coeff_without_volt_.allFinite());
  }

  bool init_precise_model(const std::filesystem::path& file_path){
    std::ifstream csvFile(file_path.string());

    if (!csvFile.is_open()){
      ROS_ERROR("Could not open rpm map file %s!", file_path.c_str());
      return false;
    }

    std::string line;

    // First line with general parameter
    if (!getline(csvFile, line)){
      ROS_ERROR("Could not read first line of rpm map csv!");
      return false;
    }


    std::istringstream ss(line);
    std::string num;

    if (!getline(ss, num, ',')){
      ROS_ERROR("Error in rpm map csv!");
      return false;
    }
    const int map_size = stoi(num);

    if (!getline(ss, num, ',')){
      ROS_ERROR("Error in rpm map csv!");
      return false;
    }
    rpm_min_ = stod(num);

    if (!getline(ss, num, ',')){
      ROS_ERROR("Error in rpm map csv!");
      return false;
    }
    rpm_max_ = stod(num);

    if (!getline(ss, num, ',')){
      ROS_ERROR("Error in rpm map csv!");
      return false;
    }
    voltage_min_ = stod(num);

    if (!getline(ss, num, ',')){
      ROS_ERROR("Error in rpm map csv!");
      return false;
    }
    voltage_max_ = stod(num);

    if ((voltage_max_ - voltage_min_) < 0.5) {
      voltage_max_ += 0.5;
    }

    lookup_table_ = Matrix<>::Zero(map_size, map_size);

    for (int row = 0; row < lookup_table_.rows(); ++row) {
      if (!getline(csvFile, line)){
        ROS_ERROR("Rpm map csv has wrong size!");
        return false;
      }
      ss = std::istringstream(line);

      for (int col = 0; col < lookup_table_.cols(); ++col) {
        if (!getline(ss, num, ',')) {
          ROS_ERROR("Rpm map csv has wrong size!");
          return false;
        }
        lookup_table_(row, col) = stod(num);
      }
    }
    map_type_ = MapType::WITH_VOLT_LOOKUP_TABLE;
    return true;
  }

  bool init_precise_model(ConstRef<Vector<4>> coeff){
    if (!coeff.allFinite()){
      ROS_ERROR("Error in rpm map coefficients.");
      return false;
    }
    coeff_with_volt_ = coeff;
    map_type_ = MapType::WITH_VOLT_CURVE_FIT;
    return true;
  }

  bool init_simple_model(ConstRef<Vector<3>> coeff){
    if (!coeff.allFinite()){
      ROS_ERROR("Error in rpm map coefficients.");
      return false;
    }
    coeff_without_volt_ = coeff;
    map_type_ = MapType::WITHOUT_VOLT_CURVE_FIT;
    return true;
  }

  Scalar map(const Scalar& rpm, const Scalar voltage = 14.0) const
  {
    if (map_type_ == MapType::WITH_VOLT_LOOKUP_TABLE){
      const int i_rpm{
          std::clamp<int>(((int)(Scalar)lookup_table_.cols() * (rpm - rpm_min_) /
          (rpm_max_ - rpm_min_)),
          0, lookup_table_.cols() - 2)};
      const int i_voltage{
          std::clamp<int>(((int)(Scalar)lookup_table_.rows() * (voltage - voltage_min_) /
          (voltage_max_ - voltage_min_)),
          0, lookup_table_.rows() - 2)};

      const Matrix<2, 2> W = lookup_table_.block<2, 2>(i_voltage, i_rpm);

      const Scalar delta_rpm = (rpm_max_ - rpm_min_) / (lookup_table_.cols() - 1);
      const Scalar x0_rpm = i_rpm * delta_rpm + rpm_min_;
      const Scalar delta_voltage =
          (voltage_max_ - voltage_min_) / (lookup_table_.rows() - 1);
      const Scalar x0_voltage = i_voltage * delta_voltage + voltage_min_;
      const Scalar x_rpm = (rpm - x0_rpm) / (delta_rpm);
      const Scalar x_voltage = (voltage - x0_voltage) / (delta_voltage);

      const Matrix<2, 1> xv_rpm{1.0 - x_rpm, x_rpm};
      const Matrix<1, 2> xv_voltage{1.0 - x_voltage, x_voltage};

      const Scalar command = xv_voltage * W * xv_rpm;

      return compensation_scale_ * command;
    }
    else if (map_type_ == MapType::WITH_VOLT_CURVE_FIT){
      // [k0, k1, k2, k3], z * z / 1e8 = k0 * y ^ k1(k2 * x * x + (1 - k2) * x) + k3,
      // where x is dshot command, y is voltage, z is rpm
      const Scalar tmp = coeff_with_volt_(0) * std::pow(voltage, coeff_with_volt_(1));
      const Scalar a = tmp * coeff_with_volt_(2);
      const Scalar b = tmp * (1-coeff_with_volt_(2));
      const Scalar c = coeff_with_volt_(3) - rpm * rpm / 1.0e8;
      Scalar discriminant = b * b - 4 * a * c;
      if (discriminant < 0){
        ROS_ERROR("Negative discriminant in rpm map with mode WITH_VOLT_CURVE_FIT.");
        discriminant = 0.0;
      }
      return compensation_scale_ * (-b + std::sqrt(discriminant)) / (2 * a);
    }
    else if (map_type_ == MapType::WITHOUT_VOLT_CURVE_FIT){
      // [k0, k1, k2], y = k0 + k1 * x + k2 * x * x  where x is rpm, y is dshot command
      return compensation_scale_ *
          (coeff_without_volt_(2)*rpm*rpm + coeff_without_volt_(1)*rpm + coeff_without_volt_(0));
    }
    else {
      ROS_ERROR("It is invalid to call map function before initialize it, "
                "please call init function to init it first");
      return 0.0;
    }
  };

  bool updateMap(Scalar cur_command,
                 Scalar cur_rpm,
                 Scalar cur_voltage = 14.0){
    // 1.Push into queue, should update the map right after mapping to get the latest command
    ros::Time now = ros::Time::now();
    command_queue_.push(std::pair<ros::Time, Scalar>(now,
                                                     cur_command));
    while (command_queue_.size() > 100){ // keep the queue small size
      command_queue_.pop();
    }

    // 2. Pop the prev_command
    //    from time range (generally 35-45 ms), based on system response time
    while (command_queue_.size() >= 1) {
      std::pair<ros::Time, Scalar> prev_stamped_command = command_queue_.front();
      double time_passed = (now - prev_stamped_command.first).toSec();
      if (time_passed > response_time_upper_bound_)
      {
        command_queue_.pop();
        continue;
      }
      if (time_passed < response_time_lower_bound_)
      {
        return false;
      }

      /**************************************************************************/
      /* Update the Model with Recursive Least Squares algorithm with vanishing memory
       * command = compensation_scale_ * map(rpm)  or
       *         = compensation_scale_ * map(rpm, voltage) */
      /**************************************************************************/
      Scalar command;
      Scalar prev_command = prev_stamped_command.second;
      if (map_type_ == MapType::WITH_VOLT_LOOKUP_TABLE ||
          map_type_ == MapType::WITH_VOLT_CURVE_FIT) {
        command = map(cur_rpm, cur_voltage);
      } else if (map_type_ == MapType::WITHOUT_VOLT_CURVE_FIT) {
        command = map(cur_rpm);
      } else {
        ROS_ERROR(
            "Rpm mapping need to init the model before update the map.");
        return false;
      }

      Scalar gamma = 1 / (rho2_ + command * P_ * command);
      Scalar K = gamma * P_ * command;
      compensation_scale_ = compensation_scale_ +
          K * (prev_command - command * compensation_scale_);
      P_ = (1 - K * command) * P_ / rho2_;

      if (compensation_scale_ > 1.15 || compensation_scale_ < 0.85) {
        ROS_ERROR(
            "Rpm mapping scale = %f, which shoule around 1. "
            "It means the thrust model is no longer accurate. "
            "Re-calibrate the dshot mapping model!",
            compensation_scale_);
        compensation_scale_ =
            compensation_scale_ > 1.15 ? 1.15 : compensation_scale_;
        compensation_scale_ =
            compensation_scale_ < 0.85 ? 0.85 : compensation_scale_;
      }
    }
    return true;
  }

 private:

  void resetMap(){
    P_ = 1e6;
    compensation_scale_ = 1.0;
  }

  //! Map type
  enum class MapType{
    NONE = -1,
    WITHOUT_VOLT_CURVE_FIT = 0,
    WITH_VOLT_CURVE_FIT = 1,
    WITH_VOLT_LOOKUP_TABLE = 2,
  };
  MapType map_type_;

  //! with voltage mapping: lookup table approach
  Matrix<> lookup_table_ = Matrix<>::Constant(1, 1, NAN);
  Scalar voltage_min_{NAN};
  Scalar voltage_max_{NAN};
  Scalar rpm_min_{NAN};
  Scalar rpm_max_{NAN};
  //! with voltage mapping: curve fit approach
  Vector<4> coeff_with_volt_{Vector<4>::Constant(INF)};
  //! without voltage mapping: curve fit approach
  Vector<3> coeff_without_volt_{Vector<3>::Constant(INF)};

  //! Update map using recursive least squares algorithm with vanishing memory
  Scalar compensation_scale_{1.0};
  Scalar P_{1e6};
  const double rho2_{0.998};
  const Scalar response_time_lower_bound_{0.035};
  const Scalar response_time_upper_bound_{0.045};
  std::queue<std::pair<ros::Time, Scalar>> command_queue_;

};

}