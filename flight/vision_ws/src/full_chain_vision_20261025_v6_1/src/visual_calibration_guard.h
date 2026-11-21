#pragma once
#include <cmath>
#include <string>
#include <boost/property_tree/ptree.hpp>

struct VisualCalibrationGuard {
  static const char* authorization() { return "observed_spot_calibration_20261026"; }
  static bool active(const std::string& mode) { return mode=="learn" || mode=="reuse"; }
  static bool profileAllowed(const std::string& mode, bool fixed, bool servo, bool baseline_clear) {
    return (mode=="off" && fixed && !servo) ||
           (active(mode) && !fixed && servo && baseline_clear);
  }
  // Scoring profile only. ON_WAIT and EXPOSE heartbeats carry the same contract.
  static bool timedVisualAimAllowed(const std::string& mode, bool fixed, bool servo,
      bool timed, bool visual_hold, double budget_s, double exposure_s,
      bool budget_elapsed, const std::string& stage) {
    return mode=="off" && !fixed && servo && timed && visual_hold &&
           std::isfinite(budget_s) && std::fabs(budget_s-1.0)<1e-6 &&
           std::isfinite(exposure_s) && std::fabs(exposure_s-3.0)<1e-6 &&
           budget_elapsed && (stage=="ON_WAIT" || stage=="EXPOSE");
  }
  static bool scoringProfileAllowed(const boost::property_tree::ptree& hover) {
    const std::string mode=hover.get<std::string>("visual_calibration_mode","off");
    const bool fixed=hover.get<bool>("fixed_position_timed");
    const bool servo=hover.get<bool>("visual_servo_enabled");
    if (hover.get<bool>("timed_visual_aim",false))
      return timedVisualAimAllowed(mode,fixed,servo,true,
          hover.get<bool>("visual_target_hold",false),hover.get<double>("visual_aim_budget_s",0.),
          hover.get<double>("exposure_s",0.),hover.get<bool>("visual_aim_budget_elapsed",false),
          hover.get<std::string>("stage",""));
    return profileAllowed(mode,fixed,servo,hover.get<bool>("calibration_baseline_clear",false)) &&
           hover.get<bool>("gates.position_stable",false) && hover.get<bool>("gates.fixed_position_in_region",false);
  }
};
