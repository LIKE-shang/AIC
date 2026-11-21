// Separate single-use automatic-landing authorization; scoring episodes unchanged.
#ifndef LANDING_ILLUMINATION_GUARD_H
#define LANDING_ILLUMINATION_GUARD_H
#include <string>
#include <stdexcept>
class LandingIlluminationGuard {
 public:
  bool used=false,active=false;
  int on_id=-1;
  void approve(bool enabled,const std::string& authorization,const std::string& phase,int id,
               int scoring_count,int last_scoring_id,bool second_clear,bool home_stopped,bool off_ready,bool continuing) {
    if (!enabled || authorization!="supervised_second_window_return_auto_landing_20261026" ||
        !second_clear || !home_stopped || scoring_count!=2)
      throw std::runtime_error("landing_scope_or_route_incomplete");
    if (continuing) {
      if (!active || id!=on_id || (phase!="LANDING_LIGHT_ON" && phase!="LANDING_HANDOVER" && phase!="AUTO_LANDING"))
        throw std::runtime_error("landing_illumination_changed_or_restarted");
    } else {
      if (used || phase!="LANDING_LIGHT_ON" || id<=last_scoring_id || !off_ready)
        throw std::runtime_error("landing_requires_new_id_OFF_and_single_episode");
      used=active=true;on_id=id;
    }
  }
  void stop(){active=false;}
  static bool modeAllowed(bool active,const std::string& mode){return mode=="OFFBOARD" || (active && mode=="AUTO.LAND");}
  static bool withinDeadline(double seconds,bool landing){return seconds<(landing?45.:5.);}
};
#endif
