// Pure two-episode authorization, tested independently of ROS/hardware.
#ifndef TWO_TARGET_EPISODE_GUARD_H
#define TWO_TARGET_EPISODE_GUARD_H
#include <vector>
#include <stdexcept>
class TwoTargetEpisodeGuard {
 public:
  // A periodic OFF worker cannot energize the laser; an earlier OFF ACK remains valid.
  static bool offReady(bool known,bool ack_on,bool startup_off_acked,bool child_running,bool child_on) {
    return known && !ack_on && startup_off_acked && !(child_running && child_on);
  }
  int count=0,active_target=0,last_on_id=-1;
  std::vector<int> selected;
  void approve(const std::vector<int>& ids,int index,int target,const std::vector<int>& done,
               int request_id,bool continuing,bool off_confirmed) {
    if (ids.size()!=2 || ids[0]<1 || ids[1]>3 || ids[0]>=ids[1] || index<0 || index>1 || target!=ids[index])
      throw std::runtime_error("invalid_selected_target_order");
    if (!selected.empty() && ids!=selected)throw std::runtime_error("selected_targets_changed");
    if (continuing) {
      if (index!=count-1 || target!=active_target || request_id!=last_on_id || done.size()!=static_cast<unsigned>(index))
        throw std::runtime_error("active_target_changed_during_exposure");
      if (index==1 && done[0]!=ids[0])throw std::runtime_error("previous_target_not_completed");
      return;
    }
    if (count>=2 || index!=count || request_id<=last_on_id || !off_confirmed || done.size()!=static_cast<unsigned>(index))
      throw std::runtime_error("next_target_without_off_or_wrong_episode");
    if (index==1 && done[0]!=ids[0])throw std::runtime_error("previous_target_not_completed");
    selected=ids;active_target=target;last_on_id=request_id;++count;
  }
};
#endif
