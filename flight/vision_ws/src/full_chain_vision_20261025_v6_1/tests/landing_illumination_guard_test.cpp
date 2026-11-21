#include "landing_illumination_guard.h"
#include "two_target_episode_guard.h"
#include <cassert>
#include <functional>
static void rejects(std::function<void()> f){bool threw=false;try{f();}catch(const std::runtime_error&){threw=true;}assert(threw);}
int main(){
 LandingIlluminationGuard g;const std::string a="supervised_second_window_return_auto_landing_20261026";
 auto start=[&](bool e,int count,bool clear,bool home,bool off){g.approve(e,a,"LANDING_LIGHT_ON",6,count,4,clear,home,off,false);};
 rejects([&]{start(false,2,true,true,true);});rejects([&]{start(true,1,true,true,true);});
 rejects([&]{start(true,2,false,true,true);});rejects([&]{start(true,2,true,false,true);});rejects([&]{start(true,2,true,true,false);});
 start(true,2,true,true,true);assert(g.used && g.active);
 for(auto p:{"LANDING_LIGHT_ON","LANDING_HANDOVER","AUTO_LANDING"})g.approve(true,a,p,6,2,4,true,true,false,true);
 rejects([&]{g.approve(true,a,"AUTO_LANDING",7,2,4,true,true,false,true);});
 assert(!LandingIlluminationGuard::modeAllowed(false,"POSCTL"));assert(!LandingIlluminationGuard::modeAllowed(true,"POSCTL"));
 assert(!LandingIlluminationGuard::modeAllowed(false,"AUTO.LAND"));assert(LandingIlluminationGuard::modeAllowed(true,"AUTO.LAND"));
 assert(!LandingIlluminationGuard::withinDeadline(5.,false));assert(LandingIlluminationGuard::withinDeadline(6.,true));assert(!LandingIlluminationGuard::withinDeadline(45.,true));
 g.stop();rejects([&]{start(true,2,true,true,true);});
 TwoTargetEpisodeGuard t;t.approve({1,3},0,1,{},1,false,true);t.approve({1,3},1,3,{1},3,false,true);
 rejects([&]{t.approve({1,3},1,3,{1},7,false,true);});assert(t.count==2);
}
