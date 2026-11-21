#include "two_target_episode_guard.h"
#include <cassert>
#include <iostream>
template<class F> void rejects(F f){bool bad=false;try{f();}catch(const std::runtime_error&){bad=true;}assert(bad);}
int main(){
 // Recorded failure: first ON arrives during the routine OFF refresh worker.
 assert(TwoTargetEpisodeGuard::offReady(true,false,true,true,false));
 assert(TwoTargetEpisodeGuard::offReady(true,false,true,false,false));
 assert(!TwoTargetEpisodeGuard::offReady(true,false,true,true,true));
 assert(!TwoTargetEpisodeGuard::offReady(false,false,true,true,false));
 assert(!TwoTargetEpisodeGuard::offReady(true,true,true,false,false));
 assert(!TwoTargetEpisodeGuard::offReady(true,false,false,false,false));
 TwoTargetEpisodeGuard refresh;
 refresh.approve({1,3},0,1,{},1,false,TwoTargetEpisodeGuard::offReady(true,false,true,true,false));
 assert(refresh.count==1);
 refresh.approve({1,3},0,1,{},1,true,false);assert(refresh.count==1);

 for(auto ids: {std::vector<int>{1,2},std::vector<int>{1,3},std::vector<int>{2,3}}){
  TwoTargetEpisodeGuard g;
  rejects([&]{g.approve(ids,1,ids[1],{ids[0]},1,false,true);});
  g.approve(ids,0,ids[0],{},1,false,true);assert(g.count==1);
  g.approve(ids,0,ids[0],{},1,true,false);assert(g.count==1);
  rejects([&]{g.approve(ids,1,ids[1],{ids[0]},5,false,false);});
  rejects([&]{g.approve(ids,1,ids[1],{},5,false,true);});
  rejects([&]{g.approve(ids,0,ids[0],{},3,false,true);});
  rejects([&]{g.approve({ids[1],ids[0]},0,ids[0],{},1,true,false);});
  g.approve(ids,1,ids[1],{ids[0]},5,false,true);assert(g.count==2);
  g.approve(ids,1,ids[1],{ids[0]},5,true,false);assert(g.count==2);
  rejects([&]{g.approve(ids,1,ids[1],{ids[0]},7,false,true);});
  rejects([&]{g.approve(ids,0,ids[0],{},5,true,false);});
 }
 TwoTargetEpisodeGuard g;rejects([&]{g.approve({3,1},0,3,{},1,false,true);});
 rejects([&]{g.approve({1,1},0,1,{},1,false,true);});
 std::cout<<"two-target C++ episode guard: all three pairs, OFF gate, order, repeat/third/tamper rejected\n";
}
