#include "visual_calibration_guard.h"
#include <cassert>
#include <limits>
#include <iostream>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>

int main(int argc, char**) {
  using G=VisualCalibrationGuard;
  if (argc>1) {
    std::string line;
    while (std::getline(std::cin,line)) {
      boost::property_tree::ptree hover;
      try {
        std::istringstream input(line);boost::property_tree::read_json(input,hover);
        std::cout<<(G::scoringProfileAllowed(hover) ? "ACCEPT" : "REJECT")<<"\n";
      } catch (const std::exception&) {std::cout<<"REJECT\n";}
    }
    return 0;
  }
  for (const auto& stage : {"ON_WAIT", "EXPOSE"}) {
    assert(G::timedVisualAimAllowed("off",false,true,true,true,1.,3.,true,stage));
    assert(!G::timedVisualAimAllowed("off",false,true,true,true,1.,3.,false,stage));
    assert(!G::timedVisualAimAllowed("off",false,true,false,true,1.,3.,true,stage));
    assert(!G::timedVisualAimAllowed("off",false,true,true,false,1.,3.,true,stage));
    assert(!G::timedVisualAimAllowed("off",true,true,true,true,1.,3.,true,stage));
    assert(!G::timedVisualAimAllowed("off",false,false,true,true,1.,3.,true,stage));
    for (const auto& mode : {"learn", "reuse", "unknown"})
      assert(!G::timedVisualAimAllowed(mode,false,true,true,true,1.,3.,true,stage));
    for (double bad : {0.,.9,1.1,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
      assert(!G::timedVisualAimAllowed("off",false,true,true,true,bad,3.,true,stage));
    for (double bad : {0.,2.,4.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
      assert(!G::timedVisualAimAllowed("off",false,true,true,true,1.,bad,true,stage));
  }
  for (const auto& stage : {"CENTER", "OFF_WAIT", "DONE", "", "ABORT"})
    assert(!G::timedVisualAimAllowed("off",false,true,true,true,1.,3.,true,stage));
  assert(G::profileAllowed("off",true,false,false));
  assert(!G::profileAllowed("off",false,true,false));
  assert(G::profileAllowed("learn",false,true,true));
  assert(!G::profileAllowed("learn",false,true,false));
  std::cout << "timed visual aim guard: ON_WAIT/EXPOSE accepted; invalid/incomplete profiles rejected\n";
}
