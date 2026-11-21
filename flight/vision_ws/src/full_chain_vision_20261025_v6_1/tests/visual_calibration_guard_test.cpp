#include "visual_calibration_guard.h"
#include <cassert>
int main() {
  assert(VisualCalibrationGuard::profileAllowed("off",true,false,false));
  for(const std::string mode:{"learn","reuse"}) {
    assert(VisualCalibrationGuard::profileAllowed(mode,false,true,true));
    assert(!VisualCalibrationGuard::profileAllowed(mode,true,true,true));
    assert(!VisualCalibrationGuard::profileAllowed(mode,false,false,true));
    assert(!VisualCalibrationGuard::profileAllowed(mode,false,true,false));
  }
  assert(!VisualCalibrationGuard::profileAllowed("unknown",false,true,true));
  assert(!VisualCalibrationGuard::profileAllowed("off",false,true,true));
}
