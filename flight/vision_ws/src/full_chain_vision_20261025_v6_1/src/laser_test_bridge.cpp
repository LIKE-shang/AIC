// Independent, fail-closed bridge to the EXISTING audited lib_pwm_control(int,int).
// No mission header, arming/mode client, navigation, setpoint or motion publisher.
#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>
#include <robot_vision/LaserDecision.h>
#include <mavros_msgs/State.h>
#include <cstddef>
#include <std_msgs/String.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <dirent.h>
#include <cstdlib>
#include <fstream>
#include <openssl/sha.h>

using Clock = std::chrono::steady_clock;
static double elapsed(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now()-t).count();
}
static bool testTopic(const std::string& s) {
  return s.compare(0, std::strlen("/circle_laser_test/"), "/circle_laser_test/") == 0;
}
static std::string escape(const std::string& s) {
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') out += '\\';
    if (static_cast<unsigned char>(c) >= 32) out += c;
  }
  return out;
}

class PwmBackend {
 public:
  explicit PwmBackend(const std::string& path) {
    // The shipped builder is Linux/AArch64; do not load an arbitrary substitute.
#if !defined(__aarch64__)
    throw std::runtime_error("Audited PWM library requires AArch64 Linux hardware");
#endif
    // Pin the audited binary BEFORE dlopen (ELF constructors may execute).
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot read audited PWM library");
    SHA256_CTX ctx; SHA256_Init(&ctx);
    char buffer[16384];
    while (input.read(buffer, sizeof(buffer)) || input.gcount())
      SHA256_Update(&ctx, buffer, static_cast<size_t>(input.gcount()));
    if (!input.eof()) throw std::runtime_error("PWM library read failed");
    unsigned char digest[SHA256_DIGEST_LENGTH]; SHA256_Final(digest, &ctx);
    std::ostringstream hex;
    for (unsigned char b : digest) hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    if (hex.str() != "e055c1e0468cc44c114c2ff408a6303bb9e0e9ece5fcf707fd3489dfb0e2734c")
      throw std::runtime_error("PWM library SHA256 differs from audited binary; re-audit, do not bypass");
    // This binary was built against a 40-byte CommandLong service object.
    using Request = mavros_msgs::CommandLong::Request;
    if (sizeof(mavros_msgs::CommandLong)!=40 || offsetof(Request,command)!=2 ||
        offsetof(Request,param1)!=8 || offsetof(Request,param2)!=12)
      throw std::runtime_error("CommandLong ABI does not match audited library");
    handle_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!handle_) throw std::runtime_error(std::string("dlopen: ")+dlerror());
    // Verified ELF symbol names in uav_mission/lib/liblib.so, not invented ctypes APIs.
    function_ = reinterpret_cast<Function>(dlsym(handle_, "_Z15lib_pwm_controlii"));
    command_ = reinterpret_cast<mavros_msgs::CommandLong*>(dlsym(handle_, "lib_ctrl_pwm"));
    if (!function_ || !command_) throw std::runtime_error("PWM library missing audited symbols");
    // Static inspection found command=187, param1=p5/50-1, param2=p6/50-1,
    // param3..param7 untouched (zero after reset). Test this runtime ABI WITHOUT sending any hardware command.
    verify(0, 0); verify(100, 100); verify(25, 75);
  }
  ~PwmBackend() { if (handle_) dlclose(handle_); }
  mavros_msgs::CommandLong build(int five, int six) {
    *command_ = mavros_msgs::CommandLong(); // Clear ALL stale fields, including response.
    function_(five, six);
    return *command_;
  }
 private:
  using Function = void (*)(int, int);
  void* handle_ = nullptr;
  Function function_ = nullptr;
  mavros_msgs::CommandLong* command_ = nullptr;
  void verify(int a, int b) {
    const auto s = build(a,b);
    const auto& q = s.request;
    if (q.broadcast || q.command != 187 || q.confirmation != 0 ||
        !std::isfinite(q.param1) || !std::isfinite(q.param2) ||
        std::fabs(q.param1-(a/50.0f-1.0f)) > 1e-6 ||
        std::fabs(q.param2-(b/50.0f-1.0f)) > 1e-6 ||
        q.param3 != 0.0f || q.param4 != 0.0f || q.param5 != 0.0f || q.param6 != 0.0f || q.param7 != 0.0f)
      throw std::runtime_error("PWM library ABI/behavior differs from audited binary: refusing hardware");
  }
};

struct Settings {
  std::string library, service, request_topic, status_topic;
  int channel, on_value, off_value, other_value, attempts, on_frames;
  double lease, call_timeout, retry, off_refresh, max_image_age, future_tolerance, state_timeout, max_on_seconds;
};

static Settings settings(ros::NodeHandle& p) {
  Settings s;
  bool enabled=false, wiring=false;
  p.param("enable_laser",enabled,false);
  p.param("hardware_wiring_confirmed",wiring,false);
  for (const std::string key : {"physical_isolation_confirmed", "independent_cutoff_confirmed", "output_mapping_confirmed"}) {
    bool confirmed=false; p.param(key,confirmed,false);
    if (!confirmed) throw std::runtime_error("Required field confirmation missing: "+key);
  }
  for (const std::string key : {"dimensions_confirmed", "calibration/confirmed", "operating_envelope_confirmed", "live_camera_verified"}) {
    bool confirmed=false; p.param(key,confirmed,false);
    if (!confirmed) throw std::runtime_error("Optical/live-source confirmation missing: "+key);
  }
  std::string feedback; p.param<std::string>("feedback_mode",feedback,"prediction_only");
  bool evidence=false;
  if (feedback=="camera") p.param("spot/validated",evidence,false);
  else if (feedback=="prediction_only") p.param("prediction_only_acknowledged",evidence,false);
  if (!evidence) throw std::runtime_error("Explicit feedback validation or prediction-only acknowledgement required");
  bool simulated=false; ros::param::param<bool>("/use_sim_time",simulated,false);
  if (simulated) throw std::runtime_error("Hardware bridge forbidden with simulated ROS time");
  p.param("laser_channel",s.channel,-1);
  p.param("laser_on_value",s.on_value,-1);
  p.param("laser_off_value",s.off_value,-1);
  p.param("other_channel_safe_value",s.other_value,-1);
  p.param("shutdown_off_attempts",s.attempts,3);
  p.param("decision_timeout",s.lease,0.7);
  p.param("on_frames",s.on_frames,5);
  p.param("max_image_age",s.max_image_age,0.7);
  p.param("future_tolerance",s.future_tolerance,0.2);
  p.param("state_timeout",s.state_timeout,3.0);
  p.param("max_on_seconds",s.max_on_seconds,5.0);
  p.param("service_call_timeout",s.call_timeout,2.0);
  p.param("retry_interval",s.retry,0.3);
  p.param("off_refresh_interval",s.off_refresh,1.0);
  p.param<std::string>("pwm_library",s.library,"");
  p.param<std::string>("command_service",s.service,"/mavros/cmd/command");
  p.param<std::string>("decision_topic",s.request_topic,"/circle_laser_test/decision");
  p.param<std::string>("laser_status_topic",s.status_topic,"/circle_laser_test/laser_status");
  s.request_topic=ros::names::resolve(s.request_topic);
  s.status_topic=ros::names::resolve(s.status_topic);
  if (!enabled || !wiring) throw std::runtime_error("Hardware bridge requires enable_laser=true AND hardware_wiring_confirmed=true");
  if (s.channel!=5 && s.channel!=6) throw std::runtime_error("Audited API only supports auxiliary channels 5 and 6");
  for (int value : {s.on_value,s.off_value,s.other_value})
    if (value<0 || value>100) throw std::runtime_error("Explicit on/off/other-channel values required, integers 0..100, NOT microseconds");
  if (s.on_value==s.off_value) throw std::runtime_error("on/off values must differ");
  if (s.library.empty()) throw std::runtime_error("pwm_library path required");
  if (!testTopic(s.request_topic) || !testTopic(s.status_topic)) throw std::runtime_error("Topics must stay under /circle_laser_test/");
  if (s.service!="/mavros/cmd/command") throw std::runtime_error("Only audited /mavros/cmd/command service allowed; remap MAVROS there");
  for (double t : {s.lease,s.call_timeout,s.retry,s.off_refresh,s.max_image_age,s.future_tolerance,s.state_timeout})
    if (!std::isfinite(t) || t<=0 || t>10) throw std::runtime_error("Timeouts must be finite, in (0,10] seconds");
  if (!std::isfinite(s.max_on_seconds) || s.max_on_seconds<3 || s.max_on_seconds>5)
    throw std::runtime_error("Independent exposure limit must be in [3,5] seconds");
  if (s.lease>0.7 || s.max_image_age>0.7) throw std::runtime_error("Handheld image lease must be <=0.7 seconds");
  if (s.on_frames<1) throw std::runtime_error("on_frames must be positive");
  if (s.attempts<1 || s.attempts>5) throw std::runtime_error("shutdown_off_attempts must be 1..5");
  return s;
}

// roscpp ServiceClient::call has no bounded response timeout. Isolate EACH call
// in an exec'ed worker, supervised by a monotonic deadline. Never fork and use
// inherited ROS threads; child performs only close/dup2/exec/_exit before exec.
struct CallResult { int magic; int transport; int success; int result; };
static const int RESULT_MAGIC = 0x4c415345;
static int worker(int argc, char** argv) {
  if (argc!=5) return 64;
  const std::string service=argv[2];
  mavros_msgs::CommandLong srv;
  std::istringstream q(argv[3]);
  int broadcast,command,confirmation;
  if (!(q>>broadcast>>command>>confirmation>>srv.request.param1>>srv.request.param2>>srv.request.param3
          >>srv.request.param4>>srv.request.param5>>srv.request.param6>>srv.request.param7)) return 65;
  if (service!="/mavros/cmd/command" || command!=187 || broadcast || confirmation ||
      srv.request.param3!=0 || srv.request.param4!=0 || srv.request.param5!=0 ||
      srv.request.param6!=0 || srv.request.param7!=0 ||
      !std::isfinite(srv.request.param1) || !std::isfinite(srv.request.param2) ||
      std::fabs(srv.request.param1)>1 || std::fabs(srv.request.param2)>1) return 66;
  srv.request.broadcast=false; srv.request.command=command; srv.request.confirmation=0;
  int fd=std::stoi(argv[4]);
  ros::M_string remaps;
  ros::init(remaps,"circle_laser_command_worker",ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  // Worker mode is internal, not an alternative way to bypass the launch gate.
  // The delivered launch fixes the supervising node name to circle_laser_bridge.
  bool allowed=false, wiring=false;
  ros::param::param<bool>("/circle_laser_bridge/enable_laser",allowed,false);
  ros::param::param<bool>("/circle_laser_bridge/hardware_wiring_confirmed",wiring,false);
  for (const std::string key : {"physical_isolation_confirmed", "independent_cutoff_confirmed", "output_mapping_confirmed"}) {
    bool confirmed=false; ros::param::param<bool>("/circle_laser_bridge/"+key,confirmed,false);
    allowed=allowed && confirmed;
  }
  if (!allowed || !wiring) {
    ROS_ERROR("PWM worker refused: explicit hardware enable/verification missing");
    ros::shutdown(); close(fd); return 67;
  }
  ros::ServiceClient c=n.serviceClient<mavros_msgs::CommandLong>(service,false);
  bool ok=c.call(srv);
  CallResult result{RESULT_MAGIC,ok ? 1 : 0,srv.response.success ? 1 : 0,static_cast<int>(srv.response.result)};
  // Unregister the anonymous worker before reporting completion, so the parent
  // cannot kill it between the ACK and ROS cleanup and leave stale graph nodes.
  c.shutdown(); ros::shutdown();
  ssize_t unused=write(fd,&result,sizeof(result)); (void)unused;
  close(fd);
  return ok && srv.response.success && srv.response.result==0 ? 0 : 1;
}

class LaserBridge {
 public:
  LaserBridge(ros::NodeHandle& n, const Settings& cfg, const std::string& executable)
    : s_(cfg), backend_(s_.library), executable_(executable) {
    status_=n.advertise<std_msgs::String>(s_.status_topic,1,true);
    sub_=n.subscribe(s_.request_topic,1,&LaserBridge::request,this);
    state_sub_=n.subscribe("/mavros/state",1,&LaserBridge::state,this);
    last_request_=last_off_ack_=Clock::now();
    next_command_=Clock::now();
    publish("startup_off_required");
    start(false); // First hardware operation is ALWAYS OFF, never 20/20.
  }
  ~LaserBridge() { cancel(); }
  void step() {
    if (!connected_ || armed_ || !have_state_ || elapsed(last_state_)>s_.state_timeout) {
      if (desired_ || (known_ && ack_on_)) fault_=true;
      desired_=false; reason_="FCU_must_be_connected_and_disarmed";
    }
    if (elapsed(last_request_)>s_.lease && desired_) {
      desired_=false; fault_=true; reason_="request_timeout";
      ROS_ERROR("Detector heartbeat timed out; forcing OFF");
    }
    if (desired_ && elapsed(episode_start_)>=s_.max_on_seconds) {
      desired_=false; fault_=true; reason_="independent_maximum_exposure";
    }
    if (!desired_ && child_>0 && in_flight_on_) {
      cancel(); known_=false; startup_off_acked_=false; fault_=true;
      next_command_=Clock::now(); reason_="cancel_pending_on_for_off";
    }
    if (child_>0) poll();
    if (child_<=0 && Clock::now()>=next_command_) {
      bool want_on=desired_ && startup_off_acked_ && !fault_;
      bool refresh=!want_on && elapsed(last_off_ack_)>=s_.off_refresh;
      if (!known_ || ack_on_!=want_on || refresh || !startup_off_acked_) start(want_on);
    }
    if (elapsed(last_status_)>0.2) publish(reason_);
  }
  void shutdownOff() {
    desired_=false; cancel();
    ROS_WARN("Shutdown: attempting OFF while ROS transport is still alive");
    bool ack=false;
    for (int i=0;i<s_.attempts;++i) {
      start(false);
      while (child_>0) { poll(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
      ack=known_ && !ack_on_;
    }
    publish(ack ? "shutdown_off_acknowledged" : "shutdown_off_FAILED_hardware_state_unknown");
    if (!ack) ROS_FATAL("Shutdown OFF not acknowledged. Hardware state UNKNOWN; use physical power cutoff");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
 private:
  Settings s_;
  PwmBackend backend_;
  std::string executable_;
  ros::Publisher status_;
  ros::Subscriber sub_, state_sub_;
  bool connected_=false, armed_=false, have_state_=false;
  double last_capture_=0;
  Clock::time_point last_state_{}, episode_start_{};
  bool desired_=false, known_=false, ack_on_=false, startup_off_acked_=false;
  bool fault_=false, seen_false_=true, in_flight_on_=false;
  pid_t child_=-1;
  int pipe_=-1, result_code_=-1;
  Clock::time_point last_request_, last_off_ack_, started_, next_command_, last_status_{};
  std::string reason_="startup", error_;
  void state(const mavros_msgs::State::ConstPtr& msg) {
    connected_=msg->connected; armed_=msg->armed;
    have_state_=true; last_state_=Clock::now();
    if (!connected_ || armed_) {
      if (desired_ || (known_ && ack_on_)) fault_=true;
      desired_=false;
    }
  }
  void request(const robot_vision::LaserDecision::ConstPtr& msg) {
    // Check a gap BEFORE accepting the new request, even if spinOnce preceded step().
    if (elapsed(last_request_)>s_.lease && desired_) { desired_=false; fault_=true; }
    last_request_=Clock::now();
    if (!msg->enable_laser || !msg->request_on || !msg->observation_valid ||
        !msg->target_valid || !msg->aim_valid || !msg->calibration_valid) {
      if (desired_ || (known_ && ack_on_)) fault_=true; // one-shot, no auto re-light
      desired_=false; seen_false_=true;
      reason_=msg->reason;
      return;
    }
    const double capture=msg->header.stamp.toSec();
    const double now=ros::Time::now().toSec(), age=now-capture;
    const bool advances=capture>last_capture_;
    if (capture<=0 || now<=0 || !advances || age>s_.max_image_age || age < -s_.future_tolerance) {
      if (desired_ || (known_ && ack_on_)) fault_=true;
      desired_=false; reason_="stale_or_replayed_decision"; return;
    }
    last_capture_=capture;
    // No M-frame grace: every ON frame must pass three-ring and aim gates.
    const bool was_on=desired_;
    const bool stable=msg->hit_count>=static_cast<unsigned>(s_.on_frames) && msg->miss_count==0;
    desired_=stable && seen_false_ && startup_off_acked_ && !fault_ &&
             connected_ && !armed_ && have_state_ && elapsed(last_state_)<=s_.state_timeout;
    if (desired_ && !was_on) episode_start_=Clock::now();
    if (was_on && !desired_) fault_=true;
    reason_=desired_ ? "stable_target_request" : "off_request_or_interlock";
  }
  void publish(const std::string& why) {
    std_msgs::String m;
    std::ostringstream o;
    const std::string state=child_>0 ? (in_flight_on_ ? "ON_PENDING" : "OFF_PENDING") :
        (!known_ ? "UNKNOWN" : (ack_on_ ? "ON_ACKNOWLEDGED" : "OFF_ACKNOWLEDGED"));
    o<<"{\"dry_run\":false,\"requested_on\":"<<(desired_ ? "true":"false")
     <<",\"state\":\""<<state<<"\",\"hardware_light_verified\":false"
     <<",\"fault_latched\":"<<(fault_ ? "true":"false")
     <<",\"startup_off_acked\":"<<(startup_off_acked_ ? "true":"false")
     <<",\"fcu_connected\":"<<((connected_ && have_state_ && elapsed(last_state_)<=s_.state_timeout) ? "true":"false")
     <<",\"fcu_armed\":"<<(armed_ ? "true":"false")
     <<",\"mav_result\":"<<result_code_<<",\"laser_channel\":"<<s_.channel
     <<",\"reason\":\""<<escape(why)<<"\",\"error\":\""<<escape(error_)<<"\"}";
    m.data=o.str(); status_.publish(m); last_status_=Clock::now();
  }
  void fail(const std::string& why) {
    known_=false; desired_=false; startup_off_acked_=false;
    fault_=true; seen_false_=false; error_=why;
    next_command_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s_.retry));
    ROS_ERROR_STREAM("PWM command failed: "<<why<<". Hardware state UNKNOWN; retrying OFF only");
    publish("command_failed");
  }
  void start(bool on) {
    const int value=on ? s_.on_value:s_.off_value;
    const int p5=s_.channel==5 ? value:s_.other_value;
    const int p6=s_.channel==6 ? value:s_.other_value;
    auto cmd=backend_.build(p5,p6);
    std::ostringstream data;
    data<<std::setprecision(9)<<0<<' '<<cmd.request.command<<' '<<0<<' '
        <<cmd.request.param1<<' '<<cmd.request.param2<<' '<<cmd.request.param3<<' '
        <<cmd.request.param4<<' '<<cmd.request.param5<<' '<<cmd.request.param6<<' '<<cmd.request.param7;
    const std::string payload=data.str();
    // Do not leak inherited ROS sockets into exec'ed workers. Build the fd list
    // BEFORE fork (other threads exist), then child uses async-signal-safe calls.
    struct rlimit limit;
    long fd_limit=1024;
    if (getrlimit(RLIMIT_NOFILE,&limit)==0) fd_limit=static_cast<long>(limit.rlim_cur);
    // open descriptors are listed using /proc, rather than a million close calls.
    std::vector<int> inherited;
    DIR* directory=opendir("/proc/self/fd");
    if (!directory) { fail("cannot enumerate inherited file descriptors"); return; }
    const int directory_fd=dirfd(directory);
    while (dirent* entry=readdir(directory)) {
      char* end=nullptr;
      const long fd=strtol(entry->d_name,&end,10);
      if (end && *end=='\0' && fd>2 && fd!=directory_fd && fd<fd_limit) inherited.push_back(static_cast<int>(fd));
    }
    closedir(directory);
    int pipes[2];
    if (::pipe(pipes)<0) { fail("pipe creation failed"); return; }
    const pid_t parent_pid=getpid();
    pid_t pid=fork();
    if (pid==0) {
      // Linux: stop an unsent/in-flight worker when its supervising process dies.
      // Cannot undo a command already delivered to the FCU.
      if (prctl(PR_SET_PDEATHSIG,SIGKILL)<0 || getppid()!=parent_pid) _exit(125);
      close(pipes[0]);
      // Descriptor 3 is private to the exec'ed service-call worker.
      if (dup2(pipes[1],3)<0) _exit(126);
      if (pipes[1]!=3) close(pipes[1]);
      for (int fd : inherited) if (fd!=3) close(fd);
      execl(executable_.c_str(),executable_.c_str(),"--command-worker",s_.service.c_str(),payload.c_str(),"3",static_cast<char*>(nullptr));
      _exit(127);
    }
    close(pipes[1]);
    if (pid<0) { close(pipes[0]); fail("fork failed"); return; }
    child_=pid; pipe_=pipes[0]; in_flight_on_=on; started_=Clock::now();
    fcntl(pipe_,F_SETFL,fcntl(pipe_,F_GETFL)|O_NONBLOCK);
    ROS_WARN_STREAM("Sending "<<(on ? "ON":"OFF")<<": auxiliary5="<<p5<<", auxiliary6="<<p6
                    <<", library command="<<cmd.request.command<<" (ACK is NOT physical light proof)");
    publish(on ? "sending_on":"sending_off");
  }
  void cancel() {
    if (child_>0) {
      kill(child_,SIGKILL);
      int status;
      while (waitpid(child_,&status,0)<0 && errno==EINTR) {}
      child_=-1;
    }
    if (pipe_>=0) { close(pipe_); pipe_=-1; }
  }
  void poll() {
    CallResult result{};
    ssize_t n=read(pipe_,&result,sizeof(result));
    if (n==static_cast<ssize_t>(sizeof(result)) && result.magic==RESULT_MAGIC) {
      const bool on=in_flight_on_;
      cancel(); result_code_=result.result;
      if (!result.transport || !result.success || result.result!=0) {
        std::ostringstream text; text<<"transport="<<result.transport<<", response.success="<<result.success<<", response.result="<<result.result;
        fail(text.str()); return;
      }
      known_=true; ack_on_=on; error_.clear();
      if (!on) {
        startup_off_acked_=true; last_off_ack_=Clock::now();
        // Fault stays latched until inspected, explicit process restart.
      }
      next_command_=Clock::now();
      publish(on ? "on_command_acknowledged":"off_command_acknowledged");
      return;
    }
    if (elapsed(started_)>s_.call_timeout) {
      cancel(); fail("service_call_timeout; pending hardware command outcome unknown"); return;
    }
    int status=0;
    pid_t done=waitpid(child_,&status,WNOHANG);
    if (done==child_) {
      child_=-1;
      // Child may write then exit between read and waitpid; drain once more.
      n=read(pipe_,&result,sizeof(result));
      close(pipe_); pipe_=-1;
      if (n==static_cast<ssize_t>(sizeof(result)) && result.magic==RESULT_MAGIC &&
          result.transport && result.success && result.result==0) {
        known_=true; ack_on_=in_flight_on_; result_code_=0; error_.clear();
        if (!ack_on_) { startup_off_acked_=true; last_off_ack_=Clock::now(); }
        next_command_=Clock::now(); publish("command_acknowledged");
      } else { result_code_=n==sizeof(result) ? result.result:-1; fail("command worker exited without accepted ACK"); }
    }
  }
};

static volatile sig_atomic_t stopping=0;
static void stopSignal(int) { stopping=1; }
int main(int argc,char** argv) {
  if (argc>1 && std::string(argv[1])=="--command-worker") return worker(argc,argv);
  ros::init(argc,argv,"circle_laser_bridge",ros::init_options::NoSigintHandler);
  signal(SIGINT,stopSignal); signal(SIGTERM,stopSignal);
  ros::NodeHandle n,p("~");
  std::unique_ptr<LaserBridge> bridge;
  int code=0;
  try {
    Settings cfg=settings(p);
    char self[4096]; ssize_t len=readlink("/proc/self/exe",self,sizeof(self)-1);
    if (len<=0) throw std::runtime_error("Cannot resolve bridge executable for command worker");
    self[len]='\0';
    bridge.reset(new LaserBridge(n,cfg,self));
    while (ros::ok() && !stopping) {
      ros::spinOnce(); bridge->step();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Laser bridge refused/stopped: "<<e.what()); code=1;
  } catch (...) {
    ROS_FATAL("Laser bridge stopped by unknown exception"); code=1;
  }
  if (bridge) {
    try { bridge->shutdownOff(); }
    catch (const std::exception& e) { ROS_FATAL_STREAM("Shutdown OFF failed: "<<e.what()); code=1; }
  }
  ros::shutdown();
  return code;
}
