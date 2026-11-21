#include <ignition/msgs/image.pb.h>
#include <ignition/transport/Node.hh>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <functional>
using namespace ignition;
using Clock = std::chrono::system_clock;
static std::ofstream out; static std::mutex mu; static bool ready=false; static long n_rgb=0,n_depth=0; static std::string ready_path;
static std::string esc(std::string s){ std::string r; for(char c:s){if(c=='"'||c=='\\')r+='\\'; r+=c;} return r; }
void cb(const msgs::Image &m, const std::string stream){ auto h=m.header(); long n=(stream=="gz_rgb"?++n_rgb:++n_depth); std::string frame="",seq="null"; for(int i=0;i<h.data_size();++i){auto d=h.data(i); if(d.key()=="frame_id"&&d.value_size())frame=d.value(0); if(d.key()=="seq"&&d.value_size())seq=d.value(0);} long long ns=(long long)h.stamp().sec()*1000000000LL+h.stamp().nsec(); auto w=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); std::lock_guard<std::mutex> l(mu); out<<"{\"stream\":\""<<stream<<"\",\"count\":"<<n<<",\"stamp_sec\":"<<h.stamp().sec()<<",\"stamp_nsec\":"<<h.stamp().nsec()<<",\"stamp_ns\":"<<ns<<",\"seq\":"<<seq<<",\"seq_reason\":\"header.data seq or null\",\"frame_id\":\""<<esc(frame)<<"\",\"recv_wall_ns\":"<<w<<"}\n"; out.flush(); if(!ready&&n_rgb>=1&&n_depth>=1){std::ofstream(ready_path)<<"READY after first RGB and depth JSONL rows\\n"; ready=true;} }
int main(int argc,char**argv){if(argc!=4)return 2; out.open(argv[1]); ready_path=argv[2]; transport::Node node; auto rgb=std::function<void(const msgs::Image&)>([](const msgs::Image&m){cb(m,"gz_rgb");}); auto dep=std::function<void(const msgs::Image&)>([](const msgs::Image&m){cb(m,"gz_depth");}); if(!node.Subscribe<msgs::Image>("/rgbd_camera/image",rgb)||!node.Subscribe<msgs::Image>("/rgbd_camera/depth_image",dep))return 3; std::this_thread::sleep_for(std::chrono::seconds(std::stoi(argv[3]))); }
