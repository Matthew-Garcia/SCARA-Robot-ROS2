#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace scara_kinematics {
constexpr double pi = 3.14159265358979323846;
using Joints = std::array<double, 4>;  // shoulder [rad], slide [m], elbow [rad], wrist [rad]
struct Geometry { double l1, l2, z0; };
struct Pose { double x, y, z, yaw; };
struct Limits { Joints lower, upper; };
inline Pose forward(const Geometry& g, const Joints& q) {
  return {g.l1*std::cos(q[0])+g.l2*std::cos(q[0]+q[2]),
          g.l1*std::sin(q[0])+g.l2*std::sin(q[0]+q[2]),g.z0+q[1],q[0]+q[2]+q[3]};
}
inline bool valid(const Joints& q, const Limits& lim) {
  for (size_t i=0;i<4;++i)
    if (!std::isfinite(q[i]) || q[i]<lim.lower[i]-1e-9 || q[i]>lim.upper[i]+1e-9) return false;
  return true;
}
inline std::vector<Joints> inverse(const Geometry& g, const Pose& p, const Joints& seed, const Limits& lim) {
  std::vector<Joints> out;
  if (!(g.l1>0 && g.l2>0) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !std::isfinite(p.yaw)) return out;
  for(double s:seed) if(!std::isfinite(s)) return out;
  double c=(p.x*p.x+p.y*p.y-g.l1*g.l1-g.l2*g.l2)/(2*g.l1*g.l2);
  if(c < -1.-1e-10 || c > 1.+1e-10) return out;
  double e=std::acos(std::clamp(c,-1.,1.));
  for(double elbow:{e,-e}) {
    double shoulder=std::atan2(p.y,p.x)-std::atan2(g.l2*std::sin(elbow),g.l1+g.l2*std::cos(elbow));
    // Enumerate equivalent angles within bounded mechanical travel, not just [-pi,pi].
    for(int k=-2;k<=2;++k) for(int m=-2;m<=2;++m) {
      double a=shoulder+2*pi*k;
      Joints q{a,p.z-g.z0,elbow,p.yaw-a-elbow+2*pi*m};
      if(valid(q,lim) && std::none_of(out.begin(),out.end(),[&](const Joints& other){
        double d=0;for(size_t i=0;i<4;++i)d+=std::abs(other[i]-q[i]);return d<1e-9;
      })) out.push_back(q);
    }
  }
  auto cost=[&](const Joints& q){double d=0;for(size_t i=0;i<4;++i){double v=(q[i]-seed[i])*(i==1?10.:1.);d+=v*v;}return d;};
  std::stable_sort(out.begin(),out.end(),[&](const Joints&a,const Joints&b){return cost(a)<cost(b);});
  return out;
}
}  // namespace scara_kinematics
