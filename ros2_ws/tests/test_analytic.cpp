#include <scara_kinematics/analytic.hpp>
#include <cassert>
#include <random>
#include <iostream>
#include <limits>
using namespace scara_kinematics;
int main(){
 Geometry g{.228,.1365,.07255508189967187};
 Limits lim{{-pi/2,-.05,-150*pi/180,-162*pi/180},{266*pi/180,.05,150*pi/180,162*pi/180}};
 std::mt19937 random(42);std::uniform_real_distribution<double> unit(0,1);
 for(int i=0;i<10000;++i){
  Joints q;for(int j=0;j<4;++j)q[j]=lim.lower[j]+unit(random)*(lim.upper[j]-lim.lower[j]);
  auto p=forward(g,q);auto solutions=inverse(g,p,q,lim);assert(!solutions.empty());
  for(auto result:solutions){assert(valid(result,lim));auto f=forward(g,result);
   assert(std::hypot(f.x-p.x,f.y-p.y)<1e-8);assert(std::abs(f.z-p.z)<1e-8);
   assert(std::abs(std::remainder(f.yaw-p.yaw,2*pi))<1e-8);
  }
  for(int j=0;j<4;++j)assert(std::abs(solutions.front()[j]-q[j])<1e-7);
 }
 Joints seed{0,0,0,0};
 assert(inverse(g,{1,0,g.z0,0},seed,lim).empty());
 assert(inverse(g,{0,0,g.z0,0},seed,lim).empty());
 assert(inverse(g,{.3,0,g.z0+.1,0},seed,lim).empty());
 assert(inverse(g,{std::numeric_limits<double>::quiet_NaN(),0,g.z0,0},seed,lim).empty());
 assert(!inverse(g,{g.l1+g.l2,0,g.z0,0},seed,lim).empty());
 auto branches=inverse(g,{.3,0,g.z0,0},seed,lim);
 assert(std::any_of(branches.begin(),branches.end(),[](auto q){return q[2]>0;}));
 assert(std::any_of(branches.begin(),branches.end(),[](auto q){return q[2]<0;}));
 std::cout<<"10,000 FK/IK round trips, angle wraps, branches, singularity and invalid-target tests passed.\n";
}
