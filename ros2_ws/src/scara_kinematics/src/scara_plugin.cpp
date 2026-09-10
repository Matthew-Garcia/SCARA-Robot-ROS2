#include <scara_kinematics/analytic.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Geometry>

namespace scara_kinematics {
class ScaraPlugin : public kinematics::KinematicsBase {
  const moveit::core::RobotModel* model_=nullptr;
  const moveit::core::JointModelGroup* group_=nullptr;
  std::vector<std::string> joints_,links_;
  Geometry geometry_{};
  Limits limits_{};
  using MsgPose=geometry_msgs::msg::Pose;
  using Error=moveit_msgs::msg::MoveItErrorCodes;
  using Options=kinematics::KinematicsQueryOptions;
  bool solve(const MsgPose& pose,const std::vector<double>& seed,const std::vector<double>& consistency,
             std::vector<double>& solution,const IKCallbackFn& cb,Error& error) const {
    error.val=Error::NO_IK_SOLUTION;
    if(!model_ || seed.size()!=4 || (!consistency.empty() && consistency.size()!=4)) return false;
    for(double v:consistency) if(!std::isfinite(v) || v<0) return false;
    Eigen::Quaterniond quat(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    if(!quat.coeffs().allFinite() || quat.norm()<1e-12) return false;
    quat.normalize();const auto R=quat.toRotationMatrix();
    // This mechanism cannot tilt the TCP. Reject roll/pitch instead of silently dropping them.
    if((R.col(2)-Eigen::Vector3d::UnitZ()).norm()>1e-5) return false;
    Joints s;std::copy(seed.begin(),seed.end(),s.begin());
    Pose p{pose.position.x,pose.position.y,pose.position.z,std::atan2(R(1,0),R(0,0))};
    for(const auto& q:inverse(geometry_,p,s,limits_)) {
      bool close=true;
      if(!consistency.empty()) for(size_t i=0;i<4;++i) if(std::abs(q[i]-s[i])>consistency[i]) close=false;
      if(!close) continue;
      std::vector<double> candidate(q.begin(),q.end());error.val=Error::SUCCESS;
      if(cb) cb(pose,candidate,error);
      if(error.val==Error::SUCCESS){solution=candidate;return true;}
    }
    error.val=Error::NO_IK_SOLUTION;return false;
  }
 public:
  bool initialize(const rclcpp::Node::SharedPtr&,const moveit::core::RobotModel& model,
                  const std::string& group,const std::string& base,const std::vector<std::string>& tips,double discretization) override {
    if(base!="base_link" || tips!=std::vector<std::string>{"tcp_link"}) return false;
    group_=model.getJointModelGroup(group);if(!group_) return false;
    joints_={"shoulder_joint","z_joint","elbow_joint","wrist_joint"};
    if(group_->getVariableNames()!=joints_) return false;
    setValues("robot_description",group,base,tips,discretization);
    model_=&model;links_=group_->getLinkModelNames();
    const auto translation=[&](const std::string& link){return model.getLinkModel(link)->getJointOriginTransform().translation().eval();};
    geometry_={translation("arm2_link").x(),translation("tool_link").x(),
       translation("shoulder_link").z()+translation("arm1_link").z()+translation("arm2_link").z()+
       translation("tool_link").z()+translation("tcp_link").z()};
    for(size_t i=0;i<4;++i){const auto& b=model.getVariableBounds(joints_[i]);limits_.lower[i]=b.min_position_;limits_.upper[i]=b.max_position_;}
    return geometry_.l1>0 && geometry_.l2>0;
  }
  const std::vector<std::string>& getJointNames() const override{return joints_;}
  const std::vector<std::string>& getLinkNames() const override{return links_;}
  bool getPositionIK(const MsgPose&p,const std::vector<double>&s,std::vector<double>&q,Error&e,const Options&) const override {
    return solve(p,s,{},q,{},e);
  }
  bool searchPositionIK(const MsgPose&p,const std::vector<double>&s,double,std::vector<double>&q,Error&e,const Options&) const override {
    return solve(p,s,{},q,{},e);
  }
  bool searchPositionIK(const MsgPose&p,const std::vector<double>&s,double,const std::vector<double>&c,std::vector<double>&q,Error&e,const Options&) const override {
    return solve(p,s,c,q,{},e);
  }
  bool searchPositionIK(const MsgPose&p,const std::vector<double>&s,double,std::vector<double>&q,const IKCallbackFn&cb,Error&e,const Options&) const override {
    return solve(p,s,{},q,cb,e);
  }
  bool searchPositionIK(const MsgPose&p,const std::vector<double>&s,double,const std::vector<double>&c,std::vector<double>&q,const IKCallbackFn&cb,Error&e,const Options&) const override {
    return solve(p,s,c,q,cb,e);
  }
  bool getPositionFK(const std::vector<std::string>& names,const std::vector<double>& q,std::vector<MsgPose>& poses) const override {
    if(!model_ || q.size()!=4) return false;
    for(double v:q)if(!std::isfinite(v))return false;
    moveit::core::RobotState state(moveit::core::RobotModelConstPtr(model_, [](const moveit::core::RobotModel*) {}));state.setToDefaultValues();state.setJointGroupPositions(group_,q);state.update();
    poses.clear();
    const Eigen::Isometry3d base_inverse=state.getGlobalLinkTransform("base_link").inverse();
    for(const auto& name:names){
      if(!model_->hasLinkModel(name)) return false;
      const Eigen::Isometry3d T=base_inverse*state.getGlobalLinkTransform(name);Eigen::Quaterniond r(T.rotation());
      MsgPose p;p.position.x=T.translation().x();p.position.y=T.translation().y();p.position.z=T.translation().z();
      p.orientation.x=r.x();p.orientation.y=r.y();p.orientation.z=r.z();p.orientation.w=r.w();poses.push_back(p);
    }return true;
  }
};
}
PLUGINLIB_EXPORT_CLASS(scara_kinematics::ScaraPlugin,kinematics::KinematicsBase)
