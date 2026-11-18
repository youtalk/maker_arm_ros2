// Pilz rejects an empty planner id ("planner ID '' does not exist"); OMPL resolves it through its
// own default_planner_config. This request adapter gives every pipeline that behaviour: an empty
// planner id becomes the pipeline's `default_planner_config` parameter.
#include <moveit/planning_interface/planning_request_adapter.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace maker_arm_moveit_config
{
class DefaultPlannerId : public planning_interface::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& ns) override
  {
    const std::string name = ns + (ns.empty() || ns.back() == '.' ? "" : ".") + "default_planner_config";
    if (!node->has_parameter(name))
      node->declare_parameter(name, std::string());  // empty: leave the id alone
    node->get_parameter(name, planner_);
  }

  std::string getDescription() const override { return "Default Planner Id"; }

  moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr&,
                                      planning_interface::MotionPlanRequest& req) const override
  {
    if (req.planner_id.empty())
      req.planner_id = planner_;
    return moveit::core::MoveItErrorCode::SUCCESS;
  }

private:
  std::string planner_;
};
}  // namespace maker_arm_moveit_config

PLUGINLIB_EXPORT_CLASS(maker_arm_moveit_config::DefaultPlannerId, planning_interface::PlanningRequestAdapter)
