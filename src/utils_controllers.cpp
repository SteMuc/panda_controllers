#include <panda_controllers/utils_controllers.h>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_controllers
{
  bool MyReadOnlyController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_node_handle)
  {
    auto fsi = robot_hw->get<franka_hw::FrankaStateInterface>();
    auto fmi = robot_hw->get<franka_hw::FrankaModelInterface>();

    // Error handling omitted for brevity
    fsh = std::make_unique<franka_hw::FrankaStateHandle>(fsi->getHandle("panda_robot"));
    fmh = std::make_unique<franka_hw::FrankaModelHandle>(fmi->getHandle("panda_model"));

    // Offer the functionality you need to the outside world, e.g. via a service:
    service =
        root_node_handle.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
            "my_model_function", [&](auto &request, auto &response)
            {
              // Do something here with the model library, e.g. getting the mass matrix.
              // Probably you also want your own service type
              auto M = fmh->getMass();
              response.message = "M[0,0] = " + std::to_string(M.at(0));
              response.success = true;
              return true; });

    return true;
  }

  void MyReadOnlyController::update(const ros::Time &time, const ros::Duration &period) 
  {
    // This is the realtime loop of your controller. Since its read-only, you can simply do nothing
    // here. Note though, that you want to avoid blocking calls here, since this can trigger
    // communication_constraints_reflex if the update loop is too slow
  }
}
PLUGINLIB_EXPORT_CLASS(panda_controllers::MyReadOnlyController,
                       controller_interface::ControllerBase)