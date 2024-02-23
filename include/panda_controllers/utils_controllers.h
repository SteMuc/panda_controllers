#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace panda_controllers
{
    class MyReadOnlyController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                       franka_hw::FrankaStateInterface>
    {

    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle& root_node_handle);
        void update(const ros::Time &, const ros::Duration &period);

    private:

        std::unique_ptr<franka_hw::FrankaStateHandle> fsh;
        std::unique_ptr<franka_hw::FrankaModelHandle> fmh;

        ros::ServiceServer service;
    };

}