#include "cargo_intake_controller/cargo_intake_controller.h"

namespace cargo_intake_controller
{

bool CargoIntakeController::init(hardware_interface::RobotHW *hw,
                                                        ros::NodeHandle                 &root_nh,
                                                        ros::NodeHandle                 &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();
	
    //read cargo intake actuator name from config file
    XmlRpc::XmlRpcValue cargo_intake_actuator_params;
    if (!controller_nh.getParam("cargo_intake_actuator_joint", cargo_intake_actuator_params))
    {
        ROS_ERROR_STREAM("Can not read cargo intake actuator name");
        return false;
    }

    //read cargo intake name from config file
    XmlRpc::XmlRpcValue cargo_intake_params;
    if (!controller_nh.getParam("cargo_intake_joint", cargo_intake_params))
    {
        ROS_ERROR_STREAM("Can not read cargo intake name");
        return false;
    }

    //initialize cargo actuator
    if (!cargo_intake_actuator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, cargo_intake_actuator_params))
    {
        ROS_ERROR("Cannot initialize cargo intake actuator joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized cargo intake actuator joint!");
    }

    //initialize cargo intake joint
    if (!cargo_intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, cargo_intake_params))
    {
        ROS_ERROR("Cannot initialize cargo intake joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized cargo intake joint!");
    }


    cargo_intake_service_ = controller_nh.advertiseService("cargo_intake_command", &CargoIntakeController::cmdService, this);
	
	return true;
}

void CargoIntakeController::starting(const ros::Time &/*time*/) {
    cargo_intake_joint_.setCommand(0.0); // set the command to the spinny part of the intake
	cargo_actuator_setCommand(-1); // set the command to the up/down part of the intake
}

void CargoIntakeController::update(const ros::Time &time, const ros::Duration &period) {
	//  cargo_intake_joint_.setCommand( * spin_command_.readFromRT() ); // set the command to the spinny intake bar
	double spin_cmd = *(spin_cmd_.readFromRT());
	bool intake_in_cmd = *(intake_in_cmd_.readFromRT());
   	double intake_in_cmd_double;
	if(intake_in_cmd == true) {
		ROS_WARN("intake in");
		intake_in_cmd_double = -1;
	}
	else if (intake_in_cmd == false) {
		intake_in_cmd_double = 1;
		ROS_WARN("intake out");
	}

	ROS_INFO_STREAM("spin command = " << spin_command << "; intake_in = " << intake_in_cmd);
	intake_joint_.setCommand(spin_command); // set the command to the spinny part of the intake
	intake_in_.setCommand(intake_in_cmd_double); // set the in/out command to the up/down part of the intake
	intake_in_.setCommand(intake_in_cmd_double); // set the in/out command to the up/down part of the intake
}

void CargoIntakeController::stopping(const ros::Time &time) {
}

bool CargoIntakeController::cmdService(cargo_intake_controller::CargoIntakeSrv::Request &req, cargo_intake_controller::CargoIntakeSrv::Response &res) {
    if(isRunning())
    {
        spin_command_.writeFromNonRT(req.power); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
		intake_in_cmd_.writeFromNonRT(req.intake_in); //take the service request for in/out (true/false???) and write to a command variable
    }
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. CargoIntakeController is not running.");
        return false;
    }
    return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(cargo_intake_controller::CargoIntakeController, controller_interface::ControllerBase)
