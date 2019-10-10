#pragma once
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/enumerated_elevator_indices.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

extern bool debug;

class AlignActionAxisConfig
{
	public:
		AlignActionAxisConfig(const std::string &name,
							  const std::string &enable_pub_topic,
							  const std::string &error_sub_topic,
							  const std::string &timeout_param,
							  const std::string &error_threshold_param)
			: name_(name)
			, enable_pub_topic_(enable_pub_topic)
			, error_sub_topic_(error_sub_topic)
			, timeout_param_(timeout_param)
			, error_threshold_param_(error_threshold_param)
		{
		}
		std::string name_;
		std::string enable_pub_topic_;
		std::string error_sub_topic_;
		std::string timeout_param_;
		std::string error_threshold_param_;
};

class AlignActionAxisState
{
	public:
		AlignActionAxisState(const std::string &name,
					  ros::NodeHandle &nh,
				const std::string &enable_pub_topic,
				const std::string &error_sub_topic,
				//void(*error_sub_cb)(const std_msgs::Float64MultiArray &msg),
				const boost::function<void(const std_msgs::Float64MultiArrayConstPtr &msg)> &error_sub_cb,
				const double timeout,
				const double error_threshold)
			: enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
			, error_sub_(nh.subscribe(error_sub_topic, 1, error_sub_cb))
			, timeout_(timeout)
			, error_threshold_(error_threshold)
		{
			aligned_   = false;
			error_     = false;
			timed_out_ = false;
		}
		ros::Publisher enable_pub_;
		ros::Subscriber error_sub_;
		bool aligned_;
		double error_;
		double timeout_;
		double error_threshold_;
		bool timed_out_;
};

//bool startup = true; //disable all pid nodes on startup
class BaseAlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		// TODO this result should be a local var
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_; //Action client for controlling the elevato


		//Publishers for enabling PID loops and cmd_vel combiner
		ros::Publisher enable_align_pub_;	//Enables the cmd_vel combiner
#if 0
		ros::Publisher enable_orient_pub_;	//Enables orient pid
		ros::Publisher enable_x_pub_;		//Enables x-direction pid
		ros::Publisher enable_y_pub_;		//Enables y-direction pid

		//Subscribers for PID error
		ros::Subscriber orient_error_sub_;	//orient error
		ros::Subscriber x_error_sub_;		//x error
		ros::Subscriber y_error_sub_;		//y error
#endif

		//Service client for forcefully stopping the robot
        ros::ServiceClient BrakeSrv_;

		std::map<std::string, AlignActionAxisState> axis_states_;
		//Set default state for align variables
		//
		//Whether we are aligned variables
		bool aligned_ = false;				//overall aligned
#if 0
		bool orient_aligned_ = false;		//orient is aligned
		bool x_aligned_ = false;			//x is aligned
		bool y_aligned_ = false;			//y is aligned

		//Error values
		double orient_error_ = 0.0;			//orient error
		double x_error_ = 0.0;				//x error
		double y_error_ = 0.0;				//y error

		//Whether axes are timed out
		bool orient_timed_out_ = false;		//orient timed out
		bool x_timed_out_ = false;			//x timed out
		bool y_timed_out_ = false;			//y timed out
#endif

		//timeouts for overall align and specific axes
		double align_timeout_ = 0.0;		//overall align timeout
#if 0
		double orient_timeout_ = 0.0;		//orient timeout
		double x_timeout_ = 0.0;			//x timeout
		double y_timeout_ = 0.0;			//y timeout

		//maximum error to be aligned
		double orient_error_threshold_ = 0.0;	//max orient error
		double x_error_threshold_ = 0.0;		//max x error
		double y_error_threshold_ = 0.0;		//max y error
#endif

		//is align preempted
		bool preempted_ = false;

		//start time of align
		double start_time_ = -1.0;

	public:
		//make the executeCB function run every time the actionlib server is called
		BaseAlignAction(const std::string &name,

						//Topics that enable align PID loops
						const std::string &enable_align_topic_, // global - turns on / off the publish_pid_cmd_vel node's output
#if 0
						const std::string &enable_orient_topic_,
						const std::string &enable_x_topic_,
						const std::string &enable_y_topic_,

						//Topics with error values
						const std::string &orient_error_topic_,
						const std::string &x_error_topic_,
						const std::string &y_error_topic_,
#endif

						//Paramater name with align timeouts
						const std::string &align_timeout_param_name_ // global timeout for entire server CB
#if 0
						const std::string &orient_timeout_param_name_,
						const std::string &x_timeout_param_name_,
						const std::string &y_timeout_param_name_,

						//Paramater name with error thresholds
						const std::string &orient_error_threshold_param_name_,
						const std::string &x_error_threshold_param_name_,
						const std::string &y_error_threshold_param_name_
#endif
							)

			: as_(nh_, name, boost::bind(&BaseAlignAction::executeCB, this, _1), false)
			, action_name_(name)
			, ac_elevator_("/elevator/elevator_server", true)		//TODO maybe make this a generic part of the robot

			//Create publishers on the enable PID loop topics
			, enable_align_pub_(nh_.advertise<std_msgs::Bool>(enable_align_topic_, 1, true))
#if 0
			, enable_orient_pub_(nh_.advertise<std_msgs::Bool>(enable_orient_topic_, 1, true))
			, enable_x_pub_(nh_.advertise<std_msgs::Bool>(enable_x_topic_, 1, true))
			, enable_y_pub_(nh_.advertise<std_msgs::Bool>(enable_y_topic_, 1, true))

			//Create subscribers on the error topics
			, orient_error_sub_(nh_.subscribe(orient_error_topic_, 1, &BaseAlignAction::orient_error_cb, this))
			, x_error_sub_(nh_.subscribe(x_error_topic_, 1, &BaseAlignAction::x_error_cb, this))
			, y_error_sub_(nh_.subscribe(y_error_topic_, 1, &BaseAlignAction::y_error_cb, this))
#endif
		{
            as_.start();

            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";

			//Create service client for abruptly stopping the robot
			BrakeSrv_ = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);

			//Get the timeout parameters
			if(!nh_.getParam(align_timeout_param_name_, align_timeout_))
				ROS_ERROR_STREAM("Could not read align_timeout_param_name_ in align_server");
#if 0
			if(!nh_.getParam(orient_timeout_param_name_, orient_timeout_))
				ROS_ERROR_STREAM("Could not read orient_timeout_param_name_ in align_server");
			if(!nh_.getParam(x_timeout_param_name_, x_timeout_))
				ROS_ERROR_STREAM("Could not read x_timeout_param_name_ in align_server");
			if(!nh_.getParam(y_timeout_param_name_, y_timeout_))
				ROS_ERROR_STREAM("Could not read y_timeout_param_name_ in align_server");

			//Get the error parameters
			if(!nh_.getParam(orient_error_threshold_param_name_, orient_error_threshold_))
				ROS_ERROR_STREAM("Could not read orient_error_threshold_param_name_ in align_server");
			if(!nh_.getParam(x_error_threshold_param_name_, x_error_threshold_))
				ROS_ERROR_STREAM("Could not read x_error_threshold_param_name_ in align_server");
			if(!nh_.getParam(y_error_threshold_param_name_, y_error_threshold_))
				ROS_ERROR_STREAM("Could not read y_error_threshold_param_name_ in align_server");
#endif
		}

		~BaseAlignAction(void)
		{
		}

		bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			double timeout;
			if(!nh_.getParam(axis_config.timeout_param_, timeout))
			{
				ROS_ERROR_STREAM("Could not read param "
						<< axis_config.timeout_param_
						<< " in align_server");
				return false;
			}
			double error_threshold;
			if(!nh_.getParam(axis_config.error_threshold_param_, error_threshold))
			{
				ROS_ERROR_STREAM("Could not read param "
						<< axis_config.error_threshold_param_
						<< " in align_server");
				return false;
			}

			axis_states_.emplace(std::make_pair(axis_config.name_,
				AlignActionAxisState(axis_config.name_,
						nh_,
						axis_config.enable_pub_topic_,
						axis_config.error_sub_topic_,
						boost::bind(&BaseAlignAction::error_term_cb, this, _1, axis_config.name_),
						timeout,
						error_threshold)));
			return true;
		}

#if 0 // TODO - not used
		bool get_axis_timed_out(const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in get_axis_timed_out");
				return false;
			}
			return axis_it->second.timed_out_;
		}
#endif
		double get_axis_error(const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in get_axis_error");
				return std::numeric_limits<double>::max();
			}
			return axis_it->second.error_;
		}

		//functions to check for preempts and timeouts
		bool check_timeout(double start_time, double timeout) {
			if(ros::Time::now().toSec() - start_time > timeout) {
				return true;
			}
			return false;
		}
		bool check_preempted() {
			if(preempted_ || as_.isPreemptRequested()) {
				return true;
			}
			return false;
		}

		//Function to dynamically load a new set of PID on a ros PID node
		//TODO make this work
		virtual void load_new_pid(std::string reconfigure_topic, double p_, double i_, double d_) {
			dynamic_reconfigure::ReconfigureRequest srv_req;
			dynamic_reconfigure::ReconfigureResponse srv_resp;
			dynamic_reconfigure::Config conf;

			dynamic_reconfigure::DoubleParameter p;
			dynamic_reconfigure::DoubleParameter i;
			dynamic_reconfigure::DoubleParameter d;
			dynamic_reconfigure::DoubleParameter p_scale;
			dynamic_reconfigure::DoubleParameter i_scale;
			dynamic_reconfigure::DoubleParameter d_scale;

			p.name = "Kp";
			p.value = p_;
			i.name = "Ki";
			i.value = i_;
			d.name = "Kd";
			d.value = d_;

			p_scale.name = "Kp_scale";
			p_scale.value = 100.0;
			i_scale.name = "Ki_scale";
			i_scale.value = 100.0;
			d_scale.name = "Kd_scale";
			d_scale.value = 100.0;

			conf.doubles.push_back(p_scale);
			conf.doubles.push_back(p);
			conf.doubles.push_back(i_scale);
			conf.doubles.push_back(i);
			conf.doubles.push_back(d_scale);
			conf.doubles.push_back(d);

			srv_req.config = conf;
			ros::service::call(reconfigure_topic, srv_req, srv_resp);
		}

		virtual void error_term_cb(const std_msgs::Float64MultiArrayConstPtr &msg,
			const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in error_term_cb");
				return;
			}
			auto &axis = axis_it->second;
			//Check if error less than threshold
			axis.aligned_ = (fabs(msg->data[0]) < axis.error_threshold_);
			axis.error_ = msg->data[0];
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, name << " error: " << axis.error_ << " aligned: " << axis.aligned_);
		}

#if 0
		//Default error callbacks for pid node
		virtual void orient_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			//Check if error less than threshold
			orient_aligned_ = (fabs(msg.data[0]) < orient_error_threshold_);
			orient_error_ =msg.data[0];
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "orient error: " << fabs(msg.data[0]));
		}
		virtual void x_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			//Check if error less than threshold
			x_aligned_ = (fabs(msg.data[0]) < x_error_threshold_);
			x_error_ = msg.data[0];
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "x error: " << fabs(msg.data[0]));
		}
		virtual void y_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			//Check if error less than threshold
			y_aligned_ = (fabs(msg.data[0]) < y_error_threshold_);
			y_error_ = msg.data[0];
			if(debug)
				ROS_WARN_STREAM_THROTTLE(1, "y error: " << fabs(msg.data[0]));
		}
#endif

		//Functions to enable align PID
		virtual void enable_align(bool enable=true) {
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_align_pub_.publish(enable_msg);
		}

		virtual void do_align(const std::string &name, ros::Rate &r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running " << name << " do_align");
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.cend())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in do_align");
				return;
			}
			auto &axis = axis_it->second;
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			axis.enable_pub_.publish(enable_msg);
			if (debug)
				ROS_INFO_STREAM("do_align(" << name  << ") : set enable_pub = " << enable);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !axis.aligned_ && !preempted_ && !axis.timed_out_) {
					enable_align(); // TODO - move outside loop?
					axis.timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					if (debug)
						ROS_INFO_STREAM("do_align(" << name
								<< ") : axis.aligned_ = " << axis.aligned_
								<< " axis.timed_out_ = " << axis.timed_out_
								<< " start_time_ = " << start_time_
								<< " timeout = " << timeout
								<< " reempted_ = " << preempted_);
					if (!axis.timed_out_ && !preempted_)
						r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				axis.enable_pub_.publish(enable_msg);
			}
		}
#if 0

		virtual void align_orient(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running align_orient");
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_orient_pub_.publish(enable_msg);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !orient_aligned_ && !preempted_ && !orient_timed_out_) {
					enable_align();
					orient_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_orient_pub_.publish(enable_msg);
			}
		}
		virtual void align_x(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_x_pub_.publish(enable_msg);

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !x_aligned_ && !preempted_ && !x_timed_out_) {
					enable_align();
					x_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_x_pub_.publish(enable_msg);
			}
		}
		virtual void align_y(ros::Rate r, bool enable=true, bool wait_for_alignment=false, double timeout=1.0, double keep_enabled=false) {
			ROS_INFO_STREAM("Running align_y");
			std_msgs::Bool enable_msg;
			enable_msg.data = enable;
			enable_y_pub_.publish(enable_msg);
			start_time_ = ros::Time::now().toSec();

			//Wait to be aligned
			if(wait_for_alignment) {
				while(ros::ok() && !y_aligned_ && !preempted_ && !y_timed_out_) {
					enable_align();
					y_timed_out_ = check_timeout(start_time_, timeout);
					preempted_ = check_preempted();
					r.sleep();
				}

				//Set end enable state to keep_enabled
				enable_msg.data = keep_enabled;
				enable_y_pub_.publish(enable_msg);
			}
		}
#endif

		//TODO ability to define just order of orient, x, and y in config or launch file with timeouts and conditions for continuing to the next step and align in one direction throughout the whole time
		//
		//TODO code for combining joystick input with align output
		//
		//TODO code for using joystick input to affect align output
		//
		//TODO Set state of align server with printouts in executeCB
		//
		//TODO add more debug printouts
		//
		//TODO Make this configurable
		virtual bool wait_for_mech(ros::Rate r, double timeout) {
			bool waiting = true;
			while(waiting && ros::ok() &&!preempted_) {
				const bool timed_out = check_timeout(start_time_, timeout);

				auto state = ac_elevator_.getState();
				if ((state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) ||
					(state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED) ||
					timed_out)
				{
					waiting = false;
					if (timed_out)
						ROS_ERROR_STREAM("align_server move_mech timed out");
					if (state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
					{
						ROS_INFO_STREAM("align_server move_mech returned preempted_");
						preempted_ = true;
					}
					if (!ac_elevator_.getResult()->success)
					{
						ROS_INFO_STREAM("align_server move_mech did not succeed");
						preempted_ = true;
					}
					else {
						return true;
					}
				}
				if(as_.isPreemptRequested()){
					preempted_ = true;
				}
				else {
					ros::spinOnce();
					r.sleep();
				}
			}
			return false;
		}
		//TODO make this configurable
		//TODO : probably best to do as the start of a
		// robot_align in a derived class, which then calls
		// the base class robot_align
		//
		// Or maybe add a generic pre_robot_align method
		// which can be overriden in the derived classes
		// It can be empty for the base class and then
		// modified as needed by other conditions
		//Function to move mech out of the way of sensors
		virtual bool move_mech(ros::Rate r, bool wait_for_result) {
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = CARGO_SHIP;
			elev_goal.place_cargo = false;
			ac_elevator_.sendGoal(elev_goal);
			if(wait_for_result) {
				return wait_for_mech(r, align_timeout_);
			}
			else {
				return true;
			}
		}

		//Disable all PID nodes
		virtual void disable_pid() {
			std_msgs::Bool false_msg;
			false_msg.data = false;

			enable_align_pub_.publish(false_msg);
			for (auto &axis : axis_states_)
				axis.second.enable_pub_.publish(false_msg);
#if 0
			enable_orient_pub_.publish(false_msg);
			enable_x_pub_.publish(false_msg);
			enable_y_pub_.publish(false_msg);
#endif
		}

		// Example align function
		// Probably best to make this pure virtual and require
		// any classes using this to define a custom one
		virtual bool robot_align()
		{
			ros::Rate r(60);
			ROS_WARN("starting robot_align");

			start_time_ = ros::Time::now().toSec();
			bool timed_out = false;

			for (auto &axis : axis_states_)
			{
				axis.second.timed_out_ = false;
				axis.second.aligned_ = false;
			}

			aligned_ = false;

			//move mech out of the way
			//move_mech(r, false);
			//enable, wait for alignment, TODO change this timeout, keep enabled
			ROS_WARN("Starting orient align");
			do_align("orient", r, true, true, align_timeout_, true);
			ROS_WARN("Ending orient align");

			//Check if it timed out or preempted while waiting
			timed_out = check_timeout(start_time_, align_timeout_);
			preempted_ = check_preempted();
			if(preempted_ || timed_out) {
				return false;
			}

			//enable, wait for alignment, default timeout, don't keep enabled
			//align_x(r, true, true);

			////Check if it timed out or preempted while waiting
			//timed_out = check_timeout(start_time_, align_timeout_);
			//preempted_ = check_preempted();
			//if(preempted_ || timed_out) {
			//	return false;
			//}
			//enable,don't wait for alignment, default timeout, don't keep enabled
			ROS_WARN("Starting y align");
			do_align("y", r, true);
			do_align("x", r, true, true);
			ROS_WARN("ending y align");

			//Check if it timed out or preempted while waiting
			timed_out = check_timeout(start_time_, align_timeout_);
			preempted_ = check_preempted();
			if(preempted_ || timed_out) {
				return false;
			}

			ROS_INFO("Base align class: align succeeded");
			return true;
		}

		//define the function to be executed when the actionlib server is called
		virtual void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			double start_time = ros::Time::now().toSec();
			disable_pid();
			ros::Rate r(20);
			bool align_succeeded = robot_align();
			bool timed_out = false;
			preempted_ = false;
			while(!timed_out && ros::ok()) {
				timed_out = ros::Time::now().toSec() - start_time > 0.1;
				r.sleep();
			}
			disable_pid(); //Disable all align PID after execution

			behaviors::AlignResult result; //variable to store result of the actionlib action

			if (std::any_of(axis_states_.cbegin(), axis_states_.cend(),
						[](const auto &axis) { return axis.second.timed_out_; } ))
			{
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted_)
			{
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			return;
		}
};

/*int main(int argc, char** argv) {
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_private_params("~");
	ros::NodeHandle n_params(n, "align_server_params");
    ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

	if(!n_params.getParam("align_timeout_", align_timeout_))
		ROS_ERROR_STREAM("Could not read align_timeout_ in align_server");
	if(!n_params.getParam("orient_timeout", orient_timeout))
		ROS_ERROR_STREAM("Could not read orient_timeout in align_server");
	if(!n_params.getParam("x_timeout", x_timeout))
		ROS_ERROR_STREAM("Could not read x_timeout in align_server");
	if(!n_params.getParam("y_timeout", y_timeout))
		ROS_ERROR_STREAM("Could not read y_timeout in align_server");
	if(!n_params.getParam("orient_error_threshold", orient_error_threshold))
		ROS_ERROR_STREAM("Could not read orient_error_threshold in align_server");
	if(!n_params.getParam("x_error_threshold", x_error_threshold))
		ROS_ERROR_STREAM("Could not read x_error_threshold in align_server");
	if(!n_params.getParam("cargo_error_threshold", y_error_threshold))
		ROS_ERROR_STREAM("Could not read cargo_error_threshold in align_server");

	if(!n_private_params.getParam("debug", debug))
		ROS_ERROR_STREAM("Could not read debug in align_server");


	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_cargo_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_cargo_pub_ = std::make_shared<ros::Publisher>();

	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("orient_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*cargo_enable_distance_pub_ = n.advertise<std_msgs::Bool>("cargo_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_camera/enable_y_pub", 1,  true);
	*enable_cargo_pub_ = n.advertise<std_msgs::Bool>("cargo_pid/pid_enable", 1,  true);
	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_align_cargo_pub_ = n.advertise<std_msgs::Bool>("align_cargo_pid/pid_enable", 1,  true);

	BaseAlignAction align_action("align_server", enable_align_hatch_pub_, enable_navx_pub_, hatch_panel_enable_distance_pub_, enable_y_pub_, "orient_pid/pid_debug", "hatch_panel_distance_pid/pid_debug", "align_with_camera/y_aligned"); //TODO make the error topics params?


	//Stop PID nodes from defaulting true
	// TODO : why not just put this call in the BaseAlignAction constructor, then move
	// all of the publishers to be straight member variables in BaseAlignAction
	std_msgs::Bool false_msg;
	false_msg.data = false;
	enable_navx_pub_.publish(false_msg);
	hatch_panel_enable_distance_pub_.publish(false_msg);
	cargo_enable_distance_pub_.publish(false_msg);
	enable_y_pub_.publish(false_msg);
	enable_cargo_pub_.publish(false_msg);
	enable_align_hatch_pub_.publish(false_msg);
	enable_align_cargo_pub_.publish(false_msg);

    ros::spin();
	return 0;
}*/
