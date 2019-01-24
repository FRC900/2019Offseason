#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "behaviors/IntakeAction.h"
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include "sensor_msgs/JointState.h"
#include <atomic>
#include <ros/console.h>

//define global variables that will be defined based on config values

double roller_power;
double intake_timeout;
double linebreak_debounce_iterations;


class CargoIntakeAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::IntakeAction> as_; //create the actionlib server
		std::string action_name_;

		ros::ServiceClient controller_client_; //create a ros client to send requests to the controller
		std::atomic<int> linebreak_true_count; //counts how many times in a row the linebreak reported there's a cube since we started trying to intake/outtake
		std::atomic<int> linebreak_false_count; //same, but how many times in a row no cube
		behaviors::IntakeResult result_; //variable to store result of the actionlib action

		//create subscribers to get data
		ros::Subscriber joint_states_sub_;
		ros::Subscriber proceed_;
		bool proceed;

	public:
		//make the executeCB function run every time the actionlib server is called
		CargoIntakeAction(const std::string &name) :
			as_(nh_, name, boost::bind(&CargoIntakeAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start(); //start the actionlib server

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot/cargo_intake_controller/cargo_intake_command", false, service_connection_header);

		//start subscribers subscribing
		joint_states_sub_ = nh_.subscribe("/frcrobot/joint_states", 1, &CargoIntakeAction::jointStateCallback, this);
	}

		~CargoIntakeAction(void) 
		{
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::IntakeGoalConstPtr &goal) {
			ros::Rate r(10);

			//define variables that will be reused for each controller call/actionlib server call
			double start_time;
			bool success; //if controller/actionlib server call succeeded

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			//send command to lower arm and run roller to the cargo intake controller ---------------------------------------
			if(!preempted && !timed_out)
			{
				ROS_ERROR("cargo intake server: lowering arm and spinning roller in");

				//reset variables
				linebreak_true_count = 0; //when this gets higher than linebreak_debounce_iterations, we'll consider the gamepiece intaked
				start_time = ros::Time::now().toSec();
				success = false;

				//define request to send to cargo intake controller
				cargo_intake_controller::CargoIntakeSrv srv;
				srv.request.power = intake_power;
				srv.request.intake_arm = false; //TODO: double check

				//send request to controller
				if(!controller_client_.call(srv))
				{
					ROS_ERROR("Srv intake call failed in cargo intake server");
				}
				//update everything by doing spinny stuff
				ros::spinOnce();

				//run a loop to wait for the controller to do its work. Stop if the action succeeded, if it timed out, or if the action was preempted
				while(!success && !timed_out && !preempted) {
					success = linebreak_true_count > linebreak_debounce_iterations;
					timed_out = (ros::Time::now().toSec()-start_time) > intake_timeout;

					if(as_.isPreemptRequested() || !ros::ok()) {
						ROS_WARN(" %s: Preempted", action_name_.c_str());
						as_.setPreempted();
						preempted = true;
					}
					else {
						r.sleep();
						ros::spinOnce();
					}
				}
			}
			//end of code for sending something to a controller --------------------------------------------------------------------------------

			//set ending state of controller no matter what happened: arm up, roller motors stopped
			//define command to send to cargo intake controller
			cargo_intake_controller;:CargoIntakeSrv srv;
			srv.request.power = 0;
			srv.request.intake_arm = true; //TODO: Double check
			//send request to controller
			if(!controller_client_.call(srv))
			{
				ROS_ERROR("Srv intake call failed in cargo intake server");
			}
			//update everything by doing spinny stuff
			ros::spinOnce();


			//log state of action and set result of action
			if(timed_out)
			{
				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			result_.timed_out = timed_out; //timed_out refers to last controller call, but applies for whole action
			result_.success = success; //success refers to last controller call, but applies for whole action
			as_.setSucceeded(result_); //pretend it succeeded no matter what, but tell what actually happened with the result - helps with SMACH
			return;
		}

		// Function to be called whenever the subscriber for the joint states topic receives a message
		// Grabs various info from hw_interface using
		// dummy joint position values
		void jointStateCallback(const sensor_msgs::JointState &joint_state)
		{
			//get index of linebreak sensor for this actionlib server
			static size_t linebreak_idx = std::numeric_limits<size_t>::max();
			if ((linebreak_idx >= joint_state.name.size()))
			{
				for (size_t i = 0; i < joint_state.name.size(); i++)
				{
					if (joint_state.name[i] == "cargo_intake_line_break") //TODO: define this in the hardware interface
						linebreak_idx = i;
				}
			}

			//update linebreak counts based on the value of the linebreak sensor
			if (linebreak_idx < joint_state.position.size())
			{
				bool linebreak_true = (joint_state.position[linebreak_idx] != 0);
				if(linebreak_true)
				{
					linebreak_true_count += 1;
					linebreak_false_count = 0;
				}
				else
				{
					linebreak_true_count = 0;
					linebreak_false_count += 1;
				}
			}
			else
			{
				static int count = 0;
				if(count % 100 == 0)
				{
					ROS_WARN("intake line break sensor not found in joint_states");
				}
				count++;
				linebreak_true_count = 0;
				linebreak_false_count += 1;
			}
		}
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "cargo_intake_server");

	//create the cargo intake actionlib server
	CargoIntakeAction cargo_intake_action("cargo_intake_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "actionlib_cargo_intake_params");

	if (!n_params.getParam("roller_power", roller_power))
		ROS_ERROR("Could not read roller_power in cargo_intake_server");

	if (!n_params.getParam("intake_timeout", intake_timeout))
		ROS_ERROR("Could not read intake_timeout in cargo_intake_server");

	if (!n.getParam("actionlib_params/linebreak_debounce_iterations", linebreak_debounce_iterations))
		ROS_ERROR("Could not read linebreak_debounce_iterations in intake_sever");

	ros::spin();
	return 0;
}
