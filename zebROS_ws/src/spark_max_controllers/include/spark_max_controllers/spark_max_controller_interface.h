#pragma once

#include <spark_max_interface/spark_max_command_interface.h>
#include <XmlRpcValue.h>

namespace spark_max_controllers
{

class SparkMaxCIParam
{
	public:
		SparkMaxCIParams(void) :
			, inverted_(false)
			, p_gain_{0}
			, i_gain_{0}
			, d_gain_{0}
			, f_gain_{0}
			, i_zone_{0}
			, d_filter_{0}
			, pidf_output_min_{-1}
			, pidf_output_max_{1}
			, pidf_reference_ctrl_{kDutyCycle} // TODO : how to set?
			, pidf_reference_slot_(0)
			, pidf_arb_feed_forward_{0}
			, forward_limit_switch_polarity_(kNormallyOpen)
			, forward_limit_switch_enabled_(false)
			, reverse_limit_switch_polarity_(kNormallyOpen)
			, reverse_limit_switch_enabled_(false)
			, current_limit_(0) // TODO : better defaults
			, current_limit_stall_(0)
			, current_limit_free_(0)
			, current_limit_rpm_(0)
			, secondary_current_limit_(0)
			, secondary_current_limit_cycles_(0)
			, idle_mode_(kCoast)
			, ramp_rate_(0)
			, follower_type_(kFollowerDisabled) // TODO : ????
			, follower_id_(-1)
			, follower_invert_(false)
			{
			}

			// Read a joint name from the given nodehandle's params
			bool readJointName(ros::NodeHandle &n)
			{
				if (!n.getParam("joint", joint_name_))
				{
					ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
					return false;
				}
				return true;
			}

			bool readCloseLoopParams(ros::NodeHandle &n)
			{
				XmlRpc::XmlRpcValue pid_param_list;

				n.getParams("pid_slot", pidf_reference_slot_);
				if (!n.getParam("close_loop_values", pid_param_list))
					return true;
				if (pid_param_list.size() <= SPARK_MAX_PID_SLOTS)
				{
					for (int i = 0; i < pid_param_list.size(); i++)
					{
						XmlRpc::XmlRpcValue &pidparams = pid_param_list[i];

						findFloatParam("p", pidparams, p_gain[i]);
						findFloatParam("i", pidparams, i_gain[i]);
						findFloatParam("d", pidparams, d_gain[i]);
						findFloatParam("f", pidparams, f_gain[i]);
						findIntParam("i_zone", pidparams, i_zone_[i]);
						findIntParam("d_filter", pidparams, d_filter_[i]);
						findIntParam("output_min", pidparams, output_min_[i]);
						findIntParam("output_max", pidparams, output_max_[i]);
						findIntParam("arb_feed_forward", pidparams, arb_feed_forward_[i]);
					}
					return true;
				}
				else
				{
					throw std::runtime_error("Too many pid_param values");
				}
				return false;
			}

			bool readLimitSwitches(ros::NodeHandle &n)
			{
				std::string str_val;
				if (n.getParam("forward_limit_switch_polarity", str_val))
				{
					hardware_interface::LimitSwitchPolarity limit_switch_normal;
					if (!stringToLimitPolarity(str_val, limit_switch_polarity))
						return false;
					forward_limit_switch_polarity_ = limit_switch_polarity;
				}
				n.getParam("forward_limit_switch_enabled", forward_limit_switch_enabled_);
				if (n.getParam("reverse_limit_switch_polarity", str_val))
				{
					hardware_interface::LimitSwitchPolarity limit_switch_normal;
					if (!stringToLimitPolarity(str_val, limit_switch_polarity))
						return false;
					reverse_limit_switch_polarity_ = limit_switch_polarity;
				}
				n.getParam("reverse_limit_switch_enabled", reverse_limit_switch_enabled_);
				return true;
			}

			bool readCurrentLmits(ros::NodeHandle &n)
			{
				n.getParam("current_limit", current_limit_);
				n.getParam("current_limit_stall", current_limit_stall_);
				n.getParam("current_limit_free", current_limit_free_);
				n.getParam("current_limit_rpm", current_limit_rpm_);
				n.getParam("secondary_current_limit", secondary_current_limit_);
				n.getParam("secondary_current_limit_cycles", secondary_current_limit_cycles_);
				return true;
			}

			bool readIdleMode(ros::NodeHandle &n)
			{
				std::string str_val;
				if (n.getParam("idle_mode", str_val))
				{
					hardware_interface::IdleMode idle_mode;
					if (!stringToIdleMode(str_val, idle_mode))
						return false;
					idle_mode_ = idle_mode;
				}
				return true;
			}

			bool readVoltageCompensation(ros::NodeHandle &c)
			{
				const bool read_enable = n.getParam("voltage_compensation_enable", voltage_compensation_enable_);
				const bool read_voltage = n.getParam("voltage_compensation_nominal_voltage", voltage_compensation_nominal_voltage_);
				if (read_enable && !read_voltage)
				{
					ROS_WARN("voltage_compensation_enable set without setting voltage_compensation_nominal_voltage, enabling with default value");
				}
				else if (!read_enable && read_voltage)
				{
					ROS_WARN("voltage_compensation_nominal_voltage set without setting voltage_compensation_enable, voltage compenstaion disabled");
				}

				return true;
			}

			bool readRampRate(ros::NodeHandle &n)
			{
				n.getParam("open_loop_ramp_rate", open_loop_ramp_rate_);
				n.getParam("closed_loop_ramp_rate", closed_loop_ramp_rate_);
				return true;
			}

			bool readSoftlimit(ros::NodeHandle &n)
			{
				const bool read_forward_enable = n.getParam("forward_softlimit_enable", forward_softlimit_enable_);
				const bool read_forward_limit  = n.getParam("forward_softlimit", forward_softlimit_);
				if (read_forward_enable && !read_forward_limit)
				{
					ROS_ERROR("Can not enable forward limit without forward_softlimit set");
					return false;
				}
				else if (!read_forward_enable && read_forward_limit)
				{
					ROS_WARN("forward softlimit value set but forward_softlimit_enable not set");
				}
				const bool read_reverse_enable = n.getParam("reverse_softlimit_enable", reverse_softlimit_enable_);
				const bool read_reverse_limit  = n.getParam("reverse_softlimit", reverse_softlimit_);
				if (read_reverse_enable && !read_reverse_limit)
				{
					ROS_ERROR("Can not enable reverse limit without reverse_softlimit set");
					return false;
				}
				else if (!read_reverse_enable && read_reverse_limit)
				{
					ROS_WARN("reverse softlimit value set but reverse_softlimit_enable not set");
				}

				return true;
			}

	private:
		bool                inverted_;

		// PID Controller
		std::array<double,                          SPARK_MAX_PID_SLOTS> p_gain_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> i_gain_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> d_gain_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> f_gain_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> i_zone_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> d_filter_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> pidf_output_min_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> pidf_output_max_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> pidf_reference_value_;
		std::array<hardware_interface::ControlType, SPARK_MAX_PID_SLOTS> pidf_reference_ctrl_;
		std::array<double,                          SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_;
		std::array<hardware_interface::ArbFFUnits,  SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_units_;
		size_t                                                           pidf_reference_slot_;


		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_;
		bool                forward_limit_switch_enabled_;
		LimitSwitchPolarity reverse_limit_switch_polarity_;
		bool                reverse_limit_switch_enabled_;

		unsigned int        current_limit_;
		unsigned int        current_limit_stall_;
		unsigned int        current_limit_free_;
		unsigned int        current_limit_RPM_;
		double              secondary_current_limit_;
		int                 secondary_current_limit_cycles_;

		IdleMode            idle_mode_;
		bool                voltage_compensation_enable_;
		double              voltage_compensation_nominal_voltage_;

		double              open_loop_ramp_rate_;
		double              closed_loop_ramp_rate_;

		hardware_interface:: ExternalFollower follower_type_;
		int                 follower_id_;
		bool                follower_invert_;

		bool                forward_softlimit_enable_;
		double              forward_softlimit_;

		bool                reverse_softlimit_enable_;
		double              reverse_softlimit_;

		unsigned int        encoder_ticks_per_rotation_;

		hardware_interface::SensorType encoder_type_;
		// Read a double named <param_type> from the array/map
		// in params
		bool findFloatParam(std::string param_type, XmlRpc::XmlRpcValue &params, double &val) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a double valid type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeDouble)
			{
				val = (double)param;
				return true;
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
			{
				val = (int)param;
				return true;
			}
			else
				throw std::runtime_error("A non-double value was passed for" + param_type);

			return false;
		}

		// Read an integer named <param_type> from the array/map
		// in params
		bool findIntParam(std::string param_type, XmlRpc::XmlRpcValue &params, int &val) const
		{
			if (!params.hasMember(param_type))
				return false;
			XmlRpc::XmlRpcValue &param = params[param_type];
			if (!param.valid())
				throw std::runtime_error(param_type + " was not a valid int type");
			if (param.getType() == XmlRpc::XmlRpcValue::TypeInt)
				val = (int)param;
			else
				throw std::runtime_error("A non-int value was passed for" + param_type);
			return false;
		}

		bool stringToLimitSwitchPolarity(const std::string &str,
									     hardware_interface::LimitSwitchPolarity &limit_switch_polarity)
		{
			if (str == "normally_open")
				limit_switch_polarity = hardware_interface::kNormallyOpen;
			else if (str == "normally_closed")
				limit_switch_polarity = hardware_interface::kNormallyClosed;
			else
			{
				ROS_ERROR_STREAM("Invalid limit switch polarity : " << str);
				return false;
			}
			return true;
		}

		bool stringToIdleMode(const std::string &str,
							  hardware_interface::IdleMode &idle_mode)
		{
			if (str == "coast")
				idle_mode = hardware_interface::kCoast;
			else if (str == "brake")
				idle_mode = hardware_interface::kBrake
			else
			{
				ROS_ERROR_STREAM("Invalid idle mode : " << str);
				return false;
			}
			return true;
		}
};

class SparkMaxControllerInterface
{
	public:
		SparkMaxControllerInterface(void)
		{
		}

		virtual bool readParams(ros::NodeHandle &n, SparkMaxCIParams &params)
		{
			return params.readJointName(n) &&
				   params.readCloseLoopParams(n) &&
				   params.readLimitSwitches(n) &&
				   params.readCurrentLimits(n) &&
				   params.readIdleMode(n) &&
				   params.readVoltageCompensation(n) &&
				   params.readRampRate(n) &&
				   params.readSoftLimit(n);
		}

		// Read params from config file and use them to
		// initialize the Spark Max hardware
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface * /*smsi*/,
								  ros::NodeHandle &n)
		{
			return init(smci, n, spark_max_, /*srv_mutex_, srv_,*/ true) &&
				   setInitialMode();
		}

		// Same as above, except pass in an array
		// of node handles. First entry is set up as the master
		// spark max and the rest are set in follower mode to follow
		// the leader
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface *smsi,
								  std::vector<ros::NodeHandle> &n)
		{
			if (!initWithNode(smci, smsi, n[0]))
				return false;

			const int follow_can_id = spark_max_.state()->getCANID();

			follower_spark_maxs_.resize(n.size() - 1);
			for (size_t i = 1; i < n.size(); i++)
			{
				follower_srv_mutexes_.push_back(nullptr);
				follower_srvs_.push_back(nullptr);
				if (!init(smci, n[i], follower_spark_maxs_[i-1], follower_srv_mutexes_[i-1], follower_srvs_[i-1], false))
					return false;
				follower_spark_maxs_[i-1]->setFollowerType(hardware_interface::kFollowerSparkMax);
				follower_spark_maxs_[i-1]->setFollowerID(follow_can_id);
				ROS_INFO_STREAM("Set up Spark Max " << follower_spark_maxs_[i-1].getName()
						<< " to follow Spark Max CAN ID " << follow_can_id
						<< " (" << spark_max_.getName() << ")");
			}

			return true;
		}


		// Init with XmlRpcValue instead of NodeHandle - XmlRpcValue
		// will be either a string or an array of strings of joints
		// to load
		virtual bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
								  hardware_interface::SparkMaxStateInterface *smsi,
								  ros::NodeHandle &controller_nh,
								  XmlRpc::XmlRpcValue param)
		{
			std::vector<ros::NodeHandle> joint_nodes;

			if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
			{
				if (param.size() == 0)
				{
					ROS_ERROR_STREAM("Joint param is an empty list");
					return false;
				}

				for (int i = 0; i < param.size(); ++i)
				{
					if (param[i].getType() != XmlRpc::XmlRpcValue::TypeString)
					{
						ROS_ERROR_STREAM("Joint param #" << i << " isn't a string.");
						return false;
					}
				}

				for (int i = 0; i < param.size(); ++i)
					joint_nodes.push_back(ros::NodeHandle(controller_nh,
								static_cast<std::string>(param[i])));
			}
			else if (param.getType() == XmlRpc::XmlRpcValue::TypeString)
			{
				joint_nodes.push_back(ros::NodeHandle(controller_nh,
							static_cast<std::string>(param)));
			}
			else
			{
				ROS_ERROR_STREAM("Joint param is neither a list of strings nor a string.");
				return false;
			}

			return initWithNode(smci, smsi, joint_nodes);
		}

		// Set the setpoint for the motor controller
		virtual void setCommand(const double command)
		{
			spark_max_->setSetPoint(command);
		}

		// Set the mode of the motor controller
		virtual void setMode(hardware_interface::ControlType mode);
		{
			spark_max_->setPIDReferenceCtrl(mode);
		}

		virtual bool setPIDFSlot(int slot)
		{
			if ((slot < 0) || (slot >= SPARK_MAX_PID_SLOTS))
			{
				ROS_ERROR ("PID slot out of range in SparkMaxControllerInterface::setPIDFSlot()");
				return false;
			}
			params_.pidf_reference_slot_ = slot;

			// If dynamic reconfigure is running update
			// the reported config there with the new internal
			// state
			syncDynamicReconfigure();

			spark_max_->setPIDFReferenceSlot(slot);
			return true;
		}

	protected:
		hardware_interface::SparkMaxCommandHandle                     spark_max_;
		SparkMaxCIParams                                              params_;

#if 0
		std::shared_ptr<dynamic_reconfigure::Server<SparkMaxnConfig>> srv_;
		std::shared_ptr<boost::recursive_mutex>                       srv_mutex_;

		// List of follower talons associated with the master
		// listed above
		std::vector<hardware_interface::SparkMaxCommandHandle>                    follower_spark_maxs_;
		std::vector<std::shared_ptr<dynamic_reconfigure::Server<SparkMaxConfig>>> follower_srvs_;
		std::vector<std::shared_ptr<boost::recursive_mutex>>                      follower_srv_mutexes_;

#endif

		// Used to set initial (and only) talon
		// mode for FixedMode derived classes
		virtual bool setInitialMode(void)
		{
			ROS_INFO_STREAM("SparkMax " << spark_max_.getName() << " Base class setInitialMode");
			return true;
		}

	private :
		virtual bool init(hardware_interface::SparkMaxCommandInterface *smci,
							ros::NodeHandle &n,
							hardware_interface::SparkMaxCommandHandle &spark_max,
							/*
							std::shared_ptr<boost::recursive_mutex> &srv_mutex,
							std::shared_ptr<dynamic_reconfigure::Server<spark_max_controllers::SparkMaxConfig>> &srv,
							*/
							bool update_params)
		{
			ROS_WARN("spark max init start");
			// Read params from startup and intialize SparkMax using them
			SparkMaxCIParams params;
			if (!readParams(n, params))
			   return false;

			bool dynamic_reconfigure;
			n.param<bool>("dynamic_reconfigure", dynamic_reconfigure, false);
			ROS_WARN("spark max init past readParams");

			spark_max = smci->getHandle(params.joint_name_);
			if (!writeParamsToHW(params, spark_max, update_params))
				return false;

			ROS_WARN("spark_max init past writeParamsToHW");
#if 0
			if (dynamic_reconfigure)
			{
				// Create dynamic_reconfigure Server. Pass in n
				// so that all the vars for the class are grouped
				// under the node's name.  Doing so allows multiple
				// copies of the class to be started, each getting
				// their own namespace.
				srv_mutex = std::make_shared<boost::recursive_mutex>();
				srv = std::make_shared<dynamic_reconfigure::Server<talon_controllers::SparkMaxConfig>>(*srv_mutex_, n);

				ROS_WARN("init updateConfig");
				// Without this, the first call to callback()
				// will overwrite anything passed in from the
				// launch file
				srv->updateConfig(params_.toConfig());

				ROS_WARN("init setCallback");
				// Register a callback function which is run each
				// time parameters are changed using
				// rqt_reconfigure or the like
				srv->setCallback(boost::bind(&SparkMaxControllerInterface::callback, this, _1, _2));
			}
#endif
			ROS_WARN("spark max init returning");

			return true;
		}

		// If dynamic reconfigure is running then update
		// the reported config there with the new internal
		// state
		void syncDynamicReconfigure(void)
		{
#if 0
			if (srv_)
			{
				SparkMaxConfig config(params_.toConfig());
				// first call in updateConfig is another lock, this is probably
				// redundant
				// boost::recursive_mutex::scoped_lock lock(*srv_mutex_);
				srv_->updateConfig(config);
			}
#endif
		}

		// Use data in params to actually set up SparkMax
		// hardware. Make this a separate method outside of
		// init() so that dynamic reconfigure callback can write
		// values using this method at any time
		virtual bool writeParamsToHW(const SparkMaxCIParams &params,
									 hardware_interface::SparkMaxCommandHandle &spark_max,
									 bool update_params = true)
		{
			spark_max->setInverted(params.inverted_);
			for (size_t i = 0; i < SPARK_MAX_PID_SLOTS; i++)
			{
				spark_max->setPGain(i, params.p_gain_[i]);
				spark_max->setIGain(i, params.i_gain_[i]);
				spark_max->setDGain(i, params.d_gain_[i]);
				spark_max->setFGain(i, params.f_gain_[i]);
				spark_max->setIZone(i, params.i_zone_[i]);
				spark_max->setDFilter(i, params.d_filter_[i]);
				spark_max->setPIDFOutputMin(i, params.pidf_output_min_[i]);
				spark_max->setPIDFOutputMax(i, params.pidf_output_max_[i]);
				spark_max->setPIDFArbFeedForward(i, params.pidf_arb_feed_forward_[i]);
			}
			spark_max->setPIDFRefrenceSlot(params.pidf_reference_slot_);

			spark_max->setForwardLimitSwitchPolarity(params.forward_limit_switch_polarity_);
			spark_max->setForwardLimitSwitchEnabled(params.forward_limit_switch_enabled_);
			spark_max->setReverseLimitSwitchPolarity(params.reverse_limit_switch_polarity_);
			spark_max->setReverseLimitSwitchEnabled(params.reverse_limit_switch_enabled_);

			spark_max->setCurrentLimit(params.current_limit_);
			spark_max->setCurrentLimitStall(params.current_limit_stall_);
			spark_max->setCurrentLimitFree(params.current_limit_free_);
			spark_max->setCurrentLimitRPM(params.current_limit_rpm_);
			spark_max->setSecondaryCurrentLimit(params.secondary_current_limit_);
			spark_max->setSecondaryCurrentLimitCycles(params.secondary_current_limit_cycles_);

			spark_max->setIdleMode(params.idle_mode_);
			spark_max->setRampRate(params.ramp_rate_);

			// Save copy of params written to HW
			// so they can be queried later?
			if (update_params)
				params_ = params;

			return true;

		}
};

// Base --^

// A derived class which disables mode switching. Any
// single-mode CI class should derive from this class
class SparkMaxFixedModeControllerInterface : public SparkMaxControllerInterface
{
	protected:
		// Disable changing mode for controllers derived from this class
		void setMode(hardware_interface::SparkMaxMode /*mode*/) override
		{
			ROS_WARN("Can't reset mode using this SparkMaxControllerInterface");
		}
};



// Duty Cycle
class SparkMaxDutyCycleControllerInterface : public SparkMaxFixedModeControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set the mode at init time - since this
			// class is derived from the FixedMode class
			// it can't be reset
			spark_max_->setPIDFreferenceCtrl(hardware_interface::kDutyCycle);
			ROS_INFO_STREAM("Set up spark max" << talon_.getName() << " in duty cycle mode");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
};

// Position
// Velocity
// Voltage
class SparkMaxCloseLoopControllerInterface : public SparkMaxFixedModeControllerInterface
{
};

class SparkMaxPositionCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(hardware_interface::kPosition);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Position mode");

			return true;
		}
};

class SparkMaxVelocityCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(hardware_interface::kVelocity);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Velocity mode");

			return true;
		}
};

class SparkMaxVoltageCloseLoopControllerInterface : public SparkMaxCloseLoopControllerInterface
{
	protected:
		bool setInitialMode(void) override
		{
			// Set to speed close loop mode
			spark_max_->setPIDFReferenceCtrl(hardware_interface::kVoltage);
			ROS_INFO_STREAM("Set up spark_max " << spark_max_.getName() << " in Close Loop Voltage mode");

			return true;
		}
};
class SparkMaxFollowerControllerInterface : public SparkMaxFixedModeControllerInterface
{
	public:
		bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
						  hardware_interface::SparkMaxStateInterface   *smsi,
						  ros::NodeHandle &n) override
		{
			if (!tsi)
			{
				ROS_ERROR("NULL SparkMaxStateInterface in SparkMaxFollowerCommandInterface");
				return false;
			}

			std::string follow_joint_name;
			if (!n.getParam("follow_joint", follow_joint_name))
			{
				ROS_ERROR("No follow joint specified for SparkMaxFollowerControllerInterface");
				return false;
			}

			hardware_interface::SparkMaxStateHandle follow_handle = smsi->getHandle(follow_joint_name);
			const int follow_device_id = follow_handle->getDeviceId();
j
			if (!common_init(smci, n, hardware_interface::kFollowerSparkMax, follow_device_id))
				return false;

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ <<
					" to follow Spark Max CAN ID " << follow_device_id <<
					" (" << follow_handle.getName() << ")");
			return true;
		}
		bool initWithNode(hardware_interface::SparkMaxCommandInterface *smci,
						  hardware_interface::TalonStateInterface      *tsi,
						  ros::NodeHandle &n) override
		{
			if (!tsi)
			{
				ROS_ERROR("NULL SparkMaxStateInterface in SparkMaxFollowerCommandInterface");
				return false;
			}

			std::string follow_joint_name;
			if (!n.getParam("follow_joint", follow_joint_name))
			{
				ROS_ERROR("No follow joint specified for SparkMaxFollowerControllerInterface");
				return false;
			}

			hardware_interface::TalonStateHandle follow_handle = tsi->getHandle(follow_joint_name);
			const int follow_device_id = follow_handle->getDeviceId();
j
			if (!common_init(smci, n, hardware_interface::kFollowerPhoenix, follow_device_id))
				return false;

			ROS_INFO_STREAM("Launching follower " << params_.joint_name_ <<
					" to follow Talon CAN ID " << follow_device_id <<
					" (" << follow_handle.getName() << ")");
			return true;
		}
		// Maybe disable the setPIDFSlot call since that makes
		// no sense for a non-PID controller mode?
		void setCommand(const double /*command*/) override
		{
			ROS_WARN("Can't set a command in follower mode!");
		}

	private:
		bool commonInit(hardware_interface::SparkMaxCommandInterface *tci,
						ros::NodeHandle &n,
						ExternalFollower follow_device_type,
						int follow_device_id)
		{
			// Call base-class init to load config params
			if (!SparkMaxControllerInterface::initWithNode(tci, nullptr, n))
			{
				ROS_ERROR("SparkMaxFollowerController base initWithNode failed");
				return false;
			}
			// Set the mode and CAN ID of talon to follow at init time -
			// since this class is derived from the FixedMode class
			// these can't be reset. Hopefully we never have a case
			// where a follower mode SparkMax changes which other
			// SparkMax it is following during a match?
			talon_->setFollowerType(follow_device_type);
			talon_->setFollowerID(follow_device_id);
		}
};
}

