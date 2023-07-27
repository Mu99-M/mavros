/**
 * @brief Actuator Motors Plugin
 * @file actuator_motors.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>
// #include <mavros_msgs/ActuatorMotors.h>
#include <std_msgs/Float32MultiArray.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Actuator Motors Plugin
 */
class ActuatorMotorsPlugin : public plugin::PluginBase {
public:
	ActuatorMotorsPlugin() : PluginBase(),
		nh("~actuator_motors")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		actuator_motors_sub = nh.subscribe("actuator_motors_sub", 10, &ActuatorMotorsPlugin::actuator_motors_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return {};
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber actuator_motors_sub;

	void actuator_motors_cb(const std_msgs::Float32MultiArray &msg)
	{
		mavlink::common::msg::ACTUATOR_MOTORS vts{};

		vts.control[0] = msg.data[0];
		vts.control[1] = msg.data[1];
		vts.control[2] = msg.data[2];
		vts.control[3] = msg.data[3];
		vts.control[4] = msg.data[4];
		vts.control[5] = msg.data[5];
		vts.control[6] = msg.data[6];
		vts.control[7] = msg.data[7];
		vts.control[8] = msg.data[8];
		vts.control[9] = msg.data[9];
		vts.control[10] = msg.data[10];
		vts.control[11] = msg.data[11];

		UAS_FCU(m_uas)->send_message_ignore_drop(vts);
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ActuatorMotorsPlugin, mavros::plugin::PluginBase)
