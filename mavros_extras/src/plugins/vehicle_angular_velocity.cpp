/**
 * @brief Vehicle Angular Velocity plugin
 * @file vehicle_angular_velocity.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/VehicleAngularVelocity.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Angular Velocity plugin
 */
class VehicleAngularVelocityPlugin : public plugin::PluginBase {
public:
	VehicleAngularVelocityPlugin() : PluginBase(),
		nh("~vehicle_angular_velocity")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// vehicle_angular_velocity_pub = nh.advertise<mavros_msgs::VehicleAngularVelocity>("vehicle_angular_velocity", 1000);
		// vehicle_angular_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vehicle_angular_velocity", 1000);
		vehicle_angular_velocity_pub = nh.advertise<geometry_msgs::Point>("vehicle_angular_velocity", 1000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&VehicleAngularVelocityPlugin::handle_vehicle_angular_velocity),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher vehicle_angular_velocity_pub;

	void handle_vehicle_angular_velocity(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VEHICLE_ANGULAR_VELOCITY &vehicle_angular_velocity)
	{
		// custom message type
		/*
		auto vmsg = boost::make_shared<mavros_msgs::VehicleAngularVelocity>();
		vmsg->header.stamp = ros::Time::now();
        vmsg->angular_velocity_x = vehicle_angular_velocity.angular_velocity_x;
        vmsg->angular_velocity_y = vehicle_angular_velocity.angular_velocity_y;
        vmsg->angular_velocity_z = vehicle_angular_velocity.angular_velocity_z;
		vehicle_angular_velocity_pub.publish(vmsg);
		*/
	
		// Vector3Stamped message type
		/*
		auto vmsg = boost::make_shared<geometry_msgs::Vector3Stamped>();
		vmsg->header.stamp = ros::Time::now();
        vmsg->vector.x = vehicle_angular_velocity.angular_velocity_x;
        vmsg->vector.y = vehicle_angular_velocity.angular_velocity_y;
        vmsg->vector.z = vehicle_angular_velocity.angular_velocity_z;
		vehicle_angular_velocity_pub.publish(vmsg);
		*/
	
		// Point message type
		auto vmsg = boost::make_shared<geometry_msgs::Point>();
        vmsg->x = vehicle_angular_velocity.angular_velocity_x;
        vmsg->y = vehicle_angular_velocity.angular_velocity_y;
        vmsg->z = vehicle_angular_velocity.angular_velocity_z;
		vehicle_angular_velocity_pub.publish(vmsg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleAngularVelocityPlugin, mavros::plugin::PluginBase)
