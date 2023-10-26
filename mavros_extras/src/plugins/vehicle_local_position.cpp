/**
 * @brief Vehicle Local Position Plugin
 * @file vehicle_local_position.cpp
 * @author Muhammad Kamal
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>

#include <mavros_msgs/VehicleLocalPosition.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>


namespace mavros {
namespace extra_plugins {
/**
 * @brief Vehicle Local Position Plugin
 */
class VehicleLocalPositionPlugin : public plugin::PluginBase {
public:
	VehicleLocalPositionPlugin() : PluginBase(),
		nh("~vehicle_local_position")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// vehicle_local_position_pub = nh.advertise<mavros_msgs::VehicleLocalPosition>("vehicle_local_position", 100000);
		
		// vehicle_pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vehicle_local_pos", 100000);
		// vehicle_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vehicle_local_vel", 100000);
		// vehicle_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>("vehicle_local_acc", 100000);

		vehicle_pos_pub = nh.advertise<geometry_msgs::Point>("vehicle_local_pos", 100000);
		vehicle_vel_pub = nh.advertise<geometry_msgs::Point>("vehicle_local_vel", 100000);
		vehicle_acc_pub = nh.advertise<geometry_msgs::Point>("vehicle_local_acc", 100000);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&VehicleLocalPositionPlugin::handle_vehicle_local_position),
		};
	}

private:
	ros::NodeHandle nh;

	// ros::Publisher vehicle_local_position_pub;
	ros::Publisher vehicle_pos_pub;
	ros::Publisher vehicle_vel_pub;
	ros::Publisher vehicle_acc_pub;

	void handle_vehicle_local_position(const mavlink::mavlink_message_t *msg, mavlink::common::msg::VEHICLE_LOCAL_POSITION &vehicle_local_position)
	{	
		//Point message type
		auto pos_msg = boost::make_shared<geometry_msgs::Point>();
		pos_msg->x = vehicle_local_position.x;
		pos_msg->y = vehicle_local_position.y;
		pos_msg->z = vehicle_local_position.z;
		vehicle_pos_pub.publish(pos_msg);

		auto vel_msg = boost::make_shared<geometry_msgs::Point>();
		vel_msg->x = vehicle_local_position.vx;
		vel_msg->y = vehicle_local_position.vy;
		vel_msg->z = vehicle_local_position.vz;
		vehicle_vel_pub.publish(vel_msg);

		auto acc_msg = boost::make_shared<geometry_msgs::Point>();
		acc_msg->x = vehicle_local_position.ax;
		acc_msg->y = vehicle_local_position.ay;
		acc_msg->z = vehicle_local_position.az;
		vehicle_acc_pub.publish(acc_msg);

		// Vector3Stamped message type
		/*
		auto pos_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
		pos_msg->header.stamp = ros::Time::now();
		pos_msg->vector.x = vehicle_local_position.x;
		pos_msg->vector.y = vehicle_local_position.y;
		pos_msg->vector.z = vehicle_local_position.z;
		vehicle_pos_pub.publish(pos_msg);

		auto vel_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
		vel_msg->header.stamp = ros::Time::now();
		vel_msg->vector.x = vehicle_local_position.vx;
		vel_msg->vector.y = vehicle_local_position.vy;
		vel_msg->vector.z = vehicle_local_position.vz;
		vehicle_vel_pub.publish(vel_msg);

		auto acc_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
		acc_msg->header.stamp = ros::Time::now();
		acc_msg->vector.x = vehicle_local_position.ax;
		acc_msg->vector.y = vehicle_local_position.ay;
		acc_msg->vector.z = vehicle_local_position.az;
		vehicle_acc_pub.publish(acc_msg);
		*/

		/*
		auto vmsg = boost::make_shared<mavros_msgs::VehicleLocalPosition>();

		vmsg->header.stamp = ros::Time::now();

		vmsg->xy_valid = vehicle_local_position.xy_valid;
		vmsg->z_valid = vehicle_local_position.z_valid;
		vmsg->v_xy_valid = vehicle_local_position.v_xy_valid;
		vmsg->v_z_valid = vehicle_local_position.v_z_valid;
		vmsg->x = vehicle_local_position.x;
		vmsg->y = vehicle_local_position.y;
		vmsg->z = vehicle_local_position.z;
		vmsg->delta_xy[0] = vehicle_local_position.delta_xy[0];
		vmsg->delta_xy[1] = vehicle_local_position.delta_xy[1];
		vmsg->xy_reset_counter = vehicle_local_position.xy_reset_counter;
		vmsg->delta_z = vehicle_local_position.delta_z;
		vmsg->z_reset_counter = vehicle_local_position.z_reset_counter;
		vmsg->vx = vehicle_local_position.vx;
		vmsg->vy = vehicle_local_position.vy;
		vmsg->vz = vehicle_local_position.vz;
		vmsg->z_deriv = vehicle_local_position.z_deriv;
		vmsg->delta_vxy[0] = vehicle_local_position.delta_vxy[0];
		vmsg->delta_vxy[1] = vehicle_local_position.delta_vxy[1];
		vmsg->vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
		vmsg->delta_vz = vehicle_local_position.delta_vz;
		vmsg->vz_reset_counter = vehicle_local_position.vz_reset_counter;
		vmsg->ax = vehicle_local_position.ax;
		vmsg->ay = vehicle_local_position.ay;
		vmsg->az = vehicle_local_position.az;
		vmsg->xy_global = vehicle_local_position.xy_global;
		vmsg->z_global = vehicle_local_position.z_global;
		vmsg->ref_lat = vehicle_local_position.ref_lat;
		vmsg->ref_lon = vehicle_local_position.ref_lon;
		vmsg->ref_alt = vehicle_local_position.ref_alt;
		vmsg->dist_bottom = vehicle_local_position.dist_bottom;
		vmsg->dist_bottom_valid = vehicle_local_position.dist_bottom_valid;
		vmsg->dist_bottom_sensor_bitfield = vehicle_local_position.dist_bottom_sensor_bitfield;
		vmsg->eph = vehicle_local_position.eph;
		vmsg->epv = vehicle_local_position.epv;
		vmsg->evh = vehicle_local_position.evh;
		vmsg->evv = vehicle_local_position.evv;
		vmsg->dead_reckoning = vehicle_local_position.dead_reckoning;
		vmsg->vxy_max = vehicle_local_position.vxy_max;
		vmsg->vz_max = vehicle_local_position.vz_max;
		vmsg->hagl_min = vehicle_local_position.hagl_min;
		vmsg->hagl_max = vehicle_local_position.hagl_max;

		vehicle_local_position_pub.publish(vmsg);
		*/
		}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VehicleLocalPositionPlugin, mavros::plugin::PluginBase)
