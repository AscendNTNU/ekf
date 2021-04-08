
// These must be defined before including TinyEKF.h
#define Nsta 6     // Two state values: pressure, temperature
#define Mobs 4     // Three measurements: baro pressure, baro temperature, LM35 temperature
#include "TinyEKF.h"



#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>

geometry_msgs::PoseStamped module_pose;

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);
            this->setQ(1, 1, .0001);

            // Same for measurement noise
            this->setR(0, 0, .0001);
            this->setR(1, 1, .0001);
            this->setR(2, 2, .0001);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0];
            fx[1] = this->x[1];

            // So process model Jacobian is identity matrix
            F[0][0] = 1;
            F[1][1] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            // A more realistic measurement function would distinguish between state value and measured value; e.g.:
            //   hx[0] = pow(this->x[0], 1.03);
            //   hx[1] = 1.005 * this->x[1];
            //   hx[2] = .9987 * this->x[1] + .001;
            hx[0] = this->x[0]; // Barometric pressure from previous state
            hx[1] = this->x[1]; // Baro temperature from previous state
            hx[2] = this->x[1]; // LM35 temperature from previous state

            // Jacobian of measurement function
            H[0][0] = 1;        // Barometric pressure from previous state
            H[1][1] = 1 ;       // Baro temperature from previous state
            H[2][1] = 1 ;       // LM35 temperature from previous state
        }
};

void perceptionPoseCallback(geometry_msgs::PoseStampedConstPtr module_pose_ptr){
    module_pose = *module_pose_ptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ros::Rate rate(30);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Starting up.");
    ros::NodeHandle node_handle;

    //subscribers
    ros::Subscriber module_pose_sub = node_handle.subscribe(
        "/simulator/module/noisy/pose", 10, &perceptionPoseCallback);

    //publishers
    //ros::Publisher filtered_module_pose_pub = 
    //        node_handle.advertise<geometry_msgs::PoseStamped>("/ekf/module/pose",10);
    ros::Publisher filtered_module_state_pub = 
            node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/state",10);
    //ros::Publisher ekf_state_pub = node_handle.advertise<double[Nsta]>("/ekf/state",10);


    Fuser ekf;
    double* X;
    mavros_msgs::PositionTarget module_state;
    module_state.header.seq = 0;

    while(ros::ok()){
        double z[Mobs]; // x, y, z, pitch
        z[0] = module_pose.pose.position.x;
        z[1] = module_pose.pose.position.y;
        z[2] = module_pose.pose.position.z;
        
        //extracting the pitch from the quaternion
        geometry_msgs::Quaternion quaternion = module_pose.pose.orientation;
        tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
        // it to zero.
        z[3] = std::isnan(pitch) ? 0.0 : pitch;

        ekf.step(z);
        X = ekf.getX(); // p, r, p', r', omega, L_mast
        const double omega  = X[4];
        const double L_mast = X[5];
        
        ekf_state_pub.publish(X);

        module_state.header.seq++; //seq is read only
        module_state.header.stamp = ros::Time::now();
        module_state.position.x = L_mast * cos(X[0]);
        module_state.position.y = L_mast * cos(X[1]);
        module_state.position.z = L_mast * cos(X[0]) * cos(X[1]);
        module_state.velocity.x = L_mast * X[2];
        module_state.velocity.y = L_mast * X[3];
        module_state.velocity.z = 0; //Not used, so not worth doing the calculations
        filtered_module_state_pub.publish(module_state);
        ros::spinOnce();
        rate.sleep();
    }

}