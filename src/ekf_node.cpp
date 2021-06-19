
// These must be defined before including TinyEKF.h
#define Nsta 6     // Nb states
#define Mobs 4     // Nb measurements
#define DEBUG false// print the matrices elements
#include "TinyEKF.h"



#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/DebugValue.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory


#define SAMPLE_TIME 25.0

geometry_msgs::PoseWithCovarianceStamped module_pose;
bool measurement_received;
mavros_msgs::PositionTarget gt_ref, gt_ref_prev;
geometry_msgs::PoseStamped header;

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, 0.0005);
            this->setQ(1, 1, 0.0005);
            this->setQ(2, 2, 0.00040);
            this->setQ(3, 3, 0.00040);
            this->setQ(4, 4, 0.0001);
            this->setQ(5, 5, 0.0001);

            // Same for measurement noise
            this->setR(0, 0, 2.667);
            this->setR(1, 1, 5.667);
            this->setR(2, 2, 5.667);
            this->setR(3, 3, 1.2667);

            for(int i =0;i<Nsta;i++)
                this->setP(i,i,this->getQ(i,i));
        }
        double pddot;
        double rddot;

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            double p = this->x[0];
            double r = this->x[1];
            double pdot = this->x[2];
            double rdot = this->x[3];
            double w = this->x[4]; //omega = wave frequency
            double L = this->x[5]; //length of the mast
            pddot = - pow(w,2) * p/SAMPLE_TIME;
            rddot = - pow(w,2) * r/SAMPLE_TIME;
            
            // Process model is f(x) = x_{k+1}
            fx[0] = p + pdot/SAMPLE_TIME;
            fx[1] = r + rdot/SAMPLE_TIME;
            fx[2] = pdot + pddot;
            fx[3] = rdot + rddot;
            fx[4] = w;
            fx[5] = L;

            // So process model Jacobian is identity matrix
            for(int i = 0;i<Nsta;i++){
                F[i][i] = 1.0;
            }
            F[0][2] = 1.0/SAMPLE_TIME;
            F[1][3] = 1.0/SAMPLE_TIME;
            F[2][0] = - pow(w,2) / SAMPLE_TIME;
            F[3][1] = - pow(w,2) / SAMPLE_TIME;
            F[2][4] = - 2*w*p/SAMPLE_TIME;
            F[3][4] = - 2*w*r/SAMPLE_TIME;

            hx[0] = L * sin(fx[0]);
            hx[1] = L * sin(fx[1]);
            hx[2] = L * cos(fx[0]) * cos(fx[1]);
            hx[3] = fx[0];

            // Jacobian of measurement function
            H[0][0] = L * cos(p); 
            H[1][1] = L * cos(r);
            H[2][0] = - L * cos(r) * sin(p);
            H[2][1] = - L * cos(p) * sin(r);
            H[3][0] = 1.0;
            H[0][5] = sin(p);
            H[1][5] = sin(r);
            H[2][5] = cos(r)* cos(p);
        }
};

void perceptionPoseCallback(geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr){
    module_pose = *module_pose_ptr;
    measurement_received = true;
}

void gt_ModulePoseCallback(geometry_msgs::PoseStampedConstPtr module_pose_ptr){
    if((module_pose_ptr->header.stamp - header.header.stamp).toSec() >0.01){
        header.header = module_pose_ptr->header;
        gt_ref_prev.position.x = gt_ref.position.x;
        gt_ref_prev.position.y = gt_ref.position.y;
        gt_ref_prev.position.z = gt_ref.position.z;
        
        gt_ref_prev.velocity.x = gt_ref.velocity.x;
        gt_ref_prev.velocity.y = gt_ref.velocity.y;
        gt_ref_prev.velocity.z = gt_ref.velocity.z;

        gt_ref_prev.acceleration_or_force.x = gt_ref.acceleration_or_force.x;
        gt_ref_prev.acceleration_or_force.y = gt_ref.acceleration_or_force.y;
        gt_ref_prev.acceleration_or_force.z = gt_ref.acceleration_or_force.z;
        

        
        gt_ref.position.x = module_pose_ptr->pose.position.x;
        gt_ref.position.y = module_pose_ptr->pose.position.y;
        gt_ref.position.z = module_pose_ptr->pose.position.z;
        
//        printf("pos_x %f\tprev_x %f\t",gt_ref.position.x, gt_ref_prev.position.x);

        gt_ref.velocity.x = (gt_ref.position.x - gt_ref_prev.position.x)*20.0;
        gt_ref.velocity.y = (gt_ref.position.y - gt_ref_prev.position.y)*20.0;
        gt_ref.velocity.z = (gt_ref.position.z - gt_ref_prev.position.z)*20.0;

        //printf("vel_x %f\tprev_x %f\n",gt_ref.velocity.x, gt_ref_prev.velocity.x);

        gt_ref.acceleration_or_force.x = (gt_ref.velocity.x - gt_ref_prev.velocity.x)*20.0;
        gt_ref.acceleration_or_force.y = (gt_ref.velocity.y - gt_ref_prev.velocity.y)*20.0;
        gt_ref.acceleration_or_force.z = (gt_ref.velocity.z - gt_ref_prev.velocity.z)*20.0;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ros::Time::init();
    ros::Rate rate(SAMPLE_TIME);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Starting up.");
    ros::NodeHandle node_handle;

    bool use_perception;
    const std::string prefix = ros::this_node::getName() + "/";
    if (!node_handle.getParam(prefix + "use_perception", use_perception)) {
        ROS_FATAL_STREAM(ros::this_node::getName() << ": Could not find parameter: " << prefix + "use_perception");
    }

    //subscribers
    ros::Subscriber module_pose_sub;
    if(use_perception){
        module_pose_sub = node_handle.subscribe(
            "/interaction_point_pose", 10, &perceptionPoseCallback);
    }
    else{
        module_pose_sub = node_handle.subscribe(
            "/simulator/module/noisy/pose", 10, &perceptionPoseCallback);
    }
    ros::Subscriber gt_module_pose_sub = node_handle.subscribe(
        "/simulator/module/ground_truth/pose", 10, &gt_ModulePoseCallback);

    //publishers
    ros::Publisher filtered_module_state_pub = 
            node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/state",10);
    //ros::Publisher future_module_state_pub = 
    //        node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/future_state",10);
    ros::Publisher ekf_state_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/state",10);
    ros::Publisher ekf_meas_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/measurement",10);

    Fuser ekf;
    double X[Nsta];
    mavros_msgs::PositionTarget module_state;
    module_state.header.seq = 0;
//    mavros_msgs::PositionTarget future_module_state;
//    future_module_state.header.seq = 0;

    mavros_msgs::DebugValue ekf_state_vector;
    ekf_state_vector.header.seq = 0;
    //ekf_state_vector.index = -1;
    ekf_state_vector.type = mavros_msgs::DebugValue::TYPE_DEBUG_ARRAY;
    ekf_state_vector.data.resize(Nsta);

    mavros_msgs::DebugValue ekf_meas_vector;
    ekf_meas_vector.header.seq = 0;
    //ekf_meas_vector.index = -1;
    ekf_meas_vector.type = mavros_msgs::DebugValue::TYPE_DEBUG_ARRAY;

    //initiate first state vector
    ekf.setX(4, 0.55);
    ekf.setX(5,2.5);
    measurement_received = false;

    module_pose.header.seq = 0;
    //wait for first measurement
    while(module_pose.header.seq ==0 && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Active.");
    
    while(ros::ok()){
        ekf.prediction();
        if(measurement_received){
            double z[Mobs]; // x, y, z, pitch
            z[0] = module_pose.pose.pose.position.x-0.0957; //mast offset
            z[1] = module_pose.pose.pose.position.y+10.0;//mast offset
            z[2] = module_pose.pose.pose.position.z;
            
            //extracting the pitch from the quaternion
            geometry_msgs::Quaternion quaternion = module_pose.pose.pose.orientation;
            tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
            // it to zero.
            z[3] = std::isnan(pitch) ? 0.0 : pitch;

            if(!ekf.update(z))
                std::cout << "error with ekf step" <<std::endl;
                //todo: restart the kf whith current state ASAP

            ekf_meas_vector.data.assign(z,z+Mobs);
            ekf_meas_vector.header.seq++;
            ekf_meas_vector.header.stamp = ros::Time::now();
            ekf_meas_pub.publish(ekf_meas_vector);
            
            measurement_received = false;
        }
        
        for(int i =0; i< Nsta;i++)
            X[i] = ekf.getX(i); // p, r, p', r', omega, L_mast
        const double L_mast = X[5];
        
        ekf_state_vector.data.assign(X,X+Nsta);
        ekf_state_vector.header.seq++;
        ekf_state_pub.publish(ekf_state_vector);        

        module_state.header.seq++; //seq is read only
        module_state.header.stamp = ros::Time::now();
        module_state.position.x = L_mast * sin(X[0])+0.0957;
        module_state.position.y = L_mast * sin(X[1])-10;
        module_state.position.z = L_mast * cos(X[0]) * cos(X[1]);
        module_state.velocity.x = L_mast * X[2]*cos(X[0]);
        module_state.velocity.y = L_mast * X[3]*cos(X[1]);
        module_state.velocity.z = 0; //Not used, so not worth doing the calculations //todo: can be used
        module_state.acceleration_or_force.x = L_mast *(X[2]*X[2]*sin(X[0]) + ekf.pddot*cos(X[0]));
        module_state.acceleration_or_force.y = L_mast *(X[3]*X[3]*sin(X[1]) + ekf.rddot*cos(X[1]));
        module_state.acceleration_or_force.z = 0; //Not used, so not worth doing the calculations
        filtered_module_state_pub.publish(module_state);
        
        //save some data:
        double ekf_output_data[7];
        ekf_output_data[0] = module_state.position.x;
        ekf_output_data[1] = module_state.position.y;
        ekf_output_data[2] = module_state.position.z;
        ekf_output_data[3] = module_state.velocity.x;
        ekf_output_data[4] = module_state.velocity.y;
        ekf_output_data[5] = X[4];
        ekf_output_data[6] = X[5];

        #if DEBUG
        std::cout << std::fixed << std::setprecision(4) << "X =\t";
        for(int i = 0 ; i<7 ; i++){
            std::cout << ekf_output_data[i] << "\t";
        }
        std::cout << std::endl;
        #endif

/*
        double* future_state = new double[Nsta];
        ekf.future_setpoint(future_prediction_time, future_state);
        future_module_state.header.seq++; //seq is read only
        future_module_state.header.stamp = ros::Time::now();
        future_module_state.position.x = L_mast * sin(future_state[0])+0.0957;
        future_module_state.position.y = L_mast * sin(future_state[1])-10;
        future_module_state.position.z = L_mast * cos(future_state[0]) * cos(future_state[1]);
        future_module_state.velocity.x = L_mast * future_state[2];
        future_module_state.velocity.y = L_mast * future_state[3];
        future_module_state.velocity.z = 0; //Not used, so not worth doing the calculations
        future_module_state_pub.publish(future_module_state);
*/

        ros::spinOnce();
        rate.sleep();
    }

}