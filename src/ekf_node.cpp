
// These must be defined before including TinyEKF.h
#define Nsta 9     // Nb states : p, r, p', r', omega, L_mast, x0, y0, z0
#define Mobs 3     // Nb measurements x,y and z
#define DEBUG false// print the matrices elements
#include "TinyEKF.h"



#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/DebugValue.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory


#define SAMPLE_FREQUENCY 25.0

geometry_msgs::PoseWithCovarianceStamped module_pose;
bool start_ekf = false;
bool measurement_received;
mavros_msgs::PositionTarget gt_ref, gt_ref_prev;
geometry_msgs::PoseStamped header;

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            // p, r, p', r', omega, L_mast, x0, y0, z0
            this->setQ(0, 0, 0.0005);
            this->setQ(1, 1, 0.0005);
            this->setQ(2, 2, 0.00040);
            this->setQ(3, 3, 0.00040);
            this->setQ(4, 4, 0.0001); // omega
            this->setQ(5, 5, 0.0002); // L_mast
            this->setQ(6, 6, 0.00005); 
            this->setQ(7, 7, 0.00005);
            this->setQ(8, 8, 0.0001);

            // Same for measurement noise
            this->setR(0, 0, 2.667);
            this->setR(1, 1, 5.667);
            this->setR(2, 2, 5.667);
            
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
            double x0 = this->x[6];
            double y0 = this->x[7];
            double z0 = this->x[8];

            pddot = - pow(w,2) * p;
            rddot = - pow(w,2) * r;
            
            // Process model is f(x) = x_{k+1}
            fx[0] = p + pdot/SAMPLE_FREQUENCY;
            fx[1] = r + rdot/SAMPLE_FREQUENCY;
            fx[2] = pdot + pddot/SAMPLE_FREQUENCY;
            fx[3] = rdot + rddot/SAMPLE_FREQUENCY;
            fx[4] = w;
            fx[5] = L;
            fx[6] = x0;
            fx[7] = y0;
            fx[8] = z0;

            // So process model Jacobian is identity matrix
            for(int i = 0;i<Nsta;i++){
                F[i][i] = 1.0;
            }
            F[0][2] = 1.0/SAMPLE_FREQUENCY;
            F[1][3] = 1.0/SAMPLE_FREQUENCY;
            F[2][0] = - pow(w,2) / SAMPLE_FREQUENCY;
            F[3][1] = - pow(w,2) / SAMPLE_FREQUENCY;
            F[2][4] = - 2*w*p/SAMPLE_FREQUENCY;
            F[3][4] = - 2*w*r/SAMPLE_FREQUENCY;

            hx[0] = L * sin(fx[0]) + x0;
            hx[1] = L * sin(fx[1]) + y0;
            hx[2] = L * cos(fx[0]) * cos(fx[1]) + z0;

            // Jacobian of measurement function
            H[0][0] = L * cos(p) *  pdot; 
            H[1][1] = L * cos(r) * rdot;
            H[2][0] = - L * cos(r) * sin(p) * pdot;
            H[2][1] = - L * cos(p) * sin(r) * rdot;
            H[0][5] = sin(p);
            H[1][5] = sin(r);
            H[2][5] = cos(r)* cos(p);
            H[0][6] = 1.0;
            H[1][7] = 1.0;
            H[2][8] = 1.0;

        }
};

Fuser ekf;
ros::Time start_time;
geometry_msgs::TransformStamped transformStamped;
tf2_ros::Buffer tfBuffer;
bool use_perception;
double x0_min, x0_max, y0_min, y0_max;

void perceptionPoseCallback(geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr){
    module_pose = *module_pose_ptr;
    if (use_perception){
        try{
            transformStamped = tfBuffer.lookupTransform("map", module_pose.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.1).sleep();
        }
        tf2::doTransform(module_pose.pose.pose,module_pose.pose.pose,transformStamped);
    }

    geometry_msgs::Point mp = module_pose.pose.pose.position;
    measurement_received = true;

    if(mp.x > x0_max)
        x0_max = mp.x;
    if(mp.x < x0_min)
        x0_min = mp.x;
    if(mp.y > y0_max)
        y0_max = mp.y;
    if(mp.y < y0_min)
        y0_min = mp.y;
    

    if(!start_time.is_zero() && (ros::Time::now()-start_time).toSec() >= M_PI*2.0/ekf.getX(4) )
    {
        double x0_avg = (x0_max+x0_min)/2;
        double y0_avg = (y0_max+y0_min)/2;
//        ROS_INFO_STREAM("ekf: x0_avg=" << x0_avg
//            << "\ty0_avg=" << y0_avg
//            << "\txmin:" << x0_min << " xmax:" <<x0_max
//            << " ymin:" << y0_min << " ymax:" <<y0_max);
        ekf.setX(6,x0_avg);
        ekf.setX(7,y0_avg);
        //ekf.setX(8,z0_avg);
    }
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

void start_ekf_callback(geometry_msgs::Point mast_base)
{
    start_ekf = true;
    ekf.setX(6,mast_base.x);
    ekf.setX(7,mast_base.y);
    ekf.setX(8,mast_base.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ros::Time::init();
    ros::Rate rate(SAMPLE_FREQUENCY);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Initializing.");
    ros::NodeHandle node_handle;

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
    ros::Subscriber start_ekf_sub = node_handle.subscribe("/ekf/start", 1, &start_ekf_callback);
        
    //publishers
    ros::Publisher filtered_module_state_pub = 
            node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/state",10);
    //ros::Publisher future_module_state_pub = 
    //        node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/future_state",10);
    ros::Publisher ekf_state_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/state",10);
    ros::Publisher ekf_meas_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/measurement",10);

//    ros::Publisher tf_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/ekf/tf_pose",10);

    tf2_ros::TransformListener tfListener(tfBuffer);

    // extrema of the mast base position
    x0_min = std::numeric_limits<double>::infinity();
    x0_max = -std::numeric_limits<double>::infinity();
    y0_min = std::numeric_limits<double>::infinity();
    y0_max = -std::numeric_limits<double>::infinity();


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

    measurement_received = false;

    //wait for starting signal
    while(!start_ekf && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Starting up.");
    
    //wait for first measurement
    module_pose.header.seq = 0;
    while(module_pose.header.seq ==0 && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    //initiate first state vector
    ekf.setX(4, 0.55);  //todo, should be parameter?
    ekf.setX(5,module_pose.pose.pose.position.z);
    
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Active.");
    start_time = ros::Time::now();
    while(ros::ok()){
        ekf.prediction();
        if(measurement_received){
            double z[Mobs]; // x, y, z
            z[0] = module_pose.pose.pose.position.x;
            z[1] = module_pose.pose.pose.position.y;
            z[2] = module_pose.pose.pose.position.z;
            
            if(!ekf.update(z))
                std::cout << "error with ekf update" <<std::endl;
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
        ekf_state_vector.header.stamp = ros::Time::now();
        ekf_state_pub.publish(ekf_state_vector);        

        module_state.header.seq++; //seq is read only
        module_state.header.stamp = ros::Time::now();
        module_state.position.x = L_mast * sin(X[0]) + X[6];
        module_state.position.y = L_mast * sin(X[1]) + X[7];
        module_state.position.z = L_mast * cos(X[0]) * cos(X[1]) + X[8];
        module_state.velocity.x = L_mast * X[2]*cos(X[0]);
        module_state.velocity.y = L_mast * X[3]*cos(X[1]);
        module_state.velocity.z = 0; //Not used, so not worth doing the calculations //todo: can be used
        module_state.acceleration_or_force.x = L_mast *(X[2]*X[2]*sin(X[0]) + ekf.pddot*cos(X[0]));
        module_state.acceleration_or_force.y = L_mast *(X[3]*X[3]*sin(X[1]) + ekf.rddot*cos(X[1]));
        module_state.acceleration_or_force.z = 0; //Not used, so not worth doing the calculations
        filtered_module_state_pub.publish(module_state);
        
        #if DEBUG
        //save some data:
        double ekf_output_data[7];
        ekf_output_data[0] = module_state.position.x;
        ekf_output_data[1] = module_state.position.y;
        ekf_output_data[2] = module_state.position.z;
        ekf_output_data[3] = module_state.velocity.x;
        ekf_output_data[4] = module_state.velocity.y;
        ekf_output_data[5] = X[4];
        ekf_output_data[6] = X[5];

        
        std::cout << std::fixed << std::setprecision(4) << "X =\t";
        for(int i = 0 ; i<7 ; i++){
            std::cout << ekf_output_data[i] << "\t";
        }
        std::cout << std::endl;
        #endif
        
        ros::spinOnce();
        rate.sleep();
    }

}
