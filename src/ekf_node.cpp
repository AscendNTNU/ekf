
// These must be defined before including TinyEKF.h
#define Nsta 6     // Nb states
#define Mobs 4     // Nb measurements
#define DEBUG false// print the matrices elements
#include "TinyEKF.h"



#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/DebugValue.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory


#define SAMPLE_TIME 25.0

geometry_msgs::PoseStamped module_pose;

const std::string ekf_output_path = std::string(get_current_dir_name()) + "/../../../ekf_output.txt";
const std::string reference_state_path = std::string(get_current_dir_name()) + "/../../../reference_state.txt";
//todo: it may be an issue calling this file the same name as in Fluid

//todo: should the title be an arrawy of char* ?
/**
 * @brief initialize the data file with a title. 
 * Should include the time as a first member
 * 
 * @param file_name name of the file to init
 * @param title array of names separated by tabulations
 */
void initLog(const std::string file_name, std::string title)
{ //create a header for the logfile.
    std::ofstream save_file_f;
     save_file_f.open(file_name);
    if(save_file_f.is_open())
    {
        save_file_f << title << std::endl;
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << file_name);
    }
}

/**
 * @brief save the given data into the file
 * 
 * @param file_name: the name of the file
 * @param data An array of the data to save
 * @param n the number of elements
 */
void saveLog(const std::string file_name, double* data, int n, int precision = 3)
{
    std::ofstream save_file_f;
    save_file_f.open (file_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(precision) //only 3 decimals
                    << ros::Time::now();
        for (int i =0; i<n;i++){
            save_file_f << "\t" << data[i];
        }
        save_file_f << std::endl;
        save_file_f.close();
    }
}

class Fuser : public TinyEKF {

    public:

        Fuser()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .005);
            this->setQ(1, 1, .005);
            this->setQ(2, 2, .04);
            this->setQ(3, 3, .04);
            this->setQ(4, 4, .001);
            this->setQ(5, 5, .0005);

            // Same for measurement noise
            this->setR(0, 0, .001667);
            this->setR(1, 1, .001667);
            this->setR(2, 2, .001667);
            this->setR(3, 3, .0001667);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            double p = this->x[0];
            double r = this->x[1];
            double pdot = this->x[2];
            double rdot = this->x[3];
            double w = this->x[4]; //omega = wave frequency
            double L = this->x[5]; //length of the mast
            
            // Process model is f(x) = x
            fx[0] = p + pdot/SAMPLE_TIME;
            fx[1] = r + rdot/SAMPLE_TIME;
            fx[2] = pdot - pow(w,2) * p/SAMPLE_TIME;
            fx[3] = rdot - pow(w,2) * r/SAMPLE_TIME;
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

void perceptionPoseCallback(geometry_msgs::PoseStampedConstPtr module_pose_ptr){
    module_pose = *module_pose_ptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ros::Time::init();
    ros::Rate rate(SAMPLE_TIME);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Starting up.");
    ros::NodeHandle node_handle;

    //subscribers
    ros::Subscriber module_pose_sub = node_handle.subscribe(
        "/simulator/module/noisy/pose", 10, &perceptionPoseCallback);

    //publishers
    ros::Publisher filtered_module_state_pub = 
            node_handle.advertise<mavros_msgs::PositionTarget>("/ekf/module/state",10);
    ros::Publisher ekf_state_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/state",10);
    ros::Publisher ekf_meas_pub = node_handle.advertise<mavros_msgs::DebugValue>("/ekf/measurement",10);

    initLog(ekf_output_path,"time\tpose_x\tpose_y\tpose_z\tvel_x\tvel_y\twave_freq\tmast_length");
    initLog(reference_state_path,"time\tpose_x\tpose_y\tpose_z"); //\vel_x\vel_y\vel_z");
    Fuser ekf;
    double X[Nsta];
    mavros_msgs::PositionTarget module_state;
    module_state.header.seq = 0;

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
    ekf.setX(4, 1);
    ekf.setX(5,2.5);

    //testing one step for debugging:
//    double z[Mobs]; // x, y, z, pitch
//    z[0] = 0.5;
//    z[1] = 0.5;
//    z[2] = 1.9;
//    z[3] = 0.25;
//    for(int i=0;i<200;i++){
//        ekf.step(z);
//        std::cout << "X = ";
//        for(int i =0; i< Nsta;i++){
//            std::cout << ekf.getX(i) << "\t"; // p, r, p', r', omega, L_mast
//        }
//        std::cout << std::endl;
//    }
    //result: X = 0.22613     0.189871        4.64347e-310    4.36543e-310    1       2.36156


    module_pose.header.seq = 0;
    //wait for first measurement
    while(module_pose.header.seq ==0 && ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Active.");
    
    while(ros::ok()){
        double z[Mobs]; // x, y, z, pitch
        z[0] = module_pose.pose.position.x-0.2; //mast offset
        z[1] = module_pose.pose.position.y+10.0;//mast offset
        z[2] = module_pose.pose.position.z;
        
        //extracting the pitch from the quaternion
        geometry_msgs::Quaternion quaternion = module_pose.pose.orientation;
        tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // If the quaternion is invalid, e.g. (0, 0, 0, 0), getRPY will return nan, so in that case we just set
        // it to zero.
        z[3] = std::isnan(pitch) ? 0.0 : pitch;

        if(!ekf.step(z))
            std::cout << "error with ekf step" <<std::endl;
            //todo: restart the kf whith current state ASAP
        
        for(int i =0; i< Nsta;i++)
            X[i] = ekf.getX(i); // p, r, p', r', omega, L_mast
        const double L_mast = X[5];
        
        ekf_state_vector.data.assign(X,X+Nsta);
        ekf_state_vector.header.seq++;
        ekf_state_pub.publish(ekf_state_vector);        

        ekf_meas_vector.data.assign(z,z+Mobs);
        ekf_meas_vector.header.seq++;
        ekf_meas_vector.header.stamp = ros::Time::now();
        ekf_meas_pub.publish(ekf_meas_vector);

        module_state.header.seq++; //seq is read only
        module_state.header.stamp = ros::Time::now();
        module_state.position.x = L_mast * sin(X[0]);
        module_state.position.y = L_mast * sin(X[1]);
        module_state.position.z = L_mast * cos(X[0]) * cos(X[1]);
        module_state.velocity.x = L_mast * X[2];
        module_state.velocity.y = L_mast * X[3];
        module_state.velocity.z = 0; //Not used, so not worth doing the calculations
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
        saveLog(ekf_output_path,ekf_output_data,7,4);

        std::cout << std::fixed << std::setprecision(4) << "X =\t";
        for(int i = 0 ; i<7 ; i++){
            std::cout << ekf_output_data[i] << "\t";
        }
        std::cout << std::endl;

        saveLog(reference_state_path,z,3,4);

        ros::spinOnce();
        rate.sleep();
    }

}