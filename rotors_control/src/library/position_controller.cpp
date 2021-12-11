/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/position_controller.h"
#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/sensfusion6.h"

#include <math.h>
#include <ros/ros.h>
#include <time.h>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

#include <inttypes.h>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>


#define M_PI                                     3.14159265358979323846  /* pi [rad]*/
#define OMEGA_OFFSET                             6874  /* OMEGA OFFSET [PWM]*/
#define ANGULAR_MOTOR_COEFFICIENT                0.2685 /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT                         426.24 /* MOTORS_INTERCEPT [rad/s]*/
#define MAX_PROPELLERS_ANGULAR_VELOCITY          2618 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/
#define MAX_R_DESIDERED                          3.4907 /* MAX R DESIDERED VALUE [rad/s]*/
#define MAX_THETA_COMMAND                        0.5236 /* MAX THETA COMMMAND [rad]*/
#define MAX_PHI_COMMAND                          0.5236 /* MAX PHI COMMAND [rad]*/
#define MAX_POS_DELTA_OMEGA                      1289 /* MAX POSITIVE DELTA OMEGA [PWM]*/
#define MAX_NEG_DELTA_OMEGA                      -1718 /* MAX NEGATIVE DELTA OMEGA [PWM]*/
#define SAMPLING_TIME                            0.01 /* SAMPLING TIME [s] */

namespace rotors_control{

PositionController::PositionController()
    : controller_active_(false),
    state_estimator_active_(false),
    dataStoring_active_(false),
    dataStoringTime_(0),
    phi_command_ki_(0),
    theta_command_ki_(0),
    p_command_ki_(0),
    q_command_ki_(0),
    r_command_ki_(0),
    delta_psi_ki_(0),
    delta_omega_ki_(0),
    hovering_gain_kd_(0){

      // The control variables are initialized to zero
      control_t_.roll = 0;
      control_t_.pitch = 0;
      control_t_.yawRate = 0;
      control_t_.thrust = 0;

      state_.angularAcc.x = 0; // Angular Acceleration x
      state_.angularAcc.y = 0; // Angular Acceleration y
      state_.angularAcc.z = 0; // Angular Acceleration z

      state_.attitude.roll = 0; // Roll
      state_.attitude.pitch = 0; // Pitch
      state_.attitude.yaw = 0; // Yaw

      state_.position.x = 0; // Position.x
      state_.position.y = 0; // Position.y
      state_.position.z = 0; // Position.z

      state_.angularVelocity.x = 0; // Angular velocity x
      state_.angularVelocity.y = 0; // Angular velocity y
      state_.angularVelocity.z = 0; // Angular velocity z

      state_.linearVelocity.x = 0; //Linear velocity x
      state_.linearVelocity.y = 0; //Linear velocity y
      state_.linearVelocity.z = 0; //Linear velocity z

      state_.attitudeQuaternion.x = 0; // Quaternion x
      state_.attitudeQuaternion.y = 0; // Quaternion y
      state_.attitudeQuaternion.z = 0; // Quaternion z
      state_.attitudeQuaternion.w = 0; // Quaternion w

}

PositionController::~PositionController() {}

//The callback saves data come from simulation into csv files
void PositionController::CallbackSaveData(const ros::TimerEvent& event){

      if(!dataStoring_active_){
         return;
      }

      ofstream filePropellersVelocity;
      ofstream fileDroneAttiude;
      ofstream filePWM;
      ofstream filePWMComponents;
      ofstream fileCommandAttiude;
      ofstream fileRCommand;
      ofstream fileOmegaCommand;
      ofstream fileXeYe;
      ofstream fileDeltaCommands;
      ofstream filePQCommands;
      ofstream fileDronePosition;

      ROS_INFO("CallbackSavaData function is working. Time: %f seconds, %f nanoseconds", odometry_.timeStampSec, odometry_.timeStampNsec);

      filePropellersVelocity.open("/home/" + user_ + "/PropellersVelocity.csv", std::ios_base::app);
      fileDroneAttiude.open("/home/" + user_ + "/DroneAttiude.csv", std::ios_base::app);
      filePWM.open("/home/" + user_ + "/PWM.csv", std::ios_base::app);
      filePWMComponents.open("/home/" + user_ + "/PWMComponents.csv", std::ios_base::app);
      fileCommandAttiude.open("/home/" + user_ + "/CommandAttitude.csv", std::ios_base::app);
      fileRCommand.open("/home/" + user_ + "/RCommand.csv", std::ios_base::app);
      fileOmegaCommand.open("/home/" + user_ + "/OmegaCommand.csv", std::ios_base::app);
      fileXeYe.open("/home/" + user_ + "/XeYe.csv", std::ios_base::app);
      fileDeltaCommands.open("/home/" + user_ + "/DeltaCommands.csv", std::ios_base::app);
      filePQCommands.open("/home/" + user_ + "/PQCommands.csv", std::ios_base::app);
      fileDronePosition.open("/home/" + user_ + "/DronePosition.csv", std::ios_base::app);

      // Saving control signals in a file
      for (unsigned n=0; n < listPropellersVelocity_.size(); ++n) {
          filePropellersVelocity << listPropellersVelocity_.at( n );
      }

      for (unsigned n=0; n < listDroneAttiude_.size(); ++n) {
          fileDroneAttiude << listDroneAttiude_.at( n );
      }

      for (unsigned n=0; n < listPWM_.size(); ++n) {
          filePWM << listPWM_.at( n );
      }

      for (unsigned n=0; n < listPWMComponents_.size(); ++n) {
          filePWMComponents << listPWMComponents_.at( n );
      }

      for (unsigned n=0; n < listCommandAttiude_.size(); ++n) {
          fileCommandAttiude << listCommandAttiude_.at( n );
      }

      for (unsigned n=0; n < listRCommand_.size(); ++n) {
          fileRCommand << listRCommand_.at( n );
      }

      for (unsigned n=0; n < listOmegaCommand_.size(); ++n) {
          fileOmegaCommand << listOmegaCommand_.at( n );
      }

      for (unsigned n=0; n < listXeYe_.size(); ++n) {
          fileXeYe << listXeYe_.at( n );
      }

      for (unsigned n=0; n < listDeltaCommands_.size(); ++n) {
          fileDeltaCommands << listDeltaCommands_.at( n );
      }

      for (unsigned n=0; n < listPQCommands_.size(); ++n) {
          filePQCommands << listPQCommands_.at( n );
      }

      for (unsigned n=0; n < listDronePosition_.size(); ++n) {
          fileDronePosition << listDronePosition_.at( n );
      }

      // Closing all opened files
      filePropellersVelocity.close();
      fileDroneAttiude.close();
      filePWM.close();
      filePWMComponents.close();
      fileCommandAttiude.close();
      fileRCommand.close();
      fileOmegaCommand.close();
      fileXeYe.close();
      fileDeltaCommands.close();
      filePQCommands.close();
      fileDronePosition.close();

      // To have a one shot storing
      dataStoring_active_ = false;

}

// Reading parameters come frame launch file
void PositionController::SetLaunchFileParameters(){

	// The boolean variable is used to inactive the logging if it is not useful
	if(dataStoring_active_){

		// Time after which the data storing function is turned on
		timer_ = n_.createTimer(ros::Duration(dataStoringTime_), &PositionController::CallbackSaveData, this, false, true);

		// Cleaning the string vector contents
    listPropellersVelocity_.clear();
    listDroneAttiude_.clear();
    listPWM_.clear();
    listPWMComponents_.clear();
    listCommandAttiude_.clear();
    listRCommand_.clear();
    listOmegaCommand_.clear();
    listXeYe_.clear();
    listDeltaCommands_.clear();
    listPQCommands_.clear();
    listDronePosition_.clear();

	}

}

// Controller gains are entered into local global variables
void PositionController::SetControllerGains(){

      xy_gain_kp_ = Eigen::Vector2f(controller_parameters_.xy_gain_kp_.x(), controller_parameters_.xy_gain_kp_.y());
      xy_gain_ki_ = Eigen::Vector2f(controller_parameters_.xy_gain_ki_.x(), controller_parameters_.xy_gain_ki_.y());

      attitude_gain_kp_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(), controller_parameters_.attitude_gain_kp_.y());
      attitude_gain_ki_ = Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(), controller_parameters_.attitude_gain_ki_.y());

      rate_gain_kp_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(), controller_parameters_.rate_gain_kp_.y(), controller_parameters_.rate_gain_kp_.z());
      rate_gain_ki_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(), controller_parameters_.rate_gain_ki_.y(), controller_parameters_.rate_gain_ki_.z());

      yaw_gain_kp_ = controller_parameters_.yaw_gain_kp_;
      yaw_gain_ki_ = controller_parameters_.yaw_gain_ki_;

      hovering_gain_kp_ = controller_parameters_.hovering_gain_kp_;
      hovering_gain_ki_ = controller_parameters_.hovering_gain_ki_;
      hovering_gain_kd_ = controller_parameters_.hovering_gain_kd_;

}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
    command_trajectory_= command_trajectory;
    controller_active_= true;
}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);

    // This is to disable the controller if we do not receive a trajectory
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }

    double PWM_1, PWM_2, PWM_3, PWM_4;
    ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);

    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);
    omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT);

    //The omega values are saturated considering physical constraints of the system
    if(!(omega_1 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_1 > 0))
        if(omega_1 > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega_1 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
           omega_1 = 0;

    if(!(omega_2 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_2 > 0))
        if(omega_2 > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega_2 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
           omega_2 = 0;

    if(!(omega_3 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_3 > 0))
        if(omega_3 > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega_3 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
           omega_3 = 0;

    if(!(omega_4 < MAX_PROPELLERS_ANGULAR_VELOCITY && omega_4 > 0))
        if(omega_4 > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega_4 = MAX_PROPELLERS_ANGULAR_VELOCITY;
        else
           omega_4 = 0;

   if(dataStoring_active_){
     // Saving drone attitude in a file
     std::stringstream tempPropellersVelocity;
     tempPropellersVelocity << omega_1 << "," << omega_2 << "," << omega_3 << "," << omega_4 << ","
             << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

     listPropellersVelocity_.push_back(tempPropellersVelocity.str());
   }

    ROS_DEBUG("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega_1, omega_2, omega_3, omega_4);
    *rotor_velocities = Eigen::Vector4d(omega_1, omega_2, omega_3, omega_4);
}

void PositionController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);

}

void PositionController::rot_wb(float x[3], const float rpy[3], bool inv)
{
    float R_wb[9];
    float g_R_wb_tmp[9];
    float h_R_wb_tmp[9];
    float b_R_wb[3];
    float R_wb_tmp;
    float b_R_wb_tmp;
    float c_R_wb_tmp;
    float d_R_wb_tmp;
    float e_R_wb_tmp;
    float f_R_wb_tmp;
    int i;
    int i1;
    int i2;
    R_wb_tmp = (float)sin(rpy[2]);
    b_R_wb_tmp = (float)cos(rpy[2]);
    c_R_wb_tmp = (float)sin(rpy[1]);
    d_R_wb_tmp = (float)cos(rpy[1]);
    e_R_wb_tmp = (float)sin(rpy[0]);
    f_R_wb_tmp = (float)cos(rpy[0]);
    g_R_wb_tmp[0] = b_R_wb_tmp;
    g_R_wb_tmp[3] = -R_wb_tmp;
    g_R_wb_tmp[6] = 0.0F;
    g_R_wb_tmp[1] = R_wb_tmp;
    g_R_wb_tmp[4] = b_R_wb_tmp;
    g_R_wb_tmp[7] = 0.0F;
    R_wb[0] = d_R_wb_tmp;
    R_wb[3] = 0.0F;
    R_wb[6] = -c_R_wb_tmp;
    g_R_wb_tmp[2] = 0.0F;
    R_wb[1] = 0.0F;
    g_R_wb_tmp[5] = 0.0F;
    R_wb[4] = 1.0F;
    g_R_wb_tmp[8] = 1.0F;
    R_wb[7] = 0.0F;
    R_wb[2] = c_R_wb_tmp;
    R_wb[5] = 0.0F;
    R_wb[8] = d_R_wb_tmp;
    for (i = 0; i < 3; i++)
    {
        b_R_wb_tmp = g_R_wb_tmp[i];
        c_R_wb_tmp = g_R_wb_tmp[i + 3];
        i1 = (int)g_R_wb_tmp[i + 6];
        for (i2 = 0; i2 < 3; i2++)
        {
            h_R_wb_tmp[i + 3 * i2] =
                (b_R_wb_tmp * R_wb[3 * i2] + c_R_wb_tmp * R_wb[3 * i2 + 1]) +
                (float)i1 * R_wb[3 * i2 + 2];
        }
    }
    g_R_wb_tmp[0] = 1.0F;
    g_R_wb_tmp[3] = 0.0F;
    g_R_wb_tmp[6] = 0.0F;
    g_R_wb_tmp[1] = 0.0F;
    g_R_wb_tmp[4] = f_R_wb_tmp;
    g_R_wb_tmp[7] = -e_R_wb_tmp;
    g_R_wb_tmp[2] = 0.0F;
    g_R_wb_tmp[5] = e_R_wb_tmp;
    g_R_wb_tmp[8] = f_R_wb_tmp;
    for (i = 0; i < 3; i++)
    {
        b_R_wb_tmp = h_R_wb_tmp[i];
        c_R_wb_tmp = h_R_wb_tmp[i + 3];
        R_wb_tmp = h_R_wb_tmp[i + 6];
        for (i1 = 0; i1 < 3; i1++)
        {
            R_wb[i + 3 * i1] = (b_R_wb_tmp * g_R_wb_tmp[3 * i1] +
                                c_R_wb_tmp * g_R_wb_tmp[3 * i1 + 1]) +
                               R_wb_tmp * g_R_wb_tmp[3 * i1 + 2];
        }
    }
    if (inv)
    {
        for (i = 0; i < 3; i++)
        {
            b_R_wb[i] = (R_wb[i] * x[0] + R_wb[i + 3] * x[1]) + R_wb[i + 6] * x[2];
        }
        x[0] = b_R_wb[0];
        x[1] = b_R_wb[1];
        x[2] = b_R_wb[2];
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            b_R_wb[i] = (R_wb[3 * i] * x[0] + R_wb[3 * i + 1] * x[1]) +
                        R_wb[3 * i + 2] * x[2];
        }
        x[0] = b_R_wb[0];
        x[1] = b_R_wb[1];
        x[2] = b_R_wb[2];
    }
}

void PositionController::mypid(const float x[12], const float xd[12], float u[4])
{
    double cosyaw = cosf(x[5]);
    double sinyaw = sinf(x[5]);
    //double body_x_set = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
    //double body_y_set = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;
    // Desired
    double body_x_set = 0;
    double body_y_set = 0;
    
    // Measured
    double body_x_state = x[0] * cosyaw + x[1] * sinyaw;
    double body_y_state = -x[0] * sinyaw + x[1] * cosyaw;
    
    double body_vx_state = x[6] * cosyaw + x[7] * sinyaw;
    double body_vy_state = -x[6] * sinyaw + x[7] * cosyaw;
    
    //Error = Desired - Measured
    double error_x = body_x_set - body_x_state;
    double error_y = body_y_set - body_y_state;
 
    double kp_xy = 0.01;
    double kd_xy = 0.001;
    //double ki_xy = 0.01;
    
    double kp_v_xy = 0.6;
    double kd_v_xy = 0.0001;
    double ki_v_xy = 0.002;
    
    double v_x = kp_xy * error_x + kd_xy * body_vx_state;
    double v_y = kp_xy * error_y + kd_xy * body_vy_state;
    
     

    double error_vx = v_x - body_vx_state;
    double error_vy = v_y - body_vy_state;
    double acc_x = 0;
    double acc_y = 0;
    
    double pitch_set = -(kp_v_xy * error_vx + acc_x * kd_v_xy + error_x * ki_v_xy);
    double roll_set = -(kp_v_xy * error_vy + acc_y * kd_v_xy + error_y * ki_v_xy);
    
    // double pitch_set = fminf(pitch_Limit, fmaxf(-pitch_Limit, pitch_set));
    // double roll_set = fminf(roll_Limit, fmaxf(-roll_Limit, roll_set));
 
 
 
    int count = 0;
    // Hovering height
    double zd;
    
    zd = 0.25;
    
 
    // Desired states
    double xdd[12] = {0, 0, 0.25, roll_set, pitch_set, 0, 0, 0, 0, 0, 0, 0};
    
    
    
    // PD
    static const double a[48] = {0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.4795307692307692,
                                0.4795307692307692,
                                0.4795307692307692,
                                0.4795307692307692,
                                -0.076923076923076927,
                                -0.076923076923076927,
                                0.076923076923076927,
                                0.076923076923076927,
                                0.076923076923076927,
                                -0.076923076923076927,
                                -0.076923076923076927,
                                0.076923076923076927,
                                -6.5217391304347834E-5,
                                6.5217391304347834E-5,
                                -6.5217391304347834E-5,
                                6.5217391304347834E-5,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.42776153846153842,
                                0.42776153846153842,
                                0.42776153846153842,
                                0.42776153846153842,
                                -0.015384615384615384,
                                -0.015384615384615384,
                                0.015384615384615384,
                                0.015384615384615384,
                                -0.015384615384615384,
                                0.015384615384615384,
                                0.015384615384615384,
                                -0.015384615384615384,
                                -0.038043478260869568,
                                0.038043478260869568,
                                -0.038043478260869568,
                                0.038043478260869568};
 

 
    double b_x[12];
    double d;
    int i;
    int k;
 
    for (i = 0; i < 12; i++) {
    b_x[i] = x[i] - xdd[i];
    }

    float f;
    for (k = 0; k < 4; k++)
    {
        f = 0.0F;
        for (i = 0; i < 12; i++)
        {
            f += a[k + (i << 2)] * b_x[i];
        }
        u[k] = ((float)sqrt(8.60953E-12F -
                            3.69664E-9F * (0.0023F - (0.0662175044F - f))) +
                -2.9342E-6F) /
               1.84832E-9F;
    }
}

void PositionController::mygs(const float x[12], const float xd[12], float u[4])
{
    static const double dv[48] = {
      -0.0111650626931185,  0.0111650626931062,   0.0111650626930822,
      -0.0111650626931557,  0.0111650626931383,   0.0111650626931322,
      -0.0111650626931444,  -0.0111650626931354,  0.160365685385961,
      0.160365685386052,    0.160365685385978,    0.160365685385899,
      -0.0275715937900411,  -0.0275715937900256,  0.0275715937900418,
      0.0275715937900276,   0.0275715937900451,   -0.0275715937899942,
      -0.0275715937899778,  0.027571593790037,    -0.0639185163509148,
      0.0639185163509139,   -0.0639185163508666,  0.063918516350876,
      -0.00792634823244657, 0.00792634823242948,  0.00792634823242092,
      -0.00792634823245076, 0.00792634823244899,  0.00792634823244369,
      -0.00792634823245077, -0.00792634823244466, 0.197452751726173,
      0.197452751726109,    0.197452751726102,    0.197452751726184,
      -0.00510298076889898, -0.00510298076889909, 0.00510298076889899,
      0.00510298076889914,  -0.00510298076889968, 0.00510298076889819,
      0.00510298076889786,  -0.00510298076889965, -0.035357366737476,
      0.0353573667374777,   -0.0353573667374742,  0.0353573667374761};
  static const double dv1[48] = {
      -0.00940105694201515, 0.00940105694205099,  0.0126696987305664,
      -0.0126696987305942,  0.0103817344882129,   0.0103817344882098,
      -0.0118747382445464,  -0.0118747382445436,  0.17026394420991,
      0.170263944209884,    0.150092801695412,    0.150092801695466,
      -0.0504208350929702,  -0.0504208350929717,  0.0185216144349851,
      0.0185216144349881,   0.0142447488016289,   -0.014244748801672,
      -0.0451974203955572,  0.0451974203955832,   -0.0762305569172367,
      0.0762305569172061,   -0.0419699480959135,  0.0419699480959242,
      -0.00622527549248678, 0.00622527549250504,  0.0105106225212333,
      -0.010510622521246,   0.00815243589581417,  0.00815243589581112,
      -0.00923502088537514, -0.00923502088537025, 0.210390918734696,
      0.210390918734695,    0.184345800194826,    0.184345800194817,
      -0.00554341584920538, -0.00554341584920612, 0.00504582254120244,
      0.00504582254120338,  -0.00528455732220759, 0.00528455732220858,
      0.00524143712681133,  -0.00524143712681208, -0.0354508768911719,
      0.0354508768911698,   -0.0350428368630403,  0.0350428368630401};
  static const double dv2[48] = {
      -0.0126696987303286,  0.0126696987304214,   0.00940105694178478,
      -0.009401056941884,   0.0118747382445779,   0.0118747382445801,
      -0.0103817344882993,  -0.0103817344883011,  0.150092801695422,
      0.150092801695423,    0.170263944210058,    0.17026394421007,
      -0.01852161443503,    -0.018521614435045,   0.0504208350930819,
      0.0504208350930918,   0.045197420395134,    -0.0451974203952517,
      -0.0142447488012465,  0.014244748801371,    -0.0419699480954315,
      0.0419699480955351,   -0.0762305569176283,  0.0762305569175202,
      -0.010510622521088,   0.0105106225211382,   0.006225275492344,
      -0.00622527549239666, 0.00923502088539085,  0.00923502088539276,
      -0.00815243589586113, -0.00815243589586266, 0.184345800194791,
      0.184345800194787,    0.210390918734764,    0.210390918734771,
      -0.00504582254120505, -0.00504582254120821, 0.00554341584921044,
      0.00554341584921234,  -0.00524143712680108, 0.00524143712680355,
      0.00528455732219843,  -0.00528455732220113, -0.035042836863031,
      0.035042836863037,    -0.0354508768911782,  0.0354508768911724};
  static const double dv3[48] = {
      -0.0118747382445054,  0.0103817344881986,   0.0103817344881673,
      -0.0118747382445268,  0.0111650657561732,   0.0111650596300287,
      -0.011165059630018,   -0.0111650657561889,  0.150092801695259,
      0.170263944210183,    0.170263944210059,    0.150092801695108,
      -0.0275715929052743,  -0.0275715946747103,  0.0275715946746953,
      0.0275715929052929,   0.0185216144349259,   -0.050420835092957,
      -0.0504208350929074,  0.0185216144349585,   -0.0710504775747472,
      0.0567784048105633,   -0.0567784048105531,  0.0710504775747457,
      -0.00923502088534701, 0.00815243589580935,  0.00815243589578457,
      -0.00923502088536218, 0.00792635196781833,  0.00792634449703705,
      -0.00792634449703087, -0.00792635196782647, 0.184345800194817,
      0.210390918734714,    0.210390918734702,    0.184345800194814,
      -0.00510298072180822, -0.00510298081598847, 0.00510298081598739,
      0.00510298072180931,  -0.0050458225411996,  0.00554341584920403,
      0.00554341584920266,  -0.00504582254120041, -0.035444685661213,
      0.0354446865471318,   -0.0354446865471285,  0.0354446856612107};
  static const double dv4[48] = {
      -0.0103817344882524,  0.0118747382445619,   0.0118747382445674,
      -0.0103817344882406,  0.0111650596299326,   0.0111650657559562,
      -0.0111650657559585,  -0.0111650596299355,  0.170263944210163,
      0.150092801695379,    0.150092801695281,    0.170263944210106,
      -0.0275715946746306,  -0.027571592905055,   0.02757159290506,
      0.0275715946746337,   0.0504208350929981,   -0.0185216144349833,
      -0.0185216144349951,  0.0504208350929837,   -0.0567784048106365,
      0.0710504775746155,   -0.0710504775746134,  0.0567784048106304,
      -0.00815243589582226, 0.00923502088537229,  0.00923502088537596,
      -0.00815243589581847, 0.00792634449699977,  0.00792635196772164,
      -0.00792635196772311, -0.00792634449700156, 0.21039091873473,
      0.18434580019482,     0.184345800194804,    0.210390918734729,
      -0.0051029808159881,  -0.00510298072180124, 0.0051029807218017,
      0.00510298081598784,  -0.00554341584920758, 0.00504582254120348,
      0.00504582254120377,  -0.00554341584920712, -0.0354446865471625,
      0.0354446856611786,   -0.0354446856611767,  0.0354446865471628};
  static const double dv5[48] = {
      -0.0104903395817337,  0.00831410962784078,  0.0122059140513534,
      -0.0129976061414834,  0.00993629467690649,  0.0107436788098404,
      -0.0117200267403416,  -0.0120901897580497,  0.160142848360382,
      0.180306461758212,    0.16034661747866,     0.139007461177192,
      -0.0437621214973809,  -0.0549883752160027,  0.0123929707597385,
      0.0239581016078723,   -0.00213005662961312, -0.0403936020036214,
      -0.0609274464791453,  0.0415196896632677,   -0.0843102440443002,
      0.0670915131527468,   -0.0349101678140205,  0.050237576572593,
      -0.00699046769728887, 0.0064494622628459,   0.0100799252870341,
      -0.0118972330236247,  0.0071119004122831,   0.00885881808705521,
      -0.00841935429716088, -0.00994396280968681, 0.197670563959638,
      0.22341233048739,     0.198027530422644,    0.169800257057204,
      -0.00536619880426642, -0.00566739340024127, 0.00487210193674952,
      0.00518645822560954,  -0.00503580664613324, 0.0058337102040946,
      0.00548319807660084,  -0.00530102608142222, -0.0355769890928321,
      0.0354900705430951,   -0.0351122405775914,  0.0351469268208095};
  static const double dv6[48] = {
      -0.0129976061413916,  0.0122059140512601,   0.00831410962778871,
      -0.0104903395816753,  0.0120901897581081,   0.0117200267403979,
      -0.0107436788099306,  -0.00993629467697541, 0.139007461177171,
      0.160346617478589,    0.1803064617582,      0.160142848360391,
      -0.0239581016079093,  -0.0123929707598002,  0.054988375216068,
      0.0437621214974463,   0.0415196896631476,   -0.0609274464790252,
      -0.0403936020035464,  -0.0021300566297073,  -0.0502375765724269,
      0.0349101678139681,   -0.067091513152852,   0.0843102440443581,
      -0.0118972330235812,  0.0100799252869774,   0.00644946226282516,
      -0.00699046769724768, 0.00994396280970773,  0.00841935429719774,
      -0.00885881808709192, -0.00711190041231987, 0.169800257057168,
      0.198027530422609,    0.223412330487434,    0.197670563959696,
      -0.00518645822561121, -0.00487210193675017, 0.00566739340024491,
      0.0053661988042678,   -0.00530102608142023, 0.0054831980765975,
      0.00583371020409327,  -0.00503580664612985, -0.0351469268208057,
      0.0351122405775965,   -0.0354900705430998,  0.0355769890928338};
  static const double dv7[48] = {
      -0.00831410962781531, 0.010490339581741,    0.0129976061414328,
      -0.0122059140513068,  0.0107436788099754,   0.00993629467696233,
      -0.0120901897581532,  -0.011720026740374,   0.18030646175821,
      0.1601428483604,      0.139007461177114,    0.160346617478513,
      -0.0549883752161358,  -0.0437621214974236,  0.0239581016079678,
      0.0123929707597599,   0.040393602003605,    0.00213005662960888,
      -0.0415196896632195,  0.0609274464791055,   -0.0670915131527988,
      0.0843102440442684,   -0.0502375765724622,  0.0349101678140299,
      -0.00644946226284635, 0.00699046769728607,  0.0118972330236111,
      -0.0100799252870098,  0.00885881808712151,  0.00711190041230838,
      -0.00994396280973748, -0.00841935429717808, 0.22341233048744,
      0.197670563959722,    0.169800257057159,    0.198027530422549,
      -0.00566739340024645, -0.00536619880426725, 0.00518645822561122,
      0.00487210193674911,  -0.00583371020409658, 0.00503580664613299,
      0.00530102608142526,  -0.0054831980765997,  -0.035490070543103,
      0.0355769890928289,   -0.0351469268208079,  0.0351122405775999};
  static const double dv8[48] = {
      -0.0122059140512444,  0.0129976061413951,   0.0104903395816912,
      -0.00831410962784375, 0.011720026740411,    0.0120901897581559,
      -0.00993629467701592, -0.0107436788099592,  0.160346617478649,
      0.139007461177104,    0.16014284836038,     0.180306461758307,
      -0.0123929707598239,  -0.0239581016079794,  0.0437621214974947,
      0.0549883752161176,   0.0609274464789975,   -0.0415196896631789,
      0.0021300566297129,   0.0403936020036441,   -0.0349101678138641,
      0.0502375765725235,   -0.0843102440444155,  0.0670915131527158,
      -0.0100799252869676,  0.0118972330235916,   0.00699046769724906,
      -0.00644946226286094, 0.00841935429720893,  0.00994396280973618,
      -0.00711190041234363, -0.00885881808711406, 0.198027530422671,
      0.16980025705713,     0.197670563959625,    0.223412330487464,
      -0.00487210193675446, -0.00518645822561585, 0.00536619880427252,
      0.00566739340024699,  -0.00548319807659703, 0.00530102608141668,
      0.0050358066461296,   -0.00583371020409382, -0.0351122405775764,
      0.0351469268207991,   -0.0355769890928471,  0.035490070543099};
  double K[48];
  double b_x[12];
  double d;

  int i;
  int k;
  memset(&K[0], 0, 48U * sizeof(double));
  d = fabs(x[3]);
  if ((d <= 0.39269908169872414) && (fabs(x[4]) <= 0.39269908169872414)) {
    memcpy(&K[0], &dv[0], 48U * sizeof(double));
  } else if ((x[3] > 0.39269908169872414) &&
             (fabs(x[4]) < 0.39269908169872414)) {
    memcpy(&K[0], &dv1[0], 48U * sizeof(double));
  } else if ((x[3] < -0.39269908169872414) &&
             (fabs(x[4]) < 0.39269908169872414)) {
    memcpy(&K[0], &dv2[0], 48U * sizeof(double));
  } else if ((d < 0.39269908169872414) && (x[4] > 0.39269908169872414)) {
    memcpy(&K[0], &dv3[0], 48U * sizeof(double));
  } else if ((d < 0.39269908169872414) && (x[4] < -0.39269908169872414)) {
    memcpy(&K[0], &dv4[0], 48U * sizeof(double));
  } else if ((x[3] > 0.39269908169872414) && (x[4] > 0.39269908169872414)) {
    memcpy(&K[0], &dv5[0], 48U * sizeof(double));
  } else if ((x[3] < -0.39269908169872414) && (x[4] > 0.39269908169872414)) {
    memcpy(&K[0], &dv6[0], 48U * sizeof(double));
  } else if ((x[3] > 0.39269908169872414) && (x[4] < -0.39269908169872414)) {
    memcpy(&K[0], &dv7[0], 48U * sizeof(double));
  } else if ((x[3] < -0.39269908169872414) && (x[4] < -0.39269908169872414)) {
    memcpy(&K[0], &dv8[0], 48U * sizeof(double));
  }
  float f;

  
    for (i = 0; i < 12; i++)
    {
        b_x[i] = x[i] - xd[i];
    }
    for (k = 0; k < 4; k++)
    {
        f = 0.0F;
        for (i = 0; i < 12; i++)
        {
            f += K[k + (i << 2)] * b_x[i];
        }
        u[k] = ((float)sqrt(8.60953E-12F -
                            3.69664E-9F * (0.0023F - (0.0662175044F - f))) +
                -2.9342E-6F) /
               1.84832E-9F;
    }



}
void PositionController::mylqr(const float x[12], const float xd[12], float u[4])
{
    static const float a[48] = {
        -0.0011179F, 0.0011179F, 0.0011179F, -0.0011179F,
        0.0011179F, 0.0011179F, -0.0011179F, -0.0011179F,
        0.771970034F, 0.771970034F, 0.771970034F, 0.771970034F,
        -0.0294773746F, -0.0294773746F, 0.0294773746F, 0.0294773746F,
        0.0294773746F, -0.0294773746F, -0.0294773746F, 0.0294773746F,
        -0.271381974F, 0.271381974F, -0.271381974F, 0.271381974F,
        0.0103843957F, 0.0103843957F, -0.0103843957F, -0.0103843957F,
        0.0103843957F, -0.0103843957F, -0.0103843957F, 0.0103843957F,
        0.140915751F, 0.140915751F, 0.140915751F, 0.140915751F,
        -0.0160540417F, -0.0160540417F, 0.0160540417F, 0.0160540417F,
        -0.0160540417F, 0.0160540417F, 0.0160540417F, -0.0160540417F,
        -0.147800714F, 0.147800714F, -0.147800714F, 0.147800714F};
    float b_x[12];
    float f;
    int i;
    int k;
    /*  Real */
    /*  a = 2.130295 * 1e-11; */
    /*  b = 1.032633 * 1e-6; */
    /*  c = 5.484560 * 1e-4 - u; */
    /*  Sim */
    for (i = 0; i < 12; i++)
    {
        b_x[i] = x[i] - xd[i];
    }
    for (k = 0; k < 4; k++)
    {
        f = 0.0F;
        for (i = 0; i < 12; i++)
        {
            f += a[k + (i << 2)] * b_x[i];
        }
        u[k] = ((float)sqrt(8.60953E-12F -
                            3.69664E-9F * (0.0023F - (0.0662175044F - f))) +
                -2.9342E-6F) /
               1.84832E-9F;
    }
}

void PositionController::mylqr_pfl(const float x[12], const float xd[12], float u[4])
{
    static const float a[48] = {
        -0.0F, -0.0F, 0.722209454F, -0.0F, -0.0F, 0.722209454F,
        -0.0F, -0.0F, -114.365929F, -0.0F, -0.0F, -0.0F,
        -0.0F, -193.310883F, -0.0F, -0.0F, -0.0F, -0.0F,
        -193.310883F, -0.0F, -0.0F, -0.0F, -0.0F, -193.310883F,
        -0.0F, -0.0F, 6.71274519F, -0.0F, -0.0F, 6.71274519F,
        -0.0F, -0.0F, -20.8764095F, -0.0F, -0.0F, -0.0F,
        -0.0F, -105.281456F, -0.0F, -0.0F, -0.0F, -0.0F,
        -105.281456F, -0.0F, -0.0F, -0.0F, -0.0F, -105.281456F};
    float xnew[12];
    float J_wb_b[9];
    float Y_idx_1;
    float t15;
    float t16;
    float t18;
    float t2;
    float t20;
    float t21;
    float t23;
    float t26;
    float t28;
    float t3;
    float t31;
    float t31_tmp;
    float t35;
    float t37;
    float t39;
    float t4;
    float t40;
    float t43;
    float t44;
    float t45;
    float t5;
    float t7;
    int i;
    int r3;
    /* COMPJ */
    /*     J_WB_B = COMPJ(IN1) */
    /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
    /*     08-Dec-2021 14:12:05 */
    t2 = (float)cos(x[3]);
    t3 = (float)cos(x[4]);
    t4 = (float)sin(x[3]);
    Y_idx_1 = (float)sin(x[4]);
    J_wb_b[0] = 1.0F;
    J_wb_b[3] = 0.0F;
    J_wb_b[4] = -t2;
    J_wb_b[5] = t4;
    J_wb_b[6] = Y_idx_1;
    J_wb_b[7] = t3 * t4;
    J_wb_b[8] = t2 * t3;
    for (i = 0; i < 12; i++)
    {
        xnew[i] = x[i];
    }
    i = 1;
    r3 = 2;
    J_wb_b[1] = 0.0F;
    J_wb_b[2] = 0.0F;
    J_wb_b[7] -= 0.0F * Y_idx_1;
    J_wb_b[8] -= 0.0F * Y_idx_1;
    if ((float)fabs(t4) > (float)fabs(-t2))
    {
        i = 2;
        r3 = 1;
    }
    J_wb_b[r3 + 3] /= J_wb_b[i + 3];
    J_wb_b[r3 + 6] -= J_wb_b[r3 + 3] * J_wb_b[i + 6];
    Y_idx_1 = x[i + 9] - x[9] * 0.0F;
    t4 = ((x[r3 + 9] - x[9] * 0.0F) - Y_idx_1 * J_wb_b[r3 + 3]) / J_wb_b[r3 + 6];
    Y_idx_1 -= t4 * J_wb_b[i + 6];
    Y_idx_1 /= J_wb_b[i + 3];
    xnew[9] = (x[9] - t4 * J_wb_b[6]) - Y_idx_1 * 0.0F;
    xnew[10] = Y_idx_1;
    xnew[11] = t4;
    for (i = 0; i < 12; i++)
    {
        xnew[i] -= xd[i];
    }
    for (i = 0; i < 4; i++)
    {
        Y_idx_1 = 0.0F;
        for (r3 = 0; r3 < 12; r3++)
        {
            Y_idx_1 += a[i + (r3 << 2)] * xnew[r3];
        }
        u[i] = Y_idx_1;
    }
    /* COMPTAU */
    /*     TAU = COMPTAU(IN1,IN2) */
    /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
    /*     08-Dec-2021 14:12:08 */
    t2 = (float)cos(xnew[3]);
    t3 = (float)cos(xnew[4]);
    t4 = (float)sin(xnew[3]);
    t5 = (float)sin(xnew[4]);
    t7 = xnew[11] * xnew[11];
    t15 = 1.41421354F * u[1] * 0.000107824511F;
    t16 = t3 * 1.41421354F * xnew[10] * xnew[11] * 0.000182030722F;
    t18 = t4 * 1.41421354F * xnew[9] * xnew[10] * 3.36183039E-5F;
    t20 = t2 * 1.41421354F * u[2] * 0.000107824511F;
    t21 = t5 * 1.41421354F * u[3] * 0.000107824511F;
    t23 = t4 * t5 * 1.41421354F * xnew[10] * xnew[11] * 0.000182030722F;
    Y_idx_1 = t2 * t3;
    t26 = Y_idx_1 * 1.41421354F * xnew[9] * xnew[11] * 3.36183039E-5F;
    t37 = t3 * t4;
    t28 = t37 * 1.41421354F * u[3] * 0.000107824511F;
    t31_tmp = t2 * t4;
    t31 = t31_tmp * (xnew[10] * xnew[10]) * 1.41421354F * 7.42062039E-5F;
    t35 = Y_idx_1 * t5 * t7 * 1.41421354F * 7.42062039E-5F;
    t39 = t4 * u[2] * 0.00140386284F;
    t40 = t2 * xnew[9] * xnew[10] * 0.00140386284F;
    t43 = Y_idx_1 * u[3] * 0.00140386284F;
    t44 = t37 * xnew[9] * xnew[11] * 0.00140386284F;
    t45 = t2 * t5 * xnew[10] * xnew[11] * 0.00140386284F;
    Y_idx_1 = 1.0F / t2 * (1.0F / t3);
    t5 = t3 * (t2 * t2) * 1.41421354F * xnew[10] * xnew[11] * 0.000148412408F;
    t37 = t31_tmp * t7 * (t3 * t3) * 1.41421354F * 7.42062039E-5F;
    Y_idx_1 = Y_idx_1 * u[0] * 0.00675F + Y_idx_1 * 0.0662175F;
    t4 = Y_idx_1 + -t15;
    Y_idx_1 = (Y_idx_1 + t15) + t16;
    /*  Real */
    /*  a = 2.130295 * 1e-11; */
    /*  b = 1.032633 * 1e-6; */
    /*  c = 5.484560 * 1e-4 - u; */
    /*  Sim */
    u[0] = 8.60953E-12F -
           3.69664E-9F *
               (0.0023F -
                ((((((((((((((((t4 + -t16) + t20) + -t18) + t23) + -t21) + -t26) +
                          -t28) +
                         t31) +
                        t5) +
                       t35) +
                      -t37) +
                     -t39) +
                    -t40) +
                   t44) +
                  t45) +
                 -t43));
    u[1] = 8.60953E-12F -
           3.69664E-9F *
               (0.0023F -
                ((((((((((((((((t4 + t18) + -t16) + -t20) + -t21) + t26) + -t23) +
                          t28) +
                         t31) +
                        t5) +
                       -t35) +
                      -t37) +
                     t39) +
                    t40) +
                   t43) +
                  -t44) +
                 -t45));
    u[2] =
        8.60953E-12F -
        3.69664E-9F *
            (0.0023F -
             (((((((((((((((Y_idx_1 + t18) + t21) + -t20) + t26) + -t23) + t28) +
                      -t31) +
                     -t5) +
                    -t35) +
                   t37) +
                  -t39) +
                 -t40) +
                t44) +
               t45) +
              -t43));
    u[3] =
        8.60953E-12F -
        3.69664E-9F *
            (0.0023F -
             (((((((((((((((Y_idx_1 + t20) + t21) + -t18) + t23) + -t26) + -t28) +
                      -t31) +
                     -t5) +
                    t35) +
                   t37) +
                  t39) +
                 t40) +
                t43) +
               -t44) +
              -t45));
    u[0] = ((float)sqrt(u[0]) + -2.9342E-6F) / 1.84832E-9F;
    u[1] = ((float)sqrt(u[1]) + -2.9342E-6F) / 1.84832E-9F;
    u[2] = ((float)sqrt(u[2]) + -2.9342E-6F) / 1.84832E-9F;
    u[3] = ((float)sqrt(u[3]) + -2.9342E-6F) / 1.84832E-9F;
}

void PositionController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    if(!state_estimator_active_)
       // When the state estimator is disable, the delta_omega_ value is computed as soon as the new odometry message is available.
       //The timing is managed by the publication of the odometry topic
       HoveringController(&control_t_.thrust);

    // Control signals are sent to the on board control architecture if the state estimator is active
    double delta_phi, delta_theta, delta_psi;
    if(state_estimator_active_){
       crazyflie_onboard_controller_.SetControlSignals(control_t_);
       crazyflie_onboard_controller_.SetDroneState(state_);
       crazyflie_onboard_controller_.RateController(&delta_phi, &delta_theta, &delta_psi);
    }
    else
    {
        RateController(&delta_phi, &delta_theta, &delta_psi);
        Quaternion2Euler(&state_.attitude.roll, &state_.attitude.pitch, &state_.attitude.yaw);
    }

    float pos[3] = {
        state_.position.x,
        state_.position.y,
        state_.position.z};
    float vel[3] = {
        state_.linearVelocity.x,
        state_.linearVelocity.y,
        state_.linearVelocity.z};
    float rpy[3] = {
        state_.attitude.roll,
        -state_.attitude.pitch, // pitch is counterclockwise in crazyS
        state_.attitude.yaw};
    float rpy_fake[3] = {
        0,
        0,
        state_.attitude.yaw};

    rot_wb(vel, rpy, false);
    // rot_wb(pos, rpy_fake, true);
    // rot_wb(vel, rpy_fake, true);

    float x[12] = {
        pos[0],
        pos[1],
        pos[2],
        rpy[0],
        rpy[1],
        rpy[2],
        vel[0],
        vel[1],
        vel[2],
        state_.angularVelocity.x,
        state_.angularVelocity.y,
        state_.angularVelocity.z,
    };

    float xd[12] = {0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float u[4];

    if (controller_parameters_.controller_type_ == 0)
    {
        mylqr(x, xd, u);
    }
    else if (controller_parameters_.controller_type_ == 2) {
        mypid(x, xd, u);
    }
    else if (controller_parameters_.controller_type_ == 3)
    {
         mygs(x, xd, u);
    }
    else
    {
        mylqr_pfl(x, xd, u);
    }

    for (size_t i = 0; i < 4; i++)
    {
        u[i] = min(max(u[i], (float)(MAX_NEG_DELTA_OMEGA + OMEGA_OFFSET)), (float)(MAX_POS_DELTA_OMEGA + OMEGA_OFFSET));
    }

    *PWM_1 = u[0];
    *PWM_2 = u[1];
    *PWM_3 = u[2];
    *PWM_4 = u[3];

    ROS_INFO_STREAM_THROTTLE(0.1, x[0] << "," << x[1] << "," << x[2] << "," << x[3] << "," << x[4] << "," << x[5] << "," << x[6] << "," << x[7] << "," << x[8] << "," << x[9] << "," << x[10] << "," << x[11]);

    if(dataStoring_active_){
      // Saving drone attitude in a file
      std::stringstream tempPWM;
      tempPWM << *PWM_1 << "," << *PWM_2 << "," << *PWM_3 << "," << *PWM_4 << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPWM_.push_back(tempPWM.str());

      // Saving drone attitude in a file
      std::stringstream tempPWMComponents;
      tempPWMComponents << control_t_.thrust << "," << delta_theta << "," << delta_phi << "," << delta_psi << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPWMComponents_.push_back(tempPWMComponents.str());
    }

    ROS_DEBUG("Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f", control_t_.thrust, delta_theta, delta_phi, delta_psi);
    ROS_DEBUG("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
}

void PositionController::XYController(double* theta_command, double* phi_command) {
    assert(theta_command);
    assert(phi_command);

    double v, u;
    u = state_.linearVelocity.x;
    v = state_.linearVelocity.y;

    double xe, ye;
    ErrorBodyFrame(&xe, &ye);

    double e_vx, e_vy;
    e_vx = xe - u;
    e_vy = ye - v;

    double theta_command_kp;
    theta_command_kp = xy_gain_kp_.x() * e_vx;
    theta_command_ki_ = theta_command_ki_ + (xy_gain_ki_.x() * e_vx * SAMPLING_TIME);
    *theta_command  = theta_command_kp + theta_command_ki_;

    double phi_command_kp;
    phi_command_kp = xy_gain_kp_.y() * e_vy;
    phi_command_ki_ = phi_command_ki_ + (xy_gain_ki_.y() * e_vy * SAMPLING_TIME);
    *phi_command  = phi_command_kp + phi_command_ki_;

    // Theta command is saturated considering the aircraft physical constraints
    if(!(*theta_command < MAX_THETA_COMMAND && *theta_command > -MAX_THETA_COMMAND))
       if(*theta_command > MAX_THETA_COMMAND)
          *theta_command = MAX_THETA_COMMAND;
       else
          *theta_command = -MAX_THETA_COMMAND;

    // Phi command is saturated considering the aircraft physical constraints
    if(!(*phi_command < MAX_PHI_COMMAND && *phi_command > -MAX_PHI_COMMAND))
       if(*phi_command > MAX_PHI_COMMAND)
          *phi_command = MAX_PHI_COMMAND;
       else
          *phi_command = -MAX_PHI_COMMAND;

    if(dataStoring_active_){
      // Saving drone attitude in a file
      std::stringstream tempCommandAttiude;
      tempCommandAttiude << *theta_command << "," << *phi_command << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listCommandAttiude_.push_back(tempCommandAttiude.str());

      // Saving drone attitude in a file
      std::stringstream tempXeYe;
      tempXeYe << xe << "," << ye << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listXeYe_.push_back(tempXeYe.str());

    }

     ROS_DEBUG("Theta_kp: %f, Theta_ki: %f", theta_command_kp, theta_command_ki_);
     ROS_DEBUG("Phi_kp: %f, Phi_ki: %f", phi_command_kp, phi_command_ki_);
     ROS_DEBUG("Phi_c: %f, Theta_c: %f", *phi_command, *theta_command);
     ROS_DEBUG("E_vx: %f, E_vy: %f", e_vx, e_vy);
     ROS_DEBUG("E_x: %f, E_y: %f", xe, ye);
}

void PositionController::YawPositionController(double* r_command) {
    assert(r_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    double yaw_error, yaw_reference;
    yaw_reference = command_trajectory_.getYaw();
    yaw_error = yaw_reference - yaw;

    double r_command_kp;
    r_command_kp = yaw_gain_kp_ * yaw_error;
    r_command_ki_ = r_command_ki_ + (yaw_gain_ki_ * yaw_error * SAMPLING_TIME);
    *r_command = r_command_ki_ + r_command_kp;

   // R command value is saturated considering the aircraft physical constraints
   if(!(*r_command < MAX_R_DESIDERED && *r_command > -MAX_R_DESIDERED))
      if(*r_command > MAX_R_DESIDERED)
         *r_command = MAX_R_DESIDERED;
      else
         *r_command = -MAX_R_DESIDERED;

   if(dataStoring_active_){
     // Saving drone attitude in a file
     std::stringstream tempRCommand;
     tempRCommand << *r_command << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

     listRCommand_.push_back(tempRCommand.str());
   }

}

void PositionController::HoveringController(double* omega) {
    assert(omega);

    double z_error, z_reference, dot_zeta;
    z_reference = command_trajectory_.position_W[2];
    z_error = z_reference - state_.position.z;

    // Velocity along z-axis from body to inertial frame
    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // Needed because both angular and linear velocities are expressed in the aircraft body frame
    dot_zeta = -sin(pitch)*state_.linearVelocity.x + sin(roll)*cos(pitch)*state_.linearVelocity.y +
	            cos(roll)*cos(pitch)*state_.linearVelocity.z;

    double delta_omega, delta_omega_kp, delta_omega_kd;
    delta_omega_kp = hovering_gain_kp_ * z_error;
    delta_omega_ki_ = delta_omega_ki_ + (hovering_gain_ki_ * z_error * SAMPLING_TIME);
    delta_omega_kd = hovering_gain_kd_ * -dot_zeta;
    delta_omega = delta_omega_kp + delta_omega_ki_ + delta_omega_kd;

    // Delta omega value is saturated considering the aircraft physical constraints
    if(delta_omega > MAX_POS_DELTA_OMEGA || delta_omega < MAX_NEG_DELTA_OMEGA)
      if(delta_omega > MAX_POS_DELTA_OMEGA)
         delta_omega = MAX_POS_DELTA_OMEGA;
      else
         delta_omega = -MAX_NEG_DELTA_OMEGA;

     *omega = OMEGA_OFFSET + delta_omega;

     if(dataStoring_active_){
       // Saving drone attitude in a file
       std::stringstream tempOmegaCommand;
       tempOmegaCommand << *omega << "," << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

       listOmegaCommand_.push_back(tempOmegaCommand.str());

       // Saving drone attitude in a file
       std::stringstream tempDroneAttitude;
       tempDroneAttitude << roll << "," << pitch << "," << yaw << ","
               << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

       listDroneAttiude_.push_back(tempDroneAttitude.str());

     }

     ROS_DEBUG("Delta_omega_kp: %f, Delta_omega_ki: %f, Delta_omega_kd: %f", delta_omega_kp, delta_omega_ki_, delta_omega_kd);
     ROS_DEBUG("Z_error: %f, Delta_omega: %f", z_error, delta_omega);
     ROS_DEBUG("Dot_zeta: %f", dot_zeta);
     ROS_DEBUG("Omega: %f, delta_omega: %f", *omega, delta_omega);

}

void PositionController::ErrorBodyFrame(double* xe, double* ye) const {
    assert(xe);
    assert(ye);

    // X and Y reference coordinates
    double x_r = command_trajectory_.position_W[0];
    double y_r = command_trajectory_.position_W[1];

    // Position error
    double x_error_, y_error_;
    x_error_ = x_r - state_.position.x;
    y_error_ = y_r - state_.position.y;

    // The aircraft attitude (estimated or not, it depends by the employed controller)
    double yaw, roll, pitch;
    Quaternion2Euler(&roll, &pitch, &yaw);

    // Tracking error in the body frame
    *xe = x_error_ * cos(yaw) + y_error_ * sin(yaw);
    *ye = y_error_ * cos(yaw) - x_error_ * sin(yaw);

}


/* FROM HERE THE FUNCTIONS EMPLOYED WHEN THE STATE ESTIMATOR IS UNABLE ARE REPORTED */

//Such function is invoked by the position controller node when the state estimator is not in the loop
void PositionController::SetOdometryWithoutStateEstimator(const EigenOdometry& odometry) {

    odometry_ = odometry;

    // Such function is invoked when the ideal odometry sensor is employed
    SetSensorData();

    if(dataStoring_active_){

      // Saving drone attitude in a file
      std::stringstream tempDronePosition;
      tempDronePosition << odometry_.position[0] << "," << odometry_.position[1] << "," << odometry_.position[2] << "," 
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listDronePosition_.push_back(tempDronePosition.str());
    }
}

// Odometry values are put in the state structure. The structure contains the aircraft state
void PositionController::SetSensorData() {

    // Only the position sensor is ideal, any virtual sensor or systems is available to get it
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];
}

void PositionController::RateController(double* delta_phi, double* delta_theta, double* delta_psi) {
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);

    double p, q, r;
    p = state_.angularVelocity.x;
    q = state_.angularVelocity.y;
    r = state_.angularVelocity.z;

    double p_command, q_command;
    AttitudeController(&p_command, &q_command);

    double r_command;
    YawPositionController(&r_command);

    double p_error, q_error, r_error;
    p_error = p_command - p;
    q_error = q_command - q;
    r_error = r_command - r;

    double delta_phi_kp, delta_theta_kp, delta_psi_kp;
    delta_phi_kp = rate_gain_kp_.x() * p_error;
    *delta_phi = delta_phi_kp;

    delta_theta_kp = rate_gain_kp_.y() * q_error;
    *delta_theta = delta_theta_kp;

    delta_psi_kp = rate_gain_kp_.z() * r_error;
    delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_.z() * r_error * SAMPLING_TIME);
    *delta_psi = delta_psi_kp + delta_psi_ki_;

    if(dataStoring_active_){
      // Saving drone attitude in a file
      std::stringstream tempDeltaCommands;
      tempDeltaCommands << *delta_phi << "," << *delta_theta << "," << *delta_psi << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listDeltaCommands_.push_back(tempDeltaCommands.str());
    }

}

void PositionController::AttitudeController(double* p_command, double* q_command) {
    assert(p_command);
    assert(q_command);

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);

    double theta_command, phi_command;
    XYController(&theta_command, &phi_command);

    double phi_error, theta_error;
    phi_error = phi_command - roll;
    theta_error = theta_command - pitch;

    double p_command_kp, q_command_kp;
    p_command_kp = attitude_gain_kp_.x() * phi_error;
    p_command_ki_ = p_command_ki_ + (attitude_gain_ki_.x() * phi_error * SAMPLING_TIME);
    *p_command = p_command_kp + p_command_ki_;

    q_command_kp = attitude_gain_kp_.y() * theta_error;
    q_command_ki_ = q_command_ki_ + (attitude_gain_ki_.y() * theta_error * SAMPLING_TIME);
    *q_command = q_command_kp + q_command_ki_;

    if(dataStoring_active_){
      // Saving drone attitude in a filehshou
      std::stringstream tempPQCommands;
      tempPQCommands << *p_command << "," << *q_command << ","
              << odometry_.timeStampSec << "," << odometry_.timeStampNsec << "\n";

      listPQCommands_.push_back(tempPQCommands.str());
    }

    ROS_DEBUG("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", phi_command, phi_error, theta_command, theta_error);

}



/* FROM HERE THE FUNCTIONS EMPLOYED WHEN THE STATE ESTIMATOR IS ABLED ARE REPORTED */

// Such function is invoked by the position controller node when the state estimator is considered in the loop
void PositionController::SetOdometryWithStateEstimator(const EigenOdometry& odometry) {

    odometry_ = odometry;
}


// The aircraft attitude is computed by the complementary filter with a frequency rate of 250Hz
void PositionController::CallbackAttitudeEstimation() {

    // Angular velocities updating
    complementary_filter_crazyflie_.EstimateAttitude(&state_, &sensors_);

    ROS_DEBUG("Attitude Callback");

}

// The high level control runs with a frequency of 100Hz
void PositionController::CallbackHightLevelControl() {

    // Thrust value
    HoveringController(&control_t_.thrust);

    // Phi and theta command signals. The Error Body Controller is invoked every 0.01 seconds. It uses XYController's outputs
    XYController(&control_t_.pitch, &control_t_.roll);

    // Yaw rate command signals
    YawPositionController(&control_t_.yawRate);

    ROS_DEBUG("Position_x: %f, Position_y: %f, Position_z: %f", state_.position.x, state_.position.y, state_.position.z);

    ROS_DEBUG("Angular_velocity_x: %f, Angular_velocity_y: %f, Angular_velocity_z: %f", state_.angularVelocity.x,
             state_.angularVelocity.y, state_.angularVelocity.z);

    ROS_DEBUG("Linear_velocity_x: %f, Linear_velocity_y: %f, Linear_velocity_z: %f", state_.linearVelocity.x,
             state_.linearVelocity.y, state_.linearVelocity.z);

}

// The aircraft angular velocities are update with a frequency of 500Hz
void PositionController::SetSensorData(const sensorData_t& sensors) {

    // The functions runs at 500Hz, the same frequency with which the IMU topic publishes new values (with a frequency of 500Hz)
    sensors_ = sensors;
    complementary_filter_crazyflie_.EstimateRate(&state_, &sensors_);

    if(!state_estimator_active_)
        state_estimator_active_= true;

    // Only the position sensor is ideal, any virtual sensor or systems is available to get these data
    // Every 0.002 seconds the odometry message values are copied in the state_ structure, but they change only 0.01 seconds
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

}


}
