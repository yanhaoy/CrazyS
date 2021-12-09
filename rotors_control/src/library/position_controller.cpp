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
      // Saving drone attitude in a file
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
