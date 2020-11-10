 /**
  * @file vrep_seing_mechanism.h
  * @brief DANIEL(以下ANI)の陽動機構をvrep上で動かすためのクラス
  * @author Yusuke Yoshizaki
  * @date 2019/09/05
  * @detail
  *  vrep_ani_master_nodeから送られてきた
  *  ダイナミクセルの角度データを受け取ってjointstateをPubulishする.
  */

#ifndef VREP_SWING_MECHANISM_SRC_VREP_SWING_MECHANISM_H_
#define VREP_SWING_MECHANISM_SRC_VREP_SWING_MECHANISM_H_

#include "ros/ros.h"

// Dynaixel
#include <std_msgs/Float32.h>

// JointState
#include <sensor_msgs/JointState.h>

class VrepSwingMechanism {
 public:
   VrepSwingMechanism();
   ~VrepSwingMechanism();

 private:
   // V-REP関係
   bool GetHandle();

   // CallBack関数
   void timerCallback(const ros::TimerEvent& event);
   void DxCallBack(std_msgs::Float32 dx_angle);

   // JointStateを登録するための関数
   void setVariablesAndPublishJointStates(double grad, double rot, double dist, ros::Time stamp);

   //揺動機構関係の変数
   double gradient_; // LRF傾斜角度[rad]
   double distance_; // LRFロッド取り付け軸のずれ[m]

   double l2_;

   double theta_m_; // dによるモータ角度の補正値θ'[rad]
   double theta_r_; // ロール角度[rad]
   double theta_p_; // ピッチ角度[rad]

   double w_m_; // モータの角速度[rad/s]
   double theta_m_0_; // モータの角度初期値[rad]

   // JointState
   sensor_msgs::JointState joint_state_;

   //Timer
   ros::Timer timer_;

   //制御周期
   double sampling_time_;

   //NodeHandle
   ros::NodeHandle nh_;

   //Publisher
   ros::Publisher joint_pub_;

   //Subscriber
   ros::Subscriber dx_sub_;

};

//条件コンパイル
#endif /* VREP_SWING_MECHANISM_SRC_VREP_SWING_MECHANISM_H_ */
