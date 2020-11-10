
 /**
  * @file vrep_swing_mechanism.cpp
  * @brief vrep内の兄ロボットの揺動機構を動かすためのクラスに変更
  * @author Yusuke Yoshizaki
  * @date 2019/09/05
  * @detail
  */

//本ソースファイルのヘッダをインクルード
#include "vrep_swing_mechanism.h"

//===== constructor & destructor ==================================================//
/** @fn
 * @brief コンストラクタ
 * @param なし
 */
VrepSwingMechanism::VrepSwingMechanism(){

  // 陽動機構関係の変数の設定
  gradient_ = 0.0*M_PI/180.0; // LRF傾斜角度[rad]
  distance_ = 0.9 * 0.001;     // LRFロッド取り付け軸のずれ[m]

  l2_ = 92.0 * 0.001;          // どっかの長さ[m]

  theta_m_ = 0.0;              // dによるモータ角度の補正値θ'[rad]
  theta_r_ = 0.0;              // ロール角度[rad]
  theta_p_ = 0.0;              // ピッチ角度[rad]

  w_m_ = 0.0;                  // モータの角速度[rad/s]
  theta_m_0_ = 0.0;            // モータの角度初期値[rad]

  // Publisher
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Subscriber
  dx_sub_ = nh_.subscribe<std_msgs::Float32>("/dx_angle", 100, &VrepSwingMechanism::DxCallBack, this);

}

/** @fn
 * @brief デストラクタ
 * @param なし
 */
VrepSwingMechanism::~VrepSwingMechanism()
{
}

//===== Callback ==================================================//
/** @fn
 * @brief Dynamixelの角度を取得する
 * @param const ros::TimerEvent& event
 * @return なし
 * @detail
 * ダイナミクセルの角度を受け取って、
 * setVariablesAndPublishJointStatesを実行する
 */
void VrepSwingMechanism::DxCallBack(std_msgs::Float32 dx_angle) {

  //回転速度と位置の更新
  w_m_= (dx_angle.data - theta_m_0_) / sampling_time_;
  theta_m_0_ = dx_angle.data;

  setVariablesAndPublishJointStates(gradient_, theta_m_0_, distance_, ros::Time::now());

}

/** @fn
 * @brief joint_staeをPublishする
 * @param　double grad, double rot, double dist, ros::Time stamp
 * @return なし
 * @detail
 *
 */
void VrepSwingMechanism::setVariablesAndPublishJointStates(double grad, double rot, double dist, ros::Time stamp) {

  // 変数定義
  double theta_m = rot; // モータ角度[rad]
  double alpha = grad; // 穴の位置[rad]
  double l1 = l2_*tan(alpha); // [m]

  double d = distance_; // LRFロッド取り付け軸のずれ[m]
  double l3 = sqrt(l1*l1 + l2_*l2_);
  double d_ = d*l3/l2_; // d'[m]
  double d_theta_m = atan2(d_, l1); // dによるモータ角度の補正量Δθ[rad]
  double l1_c = sqrt(l1*l1 + d_*d_); // dによるl1補正値l1'[m]
  double l2_c = sqrt(l2_*l2_ + d*d); // dによるl2補正値l2'[m]

  theta_m_ = theta_m - d_theta_m; // dによるモータ角度の補正値θ'[rad]
  double xm = l1_c*cos(theta_m_); // [m]
  double ym = l1_c*sin(theta_m_); // [m]
  theta_r_ = atan2(ym, l2_c); // ロール角度[rad]
  theta_p_ = atan2(-xm, sqrt(ym*ym + l2_c*l2_c)); // ピッチ角度[rad] 座標系を合わせるためxmは逆にする

  //ROS_INFO("theta_m_ = %lf[deg](d: %lf), theta_r_ = %lf[deg], theta_p = %lf[deg]", theta_m_*180.0/M_PI, d_theta_m*180.0/M_PI, theta_r_*180.0/M_PI, theta_p_*180.0/M_PI);

  int jnum = 3; //5
  joint_state_.header.stamp = stamp;
  joint_state_.name.resize(jnum);
  joint_state_.position.resize(jnum);
  int jno = 0; // joint number

  // SM base angle (fixed)
  //joint_state_.name[jno] ="base_to_sm_base";
  //joint_state_.position[jno] = 0;
  //jno++;

  // SM motor angle
  joint_state_.name[jno] ="sm_base_to_sm_motor";
  joint_state_.position[jno] = theta_m_;
  jno++;

  // SM roll angle
  joint_state_.name[jno] ="sm_base_to_sm_roll";
  joint_state_.position[jno] = theta_r_;
  jno++;

  // SM pitch angle
  joint_state_.name[jno] ="sm_roll_to_sm_pitch";
  joint_state_.position[jno] = theta_p_;
  jno++;

  // SM laser angle (fixed)
  //joint_state_.name[jno] ="sm_pitch_to_sm_laser";
  //joint_state_.position[jno] = 0;
  //jno++;

  // send the joint state
  joint_pub_.publish(joint_state_);
}
