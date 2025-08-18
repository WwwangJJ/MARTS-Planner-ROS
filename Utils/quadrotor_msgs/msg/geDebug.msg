Header header

# sensors-------------------------------------------------

float64 odom_roll
float64 odom_pitch
float64 odom_yaw

float64 odomC_roll
float64 odomC_pitch
float64 odomC_yaw

float64 ge_angle_roll
float64 ge_angle_pitch
float64 ge_angle_yaw

float64 imu_a_x
float64 imu_a_y
float64 imu_a_z

float64 imu_a_fil_x
float64 imu_a_fil_y
float64 imu_a_fil_z

float64 imu_w_x
float64 imu_w_y
float64 imu_w_z

float64 imu_w_fil_x
float64 imu_w_fil_y
float64 imu_w_fil_z

float64 imu_w_c_x
float64 imu_w_c_y
float64 imu_w_c_z

float64 imu_beta_fil_x
float64 imu_beta_fil_y
float64 imu_beta_fil_z

float64 rpm1
float64 rpm2
float64 rpm3
float64 rpm4

float64 rpm1_fil
float64 rpm2_fil
float64 rpm3_fil
float64 rpm4_fil

float64 sumrpm2 #实际转速平方和
float64 thrust_force #实际推力
float64 thrust_acc #实际推力加速度

float64 sumrpm2_fil #实际转速平方和，滤波后
float64 thrust_force_fil #实际推力，滤波后
float64 thrust_acc_fil #实际推力加速度，滤波后

float64 acc_world_x
float64 acc_world_y
float64 acc_world_z

float64 ge_acc_x #按照地效模型，实际的地效额外推力加速度
float64 ge_acc_y
float64 ge_acc_z

float64 ge_bodytorque_x #按照地效模型的反扭矩
float64 ge_bodytorque_y
float64 ge_bodytorque_z

float64 des_ge_bodytorque_x #按照地效模型的反扭矩
float64 des_ge_bodytorque_y
float64 des_ge_bodytorque_z

float64 d_ge_torque_x #按照地效模型的反扭矩的导数
float64 d_ge_torque_y
float64 d_ge_torque_z

float64 diff_ge_torque_x #扭矩变化量
float64 diff_ge_torque_y
float64 diff_ge_torque_z

float64 ext_torque_x
float64 ext_torque_y
float64 ext_torque_z

float64 predict_ext_torque_x
float64 predict_ext_torque_y
float64 predict_ext_torque_z

# 控制器接收的指令，来自traj server-------------------------------------------------
float64 des_p_x
float64 des_p_y
float64 des_p_z

float64 des_v_x
float64 des_v_y
float64 des_v_z

float64 des_a_x
float64 des_a_y
float64 des_a_z

float64 des_j_x
float64 des_j_y
float64 des_j_z

float64 des_s_x
float64 des_s_y
float64 des_s_z

float64 des_yaw
float64 des_yaw_rate
float64 des_yaw_acc


# flatness-------------------------------------------------------------------------------
float64 flatctrl_q_w
float64 flatctrl_q_x
float64 flatctrl_q_y
float64 flatctrl_q_z

float64 flatctrl_roll
float64 flatctrl_pitch
float64 flatctrl_yaw

float64 flatctrl_bodyrate_x
float64 flatctrl_bodyrate_y
float64 flatctrl_bodyrate_z

float64 flatctrl_beta_x
float64 flatctrl_beta_y
float64 flatctrl_beta_z

float64 flatctrl_thrust
float64 flatctrl_torque_x
float64 flatctrl_torque_y
float64 flatctrl_torque_z

# 根据平坦模型：地面效应带来的额外推力加速度
float64 flatctrl_ge_acc_x
float64 flatctrl_ge_acc_y
float64 flatctrl_ge_acc_z

# 根据平坦模型：阻力加速度
float64 flatctrl_drag_acc_x
float64 flatctrl_drag_acc_y
float64 flatctrl_drag_acc_z

# 根据平坦模型：合外力加速度
float64 flatctrl_ext_acc_x
float64 flatctrl_ext_acc_y
float64 flatctrl_ext_acc_z

float64 ge_limit_k

# 加速度环，由误差、外力等输出期望加速度 --------------------------------------------------------------------

float64 fb_vel_x
float64 fb_vel_y
float64 fb_vel_z
float64 cmd_vel_x
float64 cmd_vel_y
float64 cmd_vel_z


float64 compensate_acc_x #补偿给控制器的加速度
float64 compensate_acc_y
float64 compensate_acc_z
float64 fb_acc_x # 由位置和速度误差得到的反馈加速度
float64 fb_acc_y
float64 fb_acc_z
float64 cmd_acc_x # 需要推力提供的加速度
float64 cmd_acc_y
float64 cmd_acc_z
float64 cmd_acc_norm # 需要推力提供的加速度的模
float64 cmd_acc_norm_by_body # 需要推力提供的加速度的模，考虑当前姿态


# 加速度控制 输出 姿态+推力、角速度+推力，扭矩+推力、转速 ------------------------------------------------

# cmd-----------------------------

float64 cmd_q_w
float64 cmd_q_x
float64 cmd_q_y
float64 cmd_q_z

float64 cmd_roll
float64 cmd_pitch
float64 cmd_yaw


float64 fb_bodyrate_x
float64 fb_bodyrate_y
float64 fb_bodyrate_z
float64 cmd_bodyrate_x
float64 cmd_bodyrate_y
float64 cmd_bodyrate_z

float64 fb_beta_x
float64 fb_beta_y
float64 fb_beta_z
float64 cmd_beta_x
float64 cmd_beta_y
float64 cmd_beta_z

#补偿给控制器的扭矩
float64 compensate_torque_x
float64 compensate_torque_y
float64 compensate_torque_z

float64 bodytorque_x
float64 bodytorque_y
float64 bodytorque_z
float64 cmd_torque_body_x
float64 cmd_torque_body_y
float64 cmd_torque_body_z

float64 diff_indi_torque_x #INDI方法的力矩增量
float64 diff_indi_torque_y
float64 diff_indi_torque_z

float64 diff_torque_x #INDI和模型力矩增量
float64 diff_torque_y
float64 diff_torque_z

float64 cmd_rpm1
float64 cmd_rpm2
float64 cmd_rpm3
float64 cmd_rpm4
float64 cmd_sumrpm2
float64 cmd_sumrpm2_to_acc


# 转速误差引入的推力
float64 fb_thrust1
float64 fb_thrust2
float64 fb_thrust3
float64 fb_thrust4

# 油门曲线前馈推力
float64 ref_thrust1
float64 ref_thrust2
float64 ref_thrust3
float64 ref_thrust4

# 实际推力
float64 cmd_thrust1
float64 cmd_thrust2
float64 cmd_thrust3
float64 cmd_thrust4

# OLD MSGS ----------------------------------------------------------------------------------------------------------
# ref forces
float64 Fgh



# 根据平坦模型：推力加速度
float64 ref_thrust_acc



#实际作用在world系上的推力加速度，滤波后
float64 thrust_acc_fil_x 
float64 thrust_acc_fil_y
float64 thrust_acc_fil_z

# force estimate

# 滤波后的IMU加速度数据
float64 imu_acc_fil_x
float64 imu_acc_fil_y
float64 imu_acc_fil_z

#外力估计器估计的实际加速度
float64 extforce_acc_x
float64 extforce_acc_y
float64 extforce_acc_z

#外力估计器估计的实际加速度, 滤波后
float64 extforce_acc_fil_x
float64 extforce_acc_fil_y
float64 extforce_acc_fil_z








float64 poly_thrust # 根据 油门~转速 模型得到的0~1的前馈油门
float64 fb_thrust # 根据转速差PID反馈得到的补偿油门差
float64 sum_thrust # 前馈油门+反馈油门







