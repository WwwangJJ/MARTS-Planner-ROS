#include "px4ctrl/input.h"

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    int16_t temp_ch2 = APM_Tools::limit_pwm(msg.channels[2]);
    thrust_ch = APM_Tools::pwm_to_actuator(temp_ch2);

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if (!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if (!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // 1
    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else
        enter_hover_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else
        is_hover_mode = false;

    // 2
    if (is_hover_mode)
    {
        if (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if (gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;

        if (gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else
            is_command_mode = false;
    }

    // 3
    if (!is_hover_mode && !is_command_mode)
    {
        if (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else
            toggle_reboot = false;
    }
    else
        toggle_reboot = false;

    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
    if (mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

bool RC_Data_t::check_centered()
{
    // bool centered = abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5 && abs(ch[0]) < 1e-5;
    bool centered = fabs(ch[0]) < 1e-5 && fabs(ch[1]) < 1e-5 && fabs(ch[2]) < 1e-5 && fabs(ch[3]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    s(0) = msg.snap.x;
    s(1) = msg.snap.y;
    s(2) = msg.snap.z;

    // std::cout << "j1=" << j.transpose() << std::endl;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
    yaw_acc = msg.yaw_acc;
    // std::cout << "222222" << std::endl;
}

Command_transport_Data_t::Command_transport_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_transport_Data_t::feed(quadrotor_msgs::PositionCommandNewConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    pl(0) = msg.position.x;
    pl(1) = msg.position.y;
    pl(2) = msg.position.z;

    vl(0) = msg.velocity.x;
    vl(1) = msg.velocity.y;
    vl(2) = msg.velocity.z;

    al(0) = msg.acceleration.x;
    al(1) = msg.acceleration.y;
    al(2) = msg.acceleration.z;

    jl(0) = msg.jerk.x;
    jl(1) = msg.jerk.y;
    jl(2) = msg.jerk.z;

    sl(0) = msg.snap.x;
    sl(1) = msg.snap.y;
    sl(2) = msg.snap.z;

    angT(0) = msg.angT.x;
    angT(1) = msg.angT.y;
    angT(2) = msg.angT.z;

    dAngT(0) = msg.dAngT.x;
    dAngT(1) = msg.dAngT.y;
    dAngT(2) = msg.dAngT.z;

    d2AngT(0) = msg.d2AngT.x;
    d2AngT(1) = msg.d2AngT.y;
    d2AngT(2) = msg.d2AngT.z;

    d3AngT(0) = msg.d3AngT.x;
    d3AngT(1) = msg.d3AngT.y;
    d3AngT(2) = msg.d3AngT.z;

    d4AngT(0) = msg.d4AngT.x;
    d4AngT(1) = msg.d4AngT.y;
    d4AngT(2) = msg.d4AngT.z;
    flag = msg.flag;

    yaw = msg.yaw;
    yaw_rate = msg.yaw_dot;
    yaw_acc = msg.yaw_acc;
    // std::cout << "222222" << std::endl;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0);
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            ROS_ERROR("[px4ctrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}


Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;
    af(0) = a(0);
    af(1) = a(1);
    af(2) = a(2);

    // for(int i=0;i<3;i++)
    // {
    //     if(abs(a(i)) > 100)
    //         ROS_ERROR("AAAAAAAAA !!!!!!!!!!!!!!!!!!!!!!!! : %f",a(i));
    // }

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    // check the frequency
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if ( (now - last_clear_count_time).toSec() > 1.0 )
    {
        if ( one_min_count < 100 )
        {
            ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count ++;

    static filters::biquad_cascade_df1 hfil_acc[3];
    static filters::biquad_cascade_df1 hfil_w[3];
    static DerivFilter7 hfil_dw[3];
    
    static bool first_rcv = false;
    if(first_rcv == false)
    {
        for(int i = 0; i <= 1; i++)
        {
            // hfil_acc[i].param.param_200_12_order2();
            // hfil_w[i].param.param_200_12_order2();

            // hfil_acc[i].param.param_333_24_order2();
            hfil_w[i].param.param_333_20_order2();
        }  
        // hfil_acc[2].param.param_333_20_order2();
        hfil_w[2].param.param_333_9_order2();
    }
    first_rcv = true;


    for(int i = 0; i <= 2; i++)
    {
        // hfil_acc[i].step_with_scale(&a[i], &af[i], 1);
        hfil_w[i].step_with_scale(&w[i], &wf[i], 1);
        hfil_dw[i].update(w[i], msg.header.stamp.toSec());
        dw[i] = hfil_dw[i].slope();
    }
    // wf[0] = w[0];
    // wf[1] = w[1];
    // wf[2] = w[2];   
    // for(int i=0;i<3;i++)
    // {
    //     if(abs(dw[i]) > 100)
    //         ROS_ERROR("FFFFFFF !!!!!!!!!!!!!!!!!!!!!!!! : %f",dw[i]);
    // }

}

RPM_Data_t::RPM_Data_t()
{
    rcv_stamp = ros::Time(0);
    rpm[0] = 0;
    rpm[1] = 0;
    rpm[2] = 0;
    rpm[3] = 0;
    sum_rpm2 = 0;
    sum_rpm2_fil = 0;
    x_sum_rpm2 = 0;
    y_sum_rpm2 = 0;
    z_sum_rpm2 = 0;
    x_sum_rpm2_fil = 0;
    y_sum_rpm2_fil = 0;
    z_sum_rpm2_fil = 0;
}


void RPM_Data_t::feed(mavros_msgs::ESCTelemetryConstPtr pMsg)
{

    rcv_stamp = pMsg->header.stamp;
    Eigen::Vector4d esc_rpm;
    msg = *pMsg;
    esc_rpm(0) = msg.esc_telemetry[0].rpm;
    esc_rpm(1) = msg.esc_telemetry[1].rpm;
    esc_rpm(2) = msg.esc_telemetry[2].rpm;
    esc_rpm(3) = msg.esc_telemetry[3].rpm;
    // ROS_INFO("rpm1 : %.2f rpm2 : %.2f rpm3 : %.2f rpm4 : %.2f",msg.esc_telemetry[0].rpm,msg.esc_telemetry[1].rpm,msg.esc_telemetry[2].rpm,msg.esc_telemetry[3].rpm);
    msg.esc_telemetry[0].rpm = esc_rpm(1);
    msg.esc_telemetry[1].rpm = esc_rpm(2);
    msg.esc_telemetry[2].rpm = esc_rpm(3);
    msg.esc_telemetry[3].rpm = esc_rpm(0);


    for(int i = 0; i <= 3; i++)
    {
        double error = msg.esc_telemetry[i].rpm - rpm[i];
        error = fabs(error);

        if
        (
            !(msg.esc_telemetry[i].rpm == 0 && error > 2000)
            && !(msg.esc_telemetry[i].rpm > 1.5e4 && error > 5000)
        )
        {
            rpm[i] = msg.esc_telemetry[i].rpm;
            // ROS_INFO("rpm %d : %.2f",i,rpm[i]);
        }

    }
    
    double sum = 0;
    for(int i = 0; i <= 3; i++)
    {
        sum += pow(rpm[i], 2);
    }
    sum_rpm2 = sum;
    x_sum_rpm2 = pow(rpm[1], 2) + pow(rpm[2], 2) - pow(rpm[0], 2) - pow(rpm[3], 2);
    y_sum_rpm2 = pow(rpm[1], 2) + pow(rpm[3], 2) - pow(rpm[0], 2) - pow(rpm[2], 2);
    z_sum_rpm2 = pow(rpm[2], 2) + pow(rpm[3], 2) - pow(rpm[0], 2) - pow(rpm[1], 2);
    
    static int stage = 2;
    static filters::biquad_cascade_df1 hfil_rpm[4];
    static filters::biquad_cascade_df1 hfil_sumrpm;
    static filters::biquad_cascade_df1 hfil_x_sumrpm;
    static filters::biquad_cascade_df1 hfil_y_sumrpm;
    static filters::biquad_cascade_df1 hfil_z_sumrpm;

    static bool first_rcv = false;
    if(first_rcv == false)
    {
        // hfil_rpm[0].param.param_200_12_order2();
        // hfil_rpm[1].param.param_200_12_order2();
        // hfil_rpm[2].param.param_200_12_order2();
        // hfil_rpm[3].param.param_200_12_order2();
        // hfil_sumrpm.param.param_200_12_order2();

        hfil_rpm[0].param.param_333_20_order2();
        hfil_rpm[1].param.param_333_20_order2();
        hfil_rpm[2].param.param_333_20_order2();
        hfil_rpm[3].param.param_333_20_order2();

        hfil_sumrpm.param.param_333_20_order2();
        hfil_x_sumrpm.param.param_333_20_order2();
        hfil_y_sumrpm.param.param_333_20_order2();
        hfil_z_sumrpm.param.param_333_20_order2();
    }
    first_rcv = true;


    for(int i = 0; i <= 3; i++)
    {
        hfil_rpm[i].step_with_scale(&rpm[i], &rpm_fil[i], 1);
        // rpm_fil[i] = rpm[i];
    }
    hfil_sumrpm.step_with_scale(&sum_rpm2, &sum_rpm2_fil, 1);
    hfil_x_sumrpm.step_with_scale(&x_sum_rpm2, &x_sum_rpm2_fil, 1);
    hfil_y_sumrpm.step_with_scale(&y_sum_rpm2, &y_sum_rpm2_fil, 1);
    hfil_z_sumrpm.step_with_scale(&z_sum_rpm2, &z_sum_rpm2_fil, 1);

    // sum_rpm2_fil = sum_rpm2;
    // x_sum_rpm2_fil = x_sum_rpm2;
    // y_sum_rpm2_fil = y_sum_rpm2;
    // z_sum_rpm2_fil = z_sum_rpm2;


}

ExtForce_Data_t::ExtForce_Data_t()
{
  ext_force = Eigen::Vector3d::Zero();
  ext_torque = Eigen::Vector3d::Zero();

  rcv_stamp = ros::Time(0);
}


void ExtForce_Data_t::feed(geometry_msgs::TwistStampedConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    ext_force.x() = msg.twist.linear.x;
    ext_force.y() = msg.twist.linear.y;
    ext_force.z() = msg.twist.linear.z;

    // ext_force.x() = std::min(std::max(ext_force.x(), -1.0), 1.0);
    // ext_force.y() = std::min(std::max(ext_force.y(), -1.0), 1.0);
    // ext_force.z() = std::min(std::max(ext_force.z(), -0.5), 2.0);
    
    ext_torque.x() = msg.twist.angular.x;
    ext_torque.y() = msg.twist.angular.y;
    ext_torque.z() = msg.twist.angular.z;
    // printf("ext force = (%.2f, %.2f, %.2f)\r\n", ext_force(0), ext_force(1), ext_force(2));
}
