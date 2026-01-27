#include "px4ctrl/controller.h"

//////////////

using namespace std;

Controller::Controller(Parameter_t &param_) : param(param_)
{
  int_e_v.setZero();
  KF = param.gain.KF;
  Kp(0) = param.gain.Kp0;
  Kp(1) = param.gain.Kp1;
  Kp(2) = param.gain.Kp2;
  Kv(0) = param.gain.Kv0;
  Kv(1) = param.gain.Kv1;
  Kv(2) = param.gain.Kv2;
  Ka(0) = param.gain.Ka0;
  Ka(1) = param.gain.Ka1;
  Ka(2) = param.gain.Ka2;
  Kq(0) = param.gain.Kq0;
  Kq(1) = param.gain.Kq1;
  Kq(2) = param.gain.Kq2;
  Kvi(0) = param.gain.Kvi0;
  Kvi(1) = param.gain.Kvi1;
  Kvi(2) = param.gain.Kvi2;
  Kvd(0) = param.gain.Kvd0;
  Kvd(1) = param.gain.Kvd1;
  Kvd(2) = param.gain.Kvd2;
  KAng(0) = param.gain.KAngR;
  KAng(1) = param.gain.KAngP;
  KAng(2) = param.gain.KAngY;

  resetThrustMapping();
  Gravity = Eigen::Vector3d(0.0, 0.0, -param.gra);
}

/************* Algorithm0 from the Zhepei Wang, start ***************/

quadrotor_msgs::Px4ctrlDebug Controller::update_alg0(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param);
  Eigen::Vector3d translational_acc = pid_error_accelerations + des.a;
  Eigen::Quaterniond desired_attitude, idel_att;
  Eigen::Vector3d omega;
  double thrust, debug_thrust;
  translational_acc = (Gravity + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity, 1.0)).eval();

  // wmywmy
  minimumSingularityFlatWithDrag(param.mass, param.gra,
                                 des.v, des.a, des.j, des.yaw, des.yaw_rate,
                                 odom.q, idel_att, omega, debug_thrust);
  // wmywmy

  minimumSingularityFlatWithDrag(param.mass, param.gra,
                                 des.v, translational_acc, des.j, des.yaw, des.yaw_rate,
                                 odom.q, desired_attitude, u.bodyrates, thrust);

  Eigen::Vector3d thrustforce = desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d total_des_acc = computeLimitedTotalAccFromThrustForce(thrustforce, param.mass);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  // Compute the error quaternion wmywmy
  const Eigen::Quaterniond q_e = idel_att.inverse() * desired_attitude;

  Eigen::AngleAxisd rotation_vector(q_e); // debug
  Eigen::Vector3d axis = rotation_vector.axis();
  debug.fb_axisang_x = axis(0);
  debug.fb_axisang_y = axis(1);
  debug.fb_axisang_z = axis(2);
  debug.fb_axisang_ang = rotation_vector.angle();
  // wmywmy

  debug.fb_a_x = pid_error_accelerations(0); // debug
  debug.fb_a_y = pid_error_accelerations(1);
  debug.fb_a_z = pid_error_accelerations(2);
  debug.ref_v_x = des.v(0);
  debug.ref_v_y = des.v(1);
  debug.ref_v_z = des.v(2);
  debug.des_p_x = des.p(0);
  debug.des_p_y = des.p(1);
  debug.des_p_z = des.p(2);
  debug.des_q_w = desired_attitude.w(); // debug
  debug.des_q_x = desired_attitude.x();
  debug.des_q_y = desired_attitude.y();
  debug.des_q_z = desired_attitude.z();
  Eigen::Vector3d rpy;
  rpy = quat_to_rpy(idel_att);
  debug.ref_r = rpy.x();
  debug.ref_p = rpy.y();
  debug.ref_y = rpy.z();
  rpy = quat_to_rpy(desired_attitude);
  debug.des_r = rpy.x();
  debug.des_p = rpy.y();
  debug.des_y = rpy.z();
  rpy = quat_to_rpy(odom.q);
  debug.est_r = rpy.x();
  debug.est_p = rpy.y();
  debug.est_y = rpy.z();

  u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

  // limit the angular acceleration
  u.bodyrates = computeLimitedAngularAcc(bodyrate_candidate);
  // u.bodyrates += feedback_bodyrates;

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();

  return debug; // debug
};

Eigen::Vector3d Controller::computeLimitedTotalAccFromThrustForce(
    const Eigen::Vector3d &thrustforce,
    const double &mass) const
{
  Eigen::Vector3d total_acc = thrustforce / mass;

  // Limit magnitude
  if (total_acc.norm() < kMinNormalizedCollectiveAcc_)
  {
    total_acc = total_acc.normalized() * kMinNormalizedCollectiveAcc_;
  }

  // Limit angle
  if (param.max_angle > 0)
  {
    double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d z_B = total_acc.normalized();
    if (z_acc < kMinNormalizedCollectiveAcc_)
    {
      z_acc = kMinNormalizedCollectiveAcc_; // Not allow too small z-force when angle limit is enabled.
    }
    Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
    double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
    if (rot_ang > param.max_angle) // Exceed the angle limit
    {
      Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(param.max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
      total_acc = z_acc / std::cos(param.max_angle) * limited_z_B;
    }
  }

  return total_acc;
}

bool Controller::flatnessWithDrag(const Eigen::Vector3d &vel,
                                  const Eigen::Vector3d &acc,
                                  const Eigen::Vector3d &jer,
                                  const double &psi,
                                  const double &dpsi,
                                  double &thr,
                                  Eigen::Vector4d &quat,
                                  Eigen::Vector3d &omg,
                                  const double &mass,
                                  const double &grav,
                                  const double &dh,
                                  const double &dv,
                                  const double &cp,
                                  const double &veps) const
{
  const double almost_zero = 1.0e-6;

  double w0, w1, w2, dw0, dw1, dw2;
  double v0, v1, v2, a0, a1, a2, v_dot_a;
  double z0, z1, z2, dz0, dz1, dz2;
  double cp_term, w_term, dh_over_m;
  double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
  double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
  double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
  double dw_term, dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
  double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
  double c_psi, s_psi, omg_den, omg_term;

  v0 = vel(0);
  v1 = vel(1);
  v2 = vel(2);
  a0 = acc(0);
  a1 = acc(1);
  a2 = acc(2);
  cp_term = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
  w_term = 1.0 + cp * cp_term;
  w0 = w_term * v0;
  w1 = w_term * v1;
  w2 = w_term * v2;
  dh_over_m = dh / mass;
  zu0 = a0 + dh_over_m * w0;
  zu1 = a1 + dh_over_m * w1;
  zu2 = a2 + dh_over_m * w2 + grav;
  zu_sqr0 = zu0 * zu0;
  zu_sqr1 = zu1 * zu1;
  zu_sqr2 = zu2 * zu2;
  zu01 = zu0 * zu1;
  zu12 = zu1 * zu2;
  zu02 = zu0 * zu2;
  zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
  zu_norm = sqrt(zu_sqr_norm);
  if (zu_norm < almost_zero)
  {
    return false;
  }
  z0 = zu0 / zu_norm;
  z1 = zu1 / zu_norm;
  z2 = zu2 / zu_norm;
  ng_den = zu_sqr_norm * zu_norm;
  ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
  ng01 = -zu01 / ng_den;
  ng02 = -zu02 / ng_den;
  ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
  ng12 = -zu12 / ng_den;
  ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
  v_dot_a = v0 * a0 + v1 * a1 + v2 * a2;
  dw_term = cp * v_dot_a / cp_term;
  dw0 = w_term * a0 + dw_term * v0;
  dw1 = w_term * a1 + dw_term * v1;
  dw2 = w_term * a2 + dw_term * v2;
  dz_term0 = jer(0) + dh_over_m * dw0;
  dz_term1 = jer(1) + dh_over_m * dw1;
  dz_term2 = jer(2) + dh_over_m * dw2;
  dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
  dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
  dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
  f_term0 = mass * a0 + dv * w0;
  f_term1 = mass * a1 + dv * w1;
  f_term2 = mass * (a2 + grav) + dv * w2;
  thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
  if (1.0 + z2 < almost_zero)
  {
    return false;
  }
  tilt_den = sqrt(2.0 * (1.0 + z2));
  tilt0 = 0.5 * tilt_den;
  tilt1 = -z1 / tilt_den;
  tilt2 = z0 / tilt_den;
  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  quat(0) = tilt0 * c_half_psi;
  quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat(3) = tilt0 * s_half_psi;

  return true;
}

bool Controller::flatnessTransport(const Eigen::Vector3d &acc,
                                   const Eigen::Vector3d &jer,
                                   const Eigen::Vector3d &snp,
                                   const Eigen::Vector3d &indi,
                                   const Eigen::Vector3d &q,
                                   const Eigen::Vector3d &dq,
                                   const Eigen::Vector3d &d2q,
                                   const double &F,
                                   const double &dF,
                                   const double &d2F,
                                   const double &psi,
                                   const double &dpsi,
                                   const double &ddpsi,
                                   double &thr,
                                   Eigen::Quaterniond &quat,
                                   Eigen::Vector3d &omg,
                                   Eigen::Vector3d &domg,
                                   const double &massQ,
                                   const double &grav) const
{
  const double almost_zero = 1.0e-6;

  double dFq0, dFq1, dFq2, dzu_norm, zu_sqr_norm, zu_norm;
  double d2Fq0, d2Fq1, d2Fq2;
  double f_term0, f_term1, f_term2;
  double c_half_psi, s_half_psi, c_psi, s_psi;
  double dng00, dng01, dng02, dng11, dng12, dng22, ng_den, ng_den4;
  double dz_term0, dz_term1, dz_term2, d2z_term0, d2z_term1, d2z_term2;
  double z0, z1, z2, dz0, dz1, dz2, d2z0, d2z1, d2z2, zu0, zu1, zu2, zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
  double ng00, ng01, ng02, ng11, ng12, ng22;
  double omg_den, omg_term;
  double tilt0, tilt1, tilt2, tilt_den;
  double massFrac = 1.0 / massQ;

  zu0 = acc(0) - indi(0);
  zu1 = acc(1) - indi(1);
  zu2 = acc(2) - indi(2) + grav;

  zu_sqr0 = zu0 * zu0;
  zu_sqr1 = zu1 * zu1;
  zu_sqr2 = zu2 * zu2;
  zu01 = zu0 * zu1;
  zu12 = zu1 * zu2;
  zu02 = zu0 * zu2;
  zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
  zu_norm = sqrt(zu_sqr_norm);

  if (zu_norm < almost_zero)
  {
    return false;
  }

  z0 = zu0 / zu_norm;
  z1 = zu1 / zu_norm;
  z2 = zu2 / zu_norm;
  ng_den = zu_sqr_norm * zu_norm;
  ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
  ng01 = -zu01 / ng_den;
  ng02 = -zu02 / ng_den;
  ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
  ng12 = -zu12 / ng_den;
  ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
  dFq0 = massFrac * (F * dq(0) + dF * q(0));
  dFq1 = massFrac * (F * dq(1) + dF * q(1));
  dFq2 = massFrac * (F * dq(2) + dF * q(2));
  dz_term0 = jer(0) + dFq0;
  dz_term1 = jer(1) + dFq1;
  dz_term2 = jer(2) + dFq2;
  dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
  dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
  dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;

  f_term0 = massQ * (acc.x() - indi.x());
  f_term1 = massQ * (acc.y() - indi.y());
  f_term2 = massQ * ((acc.z() + grav) - indi.z());
  thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;

  // for domg
  ng_den4 = zu_sqr_norm * zu_sqr_norm;
  dzu_norm = (zu0 * dz_term0 + zu1 * dz_term1 + zu2 * dz_term2) / zu_norm;
  dng00 = (2 * zu_norm * (zu1 * dz_term1 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr1 + zu_sqr2)) / ng_den4;
  dng01 = -((dz_term0 * zu1 + dz_term1 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu1) / ng_den4;
  dng02 = -((dz_term0 * zu2 + dz_term2 * zu0) * zu_norm - 3 * dzu_norm * zu0 * zu2) / ng_den4;
  dng11 = (2 * zu_norm * (zu0 * dz_term0 + zu2 * dz_term2) - 3 * dzu_norm * (zu_sqr0 + zu_sqr2)) / ng_den4;
  dng12 = -((dz_term1 * zu2 + dz_term2 * zu1) * zu_norm - 3 * dzu_norm * zu1 * zu2) / ng_den4;
  dng22 = (2 * zu_norm * (zu0 * dz_term0 + zu1 * dz_term1) - 3 * dzu_norm * (zu_sqr0 + zu_sqr1)) / ng_den4;

  d2Fq0 = massFrac * (F * d2q(0) + 2 * dF * dq(0) + d2F * q(0));
  d2Fq1 = massFrac * (F * d2q(1) + 2 * dF * dq(1) + d2F * q(1));
  d2Fq2 = massFrac * (F * d2q(2) + 2 * dF * dq(2) + d2F * q(2));

  d2z_term0 = snp(0) + d2Fq0;
  d2z_term1 = snp(1) + d2Fq1;
  d2z_term2 = snp(2) + d2Fq2;
  d2z0 = dng00 * dz_term0 + dng01 * dz_term1 + dng02 * dz_term2 + ng00 * d2z_term0 + ng01 * d2z_term1 + ng02 * d2z_term2;
  d2z1 = dng01 * dz_term0 + dng11 * dz_term1 + dng12 * dz_term2 + ng01 * d2z_term0 + ng11 * d2z_term1 + ng12 * d2z_term2;
  d2z2 = dng02 * dz_term0 + dng12 * dz_term1 + dng22 * dz_term2 + ng02 * d2z_term0 + ng12 * d2z_term1 + ng22 * d2z_term2;

  if (1.0 + z2 < almost_zero)
  {
    return false;
  }

  tilt_den = sqrt(2.0 * (1.0 + z2));
  tilt0 = 0.5 * tilt_den;
  tilt1 = -z1 / tilt_den;
  tilt2 = z0 / tilt_den;
  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  quat.w() = tilt0 * c_half_psi;
  quat.x() = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat.y() = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat.z() = tilt0 * s_half_psi;
  c_psi = cos(psi);
  s_psi = sin(psi);
  omg_den = z2 + 1.0;
  omg_term = dz2 / omg_den;
  omg(0) = dz0 * s_psi - dz1 * c_psi -
           (z0 * s_psi - z1 * c_psi) * omg_term;
  omg(1) = dz0 * c_psi + dz1 * s_psi -
           (z0 * c_psi + z1 * s_psi) * omg_term;
  omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;
  domg(0) = s_psi * d2z0 - c_psi * d2z1 - (s_psi * z0 - c_psi * z1) * d2z2 / omg_den + c_psi * dz0 * dpsi + s_psi * dz1 * dpsi - dz2 * (c_psi * z0 * dpsi + s_psi * z1 * dpsi + s_psi * dz0 - c_psi * dz1) / omg_den + dz2 * dz2 * (s_psi * z0 - c_psi * z1) / pow(omg_den, 2);
  domg(1) = c_psi * d2z0 + s_psi * d2z1 - (c_psi * z0 + s_psi * z1) * d2z2 / omg_den + c_psi * dz1 * dpsi - s_psi * dz0 * dpsi - dz2 * (-s_psi * z0 * dpsi + c_psi * z1 * dpsi + c_psi * dz0 + s_psi * dz1) / omg_den + dz2 * dz2 * (c_psi * z0 + s_psi * z1) / pow(omg_den, 2);
  domg(2) = (z1 * d2z0 - z0 * d2z1 - dz1 * dz0 + dz0 * dz1) / omg_den - dz2 * (-dz1 * z0 + dz0 * z1) / pow(omg_den, 2) + ddpsi;

  return true;
}

bool Controller::flatnessTransportNew(const Eigen::Vector3d &acc,
                                      const Eigen::Vector3d &jer,
                                      const Eigen::Vector3d &snp,
                                      const Eigen::Vector3d &indi,
                                      const Eigen::Vector3d &q,
                                      const Eigen::Vector3d &dq,
                                      const Eigen::Vector3d &d2q,
                                      const double &F,
                                      const double &dF,
                                      const double &d2F,
                                      const double &psi,
                                      const double &dpsi,
                                      const double &ddpsi,
                                      double &thr,
                                      Eigen::Vector3d &torque,
                                      Eigen::Quaterniond &quat,
                                      Eigen::Vector3d &omg,
                                      Eigen::Vector3d &domg,
                                      const Eigen::Vector3d &J,
                                      const double &massQ,
                                      const double &grav,
                                      const double &za) const
{
  const double almost_zero = 1.0e-6;

  double zu_sqr_norm, zu_norm, zu_norm3, zu_norm5;
  double c_half_psi, s_half_psi, c_psi, s_psi;
  double z0, z1, z2, dz0, dz1, dz2, d2z0, d2z1, d2z2, zu0, zu1, zu2, zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
  double omg_den, zudzu, dzudzu, zuddzu;
  double dzu0, dzu1, dzu2, ddzu0, ddzu1, ddzu2, ddz0, ddz1, ddz2;
  double tilt0, tilt1, tilt2, tilt_den;
  double massFrac = 1.0 / massQ;
  double R00, R10, R20, R01, R11, R21;

  zu0 = acc(0) - indi(0);
  zu1 = acc(1) - indi(1);
  zu2 = acc(2) - indi(2) + grav;

  zu_sqr_norm = zu0 * zu0 + zu1 * zu1 + zu2 * zu2;
  zu_norm = sqrt(zu_sqr_norm);
  z0 = zu0 / zu_norm;
  z1 = zu1 / zu_norm;
  z2 = zu2 / zu_norm;
  zu_norm3 = zu_sqr_norm * zu_norm;
  zu_norm5 = zu_sqr_norm * zu_sqr_norm * zu_norm;

  dzu0 = jer(0) + massFrac * (F * dq(0) + dF * q(0));
  dzu1 = jer(1) + massFrac * (F * dq(1) + dF * q(1));
  dzu2 = jer(2) + massFrac * (F * dq(2) + dF * q(2));

  ddzu0 = snp(0) + massFrac * (F * d2q(0) + 2 * dF * dq(0) + d2F * q(0));
  ddzu1 = snp(1) + massFrac * (F * d2q(1) + 2 * dF * dq(1) + d2F * q(1));
  ddzu2 = snp(2) + massFrac * (F * d2q(2) + 2 * dF * dq(2) + d2F * q(2));

  zudzu = zu0 * dzu0 + zu1 * dzu1 + zu2 * dzu2;
  dzudzu = dzu0 * dzu0 + dzu1 * dzu1 + dzu2 * dzu2;
  zuddzu = zu0 * ddzu0 + zu1 * ddzu1 + zu2 * ddzu2;

  dz0 = dzu0 / zu_norm - (zudzu * zu0) / zu_norm3;
  dz1 = dzu1 / zu_norm - (zudzu * zu1) / zu_norm3;
  dz2 = dzu2 / zu_norm - (zudzu * zu2) / zu_norm3;
  ddz0 = ddzu0 / zu_norm - ((dzudzu + zuddzu) * zu0 + 2 * zudzu * dzu0) / zu_norm3 + 3 * zudzu * zudzu * zu0 / zu_norm5;
  ddz1 = ddzu1 / zu_norm - ((dzudzu + zuddzu) * zu1 + 2 * zudzu * dzu1) / zu_norm3 + 3 * zudzu * zudzu * zu1 / zu_norm5;
  ddz2 = ddzu2 / zu_norm - ((dzudzu + zuddzu) * zu2 + 2 * zudzu * dzu2) / zu_norm3 + 3 * zudzu * zudzu * zu2 / zu_norm5;

  thr = massQ * zu_norm;

  tilt_den = sqrt(2.0 * (1.0 + z2));
  tilt0 = 0.5 * tilt_den;
  tilt1 = -z1 / tilt_den;
  tilt2 = z0 / tilt_den;
  c_half_psi = cos(0.5 * psi);
  s_half_psi = sin(0.5 * psi);
  quat.w() = tilt0 * c_half_psi;
  quat.x() = tilt1 * c_half_psi + tilt2 * s_half_psi;
  quat.y() = tilt2 * c_half_psi - tilt1 * s_half_psi;
  quat.z() = tilt0 * s_half_psi;

  R00 = 1 - 2 * quat.y() * quat.y() - 2 * quat.z() * quat.z();
  R10 = 2 * quat.x() * quat.y() + 2 * quat.w() * quat.z();
  R20 = 2 * quat.x() * quat.z() - 2 * quat.w() * quat.y();
  R01 = 2 * quat.x() * quat.y() - 2 * quat.w() * quat.z();
  R11 = 1 - 2 * quat.x() * quat.x() - 2 * quat.z() * quat.z();
  R21 = 2 * quat.y() * quat.z() + 2 * quat.w() * quat.x();

  c_psi = cos(psi);
  s_psi = sin(psi);
  omg_den = z2 + 1.0;

  omg(0) = dz0 * s_psi - dz1 * c_psi - dz2 * (z0 * s_psi - z1 * c_psi) / omg_den;
  omg(1) = dz0 * c_psi + dz1 * s_psi - dz2 * (z0 * c_psi + z1 * s_psi) / omg_den;
  omg(2) = (z1 * dz0 - z0 * dz1) / omg_den;

  domg(0) = ddz0 * s_psi - ddz1 * c_psi + dz0 * c_psi * dpsi + dz1 * s_psi * dpsi - s_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den + c_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * s_psi - z1 * c_psi) / (omg_den * omg_den);
  domg(1) = ddz0 * c_psi + ddz1 * s_psi - dz0 * s_psi * dpsi + dz1 * c_psi * dpsi - c_psi * (ddz2 * z0 + dz2 * dz0 + dz2 * z1 * dpsi) / omg_den - s_psi * (ddz2 * z1 + dz2 * dz1 - dz2 * z0 * dpsi) / omg_den + dz2 * dz2 * (z0 * c_psi + z1 * s_psi) / (omg_den * omg_den);
  domg(2) = (z1 * ddz0 - z0 * ddz1) / omg_den - dz2 * (z1 * dz0 - z0 * dz1) / (omg_den * omg_den);

  torque(0) = J(0) * domg(0) + (J(2) - J(1)) * omg(1) * omg(2) - za * F * (R01 * q(0) + R11 * q(1) + R21 * q(2));
  torque(1) = J(1) * domg(1) + (J(0) - J(2)) * omg(0) * omg(2) + za * F * (R00 * q(0) + R10 * q(1) + R20 * q(2));
  torque(2) = J(2) * domg(2) + (J(1) - J(0)) * omg(1) * omg(0);

  return true;
}

void Controller::minimumSingularityFlatWithDrag(const double mass,
                                                const double grav,
                                                const Eigen::Vector3d &vel,
                                                const Eigen::Vector3d &acc,
                                                const Eigen::Vector3d &jer,
                                                const double &yaw,
                                                const double &yawd,
                                                const Eigen::Quaterniond &att_est,
                                                Eigen::Quaterniond &att,
                                                Eigen::Vector3d &omg,
                                                double &thrust) const
{

  // Drag effect parameters (Drag may cause larger tracking error in aggressive flight during our tests)
  // dv >= dh is required
  // dv is the rotor drag effect in vertical direction, typical value is 0.35
  // dh is the rotor drag effect in horizontal direction, typical value is 0.25
  // cp is the second-order drag effect, typical valye is 0.01
  // const double dh = 0.10;
  // const double dv = 0.23;
  // const double cp = 0.01;
  const double dh = 0.00;
  const double dv = 0.00;
  const double cp = 0.00;

  // veps is a smnoothing constant, do not change it
  const double veps = 0.02; // ms^-s

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ()).norm();

  Eigen::Vector4d quat;
  if (flatnessWithDrag(vel, acc, jer, yaw, yawd,
                       thrust, quat, omg,
                       mass, grav, dh, dv, cp, veps))
  {
    att = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3));
    omg_old = omg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    thrust = thrust_old;
  }

  return;
}

void Controller::minimumSingularityFlat(const double mass,
                                        const double grav,
                                        const Eigen::Vector3d &vel,
                                        const Eigen::Vector3d &acc,
                                        const Eigen::Vector3d &jer,
                                        const Eigen::Vector3d &snp,
                                        const Eigen::Vector3d &indi,
                                        const double &yaw,
                                        const double &yawd,
                                        const double &yawd2,
                                        const Eigen::Quaterniond &att_est,
                                        Eigen::Quaterniond &att,
                                        Eigen::Vector3d &omg,
                                        Eigen::Vector3d &domg,
                                        double &thrust) const
{

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d domg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ()).norm();

  Eigen::Quaterniond quat;
  if (flatnessTransport(acc, jer, snp, indi, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0, yaw, yawd, yawd2,
                        thrust, quat, omg, domg, mass, grav))
  {
    att = quat;
    omg_old = omg;
    domg_old = domg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    domg = domg_old;
    thrust = thrust_old;
  }

  return;
}

void Controller::minimumSingularityFlatNew(const double mass,
                                           const double grav,
                                           const double za,
                                           const Eigen::Vector3d &J,
                                           const Eigen::Vector3d &vel,
                                           const Eigen::Vector3d &acc,
                                           const Eigen::Vector3d &jer,
                                           const Eigen::Vector3d &snp,
                                           const Eigen::Vector3d &indi,
                                           const double &yaw,
                                           const double &yawd,
                                           const double &yawd2,
                                           const Eigen::Quaterniond &att_est,
                                           Eigen::Quaterniond &att,
                                           Eigen::Vector3d &omg,
                                           Eigen::Vector3d &domg,
                                           Eigen::Vector3d &torque,
                                           double &thrust) const
{

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d domg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ()).norm();

  Eigen::Quaterniond quat;
  if (flatnessTransportNew(acc, jer, snp, indi, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0, yaw, yawd, yawd2,
                           thrust, torque, quat, omg, domg, J, mass, grav, za))
  {
    att = quat;
    omg_old = omg;
    domg_old = domg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    domg = domg_old;
    thrust = thrust_old;
  }

  return;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void Controller::minimumSingularityTransportFlatNew(const double mass,
                                                 const double grav,
                                                 const double za,
                                                 const Eigen::Vector3d &J,
                                                 const Eigen::Vector3d &vel,
                                                 const Eigen::Vector3d &acc,
                                                 const Eigen::Vector3d &jer,
                                                 const Eigen::Vector3d &snp,
                                                 const Eigen::Vector3d &indi,
                                                 const Eigen::Vector3d &q,
                                                 const Eigen::Vector3d &dq,
                                                 const Eigen::Vector3d &d2q,
                                                 const double &F,
                                                 const double &dF,
                                                 const double &d2F,
                                                 const double &yaw,
                                                 const double &yawd,
                                                 const double &yawd2,
                                                 const Eigen::Quaterniond &att_est,
                                                 Eigen::Quaterniond &att,
                                                 Eigen::Vector3d &omg,
                                                 Eigen::Vector3d &domg,
                                                 Eigen::Vector3d &torque,
                                                 double &thrust) const
{

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d domg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ() + (F / mass) * q).norm();

  Eigen::Quaterniond quat;
  if (flatnessTransportNew(acc, jer, snp, indi, q, dq, d2q, F, dF, d2F, yaw, yawd, yawd2,
                        thrust, torque, quat, omg, domg, J, mass, grav, za))
  {
    att = quat;
    omg_old = omg;
    domg_old = domg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    domg = domg_old;
    thrust = thrust_old;
  }

  return;
}

void Controller::minimumSingularityTransportFlat(const double mass,
                                                 const double grav,
                                                 const Eigen::Vector3d &vel,
                                                 const Eigen::Vector3d &acc,
                                                 const Eigen::Vector3d &jer,
                                                 const Eigen::Vector3d &snp,
                                                 const Eigen::Vector3d &indi,
                                                 const Eigen::Vector3d &q,
                                                 const Eigen::Vector3d &dq,
                                                 const Eigen::Vector3d &d2q,
                                                 const double &F,
                                                 const double &dF,
                                                 const double &d2F,
                                                 const double &yaw,
                                                 const double &yawd,
                                                 const double &yawd2,
                                                 const Eigen::Quaterniond &att_est,
                                                 Eigen::Quaterniond &att,
                                                 Eigen::Vector3d &omg,
                                                 Eigen::Vector3d &domg,
                                                 double &thrust) const
{

  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d domg_old(0.0, 0.0, 0.0);
  static double thrust_old = mass * (acc + grav * Eigen::Vector3d::UnitZ() + (F / mass) * q).norm();

  Eigen::Quaterniond quat;
  if (flatnessTransport(acc, jer, snp, indi, q, dq, d2q, F, dF, d2F, yaw, yawd, yawd2,
                        thrust, quat, omg, domg, mass, grav))
  {
    att = quat;
    omg_old = omg;
    domg_old = domg;
    thrust_old = thrust;
  }
  else
  {
    ROS_WARN("Conor case: 1. Eactly inverted flight or 2. Unactuated falling");
    att = att_est;
    omg = omg_old;
    domg = domg_old;
    thrust = thrust_old;
  }

  return;
}

/************* Algorithm0 from Zhepei Wang, end ***************/

/************* Algorithm1 from the Zhepei Wang, start ***************/
quadrotor_msgs::Px4ctrlDebug Controller::update_alg1(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param);
  Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(pid_error_accelerations, des.a);

  debug.fb_a_x = pid_error_accelerations(0); // debug
  debug.fb_a_y = pid_error_accelerations(1);
  debug.fb_a_z = pid_error_accelerations(2);
  debug.des_a_x = total_des_acc(0);
  debug.des_a_y = total_des_acc(1);
  debug.des_a_z = total_des_acc(2);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  Eigen::Quaterniond desired_attitude;
  computeFlatInput(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q, desired_attitude, u.bodyrates);
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  debug.des_q_w = desired_attitude.w(); // debug
  debug.des_q_x = desired_attitude.x();
  debug.des_q_y = desired_attitude.y();
  debug.des_q_z = desired_attitude.z();

  u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  u.bodyrates += feedback_bodyrates;

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();

  return debug; // debug
};
void Controller::normalizeWithGrad(const Eigen::Vector3d &x,
                                   const Eigen::Vector3d &xd,
                                   Eigen::Vector3d &xNor,
                                   Eigen::Vector3d &xNord) const
{
  const double xSqrNorm = x.squaredNorm();
  const double xNorm = sqrt(xSqrNorm);
  xNor = x / xNorm;
  xNord = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
  return;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void Controller::computeFlatInput(const Eigen::Vector3d &thr_acc,
                                  const Eigen::Vector3d &jer,
                                  const double &yaw,
                                  const double &yawd,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  Eigen::Vector3d &omg) const
{
  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);

  if (thr_acc.norm() < kMinNormalizedCollectiveAcc_)
  {
    att = att_est;
    omg.setConstant(0.0);
    // ROS_WARN("Conor case, thrust is too small, thr_acc.norm()=%f", thr_acc.norm());
    return;
  }
  else
  {
    Eigen::Vector3d zb, zbd;
    normalizeWithGrad(thr_acc, jer, zb, zbd);
    double syaw = sin(yaw);
    double cyaw = cos(yaw);
    Eigen::Vector3d xc(cyaw, syaw, 0.0);
    Eigen::Vector3d xcd(-syaw * yawd, cyaw * yawd, 0.0);
    Eigen::Vector3d yc = zb.cross(xc);
    if (yc.norm() < kAlmostZeroValueThreshold_)
    {
      ROS_WARN("Conor case, pitch is close to 90 deg");
      att = att_est;
      omg = omg_old;
    }
    else
    {
      Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
      Eigen::Vector3d yb, ybd;
      normalizeWithGrad(yc, ycd, yb, ybd);
      Eigen::Vector3d xb = yb.cross(zb);
      Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
      omg(0) = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
      omg(1) = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
      omg(2) = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;
      Eigen::Matrix3d rotM;
      rotM << xb, yb, zb;
      att = Eigen::Quaterniond(rotM);
      omg_old = omg;
    }
  }
  return;
}
/************* Algorithm1 from Zhepei Wang, end ***************/

/************* Algorithm from the rotor-drag paper, start ***************/
void Controller::update_alg2(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  // Compute reference inputs that compensate for aerodynamic drag
  Eigen::Vector3d drag_acc = Eigen::Vector3d::Zero();
  computeAeroCompensatedReferenceInputs(des, odom, param, &u, &drag_acc);

  // Compute desired control commands
  const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param);
  Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(pid_error_accelerations, des.a, drag_acc);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(total_des_acc, des.yaw, odom.q);
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  if (param.use_bodyrate_ctrl)
  {
    u.bodyrates += feedback_bodyrates;
    // u.bodyrates = imu.q * odom.q.inverse() * u.bodyrates;  // Align with FCU frame
  }
  else
  {
    u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  }

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();
};

void Controller::computeAeroCompensatedReferenceInputs(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Parameter_t &param,
    Controller_Output_t *outputs,
    Eigen::Vector3d *drag_acc) const
{

  const double dx = param.rt_drag.x;
  const double dy = param.rt_drag.y;
  const double dz = param.rt_drag.z;

  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()));

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  const Eigen::Vector3d alpha =
      des.a - Gravity + dx * des.v;
  const Eigen::Vector3d beta =
      des.a - Gravity + dy * des.v;
  const Eigen::Vector3d gamma =
      des.a - Gravity + dz * des.v;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, odom.q);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm()))
  {
    const Eigen::Vector3d z_B_estimated =
        odom.q * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm()))
    {
      y_B = y_C;
    }
    else
    {
      y_B.normalize();
    }
  }
  else
  {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref(
      (Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  outputs->q = Eigen::Quaterniond(R_W_B_ref);

  // Reference thrust
  outputs->thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates
  const double B1 = outputs->thrust -
                    (dz - dx) * z_B.dot(des.v);
  const double C1 = -(dx - dy) * y_B.dot(des.v);
  const double D1 = x_B.dot(des.j) +
                    dx * x_B.dot(des.a);
  const double A2 = outputs->thrust +
                    (dy - dz) * z_B.dot(des.v);
  const double C2 = (dx - dy) * x_B.dot(des.v);
  const double D2 = -y_B.dot(des.j) -
                    dy * y_B.dot(des.a);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = des.yaw_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator))
  {
    outputs->bodyrates = Eigen::Vector3d::Zero();
  }
  else
  {
    // Compute body rates
    if (almostZero(A2))
    {
      outputs->bodyrates.x() = 0.0;
    }
    else
    {
      outputs->bodyrates.x() =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
    outputs->bodyrates.y() = (-C1 * D3 + C3 * D1) / denominator;
    outputs->bodyrates.z() = (B1 * D3 - B3 * D1) / denominator;
  }

  // Transform reference rates and derivatives into estimated body frame
  const Eigen::Matrix3d R_trans =
      odom.q.toRotationMatrix().transpose() * R_W_B_ref;
  const Eigen::Vector3d bodyrates_ref = outputs->bodyrates;

  outputs->bodyrates = R_trans * bodyrates_ref;

  // Drag accelerations
  *drag_acc = -1.0 * (R_W_B_ref * (D * (R_W_B_ref.transpose() * des.v)));
}

Eigen::Quaterniond Controller::computeDesiredAttitude(
    const Eigen::Vector3d &des_acc, const double reference_heading,
    const Eigen::Quaterniond &est_q) const
{
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  // Eigen::Vector3d des_acc2(-1.0,-1.0,-1.0);

  Eigen::Vector3d z_B;
  if (almostZero(des_acc.norm()))
  {
    // In case of free fall we keep the thrust direction to be the estimated one
    // This only works assuming that we are in this condition for a very short
    // time (otherwise attitude drifts)
    z_B = est_q * Eigen::Vector3d::UnitZ();
  }
  else
  {
    z_B = des_acc.normalized();
  }

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, est_q);

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

Eigen::Vector3d Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
    const Eigen::Vector3d &y_C,
    const Eigen::Quaterniond &est_q) const
{
  Eigen::Vector3d x_B = x_B_prototype;

  // cout << "x_B.norm()=" << x_B.norm() << endl;

  if (almostZero(x_B.norm()))
  {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        est_q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm()))
    {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    }
    else
    {
      x_B = x_B_projected.normalized();
    }
  }
  else
  {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  if (x_B.dot(x_C) < 0.0)
  {
    x_B = -x_B;
    // std::cout << "CCCCCCCCCCCCCC" << std::endl;
  }

  return x_B;
}
/************* Algorithm from the rotor-drag paper, end ***************/

// // for single quadrotor without load
// quadrotor_msgs::Px4ctrlDebug Controller::update_alg3(
//     Desired_State_t &des,
//     const Odom_Data_t &odom,
//     const RPM_Data_t &rpm_data,
//     const Imu_Data_t &imu,
//     Controller_Output_t &u,
//     const bool &is_hover_mode,
//     const bool &is_command_mode)
// {
//   // Check the given velocity is valid.
//   if (des.v(2) < -3.0)
//     ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

//   const Eigen::Matrix3d J = Eigen::Vector3d(param.inertia[0], param.inertia[1], param.inertia[2]).asDiagonal();

//   // get body torque and world external force
//   double est_thrust;
//   // Eigen::Vector3d controlTorque = getBodyControlTorque(rpm_data.rpm_fil);
//   Eigen::Vector3d controlTorque;
//   controlTorque.x() = rpm_data.x_sum_rpm2_fil * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k;
//   controlTorque.y() = rpm_data.y_sum_rpm2_fil * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k;
//   controlTorque.z() = rpm_data.z_sum_rpm2_fil * param.thr_map.torque_k;
//   // est_thrust = getBodyThrust(rpm_data.rpm_fil);
//   est_thrust = rpm_data.sum_rpm2_fil * param.thr_map.thrust_k;

//   static Eigen::Vector3d current_q = Eigen::Vector3d::Zero();
//   static double current_F = 0.0;
//   Eigen::Vector3d pid_error_accelerations, translational_acc;
//   Eigen::Vector3d extTotalAccIndi = odom.q * imu.af - (est_thrust / param.mass) * (odom.q * Eigen::Vector3d::UnitZ());
//   // extTotalAccIndi.z() = 0.0;
//   Eigen::Vector3d indiq = -extTotalAccIndi.normalized();
//   double indiF = param.mass * extTotalAccIndi.norm();
//   double psai = atan2(indiq.y(),indiq.x()) * 180 / M_PI;
//   double theta = acos(indiq.z()) * 180 / M_PI;
//   debug.extF_psi = psai;
//   debug.extF_theta = theta;
//   // extTotalAccIndi.setZero();
//   // indiq.setZero();
//   // indiF = 0;

//   Eigen::Quaterniond desired_attitude, ref_attitude;
//   Eigen::Vector3d ref_bodyrates, ref_bodyaccs;
//   double thrust, ref_thrust;

//   // cout << des.flag << endl;

//   if (des.flag == 0)
//   {
//     // Compute desired control commands
//     current_q = indiq;
//     current_F = param.mass * indiF;
//     pid_error_accelerations = computeIndiPIDErrorAcc(odom, imu, des, param, is_command_mode, is_hover_mode);
//     // Eigen::Vector3d pid_error_accelerations_fil;
//     // static filters::biquad_cascade_df1 hfil_pos[3];
//     // static bool first_rcv = false;
//     // if(first_rcv == false)
//     // {
//     //   for(int i = 0; i <= 2; i++)
//     //   {
//     //     hfil_pos[i].param.param_333_50_order2();
//     //   }
//     // }
//     // first_rcv = true;
//     // for(int i = 0; i <= 2; i++)
//     // {
//     //   hfil_pos[i].step_with_scale(&pid_error_accelerations[i], &pid_error_accelerations_fil[i], 1);
//     // }

//     translational_acc = pid_error_accelerations + des.a;
//     translational_acc = (Gravity + extTotalAccIndi + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndi, 1.0)).eval();
//     minimumSingularityFlat(param.mass, param.gra,
//                            des.v, des.a, des.j, des.s, Eigen::Vector3d::Zero(), des.yaw, des.yaw_rate,
//                            des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_thrust);

//     minimumSingularityFlat(param.mass, param.gra,
//                            des.v, translational_acc, des.j, des.s, extTotalAccIndi, des.yaw, des.yaw_rate,
//                            des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, thrust);
//   }
//   else if (des.flag == 1 || des.flag == 2)
//   {
//     if (des.flag == 1)
//     {
//       // des.q = current_q;
//       // des.F = current_F;
//       des.q = param.hover_q;
//       des.F = param.hover_F;
//     }
//     pid_error_accelerations = computeTransportIndiPIDErrorAcc(odom, imu, des, param, indiq, indiF);
//     translational_acc = pid_error_accelerations + des.a;
//     translational_acc = (Gravity + extTotalAccIndi + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndi, 1.0)).eval();

//     minimumSingularityTransportFlat(param.mass, param.gra, des.v, des.a, des.j, des.s, -(des.F * des.q) / param.mass, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
//                                     des.yaw, des.yaw_rate, des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_thrust);
//     minimumSingularityTransportFlat(param.mass, param.gra, des.v, translational_acc, des.j, des.s, extTotalAccIndi, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
//                                     des.yaw, des.yaw_rate, des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, thrust);

//   }

//   Eigen::Vector3d thrustforce = desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
//   Eigen::Vector3d total_des_acc = computeLimitedTotalAccFromThrustForce(thrustforce, param.mass);
//   Eigen::Vector3d body_z_axis = odom.q * Eigen::Vector3d::UnitZ();
//   double des_acc_norm = total_des_acc.dot(body_z_axis);

//   u.thrust = des_acc_norm * param.mass;
//   const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

//   u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
//   const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

//   // limit the angular acceleration
//   u.bodyrates = computeLimitedAngularAcc(bodyrate_candidate);
//   // u.bodyrates += feedback_bodyrates;

//   u.bodyaccs += computeFeedBackControlBodyaccs(desired_attitude, odom.q, u.bodyrates, imu.wf, param, is_hover_mode, debug);
//   // u.bodyaccs.z() = feedback_bodyrates.z();

//   u.torque_des_body = controlTorque + J * (u.bodyaccs - imu.dw);

//   u.torque_des_body.z() = (J * u.bodyaccs).z() + param.thr_map.torque_z_comp + (param.inertia[1] - param.inertia[0]) * (u.bodyrates.x() * u.bodyrates.y());
//   // u.bodyrates.cross(J * u.bodyrates).z();
//   // u.torque_des_body.z() = (J * u.bodyaccs).z() + param.thr_map.torque_z_comp;

//   u.torque_des_body.x() = limit(u.torque_des_body.x(), -0.15, 0.15);
//   u.torque_des_body.y() = limit(u.torque_des_body.y(), -0.15, 0.15);
//   u.torque_des_body.z() = limit(u.torque_des_body.z(), -0.01, 0.01);

//   Eigen::Vector4d thrust_and_torque;
//   thrust_and_torque(0) = u.thrust;
//   thrust_and_torque.segment<3>(1) = u.torque_des_body;
//   Eigen::Vector4d cmd_rpm = (param.thr_map.mix_inv * thrust_and_torque).col(0).array().sqrt() * 1e4;

//   double rpm_Limit = param.rpm_Limit;

//   cmd_rpm = calculateLimitedRPM(cmd_rpm, rpm_Limit, thrust_and_torque);

//   for (int i = 0 ;i < 4 ;i++)
//   {
//     if(isnan(cmd_rpm[i]))
//     {
//       cmd_rpm[i] = 8000;
//     }
//     else
//       cmd_rpm[i] = limit(cmd_rpm[i], 8000, 31000);
//   }

//   // 计算期望油门
//   //  cmd_rpm[0] = 15000;
//   //  cmd_rpm[1] = 15000;
//   //  cmd_rpm[2] = 15000;
//   //  cmd_rpm[3] = 15000;
//   rpm_ctrl_allthrust(cmd_rpm, rpm_data.rpm_fil, u.motor_thrust, is_hover_mode, debug);
//   // cout << "aaaaaaaaa" << endl;

//   // Used for thrust-accel mapping estimation
//   timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
//   while (timed_thrust.size() > 100)
//     timed_thrust.pop();

//   debug.ref_thr = ref_thrust;
//   debug.des_thr = u.thrust;
//   debug.est_thr = est_thrust;
//   debug.des_p_x = des.p.x();
//   debug.des_p_y = des.p.y();
//   debug.des_p_z = des.p.z();

//   debug.ref_v_x = des.v.x();
//   debug.ref_v_y = des.v.y();
//   debug.ref_v_z = des.v.z();
//   debug.fb_a_x = pid_error_accelerations(0); // debug
//   debug.fb_a_y = pid_error_accelerations(1);
//   debug.fb_a_z = pid_error_accelerations(2);
//   // debug.fb_a_x_fil = pid_error_accelerations_fil(0); // debug
//   // debug.fb_a_y_fil = pid_error_accelerations_fil(1);
//   // debug.fb_a_z_fil = pid_error_accelerations_fil(2);

//   debug.ref_a_x = des.a(0);
//   debug.ref_a_y = des.a(1);
//   debug.ref_a_z = des.a(2);
//   debug.ref_j_x = des.j(0);
//   debug.ref_j_y = des.j(1);
//   debug.ref_j_z = des.j(2);
//   debug.ref_s_x = des.s(0);
//   debug.ref_s_y = des.s(1);
//   debug.ref_s_z = des.s(2);

//   debug.des_yaw = des.yaw;
//   debug.des_yaw_rate = des.yaw_rate;
//   debug.des_yaw_acc = des.yaw_acc;

//   const Eigen::Quaterniond q_e = ref_attitude.inverse() * desired_attitude;
//   Eigen::AngleAxisd rotation_vector(q_e); // debug
//   Eigen::Vector3d axis = rotation_vector.axis();
//   debug.fb_axisang_x = axis(0);
//   debug.fb_axisang_y = axis(1);
//   debug.fb_axisang_z = axis(2);
//   debug.fb_axisang_ang = rotation_vector.angle() * 180.0 / M_PI;
//   debug.des_q_w = desired_attitude.w(); // debug
//   debug.des_q_x = desired_attitude.x();
//   debug.des_q_y = desired_attitude.y();
//   debug.des_q_z = desired_attitude.z();
//   Eigen::Vector3d rpy;
//   rpy = quat_to_rpy(ref_attitude);
//   debug.ref_r = rpy.x();
//   debug.ref_p = rpy.y();
//   debug.ref_y = rpy.z();
//   rpy = quat_to_rpy(desired_attitude);
//   debug.des_r = rpy.x();
//   debug.des_p = rpy.y();
//   debug.des_y = rpy.z();
//   rpy = quat_to_rpy(odom.q);
//   debug.est_r = rpy.x();
//   debug.est_p = rpy.y();
//   debug.est_y = rpy.z();

//   Eigen::Vector3d aw = odom.q * imu.af + Gravity;
//   debug.imu_af_x = imu.af.x();
//   debug.imu_af_y = imu.af.y();
//   debug.imu_af_z = imu.af.z() - param.gra;
//   debug.aw_x = aw.x();
//   debug.aw_y = aw.y();
//   debug.aw_z = aw.z();
//   debug.imu_wf_x = imu.wf.x();
//   debug.imu_wf_y = imu.wf.y();
//   debug.imu_wf_z = imu.wf.z();
//   debug.imu_dw_x = imu.dw.x();
//   debug.imu_dw_y = imu.dw.y();
//   debug.imu_dw_z = imu.dw.z();
//   debug.des_bodyrates_x = u.bodyrates.x();
//   debug.des_bodyrates_y = u.bodyrates.y();
//   debug.des_bodyrates_z = u.bodyrates.z();
//   debug.des_bodyaccs_x = u.bodyaccs.x();
//   debug.des_bodyaccs_y = u.bodyaccs.y();
//   debug.des_bodyaccs_z = u.bodyaccs.z();
//   debug.ref_bodyrates_x = ref_bodyrates.x();
//   debug.ref_bodyrates_y = ref_bodyrates.y();
//   debug.ref_bodyrates_z = ref_bodyrates.z();
//   debug.rpm_f1 = rpm_data.rpm_fil[0];
//   debug.rpm_f2 = rpm_data.rpm_fil[1];
//   debug.rpm_f3 = rpm_data.rpm_fil[2];
//   debug.rpm_f4 = rpm_data.rpm_fil[3];
//   debug.cmd_rpm1 = cmd_rpm(0);
//   debug.cmd_rpm2 = cmd_rpm(1);
//   debug.cmd_rpm3 = cmd_rpm(2);
//   debug.cmd_rpm4 = cmd_rpm(3);
//   debug.thro1 = u.motor_thrust[0];
//   debug.thro2 = u.motor_thrust[1];
//   debug.thro3 = u.motor_thrust[2];
//   debug.thro4 = u.motor_thrust[3];

//   debug.des_torque_x = thrust_and_torque(1);
//   debug.des_torque_y = thrust_and_torque(2);
//   debug.des_torque_z = thrust_and_torque(3);
//   debug.cont_torque_x = controlTorque(0);
//   debug.cont_torque_y = controlTorque(1);
//   debug.cont_torque_z = controlTorque(2);

//   debug.extTotalAccIndi_x = extTotalAccIndi.x();
//   debug.extTotalAccIndi_y = extTotalAccIndi.y();
//   debug.extTotalAccIndi_z = extTotalAccIndi.z();
//   debug.indi_F = indiF;
//   debug.indiF_x = -extTotalAccIndi.x() * param.mass;
//   debug.indiF_y = -extTotalAccIndi.y() * param.mass;
//   debug.indiF_z = -extTotalAccIndi.z() * param.mass;
//   debug.desFq_x = des.F * des.q.x();
//   debug.desFq_y = des.F * des.q.y();
//   debug.desFq_z = des.F * des.q.z();

//   debug.z_jwj = (param.inertia[1] - param.inertia[0]) * (u.bodyrates.x() * u.bodyrates.y());

//   debug.flag = des.flag;
//   return debug; // debug
// };

quadrotor_msgs::Px4ctrlDebug Controller::update_alg3(
    Desired_State_t &des,
    const Odom_Data_t &odom,
    const RPM_Data_t &rpm_data,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    const bool &is_hover_mode,
    const bool &is_command_mode)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  const Eigen::Vector3d J = Eigen::Vector3d(param.inertia[0], param.inertia[1], param.inertia[2]);

  // get body torque and world external force
  double est_thrust;
  Eigen::Vector3d controlTorque;
  controlTorque.x() = rpm_data.x_sum_rpm2_fil * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k;
  controlTorque.y() = rpm_data.y_sum_rpm2_fil * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k;
  controlTorque.z() = rpm_data.z_sum_rpm2_fil * param.thr_map.torque_k;
  est_thrust = rpm_data.sum_rpm2_fil * param.thr_map.thrust_k;

  Eigen::Vector3d pid_error_accelerations, translational_acc;
  Eigen::Vector3d extTotalAccIndi = odom.q * imu.af - (est_thrust / param.mass) * (odom.q * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d indiq = -extTotalAccIndi.normalized();
  Eigen::Quaterniond desired_attitude, ref_attitude;
  Eigen::Vector3d ref_bodyrates, ref_bodyaccs, ref_torque, torque;
  Eigen::Vector4d ref_thrust_and_torque, thrust_and_torque, ref_rpm, cmd_rpm;
  static Eigen::Vector3d lastExtTotalAccIndi = Eigen::Vector3d(0.0, 0.0, 0.0); 
  static Eigen::Quaterniond lastOdomq(1.0, 0.0, 0.0, 0.0);
  double thrust, ref_thrust;

  double indiF = param.mass * extTotalAccIndi.norm();
  double psai = atan2(indiq.y(), indiq.x()) * 180 / M_PI;
  double theta = acos(indiq.z()) * 180 / M_PI;
  double indiFLimit;
  Eigen::Vector3d extTotalAccIndiLimit;
  debug.extF_psi = psai;
  debug.extF_theta = theta;

  if (des.flag == 0)
  {
    indiFLimit = min(param.FUpperLimit * param.hover_F, indiF);
    debug.FUpper = param.FUpperLimit * param.hover_F;
    debug.FLower = param.FLowerLimit * 0;

    extTotalAccIndiLimit = -indiFLimit * indiq / param.mass;

    pid_error_accelerations = computeIndiPIDErrorAcc(odom, imu, des, param, is_command_mode, is_hover_mode);
    translational_acc = pid_error_accelerations + des.a;
    translational_acc = (Gravity + extTotalAccIndiLimit + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndiLimit, 1.0)).eval();
    minimumSingularityFlatNew(param.mass, param.gra, param.z_attach, J,
                           des.v, des.a, des.j, des.s, Eigen::Vector3d::Zero(), des.yaw, des.yaw_rate,
                           des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_torque, ref_thrust);

    minimumSingularityFlatNew(param.mass, param.gra, param.z_attach, J,
                           des.v, translational_acc, des.j, des.s, extTotalAccIndiLimit, des.yaw, des.yaw_rate,
                           des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, torque, thrust);
  }
  else if (des.flag == 1 || des.flag == 2)
  {
    if (des.flag == 1)
    {
      des.q = param.hover_q;
      des.F = param.hover_F;
    }
    indiFLimit = max(min(param.FUpperLimit * des.F, indiF), param.FLowerLimit * des.F);
    debug.FUpper = param.FUpperLimit * des.F;
    debug.FLower = param.FLowerLimit * des.F;
    extTotalAccIndiLimit = -indiFLimit * indiq / param.mass;
    pid_error_accelerations = computeTransportIndiPIDErrorAcc(odom, imu, des, param, indiq, indiFLimit);
    translational_acc = pid_error_accelerations + des.a;
    translational_acc = (Gravity + extTotalAccIndiLimit + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndiLimit, 1.0)).eval();

    minimumSingularityTransportFlatNew(param.mass, param.gra, param.z_attach, J, des.v, des.a, des.j, des.s, -(des.F * des.q) / param.mass, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
                                    des.yaw, des.yaw_rate, des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_torque, ref_thrust);
    minimumSingularityTransportFlatNew(param.mass, param.gra, param.z_attach, J, des.v, translational_acc, des.j, des.s, extTotalAccIndiLimit, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
                                    des.yaw, des.yaw_rate, des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, torque, thrust);
  }

  Eigen::Vector3d thrustforce = desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d total_des_acc = computeLimitedTotalAccFromThrustForce(thrustforce, param.mass);
  Eigen::Vector3d body_z_axis = odom.q * Eigen::Vector3d::UnitZ();
  double des_acc_norm = total_des_acc.dot(body_z_axis);

  u.thrust = des_acc_norm * param.mass;
  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

  // limit the angular acceleration
  u.bodyrates = computeLimitedAngularAcc(bodyrate_candidate);
  u.bodyaccs += computeFeedBackControlBodyaccs(desired_attitude, odom.q, u.bodyrates, imu.wf, param, is_hover_mode, debug);
  u.torque_des_body = controlTorque + J.cwiseProduct(u.bodyaccs - imu.dw);
  // u.torque_des_body = controlTorque + J.cwiseProduct(u.bodyaccs - imu.dw) + param.mass * Eigen::Vector3d(0.0, 0.0, param.z_attach).cross(lastOdomq.inverse() * lastExtTotalAccIndi - odom.q.inverse() * extTotalAccIndi);
  u.torque_des_body.z() = J(2) * u.bodyaccs(2) + param.thr_map.torque_z_comp + (param.inertia[1] - param.inertia[0]) * (u.bodyrates.x() * u.bodyrates.y());

  u.torque_des_body.x() = limit(u.torque_des_body.x(), -0.15, 0.15);
  u.torque_des_body.y() = limit(u.torque_des_body.y(), -0.15, 0.15);
  u.torque_des_body.z() = limit(u.torque_des_body.z(), -0.01, 0.01);

  thrust_and_torque(0) = u.thrust;
  thrust_and_torque.segment<3>(1) = u.torque_des_body;
  cmd_rpm = (param.thr_map.mix_inv * thrust_and_torque).col(0).array().sqrt() * 1e4;

  double rpm_Limit = param.rpm_Limit;
  cmd_rpm = calculateLimitedRPM(cmd_rpm, rpm_Limit, thrust_and_torque);
  for (int i = 0; i < 4; i++)
  {
    if (isnan(cmd_rpm[i]))
    {
      cmd_rpm[i] = 8000;
    }
    else
      cmd_rpm[i] = limit(cmd_rpm[i], 8000, 31000);
  }

  ref_thrust_and_torque(0) = ref_thrust;
  ref_thrust_and_torque.segment<3>(1) = ref_torque;
  ref_rpm = (param.thr_map.mix_inv * ref_thrust_and_torque).col(0).array().sqrt() * 1e4;
  lastExtTotalAccIndi = extTotalAccIndi;
  lastOdomq = odom.q;

  // 计算期望油门
  //  cmd_rpm[0] = 15000;
  //  cmd_rpm[1] = 15000;
  //  cmd_rpm[2] = 15000;
  //  cmd_rpm[3] = 15000;
  rpm_ctrl_allthrust(cmd_rpm, rpm_data.rpm_fil, u.motor_thrust, is_hover_mode, debug);

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();

  debug.ref_thr = ref_thrust;
  debug.des_thr = u.thrust;
  debug.est_thr = est_thrust;
  debug.des_p_x = des.p.x();
  debug.des_p_y = des.p.y();
  debug.des_p_z = des.p.z();

  debug.ref_v_x = des.v.x();
  debug.ref_v_y = des.v.y();
  debug.ref_v_z = des.v.z();
  debug.fb_a_x = pid_error_accelerations(0); // debug
  debug.fb_a_y = pid_error_accelerations(1);
  debug.fb_a_z = pid_error_accelerations(2);

  debug.ref_a_x = des.a(0);
  debug.ref_a_y = des.a(1);
  debug.ref_a_z = des.a(2);
  debug.ref_j_x = des.j(0);
  debug.ref_j_y = des.j(1);
  debug.ref_j_z = des.j(2);
  debug.ref_s_x = des.s(0);
  debug.ref_s_y = des.s(1);
  debug.ref_s_z = des.s(2);

  debug.des_yaw = des.yaw;
  debug.des_yaw_rate = des.yaw_rate;
  debug.des_yaw_acc = des.yaw_acc;

  const Eigen::Quaterniond q_e = ref_attitude.inverse() * desired_attitude;
  Eigen::AngleAxisd rotation_vector(q_e); // debug
  Eigen::Vector3d axis = rotation_vector.axis();
  debug.fb_axisang_x = axis(0);
  debug.fb_axisang_y = axis(1);
  debug.fb_axisang_z = axis(2);
  debug.fb_axisang_ang = rotation_vector.angle() * 180.0 / M_PI;
  debug.des_q_w = desired_attitude.w(); // debug
  debug.des_q_x = desired_attitude.x();
  debug.des_q_y = desired_attitude.y();
  debug.des_q_z = desired_attitude.z();
  Eigen::Vector3d rpy;
  rpy = quat_to_rpy(ref_attitude);
  debug.ref_r = rpy.x();
  debug.ref_p = rpy.y();
  debug.ref_y = rpy.z();
  rpy = quat_to_rpy(desired_attitude);
  debug.des_r = rpy.x();
  debug.des_p = rpy.y();
  debug.des_y = rpy.z();
  rpy = quat_to_rpy(odom.q);
  debug.est_r = rpy.x();
  debug.est_p = rpy.y();
  debug.est_y = rpy.z();

  Eigen::Vector3d aw = odom.q * imu.af + Gravity;
  debug.imu_af_x = imu.af.x();
  debug.imu_af_y = imu.af.y();
  debug.imu_af_z = imu.af.z() - param.gra;
  debug.aw_x = aw.x();
  debug.aw_y = aw.y();
  debug.aw_z = aw.z();
  debug.imu_wf_x = imu.wf.x();
  debug.imu_wf_y = imu.wf.y();
  debug.imu_wf_z = imu.wf.z();
  debug.imu_dw_x = imu.dw.x();
  debug.imu_dw_y = imu.dw.y();
  debug.imu_dw_z = imu.dw.z();
  debug.des_bodyrates_x = u.bodyrates.x();
  debug.des_bodyrates_y = u.bodyrates.y();
  debug.des_bodyrates_z = u.bodyrates.z();
  debug.des_bodyaccs_x = u.bodyaccs.x();
  debug.des_bodyaccs_y = u.bodyaccs.y();
  debug.des_bodyaccs_z = u.bodyaccs.z();
  debug.ref_bodyrates_x = ref_bodyrates.x();
  debug.ref_bodyrates_y = ref_bodyrates.y();
  debug.ref_bodyrates_z = ref_bodyrates.z();
  debug.rpm_f1 = rpm_data.rpm_fil[0];
  debug.rpm_f2 = rpm_data.rpm_fil[1];
  debug.rpm_f3 = rpm_data.rpm_fil[2];
  debug.rpm_f4 = rpm_data.rpm_fil[3];
  debug.cmd_rpm1 = cmd_rpm(0);
  debug.cmd_rpm2 = cmd_rpm(1);
  debug.cmd_rpm3 = cmd_rpm(2);
  debug.cmd_rpm4 = cmd_rpm(3);
  debug.ref_rpm1 = ref_rpm(0);
  debug.ref_rpm2 = ref_rpm(1);
  debug.ref_rpm3 = ref_rpm(2);
  debug.ref_rpm4 = ref_rpm(3);
  debug.thro1 = u.motor_thrust[0];
  debug.thro2 = u.motor_thrust[1];
  debug.thro3 = u.motor_thrust[2];
  debug.thro4 = u.motor_thrust[3];

  debug.des_torque_x = thrust_and_torque(1);
  debug.des_torque_y = thrust_and_torque(2);
  debug.des_torque_z = thrust_and_torque(3);
  debug.ref_torque_x = ref_thrust_and_torque(1);
  debug.ref_torque_y = ref_thrust_and_torque(2);
  debug.ref_torque_z = ref_thrust_and_torque(3);
  debug.cont_torque_x = controlTorque(0);
  debug.cont_torque_y = controlTorque(1);
  debug.cont_torque_z = controlTorque(2);

  debug.extTotalAccIndi_x = extTotalAccIndi.x();
  debug.extTotalAccIndi_y = extTotalAccIndi.y();
  debug.extTotalAccIndi_z = extTotalAccIndi.z();
  debug.indi_F = indiF;
  debug.indi_F_Limit = indiFLimit;
  debug.indiF_x = -extTotalAccIndi.x() * param.mass;
  debug.indiF_y = -extTotalAccIndi.y() * param.mass;
  debug.indiF_z = -extTotalAccIndi.z() * param.mass;
  debug.desFq_x = des.F * des.q.x();
  debug.desFq_y = des.F * des.q.y();
  debug.desFq_z = des.F * des.q.z();

  debug.z_jwj = (param.inertia[1] - param.inertia[0]) * (u.bodyrates.x() * u.bodyrates.y());

  debug.flag = des.flag;
  return debug; // debug
};

// for quadrotor with load
quadrotor_msgs::Px4ctrlDebug Controller::update_alg4(
    Desired_State_t &des,
    const Odom_Data_t &odom,
    const RPM_Data_t &rpm_data,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    const bool &is_hover_mode,
    const bool &is_command_mode,
    double voltage)
{
  // Check the given velocity is valid.
  if (des.v(2) < -3.0)
    ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

  const Eigen::Matrix3d J = Eigen::Vector3d(param.inertia[0], param.inertia[1], param.inertia[2]).asDiagonal();

  // get body torque and world external force
  double est_thrust;
  Eigen::Vector3d controlTorque;
  est_thrust = rpm_data.sum_rpm2_fil * param.thr_map.thrust_k;

  static Eigen::Vector3d current_q = Eigen::Vector3d::Zero();
  static double current_F = 0.0;
  Eigen::Vector3d pid_error_accelerations, translational_acc;
  Eigen::Vector3d extTotalAccIndi = odom.q * imu.af - (est_thrust / param.mass) * (odom.q * Eigen::Vector3d::UnitZ());
  // extTotalAccIndi.z() = 0.0;
  Eigen::Vector3d indiq = -extTotalAccIndi.normalized();
  double indiF = param.mass * extTotalAccIndi.norm();
  double psai = atan2(indiq.y(), indiq.x()) * 180 / M_PI;
  double theta = acos(indiq.z()) * 180 / M_PI;
  debug.extF_psi = psai;
  debug.extF_theta = theta;
  // extTotalAccIndi.setZero();
  // indiq.setZero();
  // indiF = 0;

  Eigen::Quaterniond desired_attitude, ref_attitude;
  Eigen::Vector3d ref_bodyrates, ref_bodyaccs;
  double thrust, ref_thrust;

  if (des.flag == 0)
  {
    // Compute desired control commands
    current_q = indiq;
    current_F = param.mass * indiF;
    pid_error_accelerations = computeIndiPIDErrorAcc(odom, imu, des, param, is_command_mode, is_hover_mode);

    translational_acc = pid_error_accelerations + des.a;
    translational_acc = (Gravity + extTotalAccIndi + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndi, 1.0)).eval();
    minimumSingularityFlat(param.mass, param.gra,
                           des.v, des.a, des.j, des.s, Eigen::Vector3d::Zero(), des.yaw, des.yaw_rate,
                           des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_thrust);

    minimumSingularityFlat(param.mass, param.gra,
                           des.v, translational_acc, des.j, des.s, extTotalAccIndi, des.yaw, des.yaw_rate,
                           des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, thrust);
  }
  else if (des.flag == 1 || des.flag == 2)
  {
    if (des.flag == 1)
    {
      // des.q = current_q;
      // des.F = current_F;
      des.q = param.hover_q;
      des.F = param.hover_F;
    }
    pid_error_accelerations = computeTransportIndiPIDErrorAcc(odom, imu, des, param, indiq, indiF);
    translational_acc = pid_error_accelerations + des.a;
    translational_acc = (Gravity + extTotalAccIndi + computeLimitedTotalAccFromThrustForce(translational_acc - Gravity - extTotalAccIndi, 1.0)).eval();

    minimumSingularityTransportFlat(param.mass, param.gra, des.v, des.a, des.j, des.s, -(des.F * des.q) / param.mass, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
                                    des.yaw, des.yaw_rate, des.yaw_acc, odom.q, ref_attitude, ref_bodyrates, ref_bodyaccs, ref_thrust);
    minimumSingularityTransportFlat(param.mass, param.gra, des.v, translational_acc, des.j, des.s, extTotalAccIndi, des.q, des.dq, des.d2q, des.F, des.dF, des.d2F,
                                    des.yaw, des.yaw_rate, des.yaw_acc, odom.q, desired_attitude, u.bodyrates, u.bodyaccs, thrust);
  }

  Eigen::Vector3d thrustforce = desired_attitude * (thrust * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d total_des_acc = computeLimitedTotalAccFromThrustForce(thrustforce, param.mass);

  u.thrust = computeDesiredCollectiveThrustSignal(odom.q, odom.v, total_des_acc, param, voltage);

  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q, param);

  u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
  const Eigen::Vector3d bodyrate_candidate = u.bodyrates + feedback_bodyrates;

  // limit the angular acceleration
  u.bodyrates = computeLimitedAngularAcc(bodyrate_candidate);
  // u.bodyrates += feedback_bodyrates;

  // Used for thrust-accel mapping estimation
  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();

  debug.ref_thr = ref_thrust;
  debug.des_thr = u.thrust;
  debug.est_thr = est_thrust;
  debug.des_p_x = des.p.x();
  debug.des_p_y = des.p.y();
  debug.des_p_z = des.p.z();

  debug.ref_v_x = des.v.x();
  debug.ref_v_y = des.v.y();
  debug.ref_v_z = des.v.z();
  debug.fb_a_x = pid_error_accelerations(0); // debug
  debug.fb_a_y = pid_error_accelerations(1);
  debug.fb_a_z = pid_error_accelerations(2);
  // debug.fb_a_x_fil = pid_error_accelerations_fil(0); // debug
  // debug.fb_a_y_fil = pid_error_accelerations_fil(1);
  // debug.fb_a_z_fil = pid_error_accelerations_fil(2);

  debug.ref_a_x = des.a(0);
  debug.ref_a_y = des.a(1);
  debug.ref_a_z = des.a(2);
  debug.ref_j_x = des.j(0);
  debug.ref_j_y = des.j(1);
  debug.ref_j_z = des.j(2);
  debug.ref_s_x = des.s(0);
  debug.ref_s_y = des.s(1);
  debug.ref_s_z = des.s(2);

  debug.des_yaw = des.yaw;
  debug.des_yaw_rate = des.yaw_rate;
  debug.des_yaw_acc = des.yaw_acc;

  const Eigen::Quaterniond q_e = ref_attitude.inverse() * desired_attitude;
  Eigen::AngleAxisd rotation_vector(q_e); // debug
  Eigen::Vector3d axis = rotation_vector.axis();
  debug.fb_axisang_x = axis(0);
  debug.fb_axisang_y = axis(1);
  debug.fb_axisang_z = axis(2);
  debug.fb_axisang_ang = rotation_vector.angle() * 180.0 / M_PI;
  debug.des_q_w = desired_attitude.w(); // debug
  debug.des_q_x = desired_attitude.x();
  debug.des_q_y = desired_attitude.y();
  debug.des_q_z = desired_attitude.z();
  Eigen::Vector3d rpy;
  rpy = quat_to_rpy(ref_attitude);
  debug.ref_r = rpy.x();
  debug.ref_p = rpy.y();
  debug.ref_y = rpy.z();
  rpy = quat_to_rpy(desired_attitude);
  debug.des_r = rpy.x();
  debug.des_p = rpy.y();
  debug.des_y = rpy.z();
  rpy = quat_to_rpy(odom.q);
  debug.est_r = rpy.x();
  debug.est_p = rpy.y();
  debug.est_y = rpy.z();

  Eigen::Vector3d aw = odom.q * imu.af + Gravity;
  debug.imu_af_x = imu.af.x();
  debug.imu_af_y = imu.af.y();
  debug.imu_af_z = imu.af.z() - param.gra;
  debug.aw_x = aw.x();
  debug.aw_y = aw.y();
  debug.aw_z = aw.z();
  debug.imu_wf_x = imu.wf.x();
  debug.imu_wf_y = imu.wf.y();
  debug.imu_wf_z = imu.wf.z();
  debug.imu_dw_x = imu.dw.x();
  debug.imu_dw_y = imu.dw.y();
  debug.imu_dw_z = imu.dw.z();
  debug.des_bodyrates_x = u.bodyrates.x();
  debug.des_bodyrates_y = u.bodyrates.y();
  debug.des_bodyrates_z = u.bodyrates.z();
  debug.ref_bodyrates_x = ref_bodyrates.x();
  debug.ref_bodyrates_y = ref_bodyrates.y();
  debug.ref_bodyrates_z = ref_bodyrates.z();
  debug.rpm_f1 = rpm_data.rpm_fil[0];
  debug.rpm_f2 = rpm_data.rpm_fil[1];
  debug.rpm_f3 = rpm_data.rpm_fil[2];
  debug.rpm_f4 = rpm_data.rpm_fil[3];

  debug.extTotalAccIndi_x = extTotalAccIndi.x();
  debug.extTotalAccIndi_y = extTotalAccIndi.y();
  debug.extTotalAccIndi_z = extTotalAccIndi.z();
  debug.indi_F = indiF;
  debug.indiF_x = -extTotalAccIndi.x() * param.mass;
  debug.indiF_y = -extTotalAccIndi.y() * param.mass;
  debug.indiF_z = -extTotalAccIndi.z() * param.mass;
  debug.desFq_x = des.F * des.q.x();
  debug.desFq_y = des.F * des.q.y();
  debug.desFq_z = des.F * des.q.z();

  debug.z_jwj = (param.inertia[1] - param.inertia[0]) * (u.bodyrates.x() * u.bodyrates.y());

  debug.flag = des.flag;
  return debug; // debug
};

Eigen::Vector3d Controller::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q,
    const Parameter_t &param)
{
  // Compute the error quaternion
  const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

  Eigen::AngleAxisd rotation_vector(q_e); // debug
  Eigen::Vector3d axis = rotation_vector.axis();
  debug.exec_err_axisang_x = axis(0);
  debug.exec_err_axisang_y = axis(1);
  debug.exec_err_axisang_z = axis(2);
  debug.exec_err_axisang_ang = rotation_vector.angle() * 180 / M_PI;

  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;

  if (q_e.w() >= 0)
  {
    bodyrates.x() = 2.0 * KAng(0) * q_e.x();
    bodyrates.y() = 2.0 * KAng(1) * q_e.y();
    bodyrates.z() = 2.0 * KAng(2) * q_e.z();
  }
  else
  {
    bodyrates.x() = -2.0 * KAng(0) * q_e.x();
    bodyrates.y() = -2.0 * KAng(1) * q_e.y();
    bodyrates.z() = -2.0 * KAng(2) * q_e.z();
  }

  if (bodyrates.x() > kMaxBodyratesFeedback_)
    bodyrates.x() = kMaxBodyratesFeedback_;
  if (bodyrates.x() < -kMaxBodyratesFeedback_)
    bodyrates.x() = -kMaxBodyratesFeedback_;
  if (bodyrates.y() > kMaxBodyratesFeedback_)
    bodyrates.y() = kMaxBodyratesFeedback_;
  if (bodyrates.y() < -kMaxBodyratesFeedback_)
    bodyrates.y() = -kMaxBodyratesFeedback_;
  if (bodyrates.z() > kMaxBodyratesFeedback_)
    bodyrates.z() = kMaxBodyratesFeedback_;
  if (bodyrates.z() < -kMaxBodyratesFeedback_)
    bodyrates.z() = -kMaxBodyratesFeedback_;

  // debug
  debug.fb_rate_x = bodyrates.x();
  debug.fb_rate_y = bodyrates.y();
  debug.fb_rate_z = bodyrates.z();

  return bodyrates;
}

Eigen::Vector4d Controller::calculateLimitedRPM(const Eigen::Vector4d &cmd_rpm, const double &rpm_Limit, const Eigen::Vector4d &thrust_and_torque)
{
  double rpm_max = 0;
  double rpm_max_index = 0;
  for (int i = 0; i < 4; i++)
  {
    if (cmd_rpm(i) >= rpm_max)
    {
      rpm_max = cmd_rpm(i);
      rpm_max_index = i;
    }
  }
  if (rpm_max > rpm_Limit)
  {
    Eigen::Vector4d limited_cmd_rpm;
    limited_cmd_rpm(rpm_max_index) = rpm_Limit;
    if (rpm_max_index == 0)
    {
      Eigen::Vector3d limit_torque, limited_rpm, limited_rpm2;
      limit_torque(0) = thrust_and_torque(1) - (-1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(1) = thrust_and_torque(2) - (-1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(2) = thrust_and_torque(3) - (-1 * param.thr_map.torque_k) * rpm_Limit * rpm_Limit;
      limited_rpm2 = (param.thr_map.limit_mix_0_inv * limit_torque).col(0).array() * 1e8;
      if (limited_rpm2(0) < 0)
      {
        limited_cmd_rpm(1) = 0;
      }
      else
      {
        limited_cmd_rpm(1) = sqrt(limited_rpm2(0));
      }
      if (limited_rpm2(1) < 0)
      {
        limited_cmd_rpm(2) = 0;
      }
      else
      {
        limited_cmd_rpm(2) = sqrt(limited_rpm2(1));
      }
      if (limited_rpm2(2) < 0)
      {
        limited_cmd_rpm(3) = 0;
      }
      else
      {
        limited_cmd_rpm(3) = sqrt(limited_rpm2(2));
      }
    }
    if (rpm_max_index == 1)
    {
      Eigen::Vector3d limit_torque, limited_rpm, limited_rpm2;
      limit_torque(0) = thrust_and_torque(1) - (1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(1) = thrust_and_torque(2) - (1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(2) = thrust_and_torque(3) - (-1 * param.thr_map.torque_k) * rpm_Limit * rpm_Limit;
      limited_rpm2 = (param.thr_map.limit_mix_1_inv * limit_torque).col(0).array() * 1e8;
      if (limited_rpm2(0) < 0)
      {
        limited_cmd_rpm(0) = 0;
      }
      else
      {
        limited_cmd_rpm(0) = sqrt(limited_rpm2(0));
      }
      if (limited_rpm2(1) < 0)
      {
        limited_cmd_rpm(2) = 0;
      }
      else
      {
        limited_cmd_rpm(2) = sqrt(limited_rpm2(1));
      }
      if (limited_rpm2(2) < 0)
      {
        limited_cmd_rpm(3) = 0;
      }
      else
      {
        limited_cmd_rpm(3) = sqrt(limited_rpm2(2));
      }
    }
    if (rpm_max_index == 2)
    {
      Eigen::Vector3d limit_torque, limited_rpm, limited_rpm2;
      limit_torque(0) = thrust_and_torque(1) - (1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(1) = thrust_and_torque(2) - (-1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(2) = thrust_and_torque(3) - (1 * param.thr_map.torque_k) * rpm_Limit * rpm_Limit;
      ;
      limited_rpm2 = (param.thr_map.limit_mix_2_inv * limit_torque).col(0).array() * 1e8;
      if (limited_rpm2(0) < 0)
      {
        limited_cmd_rpm(0) = 0;
      }
      else
      {
        limited_cmd_rpm(0) = sqrt(limited_rpm2(0));
      }
      if (limited_rpm2(1) < 0)
      {
        limited_cmd_rpm(1) = 0;
      }
      else
      {
        limited_cmd_rpm(1) = sqrt(limited_rpm2(1));
      }
      if (limited_rpm2(2) < 0)
      {
        limited_cmd_rpm(3) = 0;
      }
      else
      {
        limited_cmd_rpm(3) = sqrt(limited_rpm2(2));
      }
    }
    if (rpm_max_index == 3)
    {
      Eigen::Vector3d limit_torque, limited_rpm, limited_rpm2;
      limit_torque(0) = thrust_and_torque(1) - (-1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(1) = thrust_and_torque(2) - (1 * sqrt(2) / 4.0 * param.wheelbase * param.thr_map.thrust_k * 1.0) * rpm_Limit * rpm_Limit;
      limit_torque(2) = thrust_and_torque(3) - (1 * param.thr_map.torque_k) * rpm_Limit * rpm_Limit;
      ;
      limited_rpm2 = (param.thr_map.limit_mix_3_inv * limit_torque).col(0).array() * 1e8;
      if (limited_rpm2(0) < 0)
      {
        limited_cmd_rpm(0) = 0;
      }
      else
      {
        limited_cmd_rpm(0) = sqrt(limited_rpm2(0));
      }
      if (limited_rpm2(1) < 0)
      {
        limited_cmd_rpm(1) = 0;
      }
      else
      {
        limited_cmd_rpm(1) = sqrt(limited_rpm2(1));
      }
      if (limited_rpm2(2) < 0)
      {
        limited_cmd_rpm(2) = 0;
      }
      else
      {
        limited_cmd_rpm(2) = sqrt(limited_rpm2(2));
      }
    }
    return limited_cmd_rpm;
  }
  else
  {
    Eigen::Vector4d limited_cmd_rpm;
    limited_cmd_rpm[0] = cmd_rpm[0];
    limited_cmd_rpm[1] = cmd_rpm[1];
    limited_cmd_rpm[2] = cmd_rpm[2];
    limited_cmd_rpm[3] = cmd_rpm[3];
    return limited_cmd_rpm;
  }
}

Eigen::Vector3d Controller::computeFeedBackControlBodyaccs(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q,
    const Eigen::Vector3d &des_bodyrate,
    const Eigen::Vector3d &est_bodyrate,
    const Parameter_t &param,
    const bool &is_hover_mode,
    quadrotor_msgs::Px4ctrlDebug &debug)
{
  static double z_rate_error_sum = 0;
  if (is_hover_mode == false)
    z_rate_error_sum = 0;
  Eigen::Vector3d rate_error = est_q.inverse() * des_q * des_bodyrate - est_bodyrate;
  Eigen::Vector3d feedback_bodyaccs;

  z_rate_error_sum += limit(rate_error.z(), -1.0, 1.0);
  z_rate_error_sum = limit(z_rate_error_sum, -param.gain.KiMax, param.gain.KiMax);
  feedback_bodyaccs.x() = param.gain.KRateRoll * rate_error.x();
  feedback_bodyaccs.y() = param.gain.KRatePitch * rate_error.y();
  feedback_bodyaccs.z() = param.gain.KRateYaw * rate_error.z() + param.gain.KiRateYaw * z_rate_error_sum;

  debug.rate_error_z = rate_error.z();
  debug.rate_error_sum_z = z_rate_error_sum;
  debug.fbp_bodyacc_z = param.gain.KRateYaw * rate_error.z();
  debug.fbi_bodyacc_z = param.gain.KiRateYaw * z_rate_error_sum;
  debug.des_est_bodyrates_x = (est_q.inverse() * des_q * des_bodyrate).x();
  debug.des_est_bodyrates_y = (est_q.inverse() * des_q * des_bodyrate).y();
  debug.des_est_bodyrates_z = (est_q.inverse() * des_q * des_bodyrate).z();

  return feedback_bodyaccs;
}

Eigen::Vector3d Controller::computePIDErrorAcc(
    const Odom_Data_t &odom,
    const Desired_State_t &des,
    const Parameter_t &param)
{
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - odom.p(0), 1.0), -1.0);
  double x_vel_error = std::max(std::min((des.v(0) + Kp(0) * x_pos_error) - odom.v(0), 1.0), -1.0);
  acc_error(0) = Kv(0) * x_vel_error;

  // y acceleration
  double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - odom.p(1), 1.0), -1.0);
  double y_vel_error = std::max(std::min((des.v(1) + Kp(1) * y_pos_error) - odom.v(1), 1.0), -1.0);
  acc_error(1) = Kv(1) * y_vel_error;

  // z acceleration
  double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - odom.p(2), 1.0), -1.0);
  double z_vel_error = std::max(std::min((des.v(2) + Kp(2) * z_pos_error) - odom.v(2), 1.0), -1.0);
  acc_error(2) = Kv(2) * z_vel_error;

  debug.des_v_x = (des.v(0) + Kp(0) * x_pos_error); // debug
  debug.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
  debug.des_v_z = (des.v(2) + Kp(2) * z_pos_error);

  return acc_error;
}
Eigen::Vector3d Controller::computeIndiPIDErrorAcc(
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    const Desired_State_t &des,
    const Parameter_t &param,
    const bool &is_command_mode,
    const bool &is_hover_mode)
{
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error, acc, p_attach;
  static double pos_error_rmse = 0;
  static double pos_error_sum = 0;
  static long int count = 0;

  acc = odom.q * imu.af + Gravity;

  p_attach = odom.p + odom.q * Eigen::Vector3d(0.0, 0.0, param.z_attach);

  // x acceleration
  double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - p_attach(0), 2.0), -2.0);
  double x_vel_error = std::max(std::min((des.v(0) + Kp(0) * x_pos_error) - odom.v(0), 6.0), -6.0);
  // double x_acc_error = std::max(std::min((des.a(0) + Kv(0) * x_vel_error) - acc.x(), 1.0), -1.0);

  // y acceleration
  double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - p_attach(1), 2.0), -2.0);
  double y_vel_error = std::max(std::min((des.v(1) + Kp(1) * y_pos_error) - odom.v(1), 6.0), -6.0);
  // double y_acc_error = std::max(std::min((des.a(1) + Kv(1) * y_vel_error) - acc.y(), 1.0), -1.0);

  // z accelerationcomputeIndiPIDErrorAcc
  double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - p_attach(2), 2.0), -2.0);
  double z_vel_error = std::max(std::min((des.v(2) + Kp(2) * z_pos_error) - odom.v(2), 6.0), -6.0);
  // double z_acc_error = std::max(std::min((des.a(2) + Kv(2) * z_vel_error) - acc.z(), 1.0), -1.0);

  acc_error(0) = Kv(0) * x_vel_error;
  acc_error(1) = Kv(1) * y_vel_error;
  acc_error(2) = Kv(2) * z_vel_error;
  // acc_error(2) = 0;

  // acc_error(0) = Ka(0) * x_acc_error;
  // acc_error(1) = Ka(1) * y_acc_error;
  // acc_error(2) = Ka(2) * z_acc_error;

  debug.des_v_x = (des.v(0) + Kp(0) * x_pos_error); // debug
  debug.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
  debug.des_v_z = (des.v(2) + Kp(2) * z_pos_error);
  debug.des_v_all = sqrt(pow(debug.des_v_x, 2) + pow(debug.des_v_y, 2) + pow(debug.des_v_z, 2));
  debug.x_pos_error = x_pos_error;
  debug.y_pos_error = y_pos_error;
  debug.z_pos_error = z_pos_error;
  debug.pos_error_all = sqrt(pow(x_pos_error, 2) + pow(y_pos_error, 2) + pow(z_pos_error, 2));
  debug.pos_error_xy = sqrt(pow(x_pos_error, 2) + pow(y_pos_error, 2));

  if (!is_command_mode)
  {
    pos_error_sum = 0;
    count = 0;
  }
  else
  {
    pos_error_sum += (pow(x_pos_error, 2) + pow(y_pos_error, 2) + pow(z_pos_error, 2));
    count = count + 1;
    pos_error_rmse = sqrt(pos_error_sum / count);
  }

  debug.pos_error_rmse = pos_error_rmse;
  debug.x_vel_error = x_vel_error;
  debug.y_vel_error = y_vel_error;
  debug.z_vel_error = z_vel_error;
  debug.vel_error_all = sqrt(pow(x_vel_error, 2) + pow(y_vel_error, 2) + pow(z_vel_error, 2));

  return acc_error;
}

Eigen::Vector3d Controller::computeTransportIndiPIDErrorAcc(
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    const Desired_State_t &des,
    const Parameter_t &param,
    const Eigen::Vector3d &indiq,
    const double &indiF)
{
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error, acc, acc_pos_error, acc_q_error, acc_F_error, q_error, v_modi, acc_Fq_error, acc_Fq_error_fil, p_attach;
  double F_error;

  acc = odom.q * imu.af + Gravity;

  v_modi = odom.q * imu.wf.cross(Eigen::Vector3d(0.0, 0.0, -0.055));
  p_attach = odom.p + odom.q * Eigen::Vector3d(0.0, 0.0, param.z_attach);

  // x acceleration
  double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - p_attach(0), 2.0), -2.0);
  double x_vel_error = std::max(std::min((des.v(0) + Kp(0) * x_pos_error) - (odom.v(0)), 6.0), -6.0);
  // double x_acc_error = std::max(std::min((des.a(0) + Kv(0) * x_vel_error) - acc.x(), 1.0), -1.0);

  // y acceleration
  double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - p_attach(1), 2.0), -2.0);
  double y_vel_error = std::max(std::min((des.v(1) + Kp(1) * y_pos_error) - (odom.v(1)), 6.0), -6.0);
  // double y_acc_error = std::max(std::min((des.a(1) + Kv(1) * x_vel_error) - acc.y(), 1.0), -1.0);

  // z acceleration
  double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - p_attach(2), 2.0), -2.0);
  double z_vel_error = std::max(std::min((des.v(2) + Kp(2) * z_pos_error) - (odom.v(2)), 6.0), -6.0);
  // double z_acc_error = std::max(std::min((des.a(2) + Kv(2) * z_vel_error) - acc.z(), 1.0), -1.0);

  debug.des_v_x = (des.v(0) + Kp(0) * x_pos_error); // debug
  debug.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
  debug.des_v_z = (des.v(2) + Kp(2) * z_pos_error);

  acc_error(0) = Kv(0) * x_vel_error;
  acc_error(1) = Kv(1) * y_vel_error;
  acc_error(2) = Kv(2) * z_vel_error;

  // acc_error(0) = Ka(0) * x_acc_error;
  // acc_error(1) = Ka(1) * y_acc_error;
  // acc_error(2) = Ka(2) * z_acc_error;

  debug.fb_pv_x = acc_error(0);
  debug.fb_pv_y = acc_error(1);
  debug.fb_pv_z = acc_error(2);

  if (!(std::isnan(des.q(0)) || std::isnan(des.q(1)) || std::isnan(des.q(2)) || std::isnan(des.F)))
  {
    q_error = -indiq.cross(indiq.cross(des.q));
    F_error = des.F - indiF;
    acc_q_error(0) = Kq(0) * std::max(std::min(q_error(0), 2.0), -2.0);
    acc_q_error(1) = Kq(1) * std::max(std::min(q_error(1), 2.0), -2.0);
    acc_q_error(2) = Kq(2) * std::max(std::min(q_error(2), 2.0), -2.0);
    acc_F_error = KF * std::max(std::min(F_error, 6.0), -6.0) * indiq;
    // acc_Fq_error = acc_F_error + acc_q_error;
    // static filters::biquad_cascade_df1 hfil_Fq[3];
    // static bool first_rcv = false;
    // if(first_rcv == false)
    // {
    //   for(int i = 0; i <= 2; i++)
    //   {
    //     hfil_Fq[i].param.param_333_3_order2();
    //   }
    // }
    // first_rcv = true;
    // for(int i = 0; i <= 2; i++)
    // {
    //   hfil_Fq[i].step_with_scale(&acc_Fq_error[i], &acc_Fq_error_fil[i], 1);
    // }

    // acc_error += acc_Fq_error_fil;
    acc_error += acc_q_error;
    acc_error += acc_F_error;
  }

  // cout << "1111111111111"<< endl;

  debug.indi_q_err_ang = acos(indiq.dot(des.q)) * 180 / M_PI;
  debug.des_Fq_psi = atan2(des.q.y(), des.q.x()) * 180 / M_PI;
  ;
  debug.des_Fq_theta = acos(des.q.z()) * 180 / M_PI;
  debug.fb_q_x = acc_q_error(0);
  debug.fb_q_y = acc_q_error(1);
  debug.fb_q_z = acc_q_error(2);
  debug.F_error = std::max(std::min(F_error, 6.0), -6.0);

  debug.fb_F_x = acc_F_error(0);
  debug.fb_F_y = acc_F_error(1);
  debug.fb_F_z = acc_F_error(2);

  // cout <<debug.indi_q_err_ang << endl;
  // cout <<debug.des_Fq_psi << endl;
  // cout <<debug.des_Fq_theta << endl;

  debug.fb_Fq_x = acc_q_error(0) + acc_F_error(0);
  debug.fb_Fq_y = acc_q_error(1) + acc_F_error(1);
  debug.fb_Fq_z = acc_q_error(2) + acc_F_error(2);
  debug.des_F = des.F;

  debug.v_modi_x = v_modi.x();
  debug.v_modi_y = v_modi.y();
  debug.v_modi_z = v_modi.z();

  // debug.fb_Fq_error_fil_x = acc_Fq_error_fil(0);
  // debug.fb_Fq_error_fil_y = acc_Fq_error_fil(1);
  // debug.fb_Fq_error_fil_z = acc_Fq_error_fil(2);
  // cout << "2222222222222"<< endl;

  return acc_error;
}

Eigen::Vector3d Controller::computeLimitedTotalAcc(
    const Eigen::Vector3d &PIDErrorAcc,
    const Eigen::Vector3d &ref_acc,
    const Eigen::Vector3d &drag_acc /*default = Eigen::Vector3d::Zero() */) const
{
  Eigen::Vector3d total_acc;
  total_acc = PIDErrorAcc + ref_acc - Gravity - drag_acc;

  // Limit magnitude
  if (total_acc.norm() < kMinNormalizedCollectiveAcc_)
  {
    total_acc = total_acc.normalized() * kMinNormalizedCollectiveAcc_;
  }

  // Limit angle
  if (param.max_angle > 0)
  {
    double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d z_B = total_acc.normalized();
    if (z_acc < kMinNormalizedCollectiveAcc_)
    {
      z_acc = kMinNormalizedCollectiveAcc_; // Not allow too small z-force when angle limit is enabled.
    }
    Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
    double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
    if (rot_ang > param.max_angle) // Exceed the angle limit
    {
      Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(param.max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
      total_acc = z_acc / std::cos(param.max_angle) * limited_z_B;
    }
  }

  return total_acc;
}

Eigen::Vector3d Controller::computeLimitedAngularAcc(const Eigen::Vector3d candidate_bodyrate)
{
  ros::Time t_now = ros::Time::now();
  if (last_ctrl_timestamp_ != ros::Time(0))
  {
    double dura = (t_now - last_ctrl_timestamp_).toSec();
    // if (dura > 0.004)
    //   ROS_ERROR("!!!!!!!!!!!!!!dura: %.7lf",dura);
    double max_delta_bodyrate = kMaxAngularAcc_ * dura;
    Eigen::Vector3d bodyrate_out;

    if ((candidate_bodyrate(0) - last_bodyrate_(0)) > max_delta_bodyrate)
    {
      bodyrate_out(0) = last_bodyrate_(0) + max_delta_bodyrate;
    }
    else if ((candidate_bodyrate(0) - last_bodyrate_(0)) < -max_delta_bodyrate)
    {
      bodyrate_out(0) = last_bodyrate_(0) - max_delta_bodyrate;
    }
    else
    {
      bodyrate_out(0) = candidate_bodyrate(0);
    }

    if ((candidate_bodyrate(1) - last_bodyrate_(1)) > max_delta_bodyrate)
    {
      bodyrate_out(1) = last_bodyrate_(1) + max_delta_bodyrate;
    }
    else if ((candidate_bodyrate(1) - last_bodyrate_(1)) < -max_delta_bodyrate)
    {
      bodyrate_out(1) = last_bodyrate_(1) - max_delta_bodyrate;
    }
    else
    {
      bodyrate_out(1) = candidate_bodyrate(1);
    }

    if ((candidate_bodyrate(2) - last_bodyrate_(2)) > max_delta_bodyrate)
    {
      bodyrate_out(2) = last_bodyrate_(2) + max_delta_bodyrate;
    }
    else if ((candidate_bodyrate(2) - last_bodyrate_(2)) < -max_delta_bodyrate)
    {
      bodyrate_out(2) = last_bodyrate_(2) - max_delta_bodyrate;
    }
    else
    {
      bodyrate_out(2) = candidate_bodyrate(2);
    }

    last_ctrl_timestamp_ = t_now;
    last_bodyrate_ = bodyrate_out;

    return bodyrate_out;
  }
  else
  {
    last_ctrl_timestamp_ = t_now;
    last_bodyrate_ = candidate_bodyrate;
    return candidate_bodyrate;
  }
}

double Controller::computeDesiredCollectiveThrustSignal(
    const Eigen::Quaterniond &est_q,
    const Eigen::Vector3d &est_v,
    const Eigen::Vector3d &des_acc,
    const Parameter_t &param,
    double voltage)
{

  double normalized_thrust;
  const Eigen::Vector3d body_z_axis = est_q * Eigen::Vector3d::UnitZ();
  double des_acc_norm = des_acc.dot(body_z_axis);
  // double des_acc_norm = des_acc.norm();
  if (des_acc_norm < kMinNormalizedCollectiveAcc_)
  {
    des_acc_norm = kMinNormalizedCollectiveAcc_;
  }

  // This compensates for an acceleration component in thrust direction due
  // to the square of the body-horizontal velocity.
  des_acc_norm -= param.rt_drag.k_thrust_horz * (pow(est_v.x(), 2.0) + pow(est_v.y(), 2.0));

  debug.des_thr = des_acc_norm; // debug

  if (param.thr_map.accurate_thrust_model)
  {
    normalized_thrust = thr_scale_compensate * AccurateThrustAccMapping(des_acc_norm, voltage, param);
  }
  else
  {
    normalized_thrust = des_acc_norm / thr2acc;
  }

  return normalized_thrust;
}

double Controller::AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const Parameter_t &param) const
{
  if (voltage < param.low_voltage)
  {
    voltage = param.low_voltage;
    ROS_ERROR("Low voltage!");
  }
  if (voltage > 1.5 * param.low_voltage)
  {
    voltage = 1.5 * param.low_voltage;
  }

  // F=K1*Voltage^K2*(K3*u^2+(1-K3)*u)
  double a = param.thr_map.K3;
  double b = 1 - param.thr_map.K3;
  double c = -(param.mass * des_acc_z) / (param.thr_map.K1 * pow(voltage, param.thr_map.K2));
  double b2_4ac = pow(b, 2) - 4 * a * c;
  if (b2_4ac <= 0)
    b2_4ac = 0;
  double thrust = (-b + sqrt(b2_4ac)) / (2 * a);
  // if (thrust <= 0) thrust = 0; // This should be avoided before calling this function
  return thrust;
}

bool Controller::almostZero(const double value) const
{
  return fabs(value) < kAlmostZeroValueThreshold_;
}

bool Controller::almostZeroThrust(const double thrust_value) const
{
  return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
}

bool Controller::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const double voltage,
    const Eigen::Vector3d &est_v,
    const Parameter_t &param)
{

  ros::Time t_now = ros::Time::now();
  while (timed_thrust.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust.pop();
    if (param.thr_map.accurate_thrust_model)
    {
      /**************************************************************************/
      /* Model: thr = thr_scale_compensate * AccurateThrustAccMapping(est_a(2)) */
      /**************************************************************************/
      double thr_fb = AccurateThrustAccMapping(est_a(2), voltage, param);
      double gamma = 1 / (rho2 + thr_fb * P * thr_fb);
      double K = gamma * P * thr_fb;
      thr_scale_compensate = thr_scale_compensate + K * (thr - thr_fb * thr_scale_compensate);
      P = (1 - K * thr_fb) * P / rho2;
      // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr_scale_compensate, gamma, K, P);
      // fflush(stdout);

      if (thr_scale_compensate > 1.15 || thr_scale_compensate < 0.85)
      {
        ROS_ERROR("Thrust scale = %f, which shoule around 1. It means the thrust model is nolonger accurate. \
                  Re-calibrate the thrust model!",
                  thr_scale_compensate);
        thr_scale_compensate = thr_scale_compensate > 1.15 ? 1.15 : thr_scale_compensate;
        thr_scale_compensate = thr_scale_compensate < 0.85 ? 0.85 : thr_scale_compensate;
      }

      debug.thr_scale_compensate = thr_scale_compensate; // debug
      debug.voltage = voltage;
      if (param.thr_map.print_val)
      {
        ROS_WARN("thr_scale_compensate = %f", thr_scale_compensate);
      }
    }
    else
    {
      /***********************************/
      /* Model: est_a(2) = thr2acc * thr */
      /***********************************/
      if (!param.thr_map.noisy_imu)
      {
        double gamma = 1 / (rho2 + thr * P * thr);
        double K = gamma * P * thr;
        thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
        P = (1 - K * thr) * P / rho2;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
        // fflush(stdout);
      }
      else // Strongly not recommended to use!!!
      {
        double K = 10 / param.ctrl_freq_max; // thr2acc changes 10 every second when est_v(2) - des_v(2) = 1 m/s
        thr2acc = thr2acc + K * (est_v(2) - debug.des_v_z);
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, K, est_v(2), debug.des_v_z);
        // fflush(stdout);
      }
      const double hover_percentage = param.gra / thr2acc;
      if (hover_percentage > 0.8 || hover_percentage < 0.1)
      {
        // ROS_ERROR("Estimated hover_percentage >0.8 or <0.1! Perhaps the accel vibration is too high!");
        thr2acc = hover_percentage > 0.8 ? param.gra / 0.8 : thr2acc;
        thr2acc = hover_percentage < 0.1 ? param.gra / 0.1 : thr2acc;
      }
      debug.hover_percentage = hover_percentage; // debug
      if (param.thr_map.print_val)
      {
        ROS_WARN("hover_percentage = %f", debug.hover_percentage);
      }
    }

    return true;
  }

  return false;
}

void Controller::resetThrustMapping(void)
{
  thr2acc = param.gra / param.thr_map.hover_percentage;
  thr_scale_compensate = 1.0;
  P = 1e6;
}

double Controller::limit(double val, double min, double max)
{
  if (min > max)
  {
    double temp = min;
    min = max;
    max = temp;
  }
  return std::min(std::max(val, min), max);
}

double Controller::rpm_to_thrust(double rpm)
{
  rpm = limit(rpm, 8000, 31000);
  double ka, kb, kc;
  ka = param.thr_map.thrust_to_rpm2_p2;
  kb = param.thr_map.thrust_to_rpm2_p1;
  kc = param.thr_map.thrust_to_rpm2_p0 - rpm * rpm;
  // ka*th^2 + kb*th + kc = rpm^2
  double thrust = (-kb + sqrt(kb * kb - 4 * ka * kc)) / (2 * ka);
  thrust = limit(thrust, 0, 1);

  return thrust;
}

double Controller::thrust_to_rpm(double thrust)
{
  double lim_thrust = limit(thrust, 0, 1);

  double rpm2;
  rpm2 = param.thr_map.thrust_to_rpm2_p0 + param.thr_map.thrust_to_rpm2_p1 * lim_thrust + param.thr_map.thrust_to_rpm2_p2 * lim_thrust * lim_thrust;

  return sqrt(rpm2);
}

// type 2
// double Controller::rpm_to_thrust(double rpm)
// {
//   rpm = limit(rpm, 6800, 31000);
//   double ka, kb, kc;
//   if (rpm > param.thr_map.model_switch_rpm)
//   {
//     ka = param.thr_map.thrust_to_rpm2_p2;
//     kb = param.thr_map.thrust_to_rpm2_p1;
//     kc = param.thr_map.thrust_to_rpm2_p0 - rpm * rpm;
//     // ka*th^2 + kb*th + kc = rpm^2
//   }
//   else
//   {
//     ka = param.thr_map.thrust_to_rpm2_l2;
//     kb = param.thr_map.thrust_to_rpm2_l1;
//     kc = param.thr_map.thrust_to_rpm2_l0 - rpm * rpm;
//   }
//   double thrust = (-kb + sqrt(kb * kb - 4 * ka * kc)) / (2 * ka);
//   thrust = limit(thrust, 0, 1);

//   return thrust;
// }

// double Controller::thrust_to_rpm(double thrust)
// {
//   double lim_thrust = limit(thrust, 0, 1);

//   double rpm2;
//   if (lim_thrust > param.thr_map.model_switch_thrust)
//   {
//     rpm2 = param.thr_map.thrust_to_rpm2_p0 + param.thr_map.thrust_to_rpm2_p1 * lim_thrust + param.thr_map.thrust_to_rpm2_p2 * lim_thrust * lim_thrust;
//   }
//   else
//   {
//     rpm2 = param.thr_map.thrust_to_rpm2_l0 + param.thr_map.thrust_to_rpm2_l1 * lim_thrust + param.thr_map.thrust_to_rpm2_l2 * lim_thrust * lim_thrust;
//   }

//   return sqrt(rpm2);
// }

double Controller::getK(double thrust)
{
  double lim_thrust = limit(thrust, 0, 1);
  double rpm, k;
  // sqrt(ka*th^2 + kb*th + kc)
  rpm = sqrt(param.thr_map.thrust_to_rpm2_p0 + param.thr_map.thrust_to_rpm2_p1 * lim_thrust + param.thr_map.thrust_to_rpm2_p2 * lim_thrust * lim_thrust);
  k = 0.5 * (param.thr_map.thrust_to_rpm2_p1 + 2 * param.thr_map.thrust_to_rpm2_p2 * lim_thrust) / rpm;

  return k;
}

double Controller::getK2(double thrust)
{
  double lim_thrust = limit(thrust, 0, 1);

  double rpm, k;
  // ka*th^2 + kb*th + kc = rpm^2
  // rpm = sqrt(param.thr_map.thrust_to_rpm2_p0 + param.thr_map.thrust_to_rpm2_p1 * lim_thrust + param.thr_map.thrust_to_rpm2_p2 * lim_thrust * lim_thrust);
  k = (param.thr_map.thrust_to_rpm2_p1 + 2 * param.thr_map.thrust_to_rpm2_p2 * lim_thrust);

  return k;
}

// void Controller::rpm_ctrl_allthrust(
//     Eigen::Vector4d &des_rpms,
//     const double rpm[],
//     double thrust_chs[],
//     const bool &is_hover_mode,
//     quadrotor_msgs::Px4ctrlDebug &debug)
// {
//   static double acc_error_sum = 0;

//   double sum_rpm2, des_rpm_ave, k, ka, kp, ki, des_sum_rpm2;
//   double fbp_thrust, fbi_thrust, fb_thrust, acc_error;
//   double th = 0.03, sp = 6000^2; // th 0.001 , 0.02
//   des_sum_rpm2 = des_rpms.array().pow(2).sum();
//   sum_rpm2 = pow(rpm[0],2) + pow(rpm[1],2) +pow(rpm[2],2) + pow(rpm[3],2);

//   double cofficient = param.thr_map.thrust_k / param.mass;

//   k = getK(rpm_to_thrust(sqrt(des_sum_rpm2/4))); // 4*Kt/m

//   ka = k * cofficient;

//   kp = param.thr_map.rpm_sp * 1 / ka;
//   ki = 0.04 * 1 / ka;
//   // cout <<  (cofficient * getK(rpm_to_thrust(14500))) << endl;

//   acc_error = cofficient * (des_sum_rpm2 - sum_rpm2);
//   acc_error = limit(acc_error, -1, 1);
//   acc_error_sum += acc_error;
//   acc_error_sum = limit(acc_error_sum, -50, 50);

//   fbp_thrust = kp * acc_error;
//   fbi_thrust = ki * acc_error_sum;
//   fb_thrust = limit(fbp_thrust + fbi_thrust, -0.3, 0.3);

//   double ref_thrust[4] = {0};
//   for (int i = 0; i <= 3; i++)
//   {
//     ref_thrust[i] = rpm_to_thrust(des_rpms[i]);
//     thrust_chs[i] = ref_thrust[i] + fb_thrust;
//   }

//   debug.ref_thrust1 = ref_thrust[0];
//   debug.ref_thrust2 = ref_thrust[1];
//   debug.ref_thrust3 = ref_thrust[2];
//   debug.ref_thrust4 = ref_thrust[3];
//   debug.fbp_thrust = fbp_thrust;
//   debug.fbi_thrust = fbi_thrust;
//   debug.fb_thrust = fb_thrust;
//   // debug.des_rpm_ave = des_rpm_ave;
//   // debug.rpm_ave = rpm_ave;
//   // debug.rpm_error = rpm_error;
//   // debug.rpm_error_sum = rpm_error_sum;
// }

// void Controller::rpm_ctrl_allthrust(
//     Eigen::Vector4d &des_rpms,
//     const double sumrpm2,
//     double thrust_chs[],
//     quadrotor_msgs::Px4ctrlDebug &debug)
// {
//   static double rpm_error_sum = 0;

//   double rpm_ave, des_rpm_ave, k, kp, ki;
//   double fbp_thrust, fbi_thrust, rpm_error;
//   double th = 0.03, sp = 6000; // th 0.001 , 0.02
//   des_rpm_ave = sqrt(des_rpms.array().pow(2).sum() / 4.0);
//   rpm_ave = sqrt(sumrpm2 / 4);

//   k = getK(rpm_to_thrust(des_rpm_ave)); // 4*Kt/m
//   kp = 0.4 * 1 / k;
//   ki = 1.0 * th / sp;
//   rpm_error = des_rpm_ave - rpm_ave;
//   rpm_error_sum += rpm_error;
//   rpm_error_sum = limit(rpm_error_sum, -sp, sp);

//   fbp_thrust = limit(kp * rpm_error, -0.1, 0.1);
//   fbi_thrust = limit(ki * rpm_error_sum, -th, th);

//   double ref_thrust[4] = {0};
//   for (int i = 0; i <= 3; i++)
//   {
//     ref_thrust[i] = rpm_to_thrust(des_rpms[i]);
//     thrust_chs[i] = ref_thrust[i] + fbp_thrust + fbi_thrust;
//   }

//   debug.ref_thrust1 = ref_thrust[0];
//   debug.ref_thrust2 = ref_thrust[1];
//   debug.ref_thrust3 = ref_thrust[2];
//   debug.ref_thrust4 = ref_thrust[3];
//   debug.fbp_thrust = fbp_thrust;
//   debug.fbi_thrust = fbi_thrust;
//   debug.des_rpm_ave = des_rpm_ave;
//   debug.rpm_ave = rpm_ave;
//   debug.rpm_error = rpm_error;
//   debug.rpm_error_sum = rpm_error_sum;
// }

void Controller::rpm_ctrl_allthrust(
    Eigen::Vector4d &des_rpms,
    const double rpm[],
    double thrust_chs[],
    const bool &is_hover_mode,
    quadrotor_msgs::Px4ctrlDebug &debug)
{
  static double rpm_error_sum[4] = {0};
  static double acc_error_sum[4] = {0};
  static long int win_num = 0;
  static double window[4][500] = {0};
  static double window_sum[4] = {0};
  double k[4] = {0}, kp[4] = {0}, ki[4] = {0};
  double fbp_thrust[4] = {0}, fbi_thrust[4] = {0}, rpm_error[4] = {0}, fed[4] = {0};
  double th = 0.05; // th 0.001 , 0.02
  double ref_thrust[4] = {0};
  // double des_rpm2 = pow(des_rpms[0],2) + pow(des_rpms[1],2) + pow(des_rpms[2],2) + pow(des_rpms[3],2);
  // double rpm2 = pow(rpm[0],2) + pow(rpm[1],2) + pow(rpm[2],2) + pow(rpm[3],2);
  double acc_error[4] = {0};
  double cofficient = param.thr_map.thrust_k / param.mass;

  // cout << "!!!!!!!!!!!!!!!!!!!:" << is_hover_mode << endl;

  if (is_hover_mode == false)
  {
    for (int i = 0; i < 4; i++)
    {
      rpm_error_sum[i] = 0;
      window_sum[i] = 0;
      acc_error_sum[i] = 0;
      for (int winin = 0; winin < 500; winin++)
        window[i][winin] = 0;
    }

    win_num = 0;
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      k[i] = getK(rpm_to_thrust(des_rpms[i])); // 4*Kt/m
      kp[i] = param.thr_map.rpm_kp * 1 / k[i];
      // ki[i] = 1.0 * th / param.thr_map.rpm_sp;
      ki[i] = (1.0 / param.thr_map.acc_sp) * 1 / (cofficient * getK2(rpm_to_thrust(14500)));
      rpm_error[i] = des_rpms[i] - rpm[i];
      // rpm_error_sum[i] += rpm_error[i];
      acc_error[i] = cofficient * (pow(des_rpms[i], 2) - pow(rpm[i], 2));
      acc_error[i] = limit(acc_error[i], -1.0, 1.0);

      // if (win_num < param.thr_map.win_size)
      // {
      //   window[i][win_num] = acc_error[i];
      //   acc_error_sum[i] += acc_error[i];
      //   window_sum[i] += acc_error[i];
      // }
      // else
      // {
      //   window_sum[i] +=  acc_error[i] - window[i][win_num%param.thr_map.win_size];
      //   window[i][win_num%param.thr_map.win_size] = acc_error[i];
      //   acc_error_sum[i] += window_sum[i] / param.thr_map.win_size;
      // }

      acc_error_sum[i] += acc_error[i];
      // rpm_error_sum[i] = limit(rpm_error_sum[i], -param.thr_map.rpm_sp, param.thr_map.rpm_sp);

      fbp_thrust[i] = limit(kp[i] * rpm_error[i], -0.10, 0.10);
      // fbi_thrust[i] = ki[i] * rpm_error_sum[i];
      fbi_thrust[i] = ki[i] * limit(acc_error_sum[i], -param.thr_map.acc_sp, param.thr_map.acc_sp);
      // fbi_thrust[i] = limit(ki[i] * rpm_error_sum[i], -th, th);
      fed[i] = limit(fbp_thrust[i] + limit(fbi_thrust[i], -0.02, 0.15), -0.15, 0.20);
      // fed[i] = 0;

      // k[i] = getK(rpm_to_thrust(15000));
      // kp[i] = 1.0 * 1 / k[i];
      // fed[i] = -600 * kp[i];

      ref_thrust[i] = rpm_to_thrust(des_rpms[i]);
      // thrust_chs[i] = ref_thrust[i] + fbp_thrust[i] + fbi_thrust[i];
      thrust_chs[i] = ref_thrust[i] + fed[i];
    }
    win_num++;
  }

  debug.ref_thrust1 = ref_thrust[0];
  debug.ref_thrust2 = ref_thrust[1];
  debug.ref_thrust3 = ref_thrust[2];
  debug.ref_thrust4 = ref_thrust[3];
  debug.fbp_thrust1 = fbp_thrust[0];
  debug.fbi_thrust1 = fbi_thrust[0];
  debug.fbp_thrust2 = fbp_thrust[1];
  debug.fbi_thrust2 = fbi_thrust[1];
  debug.fbp_thrust3 = fbp_thrust[2];
  debug.fbi_thrust3 = fbi_thrust[2];
  debug.fbp_thrust4 = fbp_thrust[3];
  debug.fbi_thrust4 = fbi_thrust[3];
  debug.acc_error1 = acc_error[0];
  debug.acc_error2 = acc_error[1];
  debug.acc_error3 = acc_error[2];
  debug.acc_error4 = acc_error[3];
  debug.acc_errorsum1 = acc_error_sum[0];
  debug.acc_errorsum2 = acc_error_sum[1];
  debug.acc_errorsum3 = acc_error_sum[2];
  debug.acc_errorsum4 = acc_error_sum[3];
  debug.window_ave1 = window_sum[0] / param.thr_map.win_size;
  debug.window_ave2 = window_sum[1] / param.thr_map.win_size;
  debug.window_ave3 = window_sum[2] / param.thr_map.win_size;
  debug.window_ave4 = window_sum[3] / param.thr_map.win_size;

  debug.des_rpm_ave = sqrt(des_rpms.array().pow(2).sum() / 4.0);
  debug.rpm_ave = sqrt((pow(rpm[0], 2) + pow(rpm[1], 2) + pow(rpm[2], 2) + pow(rpm[3], 2)) / 4.0);
  debug.rpm_error1 = rpm_error[0];
  debug.rpm_error_sum1 = rpm_error_sum[0];
  debug.rpm_error2 = rpm_error[1];
  debug.rpm_error_sum2 = rpm_error_sum[1];
  debug.rpm_error3 = rpm_error[2];
  debug.rpm_error_sum3 = rpm_error_sum[2];
  debug.rpm_error4 = rpm_error[3];
  debug.rpm_error_sum4 = rpm_error_sum[3];
}

Eigen::Vector3d Controller::quat_to_rpy(const Eigen::Quaterniond &q)
{
  Eigen::Vector3d eulerAngle = q.toRotationMatrix().eulerAngles(2, 1, 0);

  Eigen::Vector3d rpy;
  rpy[0] = eulerAngle[2] * 180.0 / M_PI;
  rpy[1] = eulerAngle[1] * 180.0 / M_PI;
  rpy[2] = eulerAngle[0] * 180.0 / M_PI;

  if (std::fabs(rpy.y()) >= 90.0)
  {
    rpy.y() = 180.0 - rpy.y();
    rpy.x() += 180.0;
    rpy.z() += 180.0;

    for (int i = 0; i <= 2; i++)
    {
      for (int cnt = 0; cnt <= 3; cnt++)
      {
        if (rpy[i] > 180.0)
        {
          rpy[i] -= 360.0;
        }
        if (rpy[i] < -180.0)
        {
          rpy[i] += 360.0;
        }
      }
    }
  }
  return rpy;
}

Eigen::Vector3d Controller::getBodyControlTorque(const double rpm[])
{
  Eigen::Vector3d torque;
  Eigen::Matrix<double, 4, 1> rpm2;
  Eigen::Vector3d body_torque;

  for (int i = 0; i <= 3; i++)
  {
    rpm2(i, 0) = pow(rpm[i], 2);
  }
  body_torque = param.thr_map.mix.block<3, 4>(1, 0) * rpm2;
  return body_torque;
}

double Controller::getBodyThrust(const double rpm[])
{
  double thrust;
  Eigen::Matrix<double, 4, 1> rpm2;

  for (int i = 0; i <= 3; i++)
  {
    rpm2(i, 0) = pow(rpm[i], 2);
  }
  thrust = (param.thr_map.mix.block<1, 4>(0, 0) * rpm2).sum();
  return thrust;
}
