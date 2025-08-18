#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    inline void getDesOmgs(const Eigen::Vector3d &acc,
                           const Eigen::Vector3d &jer,
                           const Eigen::Vector3d &snp,
                           const Eigen::Vector3d &q,
                           const Eigen::Vector3d &dq,
                           const Eigen::Vector3d &d2q,
                           const double &F,
                           const double &dF,
                           const double &d2F,
                           const double &psi,
                           const double &dpsi,
                           const double &ddpsi,
                           Eigen::Vector4d &quat,
                           Eigen::Vector3d &omg,
                           Eigen::Vector3d &domg,
                           const double &massQ,
                           const double &grav)
    {
        double dFq0, dFq1, dFq2, dzu_norm, zu_sqr_norm, zu_norm;
        double d2Fq0, d2Fq1, d2Fq2;
        double c_half_psi, s_half_psi, c_psi, s_psi;
        double dng00, dng01, dng02, dng11, dng12, dng22, ng_den, ng_den4;
        double dz_term0, dz_term1, dz_term2, d2z_term0, d2z_term1, d2z_term2;
        double z0, z1, z2, dz0, dz1, dz2, d2z0, d2z1, d2z2, zu0, zu1, zu2, zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22;
        double omg_den, omg_term;
        double tilt0, tilt1, tilt2, tilt_den;
        double massFrac = 1.0 / massQ;

        zu0 = acc(0) + massFrac * F * q(0);
        zu1 = acc(1) + massFrac * F * q(1);
        zu2 = acc(2) + massFrac * F * q(2) + grav;
        zu_sqr0 = zu0 * zu0;
        zu_sqr1 = zu1 * zu1;
        zu_sqr2 = zu2 * zu2;
        zu01 = zu0 * zu1;
        zu12 = zu1 * zu2;
        zu02 = zu0 * zu2;
        zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
        zu_norm = sqrt(zu_sqr_norm);
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

        return;
    }

    class FlatnessMap  // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
    {
    public:
        inline void reset(const double &vehicle_mass,
                          const double &gravitational_acceleration,
                          const double &cable_length)
        {
            mass = vehicle_mass;
            massFrac = 1.0 / vehicle_mass;
            // cout << mass << "\t" << massFrac<<endl;
            grav = gravitational_acceleration;
            len = cable_length;
            return;
        }

        inline void forward(const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &jer,
                            const Eigen::Vector3d &q,
                            const Eigen::Vector3d &dq,
                            const double &F,
                            const double &dF,
                            const double &psi,
                            const double &dpsi,
                            double &thr,
                            Eigen::Vector4d &quat,
                            Eigen::Vector3d &omg)
        {
            double dFq0, dFq1, dFq2;

            F_ = F;
            dF_ = dF;
            a0 = acc(0);
            a1 = acc(1);
            a2 = acc(2);
            q0 = q(0);
            q1 = q(1);
            q2 = q(2);
            dq0 = dq(0);
            dq1 = dq(1);
            dq2 = dq(2);
            zu0 = a0 + massFrac * F * q0;
            zu1 = a1 + massFrac * F * q1;
            zu2 = a2 + massFrac * F * q2 + grav;
            // cout << "zu\t" << zu0 << "\t" << zu1 << "\t" << zu2 << endl;
            // cout << "a\t" << a0 << "\t" << a1 << "\t" << a2 << "\t" << F<<  endl;
            // cout << "q\t" << q0 << "\t" << q1 << "\t" << q2 << "\t" << grav<< endl;
            zu_sqr0 = zu0 * zu0;
            zu_sqr1 = zu1 * zu1;
            zu_sqr2 = zu2 * zu2;
            zu01 = zu0 * zu1;
            zu12 = zu1 * zu2;
            zu02 = zu0 * zu2;
            zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
            zu_norm = sqrt(zu_sqr_norm);
            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;
            ng_den = zu_sqr_norm * zu_norm;
            ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
            // cout << "ng00\t" << zu_sqr1 << "\t" << zu_sqr2 << "\t" << ng_den << "\t" << zu_norm << endl;
            ng01 = -zu01 / ng_den;
            ng02 = -zu02 / ng_den;
            ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
            // cout << "ng11\t" << zu_sqr0 << "\t" << zu_sqr2 << "\t" << ng_den << "\t" << zu_norm<< endl;
            ng12 = -zu12 / ng_den;
            ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
            dFq0 = massFrac * (F * dq0 + dF * q0);
            dFq1 = massFrac * (F * dq1 + dF * q1);
            dFq2 = massFrac * (F * dq2 + dF * q2);
            dz_term0 = jer(0) + dFq0;
            dz_term1 = jer(1) + dFq1;
            dz_term2 = jer(2) + dFq2;
            dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
            dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
            dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
            
            f_term0 = mass * a0 + F * q0;
            f_term1 = mass * a1 + F * q1;
            f_term2 = mass * (a2 + grav) + F * q2;
            thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
            // cout << "f_term0\t" << a0 << "\t" << F << "\t" << q0 << endl;
            // cout << "f_term1\t" << a1 << "\t" << F << "\t" << q1 << endl;
            // cout << "f_term2\t" << a2 << "\t" << F << "\t" << q2 << endl;
            // cout << "thr\t" << z0 << "\t" << f_term0 << "\t" << z1 << "\t" <<  f_term1 << "\t" << z2 << "\t" << f_term2<< endl;
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
            c_psi = cos(psi);
            s_psi = sin(psi);
            omg_den = z2 + 1.0;
            omg_term = dz2 / omg_den;
            omg(0) = dz0 * s_psi - dz1 * c_psi -
                     (z0 * s_psi - z1 * c_psi) * omg_term;
            omg(1) = dz0 * c_psi + dz1 * s_psi -
                     (z0 * c_psi + z1 * s_psi) * omg_term;
            omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

            return;
        }

        inline void backward(const Eigen::Vector3d &q_grad,
                             const Eigen::Vector3d &dq_grad,
                             const double &F_grad,
                             const double &thr_grad,
                             const Eigen::Vector4d &quat_grad,
                             const Eigen::Vector3d &omg_grad,
                             Eigen::Vector3d &q_total_grad,
                             Eigen::Vector3d &dq_total_grad,
                             Eigen::Vector3d &d2q_total_grad,
                             Eigen::Vector3d &d3q_total_grad,
                             double &F_total_grad,
                             double &dF_total_grad,
                             Eigen::Vector3d &acc_total_grad,
                             Eigen::Vector3d &jer_total_grad,
                             double &psi_total_grad,
                             double &dpsi_total_grad,
                             bool &is_quat_grad_not_zero,
                             bool &is_omg_grad_not_zero) const
        {
            double tempb;
            double a0b, a1b, a2b, j0b, j1b, j2b, q0b, q1b, q2b, dq0b, dq1b, dq2b;
            double dFq0b, dFq1b, dFq2b;
            double z0b, z1b, z2b, dz0b, dz1b, dz2b;
            double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
            double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
            double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
            double dz_term0b, dz_term1b, dz_term2b, f_term0b, f_term1b, f_term2b;
            double tilt_denb, tilt0b, tilt1b, tilt2b, c_half_psib, s_half_psib;
            double c_psib, s_psib, omg_denb, omg_termb;

            // is_quat_grad_not_zero = true;
            // is_omg_grad_not_zero = true;

            if (is_quat_grad_not_zero)
            {
                tilt0b = s_half_psi * quat_grad(3) + c_half_psi * quat_grad(0);
                s_half_psib = tilt0 * quat_grad(3) + tilt2 * quat_grad(1) - tilt1 * quat_grad(2);
                tilt2b = c_half_psi * quat_grad(2) + s_half_psi * quat_grad(1);
                c_half_psib = tilt2 * quat_grad(2) + tilt1 * quat_grad(1) + tilt0 * quat_grad(0);
                tilt1b = c_half_psi * quat_grad(1) - s_half_psi * quat_grad(2);
                tilt_denb = z1 * tilt1b / (tilt_den * tilt_den) - z0 * tilt2b / (tilt_den * tilt_den) + 0.5 * tilt0b;
            }
            else
            {
                tilt0b = 0.0;
                s_half_psib = 0.0;
                tilt2b = 0.0;
                c_half_psib = 0.0;
                tilt1b = 0.0;
                tilt_denb = 0.0;
            }

            if (is_omg_grad_not_zero)
            {
                omg_termb = -((z0 * c_psi + z1 * s_psi) * omg_grad(1)) - (z0 * s_psi - z1 * c_psi) * omg_grad(0);
                tempb = omg_grad(2) / omg_den;
                dpsi_total_grad = omg_grad(2);
                z1b = dz0 * tempb;
                dz0b = z1 * tempb + c_psi * omg_grad(1) + s_psi * omg_grad(0);
                z0b = -(dz1 * tempb);
                dz1b = s_psi * omg_grad(1) - z0 * tempb - c_psi * omg_grad(0);
                omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) - dz2 * omg_termb / (omg_den * omg_den); // 1 + z2
                tempb = -(omg_term * omg_grad(1));
                c_psib = dz0 * omg_grad(1) + z0 * tempb;
                s_psib = dz1 * omg_grad(1) + z1 * tempb;
                z0b = z0b + c_psi * tempb;
                z1b = z1b + s_psi * tempb;
                tempb = -(omg_term * omg_grad(0));
                s_psib = s_psib + dz0 * omg_grad(0) + z0 * tempb;
                c_psib = c_psib - dz1 * omg_grad(0) - z1 * tempb;
                z0b = z0b + s_psi * tempb;
                z1b = z1b - c_psi * tempb;
                dz2b = omg_termb / omg_den;
                z2b = omg_denb;
            }
            else
            {
                omg_termb = 0.0;
                dpsi_total_grad = 0.0;
                z1b = 0.0;
                dz0b = 0.0;
                z0b = 0.0;
                dz1b = 0.0;
                omg_denb = 0.0;
                c_psib = 0.0;
                s_psib = 0.0;
                dz2b = 0.0;
                z2b = 0.0;
            }

            z0b = z0b + tilt2b / tilt_den + f_term0 * thr_grad;
            z1b = z1b + f_term1 * thr_grad - tilt1b / tilt_den;
            z2b = z2b + tilt_denb / tilt_den + f_term2 * thr_grad;
            psi_total_grad = c_psi * s_psib + 0.5 * c_half_psi * s_half_psib - s_psi * c_psib - 0.5 * s_half_psi * c_half_psib;

            if (is_omg_grad_not_zero)
            {
                ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
                dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
                ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
                dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
                ng22b = dz_term2 * dz2b;
                dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
                ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
                ng11b = dz_term1 * dz1b;
                ng00b = dz_term0 * dz0b;

                tempb = ng22b / ng_den;
                zu_sqr0b = tempb;
                zu_sqr1b = tempb;
                ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
                zu12b = -(ng12b / ng_den);
                tempb = ng11b / ng_den;
                ng_denb = ng_denb + zu12 * ng12b / (ng_den * ng_den) - (zu_sqr0 + zu_sqr2) * tempb / ng_den;
                zu_sqr0b = zu_sqr0b + tempb;
                zu_sqr2b = tempb;
                zu02b = -(ng02b / ng_den);
                zu01b = -(ng01b / ng_den);
                tempb = ng00b / ng_den;
                ng_denb = ng_denb + zu02 * ng02b / (ng_den * ng_den) + zu01 * ng01b / (ng_den * ng_den) - (zu_sqr1 + zu_sqr2) * tempb / ng_den;
                zu_normb = zu_sqr_norm * ng_denb - (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
                zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
                zu_sqr1b = zu_sqr1b + tempb + zu_sqr_normb;
                zu_sqr2b = zu_sqr2b + tempb + zu_sqr_normb;
                zu_sqr0b = zu_sqr0b + zu_sqr_normb;
                zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
                zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
                zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;
            }
            else
            {
                dz_term0b = 0.0;
                dz_term1b = 0.0;
                dz_term2b = 0.0;
                zu_normb = - (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
                zu_sqr_normb = zu_normb / (2.0 * zu_norm);
                zu_sqr1b = zu_sqr_normb;
                zu_sqr2b = zu_sqr_normb;
                zu_sqr0b = zu_sqr_normb;
                zu2b = z2b / zu_norm + 2 * zu2 * zu_sqr2b;
                zu1b = z1b / zu_norm + 2 * zu1 * zu_sqr1b;
                zu0b = z0b / zu_norm + 2 * zu0 * zu_sqr0b;
            }

            dFq2b = dz_term2b;
            dFq1b = dz_term1b;
            dFq0b = dz_term0b;
            f_term0b = z0 * thr_grad;
            f_term1b = z1 * thr_grad;
            f_term2b = z2 * thr_grad;
            j2b = dz_term2b;
            j1b = dz_term1b;
            j0b = dz_term0b;
            // cout << "jb\t"<< j2b <<"\t" <<  j1b << "\t" <<  j0b<< endl;
            jer_total_grad(2) += j2b;
            jer_total_grad(1) += j1b;
            jer_total_grad(0) += j0b;
            d3q_total_grad(2) = len * j2b;
            d3q_total_grad(1) = len * j1b;
            d3q_total_grad(0) = len * j0b;
            a2b = mass * f_term2b + zu2b;
            a1b = mass * f_term1b + zu1b;
            a0b = mass * f_term0b + zu0b;
            acc_total_grad(2) += a2b;
            acc_total_grad(1) += a1b;
            acc_total_grad(0) += a0b;
            d2q_total_grad(2) = len * a2b;
            d2q_total_grad(1) = len * a1b;
            d2q_total_grad(0) = len * a0b;
            dq_total_grad(2) = dq_grad(2) + massFrac * F_ * dFq2b;
            dq_total_grad(1) = dq_grad(1) + massFrac * F_ * dFq1b;
            dq_total_grad(0) = dq_grad(0) + massFrac * F_ * dFq0b;
            dF_total_grad = massFrac * (q2 * dFq2b + q1 * dFq1b + q0 * dFq0b);
            q_total_grad(2) = q_grad(2) + F_ * f_term2b + massFrac * (dF_ * dFq2b + F_ * zu2b);
            q_total_grad(1) = q_grad(1) + F_ * f_term1b + massFrac * (dF_ * dFq1b + F_ * zu1b);
            q_total_grad(0) = q_grad(0) + F_ * f_term0b + massFrac * (dF_ * dFq0b + F_ * zu0b);
            F_total_grad = F_grad + q2 * f_term2b + q1 * f_term1b + q0 * f_term0b + massFrac * (dq2 * dFq2b + dq1 * dFq1b + dq0 * dFq0b) + massFrac * (q2 * zu2b + q1 * zu1b + q0 * zu0b);

            return;
        }

        // inline void backward(const Eigen::Vector3d &q_grad,
        //                      const Eigen::Vector3d &dq_grad,
        //                      const double &F_grad,
        //                      const double &thr_grad,
        //                      const Eigen::Vector4d &quat_grad,
        //                      const Eigen::Vector3d &omg_grad,
        //                      Eigen::Vector3d &q_total_grad,
        //                      Eigen::Vector3d &dq_total_grad,
        //                      Eigen::Vector3d &d2q_total_grad,
        //                      Eigen::Vector3d &d3q_total_grad,
        //                      double &F_total_grad,
        //                      double &dF_total_grad,
        //                      Eigen::Vector3d &acc_total_grad,
        //                      Eigen::Vector3d &jer_total_grad,
        //                      double &psi_total_grad,
        //                      double &dpsi_total_grad) const
        // {
        //     double tempb;
        //     double a0b, a1b, a2b, j0b, j1b, j2b, q0b, q1b, q2b, dq0b, dq1b, dq2b;
        //     double dFq0b, dFq1b, dFq2b;
        //     double z0b, z1b, z2b, dz0b, dz1b, dz2b;
        //     double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
        //     double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
        //     double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
        //     double dz_term0b, dz_term1b, dz_term2b, f_term0b, f_term1b, f_term2b;
        //     double tilt_denb, tilt0b, tilt1b, tilt2b, c_half_psib, s_half_psib;
        //     double c_psib, s_psib, omg_denb, omg_termb;

        //     tilt0b = s_half_psi * quat_grad(3) + c_half_psi * quat_grad(0);
        //     s_half_psib = tilt0 * quat_grad(3) + tilt2 * quat_grad(1) - tilt1 * quat_grad(2);
        //     tilt2b = c_half_psi * quat_grad(2) + s_half_psi * quat_grad(1);
        //     c_half_psib = tilt2 * quat_grad(2) + tilt1 * quat_grad(1) + tilt0 * quat_grad(0);
        //     tilt1b = c_half_psi * quat_grad(1) - s_half_psi * quat_grad(2);
        //     tilt_denb = z1 * tilt1b / (tilt_den * tilt_den) - z0 * tilt2b / (tilt_den * tilt_den) + 0.5 * tilt0b;
        //     omg_termb = -((z0 * c_psi + z1 * s_psi) * omg_grad(1)) - (z0 * s_psi - z1 * c_psi) * omg_grad(0);
        //     tempb = omg_grad(2) / omg_den;
        //     // cout << "tempb\t" << omg_grad(2) << endl;
        //     dpsi_total_grad = omg_grad(2);
        //     z1b = dz0 * tempb;
        //     dz0b = z1 * tempb + c_psi * omg_grad(1) + s_psi * omg_grad(0);
        //     // cout << "dz0b\t"<< z1 <<"\t" << tempb << "\t" << c_psi << "\t" << omg_grad(1) << "\t" << s_psi << "\t" << omg_grad(0) << endl;
        //     z0b = -(dz1 * tempb);
        //     dz1b = s_psi * omg_grad(1) - z0 * tempb - c_psi * omg_grad(0);
        //     // cout << "dz1b\t"<< s_psi <<"\t" << omg_grad(1) << "\t" << z0 << "\t" << tempb << "\t" << c_psi << "\t" << omg_grad(0) << endl;
        //     omg_denb = -((z1 * dz0 - z0 * dz1) * tempb / omg_den) - dz2 * omg_termb / (omg_den * omg_den);
        //     tempb = -(omg_term * omg_grad(1));
        //     c_psib = dz0 * omg_grad(1) + z0 * tempb;
        //     s_psib = dz1 * omg_grad(1) + z1 * tempb;
        //     z0b = z0b + c_psi * tempb;
        //     z1b = z1b + s_psi * tempb;
        //     tempb = -(omg_term * omg_grad(0));
        //     s_psib = s_psib + dz0 * omg_grad(0) + z0 * tempb;
        //     c_psib = c_psib - dz1 * omg_grad(0) - z1 * tempb;
        //     z0b = z0b + s_psi * tempb + tilt2b / tilt_den + f_term0 * thr_grad;
        //     z1b = z1b + f_term1 * thr_grad - c_psi * tempb - tilt1b / tilt_den;
        //     dz2b = omg_termb / omg_den;
        //     // cout << "dz2b\t"<< omg_termb <<"\t" << omg_den  << endl;
        //     z2b = omg_denb + tilt_denb / sqrt(2.0 * (z2 + 1.0)) + f_term2 * thr_grad;
        //     psi_total_grad = c_psi * s_psib + 0.5 * c_half_psi * s_half_psib - s_psi * c_psib - 0.5 * s_half_psi * c_half_psib;
        //     f_term0b = z0 * thr_grad;
        //     f_term1b = z1 * thr_grad;
        //     f_term2b = z2 * thr_grad;
        //     ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
        //     dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
        //     // cout << "dz_term0b\t"<< ng02 <<"\t" << dz2b << "\t" << ng01 << "\t" << dz1b << "\t" << ng00 << "\t" << dz0b << endl;
        //     ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
        //     dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
        //     // cout << "dz_term1b\t"<< ng12 <<"\t" << dz2b << "\t" << ng11 << "\t" << dz1b << "\t" << ng01 << "\t" << dz0b << endl;
        //     ng22b = dz_term2 * dz2b;
        //     dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
        //     ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
        //     ng11b = dz_term1 * dz1b;
        //     ng00b = dz_term0 * dz0b;
        //     dFq2b = dz_term2b;
        //     dFq1b = dz_term1b;
        //     dFq0b = dz_term0b;

        //     tempb = ng22b / ng_den;
        //     zu_sqr0b = tempb;
        //     zu_sqr1b = tempb;
        //     ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
        //     zu12b = -(ng12b / ng_den);
        //     tempb = ng11b / ng_den;
        //     ng_denb = ng_denb + zu12 * ng12b / (ng_den * ng_den) - (zu_sqr0 + zu_sqr2) * tempb / ng_den;
        //     zu_sqr0b = zu_sqr0b + tempb;
        //     zu_sqr2b = tempb;
        //     zu02b = -(ng02b / ng_den);
        //     zu01b = -(ng01b / ng_den);
        //     tempb = ng00b / ng_den;
        //     ng_denb = ng_denb + zu02 * ng02b / (ng_den * ng_den) + zu01 * ng01b / (ng_den * ng_den) - (zu_sqr1 + zu_sqr2) * tempb / ng_den;
        //     zu_normb = zu_sqr_norm * ng_denb - (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
        //     zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
        //     zu_sqr1b = zu_sqr1b + tempb + zu_sqr_normb;
        //     zu_sqr2b = zu_sqr2b + tempb + zu_sqr_normb;
        //     zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
        //     zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
        //     zu_sqr0b = zu_sqr0b + zu_sqr_normb;
        //     zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;

        //     j2b = dz_term2b;
        //     j1b = dz_term1b;
        //     j0b = dz_term0b;
        //     // cout << "jb\t"<< j2b <<"\t" <<  j1b << "\t" <<  j0b<< endl;
        //     jer_total_grad(2) += j2b;
        //     jer_total_grad(1) += j1b;
        //     jer_total_grad(0) += j0b;
        //     d3q_total_grad(2) = len * j2b;
        //     d3q_total_grad(1) = len * j1b;
        //     d3q_total_grad(0) = len * j0b;
        //     a2b = mass * f_term2b + zu2b;
        //     a1b = mass * f_term1b + zu1b;
        //     a0b = mass * f_term0b + zu0b;
        //     acc_total_grad(2) += a2b;
        //     acc_total_grad(1) += a1b;
        //     acc_total_grad(0) += a0b;
        //     d2q_total_grad(2) = len * a2b;
        //     d2q_total_grad(1) = len * a1b;
        //     d2q_total_grad(0) = len * a0b;
        //     dq_total_grad(2) = dq_grad(2) + massFrac * F_ * dFq2b;
        //     dq_total_grad(1) = dq_grad(1) + massFrac * F_ * dFq1b;
        //     dq_total_grad(0) = dq_grad(0) + massFrac * F_ * dFq0b;
        //     dF_total_grad = massFrac * (q2 * dFq2b + q1 * dFq1b + q0 * dFq0b);
        //     q_total_grad(2) = q_grad(2) + F_ * f_term2b + massFrac * (dF_ * dFq2b + F_ * zu2b);
        //     q_total_grad(1) = q_grad(1) + F_ * f_term1b + massFrac * (dF_ * dFq1b + F_ * zu1b);
        //     q_total_grad(0) = q_grad(0) + F_ * f_term0b + massFrac * (dF_ * dFq0b + F_ * zu0b);
        //     F_total_grad = F_grad + q2 * f_term2b + q1 * f_term1b + q0 * f_term0b + massFrac * (dq2 * dFq2b + dq1 * dFq1b + dq0 * dFq0b) + massFrac * (q2 * zu2b + q1 * zu1b + q0 * zu0b);

        //     return;
        // }

    private:
        double mass, massFrac, grav, len;

        double a0, a1, a2, q0, q1, q2, dq0, dq1, dq2;
        double z0, z1, z2, dz0, dz1, dz2;
        double F_, dF_;
        double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
        double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
        double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
        double dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
        double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
        double c_psi, s_psi, omg_den, omg_term;
    };
}

#endif
