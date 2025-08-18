#include <math.h>

double mass, massFrac, grav;
double thr;
double v0, v1, v2, a0, a1, a2, j0, j1, j2, q0, q1, q2, dq0, dq1, dq2;
double dFq0, dFq1, dFq2, F, dF;
double psi, dpsi;
double quat0, quat1, quat2, quat3;
double omg0, omg1, omg2;
double z0, z1, z2, dz0, dz1, dz2;
double zu_sqr_norm, zu_norm, zu0, zu1, zu2;
double zu_sqr0, zu_sqr1, zu_sqr2, zu01, zu12, zu02;
double ng00, ng01, ng02, ng11, ng12, ng22, ng_den;
double dz_term0, dz_term1, dz_term2, f_term0, f_term1, f_term2;
double tilt_den, tilt0, tilt1, tilt2, c_half_psi, s_half_psi;
double c_psi, s_psi, omg_den, omg_term;

void forward( )
{
    zu0 = a0 + massFrac * F * q0;
    zu1 = a1 + massFrac * F * q1;
    zu2 = a2 + massFrac * F * q2 + grav;
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
    dFq0 = massFrac * (F * dq0 + dF * q0);
    dFq1 = massFrac * (F * dq1 + dF * q1);
    dFq2 = massFrac * (F * dq2 + dF * q2);
    dz_term0 = j0 + dFq0;
    dz_term1 = j1 + dFq1;
    dz_term2 = j2 + dFq2;
    dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
    dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
    dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
    f_term0 = mass * a0 + F * q0;
    f_term1 = mass * a1 + F * q1;
    f_term2 = mass * (a2 + grav) + F * q2;
    thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
    tilt_den = sqrt(2.0 * (1.0 + z2));
    tilt0 = 0.5 * tilt_den;
    tilt1 = -z1 / tilt_den;
    tilt2 = z0 / tilt_den;
    c_half_psi = cos(0.5 * psi);
    s_half_psi = sin(0.5 * psi);
    quat0 = tilt0 * c_half_psi;
    quat1 = tilt1 * c_half_psi + tilt2 * s_half_psi;
    quat2 = tilt2 * c_half_psi - tilt1 * s_half_psi;
    quat3 = tilt0 * s_half_psi;
    c_psi = cos(psi);
    s_psi = sin(psi);
    omg_den = z2 + 1.0;
    omg_term = dz2 / omg_den;
    omg0 = dz0 * s_psi - dz1 * c_psi -
             (z0 * s_psi - z1 * c_psi) * omg_term;
    omg1 = dz0 * c_psi + dz1 * s_psi -
             (z0 * c_psi + z1 * s_psi) * omg_term;
    omg2 = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

    return;
}