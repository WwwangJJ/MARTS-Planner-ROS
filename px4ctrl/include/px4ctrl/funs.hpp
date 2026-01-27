#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
using namespace std;

inline Eigen::Vector2d getcs(const double &ang)
{
  Eigen::Vector2d cs;
  cs(0) = cos(ang);
  cs(1) = sin(ang);
  return cs;
}

inline Eigen::Vector3d getQ(const Eigen::Vector2d &cst,
                            const Eigen::Vector2d &csp,
                            Eigen::Vector4d &g)
{
  Eigen::Vector3d q;
  g(0) = cst(1) * csp(0);
  g(1) = cst(1) * csp(1);
  g(2) = cst(0) * csp(0);
  g(3) = cst(0) * csp(1);
  q(0) = g(0);
  q(1) = g(1);
  q(2) = cst(0);
  return q;
}

inline Eigen::Vector3d getdQ(const Eigen::Vector2d &dAng, const Eigen::Vector2d &cst, const Eigen::Vector4d &gs)
{
  double g1, g2, g3, g4;
  Eigen::Vector3d dq;
  Eigen::Matrix<double, 2, 1> base;
  Eigen::Matrix<double, 3, 2> cof;
  g1 = gs(0);
  g2 = gs(1);
  g3 = gs(2);
  g4 = gs(3);

  base << dAng(0), dAng(1);
  cof.row(0) << g3, -g2;
  cof.row(1) << g4, g1;
  cof.row(2) << -cst(1), 0.0;
  dq = (cof * base).col(0);
  return dq;
}

inline Eigen::Vector3d getd2Q(const Eigen::Vector2d &dAng,
                              const Eigen::Vector2d &d2Ang,
                              const Eigen::Vector2d &cst,
                              const Eigen::Vector4d &gs)
{
  double g1, g2, g3, g4;
  Eigen::Vector3d d2q;
  Eigen::Matrix<double, 5, 1> base;
  Eigen::Matrix<double, 3, 5> cof;
  g1 = gs(0);
  g2 = gs(1);
  g3 = gs(2);
  g4 = gs(3);
  // cout << "row0\t"<< cof.row(0)(0) << "\t"<< cof.row(0)(1) << "\t"<< cof.row(0)(2) << "\t"<< cof.row(0)(3) << "\t"<< cof.row(0)(4) << "\t"<< endl;
  // cout << "row1\t"<< cof.row(1)(0) << "\t"<< cof.row(1)(1) << "\t"<< cof.row(1)(2) << "\t"<< cof.row(1)(3) << "\t"<< cof.row(1)(4) << "\t"<< endl;
  // cout << "row2\t"<< cof.row(2)(0) << "\t"<< cof.row(2)(1) << "\t"<< cof.row(2)(2) << "\t"<< cof.row(2)(3) << "\t"<< cof.row(2)(4) << "\t"<< endl;
  base << dAng(0) * dAng(0), dAng(0) * dAng(1), d2Ang(0), dAng(1) * dAng(1), d2Ang(1);
  cof.row(0) << -g1, -2 * g4, g3, -g1, -g2;
  cof.row(1) << -g2, 2 * g3, g4, -g2, g1;
  cof.row(2) << -cst(0), 0.0, -cst(1), 0.0, 0.0;
  d2q = (cof * base).col(0);
  // cout << "row1_\t"<< cof.row(1)(0) << "\t"<< cof.row(1)(1) << "\t"<< cof.row(1)(2) << "\t"<< cof.row(1)(3) << "\t"<< cof.row(1)(4) << "\t"<< endl;
  // cout << "row2_\t"<< cof.row(2)(0) << "\t"<< cof.row(2)(1) << "\t"<< cof.row(2)(2) << "\t"<< cof.row(2)(3) << "\t"<< cof.row(2)(4) << "\t"<< endl;
  // cout << "base\t"<< base(0) << "\t"<< base(1) << "\t"<< base(2) << "\t"<< base(3) << "\t"<< base(4) << "\t"<< endl;
  // cout << "d2q\t"<< d2q(0) << "\t"<< d2q(1) << "\t"<< d2q(2) << "\t"<< endl;
  return d2q;
}

inline Eigen::Vector3d getd3Q(const Eigen::Vector2d &dAng,
                              const Eigen::Vector2d &d2Ang,
                              const Eigen::Vector2d &d3Ang,
                              const Eigen::Vector2d &cst,
                              const Eigen::Vector4d &gs)
{
  double g1, g2, g3, g4;
  Eigen::Vector3d d3q;
  Eigen::Matrix<double, 10, 1> base;
  Eigen::Matrix<double, 3, 10> cof;
  g1 = gs(0);
  g2 = gs(1);
  g3 = gs(2);
  g4 = gs(3);
  base << pow(dAng(0), 3), pow(dAng(0), 2) * dAng(1), dAng(0) * d2Ang(0), dAng(0) * pow(dAng(1), 2), d2Ang(0) * dAng(1),
      dAng(0) * d2Ang(1), d3Ang(0), pow(dAng(1), 3), dAng(1) * d2Ang(1), d3Ang(1);
  cof.row(0) << -g3, 3 * g2, -3 * g1, -3 * g3, -3 * g4, -3 * g4, g3, g2, -3 * g1, -g2;
  cof.row(1) << -g4, -3 * g1, -3 * g2, -3 * g4, 3 * g3, 3 * g3, g4, -g1, -3 * g2, g1;
  ;
  cof.row(2) << cst(1), 0.0, -3 * cst(0), 0.0, 0.0, 0.0, -cst(1), 0.0, 0.0, 0.0;
  d3q = (cof * base).col(0);
  return d3q;
}

inline Eigen::Vector3d getd4Q(const Eigen::Vector2d &dAng,
                              const Eigen::Vector2d &d2Ang,
                              const Eigen::Vector2d &d3Ang,
                              const Eigen::Vector2d &d4Ang,
                              const Eigen::Vector2d &cst,
                              const Eigen::Vector2d &csp,
                              const Eigen::Vector4d &gs)
{
  double g1, g2, g3, g4;
  Eigen::Vector3d d4q;
  g1 = gs(0);
  g2 = gs(1);
  g3 = gs(2);
  g4 = gs(3);
  d4q(0) = 6 * (-cst(1) * pow(dAng(0), 2) + cst(0) * d2Ang(0)) * (-csp(0) * pow(dAng(1), 2) - csp(1) * d2Ang(1)) -
           4 * csp(1) * dAng(1) * (-cst(0) * pow(dAng(0), 3) - 3 * cst(1) * dAng(0) * d2Ang(0) + cst(0) * d3Ang(0)) +
           4 * cst(0) * dAng(0) * (csp(1) * pow(dAng(1), 3) - 3 * csp(0) * dAng(1) * d2Ang(1) - csp(1) * d3Ang(1)) +
           csp(0) * (cst(1) * pow(dAng(0), 4) - 6 * cst(0) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(1) * pow(d2Ang(0), 2) - 4 * cst(1) * dAng(0) * d3Ang(0) + cst(0) * d4Ang(0)) +
           cst(1) * (csp(0) * pow(dAng(1), 4) + 6 * csp(1) * pow(dAng(1), 2) * d2Ang(1) - 3 * csp(0) * pow(d2Ang(1), 2) - 4 * csp(0) * dAng(1) * d3Ang(1) - csp(1) * d4Ang(1));
  d4q(1) = 6 * (-cst(1) * pow(dAng(0), 2) + cst(0) * d2Ang(0)) * (-csp(1) * pow(dAng(1), 2) + csp(0) * d2Ang(1)) +
           4 * csp(0) * dAng(1) * (-cst(0) * pow(dAng(0), 3) - 3 * cst(1) * dAng(0) * d2Ang(0) + cst(0) * d3Ang(0)) +
           4 * cst(0) * dAng(0) * (-csp(0) * pow(dAng(1), 3) - 3 * csp(1) * dAng(1) * d2Ang(1) + csp(0) * d3Ang(1)) +
           csp(1) * (cst(1) * pow(dAng(0), 4) - 6 * cst(0) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(1) * pow(d2Ang(0), 2) - 4 * cst(1) * dAng(0) * d3Ang(0) + cst(0) * d4Ang(0)) +
           cst(1) * (csp(1) * pow(dAng(1), 4) - 6 * csp(0) * pow(dAng(1), 2) * d2Ang(1) - 3 * csp(1) * pow(d2Ang(1), 2) - 4 * csp(1) * dAng(1) * d3Ang(1) + csp(0) * d4Ang(1));
  d4q(2) = cst(0) * pow(dAng(0), 4) + 6 * cst(1) * pow(dAng(0), 2) * d2Ang(0) - 3 * cst(0) * pow(d2Ang(0), 2) - 4 * cst(0) * dAng(0) * d3Ang(0) - cst(1) * d4Ang(0);

  return d4q;
}
