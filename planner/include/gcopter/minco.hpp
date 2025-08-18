/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef MINCO_HPP
#define MINCO_HPP

#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace minco
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
    };


    // MINCO for s=3 and non-uniform time
    class MINCO_S3NU
    {
    public:
        MINCO_S3NU() = default;
        ~MINCO_S3NU() { A.destroy(); }

    private:
        int N;
        int dN;
        Eigen::Vector3d contDescendDim, boundSlackDim;
        Eigen::Matrix3d headPVAL;
        Eigen::Matrix3d tailPVAL;
        Eigen::Matrix3Xd headPVAQ;
        Eigen::Matrix3Xd tailPVAQ;
        Eigen::VectorXi fixPtsNums;
        BandedSystem A;
        Eigen::MatrixXd b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

    public:
        inline void setConditions(const Eigen::Matrix3d &headState,
                                  const Eigen::Matrix3d &tailState,
                                  const Eigen::Matrix3Xd &headAngTs,
                                  const Eigen::Matrix3Xd &tailAngTs,    
                                  const Eigen::Vector3d &continuityDescendDim,
                                  const Eigen::Vector3d &boundarySlackDim,
                                  const Eigen::VectorXi &fixPointsNums,
                                  const int &pieceNum,
                                  const int &drone_num)
        {
            N = pieceNum;
            dN = drone_num;
            headPVAL = headState;
            tailPVAL = tailState;
            headPVAQ = headAngTs;
            tailPVAQ = tailAngTs;
            contDescendDim = continuityDescendDim;
            boundSlackDim = boundarySlackDim;
            fixPtsNums = fixPointsNums;
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 3 + 3 * dN);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            return;
        }

        inline void setParameters(const Eigen::MatrixX3d &inPs,
                                  const Eigen::VectorXd &angTsV,
                                  const Eigen::VectorXd &continuityDescendV,
                                  const Eigen::VectorXd &boundarySlackV,
                                  const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            b.setZero();

            Eigen::Map<const Eigen::MatrixXd> angTsM(angTsV.data(), N - 1, 3 * dN);
            Eigen::Map<const Eigen::MatrixXd> descendM(continuityDescendV.data(), N - 1, contDescendDim.sum() * dN);
            Eigen::Map<const Eigen::MatrixXd> slackM(boundarySlackV.data(), 2, boundSlackDim.sum() * dN);
            // cout << "angTsM======================================================="<< endl;
            // cout << angTsM << endl;
            // cout << "descendM======================================================="<< endl;
            // cout << descendM << endl;
            // cout << "slackM======================================================="<< endl; 
            // cout << slackM << endl;


            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            b.row(0).head(3) = headPVAL.col(0).transpose();
            b.row(1).head(3) = headPVAL.col(1).transpose();
            b.row(2).head(3) = headPVAL.col(2).transpose();
            // cout << headPVAL << endl;
            // cout << headPVAQ /M_PI << endl;
            // cout << tailPVAL << endl;
            // cout << tailPVAQ /M_PI << endl;
            // cout << angTsM /M_PI << endl;

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                if(contDescendDim(0) == 1 && contDescendDim(1) == 1 && contDescendDim(2) == 3)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       b.row(6 * i + 4).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 5);  //sss(tpT)
                       b.row(6 * i + 3)(3 + j * 3 + 2) = descendM.row(i)(j * 5 + 3);                //j(T)
                       b.row(6 * i + 8)(3 + j * 3 + 2) = descendM.row(i)(j * 5 + 4);                //a(T)
                    }
                    
                }
                else if(contDescendDim(0) == 2 && contDescendDim(1) == 2 && contDescendDim(2) == 4)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       b.row(6 * i + 4).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 8);    //sss(tpT)
                       b.row(6 * i + 3).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 8 + 3);//jjj(tpT)   
                       b.row(6 * i + 8)(3 + j * 3 + 2) = descendM.row(i)(j * 8 + 6);                  //a(T) 
                       b.row(6 * i + 7)(3 + j * 3 + 2) = descendM.row(i)(j * 8 + 7);                  //v(T)
                    }
                }

                b.row(6 * i + 5).segment<3>(0) = inPs.row(i);
                b.row(6 * i + 5).segment(3, 3 * dN) = angTsM.row(i);
                // cout << angTsM.row(i)(0) * 180 / M_PI << "\t" << angTsM.row(i)(1) * 180 / M_PI << "\t" <<angTsM.row(i)(2) << "\t" << angTsM.row(i)(3) * 180 / M_PI << "\t" << angTsM.row(i)(4) * 180 / M_PI << "\t" << angTsM.row(i)(5) << "\t" << angTsM.row(i)(6) * 180 / M_PI << "\t"<< angTsM.row(i)(7) * 180 / M_PI << "\t" << angTsM.row(i)(8) <<endl;
                // if (abs(angTsM.row(i)(1) * 180 / M_PI) > 60.0 || abs(angTsM.row(i)(4) * 180 / M_PI) > 60.0 || abs(angTsM.row(i)(7) * 180 / M_PI) > 60.0)
                // {
                //     ROS_ERROR("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
                // }
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            b.row(6 * N - 3).head(3) = tailPVAL.col(0).transpose();
            b.row(6 * N - 2).head(3) = tailPVAL.col(1).transpose();
            b.row(6 * N - 1).head(3) = tailPVAL.col(2).transpose();

            if (boundSlackDim(0) == 1 && boundSlackDim(1) == 1 && boundSlackDim(2) == 1)
            {
                for (int j = 0; j < dN; j++)
                {
                    headPVAQ.row(2).segment<3>(j * 3) = slackM.row(0).segment<3>(j * 3);
                    tailPVAQ.row(2).segment<3>(j * 3) = slackM.row(1).segment<3>(j * 3);
                }
            }
            else if (boundSlackDim(0) == 0 && boundSlackDim(1) == 0 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    headPVAQ.row(1)(j * 3 + 2) = slackM.row(0)(j * 2 + 1);
                    headPVAQ.row(2)(j * 3 + 2) = slackM.row(0)(j * 2);
                    tailPVAQ.row(1)(j * 3 + 2) = slackM.row(1)(j * 2 + 1);
                    tailPVAQ.row(2)(j * 3 + 2) = slackM.row(1)(j * 2);
                }
            }
            else if (boundSlackDim(0) == 1 && boundSlackDim(1) == 1 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    headPVAQ.row(1)(j * 3 + 2) = slackM.row(0)(j * 4 + 3);
                    headPVAQ.row(2).segment<3>(j * 3) = slackM.row(0).segment<3>(j * 4);
                    tailPVAQ.row(1)(j * 3 + 2) = slackM.row(1)(j * 4 + 3);
                    tailPVAQ.row(2).segment<3>(j * 3) = slackM.row(1).segment<3>(j * 4);
                }
            }
            else if (boundSlackDim(0) == 2 && boundSlackDim(1) == 2 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    headPVAQ.row(1).segment<3>(j * 3) = slackM.row(0).segment<3>(j * 6 + 3);   // vvv
                    headPVAQ.row(2).segment<3>(j * 3) = slackM.row(0).segment<3>(j * 6);       // aaa
                    tailPVAQ.row(1).segment<3>(j * 3) = slackM.row(1).segment<3>(j * 6 + 3);   // vvv
                    tailPVAQ.row(2).segment<3>(j * 3) = slackM.row(1).segment<3>(j * 6);       // aaa
                }
            }

            // for (int i = 0; i < fixPtsNums.size(); i++)
            // {
            //     cout << i << "\t" << inPs.row(fixPtsNums(i))(0) << "\t"<< inPs.row(fixPtsNums(i))(1) << "\t"<< inPs.row(fixPtsNums(i))(2) << "\t"<< endl;;
            // }

            b.row(0).segment(3, 3 * dN) = headPVAQ.row(0);
            b.row(1).segment(3, 3 * dN) = headPVAQ.row(1);
            b.row(2).segment(3, 3 * dN) = headPVAQ.row(2);

            b.row(6 * N - 3).segment(3, 3 * dN) = tailPVAQ.row(0);
            b.row(6 * N - 2).segment(3, 3 * dN) = tailPVAQ.row(1);
            b.row(6 * N - 1).segment(3, 3 * dN) = tailPVAQ.row(2);
            // cout <<b /M_PI <<endl;
            A.factorizeLU();
            A.solve(b);

            return;
        }

        inline void getTrajectory(Trajectory<5> &traj, const int &k) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 3 * k).transpose().rowwise().reverse());
            }
            return;
        }

        inline void getTrajs(std::vector<Trajectory<5>> &trajs) const
        {
            trajs.clear();
            trajs.reserve(dN + 1);
            for (int k = 0; k < dN + 1; k++)
            {
                for (int i = 0; i < N; i++)
                {
                    trajs[k].emplace_back(T1(i), b.block<6, 3>(6 * i, 3 * k).transpose().rowwise().reverse());
                }
            }
            return;
        }

        inline void getEnergy(double &energy) const // just load energy
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * b.row(6 * i + 3).head(3).squaredNorm() * T1(i) +
                          144.0 * b.row(6 * i + 4).head(3).dot(b.row(6 * i + 3).head(3)) * T2(i) +
                          192.0 * b.row(6 * i + 4).head(3).squaredNorm() * T3(i) +
                          240.0 * b.row(6 * i + 5).head(3).dot(b.row(6 * i + 3).head(3)) * T3(i) +
                          720.0 * b.row(6 * i + 5).head(3).dot(b.row(6 * i + 4).head(3)) * T4(i) +
                          720.0 * b.row(6 * i + 5).head(3).squaredNorm() * T5(i);
            }
            return;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            // Eigen::Map<const Eigen::VectorXd> bv(b.data(), (6 * N) * (3 + 3 * dN));
            // cout<< "b" << "\n" << b << endl;
            return b;
        }

        inline void getEnergyPartialGradByCoeffs(Eigen::VectorXd &gdCv) const
        {
            gdCv.resize((6 * N) * (3 + 3 * dN));
            Eigen::Map<Eigen::MatrixXd> gdC(gdCv.data(), 6 * N, 3 + 3 * dN);
            gdC.setZero();
            // cout << "gdC\t" << "\n"<<  gdC << endl;
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5).head(3) = 240.0 * b.row(6 * i + 3).head(3) * T3(i) +
                                             720.0 * b.row(6 * i + 4).head(3) * T4(i) +
                                             1440.0 * b.row(6 * i + 5).head(3) * T5(i);
                gdC.row(6 * i + 4).head(3) = 144.0 * b.row(6 * i + 3).head(3) * T2(i) +
                                             384.0 * b.row(6 * i + 4).head(3) * T3(i) +
                                             720.0 * b.row(6 * i + 5).head(3) * T4(i);
                gdC.row(6 * i + 3).head(3) = 72.0 * b.row(6 * i + 3).head(3) * T1(i) +
                                             144.0 * b.row(6 * i + 4).head(3) * T2(i) +
                                             240.0 * b.row(6 * i + 5).head(3) * T3(i);
                gdC.block<3, 3>(6 * i, 0).setZero();
            }
            return;
        }

        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * b.row(6 * i + 3).head(3).squaredNorm() +
                      288.0 * b.row(6 * i + 4).head(3).dot(b.row(6 * i + 3).head(3)) * T1(i) +
                      576.0 * b.row(6 * i + 4).head(3).squaredNorm() * T2(i) +
                      720.0 * b.row(6 * i + 5).head(3).dot(b.row(6 * i + 3).head(3)) * T2(i) +
                      2880.0 * b.row(6 * i + 5).head(3).dot(b.row(6 * i + 4).head(3)) * T3(i) +
                      3600.0 * b.row(6 * i + 5).head(3).squaredNorm() * T4(i);
            }
            return;
        }

        inline void propogateGrad(const Eigen::VectorXd &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::MatrixX3d &gradByPoints,
                                  Eigen::VectorXd  &gradByAngTsV,
                                  Eigen::VectorXd &gradByTimes,
                                  Eigen::VectorXd &grad,
                                  const int &dimXiL,
                                  const int &dimXiQ,
                                  const int &dimXiContinuityDesDim)

        {
            gradByTimes.resize(N);
            gradByTimes.setZero();
            gradByPoints.resize(N - 1, 3);
            gradByAngTsV.resize((N - 1) * (3 * dN));
            Eigen::Map<Eigen::MatrixXd> gradByAngTsM(gradByAngTsV.data(), N - 1, 3 * dN);
            Eigen::Map<const Eigen::MatrixXd> adjGrad_(partialGradByCoeffs.data(), 6 * N, 3 + 3 * dN);
            Eigen::Map<Eigen::MatrixXd> gradByContinuityDescendM(grad.data() + dimXiL + dimXiQ, N - 1, contDescendDim.sum() * dN);
            Eigen::Map<Eigen::MatrixXd> gradByBoundarySlackM(grad.data() + dimXiL + dimXiQ + dimXiContinuityDesDim, 2, boundSlackDim.sum() * dN);

            // cout << "adjGrad_\n" << adjGrad_ << endl;
            Eigen::MatrixXd adjGrad = adjGrad_;
            A.solveAdj(adjGrad);
            // cout << "adjGrad\n" << adjGrad << endl;
            
            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.row(i) = adjGrad.row(6 * i + 5).segment<3>(0);
                gradByAngTsM.row(i) = adjGrad.row(6 * i + 5).segment(3, 3 * dN);
                if (contDescendDim(0) == 1 && contDescendDim(1) == 1 && contDescendDim(2) == 3)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       gradByContinuityDescendM.row(i).segment<3>(5 * j) = adjGrad.row(6 * i + 4).segment<3>(3 + j * 3);  //sss(tpT)
                       gradByContinuityDescendM.row(i)(j * 5 + 3) = adjGrad.row(6 * i + 3)(3 + j * 3 + 2);               //j(T)
                       gradByContinuityDescendM.row(i)(j * 5 + 4) = adjGrad.row(6 * i + 8)(3 + j * 3 + 2);                //a(T)
                    }
                }
                else if (contDescendDim(0) == 2 && contDescendDim(1) == 2 && contDescendDim(2) == 4)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       gradByContinuityDescendM.row(i).segment<3>(8 * j) = adjGrad.row(6 * i + 4).segment<3>(3 + j * 3);  //sss(tpT)
                       gradByContinuityDescendM.row(i).segment<3>(8 * j + 3) = adjGrad.row(6 * i + 3).segment<3>(3 + j * 3);  //sss(tpT)
                       gradByContinuityDescendM.row(i)(j * 8 + 6) = adjGrad.row(6 * i + 8)(3 + j * 3 + 2);               //j(T)
                       gradByContinuityDescendM.row(i)(j * 8 + 7) = adjGrad.row(6 * i + 7)(3 + j * 3 + 2);                //a(T)
                    }
                }

            }

            if (boundSlackDim(0) == 1 && boundSlackDim(1) == 1 && boundSlackDim(2) == 1)
            {
                for (int j = 0; j < dN; j++)
                {
                    gradByBoundarySlackM.row(0).segment<3>(3 * j) = adjGrad.row(2).segment<3>(3 + j * 3); // sss(tpT)
                    gradByBoundarySlackM.row(1).segment<3>(3 * j) = adjGrad.row(6 * N - 1).segment<3>(3 + j * 3);
                }
            }
            else if (boundSlackDim(0) == 0 && boundSlackDim(1) == 0 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    gradByBoundarySlackM.row(0)(2 * j + 1) = adjGrad.row(1)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(0)(2 * j) = adjGrad.row(2)(3 + j * 3 + 2); // sss(tpT)
                    gradByBoundarySlackM.row(1)(2 * j + 1) = adjGrad.row(6 * N - 2)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(1)(2 * j) = adjGrad.row(6 * N - 1)(3 + j * 3 + 2); // sss(tpT)
                }
            }
            else if (boundSlackDim(0) == 1 && boundSlackDim(1) == 1 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    gradByBoundarySlackM.row(0)(4 * j + 3) = adjGrad.row(1)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(0).segment<3>(4 * j) = adjGrad.row(2).segment<3>(3 + j * 3); // sss(tpT)
                    gradByBoundarySlackM.row(1)(4 * j + 3) = adjGrad.row(6 * N - 2)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(1).segment<3>(4 * j) = adjGrad.row(6 * N - 1).segment<3>(3 + j * 3); // sss(tpT)
                }
            }
            else if (boundSlackDim(0) == 2 && boundSlackDim(1) == 2 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    gradByBoundarySlackM.row(0).segment<3>(6 * j + 3) = adjGrad.row(1).segment<3>(3 + j * 3);
                    gradByBoundarySlackM.row(0).segment<3>(6 * j) = adjGrad.row(2).segment<3>(3 + j * 3); // sss(tpT)
                    gradByBoundarySlackM.row(1).segment<3>(6 * j + 3) = adjGrad.row(6 * N - 2).segment<3>(3 + j * 3);
                    gradByBoundarySlackM.row(1).segment<3>(6 * j) = adjGrad.row(6 * N - 1).segment<3>(3 + j * 3); // sss(tpT)
                }
            }

            for (int i = 0; i < fixPtsNums.size(); i++)
            {
                gradByPoints.row(fixPtsNums(i)) = Eigen::Vector3d::Zero();
            }
            // cout <<gradByBoundarySlackM << endl;

            Eigen::MatrixXd B1;
            B1.resize(6, 3 + 3 * dN);
            Eigen::Matrix3Xd B2;
            B2.resize(3, 3 + 3 * dN);
            for (int i = 0; i < N - 1; i++)
            {
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1(i) * b.row(i * 6 + 2) +
                              3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) +
                              5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) +
                              20.0 * T3(i) * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1(i) * b.row(i * 6 + 4) +
                              60.0 * T2(i) * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1(i) * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);
                for(int j = 0; j < 6; j++)
                {
                    gradByTimes(i) += (B1.row(j) * adjGrad.row(6 * i + 3 + j).transpose()).sum(); 
                }
            }
            // cout << "grad time2\t" << gradByTimes << endl;

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1(N - 1) * b.row(6 * N - 4) +
                          3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) +
                          5.0 * T4(N - 1) * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) +
                          20.0 * T3(N - 1) * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1(N - 1) * b.row(6 * N - 2) +
                          60.0 * T2(N - 1) * b.row(6 * N - 1));

            for (int j = 0; j < 3; j++)
            {
                gradByTimes(N - 1) += (B2.row(j) * adjGrad.row(6 * N - 3 + j).transpose()).sum();
            }
            
            // cout << "grad time3\t" << gradByTimes << endl;
            
            gradByTimes += partialGradByTimes;
        }
    };

    class MINCO_S4NU
    {
    public:
        MINCO_S4NU() = default;
        ~MINCO_S4NU() { A.destroy(); }

    private:
        int N;
        int dN;
        double ratio;
        Eigen::Vector3d contDescendDim, boundSlackDim;
        Eigen::Matrix<double, 3, 4> headPVAL;
        Eigen::Matrix<double, 3, 4> tailPVAL;
        Eigen::Matrix4Xd headPVAQ;
        Eigen::Matrix4Xd tailPVAQ;
        Eigen::VectorXi fixPtsNums;
        BandedSystem A;
        Eigen::MatrixXd b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
        Eigen::VectorXd T6;
        Eigen::VectorXd T7;

    public:
        inline void setConditions(const Eigen::Matrix<double, 3, 4> &headState,
                                  const Eigen::Matrix<double, 3, 4> &tailState,
                                  const Eigen::Matrix4Xd &headAngTs,
                                  const Eigen::Matrix4Xd &tailAngTs,    
                                  const Eigen::Vector3d &continuityDescendDim,
                                  const Eigen::Vector3d &boundarySlackDim,
                                  const Eigen::VectorXi &fixPointsNums,
                                  const int &pieceNum,
                                  const int &drone_num,
                                  const double &rhoE)
        {
            ratio = rhoE / drone_num;
            N = pieceNum;
            dN = drone_num;
            headPVAL = headState;
            tailPVAL = tailState;
            headPVAQ = headAngTs;
            tailPVAQ = tailAngTs;
            contDescendDim = continuityDescendDim;
            boundSlackDim = boundarySlackDim;
            fixPtsNums = fixPointsNums;
            A.create(8 * N, 8, 8);
            b.resize(8 * N, 3 + 3 * dN);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);
            T6.resize(N);
            T7.resize(N);
            return;
        }

        inline void setParameters(const Eigen::MatrixX3d &inPs,
                                  const Eigen::VectorXd &angTsV,
                                  const Eigen::VectorXd &continuityDescendV,
                                  const Eigen::VectorXd &boundarySlackV,
                                  const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);
            T6 = T4.cwiseProduct(T2);
            T7 = T4.cwiseProduct(T3);

            A.reset();
            b.setZero();

            Eigen::Map<const Eigen::MatrixXd> angTsM(angTsV.data(), N - 1, 3 * dN);
            Eigen::Map<const Eigen::MatrixXd> descendM(continuityDescendV.data(), N - 1, contDescendDim.sum() * dN);
            Eigen::Map<const Eigen::MatrixXd> slackM(boundarySlackV.data(), 2, boundSlackDim.sum() * dN);
            // cout << "angTsM======================================================="<< endl;
            // cout << angTsM << endl;
            // cout << "descendM======================================================="<< endl;
            // cout << descendM << endl;
            // cout << "slackM======================================================="<< endl; 
            // cout << slackM << endl;


            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            A(3, 3) = 6.0;
            b.row(0).head(3) = headPVAL.col(0).transpose();
            b.row(1).head(3) = headPVAL.col(1).transpose();
            b.row(2).head(3) = headPVAL.col(2).transpose();
            b.row(3).head(3) = headPVAL.col(3).transpose();
            // cout << headPVAL << endl;
            // cout << headPVAQ /M_PI << endl;
            // cout << tailPVAL << endl;
            // cout << tailPVAQ /M_PI << endl;
            // cout << angTsM /M_PI << endl;

            for (int i = 0; i < N - 1; i++)
            {
                A(8 * i + 4, 8 * i + 4) = 24.0;                       // s(T) - s(0)               
                A(8 * i + 4, 8 * i + 5) = 120.0 * T1(i);
                A(8 * i + 4, 8 * i + 6) = 360.0 * T2(i);
                A(8 * i + 4, 8 * i + 7) = 840.0 * T3(i);
                A(8 * i + 4, 8 * i + 12) = -24.0;
                
                A(8 * i + 5, 8 * i + 5) = 120.0;                      //ds
                A(8 * i + 5, 8 * i + 6) = 720.0 * T1(i);
                A(8 * i + 5, 8 * i + 7) = 2520.0 * T2(i);
                A(8 * i + 5, 8 * i + 13) = -120.0;

                A(8 * i + 6, 8 * i + 6) = 720.0;                     //dds
                A(8 * i + 6, 8 * i + 7) = 5040.0 * T1(i);
                A(8 * i + 6, 8 * i + 14) = -720.0;

                A(8 * i + 7, 8 * i) = 1.0;                            //p(T)
                A(8 * i + 7, 8 * i + 1) = T1(i);
                A(8 * i + 7, 8 * i + 2) = T2(i);
                A(8 * i + 7, 8 * i + 3) = T3(i);
                A(8 * i + 7, 8 * i + 4) = T4(i);
                A(8 * i + 7, 8 * i + 5) = T5(i);
                A(8 * i + 7, 8 * i + 6) = T6(i);
                A(8 * i + 7, 8 * i + 7) = T7(i);                      

                A(8 * i + 8, 8 * i) = 1.0;                            //p(T) - p(0)
                A(8 * i + 8, 8 * i + 1) = T1(i);
                A(8 * i + 8, 8 * i + 2) = T2(i);
                A(8 * i + 8, 8 * i + 3) = T3(i);
                A(8 * i + 8, 8 * i + 4) = T4(i);
                A(8 * i + 8, 8 * i + 5) = T5(i);
                A(8 * i + 8, 8 * i + 6) = T6(i);
                A(8 * i + 8, 8 * i + 7) = T7(i);
                A(8 * i + 8, 8 * i + 8) = -1.0;   

                A(8 * i + 9, 8 * i + 1) = 1.0;                        //v(T) - v(0)
                A(8 * i + 9, 8 * i + 2) = 2.0 * T1(i);
                A(8 * i + 9, 8 * i + 3) = 3.0 * T2(i);
                A(8 * i + 9, 8 * i + 4) = 4.0 * T3(i);
                A(8 * i + 9, 8 * i + 5) = 5.0 * T4(i);
                A(8 * i + 9, 8 * i + 6) = 6.0 * T5(i);
                A(8 * i + 9, 8 * i + 7) = 7.0 * T6(i);
                A(8 * i + 9, 8 * i + 9) = -1.0;

                A(8 * i + 10, 8 * i + 2) = 2.0;                        //a(T) - a(0)
                A(8 * i + 10, 8 * i + 3) = 6.0 * T1(i);
                A(8 * i + 10, 8 * i + 4) = 12.0 * T2(i);
                A(8 * i + 10, 8 * i + 5) = 20.0 * T3(i);
                A(8 * i + 10, 8 * i + 6) = 30.0 * T4(i);
                A(8 * i + 10, 8 * i + 7) = 42.0 * T5(i);
                A(8 * i + 10, 8 * i + 10) = -2.0;

                A(8 * i + 11, 8 * i + 3) = 6.0;                        //j(T) - j(0)
                A(8 * i + 11, 8 * i + 4) = 24.0 * T1(i);
                A(8 * i + 11, 8 * i + 5) = 60.0 * T2(i);
                A(8 * i + 11, 8 * i + 6) = 120.0 * T3(i);
                A(8 * i + 11, 8 * i + 7) = 210.0 * T4(i);
                A(8 * i + 11, 8 * i + 11) = -6.0;
                    
                if(contDescendDim(0) == 3 && contDescendDim(1) == 3 && contDescendDim(2) == 5)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       b.row(8 * i + 6).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 11);     //dds(tpT) 
                       b.row(8 * i + 5).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 11 + 3); //ds(tpT)
                       b.row(8 * i + 4).segment<3>(3 + j * 3) = descendM.row(i).segment<3>(j * 11 + 6);  //s(tpT)
                       b.row(8 * i + 11)(3 + j * 3 + 2) = descendM.row(i)(j * 11 + 9);                //j(T)
                       b.row(8 * i + 10)(3 + j * 3 + 2) = descendM.row(i)(j * 11 + 10);                //a(T)
                    }
                    
                }

                b.row(8 * i + 7).segment<3>(0) = inPs.row(i);
                b.row(8 * i + 7).segment(3, 3 * dN) = angTsM.row(i);
                // cout << angTsM.row(i)(0) * 180 / M_PI <<endl;
                // cout << angTsM.row(i)(0) * 180 / M_PI << "\t" << angTsM.row(i)(1) * 180 / M_PI << "\t" <<angTsM.row(i)(2) << "\t" << angTsM.row(i)(3) * 180 / M_PI << "\t" << angTsM.row(i)(4) * 180 / M_PI << "\t" << angTsM.row(i)(5) << "\t" << angTsM.row(i)(6) * 180 / M_PI << "\t"<< angTsM.row(i)(7) * 180 / M_PI << "\t" << angTsM.row(i)(8) <<endl;
                // if (abs(angTsM.row(i)(1) * 180 / M_PI) > 60.0 || abs(angTsM.row(i)(4) * 180 / M_PI) > 60.0 || abs(angTsM.row(i)(7) * 180 / M_PI) > 60.0)
                // {
                //     ROS_ERROR("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
                // }
            }

            A(8 * N - 4, 8 * N - 8) = 1.0;
            A(8 * N - 4, 8 * N - 7) = T1(N - 1);
            A(8 * N - 4, 8 * N - 6) = T2(N - 1);
            A(8 * N - 4, 8 * N - 5) = T3(N - 1);
            A(8 * N - 4, 8 * N - 4) = T4(N - 1);
            A(8 * N - 4, 8 * N - 3) = T5(N - 1);
            A(8 * N - 4, 8 * N - 2) = T6(N - 1);
            A(8 * N - 4, 8 * N - 1) = T7(N - 1);
            A(8 * N - 3, 8 * N - 7) = 1.0;
            A(8 * N - 3, 8 * N - 6) = 2.0 * T1(N - 1);
            A(8 * N - 3, 8 * N - 5) = 3.0 * T2(N - 1);
            A(8 * N - 3, 8 * N - 4) = 4.0 * T3(N - 1);
            A(8 * N - 3, 8 * N - 3) = 5.0 * T4(N - 1);
            A(8 * N - 3, 8 * N - 2) = 6.0 * T5(N - 1);
            A(8 * N - 3, 8 * N - 1) = 7.0 * T6(N - 1);
            A(8 * N - 2, 8 * N - 6) = 2.0;
            A(8 * N - 2, 8 * N - 5) = 6.0 * T1(N - 1);
            A(8 * N - 2, 8 * N - 4) = 12.0 * T2(N - 1);
            A(8 * N - 2, 8 * N - 3) = 20.0 * T3(N - 1);
            A(8 * N - 2, 8 * N - 2) = 30.0 * T4(N - 1);
            A(8 * N - 2, 8 * N - 1) = 42.0 * T5(N - 1);
            A(8 * N - 1, 8 * N - 5) = 6.0;
            A(8 * N - 1, 8 * N - 4) = 24.0 * T1(N - 1);
            A(8 * N - 1, 8 * N - 3) = 60.0 * T2(N - 1);
            A(8 * N - 1, 8 * N - 2) = 120.0 * T3(N - 1);
            A(8 * N - 1, 8 * N - 1) = 210.0 * T4(N - 1);
            
            b.row(8 * N - 4).head(3) = tailPVAL.col(0).transpose();
            b.row(8 * N - 3).head(3) = tailPVAL.col(1).transpose();
            b.row(8 * N - 2).head(3) = tailPVAL.col(2).transpose();
            b.row(8 * N - 1).head(3) = tailPVAL.col(3).transpose();
            
            if (boundSlackDim(0) == 0 && boundSlackDim(1) == 0 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    headPVAQ.row(2)(j * 3 + 2) = slackM.row(0)(j * 2 + 1);
                    headPVAQ.row(3)(j * 3 + 2) = slackM.row(0)(j * 2);
                    tailPVAQ.row(2)(j * 3 + 2) = slackM.row(1)(j * 2 + 1);
                    tailPVAQ.row(3)(j * 3 + 2) = slackM.row(1)(j * 2);
                }
            }

            b.row(0).segment(3, 3 * dN) = headPVAQ.row(0);
            b.row(1).segment(3, 3 * dN) = headPVAQ.row(1);
            b.row(2).segment(3, 3 * dN) = headPVAQ.row(2);
            b.row(3).segment(3, 3 * dN) = headPVAQ.row(3);

            b.row(8 * N - 4).segment(3, 3 * dN) = tailPVAQ.row(0);
            b.row(8 * N - 3).segment(3, 3 * dN) = tailPVAQ.row(1);
            b.row(8 * N - 2).segment(3, 3 * dN) = tailPVAQ.row(2);
            b.row(8 * N - 1).segment(3, 3 * dN) = tailPVAQ.row(3);

            // for (int i = 0; i < fixPtsNums.size(); i++)
            // {
            //     cout << i << "\t" << inPs.row(fixPtsNums(i))(0) << "\t"<< inPs.row(fixPtsNums(i))(1) << "\t"<< inPs.row(fixPtsNums(i))(2) << "\t"<< endl;;
            // }

            // cout <<b /M_PI <<endl;
            A.factorizeLU();
            A.solve(b);

            return;
        }

        inline void getTrajectory(Trajectory<7> &traj, const int &k) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.block<8, 3>(8 * i, 3 * k).transpose().rowwise().reverse());
            }
            return;
        }

        inline void getTrajs(std::vector<Trajectory<7>> &trajs) const
        {
            trajs.clear();
            trajs.reserve(dN + 1);
            for (int k = 0; k < dN + 1; k++)
            {
                for (int i = 0; i < N; i++)
                {
                    trajs[k].emplace_back(T1(i), b.block<8, 3>(8 * i, 3 * k).transpose().rowwise().reverse());
                }
            }
            return;
        }

        inline void getEnergy(double &energyL, double &energyQ) const // just load energy
        {
            energyL = 0.0;
            energyQ = 0.0;
            double e = 0.0, e_sum;
            for (int i = 0; i < N; i++)
            {
                energyL += 576.0 * b.row(8 * i + 4).head(3).squaredNorm() * T1(i) +
                           2880.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 5).head(3)) * T2(i) +
                           4800.0 * b.row(8 * i + 5).head(3).squaredNorm() * T3(i) +
                           5760.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 6).head(3)) * T3(i) +
                           21600.0 * b.row(8 * i + 5).head(3).dot(b.row(8 * i + 6).head(3)) * T4(i) +
                           10080.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 7).head(3)) * T4(i) +
                           25920.0 * b.row(8 * i + 6).head(3).squaredNorm() * T5(i) +
                           40320.0 * b.row(8 * i + 5).head(3).dot(b.row(8 * i + 7).head(3)) * T5(i) +
                           100800.0 * b.row(8 * i + 6).head(3).dot(b.row(8 * i + 7).head(3)) * T6(i) +
                           100800.0 * b.row(8 * i + 7).head(3).squaredNorm() * T7(i);
                e = (576.0 * b.row(8 * i + 4).tail(3 * dN).squaredNorm() * T1(i) +
                     2880.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 5).tail(3 * dN)) * T2(i) +
                     4800.0 * b.row(8 * i + 5).tail(3 * dN).squaredNorm() * T3(i) +
                     5760.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 6).tail(3 * dN)) * T3(i) +
                     21600.0 * b.row(8 * i + 5).tail(3 * dN).dot(b.row(8 * i + 6).tail(3 * dN)) * T4(i) +
                     10080.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T4(i) +
                     25920.0 * b.row(8 * i + 6).tail(3 * dN).squaredNorm() * T5(i) +
                     40320.0 * b.row(8 * i + 5).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T5(i) +
                     100800.0 * b.row(8 * i + 6).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T6(i) +
                     100800.0 * b.row(8 * i + 7).tail(3 * dN).squaredNorm() * T7(i));
                e_sum += e;
                energyQ += e * ratio;
            }
            // cout << e_sum << endl;
            return;
        }

        inline const Eigen::MatrixXd &getCoeffs(void) const
        {
            // Eigen::Map<const Eigen::VectorXd> bv(b.data(), (6 * N) * (3 + 3 * dN));
            // cout<< "b" << "\n" << b << endl;
            return b;
        }

        inline void getEnergyPartialGradByCoeffs(Eigen::VectorXd &gdCv) const
        {
            gdCv.resize((8 * N) * (3 + 3 * dN));
            Eigen::Map<Eigen::MatrixXd> gdC(gdCv.data(), 8 * N, 3 + 3 * dN);
            gdC.setZero();
            // cout << "gdC\t" << "\n"<<  gdC << endl;
            for (int i = 0; i < N; i++)
            {
                gdC.row(8 * i + 7) = 10080.0 * b.row(8 * i + 4) * T4(i) +
                                     40320.0 * b.row(8 * i + 5) * T5(i) +
                                     100800.0 * b.row(8 * i + 6) * T6(i) +
                                     201600.0 * b.row(8 * i + 7) * T7(i);
                gdC.row(8 * i + 6) = 5760.0 * b.row(8 * i + 4) * T3(i) +
                                     21600.0 * b.row(8 * i + 5) * T4(i) +
                                     51840.0 * b.row(8 * i + 6) * T5(i) +
                                     100800.0 * b.row(8 * i + 7) * T6(i);
                gdC.row(8 * i + 5) = 2880.0 * b.row(8 * i + 4) * T2(i) +
                                     9600.0 * b.row(8 * i + 5) * T3(i) +
                                     21600.0 * b.row(8 * i + 6) * T4(i) +
                                     40320.0 * b.row(8 * i + 7) * T5(i);
                gdC.row(8 * i + 4) = 1152.0 * b.row(8 * i + 4) * T1(i) +
                                     2880.0 * b.row(8 * i + 5) * T2(i) +
                                     5760.0 * b.row(8 * i + 6) * T3(i) +
                                     10080.0 * b.row(8 * i + 7) * T4(i);

                gdC.row(8 * i + 7).tail(3 * dN) *= ratio;
                gdC.row(8 * i + 6).tail(3 * dN) *= ratio;
                gdC.row(8 * i + 5).tail(3 * dN) *= ratio;
                gdC.row(8 * i + 4).tail(3 * dN) *= ratio;
                // gdC.block<4, 3>(8 * i, 0).setZero();
            }
            return;
        }

        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 576.0 * b.row(8 * i + 4).head(3).squaredNorm() +
                         5760.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 5).head(3)) * T1(i) +
                         14400.0 * b.row(8 * i + 5).head(3).squaredNorm() * T2(i) +
                         17280.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 6).head(3)) * T2(i) +
                         86400.0 * b.row(8 * i + 5).head(3).dot(b.row(8 * i + 6).head(3)) * T3(i) +
                         40320.0 * b.row(8 * i + 4).head(3).dot(b.row(8 * i + 7).head(3)) * T3(i) +
                         129600.0 * b.row(8 * i + 6).head(3).squaredNorm() * T4(i) +
                         201600.0 * b.row(8 * i + 5).head(3).dot(b.row(8 * i + 7).head(3)) * T4(i) +
                         604800.0 * b.row(8 * i + 6).head(3).dot(b.row(8 * i + 7).head(3)) * T5(i) +
                         705600.0 * b.row(8 * i + 7).head(3).squaredNorm() * T6(i);
                gdT(i) += ratio * (576.0 * b.row(8 * i + 4).tail(3 * dN).squaredNorm() +
                                   5760.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 5).tail(3 * dN)) * T1(i) +
                                   14400.0 * b.row(8 * i + 5).tail(3 * dN).squaredNorm() * T2(i) +
                                   17280.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 6).tail(3 * dN)) * T2(i) +
                                   86400.0 * b.row(8 * i + 5).tail(3 * dN).dot(b.row(8 * i + 6).tail(3 * dN)) * T3(i) +
                                   40320.0 * b.row(8 * i + 4).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T3(i) +
                                   129600.0 * b.row(8 * i + 6).tail(3 * dN).squaredNorm() * T4(i) +
                                   201600.0 * b.row(8 * i + 5).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T4(i) +
                                   604800.0 * b.row(8 * i + 6).tail(3 * dN).dot(b.row(8 * i + 7).tail(3 * dN)) * T5(i) +
                                   705600.0 * b.row(8 * i + 7).tail(3 * dN).squaredNorm() * T6(i));
            }
            return;
        }

        inline void propogateGrad(const Eigen::VectorXd &partialGradByCoeffs,
                                  const Eigen::VectorXd &partialGradByTimes,
                                  Eigen::MatrixX3d &gradByPoints,
                                  Eigen::VectorXd  &gradByAngTsV,
                                  Eigen::VectorXd &gradByTimes,
                                  Eigen::VectorXd &grad,
                                  const int &dimXiL,
                                  const int &dimXiQ,
                                  const int &dimXiContinuityDesDim)

        {
            gradByTimes.resize(N);
            gradByTimes.setZero();
            gradByPoints.resize(N - 1, 3);
            gradByAngTsV.resize((N - 1) * (3 * dN));
            Eigen::Map<Eigen::MatrixXd> gradByAngTsM(gradByAngTsV.data(), N - 1, 3 * dN);
            Eigen::Map<Eigen::MatrixXd> gradByContinuityDescendM(grad.data() + dimXiL + dimXiQ, N - 1, contDescendDim.sum() * dN);
            Eigen::Map<Eigen::MatrixXd> gradByBoundarySlackM(grad.data() + dimXiL + dimXiQ + dimXiContinuityDesDim, 2, boundSlackDim.sum() * dN);
            Eigen::Map<const Eigen::MatrixXd> adjGrad_(partialGradByCoeffs.data(), 8 * N, 3 + 3 * dN);

            // cout << "adjGrad_\n" << adjGrad_ << endl;
            Eigen::MatrixXd adjGrad = adjGrad_;
            A.solveAdj(adjGrad);
            // cout << "adjGrad\n" << adjGrad << endl;
            
            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.row(i) = adjGrad.row(8 * i + 7).segment<3>(0);
                gradByAngTsM.row(i) = adjGrad.row(8 * i + 7).segment(3, 3 * dN);
                if (contDescendDim(0) == 3 && contDescendDim(1) == 3 && contDescendDim(2) == 5)
                {
                    for (int j = 0; j < dN; j++)
                    {
                       gradByContinuityDescendM.row(i).segment<3>(11 * j) = adjGrad.row(8 * i + 6).segment<3>(3 + j * 3);  //dds(tpT)
                       gradByContinuityDescendM.row(i).segment<3>(11 * j + 3) = adjGrad.row(8 * i + 5).segment<3>(3 + j * 3);
                       gradByContinuityDescendM.row(i).segment<3>(11 * j + 6) = adjGrad.row(8 * i + 4).segment<3>(3 + j * 3);
                       gradByContinuityDescendM.row(i)(j * 11 + 9) = adjGrad.row(8 * i + 11)(3 + j * 3 + 2);               //j(T)
                       gradByContinuityDescendM.row(i)(j * 11 + 10) = adjGrad.row(8 * i + 10)(3 + j * 3 + 2);                //a(T)
                    }
                }

            }

            if (boundSlackDim(0) == 0 && boundSlackDim(1) == 0 && boundSlackDim(2) == 2)
            {
                for (int j = 0; j < dN; j++)
                {
                    gradByBoundarySlackM.row(0)(2 * j + 1) = adjGrad.row(2)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(0)(2 * j) = adjGrad.row(3)(3 + j * 3 + 2); // sss(tpT)
                    gradByBoundarySlackM.row(1)(2 * j + 1) = adjGrad.row(8 * N - 2)(3 + j * 3 + 2);
                    gradByBoundarySlackM.row(1)(2 * j) = adjGrad.row(8 * N - 1)(3 + j * 3 + 2); // sss(tpT)
                }
            }

            for (int i = 0; i < fixPtsNums.size(); i++)
            {
                gradByPoints.row(fixPtsNums(i)) = Eigen::Vector3d::Zero();
            }
            // cout <<gradByBoundarySlackM << endl;

            Eigen::MatrixXd B1;
            B1.resize(8, 3 + 3 * dN);
            Eigen::Matrix4Xd B2;
            B2.resize(4, 3 + 3 * dN);
            for (int i = 0; i < N - 1; i++)
            {

                // negative crackle
                B1.row(0) = -(120.0 * b.row(i * 8 + 5) +
                              720.0 * T1(i) * b.row(i * 8 + 6) +
                              2520.0 * T2(i) * b.row(i * 8 + 7));

                // negative d_crackle
                B1.row(1) = -(720.0 * b.row(i * 8 + 6) +
                              5040.0 * T1(i) * b.row(i * 8 + 7));

                // negative dd_crackle
                B1.row(2) = -5040.0 * b.row(i * 8 + 7);
                
                // negative velocity
                B1.row(3) = -(b.row(i * 8 + 1) +
                              2.0 * T1(i) * b.row(i * 8 + 2) +
                              3.0 * T2(i) * b.row(i * 8 + 3) +
                              4.0 * T3(i) * b.row(i * 8 + 4) +
                              5.0 * T4(i) * b.row(i * 8 + 5) +
                              6.0 * T5(i) * b.row(i * 8 + 6) +
                              7.0 * T6(i) * b.row(i * 8 + 7));

                B1.row(4) = B1.row(3);

                // negative acceleration
                B1.row(5) = -(2.0 * b.row(i * 8 + 2) +
                              6.0 * T1(i) * b.row(i * 8 + 3) +
                              12.0 * T2(i) * b.row(i * 8 + 4) +
                              20.0 * T3(i) * b.row(i * 8 + 5) +
                              30.0 * T4(i) * b.row(i * 8 + 6) +
                              42.0 * T5(i) * b.row(i * 8 + 7));

                // negative jerk
                B1.row(6) = -(6.0 * b.row(i * 8 + 3) +
                              24.0 * T1(i) * b.row(i * 8 + 4) +
                              60.0 * T2(i) * b.row(i * 8 + 5) +
                              120.0 * T3(i) * b.row(i * 8 + 6) +
                              210.0 * T4(i) * b.row(i * 8 + 7));

                // negative snap
                B1.row(7) = -(24.0 * b.row(i * 8 + 4) +
                              120.0 * T1(i) * b.row(i * 8 + 5) +
                              360.0 * T2(i) * b.row(i * 8 + 6) +
                              840.0 * T3(i) * b.row(i * 8 + 7));

                for(int j = 0; j < 8; j++)
                {
                    gradByTimes(i) += (B1.row(j) * adjGrad.row(8 * i + 4 + j).transpose()).sum(); 
                }
            }
            // cout << "grad time2\t" << gradByTimes << endl;

            // negative velocity
            B2.row(0) = -(b.row(8 * N - 7) +
                          2.0 * T1(N - 1) * b.row(8 * N - 6) +
                          3.0 * T2(N - 1) * b.row(8 * N - 5) +
                          4.0 * T3(N - 1) * b.row(8 * N - 4) +
                          5.0 * T4(N - 1) * b.row(8 * N - 3) +
                          6.0 * T5(N - 1) * b.row(8 * N - 2) +
                          7.0 * T6(N - 1) * b.row(8 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(8 * N - 6) +
                          6.0 * T1(N - 1) * b.row(8 * N - 5) +
                          12.0 * T2(N - 1) * b.row(8 * N - 4) +
                          20.0 * T3(N - 1) * b.row(8 * N - 3) +
                          30.0 * T4(N - 1) * b.row(8 * N - 2) +
                          42.0 * T5(N - 1) * b.row(8 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(8 * N - 5) +
                          24.0 * T1(N - 1) * b.row(8 * N - 4) +
                          60.0 * T2(N - 1) * b.row(8 * N - 3) +
                          120.0 * T3(N - 1) * b.row(8 * N - 2) +
                          210.0 * T4(N - 1) * b.row(8 * N - 1));

            // negative snap
            B2.row(3) = -(24.0 * b.row(8 * N - 4) +
                          120.0 * T1(N - 1) * b.row(8 * N - 3) +
                          360.0 * T2(N - 1) * b.row(8 * N - 2) +
                          840.0 * T3(N - 1) * b.row(8 * N - 1));

            for (int j = 0; j < 4; j++)
            {
                gradByTimes(N - 1) += (B2.row(j) * adjGrad.row(8 * N - 4 + j).transpose()).sum();
            }
            
            // cout << "grad time3\t" << gradByTimes << endl;
            
            gradByTimes += partialGradByTimes;
        }
    };

}

#endif
