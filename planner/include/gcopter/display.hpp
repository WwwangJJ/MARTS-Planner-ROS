#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include "misc/visualizer.hpp"
#include "misc/tinycolormap.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/funs.hpp"
#include "gcopter/flatness.hpp"
#include <fstream>
using namespace std;
using namespace gcopter;

class Display // See https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf
{
private:
    int dN, pN, wN, n;
    int integralRes;
    std::vector<std::vector<Eigen::Vector3d>> angTs, posQs, qs, upperT, lowerT, upperTheta, lowerTheta, upperPsi, lowerPsi, theta, psi, T, displayed_force, thetaWpt, psiWpt, TWpt, errWpt, errs, velQMax, omgQMax, attQMax, TQMax, velQMin, omgQMin, attQMin, TQMin, velQ, omgQ, attQ, TQ;
    std::vector<std::vector<Eigen::Vector2d>> csts, csps;
    std::vector<Eigen::Vector3d> posLs, arrows, bs, initPts, endPts;

    std::vector<Eigen::Vector3d> accs, forces;
    std::vector<std::vector<Eigen::Vector4d>> quat;
    std::vector<Eigen::Vector3d> velL, accL, jerL;
    Eigen::Vector3d selPosQ, angT, acc, vel, q, force;
    Eigen::VectorXd offset;
    Eigen::Vector4d g;
    double dt = 0.01, dT, t1;
    double scl, ratio;
    double err;
    double bias;
    Eigen::VectorXd phyParams, angTBds, magnBds;

public:
    inline void reset(const Eigen::VectorXd &physicalParams, const Eigen::VectorXd &angTBounds, const Eigen::VectorXd &magnitudeBounds, 
                      const int &droneN, const double &scale, const int & integralResolution)
    {
        phyParams = physicalParams;
        angTBds = angTBounds;
        magnBds = magnitudeBounds;
        dN = droneN;
        scl = scale;
        t1 = 10.0;
        bias = 15.0;
        integralRes = integralResolution;
        g.setZero();
        offset.resize(8);
        return;
    }
    inline void displayTrajs(Visualizer &visualizer, const std::vector<Trajectory<7>> &trajs, const double &per, Eigen::Vector4d color, const double &scale, int id, tinycolormap::ColormapType color_Load)
    {
        // cout << "ACFT" << endl;
        pN = floor(trajs[0].getTotalDuration() / dt);
        n = floor(trajs[0].getTotalDuration() * per * 0.01 / dt);
        posLs.resize(pN + 1);
        velL.resize(pN + 1);
        accL.resize(pN + 1);
        jerL.resize(pN + 1);
        posQs.resize(dN);
        quat.resize(dN);
        qs.resize(dN);
        angTs.resize(dN);
        csts.resize(dN);
        csps.resize(dN);
        Eigen::Vector4d colorL;
        // colorL << 230.0 / 255.0, 210.0 / 255.0, 213.0/255.0, 1.0;
        colorL << 0, 0, 0, 0;
        ofstream p;
        p.open("/home/wangjunjie/Line_Load.csv",ios::out|ios::trunc);
        for (int i = 0; i <= pN; i++)
        {
            posLs[i] = trajs[0].getPos(i * dt);
            velL[i] = trajs[0].getVel(i * dt);
            accL[i] = trajs[0].getAcc(i * dt);
            jerL[i] = trajs[0].getJer(i * dt);
            if (id == 1)
            {
                // cout << "pos\t"<< i << "\t" << posLs[i](0) << "\t" << posLs[i](1) << "\t" << posLs[i](2) << "\t" << trajs[0].getJer(i * dt).norm() << endl;
            }
            p << posLs[i](0) << "," << posLs[i](1) << "," << posLs[i](2) << endl;
        }
        p.close();
        // if(scale != 0)
        // {
        //     // color = Eigen::Vector4d(234.0, 159.0, 147.0, 1.0 * 255)/255;
        //     color = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0 * 255)/255;
        //     visualizer.displayMarkerList(posLs, scale, color, 1000 + id, color_Load, false);
        // }
        visualizer.displayMarkerList(posLs, 0.2, colorL, 1000 + id, color_Load, false);
        ofstream p1;
        ofstream p2;
        ofstream p3;
        for (int k = 0; k < dN; k++)
        {
            posQs[k].resize(pN + 1);
            qs[k].resize(pN + 1);
            angTs[k].resize(pN + 1);
            csts[k].resize(pN + 1);
            csps[k].resize(pN + 1);

            for (int i = 0; i <= pN; i++)
            {
                angTs[k][i] = trajs[k + 1].getPos(i * dt);
                // cout << k << "\t"<< i << "\t" << angTs[k][i](0) / M_PI << "\t" << angTs[k][i](1) / M_PI << "\t" << angTs[k][i](2)<< endl;
                csts[k][i] = getcs(angTs[k][i](0));
                csps[k][i] = getcs(angTs[k][i](1) + 2 * M_PI * k / dN);
                qs[k][i] = getQ(csts[k][i], csps[k][i], g);
                posQs[k][i] = posLs[i] + phyParams(6) * qs[k][i];
            }
            color = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0 * 255)/255;
            visualizer.displayMarkerList(posQs[k], scale, color, 1000 + 2 + dN * id + k, color_Load);

            selPosQ = posLs[n] + phyParams(6) * qs[k][n];
            visualizer.displaySelectedPos(selPosQ, color, 0.2, 1000 + dN * id + k);
        }
        return ;
    }

    inline void displayForce(Visualizer &visualizer, const std::vector<Trajectory<7>> &initTrajs, const std::vector<Trajectory<7>> &optTrajs,   tinycolormap::ColormapType color_type, double color_a)
    {

        displayed_force.resize(dN);
        for (int k = 0; k < dN; k++)
        {
            displayed_force[k].resize(pN + 1);
            for(int i = 0; i <= pN; i++)
            {
                displayed_force[k][i].setZero();
                displayed_force[k][i] += scl * angTs[k][i](2) * qs[k][i];
            }
            Eigen::Vector4d color;
            color = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0 * 255)/255;

            visualizer.displayForceArrow(posQs[k], displayed_force[k], color_type, k, color, color_a);

        }
        return ;
    }
    inline void displayDrone(Visualizer &visualizer, const std::vector<Trajectory<7>> &initTrajs, const std::vector<Trajectory<7>> &optTrajs, tinycolormap::ColormapType color_type, double color_a)
    {
        for (int k = 0; k < dN; k++)
        {   
            for(int i = 0; i <= pN; i++)
            {
                double thr;
                // Eigen::Vector4d g;
                Eigen::Vector3d omg, dAngT, d2AngT, d3AngT, q, dq, d2q, d3q;
                Eigen::Vector3d posQ, velQ, accQ, jerQ;
                flatness::FlatnessMap flatmap;
                flatmap.reset(phyParams(0), phyParams(1), phyParams(6));

                quat[k].resize(pN + 1);

                dAngT = optTrajs[k + 1].getVel(i * dt);
                d2AngT = optTrajs[k + 1].getAcc(i * dt);
                d3AngT = optTrajs[k + 1].getJer(i * dt);

                q = getQ(csts[k][i], csps[k][i], g);
                dq = getdQ(dAngT.head(2), csts[k][i]);
                d2q = getd2Q(dAngT.head(2), d2AngT.head(2), csts[k][i]);
                d3q = getd3Q(dAngT.head(2), d2AngT.head(2), d3AngT.head(2), csts[k][i]);

                posQ = posLs[i] + phyParams(6) * q;
                velQ = velL[i] + phyParams(6) * dq;
                accQ = accL[i] + phyParams(6) * d2q;
                jerQ = jerL[i] + phyParams(6) * d3q;

                flatmap.forward(accQ, jerQ, q, dq, scl * angTs[k][i](2), scl * dAngT(2), 0.0, 0.0, thr, quat[k][i], omg);
            }

            Eigen::Vector4d color;
        

            color = Eigen::Vector4d(21.0, 85.0, 154.0, 1.0 * 255);
            visualizer.displayDroneArray(posQs[k], quat[k], color_type, k, color, color_a);
        }

    }
    

    inline void displayDetails(Visualizer &visualizer, const std::vector<Trajectory<7>> &initTrajs, const std::vector<Trajectory<7>> &optTrajs,
                               const std::vector<Eigen::VectorXd> velQs, const std::vector<Eigen::VectorXd> omgQs,
                               const std::vector<Eigen::VectorXd> attQs, const std::vector<Eigen::VectorXd> TQs)
    {
        // cout << "AAAAAAAA\t" << optTrajs[0].getDurations() << endl;
        ratio = initTrajs[0].getTotalDuration() / optTrajs[0].getTotalDuration();
        accs.resize(pN + 1);
        forces.resize(pN + 1);
        upperTheta.resize(dN);
        upperPsi.resize(dN);
        upperT.resize(dN);
        lowerTheta.resize(dN);
        lowerPsi.resize(dN);
        lowerT.resize(dN);
        theta.resize(dN);
        psi.resize(dN);
        T.resize(dN);
        // force.resize(dN);
        thetaWpt.resize(dN);
        psiWpt.resize(dN);
        TWpt.resize(dN);
        errs.resize(dN);
        errWpt.resize(dN);
        velQMax.resize(dN);
        velQMin.resize(dN);
        velQ.resize(dN);
        omgQMax.resize(dN);
        omgQMin.resize(dN);
        omgQ.resize(dN);
        attQMax.resize(dN);
        attQMin.resize(dN);
        attQ.resize(dN);
        TQMax.resize(dN);
        TQMin.resize(dN);
        TQ.resize(dN);
        wN = optTrajs[0].getPieceNum();
        initPts.resize(wN -1);
        endPts.resize(wN -1);

        for (int i = 0; i < wN - 1; i++)
        {
            initPts[i] = initTrajs[0].getJuncPos(i);
            endPts[i] = optTrajs[0].getJuncPos(i);
        }
        // visualizer.displayMarkerList(initPts, 0.3, Eigen::Vector4d(0.0, 0.5, 1.0, 1.0), 2800000, true, false);
        // visualizer.displayMarkerList(endPts, 0.3, Eigen::Vector4d(0.0, 1.0, 0.5, 1.0), 3800000, true, false);

        for (int k = 0; k < dN; k++)
        {
            upperTheta[k].resize(pN + 1);
            lowerTheta[k].resize(pN + 1);
            theta[k].resize(pN + 1);
            upperPsi[k].resize(pN + 1);
            lowerPsi[k].resize(pN + 1);
            psi[k].resize(pN + 1);
            upperT[k].resize(pN + 1);
            lowerT[k].resize(pN + 1);
            T[k].resize(pN + 1);
            errs[k].resize(pN + 1);
            thetaWpt[k].resize(wN - 1);
            psiWpt[k].resize(wN - 1);
            TWpt[k].resize(wN - 1);
            errWpt[k].resize(wN - 1);
            velQMax[k].resize(wN * integralRes + 1);
            velQMin[k].resize(wN * integralRes + 1);
            velQ[k].resize(wN * integralRes + 1);
            omgQMax[k].resize(wN * integralRes + 1);
            omgQMin[k].resize(wN * integralRes + 1);
            omgQ[k].resize(wN * integralRes + 1);
            attQMax[k].resize(wN * integralRes + 1);
            attQMin[k].resize(wN * integralRes + 1);
            attQ[k].resize(wN * integralRes + 1);
            TQMax[k].resize(wN * integralRes + 1);
            TQMin[k].resize(wN * integralRes + 1);
            TQ[k].resize(wN * integralRes + 1);
        }
        for (int i = 0; i <= pN; i++)
        {
            accs[i] = optTrajs[0].getAcc(i * dt);
            accs[i](2) += phyParams(1);
            accs[i] *= phyParams(7);
            forces[i].setZero();
            for (int k = 0; k < dN; k++)
            {
                forces[i] += scl * angTs[k][i](2) * qs[k][i];
            }
        }


        offset << -4.0, -6.0, -10.0, -14.0, -19.0, -22.0, -25.0, -36.0;     // 1,theta,  2,psi,  3,T,  4,force error,  5,vel,  6,omg, 7,tilt angle, 8,thrust   
        for (int k = 0; k < dN; k++)
        {
            for (int i = 0; i <= pN; i++)
            {
                upperTheta[k][i](0) = i * dt * ratio + k * bias;
                upperTheta[k][i](1) = angTBds(0) + offset(0);
                upperTheta[k][i](2) = 0.0;
                lowerTheta[k][i](0) = i * dt * ratio + k * bias;
                lowerTheta[k][i](1) = angTBds(1) + offset(0);
                lowerTheta[k][i](2) = 0.0;
                theta[k][i](0) = i * dt * ratio + k * bias;
                theta[k][i](1) = angTs[k][i](0) + offset(0);
                theta[k][i](2) = 0.0;

                upperPsi[k][i](0) = i * dt * ratio + k * bias;
                upperPsi[k][i](1) = angTBds(2) + offset(1);
                upperPsi[k][i](2) = 0.0;
                lowerPsi[k][i](0) = i * dt * ratio + k * bias;
                lowerPsi[k][i](1) = angTBds(3) + offset(1);
                lowerPsi[k][i](2) = 0.0;
                psi[k][i](0) = i * dt * ratio + k * bias;
                psi[k][i](1) = angTs[k][i](1) + offset(1);
                psi[k][i](2) = 0.0;

                upperT[k][i](0) = i * dt * ratio + k * bias;
                upperT[k][i](1) = angTBds(4) + offset(2);
                upperT[k][i](2) = 0.0;
                lowerT[k][i](0) = i * dt * ratio + k * bias;
                lowerT[k][i](1) = angTBds(5) + offset(2);
                // lowerT[k][i](1) = (180.0 * acos(qs[k][i](2)) / M_PI - 30.0) * 2.5 + offset(2);
                lowerT[k][i](2) = 0.0;
                T[k][i](0) = i * dt * ratio + k * bias;
                T[k][i](1) = angTs[k][i](2) + offset(2);
                T[k][i](2) = 0.0;

                errs[k][i](0) = i * dt * ratio + k * bias;
                errs[k][i](1) = sqrt((forces[i] - accs[i]).squaredNorm()) + offset(3);
                errs[k][i](2) = 0.0;
            }
            dT = 0.0;
            for (int i = 0; i < wN - 1; i++)
            {
                dT += optTrajs[0].getDurations()(i);
                angT = optTrajs[k + 1].getPos(dT);
                thetaWpt[k][i](0) = dT * ratio + k * bias;
                thetaWpt[k][i](1) = angT(0) + offset(0);
                thetaWpt[k][i](2) = 0.0;

                psiWpt[k][i](0) = dT * ratio + k * bias;
                psiWpt[k][i](1) = angT(1) + offset(1);
                psiWpt[k][i](2) = 0.0;

                TWpt[k][i](0) = dT * ratio + k * bias;
                TWpt[k][i](1) = angT(2) + offset(2);
                TWpt[k][i](2) = 0.0;
            }
            // visualizer.displayMarkerList(upperTheta[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 0);
            // visualizer.displayMarkerList(lowerTheta[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 1);
            // visualizer.displayMarkerList(theta[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 2); 
            // visualizer.displayMarkerList(thetaWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 3, true, false);
            // visualizer.displayMarkerList(upperPsi[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 4);
            // visualizer.displayMarkerList(lowerPsi[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 5);
            // visualizer.displayMarkerList(psi[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 6); 
            // visualizer.displayMarkerList(psiWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 7, true, false);
            // visualizer.displayMarkerList(upperT[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 8);
            // visualizer.displayMarkerList(lowerT[k], 0.1, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), k * 30 + 9);
            // visualizer.displayMarkerList(T[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 10); 
            // visualizer.displayMarkerList(TWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 11, true, false);
            // visualizer.displayMarkerList(errs[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 12); 
        }
        dT = 0.0;
        for (int i = 0; i < wN - 1; i++)
        {
            dT += optTrajs[0].getDurations()(i);
            vel = optTrajs[0].getVel(dT);
            acc = optTrajs[0].getAcc(dT);
            cout << "vel\t" << i  << "\t" << vel(0) << "\t" << vel(1) << "\t" << vel(2) << endl;
            cout << "acc\t" << i  << "\t" << acc(0) << "\t" << acc(1) << "\t" << acc(2) << endl;
            acc(2) += phyParams(1);
            acc *= phyParams(7);
            force.setZero();
            for (int k = 0; k < dN; k++)
            {
                angT = optTrajs[k + 1].getPos(dT);
                force += scl * angT(2) * getQ(getcs(angT(0)), getcs(angT(1) + 2 * M_PI * k / dN), g);
            }
            err = sqrt((force - acc).squaredNorm());
            for (int k = 0; k < dN; k++)
            {
                errWpt[k][i](0) = dT * ratio + k * bias;
                errWpt[k][i](1) = err + offset(3);
                errWpt[k][i](2) = 0.0;
            }
        }
        // for (int k = 0; k < dN; k++)
        // {
        //     visualizer.displayMarkerList(errWpt[k], 0.1, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), k * 30 + 13, true, false);
        // }

        dT = initTrajs[0].getDurations()[0] / integralRes;
        for (int i = 0; i <= wN * integralRes; i++)
        {
            for (int k = 0; k < dN; k++)
            {
                velQMax[k][i](0) = i * dT + k * bias;
                velQMax[k][i](1) = magnBds(0) + offset(4);
                velQMax[k][i](2) = 0.0;
                velQMin[k][i](0) = i * dT + k * bias;
                velQMin[k][i](1) = 0.0 + offset(4);
                velQMin[k][i](2) = 0.0;
                velQ[k][i](0) = i * dT + k * bias;
                velQ[k][i](1) = velQs[k][i] + offset(4);
                // cout << "vel\t" << k << "\t" << i << "\t" << velQs[k][i] << endl;
                velQ[k][i](2) = 0.0;
                omgQMax[k][i](0) = i * dT + k * bias;
                omgQMax[k][i](1) = magnBds(1) + offset(5);
                omgQMax[k][i](2) = 0.0;
                omgQMin[k][i](0) = i * dT + k * bias;
                omgQMin[k][i](1) = 0.0 + offset(5);
                omgQMin[k][i](2) = 0.0;
                omgQ[k][i](0) = i * dT + k * bias;
                omgQ[k][i](1) = omgQs[k][i] + offset(5);
                omgQ[k][i](2) = 0.0;
                // cout << "omg\t" << k << "\t" << i << "\t" << omgQs[k][i] << endl;
                attQMax[k][i](0) = i * dT + k * bias;
                attQMax[k][i](1) = magnBds(2) + offset(6);
                attQMax[k][i](2) = 0.0;
                attQMin[k][i](0) = i * dT + k * bias;
                attQMin[k][i](1) = 0.0 + offset(6);
                attQMin[k][i](2) = 0.0;
                attQ[k][i](0) = i * dT + k * bias;
                attQ[k][i](1) = attQs[k][i] + offset(6);
                attQ[k][i](2) = 0.0;
                // cout << "att\t" << k << "\t" << i << "\t" << attQs[k][i] * 180.0 / M_PI << endl;
                TQMax[k][i](0) = i * dT + k * bias;
                TQMax[k][i](1) = 0.5 * (magnBds(4) - magnBds(3)) + offset(7);
                TQMax[k][i](2) = 0.0;
                TQMin[k][i](0) = i * dT + k * bias;
                TQMin[k][i](1) = -0.5 * (magnBds(4) - magnBds(3)) + offset(7);
                TQMin[k][i](2) = 0.0;
                TQ[k][i](0) = i * dT + k * bias;
                TQ[k][i](1) = TQs[k][i] + offset(7);
                TQ[k][i](2) = 0.0;
                // cout << "T\t" << k << "\t" << i << "\t" << TQs[k][i] << endl;
            }
        }
        // for (int k = 0; k < dN; k++)
        // {
        //     visualizer.displayMarkerList(velQMax[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 14);
        //     visualizer.displayMarkerList(velQMin[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 15);
        //     visualizer.displayMarkerList(velQ[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 16);
        //     visualizer.displayMarkerList(omgQMax[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 17);
        //     visualizer.displayMarkerList(omgQMin[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 18);
        //     visualizer.displayMarkerList(omgQ[k], 0.2, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 19);
        //     visualizer.displayMarkerList(attQMax[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 20);
        //     visualizer.displayMarkerList(attQMin[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 21);
        //     visualizer.displayMarkerList(attQ[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 22);
        //     visualizer.displayMarkerList(TQMax[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 23);
        //     visualizer.displayMarkerList(TQMin[k], 0.2, Eigen::Vector4d(0.0, 0.0, 1.0, 1.0), k * 30 + 24);
        //     visualizer.displayMarkerList(TQ[k], 0.1, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), k * 30 + 25);
        // }


        return;
    }

    inline void displayAllows(Visualizer &visualizer, const std::vector<Trajectory<7>> &optTrajs, const double &per)
    {
        arrows.resize(2);
        n = floor(optTrajs[0].getTotalDuration() * per * 0.01 / dt);
        arrows[0] = posLs[n];
        arrows[1] = posLs[n] + accs[n] / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(1.0, 0.0, 1.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 0);
        arrows[1] = posLs[n] + forces[n] / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 1);
        arrows[1] = posLs[n] + Eigen::Vector3d(0.0, 0.0, phyParams(1)) / scl;
        visualizer.displayVectorArrow(arrows, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d(0.04, 0.07, 0.2), 0.5, 2);
        return ;
    }

    inline void displayBounds(Visualizer &visualizer, const std::vector<Trajectory<7>> &optTrajs, const double &per)
    {
        n = floor(optTrajs[0].getTotalDuration() * per * 0.01 / dt);
        arrows[0] = posLs[n];
        int m = 100;
        double psi, dp;
        dp = 2 * M_PI / m;
        bs.resize(m + 1);
        double ctma = cos(angTBds(0));
        double stma = sin(angTBds(0));
        for (int i = 0; i < m + 1; i++)
        {
          psi = i * dp;
          bs[i](0) = posLs[n](0) + phyParams(6) * stma * cos(psi);
          bs[i](1) = posLs[n](1) + phyParams(6) * stma * sin(psi);
          bs[i](2) = posLs[n](2) + 0.0;
        }
        // visualizer.displayMarkerList(bs, 0.15, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 26, false, true);
        m = 10;
        for (int k = 0; k < dN; k++)
        {
          psi = 2 * M_PI * k / dN + angTBds(2);
          bs.resize(m + 1);
          for (int i = 0; i < m + 1; i++)
          {
                bs[i](0) = posLs[n](0) + (i * phyParams(6) / m) * stma * cos(psi);
                bs[i](1) = posLs[n](1) + (i * phyParams(6) / m) * stma * sin(psi);
                bs[i](2) = posLs[n](2) + 0.0;
          }
        //   visualizer.displayMarkerList(bs, 0.15, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), 27 + k, false, true);
        }
        return ;
    }
};
