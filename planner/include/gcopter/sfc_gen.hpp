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

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

namespace sfc_gen
{

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);

        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] = g(2) - lb(2);

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();

        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;
        if (solved)
        {
            p.clear();
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;
    }

}

#endif
