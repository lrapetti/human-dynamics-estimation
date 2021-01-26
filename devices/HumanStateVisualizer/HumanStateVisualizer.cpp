/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateVisualizer.h"
#include "IHumanState.h"
#include <HumanDynamicsEstimation/HumanState.h>

#include <yarp/os/LogStream.h>
#include <iostream>

const std::string DeviceName = "HumanStateVisualizer";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class HumanStateVisualizer::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;
};

HumanStateVisualizer::HumanStateVisualizer()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateVisualizer::~HumanStateVisualizer() {}

bool HumanStateVisualizer::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isDouble())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}

bool HumanStateVisualizer::close()
{
    return true;
}

void HumanStateVisualizer::run()
{

    // Get data from the interface
    std::array<double, 3> CoMPositionInterface = pImpl->iHumanState->getCoMPosition();
    std::array<double, 3> CoMVelocityInterface = pImpl->iHumanState->getCoMVelocity();
    std::array<double, 3> basePositionInterface = pImpl->iHumanState->getBasePosition();
    std::array<double, 4> baseOrientationInterface = pImpl->iHumanState->getBaseOrientation();
    std::array<double, 6> baseVelocity = pImpl->iHumanState->getBaseVelocity();
    std::vector<double> jointPositionsInterface = pImpl->iHumanState->getJointPositions();
    std::vector<double> jointVelocitiesInterface = pImpl->iHumanState->getJointVelocities();
    std::vector<std::string> jointNames = pImpl->iHumanState->getJointNames();
    std::string baseName = pImpl->iHumanState->getBaseName();

    // TODO
    std::cout << "visualizer running" << std::endl;
}

bool HumanStateVisualizer::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
        yError() << LogPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    yInfo() << LogPrefix << "Human State interface: ";

    for (int i = 0; i < pImpl->iHumanState->getJointNames().size(); i++) {
        yInfo() << "Joint name (" << i << "): " << pImpl->iHumanState->getJointNames()[i];
    }

    if (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yError() << LogPrefix << "The IHumanState interface might not be ready";
        return false;
    }

    yDebug() << LogPrefix << "Read" << pImpl->iHumanState->getNumberOfJoints() << "joints";

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    return true;
}

void HumanStateVisualizer::threadRelease() {}

bool HumanStateVisualizer::detach()
{
    while (isRunning()) {
        stop();
    }

    pImpl->iHumanState = nullptr;

    return true;
}

bool HumanStateVisualizer::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool HumanStateVisualizer::detachAll()
{
    return detach();
}
