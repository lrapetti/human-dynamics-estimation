/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanStateVisualizer.h"
#include "IHumanState.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>

#include <iostream>

const std::string DeviceName = "HumanStateVisualizer";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class HumanStateVisualizer::impl
{
public:
    hde::interfaces::IHumanState* iHumanState = nullptr;
    iDynTree::Visualizer viz;

    iDynTree::Transform wHb;
    iDynTree::VectorDynSize joints;
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

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "urdf option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asDouble();
    const std::string urdfFileName = config.find("urdf").asString();

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    // =========================
    // INITIALIZE THE VISUALIZER
    // =========================

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return false;
    }

    iDynTree::VisualizerOptions options, textureOptions;
    // iDynTree::ITexture* textureInterface = viz.textures().add("AdditionalTexture", textureOptions);

    // pImpl->viz.camera().setPosition(iDynTree::Position(1.2, 0.0, 0.5));
    // pImpl->viz.camera().setTarget(iDynTree::Position(-0.15, 0.0, 0.15));
    // viz.camera().animator()->enableMouseControl(true);

    yInfo() << LogPrefix << "Initializing Visualization";

    // pImpl->viz.init(options);

    yInfo() << LogPrefix << "Loading Model ...";

    pImpl->viz.addModel(modelLoader.model(), "human");

    yInfo() << LogPrefix << "Model loaded";

    // pImpl->viz.camera().animator()->enableMouseControl();

    pImpl->viz.run();

    // ====================
    // INITIALIZE VARIABLES
    // ====================

    pImpl->wHb = iDynTree::Transform::Identity();
    pImpl->joints.resize(pImpl->viz.modelViz("human").model().getNrOfDOFs());
    pImpl->joints.zero();

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

    // Get human state from iHumanState
    iDynTree::Vector4 quaternion;
    quaternion.setVal(0, baseOrientationInterface.at(0));
    quaternion.setVal(1, baseOrientationInterface.at(1));
    quaternion.setVal(2, baseOrientationInterface.at(2));
    quaternion.setVal(3, baseOrientationInterface.at(3));

    iDynTree::Position position;
    position.setVal(0, basePositionInterface.at(0));
    position.setVal(1, basePositionInterface.at(1));
    position.setVal(2, basePositionInterface.at(2));

    pImpl->wHb.setRotation(iDynTree::Rotation::RotationFromQuaternion(quaternion));
    pImpl->wHb.setPosition(position);

    for (size_t iHumanInterfaceIdx = 0; iHumanInterfaceIdx < jointNames.size(); iHumanInterfaceIdx++)
    {
        std::string jointName = jointNames.at(iHumanInterfaceIdx);
        double jointVal = jointPositionsInterface.at(iHumanInterfaceIdx);

        iDynTree::JointIndex jointIndex = pImpl->viz.modelViz("human").model().getJointIndex(jointName);
        if (jointIndex != iDynTree::JOINT_INVALID_INDEX)
        {
            pImpl->joints.setVal(jointIndex, jointVal);
        }
    }

    // Update the visulizer
    yInfo() << LogPrefix << "Updating Visualizer for joints: " << pImpl->joints.size();
    pImpl->viz.modelViz("human").setPositions(pImpl->wHb, pImpl->joints);
    yInfo() << LogPrefix << "Ready to draw"; 
    
    pImpl->viz.draw();
    if (pImpl->viz.run())
    {
        pImpl->viz.draw();
    }
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
