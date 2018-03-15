#include "HumanRobotPose.h"

#include "TickTime.h"
#include "sensor_msgs_JointState.h"
#include "tf2_msgs_TFMessage.h"
#include "thrifts/HumanState.h"

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Network.h>
#include <yarp/os/Node.h>
#include <yarp/os/Property.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/RFModule.h>

#include <limits.h>
#include <vector>

const std::string LogPrefix = "human-robot-pose : ";
inline TickTime normalizeSecNSec(double yarpTimeStamp);
inline bool parseRotationMatrix(const yarp::os::Value& ini, iDynTree::Rotation& rotation);
inline bool parsePositionVector(const yarp::os::Value& ini, iDynTree::Position& position);
inline bool parseFrameListOption(const yarp::os::Value& option,
                                 std::vector<std::string>& parsedSegments);

class HumanRobotPose::impl
{
public:
    double period = 0.1;
    size_t counter = 0;
    yarp::os::Mutex mutex;
    bool autoconnect = false;

    // Human
    human::HumanState* humanStateData = nullptr;
    yarp::os::BufferedPort<human::HumanState> humanStatePort;
    std::string humanStateRemotePort;
    iDynTree::JointPosDoubleArray humanJointsPosition;
    iDynTree::Transform world_H_humanBase;
    iDynTree::JointPosDoubleArray humanJointsVelocity;
    iDynTree::KinDynComputations humanKinDynComp;

    // Robot
    bool enableRobot = false;
    iDynTree::JointPosDoubleArray robotJointsPosition;
    iDynTree::JointPosDoubleArray robotJointsVelocity;
    yarp::dev::IEncoders* iEncoders = nullptr;
    iDynTree::KinDynComputations robotKinDynComp;

    const double gravityRaw[3] = {0, 0, -9.81};
    const iDynTree::Vector3 gravity = {gravityRaw, 3};

    // Contacts
    typedef std::pair<std::string, std::string> Contact;
    Contact contact1;
    Contact contact2;
    enum
    {
        HUMAN = 0,
        ROBOT = 1
    };

    // Method #1, #2, #3 selection
    bool useSkin = false;
    bool useFixedTransform = false;

    // Method #2
    std::string fromHumanFrame;
    std::string toRobotFrame;
    iDynTree::Transform humanFrame_H_robotFrame;

    // ROS Publishers
    yarp::os::Publisher<sensor_msgs_JointState> publisher_jointState;
    yarp::os::Publisher<tf2_msgs_TFMessage> publisher_tfHuman;
    yarp::os::Publisher<tf2_msgs_TFMessage> publisher_tfRobot;

    // ROS Messages
    sensor_msgs_JointState jointState;
    tf2_msgs_TFMessage tfHuman;
    tf2_msgs_TFMessage tfRobot;

    // Methods for computing the robot pose
    iDynTree::Transform computeRobotPose_withSkin();
    iDynTree::Transform computeRobotPose_withFixedTransform();
    iDynTree::Transform computeRobotPose_withKnownContacts();
};

HumanRobotPose::HumanRobotPose()
    : pImpl{new HumanRobotPose::impl()}
{}

HumanRobotPose::~HumanRobotPose() {}

double HumanRobotPose::getPeriod()
{
    return pImpl->period;
}

bool HumanRobotPose::configure(yarp::os::ResourceFinder& rf)
{
    // ================
    // CHECK PARAMETERS
    // ================

    if (!rf.check("name")) {
        yError() << LogPrefix << "Module name is wrong or missing";
    }
    std::string moduleName = rf.find("name").asString();
    setName(moduleName.c_str());
    // Check that the name matches the module name in order to avoid
    // passing a wrong configuration file
    if (moduleName != "human-robot-pose") {
        yError() << "The moduleName parameter of the passed configuration is not human-robot-pose.";
        return false;
    }

    // MODULE PARAMETERS
    // =================

    if (!(rf.check("period") && rf.find("period").isInt())) {
        yError() << LogPrefix << "Parameter 'period' missing or invalid";
        return false;
    }

    if (!(rf.check("autoconnect") && rf.find("autoconnect").isBool())) {
        yError() << LogPrefix << "Parameter 'autoconnect' missing or invalid";
        return false;
    }

    // HUMAN PARAMETERS
    // ================

    if (!(rf.check("humanModel") && rf.find("humanModel").isString())) {
        yError() << LogPrefix << "Parameter 'humanModel' missing or invalid";
        return false;
    }

    if (!(rf.check("humanJointsListIni") && rf.find("humanJointsListIni").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointsListIni' missing or invalid";
        return false;
    }

    if (!(rf.check("humanStatePort") && rf.find("humanStatePort").isString())) {
        yError() << LogPrefix << "Parameter 'humanStatePort' missing or invalid";
        return false;
    }

    // ROBOT PARAMETERS
    // ================

    if (!(rf.check("enableRobot") && rf.find("enableRobot").isBool())) {
        yError() << LogPrefix << "Parameter 'enableRobot' missing or invalid";
        return false;
    }

    if (!(rf.check("robotName") && rf.find("robotName").isString())) {
        yError() << LogPrefix << "Parameter 'robotName' missing or invalid";
        return false;
    }
    if (!(rf.check("robotModel") && rf.find("robotModel").isString())) {
        yError() << LogPrefix << "Parameter 'robotModel' missing or invalid";
        return false;
    }

    //    if (!(rf.check("fixedFrameRobot") && rf.find("fixedFrameRobot").isString())) {
    //        yError() << LogPrefix << "Parameter 'fixedFrameRobot' missing or invalid";
    //    }

    // MODE #1 PARAMETERS (SKIN)
    // =========================

    if (!(rf.check("useSkin") && rf.find("useSkin").isBool())) {
        yError() << LogPrefix << "Parameter 'useSkin' missing or invalid";
        return false;
    }

    if (!(rf.check("skinManagerPort") && rf.find("skinManagerPort").isString())) {
        yError() << LogPrefix << "Parameter 'skinManagerPort' missing or invalid";
        return false;
    }

    // MODE #2 PARAMETERS (NO SKIN, HARDCODED TRANSFORM)
    // =================================================

    if (!(rf.check("useFixedTransform") && rf.find("useFixedTransform").isBool())) {
        yError() << LogPrefix << "Parameter 'useFixedTransform' missing or invalid";
        return false;
    }

    if (!(rf.check("fixedTransformPos") && rf.find("fixedTransformPos").isList())) {
        yError() << LogPrefix << "Parameter 'fixedTransformPos' missing or invalid";
        return false;
    }

    if (!(rf.check("fixedTransformRot") && rf.find("fixedTransformRot").isList())) {
        yError() << LogPrefix << "Parameter 'fixedTransformRot' missing or invalid";
        return false;
    }

    if (!(rf.check("fromHumanFrame") && rf.find("fromHumanFrame").isString())) {
        yError() << LogPrefix << "Parameter 'fromHumanFrame' missing or invalid";
        return false;
    }

    if (!(rf.check("toRobotFrame") && rf.find("toRobotFrame").isString())) {
        yError() << LogPrefix << "Parameter 'toRobotFrame' missing or invalid";
        return false;
    }

    // MODE #3 PARAMETERS (NO SKIN, KNOWN CONTACT FRAMES)
    // ==================================================

    if (!(rf.check("humanContactFrames") && rf.find("humanContactFrames").isList()
          && (rf.find("humanContactFrames").asList()->size() == 2))) {
        yError() << LogPrefix << "Parameter 'humanContactFrames' missing or invalid";
        return false;
    }

    if (!(rf.check("robotContactFrames") && rf.find("robotContactFrames").isList()
          && rf.find("robotContactFrames").asList()->size() == 2)) {
        yError() << LogPrefix << "Parameter 'robotContactFrames' missing or invalid";
        return false;
    }

    // ROS TOPICS
    // ==========

    if (!(rf.check("nodeName") && rf.find("nodeName").isString())) {
        yError() << LogPrefix << "Parameter 'nodeName' missing or invalid";
        return false;
    }

    if (!(rf.check("humanJointsTopic") && rf.find("humanJointsTopic").isString())) {
        yError() << LogPrefix << "Parameter 'humanJointsTopic' missing or invalid";
        return false;
    }

    if (!(rf.check("humanPoseTopic") && rf.find("humanPoseTopic").isString())) {
        yError() << LogPrefix << "Parameter 'humanPoseTopic' missing or invalid";
        return false;
    }

    if (!(rf.check("robotPoseTopic") && rf.find("robotPoseTopic").isString())) {
        yError() << LogPrefix << "Parameter 'robotPoseTopic' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    // MODULE PARAMETERS
    pImpl->period = rf.find("period").asInt() / 1000.0;
    pImpl->autoconnect = rf.find("autoconnect").asBool();

    // HUMAN PARAMETERS
    const std::string humanModel = rf.find("humanModel").asString();
    const std::string humanJointsListIni = rf.find("humanJointsListIni").asString();
    pImpl->humanStateRemotePort = rf.find("humanStatePort").asString();

    // ROBOT PARAMETERS
    pImpl->enableRobot = rf.find("enableRobot").asBool();
    const std::string robotName = rf.find("robotName").asString();
    const std::string robotModel = rf.find("robotModel").asString();
    //    const std::string fixedFrameRobot = rf.find("fixedFrameRobot").asString();

    // MODE #1 PARAMETERS
    pImpl->useSkin = rf.find("useSkin").asBool();
    if (pImpl->useSkin) {
        const std::string skinManagerPort = rf.find("skinManagerPort").asString();
    }

    // MODE #2 PARAMETERS
    pImpl->useFixedTransform = rf.find("useFixedTransform").asBool();
    if (pImpl->useFixedTransform) {
        iDynTree::Position pos;
        iDynTree::Rotation rot;
        if (!parsePositionVector(rf.find("fixedTransformPos"), pos)) {
            yError() << "Failed to parse fixedTransformPos parameter";
            return false;
        }
        if (!parseRotationMatrix(rf.find("fixedTransformRot"), rot)) {
            yError() << "Failed to parse fixedTransformRot parameter";
            return false;
        }
        pImpl->humanFrame_H_robotFrame.setPosition(pos);
        pImpl->humanFrame_H_robotFrame.setRotation(rot);

        pImpl->fromHumanFrame = rf.find("fromHumanFrame").asString();
        pImpl->toRobotFrame = rf.find("toRobotFrame").asString();
    }

    // MODE #3 PARAMETERS
    const yarp::os::Bottle* humanContactFrames = rf.find("humanContactFrames").asList();
    const yarp::os::Bottle* robotContactFrames = rf.find("robotContactFrames").asList();

    pImpl->contact1 = {humanContactFrames->get(0).asString(),
                       robotContactFrames->get(0).asString()};
    pImpl->contact2 = {humanContactFrames->get(1).asString(),
                       robotContactFrames->get(1).asString()};

    // ROS TOPICS
    const std::string nodeName = rf.find("nodeName").asString();
    const std::string humanJointsTopic = rf.find("humanJointsTopic").asString();
    const std::string humanPoseTopic = rf.find("humanPoseTopic").asString();
    const std::string robotPoseTopic = rf.find("robotPoseTopic").asString();

    // =====================
    // INITIALIZE YARP PORTS
    // =====================

    // Initialize the network
    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << LogPrefix << "YARP server wasn't found active";
        return false;
    }

    // Open ports
    const std::string humanStateInputPortName = "/" + getName() + "/humanState:i";
    if (!pImpl->humanStatePort.open(humanStateInputPortName)) {
        yError() << LogPrefix << "Failed to open port " << humanStateInputPortName;
        return false;
    }

    if (pImpl->autoconnect) {
        if (!yarp::os::Network::connect(pImpl->humanStateRemotePort, humanStateInputPortName)) {
            yError() << LogPrefix << "Failed to connect " << pImpl->humanStateRemotePort << " to "
                     << humanStateInputPortName;
            return false;
        }
    }

    // ==============================
    // INITIALIZE IENCODERS INTERFACE
    // ==============================

    const std::vector<std::string> robotControlledJoints = {
        // torso
        "torso_pitch",
        "torso_pitch",
        "torso_roll",
        "torso_yaw",
        // left arm
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_shoulder_yaw",
        "l_elbow",
        "l_wrist_prosup",
        "l_wrist_pitch",
        "l_wrist_yaw",
        // right arm
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_shoulder_yaw",
        "r_elbow",
        "r_wrist_prosup",
        "r_wrist_pitch",
        "r_wrist_yaw",
        // left leg
        "l_hip_pitch",
        "l_hip_roll",
        "l_hip_yaw",
        "l_knee",
        "l_ankle_pitch",
        "l_ankle_roll",
        // right leg
        "r_hip_pitch",
        "r_hip_roll",
        "r_hip_yaw",
        "r_knee",
        "r_ankle_pitch",
        "r_ankle_roll",
    };

    if (pImpl->enableRobot) {
        // Convert to Bottle
        yarp::os::Bottle axesNames;
        yarp::os::Bottle& axesList = axesNames.addList();
        for (const auto& axes : robotControlledJoints) {
            axesList.addString(axes);
        }

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
        const std::string cbPrefix = "/" + robotName + "/";
        remoteControlBoardsList.addString(cbPrefix + "torso");
        remoteControlBoardsList.addString(cbPrefix + "left_arm");
        remoteControlBoardsList.addString(cbPrefix + "right_arm");
        remoteControlBoardsList.addString(cbPrefix + "left_leg");
        remoteControlBoardsList.addString(cbPrefix + "right_leg");

        // Open the remotecontrolboardremapper device
        yarp::os::Property options;
        options.put("device", "remotecontrolboardremapper");
        options.put("axesNames", axesNames.get(0));
        options.put("remoteControlBoards", remoteControlBoards.get(0));
        options.put("localPortPrefix", "/" + getName());
        yarp::os::Property& remoteCBOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteCBOpts.put("writeStrict", "on");

        yarp::dev::PolyDriver remoteControlBoardRemapper;
        if (!remoteControlBoardRemapper.open(options) && !remoteControlBoardRemapper.isValid()) {
            yError() << LogPrefix
                     << "Failed to open the RemoteControlBoardRemapper with the options passed";
            return false;
        }

        // Access the interface
        if (!remoteControlBoardRemapper.view(pImpl->iEncoders)) {
            yError() << LogPrefix
                     << "Failed to get the iEncoders interface from the RemoteControlBoardRemapper";
            return false;
        }
    }

    // ======================
    // INITIALIZE HUMAN MODEL
    // ======================

    // Load the ini file containing the human joints
    std::string humanJointsListIniPath = rf.findFile(humanJointsListIni.c_str());
    if (humanJointsListIniPath.empty()) {
        yError() << LogPrefix << "ResourceFinder couldn't find ini file " + humanJointsListIni;
        return false;
    }

    yarp::os::Property config;
    if (!config.fromConfigFile(humanJointsListIniPath, /*wipe=*/true)) {
        yError() << LogPrefix << "Failed to read " << humanJointsListIniPath << " file";
        return false;
    }

    // Parse the ini file containing the human joints
    std::vector<std::string> humanJointList;
    if (!parseFrameListOption(config.find("jointList"), humanJointList)) {
        yError() << LogPrefix << "Failed parsing the joint list read from "
                 << humanJointsListIniPath;
        return false;
    }

    // Load the urdf
    std::string humanModelPath = rf.findFile(humanModel.c_str());
    if (humanModel.empty()) {
        yError() << LogPrefix << "ResourceFinder couldn't find urdf file " + humanModel;
        return false;
    }

    iDynTree::ModelLoader humanMdlLoader;
    if (!humanMdlLoader.loadReducedModelFromFile(humanModelPath, humanJointList)) {
        yError() << LogPrefix << "Failed to load reduced human model from file";
        return false;
    }

    pImpl->humanKinDynComp.loadRobotModel(humanMdlLoader.model());
    pImpl->humanKinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // ======================
    // INITIALIZE ROBOT MODEL
    // ======================

    if (pImpl->enableRobot) {
        std::string robotModelPath = rf.findFile(robotModel.c_str());
        if (robotModel.empty()) {
            yError() << LogPrefix << "ResourceFinder couldn't find urdf file " + robotModel;
            return false;
        }

        iDynTree::ModelLoader robotMdlLoader;
        if (!robotMdlLoader.loadReducedModelFromFile(robotModelPath, robotControlledJoints)) {
            yError() << LogPrefix << "Failed to load reduced robot model from file";
            return false;
        }

        pImpl->robotKinDynComp.loadRobotModel(robotMdlLoader.model());
        pImpl->robotKinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
    }

    // =========================
    // INITIALIZE ROS PUBLISHERS
    // =========================

    // Initialize the node
    yarp::os::Node node(nodeName);

    // Initialize human publishers and topics
    pImpl->publisher_tfHuman.topic(humanPoseTopic);
    pImpl->publisher_jointState.topic(humanJointsTopic);
    pImpl->tfHuman.transforms.resize(1);

    if (pImpl->enableRobot) {
        // Initialize human publishers and topics
        pImpl->publisher_tfRobot.topic(robotPoseTopic);
        pImpl->tfRobot.transforms.resize(1);
    }

    // ==========================
    // INITIALIZE OTHER RESOURCES
    // ==========================

    const unsigned humanDofs = pImpl->humanKinDynComp.getNrOfDegreesOfFreedom();
    pImpl->humanJointsPosition.reserve(humanDofs);
    pImpl->humanJointsVelocity.resize(humanDofs);
    pImpl->humanJointsVelocity.zero();

    if (pImpl->enableRobot) {
        const unsigned robotDofs = pImpl->robotKinDynComp.getNrOfDegreesOfFreedom();
        pImpl->robotJointsPosition.reserve(robotDofs);
        pImpl->robotJointsVelocity.resize(robotDofs);
        pImpl->robotJointsVelocity.zero();
    }

    return true;
}

bool HumanRobotPose::updateModule()
{
    // TODO
    if (!pImpl->autoconnect) {
        if (!yarp::os::Network::isConnected(pImpl->humanStateRemotePort,
                                            pImpl->humanStatePort.getName())) {
            yDebug() << "Ports " << pImpl->humanStateRemotePort << " and "
                     << pImpl->humanStatePort.getName()
                     << " are not connected. Waiting manual connection.";
            return true;
        }
        if (pImpl->enableRobot) {
        }
    }

    // ===================================================
    // READ AND UPDATE HUMAN JOINTS CONFIGURATION AND POSE
    // ===================================================

    pImpl->humanStateData = pImpl->humanStatePort.read(false);
    if (pImpl->humanStateData
        && !iDynTree::toiDynTree(pImpl->humanStateData->positions, pImpl->humanJointsPosition)) {
        yError() << LogPrefix << "Failed to read the human state data";
        return false;
    }

    if (pImpl->enableRobot) {
        pImpl->humanKinDynComp.setRobotState(
            pImpl->humanJointsPosition, pImpl->humanJointsVelocity, pImpl->gravity);
    }

    // Store the world_H_humanBase transform. It is required later.
    // Rotation
    iDynTree::Vector4 pelvisQuaternion;
    pelvisQuaternion(0) = pImpl->humanStateData->baseOrientationWRTGlobal.w;
    pelvisQuaternion(1) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.x;
    pelvisQuaternion(2) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.y;
    pelvisQuaternion(3) = pImpl->humanStateData->baseOrientationWRTGlobal.imaginary.z;
    pImpl->world_H_humanBase.setRotation(
        iDynTree::Rotation::RotationFromQuaternion(pelvisQuaternion));
    // Translation
    pImpl->world_H_humanBase.setPosition({
        pImpl->humanStateData->baseOriginWRTGlobal.x,
        pImpl->humanStateData->baseOriginWRTGlobal.y,
        pImpl->humanStateData->baseOriginWRTGlobal.z,
    });

    // ==========================================
    // READ AND UPDATE ROBOT JOINTS CONFIGURATION
    // ==========================================

    if (pImpl->enableRobot) {
        if (!pImpl->iEncoders) {
            yError() << LogPrefix << "iEncoders interface was not configured properly";
            return false;
        }

        if (!pImpl->iEncoders->getEncoders(pImpl->robotJointsPosition.data())) {
            yError() << LogPrefix << "Failed to read robot joints positions from the interface";
            return false;
        }

        pImpl->robotKinDynComp.setRobotState(
            pImpl->robotJointsPosition, pImpl->robotJointsVelocity, pImpl->gravity);
    }

    // =============================
    // COMPUTE FRAME TRANSFORMATIONS
    // =============================

    iDynTree::Transform robotPose;
    if (pImpl->enableRobot) {

        // Contacts detected with the skin
        if (pImpl->useSkin) {
            robotPose = pImpl->computeRobotPose_withSkin();
        }
        else {
            // Relative transform read from the configuration
            if (pImpl->useFixedTransform) {
                robotPose = pImpl->computeRobotPose_withFixedTransform();
            }
            // Estimate transformation but read contacts from the configuration
            else {
                robotPose = pImpl->computeRobotPose_withKnownContacts();
            }
        }
    }

    // ===================
    // WRITE TO ROS TOPICS
    // ===================

    // Prepare the publishers.
    // These objects are created with new every loop iteration. Instead of writing data directly
    // to them, which will cause reallocation, private pre-dimensioned variables will be used.
    tf2_msgs_TFMessage& tfMsgHuman = pImpl->publisher_tfHuman.prepare();
    sensor_msgs_JointState& jointStateMsg = pImpl->publisher_jointState.prepare();

    // Initialize common metadata
    pImpl->counter++;
    const TickTime currentTime = normalizeSecNSec(yarp::os::Time::now());

    // HUMAN JOINTS POSITION
    // =====================

    pImpl->jointState.header.stamp = currentTime;
    for (size_t index = 0; index < pImpl->jointState.position.size(); ++index) {
        pImpl->jointState.position[index] = pImpl->humanStateData->positions[index];
    }

    jointStateMsg = pImpl->jointState;
    pImpl->publisher_jointState.write();

    // HUMAN TRANSFORM world_HH_base
    // =============================

    auto humanTransform = pImpl->tfHuman.transforms.front();
    const auto humanStateData = pImpl->humanStateData;

    // Metadata
    humanTransform.header.seq = pImpl->counter;
    humanTransform.header.stamp = currentTime;
    humanTransform.header.frame_id = "ground";
    humanTransform.child_frame_id = "Pelvis";

    // Translation
    humanTransform.transform.translation.x = humanStateData->baseOriginWRTGlobal.x;
    humanTransform.transform.translation.y = humanStateData->baseOriginWRTGlobal.y;
    humanTransform.transform.translation.z = humanStateData->baseOriginWRTGlobal.z;

    // Rotation
    humanTransform.transform.rotation.x = humanStateData->baseOrientationWRTGlobal.imaginary.x;
    humanTransform.transform.rotation.y = humanStateData->baseOrientationWRTGlobal.imaginary.y;
    humanTransform.transform.rotation.z = humanStateData->baseOrientationWRTGlobal.imaginary.z;
    humanTransform.transform.rotation.w = humanStateData->baseOrientationWRTGlobal.w;

    // Publish the transform
    tfMsgHuman = pImpl->tfHuman;
    pImpl->publisher_tfHuman.write();

    // ROBOT TRANSFORM world_HR_base
    // =============================

    if (pImpl->enableRobot) {
        tf2_msgs_TFMessage& tfMsgRobot = pImpl->publisher_tfHuman.prepare();

        iDynTree::Position world_P_robotBase = robotPose.getPosition();
        iDynTree::Vector4 world_R_robotBase = robotPose.getRotation().asQuaternion();

        auto robotTransform = pImpl->tfRobot.transforms.front();

        // Metadata
        robotTransform.header.seq = pImpl->counter;
        robotTransform.header.stamp = currentTime;
        robotTransform.header.frame_id = "ground";
        robotTransform.child_frame_id = "root_link";

        // Translation
        robotTransform.transform.translation.x = world_P_robotBase(0);
        robotTransform.transform.translation.y = world_P_robotBase(1);
        robotTransform.transform.translation.z = world_P_robotBase(2);

        // Rotation
        robotTransform.transform.rotation.x = world_R_robotBase(1);
        robotTransform.transform.rotation.y = world_R_robotBase(2);
        robotTransform.transform.rotation.z = world_R_robotBase(3);
        robotTransform.transform.rotation.w = world_R_robotBase(0);

        // Publish the transform
        tfMsgRobot = pImpl->tfRobot;
        pImpl->publisher_tfRobot.write();
    }

    return true;
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withSkin()
{
    // TODO
    return {};
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withFixedTransform()
{
    iDynTree::Transform humanPelvis_H_humanFrame;
    iDynTree::Transform robotFrame_H_robotBase;

    humanPelvis_H_humanFrame = humanKinDynComp.getRelativeTransform("Pelvis", fromHumanFrame);
    robotFrame_H_robotBase = robotKinDynComp.getRelativeTransform(toRobotFrame, "root_link");

    return world_H_humanBase * humanPelvis_H_humanFrame * humanFrame_H_robotFrame
           * robotFrame_H_robotBase;
}

iDynTree::Transform HumanRobotPose::impl::computeRobotPose_withKnownContacts()
{
    // TODO
    return {};
}

bool HumanRobotPose::close()
{
    // Close yarp ports
    pImpl->humanStatePort.close();

    // Stop ROS publishers
    pImpl->publisher_tfRobot.interrupt();
    pImpl->publisher_tfRobot.close();
    pImpl->publisher_tfHuman.interrupt();
    pImpl->publisher_tfHuman.close();
    pImpl->publisher_jointState.interrupt();
    pImpl->publisher_jointState.close();

    return true;
}

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > UINT_MAX) {
        yWarning() << LogPrefix
                   << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec = sec_part;
    ret.nsec = nsec_part;
    return ret;
}

inline bool parseRotationMatrix(const yarp::os::Value& ini, iDynTree::Rotation& rotation)
{
    if (ini.isNull() || !ini.isList()) {
        return false;
    }
    yarp::os::Bottle* outerList = ini.asList();
    if (!outerList || outerList->size() != 3) {
        return false;
    }
    for (int row = 0; row < outerList->size(); ++row) {
        yarp::os::Value& innerValue = outerList->get(row);
        if (innerValue.isNull() || !innerValue.isList()) {
            return false;
        }
        yarp::os::Bottle* innerList = innerValue.asList();
        if (!innerList || innerList->size() != 3) {
            return false;
        }
        for (int column = 0; column < innerList->size(); ++column) {
            rotation.setVal(row, column, innerList->get(column).asDouble());
        }
    }
    return true;
}

inline bool parsePositionVector(const yarp::os::Value& ini, iDynTree::Position& position)
{
    if (ini.isNull() || !ini.isList()) {
        return false;
    }
    yarp::os::Bottle* list = ini.asList();
    if (!list || list->size() != 3) {
        return false;
    }
    for (int i = 0; i < list->size(); ++i) {
        position.setVal(i, list->get(i).asDouble());
    }
    return true;
}

inline bool parseFrameListOption(const yarp::os::Value& option,
                                 std::vector<std::string>& parsedSegments)
{
    if (option.isNull() || !option.isList() || !option.asList())
        return false;
    yarp::os::Bottle* frames = option.asList();
    parsedSegments.reserve(static_cast<size_t>(frames->size()));

    for (int i = 0; i < frames->size(); ++i) {
        if (frames->get(i).isString()) {
            parsedSegments.push_back(frames->get(i).asString());
        }
    }
    return true;
}
