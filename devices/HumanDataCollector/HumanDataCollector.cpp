/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "HumanDataCollector.h"
#include "IHumanState.h"
#include "IHumanWrench.h"
#include "IHumanDynamics.h"
#include "Utils.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "matio.h"

// TODO: Double check if the header works on WINDOWS
#include <experimental/filesystem>
#include <sys/stat.h>

#include <chrono>
#include <ratio>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <functional>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>

const std::string DeviceName = "HumanDataCollector";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

// Inline functions for matio variables destructor calls
auto matVarFree = [](matvar_t* matvar) -> void {Mat_VarFree(matvar);};

auto matFileClose = [](mat_t* mat) -> void {Mat_Close(mat);
                                            yInfo() << LogPrefix << "Closed mat file";};

// Unique pointer wrapping of matio variables
template <typename T>
using MatVarPtr = std::unique_ptr<T, std::function<void(T*)>>;

void writeVectorOfStringToMatCell(const std::string& name, const std::vector<std::string>& strings, mat_t* matFile)
{
    if (strings.empty()) {
        yError() << LogPrefix << "Passed string is empty";
        return;
    }

    size_t dims[2] = {strings.size(), 1};
  
    MatVarPtr<matvar_t> matCellArray(Mat_VarCreate(name.c_str(), MAT_C_CELL, MAT_T_CELL, 2, dims, nullptr, 0), matVarFree);

    if (matCellArray == nullptr) {
        yError() << LogPrefix << "Failed to create mat cell array " << name;
        return;
    }

    for(size_t i = 0 ; i < strings.size(); i++)
    {
        std::string stringToWrite = strings[i];

        size_t stringDim[2] = { 1, stringToWrite.size() };
        matvar_t *matCellElement = Mat_VarCreate("dummy", MAT_C_CHAR, MAT_T_UTF8, 2, stringDim, const_cast<char *>(stringToWrite.c_str()), 0);

        if (matCellElement == nullptr) {
            yError() << LogPrefix << "Failed to create mat cell element";
            Mat_VarFree(matCellElement);
            return;
        }

        Mat_VarSetCell(matCellArray.get(), i, matCellElement);
    }

    Mat_VarWrite(matFile, matCellArray.get(), MAT_COMPRESSION_NONE);
}

void writeVectorOfVectorOfVectorDoubleToMatStruct(const std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::vector<double>>>>& data, std::string dataName, std::vector<std::string>& linkNames, mat_t* matFile) {

    const char *mainStructFieldName[1] = {"data"};
    size_t matDataStructDims[2] = { 1, data.size() };
    MatVarPtr<matvar_t> matDataStruct(Mat_VarCreateStruct(dataName.data(), 2, matDataStructDims, mainStructFieldName, 1), matVarFree);

    // Set sub fields struct
    std::vector<std::string> matSubStructFieldNamesVec;

    auto pair = data.begin();
    for (auto& element : pair->second) {
        matSubStructFieldNamesVec.push_back(element.first);
    }

    // Construct vector of character pointers to field names
    std::vector<const char*> subFieldNames;
    for (size_t i = 0; i < matSubStructFieldNamesVec.size(); i++) {
        subFieldNames.push_back(matSubStructFieldNamesVec[i].c_str());

    }

    // Iterate over human data and set to numeric arrays of mat struct
    int index = 0;
    for (std::string linkName : linkNames) {

        std::unordered_map<std::string, std::vector<std::vector<double>>> linkData = data.at(linkName);

        size_t matSubDataStructDims[2] = { 1, 1 };
        matvar_t* matSubDataStruct = Mat_VarCreateStruct(mainStructFieldName[0], 2, matSubDataStructDims, subFieldNames.data(), matSubStructFieldNamesVec.size()); // NOTE: Using unique ptr for this struct is causing seg fault

        Mat_VarSetStructFieldByName(matDataStruct.get(), mainStructFieldName[0], index, matSubDataStruct);

        for (const std::pair<std::string, std::vector<std::vector<double>>> subPair : linkData) {


            const size_t rows = subPair.second.size();
            const size_t cols = subPair.second.at(0).size(); // Assuming same number of elements in the vector

            std::vector<double> values;

            // TODO: Check if this can be improved through STL
            for (size_t r = 0; r < rows; r++) {
                for (size_t c = 0; c < cols; c++) {
                    values.push_back(subPair.second[r].at(c));
                }
            }

            // Create a mat variable for 2d numeric array
            // NOTE: rows and cols are reversed from the regular array because mat variables are column major
            size_t dims[2] = {cols, rows};
            matvar_t *mat2darray = Mat_VarCreate(subPair.first.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, values.data(), 0);

            if (mat2darray == nullptr) {
                yError() << LogPrefix << "Failed to create a 2d numeric array to store in the mat sub struct variable";
                Mat_VarFree(mat2darray);
                return;
            }

            // Set the 2d numeric array to the mat struct variable
            Mat_VarSetStructFieldByName(matSubDataStruct, subPair.first.c_str(), 0, mat2darray);


        }

        index++;
    }

    // Write mat struct to mat file before closing
    Mat_VarWrite(matFile, matDataStruct.get(), MAT_COMPRESSION_NONE);

    yDebug() << LogPrefix << "Finished writing " << dataName << " mat struct to the mat file";
}

void writeVectorOfVectorDoubleToMatStruct(const std::unordered_map<std::string, std::vector<std::vector<double>>>& humanData, std::string dataName, mat_t* matFile) {

    // Set the mat struct field names
    std::vector<std::string> matStructFieldNamesVec;

    for (const std::pair<std::string, std::vector<std::vector<double>>> pair : humanData) {
        matStructFieldNamesVec.push_back(pair.first);
    }

    // Construct vector of character pointers to field names
    std::vector<const char*> fieldNames;
    for (size_t i = 0; i < matStructFieldNamesVec.size(); i++) {
        fieldNames.push_back(matStructFieldNamesVec[i].c_str());
    }

    // Set mat struct dimensions
    size_t matDataStructDims[2] = {1, 1};

    // Initialize mat struct variable
    MatVarPtr<matvar_t> matDataStruct(Mat_VarCreateStruct(dataName.data(), 1, matDataStructDims, fieldNames.data(), matStructFieldNamesVec.size()), matVarFree);

    if (matDataStruct == nullptr) {
        yError() << LogPrefix << "Failed to initialize matio struct variable";
        return;
    }

    // Iterate over human data and set to numeric arrays of mat struct
    for (const std::pair<std::string, std::vector<std::vector<double>>> pair : humanData) {

        const size_t rows = pair.second.size();
        const size_t cols = pair.second.at(0).size(); // Assuming same number of elements in the vector

        std::vector<double> values;

        // TODO: Check if this can be improved through STL
        for (size_t r = 0; r < rows; r++) {
            for (size_t c = 0; c < cols; c++) {
                values.push_back(pair.second[r].at(c));
            }
        }

        // Create a mat variable for 2d numeric array
        // NOTE: rows and cols are reversed from the regular array because mat variables are column major
        size_t dims[2] = {cols, rows};
        matvar_t *mat2darray = Mat_VarCreate(pair.first.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, values.data(), 0);

        if (mat2darray == nullptr) {
            yError() << LogPrefix << "Failed to create a 2d numeric array to store in the mat struct variable";
            Mat_VarFree(mat2darray);
            return;
        }

        // Set the 2d numeric array to the mat struct variable
        Mat_VarSetStructFieldByName(matDataStruct.get(), pair.first.c_str(), 0, mat2darray);

    }

    // Write mat struct to mat file before closing
    Mat_VarWrite(matFile, matDataStruct.get(), MAT_COMPRESSION_NONE);

    yDebug() << LogPrefix << "Finished writing " << dataName << " mat struct to the mat file";

}


class HumanDataCollector::impl
{
public:

    std::string portPrefix;
    bool firstData = true;

    hde::interfaces::IHumanState* iHumanState = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchMeasurements = nullptr;
    hde::interfaces::IHumanWrench* iHumanWrenchEstimates = nullptr; //This interface points to HumanDynamicsEstimator, which gives offset removed wrench measurements and the estimates
    hde::interfaces::IHumanDynamics* iHumanDynamics = nullptr;

    struct {
        bool stateProvider = false;
        bool wrenchProvider = false;
        bool dynamicsEstimator = false;
    } isAttached;

    // Human data buffer structs
    struct {

        std::vector<int64_t> time;
        std::vector<std::string> stateJointNames;
        std::vector<std::string> linkNames;
        std::vector<std::string>  wrenchMeasurementSourceNames;
        std::vector<std::string> wrenchEstimateSourceNames;
        std::vector<std::string> dynamicsJointNames;
        std::unordered_map<std::string, std::vector<std::vector<double>>> data;
        std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::vector<double>>>> linkData;

    } humanDataStruct;

    // Interface data buffers

    // Base Quantities
    std::string baseName;
    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;
    std::array<double, 7> basePoseArray;
    std::vector<double> basePoseVec;

    std::array<double, 6> baseVelocity;
    std::vector<double> baseVeclocityVec;

    // Joint Quantities
    size_t stateNumberOfJoints;
    std::vector<std::string> stateJointNames;
    std::vector<double> jointPositionsVec;
    std::vector<double> jointVelocitiesVec;

    // CoM Quantities
    std::array<double, 3> comPosition;
    std::vector<double> comPositionVec;
    std::array<double, 3> comVelocity;
    std::vector<double> comVelocityVec;
    std::array<double, 3> comBiasAcceleration;
    std::vector<double> comBiasAccelerationVec;

    // Rate of change of momentum quantities
    std::array<double, 6> rateOfChangeOfMomentumInBaseFrame;
    std::array<double, 6> rateOfChangeOfMomentumInWorldFrame;
    std::array<double, 6> rateOfChangeOfMomentumInCentroidalFrame;

    std::vector<double> rateOfChangeOfMomentumInBaseFrameVec;
    std::vector<double> rateOfChangeOfMomentumInWorldFrameVec;
    std::vector<double> rateOfChangeOfMomentumInCentroidalFrameVec;

    // Link quantities
    size_t numberOfLinks;
    std::vector<std::string> linkNames;
    std::unordered_map<std::string, iDynTree::Transform> linkTransformsMap;
    std::unordered_map<std::string, iDynTree::Twist> linkVelocitiesMap;
    std::unordered_map<std::string, iDynTree::SpatialAcc> linkAccelerationsMap;

    // Wrench Measurements
    size_t numberOfWrenchMeasurementSources;
    std::vector<std::string> wrenchMeasurementSourceNames;
    std::vector<double> wrenchMeasurementValuesVec;

    std::vector<double> task1_wrenchMeasurementsInLinkFrameVec;
    std::vector<double> task1_wrenchMeasurementsInCentroidalFrameVec;
    std::vector<double> task1_wrenchMeasurementsInBaseFrameVec;
    std::vector<double> task1_wrenchMeasurementsInWorldFrameVec;

    std::vector<double> task2_wrenchMeasurementsInLinkFrameVec;
    std::vector<double> task2_wrenchMeasurementsInCentroidalFrameVec;
    std::vector<double> task2_wrenchMeasurementsInBaseFrameVec;
    std::vector<double> task2_wrenchMeasurementsInWorldFrameVec;

    // Wrench Estimates
    size_t numberOfWrenchEstimateSources;
    std::vector<std::string> wrenchEstimateSourceNames;
    std::vector<double> wrenchEstimateValuesVec;

    std::vector<double> task1_wrenchEstimatesInLinkFrameVec;
    std::vector<double> task1_wrenchEstimatesInCentroidalFrameVec;
    std::vector<double> task1_wrenchEstimatesInBaseFrameVec;
    std::vector<double> task1_wrenchEstimatesInWorldFrameVec;

    std::vector<double> task2_wrenchEstimatesInLinkFrameVec;
    std::vector<double> task2_wrenchEstimatesInCentroidalFrameVec;
    std::vector<double> task2_wrenchEstimatesInBaseFrameVec;
    std::vector<double> task2_wrenchEstimatesInWorldFrameVec;

    // Joint Torques
    size_t dynamicsNumberOfJoints;
    std::vector<std::string> dynamicsJointNames;
    std::vector<double> jointTorquesVec;
    std::vector<std::array<double,6>> internalWrenches;

    // Yarp ports for streaming data from IHumanState interface of HumanStateProvider
    yarp::os::BufferedPort<yarp::sig::Vector> basePoseDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> baseVelocityDataPort;
    yarp::os::BufferedPort<yarp::os::Bottle> stateJointNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointPositionsDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointVelocitiesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comPositionDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comVelocityDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> comBiasAccelerationDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rateOfChangeOfMomentumInBaseFrameDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rateOfChangeOfMomentumInWorldFrameDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rateOfChangeOfMomentumInCentroidalFrameDataPort;

    // Yarp ports for streaming data from IHumanWrench interface of HumanWrenchProvider
    yarp::os::BufferedPort<yarp::os::Bottle> wrenchMeasurementSourceNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> wrenchMeasurementValuesDataPort;

    // Yarp ports for streaming data from IHumanWrench interfce and IHumanDynamics interface of HumanDynamicsEstimator
    yarp::os::BufferedPort<yarp::os::Bottle> wrenchEstimateSourceNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> wrenchEstimateValuesDataPort;

    yarp::os::BufferedPort<yarp::os::Bottle> dynamicsJointNamesDataPort;
    yarp::os::BufferedPort<yarp::sig::Vector> jointTorquesDataPort;

    // TODO: Stream link quantities to yarp ports

    // MATIO Logging
    bool matioLogger = false;
    const std::experimental::filesystem::path originalWorkingDirectory = std::experimental::filesystem::current_path();
    std::string matLogDirectory = "";
    std::string matLogFileName; //TODO: Improve file handling for multiple file logging
    MatVarPtr<mat_t> matFilePtr = nullptr;

    // Time Buffers
    std::chrono::system_clock::time_point startTime;
    std::chrono::system_clock::time_point currentTime;
};

HumanDataCollector::HumanDataCollector()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanDataCollector::~HumanDataCollector() = default;

bool HumanDataCollector::open(yarp::os::Searchable &config) {

    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period";
    }

    if (!(config.check("portPrefix") && config.find("portPrefix").isString())) {
        yInfo() << LogPrefix << "Using default portPrefix HDE ";
    }

    if (!(config.check("matLogFileName") && config.find("matLogFileName").isString())) {
        yInfo() << LogPrefix << "Using default file name matLogFile.mat";
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    const double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    pImpl->portPrefix = "/" + config.check("portPrefix", yarp::os::Value("HDE")).asString() + "/";
    pImpl->matioLogger = config.check("matioLogger", yarp::os::Value(false)).asBool();

    if (pImpl->matioLogger) {
        pImpl->matLogDirectory =  pImpl->originalWorkingDirectory.string() + "/" +
                                  config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString() + "/";

        struct stat info;

        // check if matLogDirectory exists
        if (stat(pImpl->matLogDirectory.c_str(), &info) != 0 && mkdir(config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString().c_str(), 0777) == -1) {
            yError() << LogPrefix << "Failed to created the matLogDirectory " << config.check("matLogDirectory", yarp::os::Value("matLogDirectory")).asString();
            return false;
        }

        std::experimental::filesystem::current_path(pImpl->matLogDirectory);
    }

    // Get matLogFileName value from config
    pImpl->matLogFileName = config.check("matLogFileName", yarp::os::Value("matLogFile")).asString() + ".mat";

    //TODO: Define rpc to reset the data dump or restarting the visualization??
    // Get matLogFileName value from config
    pImpl->matLogFileName = config.check("matLogFileName", yarp::os::Value("matLogFile")).asString() + ".mat";

    yInfo() << LogPrefix << "*** ===========================";
    yInfo() << LogPrefix << "*** Period                    :" << period;
    yInfo() << LogPrefix << "*** portPrefix                :" << pImpl->portPrefix;
    yInfo() << LogPrefix << "*** matioLogger               :" << pImpl->matioLogger;
    yInfo() << LogPrefix << "*** matLogDirectory           :" << pImpl->matLogDirectory;
    yInfo() << LogPrefix << "*** matLogFileName            :" << pImpl->matLogFileName;
    yInfo() << LogPrefix << "*** ===========================";

    // Set periodicThread period
    this->setPeriod(period);

    return true;
}

bool HumanDataCollector::close() {
    return true;
}

void HumanDataCollector::run()
{
    if (pImpl->firstData) {

        pImpl->firstData = false;

        // =====================================================================
        // Open yarp ports base on the attached devices for streaming human data
        // =====================================================================

        if (pImpl->isAttached.stateProvider) {

            // Open base pose data port
            const std::string basePosePortName = pImpl->portPrefix + DeviceName + "/basePose:o";
            if (!pImpl->basePoseDataPort.open(basePosePortName)) {
                yError() << LogPrefix << "Failed to open port " << basePosePortName;
                askToStop();
                return;
            }

            // Open base velocity data port
            const std::string baseVelocityPortName = pImpl->portPrefix + DeviceName + "/baseVelocity:o";
            if (!pImpl->baseVelocityDataPort.open(baseVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << baseVelocityPortName;
                askToStop();
                return;
            }

            // Open state jonit names data port
            const std::string stateJointNamesPortName = pImpl->portPrefix + DeviceName + "/stateJointNames:o";
            if (!pImpl->stateJointNamesDataPort.open(stateJointNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << stateJointNamesPortName;
                askToStop();
                return;
            }

            // Open joint positions data port
            const std::string jointPositionsPortName = pImpl->portPrefix + DeviceName + "/jointPositions:o";
            if (!pImpl->jointPositionsDataPort.open(jointPositionsPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointPositionsPortName;
                askToStop();
                return;
            }

            // Open joint velocities data port
            const std::string jointVelocitiesPortName = pImpl->portPrefix + DeviceName + "/jointVelocities:o";
            if (!pImpl->jointVelocitiesDataPort.open(jointVelocitiesPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointVelocitiesPortName;
                askToStop();
                return;
            }

            // Open CoM position data port
            const std::string comPositionPortName = pImpl->portPrefix + DeviceName + "/comPosition:o";
            if (!pImpl->comPositionDataPort.open(comPositionPortName)) {
                yError() << LogPrefix << "Failed to open port " << comPositionPortName;
                askToStop();
                return;
            }

            // Open CoM velocity data port
            const std::string comVelocityPortName = pImpl->portPrefix + DeviceName + "/comVelocity:o";
            if (!pImpl->comVelocityDataPort.open(comVelocityPortName)) {
                yError() << LogPrefix << "Failed to open port " << comVelocityPortName;
                askToStop();
                return;
            }

            // Open CoM bias acceleration data port
            const std::string comBiasAccelerationPortName = pImpl->portPrefix + DeviceName + "/comBiasAcceleration:o";
            if (!pImpl->comBiasAccelerationDataPort.open(comBiasAccelerationPortName)) {
                yError() << LogPrefix << "Failed to open port " << comBiasAccelerationPortName;
                askToStop();
                return;
            }

            // Open rate of change of momentum in base frame data port
            const std::string rateOfChangeOfMomentumInBaseFramePortName = pImpl->portPrefix + DeviceName + "/rateOfChangeOfMomentumInBaseFrame:o";
            if (!pImpl->rateOfChangeOfMomentumInBaseFrameDataPort.open(rateOfChangeOfMomentumInBaseFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << rateOfChangeOfMomentumInBaseFramePortName;
                askToStop();
                return;
            }

            // Open rate of change of momentum in world frame data port
            const std::string rateOfChangeOfMomentumInWorldFramePortName = pImpl->portPrefix + DeviceName + "/rateOfChangeOfMomentumInWorldFrame:o";
            if (!pImpl->rateOfChangeOfMomentumInWorldFrameDataPort.open(rateOfChangeOfMomentumInWorldFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << rateOfChangeOfMomentumInWorldFramePortName;
                askToStop();
                return;
            }

            // Open rate of change of momentum in centroidal frame data port
            const std::string rateOfChangeOfMomentumInCentroidalFramePortName = pImpl->portPrefix + DeviceName + "/rateOfChangeOfMomentumInCentroidalFrame:o";
            if (!pImpl->rateOfChangeOfMomentumInCentroidalFrameDataPort.open(rateOfChangeOfMomentumInCentroidalFramePortName)) {
                yError() << LogPrefix << "Failed to open port " << rateOfChangeOfMomentumInCentroidalFramePortName;
                askToStop();
                return;
            }

        }

        if (pImpl->isAttached.wrenchProvider) {

            // Open wrench measurements source names data port
            const std::string wrenchMeasurementsSourceNamesPortName = pImpl->portPrefix + DeviceName + "/wrenchMeasurementSourceNames:o";
            if (!pImpl->wrenchMeasurementSourceNamesDataPort.open(wrenchMeasurementsSourceNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchMeasurementsSourceNamesPortName;
                askToStop();
                return;
            }

            // Open wrench measurement values data port
            const std::string wrenchMeausurementsDataPortName = pImpl->portPrefix + DeviceName + "/wrenchMeasurements:o";
            if (!pImpl->wrenchMeasurementValuesDataPort.open(wrenchMeausurementsDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchMeausurementsDataPortName;
                askToStop();
                return;
            }

        }

        if (pImpl->isAttached.dynamicsEstimator) {

            // Open wrench estimates source names data port
            const std::string wrenchEstimatesSourceNamesPortName = pImpl->portPrefix + DeviceName + "/wrenchEstimateSourceNames:o";
            if (!pImpl->wrenchEstimateSourceNamesDataPort.open(wrenchEstimatesSourceNamesPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchEstimatesSourceNamesPortName;
                askToStop();
                return;
            }

            // Open wrench estimate values data port
            const std::string wrenchEstimatesDataPortName = pImpl->portPrefix + DeviceName + "/wrenchEstimates:o";
            if (!pImpl->wrenchEstimateValuesDataPort.open(wrenchEstimatesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << wrenchEstimatesDataPortName;
                askToStop();
                return;
            }

            // Open dynamics joint names data port
            const std::string dynamicsJointNamesDataPortName = pImpl->portPrefix + DeviceName + "/dynamicsJointNames:o";
            if (!pImpl->dynamicsJointNamesDataPort.open(dynamicsJointNamesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << dynamicsJointNamesDataPortName;
                askToStop();
                return;
            }

            // Open joint torques data port
            const std::string jointTorquesDataPortName = pImpl->portPrefix + DeviceName + "/jointTorques:o";
            if (!pImpl->jointTorquesDataPort.open(jointTorquesDataPortName)) {
                yError() << LogPrefix << "Failed to open port " << jointTorquesDataPortName;
                askToStop();
                return;
            }

        }

        // Initialize  matio logging
        if (pImpl->matioLogger) {

            // Get the start time
            pImpl->startTime = std::chrono::system_clock::now();

            // TODO: Handle multiple files
            std::string matLogFileFullPathName = pImpl->matLogDirectory + pImpl->matLogFileName;
            pImpl->matFilePtr = {Mat_CreateVer(matLogFileFullPathName.c_str(), nullptr, MAT_FT_MAT73), matFileClose};

            if (pImpl->matFilePtr == nullptr) {
                yError() << LogPrefix << "Failed to create " << pImpl->matLogFileName << " MAT file for logging";
                askToStop();
                return;
            }

            // Initialize time vector
            pImpl->humanDataStruct.time.clear();
            pImpl->humanDataStruct.data["time"] = std::vector<std::vector<double>>();

            // Initialize human data struct buffers
            if (pImpl->isAttached.stateProvider) {

                pImpl->humanDataStruct.stateJointNames.clear();
                pImpl->humanDataStruct.data["basePose"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["baseVelocity"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["comPosition"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["comVelocity"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["rateOfChangeOfMomentumInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["rateOfChangeOfMomentumInWorldFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["rateOfChangeOfMomentumInCentroidalFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["jointPositions"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["jointVelocities"] = std::vector<std::vector<double>>();
            }

            if (pImpl->isAttached.wrenchProvider) {

                pImpl->humanDataStruct.wrenchEstimateSourceNames.clear();
                pImpl->humanDataStruct.data["wrenchMeasurements"] = std::vector<std::vector<double>>();
            }

            if (pImpl->isAttached.dynamicsEstimator) {

                pImpl->humanDataStruct.wrenchEstimateSourceNames.clear();
                pImpl->humanDataStruct.data["wrenchEstimates"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["task1_wrenchMeasurementsInLinkFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchMeasurementsInCentroidalFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchMeasurementsInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchMeasurementsInWorldFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["task1_wrenchEstimatesInLinkFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchEstimatesInCentroidalFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchEstimatesInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task1_wrenchEstimatesInWorldFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["task2_wrenchMeasurementsInLinkFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchMeasurementsInCentroidalFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchMeasurementsInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchMeasurementsInWorldFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.data["task2_wrenchEstimatesInLinkFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchEstimatesInCentroidalFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchEstimatesInBaseFrame"] = std::vector<std::vector<double>>();
                pImpl->humanDataStruct.data["task2_wrenchEstimatesInWorldFrame"] = std::vector<std::vector<double>>();

                pImpl->humanDataStruct.dynamicsJointNames.clear();
                pImpl->humanDataStruct.data["jointTorques"] = std::vector<std::vector<double>>();
            }

        }

    }


    // =========================================================
    // Get data from different interfaces from attached devices
    // =========================================================

    if (pImpl->matioLogger) {

        // Get the current time
        pImpl->currentTime = std::chrono::system_clock::now();

        // Get the duration from start time to current time
        std::chrono::duration<int64_t, std::nano> timeDuration = pImpl->currentTime - pImpl->startTime;

        // Push back time duration to a vector
        pImpl->humanDataStruct.time.push_back(timeDuration.count());

    }

    // Get data from IHumanState interface of HumanStateProvider
    if (pImpl->isAttached.stateProvider) {

        // Base Quantities
        pImpl->baseName = pImpl->iHumanState->getBaseName();
        pImpl->basePosition = pImpl->iHumanState->getBasePosition();
        pImpl->baseOrientation = pImpl->iHumanState->getBaseOrientation();
        pImpl->baseVelocity = pImpl->iHumanState->getBaseVelocity();

        // Joint Quantities
        pImpl->stateNumberOfJoints = pImpl->iHumanState->getNumberOfJoints();
        pImpl->stateJointNames = pImpl->iHumanState->getJointNames();
        pImpl->jointPositionsVec = pImpl->iHumanState->getJointPositions();
        pImpl->jointVelocitiesVec = pImpl->iHumanState->getJointVelocities();

        // CoM Quantities
        pImpl->comPosition         = pImpl->iHumanState->getCoMPosition();
        pImpl->comVelocity         = pImpl->iHumanState->getCoMVelocity();
        pImpl->comBiasAcceleration = pImpl->iHumanState->getCoMBiasAcceleration();

        // Rate of change of momentum Quantities
        pImpl->rateOfChangeOfMomentumInBaseFrame = pImpl->iHumanState->getRateOfChangeOfMomentumInBaseFrame();
        pImpl->rateOfChangeOfMomentumInWorldFrame = pImpl->iHumanState->getRateOfChangeOfMomentumInWorldFrame();
        pImpl->rateOfChangeOfMomentumInCentroidalFrame = pImpl->iHumanState->getRateOfChangeOfMomentumInCentroidalFrame();

        // Link Quantities
        pImpl->numberOfLinks = pImpl->iHumanState->getNumberOfLinks();
        pImpl->linkNames = pImpl->iHumanState->getLinkNames();
        pImpl->linkTransformsMap = pImpl->iHumanState->getLinkTransformMeasurements();
        pImpl->linkVelocitiesMap = pImpl->iHumanState->getLinkVelocityMeasurements();
        pImpl->linkAccelerationsMap = pImpl->iHumanState->getLinkAccelerationMeasurements();

    }

    // Get data from IHumanWrench interface of HumanWrenchProvider
    if (pImpl->isAttached.wrenchProvider) {

        pImpl->numberOfWrenchMeasurementSources = pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources();
        pImpl->wrenchMeasurementSourceNames = pImpl->iHumanWrenchMeasurements->getWrenchSourceNames();
        pImpl->wrenchMeasurementValuesVec = pImpl->iHumanWrenchMeasurements->getWrenches();

    }    

    if (pImpl->isAttached.dynamicsEstimator) {

        // Get data from IHumanWrench interface of HumanDynamicsEstimator
        pImpl->numberOfWrenchEstimateSources = pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources();
        pImpl->wrenchEstimateSourceNames = pImpl->iHumanWrenchEstimates->getWrenchSourceNames();
        pImpl->wrenchEstimateValuesVec = pImpl->iHumanWrenchEstimates->getWrenches();

        pImpl->task1_wrenchMeasurementsInLinkFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Link);

        pImpl->task1_wrenchMeasurementsInCentroidalFrameVec = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Centroidal);

        pImpl->task1_wrenchMeasurementsInBaseFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Base);

        pImpl->task1_wrenchMeasurementsInWorldFrameVec      = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::World);

        pImpl->task2_wrenchMeasurementsInLinkFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Link);

        pImpl->task2_wrenchMeasurementsInCentroidalFrameVec = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Centroidal);

        pImpl->task2_wrenchMeasurementsInBaseFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::Base);

        pImpl->task2_wrenchMeasurementsInWorldFrameVec      = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                               hde::interfaces::IHumanWrench::WrenchType::Measured,
                                                                                                               hde::interfaces::IHumanWrench::WrenchReferenceFrame::World);

        // Get data from IHumanDynamics interface of HumanDynamicsEstimator
        pImpl->dynamicsNumberOfJoints = pImpl->iHumanDynamics->getNumberOfJoints();
        pImpl->dynamicsJointNames = pImpl->iHumanDynamics->getJointNames();
        pImpl->jointTorquesVec = pImpl->iHumanDynamics->getJointTorques();
        pImpl->internalWrenches = pImpl->iHumanDynamics->getInternalWrenches();


        pImpl->task1_wrenchEstimatesInLinkFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Link);

        pImpl->task1_wrenchEstimatesInCentroidalFrameVec = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Centroidal);

        pImpl->task1_wrenchEstimatesInBaseFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Base);

        pImpl->task1_wrenchEstimatesInWorldFrameVec      = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task1,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::World);

        pImpl->task2_wrenchEstimatesInLinkFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Link);

        pImpl->task2_wrenchEstimatesInCentroidalFrameVec = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Centroidal);

        pImpl->task2_wrenchEstimatesInBaseFrameVec       = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::Base);

        pImpl->task2_wrenchEstimatesInWorldFrameVec      = pImpl->iHumanWrenchEstimates->getWrenchesInFrame(hde::interfaces::IHumanWrench::TaskType::Task2,
                                                                                                            hde::interfaces::IHumanWrench::WrenchType::Estimated,
                                                                                                            hde::interfaces::IHumanWrench::WrenchReferenceFrame::World);

    }

    // Prepare buffer vectors
    // Handle arrays to vectors
    if (pImpl->isAttached.stateProvider) {

        pImpl->basePoseArray[0] = pImpl->basePosition[0];
        pImpl->basePoseArray[1] = pImpl->basePosition[1];
        pImpl->basePoseArray[2] = pImpl->basePosition[2];
        pImpl->basePoseArray[3] = pImpl->baseOrientation[0];
        pImpl->basePoseArray[4] = pImpl->baseOrientation[1];
        pImpl->basePoseArray[5] = pImpl->baseOrientation[2];
        pImpl->basePoseArray[6] = pImpl->baseOrientation[3];

        pImpl->basePoseVec = std::vector<double>(pImpl->basePoseArray.begin(), pImpl->basePoseArray.end());

        pImpl->baseVeclocityVec = std::vector<double>(pImpl->baseVelocity.begin(), pImpl->baseVelocity.end());

        pImpl->comPositionVec = std::vector<double>(pImpl->comPosition.begin(), pImpl->comPosition.end());

        pImpl->comVelocityVec = std::vector<double>(pImpl->comVelocity.begin(), pImpl->comVelocity.end());

        pImpl->comBiasAccelerationVec = std::vector<double>(pImpl->comBiasAcceleration.begin(), pImpl->comBiasAcceleration.end());

        pImpl->rateOfChangeOfMomentumInBaseFrameVec = std::vector<double>(pImpl->rateOfChangeOfMomentumInBaseFrame.begin(), pImpl->rateOfChangeOfMomentumInBaseFrame.end());

        pImpl->rateOfChangeOfMomentumInWorldFrameVec = std::vector<double>(pImpl->rateOfChangeOfMomentumInWorldFrame.begin(), pImpl->rateOfChangeOfMomentumInWorldFrame.end());

        pImpl->rateOfChangeOfMomentumInCentroidalFrameVec = std::vector<double>(pImpl->rateOfChangeOfMomentumInCentroidalFrame.begin(), pImpl->rateOfChangeOfMomentumInCentroidalFrame.end());

    }

    // Update interface data to matio buffers
    if (pImpl->matioLogger) {

        if (pImpl->isAttached.stateProvider) {

            // Set state joint names once
            if (pImpl->humanDataStruct.stateJointNames.empty()) {
                pImpl->humanDataStruct.stateJointNames = pImpl->stateJointNames;
            }

            // Set link names once
            if (pImpl->humanDataStruct.linkNames.empty()) {
                pImpl->humanDataStruct.linkNames = pImpl->linkNames;
            }

            pImpl->humanDataStruct.data["basePose"].push_back(pImpl->basePoseVec);
            pImpl->humanDataStruct.data["baseVelocity"].push_back(pImpl->baseVeclocityVec);

            pImpl->humanDataStruct.data["comPosition"].push_back(pImpl->comPositionVec);
            pImpl->humanDataStruct.data["comVelocity"].push_back(pImpl->comVelocityVec);
            pImpl->humanDataStruct.data["comBiasAcceleration"].push_back(pImpl->comBiasAccelerationVec);

            pImpl->humanDataStruct.data["rateOfChangeOfMomentumInBaseFrame"].push_back(pImpl->rateOfChangeOfMomentumInBaseFrameVec);
            pImpl->humanDataStruct.data["rateOfChangeOfMomentumInWorldFrame"].push_back(pImpl->rateOfChangeOfMomentumInWorldFrameVec);
            pImpl->humanDataStruct.data["rateOfChangeOfMomentumInCentroidalFrame"].push_back(pImpl->rateOfChangeOfMomentumInCentroidalFrameVec);

            pImpl->humanDataStruct.data["jointPositions"].push_back(pImpl->jointPositionsVec);
            pImpl->humanDataStruct.data["jointVelocities"].push_back(pImpl->jointVelocitiesVec);

            // Iterate over links
            // TODO: Improve idyntree vector handling
            for (std::string linkName : pImpl->linkNames) {

                {
                    // Handle link transform measurements buffer
                    // NOTE: Storing (pos, rot) of the transform with rot as RPY
                    std::vector<double> linkPose(6, 0.0);
                    linkPose.at(0) = pImpl->linkTransformsMap[linkName].getPosition().getVal(0);
                    linkPose.at(1) = pImpl->linkTransformsMap[linkName].getPosition().getVal(1);
                    linkPose.at(2) = pImpl->linkTransformsMap[linkName].getPosition().getVal(2);

                    linkPose.at(3) = pImpl->linkTransformsMap[linkName].getRotation().asRPY().getVal(0);
                    linkPose.at(4) = pImpl->linkTransformsMap[linkName].getRotation().asRPY().getVal(1);
                    linkPose.at(5) = pImpl->linkTransformsMap[linkName].getRotation().asRPY().getVal(2);

                    pImpl->humanDataStruct.linkData[linkName]["pose"].push_back(linkPose);
                }

                {
                    // Handle link velocity measurements buffer
                    std::vector<double> linkVel(6, 0.0);
                    for (size_t v = 0; v < pImpl->linkVelocitiesMap[linkName].size(); v++) {
                        linkVel.at(v) = pImpl->linkVelocitiesMap[linkName].getVal(v);
                    }

                    pImpl->humanDataStruct.linkData[linkName]["velocity"].push_back(linkVel);
                }

                {
                    // Handle link acceleration measurements buffer
                    std::vector<double> linkAcc(6, 0.0);
                    for (size_t a = 0; a < pImpl->linkAccelerationsMap[linkName].size(); a++) {
                        linkAcc.at(a) = pImpl->linkAccelerationsMap[linkName].getVal(a);
                    }

                    pImpl->humanDataStruct.linkData[linkName]["acceleration"].push_back(linkAcc);
                }
            }

        }

        if (pImpl->isAttached.wrenchProvider) {

            // Set wrench measurements source names once
            if (pImpl->humanDataStruct.wrenchMeasurementSourceNames.empty()) {
                pImpl->humanDataStruct.wrenchMeasurementSourceNames = pImpl->wrenchMeasurementSourceNames;
            }

            pImpl->humanDataStruct.data["wrenchMeasurements"].push_back(pImpl->wrenchMeasurementValuesVec);

        }

        if (pImpl->isAttached.dynamicsEstimator) {

            // Set wrench estimates source names once
            if (pImpl->humanDataStruct.wrenchEstimateSourceNames.empty()) {
                pImpl->humanDataStruct.wrenchEstimateSourceNames = pImpl->wrenchEstimateSourceNames;
            }

            pImpl->humanDataStruct.data["wrenchEstimates"].push_back(pImpl->wrenchEstimateValuesVec);

            pImpl->humanDataStruct.data["task1_wrenchMeasurementsInLinkFrame"].push_back(pImpl->task1_wrenchMeasurementsInLinkFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchMeasurementsInCentroidalFrame"].push_back(pImpl->task1_wrenchMeasurementsInCentroidalFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchMeasurementsInBaseFrame"].push_back(pImpl->task1_wrenchMeasurementsInBaseFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchMeasurementsInWorldFrame"].push_back(pImpl->task1_wrenchMeasurementsInWorldFrameVec);

            pImpl->humanDataStruct.data["task1_wrenchEstimatesInLinkFrame"].push_back(pImpl->task1_wrenchEstimatesInLinkFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchEstimatesInCentroidalFrame"].push_back(pImpl->task1_wrenchEstimatesInCentroidalFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchEstimatesInBaseFrame"].push_back(pImpl->task1_wrenchEstimatesInBaseFrameVec);
            pImpl->humanDataStruct.data["task1_wrenchEstimatesInWorldFrame"].push_back(pImpl->task1_wrenchEstimatesInWorldFrameVec);

            pImpl->humanDataStruct.data["task2_wrenchMeasurementsInLinkFrame"].push_back(pImpl->task2_wrenchMeasurementsInLinkFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchMeasurementsInCentroidalFrame"].push_back(pImpl->task2_wrenchMeasurementsInCentroidalFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchMeasurementsInBaseFrame"].push_back(pImpl->task2_wrenchMeasurementsInBaseFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchMeasurementsInWorldFrame"].push_back(pImpl->task2_wrenchMeasurementsInWorldFrameVec);

            pImpl->humanDataStruct.data["task2_wrenchEstimatesInLinkFrame"].push_back(pImpl->task2_wrenchEstimatesInLinkFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchEstimatesInCentroidalFrame"].push_back(pImpl->task2_wrenchEstimatesInCentroidalFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchEstimatesInBaseFrame"].push_back(pImpl->task2_wrenchEstimatesInBaseFrameVec);
            pImpl->humanDataStruct.data["task2_wrenchEstimatesInWorldFrame"].push_back(pImpl->task2_wrenchEstimatesInWorldFrameVec);


            // Set dynamics joint names once
            if (pImpl->humanDataStruct.dynamicsJointNames.empty()) {
                pImpl->humanDataStruct.dynamicsJointNames = pImpl->dynamicsJointNames;
            }

            pImpl->humanDataStruct.data["jointTorques"].push_back(pImpl->jointTorquesVec);

            // Iterate over links
            // TODO: Improve idyntree vector handling
            for (size_t linkIndex = 0; linkIndex < pImpl->linkNames.size(); linkIndex++) {

                std::string linkName = pImpl->linkNames.at(linkIndex);

                pImpl->internalWrenches.at(linkIndex);

                pImpl->humanDataStruct.linkData[linkName]["internalWrench"].push_back(std::vector<double>(pImpl->internalWrenches.at(linkIndex).begin(), pImpl->internalWrenches.at(linkIndex).end()));
            }
        }

    }

    // ============================
    // Put human data to yarp ports
    // ============================

    if (pImpl->isAttached.stateProvider) {

        // Prepare base pose data
        yarp::sig::Vector& basePoseYarpVector = pImpl->basePoseDataPort.prepare();
        YarpConversionsHelper::toYarp(basePoseYarpVector, pImpl->basePoseVec);

        // Prepare base velocity data
        yarp::sig::Vector& baseVelocityYarpVector = pImpl->baseVelocityDataPort.prepare();
        YarpConversionsHelper::toYarp(baseVelocityYarpVector, pImpl->baseVeclocityVec);

        // Prepare stateJointNames
        yarp::os::Bottle& stateJointNamesYarpBottle = pImpl->stateJointNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(stateJointNamesYarpBottle, pImpl->stateJointNames);

        // Prepare joint positions
        yarp::sig::Vector& jointPositionsYarpVector = pImpl->jointPositionsDataPort.prepare();
        YarpConversionsHelper::toYarp(jointPositionsYarpVector, pImpl->jointPositionsVec);

        // Prepare joint velocities
        yarp::sig::Vector& jointVelocitiesYarpVector = pImpl->jointVelocitiesDataPort.prepare();
        YarpConversionsHelper::toYarp(jointVelocitiesYarpVector, pImpl->jointVelocitiesVec);

        // Preprare com position
        yarp::sig::Vector& comPositionYarpVector = pImpl->comPositionDataPort.prepare();
        YarpConversionsHelper::toYarp(comPositionYarpVector, pImpl->comPositionVec);

        // Prepare com velocity
        yarp::sig::Vector& comVelocityYarpVector = pImpl->comVelocityDataPort.prepare();
        YarpConversionsHelper::toYarp(comVelocityYarpVector, pImpl->comVelocityVec);

        // Prepare com bia acceleration
        yarp::sig::Vector& comBiasAccelerationYarpVector = pImpl->comVelocityDataPort.prepare();
        YarpConversionsHelper::toYarp(comBiasAccelerationYarpVector, pImpl->comBiasAccelerationVec);

        // Rate of change of momentum in base frame
        yarp::sig::Vector& rateOfChangeOfMomentumInBaseFrameYarpVector = pImpl->rateOfChangeOfMomentumInBaseFrameDataPort.prepare();
        YarpConversionsHelper::toYarp(rateOfChangeOfMomentumInBaseFrameYarpVector, pImpl->rateOfChangeOfMomentumInBaseFrameVec);

        // Rate of change of momentum in world frame
        yarp::sig::Vector& rateOfChangeOfMomentumInWorldFrameYarpVector = pImpl->rateOfChangeOfMomentumInWorldFrameDataPort.prepare();
        YarpConversionsHelper::toYarp(rateOfChangeOfMomentumInWorldFrameYarpVector, pImpl->rateOfChangeOfMomentumInWorldFrameVec);

        // Rate of change of momentum in centroidal frame
        yarp::sig::Vector& rateOfChangeOfMomentumInCentroidalFrameYarpVector = pImpl->rateOfChangeOfMomentumInCentroidalFrameDataPort.prepare();
        YarpConversionsHelper::toYarp(rateOfChangeOfMomentumInCentroidalFrameYarpVector, pImpl->rateOfChangeOfMomentumInCentroidalFrameVec);

        // Send data through yarp ports
        pImpl->basePoseDataPort.write(true);
        pImpl->baseVelocityDataPort.write(true);
        pImpl->stateJointNamesDataPort.write(true);
        pImpl->jointPositionsDataPort.write(true);
        pImpl->jointVelocitiesDataPort.write(true);
        pImpl->comPositionDataPort.write(true);
        pImpl->comVelocityDataPort.write(true);
        pImpl->comBiasAccelerationDataPort.write(true);
        pImpl->rateOfChangeOfMomentumInBaseFrameDataPort.write(true);
        pImpl->rateOfChangeOfMomentumInWorldFrameDataPort.write(true);
        pImpl->rateOfChangeOfMomentumInCentroidalFrameDataPort.write(true);
    }

    if (pImpl->isAttached.wrenchProvider) {

        // Prepare wrench measurements source names data
        yarp::os::Bottle& wrenchMeasurementSourceNamesYarpBottle = pImpl->wrenchMeasurementSourceNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchMeasurementSourceNamesYarpBottle, pImpl->wrenchMeasurementSourceNames);

        // Prepare wrench measurement values data
        yarp::sig::Vector& wrenchMeasurementValuesYarpVector = pImpl->wrenchMeasurementValuesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchMeasurementValuesYarpVector, pImpl->wrenchMeasurementValuesVec);

        // Send data through yarp ports
        pImpl->wrenchMeasurementSourceNamesDataPort.write(true);
        pImpl->wrenchMeasurementValuesDataPort.write(true);
    }

    if (pImpl->isAttached.dynamicsEstimator) {

        // Prepare wrench estimates source names data
        yarp::os::Bottle& wrenchEstimateSourceNamesYarpBottle = pImpl->wrenchEstimateSourceNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchEstimateSourceNamesYarpBottle, pImpl->wrenchEstimateSourceNames);

        // Prepare wrench estimate values data
        yarp::sig::Vector& wrenchEstimateValuesYarpVector = pImpl->wrenchEstimateValuesDataPort.prepare();
        YarpConversionsHelper::toYarp(wrenchEstimateValuesYarpVector, pImpl->wrenchEstimateValuesVec);

        // Prepare dynamics joint names data
        yarp::os::Bottle& dynamicsJointNamesYarpBottle = pImpl->dynamicsJointNamesDataPort.prepare();
        YarpConversionsHelper::toYarp(dynamicsJointNamesYarpBottle, pImpl->dynamicsJointNames);

        // Prepare joint torques data
        yarp::sig::Vector& jointTorqesYarpVector = pImpl->jointTorquesDataPort.prepare();
        YarpConversionsHelper::toYarp(jointTorqesYarpVector, pImpl->jointTorquesVec);

        // Send data through yarp ports
        pImpl->wrenchEstimateSourceNamesDataPort.write(true);
        pImpl->wrenchEstimateValuesDataPort.write(true);
        pImpl->dynamicsJointNamesDataPort.write(true);
        pImpl->jointTorquesDataPort.write(true);
    }
}

bool HumanDataCollector::attach(yarp::dev::PolyDriver *poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver device is nullptr";
        return false;
    }

    // Check the polydriver options for device option
    yarp::os::Bottle driverOptions = poly->getOptions();
    if (!driverOptions.check("device")) {
        yError() << LogPrefix << "Passed polydriver does not have <device> as an option";
        return false;
    }

    // Get the passed device name
    const std::string deviceName = poly->getValue("device").asString();

    // Ensure the passed device is acceptable
    if (deviceName != "human_state_provider" && deviceName != "human_wrench_provider" && deviceName != "human_dynamics_estimator") {
        yError() << "This wrapper does not accept the " << deviceName << " passed as the polydriver device for attaching";
        return  false;
    }

    if (deviceName == "human_state_provider") {

        // Attach to IHumanState interface from HumanStateProvider
        if (pImpl->iHumanState || !poly->view(pImpl->iHumanState) || !pImpl->iHumanState) {
            yError() << LogPrefix << "Failed to view IHumanState interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanState->getNumberOfJoints() == 0 || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
            yError() << "The viewed IHumanState interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.stateProvider = true;
    }

    if (deviceName == "human_wrench_provider") {

        // Attach to IHumanWrench interface from HumanWrenchProvider
        if (pImpl->iHumanWrenchMeasurements || !poly->view(pImpl->iHumanWrenchMeasurements) || !pImpl->iHumanWrenchMeasurements) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchMeasurements->getNumberOfWrenchSources() != pImpl->iHumanWrenchMeasurements->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.wrenchProvider = true;

    }

    if (deviceName == "human_dynamics_estimator") {

        // Attach to IHumanWrench interface from HumanDynamicsEstimator
        if (pImpl->iHumanWrenchEstimates || !poly->view(pImpl->iHumanWrenchEstimates) || !pImpl->iHumanWrenchEstimates) {
            yError() << LogPrefix << "Failed to view IHumanWrench interface from the " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanWrenchEstimates->getNumberOfWrenchSources() != pImpl->iHumanWrenchEstimates->getWrenchSourceNames().size()) {
            yError() << "The viewed IHumanWrench interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        // Attach to IHumanDynamics interface from HumanDynamicsEstimator
        if (pImpl->iHumanDynamics || !poly->view(pImpl->iHumanDynamics) || !pImpl->iHumanDynamics) {
            yError() << LogPrefix << "Failed to view IHumanDynamics interface from the  " << deviceName << " polydriver device";
            return false;
        }

        // Check the viewed interface
        if (pImpl->iHumanDynamics->getNumberOfJoints() == 0 || pImpl->iHumanDynamics->getNumberOfJoints() != pImpl->iHumanDynamics->getJointNames().size()) {
            yError() << "The viewed IHumanDynamics interface from " << deviceName << " polydriver device might not be ready";
            return false;
        }

        yInfo() << LogPrefix << deviceName << "attach() successful";
        pImpl->isAttached.dynamicsEstimator = true;
    }

    return true;
}

bool HumanDataCollector::detach()
{
    while (isRunning()) {
        stop();
    }

    if (pImpl->matioLogger) {

        // Set the string variables elements as cell arrays to mat cell variables
        writeVectorOfStringToMatCell("stateJointNames", pImpl->humanDataStruct.stateJointNames, pImpl->matFilePtr.get());
        writeVectorOfStringToMatCell("linkNames", pImpl->humanDataStruct.linkNames, pImpl->matFilePtr.get());
        writeVectorOfStringToMatCell("wrenchMeasurementSourceNames", pImpl->humanDataStruct.wrenchMeasurementSourceNames, pImpl->matFilePtr.get());
        writeVectorOfStringToMatCell("wrenchEstimateSourceNames", pImpl->humanDataStruct.wrenchEstimateSourceNames, pImpl->matFilePtr.get());
        writeVectorOfStringToMatCell("dynamicsJointNames", pImpl->humanDataStruct.dynamicsJointNames, pImpl->matFilePtr.get());

        // Correct the time values stored in the time vector to zero start value
        std::transform( pImpl->humanDataStruct.time.begin(), pImpl->humanDataStruct.time.end(), pImpl->humanDataStruct.time.begin(),
                        std::bind2nd( std::plus<long>(), - pImpl->humanDataStruct.time.at(0) ) );

        // Create an array with time elements
        int64_t time[pImpl->humanDataStruct.time.size()];
        std::copy(pImpl->humanDataStruct.time.begin(), pImpl->humanDataStruct.time.end(), time);

        for (int64_t timeCount : pImpl->humanDataStruct.time) {

            //TODO: Double check the int64_t double conversion handling
            std::vector<double> count;
            count.push_back(timeCount);

            pImpl->humanDataStruct.data["time"].push_back(count);

        }

        // Set human data to mat struct
        writeVectorOfVectorDoubleToMatStruct(pImpl->humanDataStruct.data, "data", pImpl->matFilePtr.get());
        writeVectorOfVectorOfVectorDoubleToMatStruct(pImpl->humanDataStruct.linkData, "linkData", pImpl->linkNames, pImpl->matFilePtr.get());

    }

    // Moving back a directory above the matLogDirectory
    std::experimental::filesystem::current_path(pImpl->originalWorkingDirectory);

    if (pImpl->isAttached.stateProvider) {

        if (!pImpl->basePoseDataPort.isClosed()) {
            pImpl->basePoseDataPort.close();
        }

        if (!pImpl->baseVelocityDataPort.isClosed()) {
            pImpl->baseVelocityDataPort.close();
        }

        if (!pImpl->stateJointNamesDataPort.isClosed()) {
            pImpl->stateJointNamesDataPort.close();
        }

        if (!pImpl->jointPositionsDataPort.isClosed()) {
            pImpl->jointPositionsDataPort.close();
        }

        if (!pImpl->jointVelocitiesDataPort.isClosed()) {
            pImpl->jointVelocitiesDataPort.close();
        }

        if (!pImpl->comPositionDataPort.isClosed()) {
            pImpl->comPositionDataPort.close();
        }

        if (!pImpl->comVelocityDataPort.isClosed()) {
            pImpl->comVelocityDataPort.close();
        }

        if (!pImpl->comBiasAccelerationDataPort.isClosed()) {
            pImpl->comBiasAccelerationDataPort.close();
        }

        if (!pImpl->rateOfChangeOfMomentumInBaseFrameDataPort.isClosed()) {
            pImpl->rateOfChangeOfMomentumInBaseFrameDataPort.close();
        }

        if (!pImpl->rateOfChangeOfMomentumInWorldFrameDataPort.isClosed()) {
            pImpl->rateOfChangeOfMomentumInWorldFrameDataPort.close();
        }

        if (!pImpl->rateOfChangeOfMomentumInCentroidalFrameDataPort.isClosed()) {
            pImpl->rateOfChangeOfMomentumInCentroidalFrameDataPort.close();
        }

        pImpl->iHumanState = nullptr;
        pImpl->isAttached.stateProvider = false;

    }

    if (pImpl->isAttached.wrenchProvider) {

        if (!pImpl->wrenchMeasurementSourceNamesDataPort.isClosed()) {
            pImpl->wrenchMeasurementSourceNamesDataPort.close();
        }

        if (!pImpl->wrenchMeasurementValuesDataPort.isClosed()) {
            pImpl->wrenchMeasurementValuesDataPort.close();
        }

        pImpl->iHumanWrenchMeasurements = nullptr;
        pImpl->isAttached.wrenchProvider = false;
    }

    if (pImpl->isAttached.dynamicsEstimator) {

        if (!pImpl->wrenchEstimateSourceNamesDataPort.isClosed()) {
            pImpl->wrenchEstimateSourceNamesDataPort.close();
        }

        if (!pImpl->wrenchEstimateValuesDataPort.isClosed()) {
            pImpl->wrenchEstimateValuesDataPort.close();
        }

        if (!pImpl->dynamicsJointNamesDataPort.isClosed()) {
            pImpl->dynamicsJointNamesDataPort.close();
        }

        if (!pImpl->jointTorquesDataPort.isClosed()) {
            pImpl->jointTorquesDataPort.close();
        }

        pImpl->iHumanWrenchEstimates = nullptr;
        pImpl->iHumanDynamics = nullptr;
        pImpl->isAttached.dynamicsEstimator = false;
    }

    return true;
}

bool HumanDataCollector::attachAll(const yarp::dev::PolyDriverList &driverList)
{
    bool attachStatus = true;

    if (driverList.size() > 3) {
        yError() << LogPrefix << "This wrapper accepts at most three attached PolyDriver devices";
        return false;
    }

    for (size_t i = 0; i < driverList.size(); i++) {
        const yarp::dev::PolyDriverDescriptor* driver = driverList[i];

        if (!driver) {
            yError() << LogPrefix << "Passed PolyDriver device is nullptr";
            return false;
        }

        attachStatus = attachStatus && attach(driver->poly);
    }

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!(attachStatus && start())) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return attachStatus;
}

bool HumanDataCollector::detachAll()
{
    return detach();
}
