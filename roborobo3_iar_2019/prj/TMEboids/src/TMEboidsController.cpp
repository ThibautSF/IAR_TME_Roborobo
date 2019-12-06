/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */

#include <algorithm>
#include <vector>

#include "TMEboids/include/TMEboidsController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEboidsController::TMEboidsController(RobotWorldModel *__wm) : Controller(__wm)
{
    if (_wm->_cameraSensorsNb != NB_SENSORS)
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEboidsController::~TMEboidsController()
{
    // nothing to do.
}

void TMEboidsController::reset()
{
    // nothing to do.
}

void TMEboidsController::step()
{
    double rotation = 0.0;    // Default rotation
    double translation = 1.0; // Default translation

    //Self parameters
    double avoidWallRadius = gSensorRange * 0.5;
    double repulseRadius = gSensorRange * 0.1;
    double attractRadius = gSensorRange * 0.8;
    double deltaOrient = 0.0;
    double angle_correction = 0.3;

    //Infos gathered from others
    double avgDeltaOrient = 0;
    double avgMassCenter_X = 0;
    double avgMassCenter_Y = 0;

    std::vector<int> myneighbors; // Vector of robot neighbors
    double min_dist = gSensorRange + 1;
    double min_dist_wall = gSensorRange * 2;

    //BEGIN gathering sensor info
    for (int i = 0; i != NB_SENSORS; i++)
    {
        double dist = _wm->getCameraSensorValue(i, SENSOR_DISTANCEVALUE);

        if (getWallAt(i) == 1)
        {
            //There is a wall
            if (dist < min_dist_wall)
            {
                min_dist_wall = dist;
            }
        }
        else
        {
            int robotId = getRobotIdAt(i);
            if (robotId != -1)
            {
                //It's a robot !
                //if (!(std::find(myneighbors.begin(), myneighbors.end(), robotId) != myneighbors.end()))
                if (true)
                {
                    //Ignore already treated robots (in case a robot is in multiple sensor)
                    myneighbors.push_back(robotId);

                    // Update avg center of mass
                    avgMassCenter_X = avgMassCenter_X + gWorld->getRobot(robotId)->getWorldModel()->_xReal;
                    avgMassCenter_Y = avgMassCenter_Y + gWorld->getRobot(robotId)->getWorldModel()->_yReal;

                    // update avg delta orientation
                    double myOrient = _wm->_agentAbsoluteOrientation;
                    double targetOrient = gWorld->getRobot(robotId)->getWorldModel()->_agentAbsoluteOrientation;

                    double delta_orient = myOrient - targetOrient;
                    //Bound in [-180;180] then in [-1;1]
                    if (delta_orient >= 180.0)
                        delta_orient = -(360.0 - delta_orient);
                    else if (delta_orient <= -180.0)
                        delta_orient = -(-360.0 - delta_orient);

                    delta_orient = delta_orient / 180.0;

                    avgDeltaOrient += delta_orient;

                    if (dist < min_dist)
                    {
                        min_dist = dist;
                    }
                }
            }
            else
            {
            }
        }
    }
    //END gathering sensor info

    if (myneighbors.size() > 0)
    {
        avgDeltaOrient = avgDeltaOrient / myneighbors.size();
        avgMassCenter_X = avgMassCenter_X / myneighbors.size();
        avgMassCenter_Y = avgMassCenter_Y / myneighbors.size();
    }

    if (min_dist_wall < avoidWallRadius)
    {
        // Wall too close, avoid it !

        //translation = sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0);
        rotation = (getDistanceAt(SENSOR_FLL) * -0.5 + getDistanceAt(SENSOR_FL) * -0.75 + getDistanceAt(SENSOR_FFL) * -1 + getDistanceAt(SENSOR_FRR) * 0.5 + getDistanceAt(SENSOR_FR) * 0.75 + getDistanceAt(SENSOR_FFR) * 1);
    }
    else
    {
        if (myneighbors.size() == 0)
        {
            //nobody thus go forward.
            //(use default translation and rotation)
        }
        else
        {
            double angleDeltaToCenterOfMass = 0.0;
            if (min_dist < repulseRadius)
            {
                // Do repulsion

                angleDeltaToCenterOfMass = getAngleToTarget(_wm->_xReal, _wm->_yReal, _wm->_agentAbsoluteOrientation, avgMassCenter_X, avgMassCenter_Y);

                if (angleDeltaToCenterOfMass < 0)
                    rotation = angle_correction;
                else
                    rotation = -angle_correction;
            }
            else
            {
                if (min_dist > attractRadius)
                {
                    // Do attraction

                    angleDeltaToCenterOfMass = getAngleToTarget(_wm->_xReal, _wm->_yReal, _wm->_agentAbsoluteOrientation, avgMassCenter_X, avgMassCenter_Y);

                    if (angleDeltaToCenterOfMass < 0)
                        rotation = -angle_correction;
                    else
                        rotation = angle_correction;
                }
                else
                {
                    if (fabs(avgDeltaOrient) > deltaOrient)
                    {
                        // Do orientation
                        rotation = avgDeltaOrient;
                        if (avgDeltaOrient < 0)
                            rotation = angle_correction;
                        else
                            rotation = -angle_correction;
                    }
                    else
                    {
                        // Nothing to do thus go forward
                        //(use default translation and rotation)
                    }
                }
            }
        }
    }

    //Bound in [-1;1]
    if (rotation > 1)
    {
        rotation = 1.0;
    }
    else if (rotation <= -1)
    {
        rotation = -1.0;
    }

    //Bound in [-1;1]
    if (translation > 1)
    {
        translation = 1.0;
    }
    else if (translation <= -1)
    {
        translation = -1.0;
    }

    setRotation(rotation);
    setTranslation(translation);
}
