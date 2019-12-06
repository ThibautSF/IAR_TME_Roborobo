/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */

#include "TMEdispersion/include/TMEdispersionController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEdispersionController::TMEdispersionController(RobotWorldModel *__wm) : Controller(__wm)
{
    if (_wm->_cameraSensorsNb != NB_SENSORS)
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEdispersionController::~TMEdispersionController()
{
    // nothing to do.
}

void TMEdispersionController::reset()
{
    // nothing to do.
}

void TMEdispersionController::step()
{
    double rotation = 0.0;    // Default rotation
    double translation = 0.0; // Default translation

    int nearest = -1;
    double min_dist = gSensorRange * 2;
    double nearest_angle = 0;

    //double srcOrientation = _wm->_agentAbsoluteOrientation;

    for (int i = 0; i != NB_SENSORS; i++)
    {
        double dist = getDistanceAt(i);
        if (getWallAt(i) == 1)
        {
        }
        else
        {
            int robotId = getRobotIdAt(i);
            if (robotId != -1)
            {
                if (dist < min_dist)
                {
                    nearest = i;
                    //nearest_angle = getRobotRelativeOrientationAt(i);
                    nearest_angle = getAngleToTarget(_wm->_xReal, _wm->_yReal, _wm->_agentAbsoluteOrientation, gWorld->getRobot(robotId)->getWorldModel()->_xReal, gWorld->getRobot(robotId)->getWorldModel()->_yReal) / 180;
                    min_dist = dist;
                }
            }
            else
            {
            }
        }
    }

    if (nearest != -1)
    {
        translation = 1.0;
        rotation = -nearest_angle;
    }
    else
    {
        rotation = 1.0;
        //avoidance
        //rotation = (getDistanceAt(SENSOR_FLL) * -0.55 + getDistanceAt(SENSOR_FL) * -0.75 + getDistanceAt(SENSOR_FFL) * -1 + getDistanceAt(SENSOR_FRR) * 0.55 + getDistanceAt(SENSOR_FR) * 0.75 + getDistanceAt(SENSOR_FFR) * 1);
    }

    //Bound in [-1;1]
    if (rotation > 1)
    {
        rotation = 1.0;
    }
    else if (rotation < -1)
    {
        rotation = -1.0;
    }

    setRotation(rotation);
    setTranslation(translation);
}
