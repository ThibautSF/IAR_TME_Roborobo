/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */

#include "TMEbraitenberg/include/TMEbraitenbergController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEbraitenbergController::TMEbraitenbergController(RobotWorldModel *__wm) : Controller(__wm)
{
    if (_wm->_cameraSensorsNb != NB_SENSORS)
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEbraitenbergController::~TMEbraitenbergController()
{
    // nothing to do.
}

void TMEbraitenbergController::reset()
{
    // nothing to do.
}

void TMEbraitenbergController::step()
{
    double translation = 1.0;
    double rotation = 0.0;

    //translation = sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0);

    rotation = (getDistanceAt(SENSOR_FLL) * -0.5 + getDistanceAt(SENSOR_FL) * -0.75 + getDistanceAt(SENSOR_FFL) * -1 + getDistanceAt(SENSOR_FRR) * 0.5 + getDistanceAt(SENSOR_FR) * 0.75 + getDistanceAt(SENSOR_FFR) * 1);

    //Bound in [-1;1]
    if (rotation > 1)
    {
        rotation = 1.0;
    }
    else if (rotation <= -1)
    {
        rotation = -1.0;
    }

    setRotation(rotation);
    setTranslation(translation);
}
