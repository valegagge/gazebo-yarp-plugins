/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_DOUBLELASER_HH
#define GAZEBOYARP_DOUBLELASER_HH

#include <gazebo/common/Plugin.hh>

#include <string>

#include <yarp/dev/PolyDriverList.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
/// \class GazeboYarpDoubleLaser
/// Gazebo Plugin emulating the yarp controlBoard device in Gazebo.
///.
/// It can be configurated using the yarpConfigurationFile sdf tag,
/// that contains a Gazebo URI pointing at a yarp .ini configuration file
/// containt the configuration parameters of the controlBoard
///
/// The gazebo plugin is the "main" of the yarp device,
/// so what it should is to initialize the device, copy the
/// gazebo pointer, and return
///
/// The device will receive the gazebo pointer, parse the model,
/// and wait for yarp connections and the gazebo wait event.
///
class GazeboYarpDoubleLaser : public SensorPlugin
{
public:
    GazeboYarpDoubleLaser();
    virtual ~GazeboYarpDoubleLaser();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
    yarp::dev::PolyDriver m_wrapper_rangeFinder;
    yarp::dev::IMultipleWrapper* m_iWrap_rangeFinder;
    
    
    //yarp::dev::PolyDriverList m_doublelaser; //in this list there is only the doublelaser device
    yarp::dev::PolyDriver m_driver_doublelaser;
    yarp::dev::IMultipleWrapper* m_iWrap_doublelaser; //the wrapper interface of doublelaser
    
    
    yarp::dev::PolyDriverList m_lasers;// the contains the pointer two laser (front and back)

    yarp::os::Property m_parameters;

    std::string m_sensorName;
};

}

#endif
