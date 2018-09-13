/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_DOUBLELASERDRIVER_HH
#define GAZEBOYARP_DOUBLELASERDRIVER_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/Wrapper.h>

#include <boost/shared_ptr.hpp>


#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>

#include <string>
#include <functional>
#include <unordered_map>
#include <vector>



namespace yarp {
    namespace dev {
        class GazeboYarpDoubleLaserDriver;
        };
    }

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }

    namespace physics {
        class Model;
        class Joint;
        typedef boost::shared_ptr<Joint> JointPtr;
    }

    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const std::string YarpLaserSensorScopedName;

class yarp::dev::GazeboYarpDoubleLaserDriver:
    public yarp::dev::DeviceDriver,
    public yarp::dev::IRangefinder2D,
    public yarp::dev::IMultipleWrapper
{
public:

    GazeboYarpDoubleLaserDriver();
    virtual ~GazeboYarpDoubleLaserDriver();

    /**
     * Gazebo stuff
     */

    /**
     * Callback for the WorldUpdateBegin Gazebo event.
     */
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Callback for the WorldReset Gazebo event.
     */
    void onReset();
    
    
    /**
     * Yarp interfaces start here
     */

    bool open(yarp::os::Searchable& config);
    bool close();
    
    //IMultipleWrapper interface
    bool attachAll(const PolyDriverList &p) override;
    bool detachAll() override;
    
    //IRangefinder2D interface
    virtual bool getRawData(yarp::sig::Vector &data) override;
    virtual bool getLaserMeasurement(std::vector<LaserMeasurementData> &data) override;
    virtual bool getDeviceStatus     (Device_status &status) override;
    virtual bool getDeviceInfo       (std::string &device_info) override; //
    virtual bool getDistanceRange    (double& min, double& max) override;
    virtual bool setDistanceRange    (double min, double max) override;
    virtual bool getScanLimits        (double& min, double& max) override;
    virtual bool setScanLimits        (double min, double max) override;
    virtual bool getHorizontalResolution      (double& step) override;
    virtual bool setHorizontalResolution      (double step) override;
    virtual bool getScanRate         (double& rate) override;
    virtual bool setScanRate         (double rate) override;

private:


    double calculate(int sensNum, double distance, bool front);
    std::string m_deviceName;
    gazebo::sensors::RaySensor* m_parentSensor;
    
    yarp::dev::PolyDriver * m_driver_laserFront;
    yarp::dev::IRangefinder2D* m_dev_laserFront;

    yarp::dev::PolyDriver * m_driver_laserBack;
    yarp::dev::IRangefinder2D* m_dev_laserBack;
    
    int m_samples;
    double m_resolution;

    /**
     * Connection to the WorldUpdateBegin Gazebo event
     */
    gazebo::event::ConnectionPtr m_updateConnection;

    /**
     * Connection to the WorldReset Gazebo event
     */
    gazebo::event::ConnectionPtr m_resetConnection;

    yarp::os::Property m_pluginParameters; /**< Contains the parameters of the device contained in the yarpConfigurationFile .ini file */

    bool m_started;
    int m_clock;
    gazebo::common::Time m_previousTime;
    
    yarp::os::Mutex       m_mutex; //MI SERVE??????
};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
