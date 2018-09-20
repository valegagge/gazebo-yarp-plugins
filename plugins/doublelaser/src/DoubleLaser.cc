/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "DoubleLaser.hh"
#include "DoubleLaserDevice.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>

#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


using namespace std;
namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpDoubleLaser)

    GazeboYarpDoubleLaser::GazeboYarpDoubleLaser() : m_iWrap_rangeFinder(0), m_iWrap_doublelaser(0)
    {}

    GazeboYarpDoubleLaser::~GazeboYarpDoubleLaser()
    {
        if (m_iWrap_doublelaser) 
        {
            m_iWrap_doublelaser->detachAll();
            m_iWrap_rangeFinder = 0;
        }
        
        if (m_iWrap_rangeFinder) 
        {
            m_iWrap_rangeFinder->detachAll();
            m_iWrap_rangeFinder = 0;
        }

        if (m_wrapper_rangeFinder.isValid()) 
        {
            m_wrapper_rangeFinder.close();
        }


        for (int n = 0; n < m_lasers.size(); n++) {
            std::string scopedDeviceName = m_sensorName + "::" + m_lasers[n]->key.c_str();
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
        }

        GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
        yarp::os::Network::fini();
    }


    bool GazeboYarpDoubleLaser::readConfigurationFromFile(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!_sdf->HasElement("yarpConfigurationFile"))
        {
            yError() << "GazeboYarpDoubleLaser: error: unabble to load configuration";
            return false;
        }

        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path == "")
        {
            yError() << "GazeboYarpDoubleLaser: ini file path is empty";
            return false;
        }

        GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

        bool wipe = false; //in order to not clear m_parameters
        if (! m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
        {
            yError()  << "GazeboYarpDoubleLaser: error reading parameters from config file= " << ini_file_name << "in" <<ini_file_path ;
            return false;
        }

        m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

        return true;
    }



    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpDoubleLaser::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // 1) check params
        if (!_parent)
        {
            gzerr << "GazeboYarpDoubleLaser plugin requires a parent.\n";
            return;
        }

        //2) check yarp network
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpDoubleLaser : yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        m_sensorName = _parent->GetScopedName();
        
        //3) load configuration from sdf
        if(!readConfigurationFromFile( _parent, _sdf))
            return;

        // 4) Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));


        // 5) Add the gazebo_doubleLaser device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::DoubleLaserDevice>("DoubleLaser", "Rangefinder2DWrapper", "GazeboYarpDoubleLaserDriver"));



        yError()<< "LOAD parameters-----------------------------------------------";
        yError() << m_parameters.toString();
        yError() << "-------------------------------------------------------------";

        //6) open wrapper Rangefinder2DWrapper
        yarp::os::Property wrapper_parameters;
        if(!m_parameters.check("WRAPPER"))
        {
            yError() << "GazeboYarpDoubleLaser: [WRAPPER] group is missing in configuration file";
            return;
        }
        wrapper_parameters.fromString(m_parameters.findGroup("WRAPPER").toString());
        if(m_parameters.check("ROS"))
        {
            wrapper_parameters.addGroup("ROS").fromString(m_parameters.findGroup("ROS").toString());
        }
        else
        {
            yInfo() << "GazeboYarpDoubleLaser: ROS group is missing in configuration file";
        }


        yError()<< "WRAPPER parameters-----------------------------------------------";
        yError() << wrapper_parameters.toString();
        yError() << "-------------------------------------------------------------";

        if(! m_wrapper_rangeFinder.open(wrapper_parameters))
        {
            yError()<<"GazeboYarpDoubleLaser failed: error in opening yarp driver wrapper Rangefinder2DWrapper!!!";
            return;
        }
//         else
//         {
//             yError() << "GazeboYarpDoubleLaser : ho aperto il wrapper Rangefinder2DWrapper ";
//
//         }

        if (!m_wrapper_rangeFinder.view(m_iWrap_rangeFinder)) 
        {
            yError("GazeboYarpDoubleLaser : wrapper interface not found, load failed. Rangefinder2DWrapper");
            return;
        }
//         else
//         {
//              yError() << "GazeboYarpDoubleLaser : ho ho fatto la view di Rangefinder2DWrapper ";
//         }


        // 8) Open the driver DoubleLaser
        yError() << "GazeboYarpDoubleLaser : sto per aprire il device doublelaser..... ";


        yarp::os::Property doublelaser_dev_parameters;
        if(!m_parameters.check("DEVICE-CFG"))
        {
            yError() << "GazeboYarpDoubleLaser: [DEVICE-CFG] group is missing in configuration file";
            return;
        }
        doublelaser_dev_parameters.fromString(m_parameters.findGroup("DEVICE-CFG").toString());

        if(!m_parameters.check("LASERFRONT-CFG"))
        {
            yError() << "GazeboYarpDoubleLaser: LASERFRONT-CFG group is missing in configuration file";
            return;
        }

        doublelaser_dev_parameters.addGroup("LASERFRONT-CFG").fromString(m_parameters.findGroup("LASERFRONT-CFG").toString());

        if(!m_parameters.check("LASERBACK-CFG"))
        {
            yError() << "GazeboYarpDoubleLaser: LASERBACK-CFG group is missing in configuration file";
            return;
        }
        doublelaser_dev_parameters.addGroup("LASERBACK-CFG").fromString(m_parameters.findGroup("LASERBACK-CFG").toString());

        if( m_driver_doublelaser.open(doublelaser_dev_parameters) )
        {
        
            yError() << "GazeboYarpDoubleLaser : ho aperto il driver doubleLaser ";
        }
        else 
        {
            yError()<<"GazeboYarpDoubleLaser Plugin failed: error in opening yarp driver doubleLaser";
            return;
        }
        

        // 9 )finds device of laser front and laser back. the device names are written in configuration .ini file
        yarp::os::Bottle &front_name = m_parameters.findGroup("LASERFRONT-CFG").findGroup("sensorName");
        if(front_name.isNull())
        {
            yError() << "GazeboYarpDoubleLaser: cannot find LASERFRONT-CFG.sensorName parameter";
            return;
        }
         yarp::os::Bottle &back_name = m_parameters.findGroup("LASERBACK-CFG").findGroup("sensorName");
        if(back_name.isNull())
        {
            yError() << "GazeboYarpDoubleLaser: cannot find LASERBACK-CFG.sensorName parameter";
            return;
        }


        std::string laserFront_name = front_name.find("sensorName").asString();
        std::string laserBack_name = back_name.find("sensorName").asString();
       // yError() << "GazeboYarpDoubleLaser: laserFront=" <<  laserFront_name <<" laserBack=" << laserBack_name ;


        m_driver_laserFront = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserFront_name);
        if(m_driver_laserFront == nullptr)
        {
            yError() << "GazeboYarpDoubleLaser: cannot find laserFront device";
            return;
        }
//         else
//         {
//             yError() << "****GazeboYarpDoubleLaser: finded laserFront device. OK";
//         }




        m_driver_laserBack = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserBack_name);
        if(m_driver_laserBack == nullptr)
        {
            yError() << "GazeboYarpDoubleLaser: cannot find laserBack device";
            return;
        }
//         else
//         {
//             yError() << "****GazeboYarpDoubleLaser: finded laserBack device. OK";
//         }



        yarp::dev::PolyDriverList listoflasers; //it will contain front and back lasera
        yarp::dev::PolyDriverDescriptor laserFront_desc;
        yarp::dev::PolyDriverDescriptor laserBack_desc;

        laserFront_desc.poly = m_driver_laserFront;
        laserFront_desc.key = laserFront_name;
        listoflasers.push(laserFront_desc);

        laserBack_desc.poly = m_driver_laserBack;
        laserBack_desc.key = laserBack_name;
        listoflasers.push(laserBack_desc);

        m_driver_doublelaser.view(m_iWrap_doublelaser);
        if(!m_iWrap_doublelaser->attachAll(listoflasers))
        {
            yError() << "GazeboYarpDoubleLaser: error douring attaching double laser to front and back laser devices";
            //TODO: chiudi il devicedouble
            return;
        }


        // 10 )attach rangefinder wrapper to double laser
        yarp::dev::PolyDriverList listofdoubellaser; //it will contain only double laser
        yarp::dev::PolyDriverDescriptor doublelaser_desc;
        doublelaser_desc.poly = &m_driver_doublelaser;
        
        listofdoubellaser.push(doublelaser_desc);
        
        if(m_iWrap_rangeFinder->attachAll(listofdoubellaser))
        {
            yError() << "GazeboYarpDoubleLaser: ho fatto attach di rangeFinder wrapper al double laser. OK";
        }
        else
        {
            yError() << "GazeboYarpDoubleLaser: ERRORE mentre facevo attach di rangeFinder wrapper al double laser.";
        }
        
         
         
         
      //Insert the pointer in the singleton handler for retriving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

     }

}
