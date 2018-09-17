/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <cstdio>
#include "DoubleLaserDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Events.hh>


#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <math.h>
#include <cmath>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


GazeboYarpDoubleLaserDriver::GazeboYarpDoubleLaserDriver() : m_deviceName(""), m_inited(false), m_onSimulation(true) //TODO: da cambiare!!
{
    yWarning () <<"....GazeboYarpDoubleLaserDriver::sono nel costruttore...";
}

GazeboYarpDoubleLaserDriver::~GazeboYarpDoubleLaserDriver() {}


void GazeboYarpDoubleLaserDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{

}

void GazeboYarpDoubleLaserDriver::onReset()
{
}


// bool GazeboYarpDoubleLaserDriver::attachAll(const PolyDriverList &p)
// {
//     if(p.size()!=2)
//     {
//         yError() << "GazeboYarpDoubleLaserDriver attach whin wrong num of drivers";
//     }
//  //here I suppose that first polydriver is the front laser, while the second the back laser
//     m_driver_laserFront = p[0]->poly;
//     m_driver_laserBack = p[1]->poly;
//
//
//
//     if(m_driver_laserFront == nullptr)
//     {
//         yError() << "GazeboYarpDoubleLaserDriver: cannot find laserFront device";
//         return false;
//     }
//         else
//     {
//         yError() << "****GazeboYarpDoubleLaserDriver: finded laserFront device. OK";
//     }
//
//
//     if(!m_driver_laserFront->view(m_dev_laserFront))
//     {
//         yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser front";
//         return false;
//     }
//     else
//     {
//         yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser front. OK";
//     }
//
//     if(m_driver_laserBack == nullptr)
//     {
//         yError() << "GazeboYarpDoubleLaserDriver: cannot find laserBack device";
//         return false;
//     }
//     else
//     {
//         yError() << "****GazeboYarpDoubleLaserDriver: finded laserBack device. OK";
//     }
//
//     if(!m_driver_laserBack->view(m_dev_laserBack))
//     {
//         yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser Back";
//         return false;
//     }
//     else
//     {
//         yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser Back. OK";
//     }
//
//     init();
//     return true;
// }
// bool GazeboYarpDoubleLaserDriver::detachAll()
// {;}


bool GazeboYarpDoubleLaserDriver::getLasersFromGazebo(yarp::os::Searchable& config)
{

    if(!config.check("laserFront"))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laser1 parameter";
        return false;
    }

    if(!config.check("laserBack"))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laser2 parameter";
        return false;
    }


    std::string laserFront_name= config.find("laserFront").asString();
    std::string laserBack_name= config.find("laserBack").asString();
    yError() << "GazeboYarpDoubleLaserDriver: laserFront=" <<  laserFront_name <<" laserBack=" << laserBack_name ;


    m_driver_laserFront = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserFront_name);
    if(m_driver_laserFront == nullptr)
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laserFront device";
        return false;
    }
        else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: finded laserFront device. OK";
    }




    m_driver_laserBack = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserBack_name);
    if(m_driver_laserBack == nullptr)
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot find laserBack device";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: finded laserBack device. OK";
    }


//     //MI SERVER IL PARENT SENSOR???
//     m_parentSensor = dynamic_cast<gazebo::sensors::RaySensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
//     m_started = true;
//
//     if (!m_parentSensor)
//     {
//         yError() << "Error, sensor" <<  sensorScopedName << "was not found" ;
//         return  false ;
//     }
//

    return true;
}


bool GazeboYarpDoubleLaserDriver::open(yarp::os::Searchable& config)
{
    yarp::os::LockGuard guard(m_mutex);

    if(m_onSimulation)
    {
        if(!getLasersFromGazebo(config))
            return false;

        //Connect the driver to the gazebo simulation
        this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpDoubleLaserDriver::onUpdate, this, _1));

        //DEVO AGGIUNGERLO???
        // Connect the onReset method to the WorldReset event callback
        this->m_resetConnection =
        gazebo::event::Events::ConnectWorldReset(boost::bind(&GazeboYarpDoubleLaserDriver::onReset, this));

    }
    else
    {
        //TODO: add code to opens rpLidar devices. Currently return false
        return false;
    }



    
    if(!m_driver_laserFront->view(m_dev_laserFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser front";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser front. OK";
    }
    



    
    if(!m_driver_laserBack->view(m_dev_laserBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: cannot get interface of laser Back";
        return false;
    }
    else
    {
        yError() << "****GazeboYarpDoubleLaserDriver: get interface of laser Back. OK";
    }

    
    if(!init())
        return false;

     //AGGIUNGI QUI ROBA PER LEGGERE LA CONFIGURAZIONE DEI LASER!
    yError() << "****GazeboYarpDoubleLaserDriver: ora dovrei leggere la configurazione!!!! OK";

    return true;
    
}

bool GazeboYarpDoubleLaserDriver::close()
{
    this->m_updateConnection.reset();
    return true;
}

bool GazeboYarpDoubleLaserDriver::init(void )
{

    double minFront, maxFront, minBack, maxBack;
    if(!m_dev_laserFront->getScanLimits(minFront, maxFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting scan limits for front laser";
        return false;
    }

    if(!m_dev_laserBack->getScanLimits(minBack, maxBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting scan limits for back laser";
        return false;
    }

    if( (minFront != minBack) || (maxFront != maxBack) )
    {
        yError() << "GazeboYarpDoubleLaserDriver: front and back laser differ in scan limits";
        return false;
    }

    double resolutionFront, resolutionBack;
    if(!m_dev_laserFront->getHorizontalResolution(resolutionFront))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting resolution for front laser";
        return false;
    }

    if(!m_dev_laserBack->getHorizontalResolution(resolutionBack))
    {
        yError() << "GazeboYarpDoubleLaserDriver: error getting resolution for back laser";
        return false;
    }

    if(resolutionFront != resolutionBack)
    {
        yError() << "GazeboYarpDoubleLaserDriver: front and back laser differ in resolution";
        return false;
    }

    //here I'm sure front and back have same config
    m_samples = (maxFront -minFront) /resolutionFront;

    m_resolution = resolutionFront;
    //TODO: aggiungi qui le verifiche di uaguale configurazione dei due laser.
    //Per ora, nelle funzioni get ritorno le info del front

    m_inited = true;
    yError() << "****GazeboYarpDoubleLaserDriver: inited: resolution=" <<  m_resolution;
    return true;

}

bool GazeboYarpDoubleLaserDriver::getDistanceRange(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getDistanceRange(min, max);
}

bool GazeboYarpDoubleLaserDriver::setDistanceRange(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setDistanceRange not yet implemented");
    return false;
}

bool GazeboYarpDoubleLaserDriver::getScanLimits(double& min, double& max)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getScanLimits(min, max);
    return true;
}

bool GazeboYarpDoubleLaserDriver::setScanLimits(double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setScanLimits not yet implemented");
    return false;
}

bool GazeboYarpDoubleLaserDriver::getHorizontalResolution(double& step)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    return m_dev_laserFront->getHorizontalResolution(step);
}

bool GazeboYarpDoubleLaserDriver::setHorizontalResolution(double step)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setHorizontalResolution not yet implemented");
    return false;
}

bool GazeboYarpDoubleLaserDriver::getScanRate(double& rate)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("getScanRate not yet implemented");
    return true;
}

bool GazeboYarpDoubleLaserDriver::setScanRate(double rate)
{
    yarp::os::LockGuard guard(m_mutex);
    yWarning("setScanRate not yet implemented");
    return false;
}

#define PI 3.14159265

static double convertAngle_user2Hw(double userAngle)
{
    double hwAngle =  userAngle + 90.0;
    if(hwAngle>360)
        hwAngle = hwAngle - 360.0;
    return hwAngle;
}

static double convertAngle_hw2user(double hwAngle)
{
    double userAngle = hwAngle-90.0;
    if(userAngle<0)
        userAngle = userAngle+360.0;
    return userAngle;
}

static double convertAngle_degree2rad(double angle)
{
    return angle*PI/180.0;
}


static double convertAngle_rad2degree(double angle)
{
    return angle*180.0/PI;
}


void GazeboYarpDoubleLaserDriver::calculate(int sensNum, double distance, bool front, int &newSensNum, double &newdistance)
{
    //calculate the input angle in degree
    double angle_input = (sensNum*m_resolution);

    //converto fro user pace to hw space and to rad
    double hw_input_angle = convertAngle_user2Hw(angle_input);
    double angle_rad = convertAngle_degree2rad(hw_input_angle);

    //calculate vertical and horizontal components of input angle
    double Ay = std::abs(sin(angle_rad)*distance);
    double Ax = std::abs(cos(angle_rad)*distance);

    //calculate vertical and horizontal components of new angle with offset. the translation is only on y componet.
    double By, Bx = Ax;
    if(front)
    {
        By = Ay + 0.07;
    }
    else
    {
        By = Ay  + 0.085;
    }

    double betarad = atan(By/Bx);
    double beta = convertAngle_rad2degree(betarad);
    double angle2;

    //atan has codominio = (-PI/2 , PI/2), but since the input is only >0 than the atan output is in (0, PI/2).
    //Now I need to normalize the angle according to the input angle.
    double beta2;

    if(hw_input_angle>=90.0 && hw_input_angle<180)
        beta2=180-beta;
    else if(hw_input_angle >=180 && hw_input_angle <270)
        beta2= beta+180;
    else if(hw_input_angle >=270)
        beta2=360-beta;
    else
        beta2=beta;


    //now I check the difference between the input angle and the calculated angle in order to understand if I need to change the slot (sensorNum)
    double diff=beta2-hw_input_angle;
    newSensNum= sensNum+ round(diff);
    if(newSensNum>=m_samples)
    {
        newSensNum = newSensNum-m_samples;
        yError() << "GazeboYarpDoubleLaserDriver::calculate...something stange has been happened";
    }

    newdistance = std::sqrt((Bx*Bx)+(By*By));


    //------debug stuff----
    std::string res= "==>OK";
    if(std::abs(diff)>m_resolution)
        res="==> change slot!!!";

    std::string laser="front";
    if(!front)
        laser="back";
    //yError() << laser << "ANGLE_in=" << angle_input << "ANGLE=" << angle << "BETA="<<beta  <<"BETA2=" << beta2 <<  "  angle2=" <<angle2 << "DIFF="<< diff<<  res;

    double beta3 = convertAngle_hw2user(beta2);

    //newSensNum=round(beta3/m_resolution); per semplificare i calcoli la calcolo cosi:

    yError() << laser << "INPUT_A=" <<angle_input << "("<<sensNum<<")"<< "ORIGINAL_D=" << distance<< "ORIGINAL_A=" << hw_input_angle << "NEW_DIST="<< newdistance << "NEW_ang=" << beta2  << "OUPUT_A="<< beta3 << "sensnum=" << newSensNum << "DIFF=" << diff <<  res;


}



bool GazeboYarpDoubleLaserDriver::getRawData(yarp::sig::Vector &out)
{
    yarp::os::LockGuard guard(m_mutex);

    if(!m_inited)
        return false;
    yarp::sig::Vector dataFront;
    yarp::sig::Vector dataBack;
    //if(out.size() != m_samples)
        out.resize(m_samples, INFINITY);
    
    if(!m_dev_laserFront->getRawData(dataFront))
        return false;
    if(!m_dev_laserBack->getRawData(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
        double tmp;
        int newindex;
        if(dataFront[i]!= INFINITY)
            calculate(i, dataFront[i], true, newindex, tmp);
        else if(dataBack[i] != INFINITY)
            calculate(i, dataBack[i], false, newindex, tmp);
        else
        {
            tmp = INFINITY;
            newindex = i;
        }
        out[newindex] = tmp;
    }
    
    return true;
}

bool GazeboYarpDoubleLaserDriver::getLaserMeasurement(std::vector<LaserMeasurementData> &data)
{
    yarp::os::LockGuard guard(m_mutex);
    
    if(!m_inited)
        return false;
    std::vector<LaserMeasurementData> dataFront;
    std::vector<LaserMeasurementData> dataBack;
    if(data.size() != m_samples)
        data.resize(m_samples);
    
    if(!m_dev_laserFront->getLaserMeasurement(dataFront))
        return false;
    if(!m_dev_laserBack->getLaserMeasurement(dataBack))
        return false;
    
    for(int i=0; i<m_samples; i++)
    {
        double rhoFront, thetaFront, rhoBack, thetaBack;
        dataFront[i].get_polar(rhoFront, thetaFront);
        dataBack[i].get_polar(rhoBack, thetaBack);
        
        if(rhoFront!= INFINITY)
            data[i].set_polar(rhoFront, thetaFront);
        else  if(rhoBack != INFINITY)
                data[i].set_polar(rhoBack, thetaBack);
        else
            data[i].set_polar(INFINITY, i*m_resolution);
    }

    return true;
}
bool GazeboYarpDoubleLaserDriver::getDeviceStatus(Device_status &status)
{
    yarp::os::LockGuard guard(m_mutex);
    if(!m_inited)
        return false;
    Device_status statusFront, statusBack;
    if(! m_dev_laserFront->getDeviceStatus(statusFront))
        return false;
    if(! m_dev_laserBack->getDeviceStatus(statusBack))
        return false;
    if(statusFront == statusBack)
        status = statusFront;
    else
    {
        //TODO
        yError() << "GazeboYarpDoubleLaserDriver: the status of laser front (" << statusFront << ") differs from the status of laser back (" << statusBack << ")";
        return false;
    }

    return true;
}

bool GazeboYarpDoubleLaserDriver::getDeviceInfo (std::string &device_info)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}

