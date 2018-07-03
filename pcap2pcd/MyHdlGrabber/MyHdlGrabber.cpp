#if defined(_WIN32) || defined(_WIN64)
#pragma warning( disable : 4819 )
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <pcl/io/boost.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include "MyHdlGrabber.h"

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include "string.h"


using namespace std;
using namespace pcl;

double *MyHdlGrabber::cos_lookup_table_ = NULL;
double *MyHdlGrabber::sin_lookup_table_ = NULL;

MyHdlGrabber::MyHdlGrabber(const std::string& correctionsFile,
                             const std::string& pcapFile):
    pcapPtr(NULL)
  , sweepCount(0)
  , cloudCount(0)
  , lastOffset(0)
  , source_address_filter_ ()
  , source_port_filter_ (443)
  , hdl_read_socket_service_ ()
  , hdl_read_socket_ (NULL)
  , my_pcap_file_ (pcapFile)
  , pcap_thread (NULL)
  , running_(false)
  , last_azimuth_ (65000)
  , min_distance_threshold_(2.0) 
  , max_distance_threshold_(10000.0)
{
  initialize (correctionsFile);
}

void MyHdlGrabber::initialize (const std::string& correctionsFile)
{
  if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL)
  {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (int i = 0; i < HDL_NUM_ROT_ANGLES; i++) 
    {
      double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
      cos_lookup_table_[i] = std::cos (rad);
      sin_lookup_table_[i] = std::sin (rad);
    }
  }

  loadCorrectionsFile (correctionsFile);

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
    //printf("[%d]->sinVertCorrection=%f, cosVertOffsetCorrection=%f\n",i,laser_corrections_[i].sinVertCorrection,laser_corrections_[i].cosVertOffsetCorrection);
  }
}


/////////////////////////////////////////////////////////////////////////////
void MyHdlGrabber::loadCorrectionsFile (const std::string& correctionsFile)
{
  boost::property_tree::ptree pt;
  try
  {
    read_xml (correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const&)
  {
    cout << "[pcl::HDLGrabber::loadCorrectionsFile] Error reading calibration file " << correctionsFile <<"\n";
    return;
  }

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child ("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
      {
        if (px.first == "px")
        {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          double azimuth = 0, vertCorrection = 0, distCorrection = 0,
                 vertOffsetCorrection = 0, horizOffsetCorrection = 0;

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
          {
            if (item.first == "id_")
              index = atoi (item.second.data ().c_str ());
            if (item.first == "rotCorrection_")
              azimuth = atof (item.second.data ().c_str ());
            if (item.first == "vertCorrection_")
              vertCorrection = atof (item.second.data ().c_str ());
            if (item.first == "distCorrection_")
              distCorrection = atof (item.second.data ().c_str ());
            if (item.first == "vertOffsetCorrection_")
              vertOffsetCorrection = atof (item.second.data ().c_str ());
            if (item.first == "horizOffsetCorrection_")
              horizOffsetCorrection = atof (item.second.data ().c_str ());
          }
          if (index != -1)
          {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vertCorrection;
            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
          }
        }
      }
    }
  }
}



MyHdlGrabber::~MyHdlGrabber() throw() { stop(); }

bool MyHdlGrabber::hasClouds() { return (running_ && header->len == 1248); }

pcl::PointCloud<pcl::PointXYZI>::Ptr MyHdlGrabber::getNextCloud(set<pair<string, string> > &timestamp)
{
  unsigned char *buff;
  int sz = (header->len-42);
  pcl::PointCloud<pcl::PointXYZI>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZI>);
  buff = (unsigned char*)malloc(sz*sizeof(unsigned char));
  while (sz > 0)
  {
    memcpy(buff,data+42,sz*sizeof(unsigned char));
    if (pcapPtr==NULL || (pcap_next_ex(pcapPtr,&header,&data)==-1 && !terminate_))
    {
      running_ = false;
      break;
    }
    else
    {
      sz = (header->len-42);
    }
   
    if ( completePointCloud((HDLDataPacket*)(buff),myCloud,timestamp)) // return 1 frame
    {
      break;
    }
  }

  free(buff);
  myCloud->header.seq = sweepCount++;
  cloudCount++;
  return myCloud;
}

void MyHdlGrabber::start()
{
  char err[500];
  terminate_ = false;
  
  if (running_)return;
  
  pcapPtr = pcap_open_offline(my_pcap_file_.c_str(),err);
  if (pcapPtr == NULL)
  {
     cout<< "FAILED\n" << err;
  }
  else
  {
    cout << "Opened " << my_pcap_file_ << " successfully...\n";
    if (pcap_next_ex(pcapPtr,&header,&data)!=-1 && !terminate_)
    {
      running_ = true;   
    }
  
  }
}

void MyHdlGrabber::stop ()
{
  terminate_ = true;

  if (pcap_thread != NULL)
  {
    pcap_thread->interrupt ();
    pcap_thread->join ();
    delete pcap_thread;
    pcap_thread = NULL;
  }
}

bool MyHdlGrabber::isRunning() const {return running_;}
std::string MyHdlGrabber::getName() const { return std::string("MyHdlGrabber");}
float MyHdlGrabber::getFramesPerSecond() const { return 10.0f;}


bool MyHdlGrabber::completePointCloud (HDLDataPacket *dataPacket, pcl::PointCloud<pcl::PointXYZI>::Ptr myCloud, 
  set<pair<string, string> > &timestamp)
{
  long int addr = dataPacket->blank2 & 0xFFFF;
  myCloud->header.stamp = dataPacket->gpsTimestamp;  
  //unsigned long int convert to string
  string status_type = "";
  status_type += dataPacket->blank1;
  // long int convert to string
  stringstream stream;  
  stream << addr;  
  string temp = stream.str();

  if(status_type == "Y" || status_type == "N" || status_type == "D" || status_type == "H" || status_type == "M" || status_type == "S")
  {
    if(status_type == "H") // time zone conversion : hour + 8 
    {
      int int_temp = 0;
      int_temp = atoi(temp.c_str());
      int_temp += 8;
      stream.str("");
      stream<<int_temp; 
      temp = stream.str();
    }
    timestamp.insert(pair<string, string>(status_type, temp));
  } 
  

  for (int i = lastOffset/HDL_LASER_PER_FIRING; i < HDL_FIRING_PER_PKT; ++i)
  {
    HDLFiringData firingData = dataPacket->firingData[i]; 

    int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

    for (int j = lastOffset%HDL_LASER_PER_FIRING; j < HDL_LASER_PER_FIRING; j++)
    {
        
      if (firingData.rotationalPosition < last_azimuth_)
      {
        if (myCloud->size () > 0)
        {
          myCloud->is_dense = true;
	        lastOffset = i*HDL_LASER_PER_FIRING + j;
          return true;
        }

      }

      PointXYZI xyzi;

      computeXYZI (xyzi, firingData.rotationalPosition, firingData.laserReturns[j], laser_corrections_[j + offset],j);
      
      if ((boost::math::isnan)(xyzi.x) ||
          (boost::math::isnan)(xyzi.y) ||
          (boost::math::isnan)(xyzi.z)) 
      {
        continue;
      }

      myCloud->push_back (xyzi);

      last_azimuth_ = firingData.rotationalPosition;



    }

  }
   
  lastOffset = 0;
  return false;
}

void MyHdlGrabber::computeXYZI (pcl::PointXYZI& point, int azimuth, 
                              HDLLaserReturn laserReturn, HDLLaserCorrection correction, int lIdx)
{
  double cosAzimuth, sinAzimuth;

  double distanceM = laserReturn.distance * 0.002;

  if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) 
  {
    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    point.intensity = 0;
    return;
  }

  if (correction.azimuthCorrection == 0)
  {
    cosAzimuth = cos_lookup_table_[azimuth];
    sinAzimuth = sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians ((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }

  distanceM += correction.distanceCorrection;

  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;

  point.x = static_cast<float> (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  point.y = static_cast<float> (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  point.intensity = static_cast<uint8_t> (laserReturn.intensity);

}