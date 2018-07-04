
#if defined(_WIN32) || defined(_WIN64)
#pragma warning( disable : 4819 )
#endif 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/console/parse.h>
#include <string>
#include <pcl/io/boost.h>
#include <pcl/io/pcd_io.h>
#include "MyHdlGrabber.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <typeinfo>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;


typedef PointCloud<PointXYZI> Cloud;
typedef PointCloud<PointXYZI>::Ptr CloudPtr;
typedef PointCloud<PointNormal> NCloud;
typedef PointCloud<PointNormal>::Ptr NCloudPtr;
char buffer[1000];


void
	usage (char ** argv)
{
	cout << "usage: " << argv[0]
	<< " '[-cal <path-to-calibration-file>] [-pcap <path-to-pcap-file>] [-don] [-h | --help]"
		<< endl;
	cout << argv[0] << " -h | --help : shows this help" << endl
		<< "-don will also output Density of Normal filtered clouds files in an inclosed dir */don";
	return;
}

int main (int argc, char ** argv)
{
	CloudPtr cloud_;
	NCloudPtr donCloud(new NCloud);
	std::string hdlCalibration,pcapFile,dirName,datapath,workpath,root;
	std::string year,month,day,hour,minute;
	struct dirent *dirp; 
	char fName[500];
    char datetime[500];
	uint64_t pre_timestamp = -1;
	int second;
	parse_argument (argc, argv, "-pcap", pcapFile);
	parse_argument (argc, argv, "-cal", hdlCalibration);
	
	getcwd(buffer,1000); 
	root = buffer;
	workpath = (root.substr(0,root.find_last_of('/',40))); // ../get_gpstimestamp
	datapath = workpath + pcapFile;
    DIR* dir = opendir(datapath.c_str()); 
	

	if (find_switch (argc, argv, "-h") || 
		find_switch (argc, argv, "--help") ||
		!(find_switch (argc, argv, "-cal") && find_switch (argc, argv, "-pcap")))
	{
		usage (argv);
		return (0);
	}
	
	
    
    while ((dirp = readdir(dir)) != NULL ) 
    {
        if (dirp->d_type == DT_REG)        	
        {
	        
	        printf("%s\n", dirp->d_name);//char
	        std::string filename = "";	  
	        filename = datapath.c_str();
	        filename += dirp->d_name; 
	        dirName =filename.substr(0,filename.find_last_of('.',filename.length()));
	        cout << dirName << endl;
			boost::filesystem::create_directory(dirName.c_str());
			MyHdlGrabber grabber (hdlCalibration,filename);
			grabber.start();
			

			while(grabber.hasClouds())
			{ 
				vector<pair<string, string> > timestamp;
				vector<pair<string, string> >::iterator iter;
				cloud_ = grabber.getNextCloud(timestamp);
				 // end of pcap 
			    if(cloud_->header.stamp == pre_timestamp) break;
			    else pre_timestamp = cloud_->header.stamp;
			    
				for(iter=timestamp.begin();iter!=timestamp.end();iter++)
				{
					if (iter->first == "Y") year = iter->second;
					if (iter->first == "N") month = iter->second;
					if (iter->first == "D") day = iter->second;    
					if (iter->first == "H") hour = iter->second;
					if (iter->first == "M") minute = iter->second;
				}
				second = cloud_->header.stamp/1000000 - atoi(minute.c_str())*60;
			    sprintf(datetime,"20%02d-%02d-%02d-%02d-%02d-%02d-",atoi(year.c_str()),atoi(month.c_str()),atoi(day.c_str()),atoi(hour.c_str()),atoi(minute.c_str()),second);
				sprintf(fName,"%s/%s%lu.pcd",dirName.c_str(),datetime,(cloud_->header.stamp % 1000000) / 1000);
			    cout << "Saving " << fName << " to file...\n";
				pcl::io::savePCDFileBinaryCompressed(fName,*cloud_);  
			   

			}
	     	  
    	}

    }

    closedir(dir);
	return (0);
}
