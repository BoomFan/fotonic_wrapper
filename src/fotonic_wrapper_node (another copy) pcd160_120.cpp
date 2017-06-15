/*
 * Copyright 2012 Fotonic
 *
 * License: Public Domain
 *
 */

//
// example program that shows how to use the FZ-API to control the camera.
// the code
//  - connect to the first camera found
//  - get some version info and print it
//  - set some parameters
//  - start the camera
//  - get a number of frames
//  - save one image to disc in 3DD format
//  - stop the camera
//  - disconnect and exit
//

//standard includes
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "std_msgs/String.h"

//opencv includes
#include <cv.h>
#include <highgui.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//ROS Includes
#include <ros/ros.h>

//FOTONIC Includes
#include "../include/fz_api.h"

//pcl Includes
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#define MAX_DEVICES 40

using namespace cv;
std::string image_topic_name = "fotonic/image";
std::string image_undistort_topic_name = "fotonic/image/undistort";
std::string pcd_topic_name = "/fotonic/point_cloud/cloud";
std::string pcd_unfiltered_topicname = "/fotonic/unfiltered/point_cloud/cloud";
std::string pcd_forplane_topicname  = "/fotonic/forplane/point_cloud/cloud";
std::string pcd_planed_topicname = "/fotonic/planed/point_cloud/cloud";
std::string fotonic_link_name = "fotonic_link";


// write image data to disc, replayable by 3DDisplay
bool SaveImageAs3DD(short *pPixels, FZ_FRAME_HEADER *pstFrmHdr)
{
	// writes one image to one .3dd file (binary header+raw intel fmt)
	bool bResult = false;
	int n;
	int iWidth = pstFrmHdr->ncols;
	int iHeight = pstFrmHdr->nrows;
	int iPixelSize = pstFrmHdr->bytesperpixel;

	char szFilename[256];
	sprintf(szFilename, "fz_example.3dd");

	FILE *pFile = fopen(szFilename, "wb");
	if(!pFile) return false;

	// write header as it is
	n = (int)fwrite(pstFrmHdr, sizeof(FZ_FRAME_HEADER), 1, pFile);
	if(n!=1) goto error;

	// write image data as it is
	n = (int)fwrite(pPixels, iWidth*iHeight*iPixelSize, 1, pFile);
	if(n!=1) goto error;

	bResult = true;
error:
	fclose(pFile);
	return bResult;
}


FZ_FRAME_HEADER stFrameHdr;
short aImage[640*480*4];

void FZLogCallback(char *szMetadata, char *szMessage)
{
	printf("LOG CALLBACK: [%s] %s\n", szMetadata, szMessage);
}

int main(int argc, char** argv)
{
	FZ_DEVICE_INFO aDeviceInfo[MAX_DEVICES];
	FZ_Result iResult;
	FZ_Device_Handle_t hDevice;
	FZ_CmdRespCode_t iRespCode;
	int iRespBytes;
	char szInfo[128];
	uint16_t iDeviceType;
	uint16_t iMode;
	uint16_t iShutterMs10 = 20; //2.0ms
	uint16_t iFPS = 40;
	bool bSaveOK;
    Mat myimage(120,160, CV_8UC3);
    Mat myundistortimage(120,160, CV_8UC3);


    //Initialize parameters for camera distortion correction
	Mat intrinsic = Mat(3, 3, CV_32FC1); 
	intrinsic.at<float>(0, 0) = 147.006565; 
	intrinsic.at<float>(0, 1) = 0; 
	intrinsic.at<float>(0, 2) = 79.674319; 
	intrinsic.at<float>(1, 0) = 0; 
	intrinsic.at<float>(1, 1) = 145.336332; 
	intrinsic.at<float>(1, 2) = 67.733072; 
	intrinsic.at<float>(2, 0) = 0; 
	intrinsic.at<float>(2, 1) = 0; 
	intrinsic.at<float>(2, 2) = 1; 

	Mat distcoeffs = Mat(5,1,CV_32FC1); 
	distcoeffs.at<float>(0,0) = -0.534127; 
	distcoeffs.at<float>(1,0) = 0.320289; 
	distcoeffs.at<float>(2,0) = -0.008662; 
	distcoeffs.at<float>(3,0) = 0.000015; 
	distcoeffs.at<float>(4,0) = 0.000000; 

	sensor_msgs::CameraInfo info_fotonic;

	// init FZ_API
	FZ_Init();

	// set desired FZ-API logging
	int iFlags = FZ_LOG_TO_STDOUT;
	iFlags |= FZ_LOG_ERROR | FZ_LOG_WARN | FZ_LOG_INFO;
	FZ_SetLogging(iFlags, NULL, FZLogCallback);

	// find connected sensors (enumerate)
	int iNumDevices = MAX_DEVICES;
	iResult = FZ_EnumDevices2(aDeviceInfo, &iNumDevices);
	if(iResult==FZ_TOO_MANY_DEVICES) iResult = FZ_Success;
	if(iResult!=FZ_Success || iNumDevices<1) {
		if(iResult!=FZ_Success) printf("ERROR: FZ_EnumDevices2 code 0x%02x\n", iResult);
		else printf("ERROR: No FZ devices are found\n");
	}
	printf("Found %d FZ devices\n", iNumDevices);

	// at least one device is found, open
	iResult = FZ_Open(aDeviceInfo[0].szPath, 0, &hDevice);
	if(iResult!=FZ_Success) {
		printf("ERROR: FZ_Open (code 0x%02x)\n", iResult);
	}

	// get device type
	iRespBytes = sizeof(iDeviceType);
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_GET_DEVICE_TYPE, NULL, 0,
		&iRespCode, &iDeviceType, &iRespBytes);
	if(iRespCode != (int)R_CMD_DE_ACK) iDeviceType = FZ_DEVICE_TYPE_JAGUAR;
	printf("DeviceType: %d\n", iDeviceType);

	// get and print info strings
	iRespBytes = 127;
	iResult = FZ_IOCtl(hDevice,
		CMD_API_GET_VERSION, NULL, 0,
		&iRespCode, szInfo, &iRespBytes);
	if(iResult!=FZ_Success);
	printf("FZ_API version: %s\n", szInfo);
	iRespBytes = 127;
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_GET_VERSION, NULL, 0,
		&iRespCode, szInfo, &iRespBytes);
	if(iResult!=FZ_Success);
	printf("DE version: %s\n", szInfo);
	iRespBytes = 127;
	iResult = FZ_IOCtl(hDevice,
		CMD_CA_GET_VERSION, NULL, 0,
		&iRespCode, szInfo, &iRespBytes);
	if(iResult!=FZ_Success);
	printf("CA version: %s\n", szInfo);
	iRespBytes = 127;
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_GET_PCODE, NULL, 0,
		&iRespCode, szInfo, &iRespBytes);
	if(iResult!=FZ_Success);
	printf("Product code: %s\n", szInfo);
	iRespBytes = 127;
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_GET_UNIT_NO, NULL, 0,
		&iRespCode, szInfo, &iRespBytes);
	if(iResult!=FZ_Success);
	printf("S/N: %s\n", szInfo);

	// set mode
	if(iDeviceType==FZ_DEVICE_TYPE_JAGUAR)          iMode = DE_MODE_BM_TEMPORAL;
	else if(iDeviceType==FZ_DEVICE_TYPE_PANASONIC)  iMode = DE_MODE_PA_Z; 
	else if(iDeviceType==FZ_DEVICE_TYPE_PRIMESENSE) iMode = DE_MODE_640X480_30;
	else if(iDeviceType==FZ_DEVICE_TYPE_PRIMESENSE_C)  iMode = DE_MODE_640X480_640X480; 
	else if(iDeviceType==FZ_DEVICE_TYPE_PRIMESENSE_N) iMode = DE_MODE_640X480_640X480;
	else {
		printf("ERROR: Unknown DeviceType (%d)\n", iDeviceType);
	}
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_SET_MODE, &iMode, sizeof(iMode),
		&iRespCode, NULL, NULL);
	if( iResult!=FZ_Success ) {
		printf("ERROR: FZ_IOCtl CMD_DE_SET_MODE failed (code 0x%02x)\n", iResult);
	}
	// set shutter
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_SET_SHUTTER, &iShutterMs10, sizeof(iShutterMs10),
		&iRespCode, NULL, NULL);
	if( iResult!=FZ_Success ) {
		printf("ERROR: FZ_IOCtl CMD_DE_SET_SHUTTER failed (code 0x%02x)\n", iResult);
	}
	// set frame rate
	iResult = FZ_IOCtl(hDevice,
		CMD_DE_SET_FPS, &iFPS, sizeof(iFPS),
		NULL, NULL, NULL);
	if( iResult!=FZ_Success ) {
		printf("ERROR: FZ_IOCtl CMD_DE_SET_FPS failed (code 0x%02x)\n", iResult);
	}

	// start sensor
	iResult = FZ_IOCtl(hDevice, CMD_DE_SENSOR_START, NULL, 0, NULL, NULL, NULL);
	if( iResult!=FZ_Success ) {
		printf("ERROR: FZ_IOCtl CMD_DE_SENSOR_START failed (code 0x%02x)\n", iResult);
	}

	
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	//image_transport::CameraPublisher pub1 = it.advertiseCamera("fotonic/image", 1);
	image_transport::Publisher pub1 = it.advertise(image_topic_name, 1);
	image_transport::Publisher pub2 = it.advertise(image_undistort_topic_name, 1);
	// cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointCloud<pcl::PointXYZI> cloud_unfiltered;
	ros::NodeHandle nh2;

	
	ros::Publisher pub_cloud = nh2.advertise<pcl::PointCloud<pcl::PointXYZI> >(pcd_topic_name, 1);
	ros::Publisher pub_cloud_unfiltered = nh2.advertise<pcl::PointCloud<pcl::PointXYZI> >(pcd_unfiltered_topicname, 1);
	ros::Publisher pub_cloud_forplane = nh2.advertise<pcl::PointCloud<pcl::PointXYZI> >(pcd_forplane_topicname, 1);
	ros::Publisher pub_cloud_planed = nh2.advertise<pcl::PointCloud<pcl::PointXYZI> >(pcd_planed_topicname, 1);
	
	bool p_filter_;
	nh2.param<bool>("use_filter", p_filter_, true);


	// Initialize plane segmentation parameters to the filtered pointclouds. Tutorial:
	// http://pointclouds.org/documentation/tutorials/planar_segmentation.php
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	// // Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	// // Optional
  	seg.setOptimizeCoefficients (true);
  	// Mandatory
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	// seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  	
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setDistanceThreshold (0.01);


	// publish ROS topics in the while loop
	ros::Rate loop_rate(30);
	while (nh.ok()) {
		ros::Time tic = ros::Time::now();

		// get images
		for(int i=0; i<1; i++) {
			size_t iBufSize = sizeof(aImage);
			iResult = FZ_GetFrame(hDevice, &stFrameHdr, aImage, &iBufSize);
			if( iResult!=FZ_Success ) {
				printf("ERROR: FZ_GetFrame failed (code 0x%02x)\n", iResult);
	            FZ_Exit();
			}
			// printf("Got frame %d\n", i);
			
			//Get temperature every 40'th frame
			
			// if (i%40 == 0){
			// 	int iTempLED = 0;		
			// 	iRespBytes = sizeof(short);
			// 	iResult = FZ_IOCtl(hDevice,CMD_DE_GET_LED_TEMP, NULL, 0,&iRespCode, &iTempLED, &iRespBytes);
			// 	if(iRespCode != (int)R_CMD_DE_ACK) iTempLED = 0;
				
			// 	printf("Temp : %d\n", iTempLED);

			// }

		}
		ros::Time toc = ros::Time::now();
		ros::Duration diff = toc - tic;
	    std::cout <<"Getting data from sensor took: "<< diff <<" seconds" << std::endl;

		tic = ros::Time::now();
		// save the last image on disc
		float maxz=0;
	    for (int k = 0; k < 120; k++)
	        for (int j=0; j < 160; j++)
	        {
	   	//Data of aImage. The channels are in the order Brightness,Z,X,Y. The channels are 16 bit integers. On most platforms that corresponds to "short int".
		//The B and Z channels are 16 bit unsigned, while X and Y are 16 bit signed, with 0 in the center of the image.
		//All X, Y and Z distances are measured in millimeters. 
		//The maximum value that a 16 bit integer can have is 65535, but the camera will not return any values higher than around 9000 mm.
		//The brightness has a maximum value of 2046.
	        	
	        	int tempvB = aImage[k*4*160+j];
	            int tempvZ = aImage[k*4*160+160+j];
	            int tempvX = aImage[k*4*160+2*160+j];
	            int tempvY = aImage[k*4*160+3*160+j];
	            if(maxz<(float)tempvZ/1000){
					maxz=(float)tempvZ/1000;
				}

				//As it is said abouve, the value scope of brightness is 0--2046
	            float color = tempvB*255/2046;
	            if(color>128){
	                color = (color-128)/3+128;
	            }
	            else if(color<121){
	                color = sqrt(color)*11;
	            }
	            myimage.at<Vec3b>(k,j)[0]= color;  
	            myimage.at<Vec3b>(k,j)[1] = color;
	            myimage.at<Vec3b>(k,j)[2] = color;
	        }

	    ROS_INFO("Max z(camera axis) = %f", maxz);
	    cv::flip(myimage, myimage, 0);  
		 // imwrite( "../images/Image.jpg", myimage ); //No need

		 // namedWindow( "Image", CV_WINDOW_AUTOSIZE ); //No need
		 // imshow( "Image", myimage ); //No need
	    ros::Time now = ros::Time::now();
	    undistort(myimage, myundistortimage, intrinsic, distcoeffs); 
	    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", myimage).toImageMsg();
		// Get current time
		msg1->header.stamp = now;
	    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", myundistortimage).toImageMsg();
		msg2->header.stamp = now;

		toc = ros::Time::now();
		diff = toc - tic;
	    std::cout <<"Publishing sensor image took: "<< diff <<" seconds" << std::endl;



	    
		// Convert aImage to point cloud
	    tic = ros::Time::now();
		//Creat pointcloud for original sensor data
		cloud = pcl::PointCloud<pcl::PointXYZI>();
		cloud.width = 160;
		cloud.height = 120;
		cloud.points.resize(160 * 120);

		//Creat pointcloud for filtered sensor data
		cloud_unfiltered = pcl::PointCloud<pcl::PointXYZI>();
		cloud_unfiltered.width = 160;
		cloud_unfiltered.height = 120;
		cloud_unfiltered.points.resize(160 * 120);

		//Creat pointcloud for plane segmented sensor data
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_forplane(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_forplane->width = 160;
		cloud_forplane->height = 120;
		cloud_forplane->points.resize(160 * 120);
		
		

		// Now start to put sensor data into pointclouds data
		//Image lists fields by row. Don't ask me why.
		int ct = 0;
		float dist = 0;
		float pt_x = 0;
		float pt_y = 0;
		float pt_z = 0;
		float pt_b = 0;
		for(int i = 0; i < 120; i++){
			for(int j = 0; j < 160; j++){
				pt_b =   (float) aImage[i*4*160 + j]; 				//Brightness
				pt_y = -0.001*(float) aImage[i * 4 * 160 + 160 + j];			//camera_Z
				pt_x = -0.001*(float) aImage[i * 4 * 160  + 2 * 160 + j] ;	//camera_X
				pt_z = 0.001*(float) aImage[i * 4 * 160  + 3 * 160 + j] ;		//camera_Y

				if (p_filter_) {		//If a point is too close to the camera but too dark, then we say it is noise.
					dist = pt_y*pt_y + pt_x*pt_x + pt_z*pt_z ;
					if (dist > 1 && pt_y < 0){ 		//Keep those points in a larger range and also in front of the camera.
						cloud.points[ct].intensity =  pt_b; 				//Brightness
						cloud.points[ct].y = pt_y;							//camera_Z
						cloud.points[ct].x = pt_x;							//camera_X
						cloud.points[ct].z = pt_z;							//camera_Y

						cloud_forplane->points[ct].y = pt_y;							//camera_Z
						cloud_forplane->points[ct].x = pt_x;							//camera_X
						cloud_forplane->points[ct].z = pt_z;							//camera_Y
						// cloud_forplane->points.push_back (pcl::PointXYZ(pt_x, pt_y, pt_z));
					}
					else if(pt_b>300 && pt_y < 0.05){	//Keep those points in a closer range but with high brightness and also in front of the camera.
						cloud.points[ct].intensity =  pt_b; 				//Brightness
						cloud.points[ct].y = pt_y;							//camera_Z
						cloud.points[ct].x = pt_x;							//camera_X
						cloud.points[ct].z = pt_z;							//camera_Y

						cloud_forplane->points[ct].y = pt_y;							//camera_Z
						cloud_forplane->points[ct].x = pt_x;							//camera_X
						cloud_forplane->points[ct].z = pt_z;							//camera_Y
						// cloud_forplane->points.push_back (pcl::PointXYZ(pt_x, pt_y, pt_z));

					}
					// else{
					// 	ROS_INFO("One noise has been filtered out");
					// }
					cloud_unfiltered.points[ct].intensity =  pt_b; 				//Brightness
					cloud_unfiltered.points[ct].y = pt_y;							//camera_Z
					cloud_unfiltered.points[ct].x = pt_x;							//camera_X
					cloud_unfiltered.points[ct].z = pt_z;							//camera_Y
				}
				else{
					cloud.points[ct].intensity =  pt_b; 				//Brightness
					cloud.points[ct].y = pt_y;							//camera_Z
					cloud.points[ct].x = pt_x;							//camera_X
					cloud.points[ct].z = pt_z;							//camera_Y

					cloud_forplane->points[ct].y = pt_y;							//camera_Z
					cloud_forplane->points[ct].x = pt_x;							//camera_X
					cloud_forplane->points[ct].z = pt_z;							//camera_Y
					// cloud_forplane->points.push_back (pcl::PointXYZ(pt_x, pt_y, pt_z));

				}
				
				ct++;

			}
		}
		// cloud_forplane->width = ct;
		// cloud_forplane->height = 1;
		// cloud_forplane->points.resize(ct * 1);
		toc = ros::Time::now();
		diff = toc - tic;
	    std::cout <<"Publishing pointclouds took: "<< diff <<" seconds" << std::endl;


		// Fit pointclouds into a Plane model. Tutorial:
		// http://pointclouds.org/documentation/tutorials/planar_segmentation.php
		tic = ros::Time::now();

		//Creat pointcloud for plane-segmented sensor data
		pcl::PointCloud<pcl::PointXYZI> cloud_planed;
		cloud_planed = pcl::PointCloud<pcl::PointXYZI>();
		cloud_planed.width = 160;
		cloud_planed.height = 120;
		cloud_planed.points.resize(160 * 120);
		// cloud_planed.width = ct;
		// cloud_planed.height = 1;
		// cloud_planed.points.resize(ct * 1);


		seg.setInputCloud (cloud_forplane);
	  	seg.segment (*inliers, *coefficients);

	  	if (inliers->indices.size () == 0)
	  	{
	    	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	    	return (-1);
	  	}

	  	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
	                                      << coefficients->values[1] << " "
	                                      << coefficients->values[2] << " " 
	                                      << coefficients->values[3] << std::endl;

	    int pt_index;
	  	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	  	for (size_t i = 0; i < inliers->indices.size (); ++i){
	  		pt_index = inliers->indices[i];
	  		cloud_planed.points[pt_index].intensity =  4000; 
	  		cloud_planed.points[pt_index].x = cloud_forplane->points[pt_index].x;
	  		cloud_planed.points[pt_index].y = cloud_forplane->points[pt_index].y;
	  		cloud_planed.points[pt_index].z = cloud_forplane->points[pt_index].z;
		  	// std::cerr << "pt index:" << pt_index << "   x,y,z =  " << cloud_forplane->points[pt_index].x << " "
     //                                       << cloud_forplane->points[pt_index].y << " "
     //                                       << cloud_forplane->points[pt_index].z << std::endl;
	  	}
	  	toc = ros::Time::now();
		diff = toc - tic;
	    std::cout <<"Fitting plane model took: "<< diff <<" seconds" << std::endl << std::endl;


		// Publish all the pointclouds into ROS
		pcl_conversions::toPCL(now, cloud.header.stamp);
		pcl_conversions::toPCL(now, cloud_unfiltered.header.stamp);
		pcl_conversions::toPCL(now, cloud_forplane->header.stamp);
		pcl_conversions::toPCL(now, cloud_planed.header.stamp);
		cloud.header.frame_id = fotonic_link_name;
		cloud_unfiltered.header.frame_id = fotonic_link_name;
		cloud_forplane->header.frame_id = fotonic_link_name;
		cloud_planed.header.frame_id = fotonic_link_name;
		pub_cloud.publish(cloud);
		pub_cloud_unfiltered.publish(cloud_unfiltered);
		pub_cloud_forplane.publish(*cloud_forplane);
		pub_cloud_planed.publish(cloud_planed);

		

		pub1.publish(msg1);
		  // pub1.CameraPublisher::publish(msg1,info_fotonic,t);//this is wrong
		pub2.publish(msg2);
		ros::spinOnce();
		loop_rate.sleep();
	}





// 	bSaveOK = SaveImageAs3DD(aImage, &stFrameHdr);
// 	if(!bSaveOK) {
// 		printf("ERROR: SaveImageAs3DD failed\n");
// 		goto error;
// 	}

// 	// stop sensor
// 	FZ_IOCtl(hDevice, CMD_DE_SENSOR_STOP, NULL, 0, NULL, NULL, NULL);
// 	if( iResult!=FZ_Success ) {
// 		printf("ERROR: FZ_IOCtl CMD_DE_SENSOR_STOP failed (code 0x%02x)\n", iResult);
// 		goto error;
// 	}

	// disconnect
	FZ_Close(hDevice);

	// proper exit of FZ_API
	FZ_Exit();

// 	printf("ALL DONE OK\n");
// 	return 0;
// error:
// 	// proper exit of FZ_API
// 	FZ_Exit();

	// printf("Some error occured\n");
	// return -1;
}
