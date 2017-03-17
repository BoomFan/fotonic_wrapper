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





#define MAX_DEVICES 40

using namespace cv;

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
	image_transport::Publisher pub1 = it.advertise("fotonic/image", 1);
	image_transport::Publisher pub2 = it.advertise("fotonic/image/undistort", 1);
	// cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);

	pcl::PointCloud<pcl::PointXYZI> cloud;
	ros::NodeHandle nh2;

	
	ros::Publisher pub_cloud = nh2.advertise<pcl::PointCloud<pcl::PointXYZI> >("/fotonic/point_cloud/cloud", 1);
	
	

	// publish ROS topics
	ros::Rate loop_rate(30);
		 while (nh.ok()) {

	// get images
	for(int i=0; i<1; i++) {
		size_t iBufSize = sizeof(aImage);
		iResult = FZ_GetFrame(hDevice, &stFrameHdr, aImage, &iBufSize);
		if( iResult!=FZ_Success ) {
			printf("ERROR: FZ_GetFrame failed (code 0x%02x)\n", iResult);
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
	

		//ros::Time t = ros::Time::now(); // Get current time
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
            myimage.at<Vec3b>(k,j)[0]= tempvB*255/2046;  
            myimage.at<Vec3b>(k,j)[1] = tempvB*255/2046;
            myimage.at<Vec3b>(k,j)[2] = tempvB*255/2046;
        }
        printf("\n max z=%f", maxz);
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



		//Convert image to point cloud
		cloud = pcl::PointCloud<pcl::PointXYZI>();
		cloud.width = 160;
		cloud.height = 120;
		cloud.points.resize(160 * 120);
		
		
		//Image lists fields by row. Don't ask me why.
		int ct = 0;
		
		for(int i = 0; i < 120; i++){
			for(int j = 0; j < 160; j++){
			cloud.points[ct].intensity =   (float) aImage[i*4*160 + j]; 				//Brightness
			cloud.points[ct].y = -0.001*(float) aImage[i * 4 * 160 + 160 + j];			//camera_Z
			cloud.points[ct].x = -0.001*(float) aImage[i * 4 * 160  + 2 * 160 + j] ;	//camera_X
			cloud.points[ct].z = 0.001*(float) aImage[i * 4 * 160  + 3 * 160 + j] ;		//camera_Y
			
			ct++;

			}
		}

		pcl_conversions::toPCL(now, cloud.header.stamp);
		cloud.header.frame_id = "fotonic_link";
		pub_cloud.publish(cloud);
		

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
