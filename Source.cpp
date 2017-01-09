// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include <pxcsensemanager.h>  
#include <pxcsession.h>  
#include <iostream>  
#include <string>  
#include <stdio.h>  
#include <windows.h>
#include <pxc3dseg.h>
#include <pxcfacemodule.h>
#include <pxcfaceconfiguration.h>
#include <tchar.h>
#include "SerialClass.h"
#include "json.hpp"
#include "windows.h"

//#include <opencv2\opencv.hpp>    
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"

#define WIDTH 640  
#define HEIGHT 480  

using namespace std;
using json = nlohmann::json;
//using namespace cv;

int main(int argc, char** argv)
{
	PXCSenseManager *psm = 0;
	psm = PXCSenseManager::CreateInstance();
	if (!psm)
	{
		wprintf_s(L"Unabel to create the PXCSenseManager\n");
		return 1;
	}
	pxcStatus sts;

	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, WIDTH, HEIGHT);

	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, WIDTH, HEIGHT);

	psm->EnableFace();  // Enable face tracking

	sts = psm->Init();
	if (sts != PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Unabel to Initializes the pipeline\n");
		return 2;
	}

	PXCImage *colorIm, *depthIm;
	PXCImage::ImageData depth_data, color_data;
	PXCImage::ImageInfo depth_info, color_info;

	PXCFaceModule *m_face = psm->QueryFace();     // Get a face instance (or inside the AcquireFrame/ReleaseFrame loop) for configuration.
	PXCFaceData *fdata = m_face->CreateOutput();        // face is a PXCFaceModule instance
	PXCFaceConfiguration* faceConfiguration = m_face->CreateActiveConfiguration();
	////设置3d跟踪模式
	faceConfiguration->SetTrackingMode(PXCFaceConfiguration::TrackingModeType::FACE_MODE_COLOR_PLUS_DEPTH);
	faceConfiguration->detection.isEnabled = true;
	faceConfiguration->ApplyChanges();
//////////////////////////serial communication///////////////////////
	Serial * SP = new Serial("COM3");
	if (SP->IsConnected())
		printf("We're connected");

	int sendresult = 0;
//////////////////////////////////loop//////////////////////////////
	while (psm->AcquireFrame(true) >= PXC_STATUS_NO_ERROR)

	{
		if (psm->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;

		PXCCapture::Sample *sample = psm->QuerySample();

		colorIm = sample->color;
		depthIm = sample->depth;

		if (colorIm->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &color_data) < PXC_STATUS_NO_ERROR)
			wprintf_s(L"未正常获取彩色图\n");
		if (depthIm->AcquireAccess(PXCImage::ACCESS_READ, &depth_data) < PXC_STATUS_NO_ERROR)
			wprintf_s(L"未正常获取深度图\n");

		depth_info = sample->depth->QueryInfo();
		color_info = sample->color->QueryInfo();

		fdata->Update();

		pxcI32 nfaces = fdata->QueryNumberOfDetectedFaces();      // Get the number of tracked faces
		cout << "current face number:" << nfaces << endl;

		if (nfaces != 0)
		{
			// Retrieve the face landmark data instance
			PXCFaceData::Face *face = fdata->QueryFaceByIndex(0);
			if (face != NULL)
			{
				PXCFaceData::PoseData *pdata = face->QueryPose();
				// retrieve the pose information
				if (pdata != NULL)
				{
					PXCFaceData::PoseEulerAngles angles;
					if (pdata->QueryPoseAngles(&angles))
					{
						//cout << " roll:" << angles.roll << " pitch:" << angles.pitch << " yaw:" << angles.yaw << endl;
                        int roll = angles.roll;
                        int pitch = angles.pitch-10;
                        //roll = 3; pitch = 3;
                        cout << " roll:" << roll << " pitch:" << pitch  << endl;
                        json angle = {
                            { "r", roll },
                            { "p", pitch}
                        };
                        
                        string angleString = angle.dump();  
                        
                        char *p = strdup(angleString.c_str());
						sendresult = SP->WriteData(p, strlen(p)+1);
                        Sleep(1000);
                        if (sendresult > 0) {
                            cout << sendresult << " " << strlen(p) << endl;
                        }
                        
					}
				}
			}
		}

		depthIm->ReleaseAccess(&depth_data);
		colorIm->ReleaseAccess(&color_data);

		psm->ReleaseFrame();

	}
	psm->Release();
}
