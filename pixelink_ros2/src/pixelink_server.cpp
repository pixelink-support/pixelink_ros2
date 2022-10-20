#include "PixeLINKApi.h"
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "pixelink_ros2_interfaces/srv/no_params.hpp"
#include "pixelink_ros2_interfaces/srv/set_params.hpp"
#include "pixelink_ros2_interfaces/srv/set_params_roi.hpp"
#include "pixelink_ros2_interfaces/srv/get_params.hpp"
#include "pixelink_ros2_interfaces/srv/get_params_roi.hpp"
#include "pixelink_ros2_interfaces/srv/get_serial_numbers.hpp"
#include "pixelink_ros2_interfaces/srv/serial_param.hpp"
#include "pixelink_ros2_interfaces/srv/get_camera_info.hpp"
#include "pixelink_ros2_interfaces/srv/get_params_pixel_addressing.hpp"
#include "pixelink_ros2_interfaces/srv/set_params_pixel_addressing.hpp"
#include "pixelink_ros2_interfaces/srv/capture_params.hpp"
#include "pixelink_ros2_interfaces/srv/capture_video_params.hpp"
#include "pixelink_ros2_interfaces/srv/get_supported_pixel_formats.hpp"
#include "pixelink_ros2_interfaces/srv/get_pixel_format.hpp"
#include "pixelink_ros2_interfaces/srv/set_pixel_format.hpp"
#include "pixelink_ros2_interfaces/srv/set_trigger_params.hpp"
#include "pixelink_ros2_interfaces/srv/set_gpio_params.hpp"
#include "pixelink_ros2_interfaces/srv/get_range.hpp"
#include "pixelink_ros2_interfaces/srv/get_roi_range.hpp"

#include "image_transport/image_transport.hpp"
#include "pixelinkHelper.h"
#include "pixelFormat.h"

//#include "sensor_msgs/Image.h"
//#include "sensor_msgs/msg/imu.hpp"

HANDLE hCamera = 0;
bool isStreaming = false;
HWND previewHandle;
std::string encodingString;

int captureAttempts = 0;
std::string saveImageName;
bool captureAsBmp = false;
bool captureAsJpg = false;
bool captureAsTiff = false;
bool captureAsPsd = false;
bool captureAsRaw = false;


void setReturnString(int * rc, std::string * returnString)
{
	if (API_SUCCESS(*rc))
	{
		*returnString = "Success";
	}
	else
	{
		ERROR_REPORT errorReport;
		PxLGetErrorReport(hCamera,&errorReport);
		int length = sizeof(*returnString) / sizeof(S8);
		char array[length]  = { }; 
		for (int i=0;i<length;i++)
		{
			array[i] = errorReport.strReturnCode[i];
		}
		std::string s(array);
		*returnString = s;
	}
}

//get value
PXL_RETURN_CODE getValueSimple (ULONG feature, float* value)
{
    PXL_RETURN_CODE rc = ApiSuccess;
    float featureValue;
    ULONG flags;
    ULONG numParams = 1;

    rc = PxLGetFeature (hCamera, feature, &flags, &numParams, &featureValue);
    if (!API_SUCCESS(rc)) return rc;

    *value = featureValue;

    return ApiSuccess;
}

//set value
PXL_RETURN_CODE setValueSimple (ULONG feature, float value)
{
    PXL_RETURN_CODE rc = ApiSuccess;

	if (isStreaming && !(pixelinkHelper::IsSettableWhileStreaming((U32)feature)))
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop Stream to set feature");
		PxLSetStreamState(hCamera,STOP_STREAM);
	}

	rc = PxLSetFeature (hCamera, feature, FEATURE_FLAG_MANUAL, 1, &value);
    
    	if (isStreaming && !(pixelinkHelper::IsSettableWhileStreaming((U32)feature)))
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Restart stream");
		PxLSetStreamState(hCamera,START_STREAM);
	}

    return rc;
}



void initialize(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	HANDLE handle;
	PXL_RETURN_CODE rc = PxLInitialize(0, &handle);
	response->return_code = rc;
	
	if (API_SUCCESS(rc))
	{
		hCamera = handle;
		response->return_string = "Success";
	}
	else
	{
		ERROR_REPORT errorReport;
		PxLGetErrorReport(hCamera,&errorReport);
		int length = sizeof(errorReport.strReturnCode) / sizeof(S8);
		char array[length]  = { }; 
		for (int i=0;i<length;i++)
		{
			array[i] = errorReport.strReturnCode[i];
		}
		std::string s(array);
		response->return_string = s;
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialize Camera: %s",response->return_string.c_str());
}



void initializeWithSerial(const std::shared_ptr<pixelink_ros2_interfaces::srv::SerialParam::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SerialParam::Response>       response)
{
	HANDLE handle;
	PXL_RETURN_CODE rc = PxLInitialize(request->serial, &handle);
	response->return_code = rc;
	
	if (API_SUCCESS(rc))
	{
		hCamera = handle;
		response->return_string = "Success";
	}
	else
	{
		ERROR_REPORT errorReport;
		PxLGetErrorReport(hCamera,&errorReport);
		int length = sizeof(errorReport.strReturnCode) / sizeof(S8);
		char array[length]  = { }; 
		for (int i=0;i<length;i++)
		{
			array[i] = errorReport.strReturnCode[i];
		}
		std::string s(array);
		response->return_string = s;
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialize Camera: %s",response->return_string.c_str());
}

void uninitialize(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLUninitialize(hCamera);
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Uninitialize Camera: %s",response->return_string.c_str());
}

void getSerialNumbers(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetSerialNumbers::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetSerialNumbers::Response>       response)
{
	U32 numCameras;

	PXL_RETURN_CODE rc = PxLGetNumberCamerasEx(NULL, &numCameras);
	
	if (API_SUCCESS(rc)) 
	{
		U32* pSerialNumbers = (U32*)malloc(sizeof(U32) * numCameras);

		if (NULL != pSerialNumbers) 
		{
			PxLGetNumberCameras(&pSerialNumbers[0], &numCameras);
		    
			std::vector<int> vect;

			for (int i=0;i<numCameras;i++)
			{
			vect.push_back(pSerialNumbers[i]);
			}
		    
			response->serials = vect;
			
			response->return_string = "Success";
		}
 	}
 	else
	{
		ERROR_REPORT errorReport;
		PxLGetErrorReport(hCamera,&errorReport);
		int length = sizeof(errorReport.strReturnCode) / sizeof(S8);
		char array[length]  = { }; 
		for (int i=0;i<length;i++)
		{
			array[i] = errorReport.strReturnCode[i];
		}
		std::string s(array);
		response->return_string = s;
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Camera Serial Numbers: %s",response->return_string.c_str());
}

std::string ToString(S8* text, int length)
{
	char array[length]  = { }; 
	for (int i=0;i<length;i++)
	{
		array[i] = text[i];
	}
	std::string s(array);	
	return s;
}

void getCameraInfo(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetCameraInfo::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetCameraInfo::Response>       response)
{
	CAMERA_INFO cameraInfo;
	
	PXL_RETURN_CODE rc = PxLGetCameraInfoEx(hCamera, &cameraInfo, sizeof(CAMERA_INFO));
	
	response->return_code = rc;
	
	if (API_SUCCESS(rc))
	{
		response->camera_name = ToString(cameraInfo.CameraName,256);
		
		response->description = ToString(cameraInfo.Description,256);
		response->firmware_version = ToString(cameraInfo.FirmwareVersion,12);
		response->bootload_version = ToString(cameraInfo.BootloadVersion,12);
		response->fpga_version = ToString(cameraInfo.FPGAVersion,12);
		response->xml_version = ToString(cameraInfo.XMLVersion,12);
		response->model_name = ToString(cameraInfo.ModelName,33);
		response->serial_number = ToString(cameraInfo.SerialNumber,33);
		response->vendor_name = ToString(cameraInfo.VendorName,33);

		response->return_string = "Success";
	}
	else
	{
		ERROR_REPORT errorReport;
		PxLGetErrorReport(hCamera,&errorReport);
		int length = sizeof(errorReport.strReturnCode) / sizeof(S8);
		char array[length]  = { }; 
		for (int i=0;i<length;i++)
		{
			array[i] = errorReport.strReturnCode[i];
		}
		std::string s(array);
		response->return_string = s;
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Camera Info: %s",response->return_string.c_str());
}


void startStream(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{

	PXL_RETURN_CODE rc = PxLSetStreamState(hCamera,START_STREAM);
	response->return_code = rc;
	
	if (API_SUCCESS(rc))
	{
		isStreaming = true;
		
		int numberOfParams = 4;
		float parms[numberOfParams];
		U32 flags = 0;
		U32 numParams = numberOfParams;
		PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_FORMAT, &flags, &numParams, &parms[0]);
		
		encodingString = PxLPixelFormat::toSensorMessageEncoding(parms[0]);
	}
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Stream: %s",response->return_string.c_str());
}

void stopStream(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLSetStreamState(hCamera,STOP_STREAM);
	response->return_code = rc;
	
	if (API_SUCCESS(rc))
	{
		isStreaming = false;
	}
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop Stream: %s",response->return_string.c_str());
}

void startPreview(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLSetPreviewState(hCamera, START_PREVIEW, &previewHandle);
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Preview: %s",response->return_string.c_str());
}

void stopPreview(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLSetPreviewState(hCamera, STOP_PREVIEW, &previewHandle);
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop Preview: %s",response->return_string.c_str());
}

void captureImageAsBmp(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	saveImageName = request->file_name.c_str();
	
	if (saveImageName.find(".bmp") == std::string::npos)
	{
		saveImageName = saveImageName + ".bmp";
	}

	captureAsBmp = true;
}

void captureImageAsTiff(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	saveImageName = request->file_name.c_str();
	
	if (saveImageName.find(".tiff") == std::string::npos)
	{
		saveImageName = saveImageName + ".tiff";
	}

	captureAsTiff = true;
}

void captureImageAsPsd(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	saveImageName = request->file_name.c_str();
	
	if (saveImageName.find(".psd") == std::string::npos)
	{
		saveImageName = saveImageName + ".psd";
	}

	captureAsPsd = true;
}

void captureImageAsJpg(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	saveImageName = request->file_name.c_str();
	
	if (saveImageName.find(".jpg") == std::string::npos)
	{
		saveImageName = saveImageName + ".jpg";
	}

	captureAsJpg = true;
}

void captureImageAsRaw(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	saveImageName = request->file_name.c_str();
	
	if (saveImageName.find(".bin") == std::string::npos)
	{
		saveImageName = saveImageName + ".bin";
	}

	captureAsRaw = true;
}

void captureVideoAsAvi(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureVideoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureVideoParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	std::string testString = request->file_name.c_str();

	if (testString.find(".avi") == std::string::npos)
	{
		testString = testString + ".avi";
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Capture Video As Avi");

	float pixelFormat;
	
	PXL_RETURN_CODE rc = getValueSimple(FEATURE_PIXEL_FORMAT, &pixelFormat);
	
	if (pixelFormat != PIXEL_FORMAT_MONO8 && pixelFormat != PIXEL_FORMAT_YUV422)
	{
		response->return_code = -1;
		response->return_string = "Error: Pixel format must be mono8 or YUV422 to capture video";
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture Video As Avi: %s",response->return_string.c_str());
		return;
	}
	
	rc = pixelinkHelper::GetVideo(hCamera, testString,request->record_time,request->decimation,request->playback_frame_rate);	
		
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture Video As Avi: %s",response->return_string.c_str());
}

void captureVideoAsMp4(const std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureVideoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::CaptureVideoParams::Response>       response)
{
	if (isStreaming == false)
	{
		response->return_code = -1;
		response->return_string = "CameraNotStreamingError";
		return;
	}

	std::string testString = request->file_name.c_str();

	if (testString.find(".mp4") == std::string::npos)
	{
		testString = testString + ".mp4";
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Capture Video As Mp4");

	float pixelFormat;
	
	PXL_RETURN_CODE rc = getValueSimple(FEATURE_PIXEL_FORMAT, &pixelFormat);
	
	if (pixelFormat != PIXEL_FORMAT_MONO8 && pixelFormat != PIXEL_FORMAT_YUV422)
	{
		response->return_code = -1;
		response->return_string = "Error: Pixel format must be mono8 or YUV422 to capture video";
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture Video As MP4: %s",response->return_string.c_str());
		return;
	}
	
	rc = pixelinkHelper::GetVideo(hCamera, testString,request->record_time,request->decimation,request->playback_frame_rate);	
		
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Capture Video As Mp4: %s",response->return_string.c_str());
}

//generic feature
void getFeature(int featureVariable, int numberOfParams, int * returnCode, std::string * returnString, std::vector<float> * features)
{
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, featureVariable, &flags, &numParams, &parms[0]);
	
	*returnCode = rc;
	
	setReturnString(&rc,returnString);
	
	std::vector<float> vect;
	
	for (int i=0;i<numberOfParams;i++)
	{
		vect.push_back(parms[i]);
	}
	
	*features = vect;
}    


//get range
PXL_RETURN_CODE getRange (ULONG feature, float* min, float* max)
{
    PXL_RETURN_CODE rc = ApiSuccess;
    ULONG  featureSize = 0;

    rc = PxLGetCameraFeatures (hCamera, feature, NULL, &featureSize);
    if (!API_SUCCESS(rc)) return rc;
    std::vector<BYTE> featureStore(featureSize);
    PCAMERA_FEATURES pFeatureInfo= (PCAMERA_FEATURES)&featureStore[0];
    rc = PxLGetCameraFeatures (hCamera, feature, pFeatureInfo, &featureSize);
    if (!API_SUCCESS(rc)) return rc;

    if (1 != pFeatureInfo->uNumberOfFeatures ||
        NULL == pFeatureInfo->pFeatures ||
        NULL == pFeatureInfo->pFeatures->pParams) return ApiInvalidParameterError;

    *min = pFeatureInfo->pFeatures->pParams->fMinValue;
    *max = pFeatureInfo->pFeatures->pParams->fMaxValue;

    return ApiSuccess;
}

PXL_RETURN_CODE getRangeOfParameter (ULONG feature, int parameter, float* min, float* max)
{
    PXL_RETURN_CODE rc = ApiSuccess;
    ULONG  featureSize = 0;

    rc = PxLGetCameraFeatures (hCamera, feature, NULL, &featureSize);
    if (!API_SUCCESS(rc)) return rc;
    std::vector<BYTE> featureStore(featureSize);
    
    PCAMERA_FEATURES pFeatureInfo= (PCAMERA_FEATURES)&featureStore[0];
    rc = PxLGetCameraFeatures (hCamera, feature, pFeatureInfo, &featureSize);
    if (!API_SUCCESS(rc)) return rc;

    *min = (int)pFeatureInfo->pFeatures->pParams[parameter].fMinValue;
    *max = (int)pFeatureInfo->pFeatures->pParams[parameter].fMaxValue;

    return ApiSuccess;
}

PXL_RETURN_CODE getRoiMinMaxRange (U32 feature, float* minRoi, float* maxRoi)
{

    PXL_RETURN_CODE rc = ApiSuccess;
    ULONG  featureSize = 0;

    rc = PxLGetCameraFeatures (hCamera, feature, NULL, &featureSize);
    if (!API_SUCCESS(rc)) return rc;
    std::vector<BYTE> featureStore(featureSize);
    
    PCAMERA_FEATURES pFeatureInfo= (PCAMERA_FEATURES)&featureStore[0];
    rc = PxLGetCameraFeatures (hCamera, feature, pFeatureInfo, &featureSize);
    if (!API_SUCCESS(rc)) return rc;

    if (1 != pFeatureInfo->uNumberOfFeatures ||
        NULL == pFeatureInfo->pFeatures ||
        NULL == pFeatureInfo->pFeatures->pParams ||
        pFeatureInfo->pFeatures->uNumberOfParameters < 4) return ApiInvalidParameterError;

	minRoi[0] = (int)pFeatureInfo->pFeatures->pParams[0].fMinValue;
	minRoi[1] = (int)pFeatureInfo->pFeatures->pParams[1].fMinValue;
	minRoi[2] = (int)pFeatureInfo->pFeatures->pParams[2].fMinValue;
	minRoi[3] = (int)pFeatureInfo->pFeatures->pParams[3].fMinValue;

	maxRoi[0] = (int)pFeatureInfo->pFeatures->pParams[0].fMaxValue;
	maxRoi[1] = (int)pFeatureInfo->pFeatures->pParams[1].fMaxValue;
	maxRoi[2] = (int)pFeatureInfo->pFeatures->pParams[2].fMaxValue;
	maxRoi[3] = (int)pFeatureInfo->pFeatures->pParams[3].fMaxValue;

	return ApiSuccess;
}


void setFeature(int featureVariable, int numberOfParams, int * returnCode, std::string * returnString, std::vector<float> * features)
{
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, featureVariable, &flags, &numParams, &parms[0]);
	
	for (int i=0;i<numberOfParams;i++)
	{
		parms[i] = (*features)[i];
	}
	
	if (isStreaming && !(pixelinkHelper::IsSettableWhileStreaming((U32)featureVariable)))
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop Stream to set feature");
		PxLSetStreamState(hCamera,STOP_STREAM);
	}
	
	rc = PxLSetFeature(hCamera, featureVariable, flags, numParams, &parms[0]);
	
	if (isStreaming && !(pixelinkHelper::IsSettableWhileStreaming((U32)featureVariable)))
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Restart stream");
		PxLSetStreamState(hCamera,START_STREAM);
	}
	
	
	*returnCode = rc;
	
	setReturnString(&rc,returnString);
}

// enable/disable AutoRoi
void enableAutoRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_AUTO_ROI, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
	rc = PxLSetFeature(hCamera, FEATURE_AUTO_ROI, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Auto Roi: %s",response->return_string.c_str());
}
void disableAutoRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_AUTO_ROI, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
	rc = PxLSetFeature(hCamera, FEATURE_AUTO_ROI, (FEATURE_FLAG_OFF), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Auto Roi: %s",response->return_string.c_str());
}


//exposure
void getExposure(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_EXPOSURE,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Exposure: %s %f",response->return_string.c_str(),response->params[0]);
}
void setExposure(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_EXPOSURE,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Exposure: %s %f",response->return_string.c_str(),request->params[0]);
}
void getExposureRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_EXPOSURE, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Exposure Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}
void runAutoExposure(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, (FEATURE_FLAG_ONEPUSH), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto Exposure: %s",response->return_string.c_str());
}
void enableContinuousExposure(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, (FEATURE_FLAG_AUTO), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Continuous Exposure: %s",response->return_string.c_str());
}
void disableContinuousExposure(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_EXPOSURE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_EXPOSURE, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Continuous Exposure: %s",response->return_string.c_str());
}


//gain
void getGain(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_GAIN,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Gain: %s %f",response->return_string.c_str(),response->params[0]);
}
void setGain(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_GAIN,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Gain: %s %f",response->return_string.c_str(),request->params[0]);
}
void getGainRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_GAIN, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Gain Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}
void runAutoGain(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_GAIN, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_GAIN, (FEATURE_FLAG_ONEPUSH), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto Gain: %s",response->return_string.c_str());
}
void enableContinuousGain(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_GAIN, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_GAIN, (FEATURE_FLAG_AUTO), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto Gain: %s",response->return_string.c_str());
}
void disableContinuousGain(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_GAIN, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_GAIN, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto Gain: %s",response->return_string.c_str());
}

//HDR
void enableCameraHDR(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	parms[0] = 1.0f;
	PXL_RETURN_CODE rc = PxLSetFeature(hCamera, FEATURE_GAIN_HDR, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);

	response->return_code = rc;
	setReturnString(&rc,&(response->return_string));

	if (isStreaming)
	{
		PxLSetStreamState(hCamera,STOP_STREAM);
		PxLSetStreamState(hCamera,START_STREAM);
	}
   

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Camera HDR: %s %f",response->return_string.c_str(),parms[0]);
}
void enableInterleavedHDR(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	parms[0] = 2.0f;
	PXL_RETURN_CODE rc = PxLSetFeature(hCamera, FEATURE_GAIN_HDR, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);

	response->return_code = rc;
	setReturnString(&rc,&(response->return_string));
	
	if (isStreaming)
	{
		PxLSetStreamState(hCamera,STOP_STREAM);
		PxLSetStreamState(hCamera,START_STREAM);
	}
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Interleaved HDR: %s %f",response->return_string.c_str(),parms[0]);
}
void disableHDR(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	parms[0] = 0.0f;
	PXL_RETURN_CODE rc = PxLSetFeature(hCamera, FEATURE_GAIN_HDR, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);

	response->return_code = rc;
	setReturnString(&rc,&(response->return_string));
	
	if (isStreaming)
	{
		PxLSetStreamState(hCamera,STOP_STREAM);
		PxLSetStreamState(hCamera,START_STREAM);
	}

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable HDR: %s %f",response->return_string.c_str(),parms[0]);
}
void getHDR(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_GAIN_HDR,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get HDR: %s %f",response->return_string.c_str(),response->params[0]);
}




//focus
void getFocus(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_FOCUS,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Focus: %s %f",response->return_string.c_str(),response->params[0]);
}
void setFocus(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_FOCUS,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Focus: %s %f",response->return_string.c_str(),request->params[0]);
}
void runAutoFocus(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 1;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FOCUS, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_FOCUS, (FEATURE_FLAG_ONEPUSH), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto Focus: %s",response->return_string.c_str());
}
void getFocusRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_FOCUS, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Focus Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}




//color temp
void getColorTemperature(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_COLOR_TEMP,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Color Temperature: %s %f",response->return_string.c_str(),response->params[0]);
}
void setColorTemperature(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_COLOR_TEMP,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Color Temperature: %s %f",response->return_string.c_str(),request->params[0]);
}
void getColorTemperatureRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_COLOR_TEMP, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Color Temperature Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}

//white balance
void getWhiteBalance(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_WHITE_SHADING,3,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get White Balance: %s %f %f %f",response->return_string.c_str(),response->params[0],response->params[1],response->params[2]);
}
void setWhiteBalance(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_WHITE_SHADING,3,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set White Balance: %s %f %f %f",response->return_string.c_str(),request->params[0],request->params[1],request->params[2]);
}
void runAutoWhiteBalance(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_WHITE_SHADING, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_WHITE_SHADING, (FEATURE_FLAG_ONEPUSH), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run Auto White Balance: %s",response->return_string.c_str());
}
void getWhiteBalanceRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRangeOfParameter(FEATURE_WHITE_SHADING,0, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get White Balance Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}

//frame rate
void getFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_FRAME_RATE,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Frame Rate: %s %f",response->return_string.c_str(),response->params[0]);
}
void setFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_FRAME_RATE,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Frame Rate: %s %f",response->return_string.c_str(),request->params[0]);
}
void getFrameRateRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_FRAME_RATE, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Frame Rate Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}
void enableContinuousFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FRAME_RATE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_FRAME_RATE, (FEATURE_FLAG_OFF), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Continuous Frame Rate: %s",response->return_string.c_str());
}
void disableContinuousFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FRAME_RATE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_FRAME_RATE, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Continuous Frame Rate: %s",response->return_string.c_str());
}
void enableFixedFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_SPECIAL_CAMERA_MODE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_SPECIAL_CAMERA_MODE, (FEATURE_FLAG_PRESENCE), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Fixed Frame Rate: %s",response->return_string.c_str());
}
void disableFixedFrameRate(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_SPECIAL_CAMERA_MODE, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_SPECIAL_CAMERA_MODE, (FEATURE_FLAG_OFF), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Fixed Frame Rate: %s",response->return_string.c_str());
}

//Roi
void getRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Response>       response)
{
	std::vector<float> features;

	getFeature((int)FEATURE_ROI,4,&(response->return_code),&(response->return_string),&(features));

	response->left = features[0];
	response->top = features[1];
	response->width = features[2];
	response->height = features[3];

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Roi: %s %f %f %f %f",response->return_string.c_str(),response->left,response->top,response->width,response->height);
}
void setRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Response>       response)
{
	std::vector<float> features;
	features.push_back(request->left);
	features.push_back(request->top);
	features.push_back(request->width);
	features.push_back(request->height);

	setFeature((int)FEATURE_ROI,4,&(response->return_code),&(response->return_string),&(features));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Roi: %s %f %f %f %f",response->return_string.c_str(),request->left,request->top,request->width,request->height);
}
void getRoiRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Response>       response)
{
	float min[4], max[4];
        getRoiMinMaxRange(FEATURE_ROI, &min[0], &max[0]);
	response->return_string = "success";
	response->min_left = min[0];
	response->max_left = max[0];
	
	response->min_top = min[1];
	response->max_top = max[1];
	
	response->min_width = min[2];
	response->max_width = max[2];
	
	response->min_height = min[2];
	response->max_height = max[2];
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Roi Range: %s MinLeft:%f MaxLeft:%f MinTop:%f MaxTop:%f MinWidth:%f MaxWidth:%f MinHeight:%f MaxHeight:%f",response->return_string.c_str(),response->min_left,response->max_left,response->min_top,response->max_top,response->min_width,response->max_width,response->min_height,response->max_height);
}

//Focus Roi
void getFocusScoreRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Response>       response)
{
	std::vector<float> features;

	getFeature((int)FEATURE_SHARPNESS_SCORE,5,&(response->return_code),&(response->return_string),&(features));

	response->left = features[0];
	response->top = features[1];
	response->width = features[2];
	response->height = features[3];

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get FocusScore Roi: %s %f %f %f %f",response->return_string.c_str(),response->left,response->top,response->width,response->height);
}
void setFocusScoreRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Response>       response)
{
	std::vector<float> features;
	features.push_back(request->left);
	features.push_back(request->top);
	features.push_back(request->width);
	features.push_back(request->height);
	features.push_back(0);

	setFeature((int)FEATURE_SHARPNESS_SCORE,5,&(response->return_code),&(response->return_string),&(features));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set FocusScore Roi: %s %f %f %f %f",response->return_string.c_str(),request->left,request->top,request->width,request->height);
}
void getFocusScoreRoiRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Response>       response)
{
	float min[4], max[4];
        getRoiMinMaxRange(FEATURE_SHARPNESS_SCORE, &min[0], &max[0]);
	response->return_string = "success";
	response->min_left = min[0];
	response->max_left = max[0];
	
	response->min_top = min[1];
	response->max_top = max[1];
	
	response->min_width = min[2];
	response->max_width = max[2];
	
	response->min_height = min[2];
	response->max_height = max[2];
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Focus Score Roi Range: %s MinLeft:%f MaxLeft:%f MinTop:%f MaxTop:%f MinWidth:%f MaxWidth:%f MinHeight:%f MaxHeight:%f",response->return_string.c_str(),response->min_left,response->max_left,response->min_top,response->max_top,response->min_width,response->max_width,response->min_height,response->max_height);
}




//Auto Roi
void getAutoRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsRoi::Response>       response)
{
	std::vector<float> features;

	getFeature((int)FEATURE_AUTO_ROI,4,&(response->return_code),&(response->return_string),&(features));

	response->left = features[0];
	response->top = features[1];
	response->width = features[2];
	response->height = features[3];

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Auto Roi: %s %f %f %f %f",response->return_string.c_str(),response->left,response->top,response->width,response->height);
}
void setAutoRoi(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsRoi::Response>       response)
{
	std::vector<float> features;
	features.push_back(request->left);
	features.push_back(request->top);
	features.push_back(request->width);
	features.push_back(request->height);

	setFeature((int)FEATURE_AUTO_ROI,4,&(response->return_code),&(response->return_string),&(features));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Auto Roi: %s %f %f %f %f",response->return_string.c_str(),request->left,request->top,request->width,request->height);
}
void getAutoRoiRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRoiRange::Response>       response)
{
	float min[4], max[4];
        getRoiMinMaxRange(FEATURE_AUTO_ROI, &min[0], &max[0]);
	response->return_string = "success";
	response->min_left = min[0];
	response->max_left = max[0];
	
	response->min_top = min[1];
	response->max_top = max[1];
	
	response->min_width = min[2];
	response->max_width = max[2];
	
	response->min_height = min[2];
	response->max_height = max[2];
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Auto Roi Range: %s MinLeft:%f MaxLeft:%f MinTop:%f MaxTop:%f MinWidth:%f MaxWidth:%f MinHeight:%f MaxHeight:%f",response->return_string.c_str(),response->min_left,response->max_left,response->min_top,response->max_top,response->min_width,response->max_width,response->min_height,response->max_height);
}

//saturation
void getSaturation(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_SATURATION,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Saturation: %s %f",response->return_string.c_str(),response->params[0]);
}
void setSaturation(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_SATURATION,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Saturation: %s %f",response->return_string.c_str(),request->params[0]);
}
void getSaturationRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_SATURATION, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Saturation Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}

//gamma
void getGamma(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_GAMMA,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Gamma: %s %f",response->return_string.c_str(),response->params[0]);
}
void setGamma(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_GAMMA,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Gamma: %s %f",response->return_string.c_str(),request->params[0]);
}
void getGammaRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_GAMMA, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Gamma Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}
void enableGamma(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_GAMMA, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_GAMMA, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Gamma: %s",response->return_string.c_str());
}
void disableGamma(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_GAMMA, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_GAMMA, (FEATURE_FLAG_OFF), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Gamma: %s",response->return_string.c_str());
}



//pixel addressing
void getPixelAddressing(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsPixelAddressing::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParamsPixelAddressing::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &parms[0]);

	std::string mode;

	if (parms[1] == 0)
	{
		mode = "decimation";
	}
	else if (parms[1] == 2)
	{
		mode = "binning";
	}
	
	response->mode = mode;
	
	response->horizontal = parms[2];
	response->vertical = parms[3];
	
	response->return_code = rc;
	setReturnString(&rc,&(response->return_string));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Pixel Addressing: %s %s %fx%f",response->return_string.c_str(), response->mode.c_str(),response->horizontal,response->vertical);
}
void disablePixelAddressing(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[2] = 1.0f;
		parms[3] = 1.0f;
		rc = PxLSetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Pixel Addressing: %s",response->return_string.c_str());
}
void setPixelAddressingBinning(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[1] = 2;//set to binning
		parms[2] = request->horizontal;
		parms[3] = request->vertical;
		rc = PxLSetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Pixel Addressing Binning: %s %fx%f",response->return_string.c_str(), request->horizontal,request->vertical);
}
void setPixelAddressingDecimation(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[1] = 0;//set to decimation
		parms[2] = request->horizontal;
		parms[3] = request->vertical;
		rc = PxLSetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Pixel Addressing Decimation: %s %fx%f",response->return_string.c_str(), request->horizontal,request->vertical);
}
void getPixelAddressingRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRangeOfParameter(FEATURE_PIXEL_ADDRESSING,2, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Pixel Addressing Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}

//rotation
void getRotation(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_ROTATE,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Rotation: %s %f",response->return_string.c_str(),response->params[0]);
}
void setRotation(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	setFeature((int)FEATURE_ROTATE,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Rotation: %s %f",response->return_string.c_str(),request->params[0]);
}

//flip
void enableHorizontalFlip(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 2;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FLIP, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[0] = 1;
		rc = PxLSetFeature(hCamera, FEATURE_FLIP, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Horizontal Flip: %s",response->return_string.c_str());
}
void disableHorizontalFlip(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 2;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FLIP, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[0] = 0;
		rc = PxLSetFeature(hCamera, FEATURE_FLIP, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Horizontal Flip: %s",response->return_string.c_str());
}
void enableVerticalFlip(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 2;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FLIP, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[1] = 1;
		rc = PxLSetFeature(hCamera, FEATURE_FLIP, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable Vertical Flip: %s",response->return_string.c_str());
}
void disableVerticalFlip(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 2;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_FLIP, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		parms[1] = 0;
		rc = PxLSetFeature(hCamera, FEATURE_FLIP, flags, numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Vertical Flip: %s",response->return_string.c_str());
}
void getFlip(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_FLIP,2,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Flip: %s Horizontal=%f Vertical=%f",response->return_string.c_str(),response->params[0],response->params[1]);
}



//bandwidth limit
void getBandwidthLimit(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetParams::Response>       response)
{
	getFeature((int)FEATURE_BANDWIDTH_LIMIT,1,&(response->return_code),&(response->return_string),&(response->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Bandwidth Limit: %s %f",response->return_string.c_str(),response->params[0]);
}
void setBandwidthLimit(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_BANDWIDTH_LIMIT, &flags, &numParams, &parms[0]);
	
	if ((int)(flags & FEATURE_FLAG_OFF) == (int)FEATURE_FLAG_OFF)
	{
		response->return_code = -1;
		response->return_string = "BandwidthLimitNotEnabledError";
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Bandwidth Limit: %s %f",response->return_string.c_str(),request->params[0]);
		return;
	}

	setFeature((int)FEATURE_BANDWIDTH_LIMIT,1,&(response->return_code),&(response->return_string),&(request->params));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Bandwidth Limit: %s %f",response->return_string.c_str(),request->params[0]);
}
void getBandwidthLimitRange(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetRange::Response>       response)
{
	float min, max;
        getRange(FEATURE_BANDWIDTH_LIMIT, &min, &max);
	response->return_string = "success";
	response->min = min;
	response->max = max;
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Bandwidth Limit Range: %s Min:%f Max:%f",response->return_string.c_str(),response->min,response->max);
}
void enableBandwidthLimit(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_BANDWIDTH_LIMIT, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_BANDWIDTH_LIMIT, (FEATURE_FLAG_MANUAL), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enable BandwidthLimit: %s",response->return_string.c_str());
}
void disableBandwidthLimit(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	int numberOfParams = 4;
	float parms[numberOfParams];
	U32 flags = 0;
	U32 numParams = numberOfParams;
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_BANDWIDTH_LIMIT, &flags, &numParams, &parms[0]);
	
	if (API_SUCCESS(rc))
	{
		rc = PxLSetFeature(hCamera, FEATURE_BANDWIDTH_LIMIT, (FEATURE_FLAG_OFF), numParams, &parms[0]);
	}
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Bandwidth Limit: %s",response->return_string.c_str());
}

//save and load settings
void saveSettings(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLSaveSettings(hCamera,1);
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Save Settings: %s",response->return_string.c_str());
}
void loadSettings(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLLoadSettings(hCamera,1);
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Load Settings: %s",response->return_string.c_str());
}
void restoreFactoryDefaults(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{
	PXL_RETURN_CODE rc = PxLLoadSettings(hCamera,0);
	
	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Restore Factory Defaults: %s",response->return_string.c_str());
}

//pixel format
void getSupportedPixelFormats(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetSupportedPixelFormats::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetSupportedPixelFormats::Response>       response)
{	
	std::vector<std::string> supportedPixelFormats; 
	float min, max, currentValue;
        bool restoreRequired = false;
        PXL_RETURN_CODE rc;
	
	//get min/max possible pixel formats and get current pixel format
        getRange(FEATURE_PIXEL_FORMAT, &min, &max);
        getValueSimple(FEATURE_PIXEL_FORMAT, &currentValue);
        
        //add min pixel format to list
        supportedPixelFormats.push_back(PxLPixelFormat::fromApi(min));
        
        //try setting every pixel format within range and add to list if it sets successfully
	for (float candidate = min+1.0; candidate < max; candidate+=1.0)
	{
		if (find (supportedPixelFormats.begin(),
			  supportedPixelFormats.end(),
			  PxLPixelFormat::fromApi(candidate)) != supportedPixelFormats.end())
		{
		    // This entry is already there.  That can happen as setValueSimple will accept
		    // both the generic color description (like PIXEL_FORMAT_BAYER8) and the more
		    // specific color descriptor (like PIXEL_FORMAT_BAYER8_GBRG).
		    continue;
		}
		rc = setValueSimple(FEATURE_PIXEL_FORMAT, candidate);
		if (API_SUCCESS(rc)) {
		    restoreRequired = true;
		    supportedPixelFormats.push_back(PxLPixelFormat::fromApi(candidate));
		}
	}

	// We know the camera support the 'max' pixel format so add it if needed
	if (max > min &&
	find (supportedPixelFormats.begin(),
	      supportedPixelFormats.end(),
	      PxLPixelFormat::fromApi(max)) == supportedPixelFormats.end())
	{
	supportedPixelFormats.push_back(PxLPixelFormat::fromApi(max));
	}

	    // restore the old value (if necessary)
	if (restoreRequired) setValueSimple(FEATURE_PIXEL_FORMAT, currentValue);

	response->pixel_formats = supportedPixelFormats;
	response->return_string = "Success";

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Supported Pixel Formats: %s",response->return_string.c_str());
}

//get pixel format
void getPixelFormat(const std::shared_ptr<pixelink_ros2_interfaces::srv::GetPixelFormat::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::GetPixelFormat::Response>       response)
{	
	float currentValue;
	
	PXL_RETURN_CODE rc = getValueSimple(FEATURE_PIXEL_FORMAT, &currentValue);

	if (API_SUCCESS(rc))
	{
		response->pixel_format = PxLPixelFormat::fromApi(currentValue);
	}

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Pixel Format: %s %s",response->return_string.c_str(),response->pixel_format.c_str());
}

//get pixel format
void setPixelFormat(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetPixelFormat::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetPixelFormat::Response>       response)
{	
	
	PXL_RETURN_CODE rc = setValueSimple(FEATURE_PIXEL_FORMAT, PxLPixelFormat::toApi(request->pixel_format));

	if (API_SUCCESS(rc))
	{
		int numberOfParams = 4;
		float parms[numberOfParams];
		U32 flags = 0;
		U32 numParams = numberOfParams;
		PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_PIXEL_FORMAT, &flags, &numParams, &parms[0]);
		
		encodingString = PxLPixelFormat::toSensorMessageEncoding(parms[0]);
	}

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Pixel Format: %s %s",response->return_string.c_str(),request->pixel_format.c_str());
}

//
// Triggering
//
PXL_RETURN_CODE SetTriggering(int mode, int triggerType, int polarity, float delay, float param)
{
	U32 flags;
	U32 numParams = FEATURE_TRIGGER_NUM_PARAMS;
	float params[FEATURE_TRIGGER_NUM_PARAMS];

	// Read current settings
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_TRIGGER, &flags, &numParams, &params[0]);

	// Very important step: Enable triggering by clearing the FEATURE_FLAG_OFF bit
	flags = ENABLE_FEATURE(flags, true);

	// Assign the new values...
	params[FEATURE_TRIGGER_PARAM_MODE]	= (float)mode;
	params[FEATURE_TRIGGER_PARAM_TYPE]	= (float)triggerType;
	params[FEATURE_TRIGGER_PARAM_POLARITY]	= (float)polarity;
	params[FEATURE_TRIGGER_PARAM_DELAY]	= delay;
	params[FEATURE_TRIGGER_PARAM_PARAMETER]	= param;

	// ... and write them to the camera
	rc = PxLSetFeature(hCamera, FEATURE_TRIGGER, flags, numParams, &params[0]);
	
	return rc;
}

PXL_RETURN_CODE DisableTriggering()
{
	U32 flags;
	U32 numParams = 5;
	float params[5];

	// Read current settings
	PXL_RETURN_CODE rc = PxLGetFeature(hCamera, FEATURE_TRIGGER, &flags, &numParams, &params[0]);

	// Disable triggering
	flags = ENABLE_FEATURE(flags, false);

	rc = PxLSetFeature(hCamera, FEATURE_TRIGGER, flags, numParams, &params[0]);
	
	return rc;
}

void setTriggering(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetTriggerParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetTriggerParams::Response>       response)
{	
	PXL_RETURN_CODE rc = SetTriggering(request->mode, request->trigger_type, request->polarity, request->delay, request->parameters);

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set Triggering: %s",response->return_string.c_str());
}

void disableTriggering(const std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::NoParams::Response>       response)
{	
	PXL_RETURN_CODE rc = DisableTriggering();

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disable Triggering: %s",response->return_string.c_str());
}



//gpio
PXL_RETURN_CODE SetGpioValue (bool enabled, int gpioNum, int mode, int polarity, float param1, float param2, float param3)
{
    float values[6];
    ULONG flags = 0;
    ULONG numParameters = 6;

    flags = enabled ? FEATURE_FLAG_MANUAL : FEATURE_FLAG_OFF;

    values [FEATURE_GPIO_PARAM_GPIO_INDEX] = (float)gpioNum;
    values [FEATURE_GPIO_PARAM_MODE] = mode;
    values [FEATURE_TRIGGER_PARAM_POLARITY] = polarity;
    values [FEATURE_GPIO_PARAM_PARAM_1] = param1;
    values [FEATURE_GPIO_PARAM_PARAM_2] = param2;
    values [FEATURE_GPIO_PARAM_PARAM_3] = param3;

    return PxLSetFeature (hCamera, FEATURE_GPIO, flags, numParameters, values);
}

void enableGpio(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetGpioParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetGpioParams::Response>       response)
{	
	PXL_RETURN_CODE rc = SetGpioValue(true, request->gpio_number, request->mode, request->polarity, request->parameter_one, request->parameter_two, request->parameter_three);

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set gpio: %s",response->return_string.c_str());
}

void disableGpio(const std::shared_ptr<pixelink_ros2_interfaces::srv::SetGpioParams::Request> request,
          std::shared_ptr<pixelink_ros2_interfaces::srv::SetGpioParams::Response>       response)
{	
	PXL_RETURN_CODE rc = SetGpioValue(false, request->gpio_number, request->mode, request->polarity, request->parameter_one, request->parameter_two, request->parameter_three);

	response->return_code = rc;
	
	setReturnString(&rc,&(response->return_string));	

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set gpio: %s",response->return_string.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pixelink_server");

	std::string str = std::string(node->get_name());
	
	if (str != "pixelink_server")
	{
		str = "/" + str + "/";
	}
	else
	{
		str = "";
	}
	
//initialize    
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr initializeService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "initialize",  &initialize); 
//initialize with serial
  rclcpp::Service<pixelink_ros2_interfaces::srv::SerialParam>::SharedPtr initializeWithSerialService =                
    node->create_service<pixelink_ros2_interfaces::srv::SerialParam>(str + "initializeWithSerial",  &initializeWithSerial); 
//uninitialize    
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr uninitializeService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "uninitialize",  &uninitialize); 

// start/stop stream    
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr startStreamService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "startStream",  &startStream);     
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr stopStreamService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "stopStream",  &stopStream); 

//GetSerialNumbers
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetSerialNumbers>::SharedPtr getSerialNumbersService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetSerialNumbers>(str + "getSerialNumbers",  &getSerialNumbers);     

//GetCameraInfo
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetCameraInfo>::SharedPtr getCameraInfoService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetCameraInfo>(str + "getCameraInfo",  &getCameraInfo);    

// start/stop preview    
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr startPreviewService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "startPreview",  &startPreview); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr stopPreviewService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "stopPreview",  &stopPreview);  
 
 //captureImages
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureParams>::SharedPtr captureImageAsBmpService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureParams>(str + "captureImageAsBmp",  &captureImageAsBmp);  
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureParams>::SharedPtr captureImageAsTiffService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureParams>(str + "captureImageAsTiff",  &captureImageAsTiff); 
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureParams>::SharedPtr captureImageAsPsdService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureParams>(str + "captureImageAsPsd",  &captureImageAsPsd); 
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureParams>::SharedPtr captureImageAsJpgService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureParams>(str + "captureImageAsJpg",  &captureImageAsJpg);  
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureParams>::SharedPtr captureImageAsRawService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureParams>(str + "captureImageAsRaw",  &captureImageAsRaw); 

//captureVideo
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureVideoParams>::SharedPtr captureVideoAsAviService =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureVideoParams>(str + "captureVideoAsAvi",  &captureVideoAsAvi); 
   rclcpp::Service<pixelink_ros2_interfaces::srv::CaptureVideoParams>::SharedPtr captureVideoAsMp4Service =                
    node->create_service<pixelink_ros2_interfaces::srv::CaptureVideoParams>(str + "captureVideoAsMp4",  &captureVideoAsMp4); 

//exposure    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getExposureService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getExposure",  &getExposure); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setExposureService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setExposure",  &setExposure);
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr runAutoExposureService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "runAutoExposure",  &runAutoExposure);      
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableContinuousExposureService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableContinuousExposure",  &enableContinuousExposure);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableContinuousExposureService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableContinuousExposure",  &disableContinuousExposure);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getExposureRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getExposureRange",  &getExposureRange);    

//gain    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getGainService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getGain",  &getGain); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setGainService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setGain",  &setGain);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr runAutoGainService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "runAutoGain",  &runAutoGain);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableContinuousGainService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableContinuousGain",  &enableContinuousGain);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableContinuousGainService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableContinuousGain",  &disableContinuousGain);
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getGainRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getGainRange",  &getGainRange);   

//HDR
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableCameraHDRService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableCameraHDR",  &enableCameraHDR);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableInterleavedHDRService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableInterleavedHDR",  &enableInterleavedHDR);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableHDRService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableHDR",  &disableHDR);
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getHDRService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getHDR",  &getHDR); 


//focus    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getFocusService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getFocus",  &getFocus); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setFocusService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setFocus",  &setFocus);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr runAutoFocusService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "runAutoFocus",  &runAutoFocus);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getFocusRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getFocusRange",  &getFocusRange); 

//color temp    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getColorTemperatureService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getColorTemperature",  &getColorTemperature); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setColorTemperatureService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setColorTemperature",  &setColorTemperature); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getColorTemperatureRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getColorTemperatureRange",  &getColorTemperatureRange);   

//WhiteBalance    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getWhiteBalanceService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getWhiteBalance",  &getWhiteBalance); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setWhiteBalanceService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setWhiteBalance",  &setWhiteBalance);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr runAutoWhiteBalanceService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "runAutoWhiteBalance",  &runAutoWhiteBalance); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getWhiteBalanceRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getWhiteBalanceRange",  &getWhiteBalanceRange); 

//Frame Rate    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getFrameRate",  &getFrameRate); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setFrameRate",  &setFrameRate);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableContinuousFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableContinuousFrameRate",  &enableContinuousFrameRate);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableContinuousFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableContinuousFrameRate",  &disableContinuousFrameRate); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableFixedFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableFixedFrameRate",  &enableFixedFrameRate);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableFixedFrameRateService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableFixedFrameRate",  &disableFixedFrameRate); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getFrameRateRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getFrameRateRange",  &getFrameRateRange); 

//roi    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParamsRoi>::SharedPtr getRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParamsRoi>(str + "getRoi",  &getRoi); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParamsRoi>::SharedPtr setRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParamsRoi>(str + "setRoi",  &setRoi);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRoiRange>::SharedPtr getRoiRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRoiRange>(str + "getRoiRange",  &getRoiRange); 


//FocusScore roi    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParamsRoi>::SharedPtr getFocusScoreRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParamsRoi>(str + "getFocusScoreRoi",  &getFocusScoreRoi); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParamsRoi>::SharedPtr setFocusScoreRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParamsRoi>(str + "setFocusScoreRoi",  &setFocusScoreRoi);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRoiRange>::SharedPtr getFocusScoreRoiRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRoiRange>(str + "getFocusScoreRoiRange",  &getFocusScoreRoiRange); 

//Auto roi    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParamsRoi>::SharedPtr getAutoRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParamsRoi>(str + "getAutoRoi",  &getAutoRoi); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParamsRoi>::SharedPtr setAutoRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParamsRoi>(str + "setAutoRoi",  &setAutoRoi); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableAutoRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableAutoRoi",  &enableAutoRoi);     
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableAutoRoiService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableAutoRoi",  &disableAutoRoi); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRoiRange>::SharedPtr getAutoRoiRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRoiRange>(str + "getAutoRoiRange",  &getAutoRoiRange); 


//saturation    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getSaturationService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getSaturation",  &getSaturation); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setSaturationService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setSaturation",  &setSaturation);
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getSaturationRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getSaturationRange",  &getSaturationRange);  

//gamma    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getGammaService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getGamma",  &getGamma); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setGammaService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setGamma",  &setGamma);
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableGammaService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableGamma",  &enableGamma);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableGammaService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableGamma",  &disableGamma); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getGammaRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getGammaRange",  &getGammaRange);  

//pixel addressing    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParamsPixelAddressing>::SharedPtr getPixelAddressingService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParamsPixelAddressing>(str + "getPixelAddressing",  &getPixelAddressing); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disablePixelAddressingService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disablePixelAddressing",  &disablePixelAddressing); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing>::SharedPtr setPixelAddressingBinningService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing>(str + "setPixelAddressingBinning",  &setPixelAddressingBinning);
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing>::SharedPtr setPixelAddressingDecimationService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParamsPixelAddressing>(str + "setPixelAddressingDecimation",  &setPixelAddressingDecimation);
      rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getPixelAddressingRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getPixelAddressingRange",  &getPixelAddressingRange); 

//rotation    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getRotationService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getRotation",  &getRotation); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setRotationService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setRotation",  &setRotation);
    
//flip
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableHorizontalFlipService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableHorizontalFlip",  &enableHorizontalFlip);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableHorizontalFlipService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableHorizontalFlip",  &disableHorizontalFlip); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableVerticalFlipService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableVerticalFlip",  &enableVerticalFlip);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableVerticalFlipService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableVerticalFlip",  &disableVerticalFlip); 
   rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getFlipService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getFlip",  &getFlip); 
 
//bandwidth limit    
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetParams>::SharedPtr getBandwidthLimitService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetParams>(str + "getBandwidthLimit",  &getBandwidthLimit); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetParams>::SharedPtr setBandwidthLimitService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetParams>(str + "setBandwidthLimit",  &setBandwidthLimit);
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr enableBandwidthLimitService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "enableBandwidthLimit",  &enableBandwidthLimit);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableBandwidthLimitService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableBandwidthLimit",  &disableBandwidthLimit);
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetRange>::SharedPtr getBandwidthLimitRangeService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetRange>(str + "getBandwidthLimitRange",  &getBandwidthLimitRange);     
    
// save and load settings
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr saveSettingsService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "saveSettings",  &saveSettings);  
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr loadSettingsService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "loadSettings",  &loadSettings);  
   rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr restoreFactoryDefaultsService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "restoreFactoryDefaults",  &restoreFactoryDefaults);    

//pixelformats
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetSupportedPixelFormats>::SharedPtr getSupportedPixelFormatsService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetSupportedPixelFormats>(str + "getSupportedPixelFormats",  &getSupportedPixelFormats); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::GetPixelFormat>::SharedPtr getPixelFormatService =                
    node->create_service<pixelink_ros2_interfaces::srv::GetPixelFormat>(str + "getPixelFormat",  &getPixelFormat); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetPixelFormat>::SharedPtr setPixelFormatService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetPixelFormat>(str + "setPixelFormat",  &setPixelFormat); 
    
//triggering
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetTriggerParams>::SharedPtr setTriggeringService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetTriggerParams>(str + "setTriggering",  &setTriggering); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::NoParams>::SharedPtr disableTriggeringService =                
    node->create_service<pixelink_ros2_interfaces::srv::NoParams>(str + "disableTriggering",  &disableTriggering); 

//gpio
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetGpioParams>::SharedPtr enableGpioService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetGpioParams>(str + "enableGpio",  &enableGpio); 
  rclcpp::Service<pixelink_ros2_interfaces::srv::SetGpioParams>::SharedPtr disableGpioService =                
    node->create_service<pixelink_ros2_interfaces::srv::SetGpioParams>(str + "disableGpio",  &disableGpio); 
    

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to accept camera commands");     

	rclcpp::WallRate loop_rate(5);
	image_transport::ImageTransport it(node);
	image_transport::Publisher pub = it.advertise(str + "image", 1);	
	
	int count = 0;
	
	while(rclcpp::ok())
	{	
		//camera is initialized and streaming 
		if (isStreaming == true && hCamera != 0)
		{
			sensor_msgs::msg::Image msg;
			
			FRAME_DESC desc;
			desc.uSize = sizeof(FRAME_DESC);
			
			U32 bufferSize = pixelinkHelper::DetermineRawImageSize(hCamera);
			
			std::vector<uint8_t> buffer(bufferSize);
			
			PXL_RETURN_CODE rc = PxLGetNextFrame(hCamera, bufferSize, &buffer[0], &desc);
			
			std::string returnString = "Success";
			setReturnString(&rc,&(returnString));
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get next frame: %s",returnString.c_str());
			
			if (API_SUCCESS(rc))
			{
				if (captureAsBmp == true)
				{
					int retVal = pixelinkHelper::GetSnapshotAndSave(hCamera, IMAGE_FORMAT_BMP, saveImageName.c_str(), (char*)&buffer[0], bufferSize, &desc);
				
					if (API_SUCCESS(retVal) || captureAttempts >= 5)
					{
						captureAttempts = 0;
						captureAsBmp = false;
					}
					else
					{
						captureAttempts++;
					}
				}
				else if (captureAsJpg == true)
				{
					int retVal = pixelinkHelper::GetSnapshotAndSave(hCamera, IMAGE_FORMAT_JPEG, saveImageName.c_str(), (char*)&buffer[0], bufferSize, &desc);
				
					if (API_SUCCESS(retVal) || captureAttempts >= 5)
					{
						captureAttempts = 0;
						captureAsJpg = false;
					}
					else
					{
						captureAttempts++;
					}
				}
				else if (captureAsTiff == true)
				{
					int retVal = pixelinkHelper::GetSnapshotAndSave(hCamera, IMAGE_FORMAT_TIFF, saveImageName.c_str(), (char*)&buffer[0], bufferSize, &desc);
				
					if (API_SUCCESS(retVal) || captureAttempts >= 5)
					{
						captureAttempts = 0;
						captureAsTiff = false;
					}
					else
					{
						captureAttempts++;
					}
				}
				else if (captureAsPsd == true)
				{
					int retVal = pixelinkHelper::GetSnapshotAndSave(hCamera, IMAGE_FORMAT_PSD, saveImageName.c_str(), (char*)&buffer[0], bufferSize, &desc);
				
					if (API_SUCCESS(retVal) || captureAttempts >= 5)
					{
						captureAttempts = 0;
						captureAsPsd = false;
					}
					else
					{
						captureAttempts++;
					}
				}
				else if (captureAsRaw == true)
				{
					int retVal = pixelinkHelper::GetSnapshotAndSave(hCamera, IMAGE_FORMAT_RAW_RGB24, saveImageName.c_str(), (char*)&buffer[0], bufferSize, &desc);
				
					if (API_SUCCESS(retVal) || captureAttempts >= 5)
					{
						captureAttempts = 0;
						captureAsRaw = false;
					}
					else
					{
						captureAttempts++;
					}
				}
			}
			
			
			msg.width = desc.Roi.fWidth;
			msg.height	= desc.Roi.fHeight;
			msg.step	= bufferSize / desc.Roi.fHeight;
			msg.data.resize(bufferSize);			
			memcpy(&(msg.data[0]),&buffer[0],bufferSize);
			msg.encoding = encodingString;
			msg.is_bigendian = 0;
			
			pub.publish(msg);
		}
		
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}


  //rclcpp::spin(node);
  rclcpp::shutdown();
}
