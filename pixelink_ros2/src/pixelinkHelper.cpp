#include "PixeLINKApi.h"
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "pixelinkHelper.h"
#include "pixelFormat.h"
#include <memory>
#include <vector>


//
// Given the pixel format, return the size of a individual pixel (in bytes)
//
// Returns 0 on failure.
//
U32 
pixelinkHelper::GetPixelSize(U32 pixelFormat)
{
	U32 retVal = 0;

	switch(pixelFormat) {
	
		case PIXEL_FORMAT_MONO8:
		case PIXEL_FORMAT_BAYER8_GRBG:
		case PIXEL_FORMAT_BAYER8_RGGB:
		case PIXEL_FORMAT_BAYER8_GBRG:
		case PIXEL_FORMAT_BAYER8_BGGR:
			retVal = 1;
			break;

		case PIXEL_FORMAT_YUV422:
		case PIXEL_FORMAT_MONO16:
		case PIXEL_FORMAT_BAYER16_GRBG:
		case PIXEL_FORMAT_BAYER16_RGGB:
		case PIXEL_FORMAT_BAYER16_GBRG:
		case PIXEL_FORMAT_BAYER16_BGGR:
			retVal = 2;
			break;

		case PIXEL_FORMAT_RGB24:
			retVal = 3;
			break;

		case PIXEL_FORMAT_RGB48:
		case PIXEL_FORMAT_STOKES4_12:
		case PIXEL_FORMAT_POLAR4_12:
		case PIXEL_FORMAT_POLAR_RAW4_12:
		case PIXEL_FORMAT_HSV4_12:
			retVal = 6;
			break;

		default:
			assert(0);
			break;
	}
	return retVal;
}
//
// Query the camera for region of interest (ROI), decimation, and pixel format
// Using this information, we can calculate the size of a raw image
//
// Returns 0 on failure
//
U32 
pixelinkHelper::DetermineRawImageSize(HANDLE hCamera)
{
	float parms[4];		// reused for each feature query
	U32 roiWidth;
	U32 roiHeight;
	U32 pixelAddressingValue;		// integral factor by which the image is reduced
	U32 pixelFormat;
	U32 numPixels;
	U32 pixelSize;
	U32 flags = FEATURE_FLAG_MANUAL;
	U32 numParams;

	assert(0 != hCamera);

	// Get region of interest (ROI)
	numParams = 4; // left, top, width, height
	PxLGetFeature(hCamera, FEATURE_ROI, &flags, &numParams, &parms[0]);
	roiWidth	= (U32)parms[FEATURE_ROI_PARAM_WIDTH];
	roiHeight	= (U32)parms[FEATURE_ROI_PARAM_HEIGHT];
	
	// Get camera HDR interleaved
	numParams = 1; // interleaved = 2
	PxLGetFeature(hCamera, FEATURE_GAIN_HDR, &flags, &numParams, &parms[0]);
	if (parms[0] == 2){
	roiWidth = roiWidth * 2;
	}
	
	// Query pixel addressing 
        // assume no pixel addressing (in case it is not supported)
	parms[FEATURE_PIXEL_ADDRESSING_PARAM_VALUE] = 1.0;
	numParams = 2; // pixel addressing value, pixel addressing type (e.g. bin, average, ...)
	PxLGetFeature(hCamera, FEATURE_PIXEL_ADDRESSING, &flags, &numParams, &parms[0]);
	pixelAddressingValue = (U32)parms[FEATURE_PIXEL_ADDRESSING_PARAM_VALUE];

	// We can calulate the number of pixels now.
	numPixels = (roiWidth / pixelAddressingValue) * (roiHeight / pixelAddressingValue);

	// Knowing pixel format means we can determine how many bytes per pixel.
	numParams = 1;
	PxLGetFeature(hCamera, FEATURE_PIXEL_FORMAT, &flags, &numParams, &parms[0]);
	pixelFormat = (U32)parms[0];

	// And now the size of the frame
	pixelSize = GetPixelSize(pixelFormat);

	return numPixels * pixelSize;
}
//
// Given a buffer with a raw image, create and return a 
// pointer to a new buffer with the encoded image. 
//
// NOTE: The caller becomes the owner of the buffer containing the 
//		 encoded image, and therefore must free the 
//		 buffer when done with it.
//
// Returns SUCCESS or FAILURE
//
int
pixelinkHelper::EncodeRawImage(const char* pRawImage,
			   const FRAME_DESC* pFrameDesc, 
			   U32 encodedImageFormat, 
			   char** ppEncodedImage, 
			   U32* pEncodedImageSize)
{
	U32 encodedImageSize = 0;
	char* pEncodedImage;

	assert(NULL != pRawImage);
	assert(NULL != pFrameDesc);
	assert(NULL != ppEncodedImage);
	assert(NULL != pEncodedImageSize);

	// How big is the encoded image going to be?
	// Pass in NULL for the encoded image pointer, and the result is
	// returned in encodedImageSize
	if (API_SUCCESS(PxLFormatImage((LPVOID)pRawImage, (FRAME_DESC*)pFrameDesc, encodedImageFormat, NULL, &encodedImageSize))) {
		assert(encodedImageSize > 0);
		pEncodedImage = (char*)malloc(encodedImageSize);
		// Now that we have a buffer for the encoded image, ask for it to be converted.
		// NOTE: encodedImageSize is an IN param here because we're telling PxLFormatImage two things:
		//       1) pointer to the buffer
		//       2) the size of the buffer
		if (NULL != pEncodedImage) {
			if (API_SUCCESS(PxLFormatImage((LPVOID)pRawImage, (FRAME_DESC*)pFrameDesc, encodedImageFormat, pEncodedImage, &encodedImageSize))) {
				*ppEncodedImage = pEncodedImage; // handing over ownership of buffer to caller
				*pEncodedImageSize = encodedImageSize;
				return SUCCESS;
			}
			free(pEncodedImage);
        	}
	}

	return FAILURE;
}
//
// Save a buffer to a file
// This overwrites any existing file
//
// Returns SUCCESS or FAILURE
//
int
pixelinkHelper::SaveImageToFile(const char* pFilename, const char* pImage, U32 imageSize)
{
	size_t numBytesWritten;
	FILE* pFile;

	assert(NULL != pFilename);
	assert(NULL != pImage);
	assert(imageSize > 0);

	// Open our file for binary write
	pFile = fopen(pFilename, "wb");
	if (NULL == pFile) {
		return FAILURE;
	}

	numBytesWritten = fwrite((void*)pImage, sizeof(char), imageSize, pFile);

	fclose(pFile);
	
	return ((U32)numBytesWritten == imageSize) ? SUCCESS : FAILURE;
}

//
// NOTE: PxLGetNextFrame can return ApiCameraTimeoutError on occasion. 
// How you handle this depends on your situation and how you use your camera. 
// For this sample app, we'll just retry a few times.
//
PXL_RETURN_CODE 
pixelinkHelper::GetNextFrame(HANDLE hCamera, U32 bufferSize, void* pFrame, FRAME_DESC* pFrameDesc)
{
	int numTries = 0;
	const int MAX_NUM_TRIES = 4;
	PXL_RETURN_CODE rc = ApiUnknownError;

	for(numTries = 0; numTries < MAX_NUM_TRIES; numTries++) {
		// Important that we set the frame desc size before each and every call to PxLGetNextFrame
		pFrameDesc->uSize = sizeof(FRAME_DESC);
		rc = PxLGetNextFrame(hCamera, bufferSize, pFrame, pFrameDesc);
		if (API_SUCCESS(rc)) {
			break;
		}
	}

	return rc;
}

//
// Capture an image from the camera.
// 
// NOTE: PxLGetNextFrame is a blocking call. 
// i.e. PxLGetNextFrame won't return until an image is captured.
// So, if you're using hardware triggering, it won't return until the camera is triggered.
//
int
pixelinkHelper::GetRawImage(HANDLE hCamera, char* pRawImage, U32 rawImageSize, FRAME_DESC* pFrameDesc)
{
	int retVal;

	assert(0 != hCamera);
	assert(NULL != pRawImage);
	assert(rawImageSize > 0);
	assert(NULL != pFrameDesc);


	// Put camera into streaming state so we can capture an image
	if (!API_SUCCESS(PxLSetStreamState(hCamera, START_STREAM))) {
		return FAILURE;
	}

	// Get an image
	retVal = GetNextFrame(hCamera, rawImageSize, (LPVOID*)pRawImage, pFrameDesc);

	// Done capturing, so no longer need the camera streaming images.
	PxLSetStreamState(hCamera, STOP_STREAM);

	return (API_SUCCESS(retVal)) ? SUCCESS : FAILURE;
}

int 
pixelinkHelper::GetSnapshotAndSave(HANDLE hCamera, U32 imageFormat, const char* pFilename, char* pRawImage, U32 rawImageSize, FRAME_DESC* pFrameDesc)
{
	U32 encodedImageSize;
	char* pEncodedImage;
	int retVal = FAILURE;

	// Encode the raw image into something displayable
	if (EncodeRawImage(pRawImage, pFrameDesc, imageFormat, &pEncodedImage, &encodedImageSize) == SUCCESS) {
		if (SaveImageToFile(pFilename, pEncodedImage, encodedImageSize) == SUCCESS) {
			retVal = SUCCESS;
		}
		free(pEncodedImage);
	}
	
	return retVal;
}

bool 
pixelinkHelper::IsSettableWhileStreaming(U32 featureId)
{
	switch(featureId)
	{
		case FEATURE_LOOKUP_TABLE:
		case FEATURE_FRAME_RATE:
		case FEATURE_PIXEL_FORMAT:
		case FEATURE_PIXEL_ADDRESSING:
		case FEATURE_ROI:
		case FEATURE_TRIGGER:
			return false;
		default:
			return true;
	}
}


#define ASSERT(x)	do { assert((x)); } while(0)
// 'Globals' shared between our main line, and the clip callback
static bool captureFinished = false;
static U32  numImagesStreamed = 0;
static PXL_RETURN_CODE captureRc = ApiSuccess;
static U32 CaptureDoneCallback(HANDLE hCamera, U32 numFramesCapture, PXL_RETURN_CODE returnCode);

//
// Get a video from the camera, and save to a file.
//
int 
pixelinkHelper::GetVideo(HANDLE hCamera, std::string fileName, float videoRecordTime, U32 videoDecimation, float playbackFrameRate)
{
    U32  recordTime = videoRecordTime;
    U32  decimation = videoDecimation;
    U32  frameRate = playbackFrameRate;
    std::string test = fileName;
    char* rootName = (char *)test.c_str();
    std::vector<char> aviFile(256,0);
    std::vector<char> mp4File(256,0);

    //Determine the effective frame rate for the camera, and the number of images we will need to
    //capture the video of the requested length then start the stream
    float cameraFps = EffectiveFrameRate(hCamera,frameRate);
    U32   numImages = (U32)(((float)recordTime) * cameraFps);
    numImages = numImages/decimation + 1; // Include the decimation factor

    std::vector<char> h264File(256,0);
    strncpy (&h264File[0],rootName,256);
    strncat (&h264File[0],".h264",256);
    CLIP_ENCODING_INFO clipInfo;

    clipInfo.uStreamEncoding = CLIP_ENCODING_H264;
    clipInfo.uDecimationFactor = decimation;
    clipInfo.playbackFrameRate = (float)frameRate;
    clipInfo.playbackBitRate = CLIP_PLAYBACK_BITRATE_DEFAULT;

    captureFinished = false;
    PXL_RETURN_CODE rc = PxLGetEncodedClip (hCamera, 20, &h264File[0], &clipInfo, CaptureDoneCallback);

    while (!captureFinished) //wait for capture to finish
    {
	
    }

    if (API_SUCCESS (captureRc))
    {
	if (fileName.find(".avi") != std::string::npos)
	{
		rc = PxLFormatClipEx (&h264File[0], rootName, CLIP_ENCODING_H264, CLIP_FORMAT_AVI);
	}
	else
	{
		rc = PxLFormatClipEx (&h264File[0], rootName, CLIP_ENCODING_H264, CLIP_FORMAT_MP4);
	}
    } else {
       rc = captureRc;
    }    
    
    return rc;

}

//
// Function that's called when PxLGetClip is finished capturing frames, or can't continue
// capturing frames.
//
static U32 CaptureDoneCallback(HANDLE hCamera, U32 numFramesCapture, PXL_RETURN_CODE returnCode)
{
    // Just record the capture information into our shared (global) varaibles so the main line
    // can report/take action on the result.
    numImagesStreamed = numFramesCapture;
    captureRc = returnCode;
    captureFinished = true;
    return ApiSuccess;
}

// 
// Returns the frame rate being used by the camera.  Ideally, this is simply FEAUTURE_ACTUAL_FRAME_RATE, but
// some older cameras do not support that.  If that is the case, use FEATURE_FRAME_RATE, which is 
// always supported.
//
float 
pixelinkHelper::EffectiveFrameRate (HANDLE hCamera,U32 playbackframeRate)
{
    float frameRate = playbackframeRate;
    PXL_RETURN_CODE rc;
    
    //
    // Step 1
    //      Determine if the camera supports FEATURE_ACTUAL_FRAME_RATE

    // How big a buffer will we need to hold the information about the trigger feature?
    U32 bufferSize = -1;
    U32 frameRateFeature = FEATURE_FRAME_RATE;
    if (API_SUCCESS (PxLGetCameraFeatures(hCamera, FEATURE_ACTUAL_FRAME_RATE, NULL, &bufferSize)))
    {
	    ASSERT(bufferSize > 0);

	    // Declare a buffer and read the feature information
	    std::vector<U8> buffer(bufferSize, 0);  // zero-initialized buffer
	    CAMERA_FEATURES* pCameraFeatures = (CAMERA_FEATURES*)&buffer[0];
	    if (API_SUCCESS (PxLGetCameraFeatures(hCamera, FEATURE_ACTUAL_FRAME_RATE, pCameraFeatures, &bufferSize)))
            {
                //
                //  Step 2
                //      Get the 'best available' frame rate of the camera
                if (pCameraFeatures[0].pFeatures->uFlags & FEATURE_FLAG_PRESENCE)
                {
                    frameRateFeature = FEATURE_ACTUAL_FRAME_RATE;
                }
            }
    }
    
    U32 flags;
    U32 numParams = 1;
    rc = PxLGetFeature (hCamera, frameRateFeature, &flags, &numParams, &frameRate);
    ASSERT(API_SUCCESS(rc));

    return frameRate;
}





