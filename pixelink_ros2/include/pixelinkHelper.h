#include "PixeLINKApi.h"
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>

// Local macros for return values
#ifndef SUCCESS
#define SUCCESS (0)
#endif
#ifndef FAILURE
#define FAILURE (1)
#endif

class pixelinkHelper
{
public:
	static PXL_RETURN_CODE GetNextFrame(HANDLE hCamera, U32 bufferSize, void* pFrame, FRAME_DESC* pFrameDesc);
	static int GetSnapshotAndSave(HANDLE hCamera, U32 imageFormat, const char* pFilename, char* pRawImage, U32 rawImageSize, FRAME_DESC* pFrameDesc);
	static bool IsSettableWhileStreaming(U32 imageFormat);
	static int GetVideo(HANDLE hCamera, std::string fileName, float videoRecordTime, U32 videoDecimation, float playbackFrameRate);
	static U32 DetermineRawImageSize(HANDLE hCamera);
private:
	static U32 GetPixelSize(U32 pixelFormat);
	static int EncodeRawImage(const char*, const FRAME_DESC*, U32, char**, U32*);
	static int SaveImageToFile(const char* pFilename, const char* pImage, U32 imageSize);
	static int GetRawImage(HANDLE hCamera, char* pRawImage, U32 rawImageSize, FRAME_DESC* pFrameDesc);
	static int GetParameters (int argc, char* argv[], U32* recordTime, U32* decimation, U32* imagePeriod, U32* frameRte, char** fileNames);
	static float EffectiveFrameRate (HANDLE hCamera,U32 playbackframeRate);
};

