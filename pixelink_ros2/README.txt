INTALLATION:

1)Install Pixelink SDK as described in the SDK documentation.

2)Install ROS2 as described on the ROS2 website -> https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

3)Install Colcon in order to build the Pixelink Ros2 Package -> https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

3)Create a workspace as described in the above tutorial and place the Pixelink ROS2 package in it. Navigate to the root of the workspace.

4)Run the following command to build the package:

	colcon build --packages-above pixelink_ros2

HOW TO USE:

To use the Pixelink Ros2 adapter, open a new terminal. Navigate to the root of your workspace. Setup the environment in the terminal to run ros2 and pixelink_ros2 by using the following two commands:

	source /opt/ros/humble/setup.bash
	. install/setup.bash

NOTE: Be sure to set up your environment each time you open a new terminal to use ros2 and pixelink_ros2. If you do not want to have to do this every time you open a new terminal you can setup the environment using your shell startup script. See this tutorial for more information -> https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html



To run pixelink_ros2 with a single camera, open a new terminal and run the following line:

	ros2 run pixelink_ros2 server

You can get a list of all Pixelink camera services and their respective service types by using the command:

	ros2 service list -t

If you are planning on using multiple cameras you must specifiy a server name:

	ros2 run pixelink_ros2 server --ros-args --remap __node:=<SERVER_NAME>
	
	NOTE:If you specify a server name the server name will act as a prefix to the existing service name. For example, if you name your server "pixelink1" the "/initialize" 	service name will become "/pixelink1/intialize". Use the command "ros2 service list -t" to ensure you are using the correct service name.	
	
Once the server is running you must initialize your camera. If a single camera is connected you can use this command to initialize it:

	ros2 service call /initialize pixelink_ros2_interfaces/srv/NoParams
	
If using multiple cameras you must specify which camera you would like to initialize by specifying a serial number:

	ros2 service call /initializeWithSerial pixelink_ros2_interfaces/srv/SerialParam "{serial: 999999999}"
	
To get a list of the serial numbers of the attached cameras use this command:

	ros2 service call /getSerialNumbers pixelink_ros2_interfaces/srv/GetSerialNumbers
	
Once the camera is initialized you can start the camera stream with this command:

	ros2 service call /startStream pixelink_ros2_interfaces/srv/NoParams
	
Images captured by the camera will be published as a topic. The naming structure of this topic is based on the same of the pixelink_ros2 server. Find the published topics by using this command:

	ros2 topic list	
	
You can test if the camera is streaming correctly by using image_view. Install image view by using:

	sudo apt-get install ros-<CURRENT ROS2 DISTRO>-image-view

To view the image in image view use this command:

	ros2 run image_view image_view --ros-args --remap image:=<TOPIC_NAME>

To stop the stream use:

	ros2 service call /stopStream pixelink_ros2_interfaces/srv/NoParams

To uninitialize the camera use:

	ros2 service call /uninitialize pixelink_ros2_interfaces/srv/NoParams
	

CAMERA TOOLS AND FEATURES:

Initialize/Uninitialize Camera:

	Initialize:	
		Parameters:
		none
		
		example service call:
		ros2 service call /initialize pixelink_ros2_interfaces/srv/NoParams
		
	Initialize with Serial Number:	
		Parameters:
		none
		
		example service call:
		ros2 service call /initializeWithSerial pixelink_ros2_interfaces/srv/SerialParam "{serial: 999999999}"
		
	Uninitialize:	
		Parameters:
		none
		
		example service call:
		ros2 service call /uninitialize pixelink_ros2_interfaces/srv/NoParams

Start/Stop Stream:

	Start Stream:	
		Parameters:
		none
		
		example service call:
		ros2 service call /startStream pixelink_ros2_interfaces/srv/NoParams
		
	Stop Stream:	
		Parameters:
		none
		
		example service call:
		ros2 service call /stopStream pixelink_ros2_interfaces/srv/NoParams

Get Camera Information:
	Parameters:
	none
	
	example service call:
	ros2 service call /getCameraInfo pixelink_ros2_interfaces/srv/GetCameraInfo

Preview Stream:

	Start Preview:	
		Parameters:
		none
		
		example service call:
		ros2 service call /startPreview pixelink_ros2_interfaces/srv/NoParams
		
	Stop Preview:	
		Parameters:
		none
		
		example service call:
		ros2 service call /stopPreview pixelink_ros2_interfaces/srv/NoParams

Save Images:

	Parameters:
	string file_name

	example service call:
	ros2 service call /captureImageAsBmp pixelink_ros2_interfaces/srv/CaptureParams "{file_name: testImage.bmp}"
	ros2 service call /captureImageAsJpg pixelink_ros2_interfaces/srv/CaptureParams "{file_name: testImage.jpg}"
	ros2 service call /captureImageAsPsd pixelink_ros2_interfaces/srv/CaptureParams "{file_name: testImage.psd}"
	ros2 service call /captureImageAsTiff pixelink_ros2_interfaces/srv/CaptureParams "{file_name: testImage.tiff}"
	ros2 service call /captureImageAsRaw pixelink_ros2_interfaces/srv/CaptureParams "{file_name: testImage.raw}"
	
Capture Video:

	Parameters:
	string file_name
	float32 record_time (in seconds)
	int32 decimation (skip capturing every Nth) frame
	float32 playback_frame_rate (frame rate the captured video will play at)
	
	example service call:
	ros2 service call /captureVideoAsAvi pixelink_ros2_interfaces/srv/CaptureVideoParams "{file_name: testVideo.avi, record_time: 3, decimation: 1, playback_frame_rate: 24}"
	ros2 service call /captureVideoAsAvi pixelink_ros2_interfaces/srv/CaptureVideoParams "{file_name: testVideo.mp4, record_time: 3, decimation: 1, playback_frame_rate: 24}"

Exposure:
	Get Exposure:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getExposure pixelink_ros2_interfaces/srv/GetParams

	Set Exposure:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setExposure pixelink_ros2_interfaces/srv/SetParams "{params: {0.03}}"
		
	Get Exposure Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getExposureRange pixelink_ros2_interfaces/srv/GetRange
		
	Run Auto Exposure:	
		Parameters:
		none
		
		example service call:
		ros2 service call /runAutoExposure pixelink_ros2_interfaces/srv/NoParams
		
	Enable Continuous Exposure:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableContinuousExposure pixelink_ros2_interfaces/srv/NoParams
		
	Disable Continuous Exposure:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableContinuousExposure pixelink_ros2_interfaces/srv/NoParams



Gain:
	Get Gain:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getGain pixelink_ros2_interfaces/srv/GetParams

	Set Gain:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setGain pixelink_ros2_interfaces/srv/SetParams "{params: {12.0}}"
		
	Get Gain Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getGainRange pixelink_ros2_interfaces/srv/GetRange
		
	Run Auto Gain:	
		Parameters:
		none
		
		example service call:
		ros2 service call /runAutoGain pixelink_ros2_interfaces/srv/NoParams
		
	Enable Continuous Gain:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableContinuousGain pixelink_ros2_interfaces/srv/NoParams
		
	Disable Continuous Gain:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableContinuousGain pixelink_ros2_interfaces/srv/NoParams

HDR:
	Enable Camera HDR:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableCameraHDR pixelink_ros2_interfaces/srv/NoParams

	Enable Interleaved HDR:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableInterleavedHDR pixelink_ros2_interfaces/srv/NoParams
		
	Disable HDR:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableHDR pixelink_ros2_interfaces/srv/NoParams
		
	Get HDR:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getHDR pixelink_ros2_interfaces/srv/GetParams

Gamma:
	Get Gamma:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getGamma pixelink_ros2_interfaces/srv/GetParams

	Set Gamma:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setGamma pixelink_ros2_interfaces/srv/SetParams "{params: {2.2}}"
		
	Get Gamma Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getGammaRange pixelink_ros2_interfaces/srv/GetRange
		
	Enable Gamma:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableGamma pixelink_ros2_interfaces/srv/NoParams
		
	Disable Gamma:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableGamma pixelink_ros2_interfaces/srv/NoParams

Saturation:
	Get Saturation:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getSaturation pixelink_ros2_interfaces/srv/GetParams

	Set Saturation:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setSaturation pixelink_ros2_interfaces/srv/SetParams "{params: {150}}"
		
	Get Saturation Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getSaturationRange pixelink_ros2_interfaces/srv/GetRange

Frame Rate:
	Get Frame Rate:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFrameRate pixelink_ros2_interfaces/srv/GetParams

	Set Frame Rate:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setFrameRate pixelink_ros2_interfaces/srv/SetParams "{params: {28}}"
		
	Get Frame Rate Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFrameRateRange pixelink_ros2_interfaces/srv/GetRange
		
	Enable Continuous Frame Rate:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableContinuousFrameRate pixelink_ros2_interfaces/srv/NoParams
		
	Disable Continuous Frame Rate:		
		Parameters:
		none
		
		example service call:
		ros2 service call /disableContinuousFrameRate pixelink_ros2_interfaces/srv/NoParams

	Enable Fixed Frame Rate:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableFixedFrameRate pixelink_ros2_interfaces/srv/NoParams
		
	Disable Fixed Frame Rate:		
		Parameters:
		none
		
		example service call:
		ros2 service call /disableFixedFrameRate pixelink_ros2_interfaces/srv/NoParams

White Balance:
	Get White Balance:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getWhiteBalance pixelink_ros2_interfaces/srv/GetParams

	Set White Balance:
		Parameters:
		float32[] params (0-RED, 1-GREEN, 2-BLUE)
		
		example service call:
		ros2 service call /setWhiteBalance pixelink_ros2_interfaces/srv/SetParams "{params: {1.0,0.9,1.2}}"
		
	Run Auto White Balance:	
		Parameters:
		none
		
		example service call:
		ros2 service call /runAutoWhiteBalance pixelink_ros2_interfaces/srv/NoParams
		
	Get White Balance Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getWhiteBalanceRange pixelink_ros2_interfaces/srv/GetRange

Pixel Addressing:
	Get Pixel Addressing:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getPixelAddressing pixelink_ros2_interfaces/srv/GetParamsPixelAddressing

	Set Pixel Addressing Binning:
		Parameters:
		float32 horizontal
		float32 vertical
		
		example service call:
		ros2 service call /setPixelAddressingBinning pixelink_ros2_interfaces/srv/SetParamsPixelAddressing "{horizontal: 2.0, vertical: 2.0}"
		
	Set Pixel Addressing Decimation:
		Parameters:
		float32 horizontal
		float32 vertical
		
		example service call:
		ros2 service call /setPixelAddressingDecimation pixelink_ros2_interfaces/srv/SetParamsPixelAddressing "{horizontal: 2.0, vertical: 2.0}"
		
	Disable Pixel Addressing:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disablePixelAddressing pixelink_ros2_interfaces/srv/NoParams
		
	Get Pixel Addressing Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getPixelAddressingRange pixelink_ros2_interfaces/srv/GetRange

Pixel Format:
	Get Supported Pixel Formats:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getSupportedPixelFormats pixelink_ros2_interfaces/srv/GetSupportedPixelFormats
		
	Get Pixel Format:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getPixelFormat pixelink_ros2_interfaces/srv/GetPixelFormat

	Set Pixel Format:
		Parameters:
		string pixel_format
		
		example service call:
		ros2 service call /setPixelFormat pixelink_ros2_interfaces/srv/SetPixelFormat "{pixel_format: MONO8}"

Flip:
	Get Flip:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFlip pixelink_ros2_interfaces/srv/GetParams
		
	Enable Horizontal Flip:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableHorizontalFlip pixelink_ros2_interfaces/srv/NoParams
		
	Disable Horizontal Flip:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableHorizontalFlip pixelink_ros2_interfaces/srv/NoParams

	Enable Vertical Flip:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableVerticalFlip pixelink_ros2_interfaces/srv/NoParams
		
	Disable Vertical Flip:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableVerticalFlip pixelink_ros2_interfaces/srv/NoParams

Rotation:
	Get Rotation:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getRotation pixelink_ros2_interfaces/srv/GetParams

	Set Rotation:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setRotation pixelink_ros2_interfaces/srv/SetParams "{params: {270}}"

Bandwidth Limit:
	Get Bandwidth Limit:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getBandwidthLimit pixelink_ros2_interfaces/srv/GetParams

	Set Bandwidth Limit:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setBandwidthLimit pixelink_ros2_interfaces/srv/SetParams "{params: {2600}}"
		
	Get Bandwidth Limit Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getBandwidthLimitRange pixelink_ros2_interfaces/srv/GetRange
		
	Enable Bandwidth Limit:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableBandwidthLimit pixelink_ros2_interfaces/srv/NoParams
		
	Disable Bandwidth Limit:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableBandwidthLimit pixelink_ros2_interfaces/srv/NoParams

Focus:
	Get Focus:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFocus pixelink_ros2_interfaces/srv/GetParams

	Set Focus:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setFocus pixelink_ros2_interfaces/srv/SetParams "{params: {16000}}"
		
	Run Auto Focus:	
		Parameters:
		none
		
		example service call:
		ros2 service call /runAutoFocus pixelink_ros2_interfaces/srv/NoParams
		
	Get Focus Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFocusRange pixelink_ros2_interfaces/srv/GetRange

Color Temperature:
	Get Color Temperature:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getColorTemperature pixelink_ros2_interfaces/srv/GetParams

	Set Color Temperature:
		Parameters:
		float32[] params
		
		example service call:
		ros2 service call /setColorTemperature pixelink_ros2_interfaces/srv/SetParams "{params: {5000}}"
		
	Get Color Temperature Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getColorTemperatureRange pixelink_ros2_interfaces/srv/GetRange

Triggering:

	Set Triggering:	
		Parameters: (see Pixelink knowledge base for a detailed explanation of camera triggering parameters)
		int32 mode
		int32 trigger_type
		int32 polarity
		float32 delay
		float32 parameters
		
		example service call:
		ros2 service call /setTriggering pixelink_ros2_interfaces/srv/SetTriggerParams "{mode: 0, trigger_type: 1, polarity: 0, delay: 0, parameters: 0}"
		
	Disable Triggering:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableTriggering pixelink_ros2_interfaces/srv/NoParams

Gpio:

	Enable GPIO:	
		Parameters: (see Pixelink knowledge base for a detailed explanation of camera gpio parameters)
		int32 gpio_number
		int32 mode
		int32 polarity
		float32 parameter_one
		float32 parameter_two
		float32 parameter_three
		
		example service call:
		ros2 service call /enableGpio pixelink_ros2_interfaces/srv/SetGpioParams "{gpio_number: 0, mode: 0, polarity: 0, parameter_one: 0, parameter_two: 0, parameter_three: 0}"
		
	Disable GPIO:	
		Parameters:
		int32 gpio_number
		int32 mode
		int32 polarity
		float32 parameter_one
		float32 parameter_two
		float32 parameter_three
		
		example service call:
		ros2 service call /disableGpio pixelink_ros2_interfaces/srv/SetGpioParams "{gpio_number: 0, mode: 0, polarity: 0, parameter_one: 0, parameter_two: 0, parameter_three: 0}"

Region of Interest:
	Get Region of Interest:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getRoi pixelink_ros2_interfaces/srv/GetParamsRoi

	Set Region of Interest:
		Parameters:
		float32 left
		float32 top
		float32 width
		float32 height
		
		example service call:
		ros2 service call /setRoi pixelink_ros2_interfaces/srv/SetParamsRoi "{left: 0, top: 0, width: 1024, height: 1024}"
		
	Get Region of Interest Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getRoiRange pixelink_ros2_interfaces/srv/GetRoiRange
		
		

Focus Score Roi:
	Get Focus Score Roi:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFocusScoreRoi pixelink_ros2_interfaces/srv/GetParamsRoi

	Set Focus Score Region of Interest:
		Parameters:
		float32 left
		float32 top
		float32 width
		float32 height
		
		example service call:
		ros2 service call /setFocusScoreRoi pixelink_ros2_interfaces/srv/SetParamsRoi "{left: 0, top: 0, width: 1024, height: 1024}"
		
	Get Focus Score Region of Interest Range:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getFocusScoreRoiRange pixelink_ros2_interfaces/srv/GetRoiRange

Auto Roi:
	Get Auto Roi:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getAutoRoi pixelink_ros2_interfaces/srv/GetParamsRoi

	Set Auto Roi:
		Parameters:
		float32 left
		float32 top
		float32 width
		float32 height
		
		example service call:
		ros2 service call /setAutoRoi pixelink_ros2_interfaces/srv/SetParamsRoi "{left: 0, top: 0, width: 1024, height: 1024}"

	Enable Auto Roi:	
		Parameters:
		none
		
		example service call:
		ros2 service call /enableAutoRoi pixelink_ros2_interfaces/srv/NoParams
		
	Disable Auto Roi:	
		Parameters:
		none
		
		example service call:
		ros2 service call /disableAutoRoi pixelink_ros2_interfaces/srv/NoParams
		
	Get Auto Roi:	
		Parameters:
		none
		
		example service call:
		ros2 service call /getAutoRoiRange pixelink_ros2_interfaces/srv/GetRoiRange

Save/Load Camera Settings:
	Save Camera Settings	
		Parameters:
		none
		
		example service call:
		ros2 service call /saveSettings pixelink_ros2_interfaces/srv/NoParams

	Load Camera Settings	
		Parameters:
		none
		
		example service call:
		ros2 service call /loadSettings pixelink_ros2_interfaces/srv/NoParams

	Restore Factory Defaults:
		Parameters:
		none
		
		example service call:
		ros2 service call /restoreFactoryDefaults pixelink_ros2_interfaces/srv/NoParams	









