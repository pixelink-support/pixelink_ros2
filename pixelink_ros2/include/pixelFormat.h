
/***************************************************************************
 *
 *     File: pixelFormat.h
 *
 *     Description: Simple wrapper class for all of the pixel format controls
 *
 */

#if !defined(PIXELINK_PIXEL_FORMAT_H)
#define PIXELINK_PIXEL_FORMAT_H

#include <assert.h>
#include "PixeLINKApi.h"
#include <stdio.h>
#include <stdlib.h>


class PxLPixelFormat
{
public:

    inline static std::string fromApi(float apiPixelFormat)
    {
        switch ((int)apiPixelFormat)
        {
        case PIXEL_FORMAT_MONO8:
            return "MONO8";
        case PIXEL_FORMAT_MONO16:
            return "MONO16";
        case PIXEL_FORMAT_MONO10_PACKED_MSFIRST:
            return "MONO10_PACKED";
        case PIXEL_FORMAT_MONO12_PACKED:
        case PIXEL_FORMAT_MONO12_PACKED_MSFIRST:
            return "MONO12_PACKED";
        case PIXEL_FORMAT_BAYER8:
        case PIXEL_FORMAT_BAYER8_RGGB:
        case PIXEL_FORMAT_BAYER8_GBRG:
        case PIXEL_FORMAT_BAYER8_BGGR:
            return "BAYER8";
        case PIXEL_FORMAT_BAYER16:
        case PIXEL_FORMAT_BAYER16_RGGB:
        case PIXEL_FORMAT_BAYER16_GBRG:
        case PIXEL_FORMAT_BAYER16_BGGR:
            return "BAYER16";
        case PIXEL_FORMAT_BAYER10_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_RGGB_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_GBRG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_BGGR_PACKED_MSFIRST:
            return "BAYER10_PACKED";
        case PIXEL_FORMAT_BAYER12_PACKED:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED:
        case PIXEL_FORMAT_BAYER12_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED_MSFIRST:
            return "BAYER12_PACKED";
        case PIXEL_FORMAT_YUV422:
            return "YUV422";
        case PIXEL_FORMAT_STOKES4_12:
            return "STOKES";
        case PIXEL_FORMAT_POLAR4_12:
            return "POLAR";
        case PIXEL_FORMAT_POLAR_RAW4_12:
            return "POLAR_RAW";
        case PIXEL_FORMAT_HSV4_12:
            return "HSV";
        case PIXEL_FORMAT_RGB24_NON_DIB:
            return "RGB24";
        case PIXEL_FORMAT_BGR24_NON_DIB:
            return "BGR24";
        default:
            return "MONO8";
        }
    }

    inline static float toApi (std::string pixelFormat)
    {

        if 	(pixelFormat == "MONO8")          return (float) PIXEL_FORMAT_MONO8;
        else if (pixelFormat == "MONO16")         return (float) PIXEL_FORMAT_MONO16;
        else if (pixelFormat == "MONO10_PACKED")  return (float) PIXEL_FORMAT_MONO10_PACKED_MSFIRST;
        else if (pixelFormat == "MONO12_PACKED")  return (float) PIXEL_FORMAT_MONO12_PACKED_MSFIRST;
        else if (pixelFormat == "BAYER8")         return (float) PIXEL_FORMAT_BAYER8;
        else if (pixelFormat == "BAYER16")        return (float) PIXEL_FORMAT_BAYER16;
        else if (pixelFormat == "BAYER10_PACKED") return (float) PIXEL_FORMAT_BAYER10_PACKED_MSFIRST;
        else if (pixelFormat == "BAYER12_PACKED") return (float) PIXEL_FORMAT_BAYER12_PACKED_MSFIRST;
        else if (pixelFormat == "YUV422")         return (float) PIXEL_FORMAT_YUV422;
        else if (pixelFormat == "STOKES")         return (float) PIXEL_FORMAT_STOKES4_12;
        else if (pixelFormat == "POLAR")          return (float) PIXEL_FORMAT_POLAR4_12;
        else if (pixelFormat == "POLAR_RAW")      return (float) PIXEL_FORMAT_POLAR_RAW4_12;
        else if (pixelFormat == "HSV")            return (float) PIXEL_FORMAT_HSV4_12;
        else if (pixelFormat == "RGB24")          return (float) PIXEL_FORMAT_RGB24_NON_DIB;
        else if (pixelFormat == "BGR24")          return (float) PIXEL_FORMAT_BGR24_NON_DIB;
        else            			  return (float) PIXEL_FORMAT_MONO8;
    }

    inline static std::string toSensorMessageEncoding(float apiPixelFormat)
    {
        switch ((int)apiPixelFormat)
        {
	case PIXEL_FORMAT_MONO8:
		return "mono8";
	case PIXEL_FORMAT_MONO16:
		return "mono16";
	case PIXEL_FORMAT_MONO10_PACKED_MSFIRST:
		return "MONO10_PACKED";
	case PIXEL_FORMAT_MONO12_PACKED:
	case PIXEL_FORMAT_MONO12_PACKED_MSFIRST:
		return "MONO12_PACKED";
	case PIXEL_FORMAT_BAYER8:
	case PIXEL_FORMAT_BAYER8_RGGB:
		return "bayer_rggb8";
	case PIXEL_FORMAT_BAYER8_GBRG:
		return "bayer_gbrg8";
	case PIXEL_FORMAT_BAYER8_BGGR:
		return "bayer_bggr8";
	case PIXEL_FORMAT_BAYER16:
	case PIXEL_FORMAT_BAYER16_RGGB:
		return "bayer_rggb16";
	case PIXEL_FORMAT_BAYER16_GBRG:
		return "bayer_gbrg16";
	case PIXEL_FORMAT_BAYER16_BGGR:
		return "bayer_bggr8";
	case PIXEL_FORMAT_YUV422:
		return "yuv422";
	case PIXEL_FORMAT_RGB24_NON_DIB:
		return "rgb8";
	case PIXEL_FORMAT_BGR24_NON_DIB:
		return "bgr8";
	default:
		return "MONO8";
        }
    }


    /**
     * Function: GetBytesPerPixel()
     *
     * Description: Return the number of bits per pixel for a specific Pixel Format
     *
     */
    inline static float bytesPerPixel (U32 pixelFormat)
    {
        switch (pixelFormat)
        {
        case PIXEL_FORMAT_MONO8:
        case PIXEL_FORMAT_BAYER8_RGGB:
        case PIXEL_FORMAT_BAYER8_GRBG:
        case PIXEL_FORMAT_BAYER8_GBRG:
        case PIXEL_FORMAT_BAYER8_BGGR:
            return 1.0f;
        case PIXEL_FORMAT_MONO12_PACKED:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED:
        case PIXEL_FORMAT_BAYER12_GRBG_PACKED:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED:
        case PIXEL_FORMAT_MONO12_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_RGGB_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_GRBG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_GBRG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER12_BGGR_PACKED_MSFIRST:
            return 1.5f;
        case PIXEL_FORMAT_MONO10_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_RGGB_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_GRBG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_GBRG_PACKED_MSFIRST:
        case PIXEL_FORMAT_BAYER10_BGGR_PACKED_MSFIRST:
            return 1.25f;
        case PIXEL_FORMAT_MONO16:
        case PIXEL_FORMAT_BAYER16_RGGB:
        case PIXEL_FORMAT_BAYER16_GRBG:
        case PIXEL_FORMAT_BAYER16_GBRG:
        case PIXEL_FORMAT_BAYER16_BGGR:
        case PIXEL_FORMAT_YUV422:
            return 2.0f;
        case PIXEL_FORMAT_RGB24:
        case PIXEL_FORMAT_RGB24_NON_DIB:  // Bugzilla.1239
        case PIXEL_FORMAT_BGR24_NON_DIB:
            return 3.0f;

        case PIXEL_FORMAT_RGB48:
        case PIXEL_FORMAT_RGB48_DIB: // Bugzilla.1239
            return 6.0f;
        case PIXEL_FORMAT_STOKES4_12:
        case PIXEL_FORMAT_POLAR4_12:
        case PIXEL_FORMAT_POLAR_RAW4_12:
        case PIXEL_FORMAT_HSV4_12:
            return 6.0f;
        default:
            assert("Invalid pixel format");
        }

        return -1.0f;
    }

};

#endif // !defined(PIXELINK_PIXEL_FORMAT_H)
