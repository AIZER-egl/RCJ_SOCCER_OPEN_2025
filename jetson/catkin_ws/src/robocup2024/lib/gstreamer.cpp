#include "gstreamer.h"

Gstreamer::Gstreamer() {
    gstreamer_command = "";
}

std::string Gstreamer::get_command() {
    this -> gstreamer_command = "nvarguscamerasrc ";

    /**
     * Name         : The name of the object
     *                flags: readable, writeable
     *                String. Default: "nvarguscamerasrc0"
     * */
    if ((this -> parameters).name != "nvarguscamerasrc0")
        this -> gstreamer_command += "name=" + (this -> parameters).name + " ";

    /**
     * Parent       : The parent of the object
     *                flags: readable, writeable
     *                Object of type "GstObject"
     * */
    // Parent is not implemented in this class

    /**
     * blockSize    : Size of the buffer block in bytes (-1 = default)
     *                flags: readable, writeable
     *                Unsigned Integer. Range: 1 - 4294967295 Default: 4096
     * */
     if ((this -> parameters).block_size != 4096 && (this -> parameters).block_size != -1)
        this -> gstreamer_command += "blocksize=" + std::to_string((this -> parameters).block_size) + " ";

    /**
     * num-buffers  : Number of buffers to output before sending EOS (-1 = unlimited)
     *               flags: readable, writeable
     *               Integer. Range: -1 - 2147483647 Default: -1
     */
    if ((this -> parameters).num_buffers != -1)
        this -> gstreamer_command += "num-buffers=" + std::to_string((this -> parameters).num_buffers) + " ";

    /**
     * typefind     : Run typefind before negotiating (deprecated, non-functional)
     *               flags: readable, writeable
     *               Boolean. Default: false
     */
    if ((this -> parameters).type_find)
        this -> gstreamer_command += "typefind=" + std::to_string((this -> parameters).type_find) + " ";

    /**
     * do-timestamp : Apply current stream time to buffers
     *               flags: readable, writeable
     *               Boolean. Default: true
     */
     if (!(this -> parameters).do_timestamp)
        this -> gstreamer_command += "do-timestamp=" + std::to_string((this -> parameters).do_timestamp) + " ";

    /**
     * silent       : Produce verbose output
     *               flags: readable, writeable
     *               Boolean. Default: true
     */
    if (!(this -> parameters).silent)
        this -> gstreamer_command += "silent=" + std::to_string((this -> parameters).silent) + " ";

    /**
     * timeout      : timeout to capture in seconds (Either specify timeout or num-buffers, not both)
     *               flags: readable, writeable
     *               Integer64. Range: 0 - 2147483647 Default: 0
     */
    if ((this -> parameters).timeout != 0 && (this -> parameters).num_buffers == -1)
        this -> gstreamer_command += "timeout=" + std::to_string((this -> parameters).timeout) + " ";

    /**
     * wbmode       : White balance affects the color temperature of the photo
     *               flags: readable, writeable
     *               Enum "GstNvArgusCamWBMode" Default: 1, "auto"
     *               (0): off              - GST_NVCAM_WB_MODE_OFF
     *               (1): auto             - GST_NVCAM_WB_MODE_AUTO
     *               (2): incandescent     - GST_NVCAM_WB_MODE_INCANDESCENT
     *               (3): fluorescent      - GST_NVCAM_WB_MODE_FLUORESCENT
     *               (4): warm-fluorescent - GST_NVCAM_WB_MODE_WARM_FLUORESCENT
     *               (5): daylight         - GST_NVCAM_WB_MODE_DAYLIGHT
     *               (6): cloudy-daylight  - GST_NVCAM_WB_MODE_CLOUDY_DAYLIGHT
     *               (7): twilight         - GST_NVCAM_WB_MODE_TWILIGHT
     *               (8): shade            - GST_NVCAM_WB_MODE_SHADE
     *               (9): manual           - GST_NVCAM_WB_MODE_MANUAL
     */
    if ((this -> parameters).white_balance != AUTO)
        this -> gstreamer_command += "wbmode=" + std::to_string((this -> parameters).white_balance) + " ";

    /**
     * saturation  : Property to adjust saturation value
     *              flags: readable, writeable
     *              Float. Range: 0 - 2 Default: 1
     */
    if ((this -> parameters).saturation != 1.0)
        this -> gstreamer_command += "saturation=" + std::to_string((this -> parameters).saturation) + " ";

    /**
     * sensor-id   : Set the id of the camera sensor to use. Default 0.
     *             flags: readable, writeable
     *             Integer. Range: 0 - 255 Default: 0
     */
    if ((this -> parameters).sensor_id != 0)
        this -> gstreamer_command += "sensor-id=" + std::to_string((this -> parameters).sensor_id) + " ";

    /**
     * sensor-mode : Set the source sensor mode to use. Default -1 (Select the best match)
     *             flags: readable, writeable
     *             Integer. Range: -1 - 255 Default: -1
     */
    if ((this -> parameters).sensor_mode != -1)
        this -> gstreamer_command += "sensor-mode=" + std::to_string((this -> parameters).sensor_mode) + " ";

    /**
     * total-sensor-modes : Total number of sensor modes available. Default 0.
     *                    flags: readable, writeable
     *                    Integer. Range: 0 - 255 Default: 0
     */
    if ((this -> parameters).total_sensor_modes != 0)
        this -> gstreamer_command += "total-sensor-modes=" + std::to_string((this -> parameters).total_sensor_modes) + " ";

    /**
     * exposuretimerange : Set the exposure time range in nanoseconds. Default 0
     *                     Use string with values of Exposure time range (low, high)
     *                     in that order, to set the property.
     *                     eg: exposuretimerange="34000 358733000"
     *                     flags: readable, writeable
     *                     String. Default: null
     */
    if ((this -> parameters).min_exposure_timerange != 0 && (this -> parameters).max_exposure_timerange != 0)
        this -> gstreamer_command += "exposuretimerange=\"" + std::to_string((this -> parameters).min_exposure_timerange) + " " + std::to_string((this -> parameters).max_exposure_timerange) + "\" ";

    /**
     * gainrange   : Property to adjust gain range
     *             Use string with values of Gain TIme Range (low, high)
     *             in that order, to set the property.
     *             eg: gainrange="1 16"
     *             flags: readable, writeable
     *             String. Default: null
     */
    if ((this -> parameters).min_gain_range != 0 && (this -> parameters).max_gain_range != 0)
        this -> gstreamer_command += "gainrange=\"" + std::to_string((this -> parameters).min_gain_range) + " " + std::to_string((this -> parameters).max_gain_range) + "\" ";

    /**
     * ispdigitalgainrange : Property to adjust digital gain range
     *                     Use string with values of ISP Digital Gain Range (low, high)
     *                     in that order, to set the property.
     *                     eg: ispdigitalgainrange="1 8"
     *                     flags: readable, writeable
     *                     String. Default: null
     */
    if ((this -> parameters).min_isp_digital_gain_range != 0 && (this -> parameters).max_isp_digital_gain_range != 0)
        this -> gstreamer_command += "ispdigitalgainrange=\"" + std::to_string((this -> parameters).min_isp_digital_gain_range) + " " + std::to_string((this -> parameters).max_isp_digital_gain_range) + "\" ";

    /**
     * tnr-strength : Property to adjust temporal noise reduction strength
     *             flags: readable, writeable
     *             Float. Range: -1 - 1 Default: -1
     */
    if ((this -> parameters).tnr_strength != -1)
        this -> gstreamer_command += "tnr-strength=" + std::to_string((this -> parameters).tnr_strength) + " ";

    /**
     * tnr-mode    : Property to adjust temporal noise reduction mode
     *            flags: readable, writeable
     *            Enum "GstNvArgusCamTNRMode" Default: 1, "NoiseReduction_Fast"
     *            (0): NoiseReduction_Off         - GST_NVCAM_TNR_MODE_OFF
     *            (1): NoiseReduction_Fast        - GST_NVCAM_TNR_MODE_FAST
     *            (2): NoiseReduction_HighQuality  - GST_NVCAM_TNR_MODE_HIGH_QUALITY
     */
    if ((this -> parameters).tnr_mode != NOISE_REDUCTION_FAST)
        this -> gstreamer_command += "tnr-mode=" + std::to_string((this -> parameters).tnr_mode) + " ";

    /**
     * ee-mode     : Property to adjust edge enhancement mode
     *             flags: readable, writeable
     *             Enum "GstNvArgusCamEEMode" Default: 1, "EdgeEnhancement_Fast"
     *             (0): EdgeEnhancement_Off         - GST_NVCAM_EE_MODE_OFF
     *             (1): EdgeEnhancement_Fast        - GST_NVCAM_EE_MODE_FAST
     *             (2): EdgeEnhancement_HighQuality - GST_NVCAM_EE_MODE_HIGH_QUALITY
     */
    if ((this -> parameters).edge_enhancement_mode != EDGE_ENHANCEMENT_FAST)
        this -> gstreamer_command += "ee-mode=" + std::to_string((this -> parameters).edge_enhancement_mode) + " ";

    /**
     * ee-strength : Property to adjust edge enhancement strength
     *            flags: readable, writeable
     *            Float. Range: 0 - 1 Default: -1
     */
//    if ((this -> parameters).edge_enhancement_strength != -1)
//        this -> gstreamer_command += "ee-strength=" + std::to_string((this -> parameters).edge_enhancement_strength) + " ";

    /**
     * aeantibanding : Property to adjust AE antibanding mode
     *              flags: readable, writeable
     *              Enum "GstNvArgusCamAEAntiBandingMode" Default: 1, "AE_AntiBanding_Mode_Auto"
     *              (0): AE_AntiBanding_Mode_Off   - GST_NVCAM_AE_ANTIBANDING_MODE_OFF
     *              (1): AE_AntiBanding_Mode_Auto  - GST_NVCAM_AE_ANTIBANDING_MODE_AUTO
     *              (2): AE_AntiBanding_Mode_50HZ   - GST_NVCAM_AE_ANTIBANDING_MODE_50HZ
     *              (3): AE_AntiBanding_Mode_60HZ   - GST_NVCAM_AE_ANTIBANDING_MODE_60HZ
     */
    if ((this -> parameters).ae_anti_banding_mode != AE_ANTI_BANDING_MODE_AUTO)
        this -> gstreamer_command += "aeantibanding=" + std::to_string((this -> parameters).ae_anti_banding_mode) + " ";

    /**
     * exposurecompensation : Property to adjust exposure compensation value
     *                      flags: readable, writeable
     *                      Float. Range: -2 - 2 Default: 0
     */
    if ((this -> parameters).exposure_compensation != 0)
        this -> gstreamer_command += "exposurecompensation=" + std::to_string((this -> parameters).exposure_compensation) + " ";

    /**
     * aelock      : Set or unset the auto exposure lock
     *            flags: readable, writeable
     *            Boolean. Default: false
     */
    if ((this -> parameters).ae_lock)
        this -> gstreamer_command += "aelock=" + std::to_string((this -> parameters).ae_lock) + " ";

    /**
     * aeregion    : Property to set region of interest for auto exposure
     *             with values of ROI coordinates (left, top, right, bottom)
     *             and weight (float number) in that order, to set the property.
     *             use for example: aeregion="0 0 256 256 1"
     *             flags: readable, writeable
     *             String. Default: null
     */
    if (
            (this -> parameters).auto_exposure_region.left != 0 &&
            (this -> parameters).auto_exposure_region.top != 0 &&
            (this -> parameters).auto_exposure_region.right != 0 &&
            (this -> parameters).auto_exposure_region.bottom != 0 &&
            (this -> parameters).auto_exposure_region.weight != 0.0
       )
        this -> gstreamer_command += "aeregion=\"" + std::to_string((this -> parameters).auto_exposure_region.left) + " " + std::to_string((this -> parameters).auto_exposure_region.top) + " " + std::to_string((this -> parameters).auto_exposure_region.right) + " " + std::to_string((this -> parameters).auto_exposure_region.bottom) + " " + std::to_string((this -> parameters).auto_exposure_region.weight) + "\" ";

    /**
     * awblock     : Set or unset the auto white balance lock
     *           flags: readable, writeable
     *           Boolean. Default: false
     */
    if ((this -> parameters).awb_lock)
        this -> gstreamer_command += "awblock=" + std::to_string((this -> parameters).awb_lock) + " ";

    /**
     * bufapi-version : Set the version of the buffer api to use
     *               flags: readable, writeable
     *               Boolean. Default: false
     */
    if ((this -> parameters).buf_api_version)
        this -> gstreamer_command += "bufapi-version=" + std::to_string((this -> parameters).buf_api_version) + " ";



    this -> gstreamer_command += "! video/x-raw(memory:NVMM), ";
    this -> gstreamer_command += "width=(int)" + std::to_string((this -> parameters).width) + ", ";
    this -> gstreamer_command += "height=(int)" + std::to_string((this -> parameters).height) + ", ";
    this -> gstreamer_command += "framerate=(fraction)" + std::to_string((this -> parameters).framerate) + "/1 ";
    this -> gstreamer_command += "! nvvidconv flip-method=0 ";
    this -> gstreamer_command += "! video/x-raw, width=(int)" + std::to_string((this -> parameters).width) + ", height=(int)" + std::to_string((this -> parameters).height) + ", format=(string)BGRx ";
    this -> gstreamer_command += "! videoconvert ";
    this -> gstreamer_command += "! video/x-raw, format=(string)BGR ";
    this -> gstreamer_command += "! appsink";

    return gstreamer_command;
}

std::string Gstreamer::set_name(std::string name) {
    if (name.empty()) {
        throw std::invalid_argument("Name cannot be empty");
    }
    (this -> parameters).name = name;
    return name;
}

unsigned int Gstreamer::set_block_size(unsigned int block_size) {
    (this -> parameters).block_size = block_size;
    return block_size;
}

int Gstreamer::set_num_buffers(int num_buffers) {
    if (num_buffers < -1) {
        throw std::invalid_argument("Number of buffers cannot be less than -1");
    }
    (this -> parameters).num_buffers = num_buffers;
    return num_buffers;
}

bool Gstreamer::set_type_find(bool type_find) {
    (this -> parameters).type_find = type_find;
    return type_find;
}

bool Gstreamer::set_do_timestamp(bool do_timestamp) {
    (this -> parameters).do_timestamp = do_timestamp;
    return do_timestamp;
}

bool Gstreamer::set_silent(bool silent) {
    (this -> parameters).silent = silent;
    return silent;
}

int Gstreamer::set_sensor_id(int sensor_id) {
    if (sensor_id < 0 || sensor_id > 255) {
        throw std::invalid_argument("Sensor ID must be between 0 and 255");
    }
    (this -> parameters).sensor_id = sensor_id;
    return sensor_id;
}

int Gstreamer::set_sensor_mode(int sensor_mode) {
    if (sensor_mode < -1 || sensor_mode > 255) {
        throw std::invalid_argument("Sensor mode must be between -1 and 255");
    }
    (this -> parameters).sensor_mode = sensor_mode;
    return sensor_mode;
}

int Gstreamer::set_total_sensor_modes(int total_sensor_modes) {
    if (total_sensor_modes < 0 || total_sensor_modes > 255) {
        throw std::invalid_argument("Total sensor modes must be between 0 and 255");
    }
    (this -> parameters).total_sensor_modes = total_sensor_modes;
    return total_sensor_modes;
}

int Gstreamer::set_min_exposure_timerange(int min_exposure_timerange) {
    if (min_exposure_timerange < 0) {
        throw std::invalid_argument("Minimum exposure time range must be greater than 0");
    }
    (this -> parameters).min_exposure_timerange = min_exposure_timerange;
    return min_exposure_timerange;
}

int Gstreamer::set_max_exposure_timerange(int max_exposure_timerange) {
    if (max_exposure_timerange < 0) {
        throw std::invalid_argument("Maximum exposure time range must be greater than 0");
    }
    (this -> parameters).max_exposure_timerange = max_exposure_timerange;
    return max_exposure_timerange;
}

float Gstreamer::set_min_gain_range(float min_gain_range) {
    if (min_gain_range < 0) {
        throw std::invalid_argument("Minimum gain range must be greater than 0");
    }
    (this -> parameters).min_gain_range = min_gain_range;
    return min_gain_range;
}

float Gstreamer::set_max_gain_range(float max_gain_range) {
    if (max_gain_range < 0) {
        throw std::invalid_argument("Maximum gain range must be greater than 0");
    }
    (this -> parameters).max_gain_range = max_gain_range;
    return max_gain_range;
}

float Gstreamer::set_min_isp_digital_gain_range(float min_isp_digital_gain_range) {
    if (min_isp_digital_gain_range < 0) {
        throw std::invalid_argument("Minimum ISP digital gain range must be greater than 0");
    }
    (this -> parameters).min_isp_digital_gain_range = min_isp_digital_gain_range;
    return min_isp_digital_gain_range;
}

float Gstreamer::set_max_isp_digital_gain_range(float max_isp_digital_gain_range) {
    if (max_isp_digital_gain_range < 0) {
        throw std::invalid_argument("Maximum ISP digital gain range must be greater than 0");
    }
    (this -> parameters).max_isp_digital_gain_range = max_isp_digital_gain_range;
    return max_isp_digital_gain_range;
}

float Gstreamer::set_tnr_strength(float tnr_strength) {
    if (tnr_strength < 0 || tnr_strength > 1) {
        throw std::invalid_argument("Temporal noise reduction strength must be between 0 and 1");
    }
    (this -> parameters).tnr_strength = tnr_strength;
    return tnr_strength;
}

float Gstreamer::set_saturation(float saturation) {
    if (saturation < 0 || saturation > 2) {
        throw std::invalid_argument("Saturation must be between 0 and 2");
    }
    (this -> parameters).saturation = saturation;
    return saturation;
}

float Gstreamer::set_exposure_compensation(float exposure_compensation) {
    if (exposure_compensation < -2 || exposure_compensation > 2) {
        throw std::invalid_argument("Exposure compensation must be between -2 and 2");
    }
    (this -> parameters).exposure_compensation = exposure_compensation;
    return exposure_compensation;
}

bool Gstreamer::set_ae_lock(bool ae_lock) {
    (this -> parameters).ae_lock = ae_lock;
    return ae_lock;
}

bool Gstreamer::set_awb_lock(bool awb_lock) {
    (this -> parameters).awb_lock = awb_lock;
    return awb_lock;
}

bool Gstreamer::set_buf_api_version(bool buf_api_version) {
    (this -> parameters).buf_api_version = buf_api_version;
    return buf_api_version;
}

Gstreamer::ROI_Coordinates Gstreamer::set_auto_exposure_region(Gstreamer::ROI_Coordinates auto_exposure_region) {
    if (
            auto_exposure_region.left < 0 ||
            auto_exposure_region.top < 0 ||
            auto_exposure_region.right < 0 ||
            auto_exposure_region.bottom < 0 ||
            auto_exposure_region.weight < 0.0
            ) {
        throw std::invalid_argument("Auto exposure region values must be greater than 0");
    }
    if (auto_exposure_region.left > auto_exposure_region.right) {
        throw std::invalid_argument("Left value must be less than right value");
    }
    if (auto_exposure_region.top > auto_exposure_region.bottom) {
        throw std::invalid_argument("Top value must be less than bottom value");
    }
    if (auto_exposure_region.weight > 1.0) {
        throw std::invalid_argument("Weight value must be less than 1.0");
    }

    (this -> parameters).auto_exposure_region = auto_exposure_region;
    return auto_exposure_region;
}

Gstreamer::WhiteBalance Gstreamer::set_white_balance(Gstreamer::WhiteBalance white_balance) {
    (this -> parameters).white_balance = white_balance;
    return white_balance;
}

Gstreamer::TnrMode Gstreamer::set_tnr_mode(Gstreamer::TnrMode tnr_mode) {
    (this -> parameters).tnr_mode = tnr_mode;
    return tnr_mode;
}

Gstreamer::EdgeEnhancementMode Gstreamer::set_edge_enhancement_mode(Gstreamer::EdgeEnhancementMode edge_enhancement_mode) {
    (this -> parameters).edge_enhancement_mode = edge_enhancement_mode;
    return edge_enhancement_mode;
}

Gstreamer::AeAntiBandingMode Gstreamer::set_ae_anti_banding_mode(Gstreamer::AeAntiBandingMode ae_anti_banding_mode) {
    (this -> parameters).ae_anti_banding_mode = ae_anti_banding_mode;
    return ae_anti_banding_mode;
}

int Gstreamer::set_width(int width) {
    if (width < 0) {
        throw std::invalid_argument("Width must be greater than 0");
    }
    (this -> parameters).width = width;
    return width;
}

int Gstreamer::set_height(int height) {
    if (height < 0) {
        throw std::invalid_argument("Height must be greater than 0");
    }
    (this -> parameters).height = height;
    return height;
}

float Gstreamer::set_framerate(float framerate) {
    if (framerate < 0) {
        throw std::invalid_argument("Framerate must be greater than 0");
    }
    (this -> parameters).framerate = framerate;
    return framerate;
}