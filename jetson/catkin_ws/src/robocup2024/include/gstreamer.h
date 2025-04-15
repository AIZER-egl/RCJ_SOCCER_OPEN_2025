#ifndef ROBOCUP2024_GSTREAMER_H
#define ROBOCUP2024_GSTREAMER_H

#include <string>
#include <stdexcept>

class Gstreamer {
public:
    typedef enum {
        OFF = 0,
        AUTO = 1,
        INCANDESCENT = 2,
        FLUORESCENT = 3,
        WARM_FLUORESCENT = 4,
        DAYLIGHT = 5,
        CLOUDY_DAYLIGHT = 6,
        TWILIGHT = 7,
        SHADE = 8,
        MANUAL = 9
    } WhiteBalance;

    typedef enum {
        NOISE_REDUCTION_OFF = 0,
        NOISE_REDUCTION_FAST = 1,
        NOISE_REDUCTION_HIGH_QUALITY = 2
    } TnrMode;

    typedef enum {
        EDGE_ENHANCEMENT_OFF = 0,
        EDGE_ENHANCEMENT_FAST = 1,
        EDGE_ENHANCEMENT_HIGH_QUALITY = 2
    } EdgeEnhancementMode;

    typedef enum {
        AE_ANTI_BANDING_MODE_OFF = 0,
        AE_ANTI_BANDING_MODE_AUTO = 1,
        AE_ANTI_BANDING_MODE_50HZ = 2,
        AE_ANTI_BANDING_MODE_60HZ = 3
    } AeAntiBandingMode;

    struct ROI_Coordinates {
        int left;
        int top;
        int right;
        int bottom;
        float weight;
    };
private:
    std::string gstreamer_command;

    struct GStreamerParameters {
        std::string name = "nvarguscamerasrc0";
        unsigned int block_size = 4096;
        int num_buffers = -1;
        bool type_find = false;
        bool do_timestamp = true;
        bool silent = true;
        int timeout = 0;

        int sensor_id = 0;
        int sensor_mode = -1;
        int total_sensor_modes = 0;
        int min_exposure_timerange = 0;
        int max_exposure_timerange = 0;
        int min_gain_range = 0;
        int max_gain_range = 0;
        int min_isp_digital_gain_range = 0;
        int max_isp_digital_gain_range = 0;

        float tnr_strength = -1.0;
        float saturation = 1.0;
        float exposure_compensation = 0.0;

        bool ae_lock = false;
        bool awb_lock = false;
        bool buf_api_version = false;

        ROI_Coordinates auto_exposure_region = {0, 0, 0, 0, 0.0};

        WhiteBalance white_balance = AUTO;
        TnrMode tnr_mode = NOISE_REDUCTION_FAST;
        EdgeEnhancementMode edge_enhancement_mode = EDGE_ENHANCEMENT_FAST;
        AeAntiBandingMode ae_anti_banding_mode = AE_ANTI_BANDING_MODE_AUTO;

        int width = 1280;
        int height = 720;
        float framerate = 30.0;
    };

    GStreamerParameters parameters;
public:

    Gstreamer();

    std::string get_command();

    std::string set_name(std::string name);

    unsigned int set_block_size(unsigned int block_size);
    unsigned int set_timeout(unsigned int timeout);
    int set_num_buffers(int num_buffers);
    bool set_type_find(bool type_find);
    bool set_do_timestamp(bool do_timestamp);
    bool set_silent(bool silent);

    int set_sensor_id (int sensor_id);
    int set_sensor_mode (int sensor_mode);
    int set_total_sensor_modes (int total_sensor_modes);
    int set_min_exposure_timerange (int min_exposure_timerange);
    int set_max_exposure_timerange (int max_exposure_timerange);
    float set_min_gain_range (float min_gain_range);
    float set_max_gain_range (float max_gain_range);
    float set_min_isp_digital_gain_range (float min_isp_digital_gain_range);
    float set_max_isp_digital_gain_range (float max_isp_digital_gain_range);

    float set_tnr_strength (float tnr_strength);
    float set_saturation(float saturation);
    float set_exposure_compensation(float exposure_compensation);

    bool set_ae_lock(bool ae_lock);
    bool set_awb_lock(bool awb_lock);
    bool set_buf_api_version(bool buf_api_version);

    ROI_Coordinates set_auto_exposure_region(ROI_Coordinates auto_exposure_region);

    WhiteBalance set_white_balance(WhiteBalance white_balance);
    TnrMode set_tnr_mode(TnrMode tnr_mode);
    EdgeEnhancementMode set_edge_enhancement_mode(EdgeEnhancementMode edge_enhancement_mode);
    AeAntiBandingMode set_ae_anti_banding_mode(AeAntiBandingMode ae_anti_banding_mode);

    int set_width(int width);
    int set_height(int height);
    float set_framerate(float framerate);
};


#endif //ROBOCUP2024_GSTREAMER_H
