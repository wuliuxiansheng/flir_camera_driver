/**
 * \file
 *
 * Grasshopper2 camera driver
 *
 * \author Chao Liu
 */

#include "spinnaker_camera_driver/gh2.h"
#include "spinnaker_camera_driver/set_property.h"

namespace spinnaker_camera_driver {
Gh2::Gh2(Spinnaker::GenApi::INodeMap *node_map) : Camera(node_map) {}

Gh2::~Gh2() {}

void Gh2::setFrameRate(const float frame_rate) {
    setProperty(node_map_, "AcquisitionFrameRateEnabled", true);
    setProperty(node_map_, "AcquisitionFrameRateAuto",
                static_cast< std::string >("Off"));
    Spinnaker::GenApi::CFloatPtr ptrAcquisitionFrameRate =
        node_map_->GetNode("AcquisitionFrameRate");
    ROS_DEBUG_STREAM("Minimum Frame Rate: \t "
                     << ptrAcquisitionFrameRate->GetMin());
    ROS_DEBUG_STREAM("Maximum Frame rate: \t "
                     << ptrAcquisitionFrameRate->GetMax());

    // Finally Set the Frame Rate
    setProperty(node_map_, "AcquisitionFrameRate", frame_rate);

    ROS_DEBUG_STREAM("Current Frame rate: \t "
                     << ptrAcquisitionFrameRate->GetValue());
}

void Gh2::setNewConfiguration(const SpinnakerConfig &config,
                              const uint32_t &level) {
    try {
        if (level >= LEVEL_RECONFIGURE_STOP) {
            setImageControlFormats(config);
            setFrameRate(static_cast< float >(config.acquisition_frame_rate));
            setProperty(node_map_, "AcquisitionFrameRateEnabled",
                        config.acquisition_frame_rate_enable);

            setProperty(node_map_, "TriggerMode", std::string("Off"));
            setProperty(node_map_, "TriggerSource", config.trigger_source);
            setProperty(node_map_, "TriggerActivation",
                        config.trigger_activation_mode);
            setProperty(node_map_, "TriggerMode", config.enable_trigger);

            setProperty(node_map_, "LineSelector", config.line_selector);
            setProperty(node_map_, "LineMode", config.line_mode);

            // Set auto exposure
            setProperty(node_map_, "ExposureMode", config.exposure_mode);
            setProperty(node_map_, "ExposureAuto", config.exposure_auto);
            if (config.exposure_auto.compare(std::string("Off")) == 0) {
                setProperty(node_map_, "ExposureTime",
                            static_cast< float >(config.exposure_time));
            }

            // Set gain
            setProperty(node_map_, "GainAuto", config.auto_gain);
            if (config.auto_gain.compare(std::string("Off")) == 0) {
                setProperty(node_map_, "Gain",
                            static_cast< float >(config.gain));
            }

            // Set brightness
            setProperty(node_map_, "BlackLevel",
                        static_cast< float >(config.brightness));

            // Set gamma
            if (config.gamma_enable) {
                setProperty(node_map_, "GammaEnabled",
                            config.gamma_enable); // GH3 includes -ed
                setProperty(node_map_, "Gamma",
                            static_cast< float >(config.gamma));
            }

            // Set white balance
            if (IsAvailable(node_map_->GetNode("BalanceWhiteAuto"))) {
                setProperty(node_map_, "BalanceWhiteAuto",
                            config.auto_white_balance);
                if (config.auto_white_balance.compare(std::string("Off")) ==
                    0) {
                    setProperty(node_map_, "BalanceRatioSelector", "Blue");
                    setProperty(
                        node_map_, "BalanceRatio",
                        static_cast< float >(config.white_balance_blue_ratio));
                    setProperty(node_map_, "BalanceRatioSelector", "Red");
                    setProperty(
                        node_map_, "BalanceRatio",
                        static_cast< float >(config.white_balance_red_ratio));
                }
            }
        }
    } catch (const Spinnaker::Exception &e) {
        throw std::runtime_error(
            "[Gh2::setNewConfiguration] Failed to set configuration: " +
            std::string(e.what()));
    }
}

// Image Size and Pixel Format
void Gh2::setImageControlFormats(
    const spinnaker_camera_driver::SpinnakerConfig &config) {
    // Set Binning and Decimation
    setProperty(node_map_, "BinningVertical", config.image_format_y_binning);
    // Grab the Max values after decimation
    Spinnaker::GenApi::CIntegerPtr height_max_ptr =
        node_map_->GetNode("HeightMax");
    if (!IsAvailable(height_max_ptr) || !IsReadable(height_max_ptr)) {
        throw std::runtime_error(
            "[Gh2::setImageControlFormats] Unable to read HeightMax");
    }
    height_max_ = height_max_ptr->GetValue();
    Spinnaker::GenApi::CIntegerPtr width_max_ptr =
        node_map_->GetNode("WidthMax");
    if (!IsAvailable(width_max_ptr) || !IsReadable(width_max_ptr)) {
        throw std::runtime_error(
            "[Gh2::setImageControlFormats] Unable to read WidthMax");
    }
    width_max_ = width_max_ptr->GetValue();

    // Offset first encase expanding ROI
    // Apply offset X
    setProperty(node_map_, "OffsetX", 0);
    // Apply offset Y
    setProperty(node_map_, "OffsetY", 0);

    // Set Width/Height
    if (config.image_format_roi_width <= 0 ||
        config.image_format_roi_width > width_max_)
        setProperty(node_map_, "Width", width_max_);
    else
        setProperty(node_map_, "Width", config.image_format_roi_width);
    if (config.image_format_roi_height <= 0 ||
        config.image_format_roi_height > height_max_)
        setProperty(node_map_, "Height", height_max_);
    else
        setProperty(node_map_, "Height", config.image_format_roi_height);

    // Apply offset X
    setProperty(node_map_, "OffsetX", config.image_format_x_offset);
    // Apply offset Y
    setProperty(node_map_, "OffsetY", config.image_format_y_offset);

    // Set Pixel Format
    setProperty(node_map_, "PixelFormat", config.image_format_color_coding);
}

} // namespace spinnaker_camera_driver
