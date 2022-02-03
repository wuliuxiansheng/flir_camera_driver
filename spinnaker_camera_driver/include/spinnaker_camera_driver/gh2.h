/**
 * \file
 *
 * Grasshopper2 camera driver
 *
 * \author Chao Liu
 */

#ifndef SPINNAKER_CAMERA_DRIVER_GH2_H
#define SPINNAKER_CAMERA_DRIVER_GH2_H

#include "spinnaker_camera_driver/camera.h"
#include <SpinGenApi/INodeMap.h>
#include <spinnaker_camera_driver/SpinnakerConfig.h>
#include <sys/types.h>

namespace spinnaker_camera_driver
{

class Gh2 : public Camera
{
  public:
    explicit Gh2(Spinnaker::GenApi::INodeMap* node_map);
    ~Gh2();
    void setFrameRate(const float frame_rate);
    void setNewConfiguration(const SpinnakerConfig& config, const uint32_t& level);

  private:
    void setImageControlFormats(const spinnaker_camera_driver::SpinnakerConfig& config);
};

} // namespace

#endif // include guard
