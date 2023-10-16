#ifndef FR_APRIL_TAG_H
#define FR_APRIL_TAG_H

#include "frame_recipient.h"
#include "april_tag_manager.h"
#include <apriltag/apriltag.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

class FRAprilTag : public FrameRecipient {
public:
    FRAprilTag(AprilTagManager* manager,
                const std::string& _rgb_cam,
                const std::string& _depth_cam);
    void initializePacket(DataPacket *dp);
    void receiveFrame(DataPacket *dp);

protected:
    void detectCorners(cv::Mat img, DataPacket *dp); 
    
    apriltag_detector_t *td;
    apriltag_family_t *tf;

    AprilTagManager* _manager;
    size_t _frame_count;
    const std::string _rgb_cam;
    const std::string _depth_cam;
    const bool _use_depth;
        
};

#endif