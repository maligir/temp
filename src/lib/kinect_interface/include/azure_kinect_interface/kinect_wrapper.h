#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "do_once.h"
#include "kinect_packet.h"
#include "frame_recipient.h"

#include <k4a/k4a.hpp>

class KinectFrameRecipient;
class KFRRecorder;
class KFRRecorderFactory;

class KinectWrapper : public DoOnce {
public:
    KinectWrapper(uint8_t deviceIndex = 0);
    ~KinectWrapper();

    KFRRecorder *getKFRRecorder(KFRRecorderFactory &kfrrf);

    void start();
    void doOnce();

    void setFrameRecipient(FrameRecipient *fr) {
        _fr = fr;
    }

    FrameRecipient* getLastFR() {
        if (_fr) {
            return _fr->getLastFR();
        } else {
            return nullptr;
        }
    }

    k4a::calibration& GetCalibration();

    KinectPacket _pkt;

protected:
    FrameRecipient* _fr;
    k4a::device _device;
    k4a_device_configuration_t _config;
    k4a::calibration _sensor_calibration;
    //  JWH Will eventually need these, which can be obtained from k4a::calibration (and need not be pulled out, since we have the calibration object)
    //  k4a_calibration_extrinsics_t
    //  k4a_calibration_intrinsics_t
    //  Azure Kinect devices are calibrated with Brown Conrady which is compatible with OpenCV.
    //  From: https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/unionk4a__calibration__intrinsic__parameters__t.html
};

#endif