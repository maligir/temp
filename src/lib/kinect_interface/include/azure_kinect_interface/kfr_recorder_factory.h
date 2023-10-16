#ifndef KFR_RECORDER_FACTORY_H
#define KFR_RECORDER_FACTORY_H

#include "kfr_recorder.h"

#include <k4arecord/record.hpp>

class KFRRecorderFactory {
public:
    KFRRecorderFactory(const char *path);
    ~KFRRecorderFactory();

    KFRRecorder *getRecorder(
        const k4a::device &device,
        const k4a_device_configuration_t &device_configuration);

protected:
    const char *_path;
};

#endif