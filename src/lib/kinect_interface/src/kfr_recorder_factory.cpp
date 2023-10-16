#include "kfr_recorder_factory.h"
#include "kfr_recorder.h"

KFRRecorderFactory::KFRRecorderFactory(const char *path) : _path(path) {}
KFRRecorderFactory::~KFRRecorderFactory() {}

KFRRecorder *KFRRecorderFactory::getRecorder(
    const k4a::device &device,
    const k4a_device_configuration_t &device_configuration) {
    return new KFRRecorder(_path, device, device_configuration);
}