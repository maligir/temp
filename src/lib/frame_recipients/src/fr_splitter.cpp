#include "fr_splitter.h"

using namespace std;

FRSplitter::FRSplitter() {}
FRSplitter::~FRSplitter() {}

void FRSplitter::initializePacket(DataPacket *dp) {
    for(vector<FrameRecipient *>::iterator it = _fr.begin(); it != _fr.end(); it++) {
        (*it)->initializePacket(dp);
    }
}

void FRSplitter::receiveFrame(DataPacket *dp) {
    for(vector<FrameRecipient *>::iterator it = _fr.begin(); it != _fr.end(); it++) {
        (*it)->receiveFrame(dp);
    }
}

void FRSplitter::addFrameRecipient(FrameRecipient *fr) {
    _fr.push_back(fr);
}
