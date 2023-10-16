#include "fr_save_packet.h"
#include "data_packet.h"

FRSavePacket::FRSavePacket(std::vector<DataPacket*>& packets): _packets(packets) {}

FRSavePacket::~FRSavePacket() {}

void FRSavePacket::initializePacket(DataPacket *dp) {}

void FRSavePacket::receiveFrame(DataPacket *dp) {
    _packets.push_back(dp);
}