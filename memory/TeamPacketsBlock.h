#ifndef TEAMPACKETSBLOCK_
#define TEAMPACKETSBLOCK_

#include <common/TeamPacket.h>
#include "MemoryBlock.h"
#include <common/WorldObject.h>

// to be filled in by incoming team packets
struct TeamPacketsBlock : public MemoryBlock {
public:
  TeamPacketsBlock()  {
    header.version = 11;
    header.size = sizeof(TeamPacketsBlock);

    // default values
    for (int i = 0; i <= WO_TEAM_LAST; i++){
      frameReceived[i] = -10000;
      ballUpdated[i] = false;
      oppUpdated[i] = false;
      tp[i].robotIP = -1;
      tp[i].robotNumber = 0;
    }
    sinceLastTeamPacketIn=100;
  }

  TeamPacket getPkt(unsigned int pkt) {
    return tp[pkt];
  }
  
  TeamPacket* getPktPtr(unsigned int pkt) {
    return &(tp[pkt]);
  }

  int getFrameReceived(unsigned int pkt) {
    return frameReceived[pkt];
  }

  int getMissedFromUs(unsigned int mate, unsigned int me){
    return tp[mate].packetsMissed[me];
  }

  // Array of 6, so we can index from robot ID's 1-5. Index 0 not used.
  TeamPacket tp[WO_TEAM_LAST+1];

  int frameReceived[WO_TEAM_LAST+1];
  bool ballUpdated[WO_TEAM_LAST+1];
  bool oppUpdated[WO_TEAM_LAST+1];

  int sinceLastTeamPacketIn;
};

#endif 
