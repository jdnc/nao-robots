#ifndef WORLDOJBECTBLOCK_
#define WORLDOJBECTBLOCK_

#include <common/WorldObject.h>
#include <common/States.h>
#include <common/Field.h>

#include "MemoryBlock.h"

struct WorldObjectBlock : public MemoryBlock {
 public:
  WorldObjectBlock()  {
    header.version = 3;
    header.size = sizeof(WorldObjectBlock);
  }

  void init(int team) {
    std::cout << "Initialising Worldobjects for Team " << team << std::endl;
    /** Initialises the real world location of all the objects in
     * the world.
     */
    // Set the type for each object
    for (int i = 0; i < NUM_WORLD_OBJS; i++) {
      objects_[i].type=i;
      objects_[i].reset();
    }

    // set landmark locations
    for (int i = 0; i < NUM_LANDMARKS; i++) {
      WorldObject *wo = &(objects_[i + LANDMARK_OFFSET]);
      wo->loc = landmarkLocation[i];

      // set heights
      if (wo->isGoal()){
        wo->upperHeight = GOAL_HEIGHT;
        wo->lowerHeight = 0;
        wo->elevation = (wo->upperHeight + wo->lowerHeight) / 2;
      } else {
        wo->upperHeight = 0;
        wo->lowerHeight = 0;
        wo->elevation = 0;
      }
    }


    // set intersection locations
    for (int i = 0; i < NUM_INTERSECTIONS; i++){
      WorldObject *wo = &(objects_[i + INTERSECTION_OFFSET]);
      wo->loc = intersectionLocation[i];

      // set heights
      wo->upperHeight = 0;
      wo->lowerHeight = 0;
      wo->elevation = 0;
    }

    // set line locations
    for (int i = 0; i < NUM_LINES; i++){
      WorldObject *wo = &(objects_[i + LINE_OFFSET]);
      wo->loc = lineLocationStarts[i];
      wo->endLoc = lineLocationEnds[i];
      wo->lineLoc = Line2D(wo->loc, wo->endLoc);

      // set heights
      wo->upperHeight = 0;
      wo->lowerHeight = 0;
      wo->elevation = 0;


      //cout << wo->lineLoc << endl << flush;
    }

    // set penalty cross locations
    for (int i = 0; i < NUM_CROSSES; i++){
      WorldObject* wo = &(objects_[i + CROSS_OFFSET]);
      Point2D loc = oppCrossLocation;
      if ((i+CROSS_OFFSET) == WO_OWN_PENALTY_CROSS)
        loc = ownCrossLocation;

      wo->loc = loc;
      wo->upperHeight = 0;
      wo->lowerHeight = 0;
      wo->elevation = 0;
    }

  }

  /** Resets all world objects. This mainly sets them to be not seen.*/
  void reset() {
    for (int i = 0; i < NUM_WORLD_OBJS; i++) {
      objects_[i].reset();
    }
  }

  WorldObject getObj(unsigned int ind) {
    return objects_[ind];
  }

  WorldObject* getObjPtr(unsigned int ind) {
    return &(objects_[ind]);
  }

  WorldObject objects_[NUM_WORLD_OBJS];

};



#endif
