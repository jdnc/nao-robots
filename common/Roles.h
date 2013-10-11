#ifndef ROLES_H
#define ROLES_H

#include <string>

enum Role {
  UNDEFINED = 0,
  KEEPER = 1,
  DEFENDER = 2,
  FORWARD = 3,
  SUPPORTER = 4,
  CHASER = 5,
  MIDFIELD = 6,
  CAUTIOUS_DEFENDER = 7,
  SET_PLAY_RECEIVER = 8,
  NONE = 9,
  NUM_ROLES = NONE
};


const std::string roleNames[] = {
  "undefined",
  "KEEPER",
  "DEFENDER",
  "FORWARD",
  "SUPPORTER",
  "CHASER",
  "MIDFIELD",
  "CAUTIOUS_DEFENDER",
  "SET_PLAY_RECEIVER",
  "none"
};


const std::string roleAbbrevs[] = {
  "undefined",
  "K",
  "D",
  "F",
  "S",
  "C",
  "M",
  "CD",
  "R",
  "none"
};

#endif
