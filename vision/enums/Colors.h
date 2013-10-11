#ifndef COLORS_H
#define COLORS_H

enum Color {
  c_UNDEFINED     = 0,
  c_FIELD_GREEN   = 1,
  c_WHITE         = 2,
  c_ORANGE        = 3,
  c_PINK          = 4,
  c_BLUE          = 5,
  c_YELLOW        = 6,
  c_ROBOT_WHITE   = 7,
  NUM_COLORS      = 8
};
#define FLAG_GREEN (1 << c_GREEN)
#define FLAG_WHITE (1 << c_WHITE)
#define FLAG_ORANGE (1 << c_ORANGE)
#define FLAG_PINK (1 << c_PINK)
#define FLAG_BLUE (1 << c_BLUE)
#define FLAG_YELLOW (1 << c_YELLOW)

#define COLOR_NAME(c) ( \
    c == 0 ? "UNDEFINED" \
    : c == 1 ? "GREEN" \
    : c == 2 ? "WHITE" \
    : c == 3 ? "ORANGE" \
    : c == 4 ? "PINK" \
    : c == 5 ? "BLUE" \
    : c == 6 ? "YELLOW" \
    : c == 7 ? "ROBOT WHITE" \
    : "INVALID")

#define isInFlags(c,flags) (flags & (1 << c))
#endif
