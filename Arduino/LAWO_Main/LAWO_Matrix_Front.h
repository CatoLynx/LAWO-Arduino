#include <Arduino.h>

#define MATRIX_WIDTH 126
#define MATRIX_HEIGHT 16
#define PANEL_WIDTH 28

byte PANEL_LINES[5] = {1, 2, 3, 0, 4};

bool hasHalfPanelOffset(byte colIndex) {
  /*
   * Determine if the given column index belongs to a panel which was preceded by a 14-col half panel.
   */

  // In the 126x16 matrix, the last panel (index 4; first one from the end) is a short one
  return (colIndex / PANEL_WIDTH < 4);
}
