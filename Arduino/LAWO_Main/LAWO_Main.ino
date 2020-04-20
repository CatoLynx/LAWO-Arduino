// Copyright (C) 2016 Julian Metzler

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// COMPILE WITH RX BUFFER SIZE 1024

/*
 * MATRIX-SPECIFIC INCLUDE
 *
 * Since different matrix configurations have different properties, we put them in different external files
 */

#include <avr/wdt.h>
#include "LAWO_Matrix_Panel16.h"

/*
 * PIN DECLARATIONS
 */

#define ROW_A0 6
#define ROW_A1 7
#define ROW_A2 5
#define ROW_A3 4
#define COL_A0 A4
#define COL_A1 A3
#define COL_A2 A2
#define COL_A3 A0
#define COL_A4 13
#define ROWS_BOTTOM 3
#define ROWS_TOP 2
#define D A1
#define E1 11
#define E2 8
#define E3 9
#define E4 10
#define E5 12
#define LED A5

/*
 * GLOBAL CONSTANTS
 */

#define BAUDRATE 57600
#define SERIAL_TIMEOUT 5000
#define FLIP_DURATION 500 // in microseconds
#define FLIP_PAUSE_DURATION 250 // in microseconds

// The associations between address and actually selected matrix row are not 1:1, so we need to use a lookup table
// The values in this array represent the address needed to drive the row corresponding to the array index
byte ROW_TABLE_BLACK[8] = {10, 11, 8, 9, 14, 15, 12, 13}; // Black = lowside
byte ROW_TABLE_YELLOW[8] = {7, 6, 5, 4, 3, 1, 2, 0}; // Yellow = highside
byte E_LINES[5] = {E1, E2, E3, E4, E5};
unsigned int matrixData[MATRIX_WIDTH] = {0};

bool matrixClean = false;
bool pixelInverting = false;
bool activeState = true;
bool quickUpdate = true;

enum SERIAL_STATUSES {
  SUCCESS = 0xFF,
  TIMEOUT = 0xE0,
  ERROR = 0xEE,
};

/*
 * PROGRAM CODE
 */

void enterProgrammingMode() {
  /*
   * Use the watchdog timer to reset the Arduino as if the reset key had been pressed.
   * The Arduino will only recover from programming mode after the power has been cycled, whether a sketch has been uploaded or not.
   */

  wdt_enable(WDTO_15MS);
  for (;;);
}

void selectColumn(byte colIndex) {
  /*
   * Select the appropriate panel for the specified column index and set the column address pins accordingly.
   */

  // In the case of a matrix with a 14-col panel at the end instead of a 28-col one, we need to remember that our panel index is off by half a panel, so flip the MSB
  bool halfPanelOffset = hasHalfPanelOffset(colIndex);

  // Additionally, the address needs to be reversed because of how the panels are connected
  colIndex = MATRIX_WIDTH - colIndex - 1;

  // Since addresses start from the beginning in every panel, we need to wrap around after reaching the end of a panel
  byte address = colIndex % PANEL_WIDTH;

  // A quirk of the FP2800 chip used to drive the columns is that addresses divisible by 8 are not used, so we need to skip those
  address += (address / 7) + 1;

  digitalWrite(COL_A0, address & 1);
  digitalWrite(COL_A1, address & 2);
  digitalWrite(COL_A2, address & 4);
  digitalWrite(COL_A3, address & 8);
  digitalWrite(COL_A4, halfPanelOffset ? !(address & 16) : address & 16);
}

void selectRow(byte rowIndex, bool yellow) {
  /*
   * Select the specified row by writing its address to the line decoders and selecting the appropriate line decoder.
   * This function uses the row lookup tables to determine the address for the desired row number.
   * The yellow parameter tells the function which lookup table to use.
   */

  // Determine and output the address
  byte address = yellow ? ROW_TABLE_YELLOW[rowIndex % 8] : ROW_TABLE_BLACK[rowIndex % 8];
  digitalWrite(ROW_A0, address & 1);
  digitalWrite(ROW_A1, address & 2);
  digitalWrite(ROW_A2, address & 4);
  digitalWrite(ROW_A3, address & 8);

  // Determine which line decoder to use
  if (rowIndex <= 7) {
    // Use the line decoder for the top 8 rows
    digitalWrite(ROWS_BOTTOM, HIGH);
    digitalWrite(ROWS_TOP, LOW);
  } else {
    // Use the line decoder for the bottom 8 rows
    digitalWrite(ROWS_TOP, HIGH);
    digitalWrite(ROWS_BOTTOM, LOW);
  }

  // Set the D line of the FP2800 chip according to the yellow parameter (highside for black, lowside for yellow)
  digitalWrite(D, !yellow);
}

void deselect() {
  /*
   * Deselect all rows by inhibiting the line decoders
   * and deselect the column by setting col address 0
   */

  digitalWrite(ROWS_BOTTOM, HIGH);
  digitalWrite(ROWS_TOP, HIGH);

  digitalWrite(ROW_A0, LOW);
  digitalWrite(ROW_A1, LOW);
  digitalWrite(ROW_A2, LOW);
  digitalWrite(ROW_A3, LOW);

  digitalWrite(COL_A0, LOW);
  digitalWrite(COL_A1, LOW);
  digitalWrite(COL_A2, LOW);
  digitalWrite(COL_A3, LOW);
  digitalWrite(COL_A4, LOW);
}

void flip(byte panelIndex) {
  /*
   * Send an impulse to the specified panel to flip the currently selected dot.
   */

  // Get the enable line for the specified panel
  byte e = E_LINES[PANEL_LINES[panelIndex]];

  digitalWrite(e, HIGH);
  delayMicroseconds(FLIP_DURATION);
  digitalWrite(e, LOW);
  delayMicroseconds(FLIP_PAUSE_DURATION);
}

void setBacklight(bool status) {
  /*
   * Enable or disable the LED backlight of the matrix.
   */

  digitalWrite(LED, !status);
}

void setMatrix(unsigned int* newBitmap, unsigned int* oldBitmap) {
  /*
   * Write a bitmap to the matrix.
   */

  for (int col = 0; col < MATRIX_WIDTH; col++) {
    unsigned int newColData = newBitmap[col];
    unsigned int oldColData;
    bool colChanged;

    // Determine whether the current column has been changed
    if (quickUpdate && oldBitmap) {
      // We're in delta mode, compare the two bitmaps and refresh only the pixels that have changed
      oldColData = oldBitmap[col];
      colChanged = newColData != oldColData;
    } else {
      // We don't have anything to compare or Quick Update is disabled, so just do a full refresh
      oldColData = 0x0000;
      colChanged = true;
    }
    if (!colChanged) continue;

    // Which panel are we on?
    byte panel = col / PANEL_WIDTH;

    selectColumn(col);
    for (int row = 0; row < MATRIX_HEIGHT; row++) {
      // Determine whether the current pixel has been changed
      bool pixelChanged = !quickUpdate || !oldBitmap || (oldColData & (1 << row)) != (newColData & (1 << row));
      if (!pixelChanged) continue;
      byte newPixelValue = !!(newColData & (1 << row));
      newPixelValue ^= pixelInverting;
      selectRow(MATRIX_HEIGHT - row - 1, newPixelValue);
      flip(panel);
    }
  }
  deselect();
  if (!oldBitmap) matrixClean = true;
}

void receiveBitmap() {
  // Receive number of columns
  byte numBytes;
  if (!readBytesOrTimeoutError(&numBytes, 1)) return;
  // Receive bitmap data
  byte serBuf[numBytes];
  unsigned int newBitmap[numBytes / 2];
  if (!readBytesOrTimeoutError(serBuf, numBytes)) return;
  // Piece the bytes together
  for (int i = 0; i < numBytes; i += 2) {
    newBitmap[i / 2] = (serBuf[i] << 8) + serBuf[i + 1];
  }
  // Write the bitmap to the matrix
  if (activeState) setMatrix(newBitmap, matrixClean ? matrixData : NULL);
  memcpy(matrixData, newBitmap, MATRIX_WIDTH * 2);
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void serialResponse(byte status) {
  Serial.write(status);
}

bool readBytesOrTimeout(byte* buffer, int length) {
  int startTime = millis();
  for (int n = 0; n < length; n++) {
    while (!Serial.available()) {
      int timeTaken = millis() - startTime;
      if (timeTaken >= SERIAL_TIMEOUT || timeTaken < 0) { // Second test in case millis rolled over
        return false;
      }
    }
    buffer[n] = Serial.read();
  }
  return true;
}

bool readBytesOrTimeoutError(byte* buffer, int length) {
  bool success = readBytesOrTimeout(buffer, length);
  if (!success) {
    serialResponse(TIMEOUT);
  }
  return success;
}

void doSerialCommunication() {
  /*
   * SERIAL PROTOCOL
   *
   * Explanation:
   *   0x00> - Byte from PC to Arduino
   *  <0x00  - Byte from Arduino to PC
   *
   * Status codes:
   *   <0xFF  - Success
   *   <0xE0  - Timeout while receiving serial data
   *   <0xEE  - Generic Error
   *
   * 0xFF> - Start Byte
   * 0xAn> - Action byte:
   *   0xA0> - Send bitmap:
   *     byte> - Number of bytes to be sent (each column being two bytes)
   *     data> - Bitmap data, two consecutive representing a column
   *  <byte  - Status code
   *   0xA1> - Set LED backlight
   *     byte> - Backlight state (OFF=0x00, ON=0x01)
   *  <byte - Status code
   *   0xA2> - Set pixel inverting
   *     byte> - Inverting state (OFF=0x00, ON=0x01)
   *  <byte - Status code
   *   0xA3> - Set matrix active (If inactive, won't respond to matrix changes until active again)
   *     byte> - Active state (OFF=0x00, ON=0x01)
   *  <byte - Status code
   *   0xA4> - Set quick update (If active, only pixels that have changed will be flipped)
   *     byte> - Quick Update state (OFF=0x00, ON=0x01)
   *  <byte - Status code
   *
   *   0xAF> - Enter programming mode
   *  <byte - Confirmation (always 0xFF)
   */

  if (!Serial.available()) return;
  // Check for start byte
  byte startByte;
  if (!readBytesOrTimeout(&startByte, 1)) return;
  if (startByte != 0xFF) return;

  // Check action byte
  byte actionByte;
  if (!readBytesOrTimeoutError(&actionByte, 1)) return;
  switch (actionByte) {
    // Send bitmap
    case 0xA0: {
        receiveBitmap();
        clearSerialBuffer();
        serialResponse(SUCCESS);
        break;
      }

    // Set LED backlight
    case 0xA1: {
        byte valueByte;
        if (!readBytesOrTimeoutError(&valueByte, 1)) return;
        setBacklight(!!valueByte);
        clearSerialBuffer();
        serialResponse(SUCCESS);
        break;
      }

    // Set pixel inverting
    case 0xA2: {
        byte valueByte;
        if (!readBytesOrTimeoutError(&valueByte, 1)) return;
        matrixClean = pixelInverting == !!valueByte; // Only set the matrix dirty flag when the value has actually changed
        pixelInverting = !!valueByte;
        clearSerialBuffer();
        serialResponse(SUCCESS);
        break;
      }

    // Set active state
    case 0xA3: {
        byte valueByte;
        if (!readBytesOrTimeoutError(&valueByte, 1)) return;
        activeState = !!valueByte;
        if (activeState) setMatrix(matrixData, NULL);
        clearSerialBuffer();
        serialResponse(SUCCESS);
        break;
      }

    // Set Quick Update state
    case 0xA4: {
        byte valueByte;
        if (!readBytesOrTimeoutError(&valueByte, 1)) return;
        quickUpdate = !!valueByte;
        clearSerialBuffer();
        serialResponse(SUCCESS);
        break;
      }

    // Enter programming mode
    case 0xAF: {
        clearSerialBuffer();
        serialResponse(SUCCESS);
        enterProgrammingMode();
        break;
      }
  }
}

void debug() {
  for (byte col = 0; col < MATRIX_WIDTH; col++) {
    byte panel = col / PANEL_WIDTH;
    selectColumn(col);
    for (int row = 0; row < MATRIX_HEIGHT; row++) {
      selectRow(MATRIX_HEIGHT - row - 1, 1);
      flip(panel);
    }
  }
  deselect();
  delay(5000);

  for (byte col = 0; col < MATRIX_WIDTH; col++) {
    byte panel = col / PANEL_WIDTH;
    selectColumn(col);
    for (int row = 0; row < MATRIX_HEIGHT; row++) {
      selectRow(MATRIX_HEIGHT - row - 1, 0);
      flip(panel);
    }
  }
  deselect();
  delay(5000);
}

void setup() {
  Serial.begin(BAUDRATE);

  // Set those two pins high before setting them as outputs, since they are connected to active-low inputs
  digitalWrite(ROWS_BOTTOM, HIGH);
  digitalWrite(ROWS_TOP, HIGH);

  pinMode(ROW_A0, OUTPUT);
  pinMode(ROW_A1, OUTPUT);
  pinMode(ROW_A2, OUTPUT);
  pinMode(ROW_A3, OUTPUT);
  pinMode(COL_A0, OUTPUT);
  pinMode(COL_A1, OUTPUT);
  pinMode(COL_A2, OUTPUT);
  pinMode(COL_A3, OUTPUT);
  pinMode(COL_A4, OUTPUT);
  pinMode(ROWS_BOTTOM, OUTPUT);
  pinMode(ROWS_TOP, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(E3, OUTPUT);
  pinMode(E4, OUTPUT);
  pinMode(E5, OUTPUT);
  pinMode(LED, OUTPUT);

  setBacklight(false);
  delay(1000);
  setBacklight(true);
  delay(1000);
  setBacklight(false);
}

void loop() {
  doSerialCommunication();
}
