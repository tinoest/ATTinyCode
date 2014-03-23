#!/bin/sh -x
#
# Usage: Arduino-CrossPack /Applications/Arduino.app
#
AVR="$1/Contents/Resources/Java/hardware/tools/avr"
AVR="$1/Contents/Resources/Java/hardware/tools/avr-old"
 
sudo mv -f "$AVR" "$AVRMV"
sudo ln -s /usr/local/CrossPack-AVR "$AVR"
sudo codesign -fs - "$1"
