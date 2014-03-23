#!/bin/sh -x
#
# Usage: Arduino-Crosspack /Applications/Arduino.app
#
AVR="$1/Contents/Resources/Java/hardware/tools/avr"
 
sudo rm -rf "$AVR"
sudo ln -s /usr/local/CrossPack-AVR "$AVR"
sudo codesign -fs - "$1"
