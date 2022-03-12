#!/usr/bin/env sh
set -e

cd ..
PEDAL=1 scons -u
cd pedal

./enter_canloader.py ../obj/pedal.bin.signed
