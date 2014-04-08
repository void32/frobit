#!/usr/bin/env python
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
with open("/home/morten/roswork/sdu/src/fmLib/platform_support/sdu_frobit/scripts/frobit_test_set_speed.py", 'r') as fh:
    exec(fh.read())
