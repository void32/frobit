#!/usr/bin/env python
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
with open("/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote/mission_planners/remote.py", 'r') as fh:
    exec(fh.read())
