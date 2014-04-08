#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/usr/local/lib/python2.7/dist-packages:/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote" \
    "/usr/bin/python" \
    "/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote/setup.py" \
    build --build-base "/home/morten/roswork/sdu/src/fmApp/sdu_frobit_remote" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
