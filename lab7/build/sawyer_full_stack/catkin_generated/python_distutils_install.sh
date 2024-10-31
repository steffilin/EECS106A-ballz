#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/src/sawyer_full_stack"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/install/lib/python3/dist-packages:/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/build" \
    "/usr/bin/python3" \
    "/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/src/sawyer_full_stack/setup.py" \
    egg_info --egg-base /home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/build/sawyer_full_stack \
    build --build-base "/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/build/sawyer_full_stack" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/install" --install-scripts="/home/cc/ee106a/fa24/class/ee106a-ail/project/lab7/install/bin"