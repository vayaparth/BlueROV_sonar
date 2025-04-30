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

echo_and_run cd "/home/ubuntu/sonar_ws/src/ping_sonar"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/sonar_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/sonar_ws/install/lib/python3/dist-packages:/home/ubuntu/sonar_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/sonar_ws/build" \
    "/usr/bin/python3" \
    "/home/ubuntu/sonar_ws/src/ping_sonar/setup.py" \
     \
    build --build-base "/home/ubuntu/sonar_ws/build/ping_sonar" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/sonar_ws/install" --install-scripts="/home/ubuntu/sonar_ws/install/bin"
