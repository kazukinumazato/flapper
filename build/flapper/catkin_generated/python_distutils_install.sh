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

echo_and_run cd "/home/mech-user/flapper_ws/src/flapper"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mech-user/flapper_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mech-user/flapper_ws/install/lib/python3/dist-packages:/home/mech-user/flapper_ws/build/flapper/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mech-user/flapper_ws/build/flapper" \
    "/home/mech-user/.pyenv/shims/python3" \
    "/home/mech-user/flapper_ws/src/flapper/setup.py" \
     \
    build --build-base "/home/mech-user/flapper_ws/build/flapper" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/mech-user/flapper_ws/install" --install-scripts="/home/mech-user/flapper_ws/install/bin"
