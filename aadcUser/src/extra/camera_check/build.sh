export PKG_CONFIG_PATH=/opt/opencv/3.0.0/lib/pkgconfig/
g++ -o check camera_check.cpp `pkg-config --cflags --libs opencv`
