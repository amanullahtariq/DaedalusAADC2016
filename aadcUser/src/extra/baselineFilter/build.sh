export PKG_CONFIG_PATH=/opt/opencv/3.0.0/lib/pkgconfig/
g++ -o blend baseline_mean.cpp `pkg-config --cflags --libs opencv`
g++ -o process processing.cpp `pkg-config --cflags --libs opencv`
