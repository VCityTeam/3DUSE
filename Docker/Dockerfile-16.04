FROM ubuntu:16.04
LABEL vendor="RICT project"

# Core dependencies
RUN apt-get update
RUN apt-get -y install libboost-all-dev
RUN apt-get -y install libassimp-dev
RUN apt-get -y install libxml2-dev
RUN apt-get install -y libgdal1-dev  # Pulls version 1.10.1
                                     # Check with apt-cache policy libgdal1-dev

# Notes concerning the GDAL dependency
# The last documented GDAL version in the 1.X series that worked with
# 3DUSE is 1.11.4. That version, that is not packaged for Ubuntu, can be
# installed with the following commands (see below). Yet this installation
# fails in being effective for 3DUSE, because GDAL has many optional 
# sub-dependencies (e.g. proj, json-c, libgeotiff, libtiff, libspatialite,
# liblwgeom 2.4.4) that are not automatically pulled by the following
# manual install. And by default when GDAL is not compiled without prior
# install of say libgeos-dev, then some methods will change their behavior
# and e.g. OGRPolygon::IsValid() will always return false.
# cd /tmp
# wget http://download.osgeo.org/gdal/1.11.4/gdal-1.11.4.tar.gz
# tar zxvf gdal-1.11.4.tar.gz
# cd gdal-1.11.4
# mkdir $HOME/local/
# ./configure --prefix=$HOME/local/gdal-1.11.4 --enable-shared
# make
# make install

# Graphical User Interface dependencies
RUN apt-get -y install libopenscenegraph-dev
RUN apt-get -y install qt4-default libqt4-opengl-dev


# Software engineering tools (build stages)
RUN apt-get -y install build-essential g++ # Pulls gcc, make...
RUN apt-get -y install git
RUN apt-get -y install cmake
RUN apt-get -y install wget

# PCL is optionnal: when selected it has its own sub-dependencies
RUN apt-get -y install libeigen3-dev
RUN apt-get -y install libflann-dev
RUN apt-get -y install libqhull-dev
RUN apt-get -y install libpng12-dev

# For 3DUSE, a specific version of PCL is required. Proceed with manual install
RUN cd /tmp &&                                 \
    wget https://download.gforge.liris.cnrs.fr/meppbin/travis-trusty/pcl-pcl-1.7.2.travis-trusty.tgz && \
    tar zxf pcl-pcl-1.7.2.travis-trusty.tgz && \
    cd pcl-pcl-1.7.2/buildR &&                 \
    make &&                                    \
    make install  # Must be root

# Eventually install 3DUSE internal (embarked) dependencies
RUN cd                                                 && \
    git clone https://github.com/MEPP-team/3DUSE       && \
    cd 3DUSE                                           && \
    cd externals/laslib                                && \
    mkdir Build                                        && \
    cd Build                                           && \
    cmake .. -DCMAKE_BUILD_TYPE=Release                   \
             -DCMAKE_INSTALL_PREFIX=$HOME/local/laslib && \
    make                                               && \
    make install
# And terminate the install by building 3DUSE itself
RUN cd ~/3DUSE                                      && \
    mkdir Build                                     && \
    cd Build                                        && \
    cmake --verbose                                    \
    -DBUILD_GUI_QT4=ON                                 \
    -DBUILD_PCL=ON                                     \
    -DLASLIB_INCLUDE_DIR=~/local/laslib/include        \
    -DLASLIB_LIBRARY=~/local/laslib/lib/liblaslib.a    \
    ..                                              && \
    make                                            && \
    make install
