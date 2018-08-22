FROM apolloauto/apollo:dev-x86_64-20180702_1140

RUN DEBIAN_FRONTEND=noninteractive \
    apt-get install -y autoconf \
       autoconf \
       libtool \
       pkg-config \
       python \
       libxext-dev \
       x11proto-gl-dev \
       mesa-utils

WORKDIR /opt/libglvnd
RUN git clone --branch=v1.0.0 https://github.com/NVIDIA/libglvnd.git . && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/x86_64-linux-gnu && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/x86_64-linux-gnu -type f -name 'lib*.la' -delete
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        gcc-multilib \
        libxext-dev:i386 \
        libx11-dev:i386 && \
    rm -rf /var/lib/apt/lists/*
COPY ./10_nvidia.json /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json
RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig
ENV LD_LIBRARY_PATH /usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}


ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility
ENV NVIDIA_VISIBLE_DEVICES all

WORKDIR /apollo

RUN mkdir glfw && \
    cd glfw && \
    wget -O - https://github.com/glfw/glfw/archive/3.2.1.tar.gz | tar -xz && \
    cd glfw-3.2.1 && \
    cmake . -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_DOCS=OFF -DBUILD_SHARED_LIBS=ON && \ 
    make && \
    make install && \
    cd /apollo && \
    rm -rf glfw-3.2.1

# may not be needed if not already installed
RUN apt remove -y libglfw3 libglfw3-dev

RUN mkdir glew && cd glew && \
    wget -O - https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.tgz/download | tar -xz && \
    cd glew-2.1.0 && \
    cd include && \
    mkdir KHR && \
    cd KHR && \
    wget https://www.khronos.org/registry/EGL/api/KHR/khrplatform.h && \
    cd .. && \
    mkdir EGL && \
    cd EGL && \
    wget https://www.khronos.org/registry/EGL/api/EGL/eglplatform.h && \
    cd ../.. && \
    make && \
    GLEW_DEST=/usr/local SYSTEM=linux-egl make install && \
    cd /apollo && \
    rm -rf glew-2.1.0

RUN rm /usr/lib/libGLEW* /usr/lib64/libGLEW*

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib64

# dependencies for rosbridge
RUN pip install --upgrade empy \
    mangopy \
    zope.interface \
    pillow \
    twisted

RUN touch /usr/local/lib/python2.7/dist-packages/zope/__init__.py

# compile and include libpcl without avx2
COPY patch/libpcl.patch /tmp/

RUN wget -O - https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz | tar -xz && \
    cd pcl-pcl-1.7.2 && \
    patch -i /tmp/libpcl.patch && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j`nproc` && \
    cp -a lib/* /usr/local/lib/ && \
    ldconfig && \
    cd /apollo && \
    rm -rf pcl-pcl-1.7.2

RUN rm -rf /home/tmp/ros/lib/python2.7/dist-packages/numpy && \
    /usr/local/miniconda2/bin/conda install -y matplotlib && \
    /usr/local/miniconda2/bin/conda install -y -c menpo opencv
