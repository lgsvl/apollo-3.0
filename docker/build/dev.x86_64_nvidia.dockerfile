FROM apolloauto/apollo:dev-x86_64-20180702_1140

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get install -y autoconf \
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


