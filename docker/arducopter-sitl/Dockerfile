FROM dynamic-grasp:ros-one-jammy-arm64

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-one-mavros geographiclib-tools python3-pip python3-lxml python3-pexpect \
    python3-future python3-empy pkg-config ccache libtool libxml2-dev libxslt1-dev \
    && python3 -m pip install --no-cache-dir pymavlink==2.4.49 \
    && geographiclib-get-geoids egm96-5 \
    && rm -rf /var/lib/apt/lists/*

ARG COPTER_REF=Copter-4.7.2
RUN git clone --depth 1 --branch "${COPTER_REF}" --recurse-submodules \
      --shallow-submodules https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter -j6 \
    && git rev-parse HEAD > /opt/ardupilot-build-commit

ENV HIARM_COPTER_REF=${COPTER_REF}
