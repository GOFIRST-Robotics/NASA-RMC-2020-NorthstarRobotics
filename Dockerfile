FROM dydxdt/northstar-rmc:v1

WORKDIR /app

# Setup our stuff

COPY src/aruco_localization /app/src/aruco_localization
COPY src/decawave /app/src/decawave
COPY src/formatter_string /app/src/formatter_string
COPY src/navx /app/src/navx
COPY src/rovr_common /app/src/rovr_common
COPY src/rovr_control /app/src/rovr_control
COPY src/rovr_description /app/src/rovr_description
COPY src/serial /app/src/serial
COPY src/socketcan_bridge /app/src/socketcan_bridge
COPY src/socketcan_interface /app/src/socketcan_interface
COPY src/telecom /app/src/telecom
COPY devel/* /app/devel/
COPY documentation/* /app/documentation/

RUN cp /opt/ros/melodic/share/catkin/cmake/toplevel.cmake /app/src/CMakeLists.txt

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'