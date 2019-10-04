FROM dydxdt/northstar-rmc:v1

WORKDIR /app

# Setup our stuff

COPY devel/ /app/devel/
COPY documentation/ /app/documentation/
COPY src/ /app/src

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'