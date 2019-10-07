FROM dydxdt/northstar-rmc:v1

WORKDIR /app

# Copy our files into the image

COPY documentation/ /app/documentation/
COPY src/ /app/src
COPY .catkin_tools /app/.catkin_tools
