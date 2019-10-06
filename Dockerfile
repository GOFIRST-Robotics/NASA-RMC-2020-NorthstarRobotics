FROM dydxdt/northstar-rmc:v1

WORKDIR /app

# Copy our files into the image

COPY devel/ /app/devel/
COPY documentation/ /app/documentation/
COPY src/ /app/src
