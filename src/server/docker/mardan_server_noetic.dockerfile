ARG BASE_IMAGE=ros:noetic-ros-base
FROM $BASE_IMAGE

RUN apt-get update && apt-get install -y \
    wget curl

RUN apt-get update && apt-get -y upgrade
RUN curl -sL https://deb.nodesource.com/setup_14.x  | bash -
RUN apt-get -y install nodejs
RUN npm install -g nodemon

WORKDIR /opt/src
COPY /src/server/src /opt/server/src/

COPY ./src/server/autostart.sh /
RUN chmod +x /autostart.sh

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/autostart.sh"]
CMD ["bash"]
