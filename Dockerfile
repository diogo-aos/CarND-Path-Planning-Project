from ubuntu:18.04

RUN apt-get update
RUN apt-get install -y g++
RUN apt-get install -y gcc
RUN apt-get install -y cmake
RUN apt-get install -y git
RUN apt-get install -y sudo

COPY install-ubuntu.sh /home/install-ubuntu.sh
RUN sh /home/install-ubuntu.sh