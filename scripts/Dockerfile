FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
gawk wget git diffstat unzip texinfo \
gcc build-essential chrpath socat cpio \
python3 python3-pip python3-pexpect \
xz-utils debianutils iputils-ping \
python3-git python3-jinja2 \
libsdl1.2-dev xterm sudo

WORKDIR /workspace
