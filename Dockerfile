FROM debian:jessie AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN dpkg --add-architecture i386 \
	&& apt-get update -y \
	&& apt-get upgrade -y \
	&& apt-get install -y --no-install-recommends ca-certificates curl cmake git ninja-build \
		libc6:i386 libx11-6:i386 libxext6:i386 libstdc++6:i386 libexpat1:i386

#Install xc8 compiler
RUN curl -fSL -A "Mozilla/4.0" -o /tmp/xc8.run "http://www.microchip.com/mplabxc8linux" \
	&& chmod a+x /tmp/xc8.run \
	&& /tmp/xc8.run --mode unattended --unattendedmodeui none --netservername localhost --LicenseType FreeMode --prefix /opt/microchip/xc8 \
	&& rm /tmp/xc8.run
ENV PATH $PATH:/opt/microchip/xc8/bin
