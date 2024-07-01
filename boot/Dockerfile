#
# Author: Irreq
#
# Dockerfile for TFTP server to host bitstreams and .elf file for the FPGA
#
# To build the container run the following command:
# 
# docker build -t tftpd .
#
# To run the container
# 
# docker run -it -d --rm --network=host -v $(pwd)/tftp:/var/lib/tftpboot --name tftp-server tftpd
#
# To stop the service
#
# docker kill tftp-server

FROM ubuntu:22.04

# Install tftpd-hpa deamon
RUN apt-get update
RUN apt-get install -y tftpd-hpa
VOLUME /var/lib/tftpboot

EXPOSE 69

CMD /usr/sbin/in.tftpd --foreground --user tftp -a 0.0.0.0:69 -v /var/lib/tftpboot -s
