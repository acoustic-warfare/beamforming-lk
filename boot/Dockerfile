# Docker-tftpd
FROM ubuntu:latest

# Build
# docker build -t tftpd .

# Start the container
# docker run -it -d --rm --network=host -v YOUR_PATH:/var/lib/tftpboot --name tftp-server tftpd

# Stop the service
# docker kill tftp-server

# Install tftpd-hpa deamon
RUN apt-get update
RUN apt-get install -y tftpd-hpa
VOLUME /var/lib/tftpboot

EXPOSE 69

CMD /usr/sbin/in.tftpd --foreground --user tftp -a 0.0.0.0:69 -v /var/lib/tftpboot -s
