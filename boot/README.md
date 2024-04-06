# Boot 
This directory contains required boot sequence for FPGA and TFTP server. This
enables the user to give the FPGA access to the files `bitstream` and `ps.elf`
(https://github.com/acoustic-warfare/FPGA-sampling/releases/tag/2.0)
which are required to boot. This implementation suggest having the above files
in the directory `tftp/` and have the system's IP address at a static `10.0.0.1`
which may be configured in settings. 

## Prerequisites 
Install `docker` (https://www.docker.com/) if not configured properly you may
need root privileges to run the following commands. If the Docker daemon is not
running, you may start it using `sudo dockerd`.

## Setup TFTP Server

Build the server 

    docker build -t tftpd .

Start the server-container. This will require absolute path to the TFTP
directory containing your files. 
    
    docker run -it -d --rm --network=host -v $(pwd)/tftp:/var/lib/tftpboot --name tftp-server tftpd

Stopping the service 

    docker kill tftp-server

    
    
