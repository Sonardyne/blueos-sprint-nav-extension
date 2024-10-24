#!/bin/sh

# Must be run from top level of directory and run as sudo
# If running this file no need to run any other files in this directory

# Get the host IP address which matches 192.168.2.255 and write it to file for systemd services and python server to access
hostname -I | grep -o '192.168.2.\b[0-9]\b' |  awk '{print "IP_ADDRESS=" $0}' > /webui/ip_address.txt

# Install gRPC-Web proxy service
./deployment/proxy_service/create_proxy_service.sh

# Create a server to host the WebUI
./deployment/create_webui_nginx.sh

python3 ./webui_backend/grpcServer.py