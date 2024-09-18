#!/bin/sh

# Must be run from top level of directory and run as sudo
# If running this file no need to run any other files in this directory

# Install gRPC-Web proxy service
./deployment/proxy_service/create_proxy_service.sh

# Create a server to host the WebUI
./deployment/create_webui_nginx.sh

python3 ./webui_backend/grpcServer.py