#!/bin/sh

# Must be run from top level of directory and run as sudo

# Setup the systemd service based on processor architecture
ARCH=$(uname -m)

case "$ARCH" in
    x86_64)
        cp ./deployment/proxy_service/x86_64/grpc_web_proxy.service /etc/systemd/system/;;
    arm*)
        cp ./deployment/proxy_service/arm/grpc_web_proxy.service /etc/systemd/system/;;
    aarch64)
        cp ./deployment/proxy_service/arm/grpc_web_proxy.service /etc/systemd/system/;;
    *)
        printf 'Unsupported Architecture: "%s"\n' "$ARCH" >&2; exit 2;;
esac

# Enable and start the systemd service 
systemctl daemon-reload
systemctl enable grpc_web_proxy.service
systemctl start grpc_web_proxy.service