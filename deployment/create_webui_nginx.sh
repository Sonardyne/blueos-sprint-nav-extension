#!/bin/sh

# Must be run from top level of directory and run as sudo

# Copy application to /var/www/
cp -r ./dist/blueos-sprint-nav-extension /var/www

# Copy nginx.conf to host webUI
cp ./deployment/nginx.conf /etc/nginx

# Restart nginx. WebUI will be hosted at the address and port specified in the server file
systemctl restart nginx
