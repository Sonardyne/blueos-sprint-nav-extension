# Copyright 2024 Sonardyne

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Create a target for building dependencies
FROM ubuntu:22.04 AS build-dependencies

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ=Etc/UTC

ARG ARCH
ARG TARGETARCH

# Install dependencies
RUN apt clean && apt autoclean && apt update && apt install -y \
        build-essential \
        git \
        libxml2 \
        libxml2-dev \
        libxml2-utils \
        libxslt-dev \
        meson \
        python3 \
        nginx \
        ninja-build \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        systemctl \
        vim \
        wget && \
        rm -rf /var/lib/apt/lists/*

ENV PATH=$PATH:/usr/local/go/bin
ENV GOPATH=/go/
ENV GOARCH=${TARGETARCH}
ENV GOPROXY=https://proxy.golang.org,direct
ENV GOMAXPROCS=1

RUN if [ "${TARGETARCH}" = "arm64" ]; \
        then GOARCH=arm; \
        fi

# Must update GO due to gRPC-Web dependency. ARMv7 architecture must be set to arm64 
RUN if [ "${TARGETARCH}" = "arm" ]; \
        then wget https://go.dev/dl/go1.23.0.linux-arm64.tar.gz && rm -rf /usr/local/go && tar -C /usr/local -xzf go1.23.0.linux-arm64.tar.gz; \
        else wget https://go.dev/dl/go1.23.0.linux-${TARGETARCH}.tar.gz && rm -rf /usr/local/go && tar -C /usr/local -xzf go1.23.0.linux-${TARGETARCH}.tar.gz; \
        fi

RUN git clone https://github.com/improbable-eng/grpc-web.git $GOPATH/src/github.com/improbable-eng/grpc-web && \
        cd $GOPATH/src/github.com/improbable-eng/grpc-web && \
        go get ./go/grpcwebproxy && \
        go install ./go/grpcwebproxy

# Create a target for building the BlueOS Extension WebUI
FROM build-dependencies AS build-blueos-docker-image

WORKDIR /webui

ADD . /webui

RUN mkdir /webui/logs

RUN python3 -m pip install -r ./webui_backend/requirements.txt

RUN cd ./webui_backend && python3 -m grpc_tools.protoc -I=../protobuf --python_out=. --grpc_python_out=. ../protobuf/message-service.proto

EXPOSE 9091/tcp 

LABEL version="1.0.4"
LABEL permissions='\
          {\
            "NetworkMode": "host", \
              "ExposedPorts": {\
                "9091/tcp": {}\
            },\
            "HostConfig": {\
              "Privileged": true,\
              "NetworkMode": "host",\
              "Binds":[\
                "/dev:/dev", \
                "/usr/blueos/extensions/data-logger:/webui/logs", \
              ],\
              "PortBindings": {\
                "9091/tcp": [\
                  {\
                    "HostPort": ""\
                  }\
                ]\
              }\
            }\
          }'
LABEL authors='[\
          {\
            "name": "Sonardyne International Limited",\
            "email": "support@sonardyne.com"\
          }\
        ]'
LABEL company='{\
          "about": "",\
          "name": "Sonardyne International Limited",\
          "email": "support@sonardyne.com"\
        }'

LABEL type="device-integration"
LABEL tags='[\
        "positioning",\
        "navigation",\
        "dvl",\
        "ins",\
        "position-hold",\
        ]'
LABEL readme='https://raw.githubusercontent.com/Sonardyne/blueos-sprint-nav-extension/main/README.md'
LABEL links='{\
        "website": "https://www.sonardyne.com/",\
        "support": "https://www.sonardyne.com/support-centre/"\
        }'

ENTRYPOINT ["./deployment/deploy_blueos_extension.sh"]

# Create a target for running the BlueOS Extension WebUI tests. Used for CI/CD
FROM build-blueos-docker-image AS run-unit-tests

RUN python3 -m pip install pytest

RUN cd /webui/webui_backend/tests/ && pytest test-hnavDecode.py --junitxml=unit_test_report.xml

# Create a target for exporting the unit tests. Used for CI/CD
FROM scratch AS export-tests

COPY --from=run-unit-tests /webui/webui_backend/tests/unit_test_report.xml /