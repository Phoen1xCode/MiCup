#!/bin/bash

set -e

LOCAL_CODE=~/MiCup
CONTAINER_NAME=cyberdog_dev
IMAGE=cyberdog_sim:v2026

xhost +local:docker >/dev/null 2>&1 || xhost +

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "==> 容器 ${CONTAINER_NAME} 正在运行,挂入新终端"
        docker exec -it ${CONTAINER_NAME} /bin/bash
    else
        echo "==> 启动已有容器 ${CONTAINER_NAME}"
        docker start -ai ${CONTAINER_NAME}
    fi
else
    echo "==> 首次创建开发容器 ${CONTAINER_NAME}"
    docker run -it \
        --shm-size="1g" \
        --privileged=true \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ${LOCAL_CODE}:/home \
        -w /home \
        --name ${CONTAINER_NAME} \
        ${IMAGE}
fi
