#!/bin/bash

# 사용할 이미지 이름과 컨테이너 이름을 변수로 지정합니다. (나중에 여기만 바꾸면 됩니다)
# Define image and container names as variables for easy modification.
IMAGE_NAME="hri_drone_env:latest"
CONTAINER_NAME="drone_workspace"

# 호스트 컴퓨터의 화면(X Server) 접근 권한을 모든 로컬 사용자에게 허용합니다. (가재보 화면 출력용)
# Allow local root access to the X server display for GUI applications.
xhost +local:root

# 만약 동일한 이름의 컨테이너가 이미 존재한다면, 충돌을 방지하기 위해 강제로 삭제합니다.
# If a container with the same name exists, remove it forcefully to avoid conflicts.
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "기존 컨테이너($CONTAINER_NAME)를 삭제하고 새로 시작합니다..."
    docker rm -f $CONTAINER_NAME
fi

# 도커 컨테이너를 실행합니다. 드론 프로젝트에 필요한 모든 하드웨어 권한과 볼륨 설정을 포함합니다.
# Run the Docker container with full hardware privileges and volume configurations.
docker run -it \
    --name $CONTAINER_NAME \
    --privileged \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/wearable_project/src:/app/src" \
    $IMAGE_NAME /bin/bash

# 컨테이너가 종료되면 화면 접근 권한을 다시 원래대로(안전하게) 되돌려 놓습니다.
# Revoke the X server access permission after the container exits for security.
xhost -local:root
