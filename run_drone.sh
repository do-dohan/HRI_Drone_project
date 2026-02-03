#!/bin/bash

# 1. 화면 권한 허용 (GUI 실행용)
xhost +

# 2. 컨테이너 이름 정의 (여기가 중요! 실제 실행되는 이름과 맞춰야 함)
# 로그에 'hri_drone_container Created'라고 떴으므로 이걸로 수정합니다.
CONTAINER_NAME="hri_drone_container"

# 3. 컨테이너 실행 (없으면 만들고, 꺼져있으면 켭니다)
echo "🚀 Starting Container..."
docker compose up -d

# 4. 컨테이너 접속
echo "🔌 Entering Workspace..."
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    docker exec -it $CONTAINER_NAME bash
else
    echo "❌ Error: Container is not running!"
    echo "Check logs: docker logs $CONTAINER_NAME"
fi