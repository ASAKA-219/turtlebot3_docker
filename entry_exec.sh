#!/bin/bash
container_name=$(docker ps --format ',{{.Names}}')
image_name="noetic:tb3"
image_exists=$(docker images -q "${image_name}")

if [ -n "$image_exists" ] ;then
  #イメージが存在する場合
  echo "Dockerイメージ" $image_name "を見つけたよ！"${container_name} "を起動するね！"
  docker compose up -d
else
  echo "Dockerイメージ" $image_name "は見つからなかったよ！別の方法でコンテナを起動するね！"
  docker compose up -d --build
fi
docker compose exec turtlebot3_noetic /bin/bash
