#!/bin/bash
image_name="humble_tb3:burger"
container="turtlebot3_burger"

image_exists=$(docker images -q "${image_name}")

# 条件分岐でイメージが存在するかを判断
if [ -n "$image_exists" ]; then
  # イメージが存在する場合の処理
  echo "イメージ"${image_name}"を見つけたよ！"${image_name}"を実行するね！"
  docker compose up -d $container
else
  # イメージが存在しない場合の処理
  echo "イメージ" ${image_name} "は見つからなかったよ！別の方法でコンテナを起動するよ！"
  docker compose up -d --build
fi
docker compose exec $container /bin/bash

