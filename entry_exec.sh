#!/bin/bash
container_name=$(docker ps --format ',{{.Names}}')
if [ "$container_name == noetic-tb3" ] ;then
  echo $container_name 'is already running.'
  docker compose up -d
else
  docker compose up -d --build
fi
docker compose exec turtlebot3_noetic /bin/bash
