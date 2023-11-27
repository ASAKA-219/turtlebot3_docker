# turtlebot3 simulation in docker
　エントリー（docker-composeの方をinstallしている場合）
```bash
docker-compose up -d --build turtlebot3_noetic; docker-compose exec turtlebot3_noetic /bin/bash
```

エントリー（docker composeの方をinstallしている場合）
```bash
docker compose up -d --build turtlebot3_noetic; docker compose exec turtlebot3_noetic /bin/bash
```
