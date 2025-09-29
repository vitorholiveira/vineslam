### 1. Allow X11 access

On your **host machine**, run:

```bash
xhost +local:docker
```

### 2. Build and start the container

```bash
docker compose up --build
```

### 3. Access the container shell (if needed):

```bash
docker exec -it rviz_container bash
```

### 4. Build with colcon inside the container
```bash
source "/opt/ros/foxy/setup.bash"
cd src/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
