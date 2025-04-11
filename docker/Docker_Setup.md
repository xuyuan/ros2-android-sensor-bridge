# Docker Deployment for Mobile Sensor Bridge

This directory contains Docker configuration files to easily deploy the Mobile Sensor Bridge for ROS2.

## Prerequisites

- Docker
- Docker Compose
- Git

## Quick Start

1. Clone the repository:
   ```bash
   git clone https://github.com/VedantC2307/ros2-android-sensor-bridge.git
   cd ros2-android-sensor-bridge
   ```

2. Copy the Dockerfile, docker-entrypoint.sh, and docker-compose.yml files to the repository root.

3. Build and start the Docker container:
   ```bash
   docker-compose up --build
   ```

4. Access the web interface on your mobile device:
   - Open a browser on your mobile device and navigate to `https://<your_computer_ip>:4000`
   - Accept the security warnings about the self-signed certificate
   - Grant permissions for camera, microphone, and AR features when prompted

## Manual Docker Build

If you prefer to build and run the Docker container manually:

1. Build the Docker image:
   ```bash
   docker build -t mobile_sensor_bridge .
   ```

2. Run the container:
   ```bash
   docker run -it --rm \
     --network host \
     --privileged \
     --env="DISPLAY=$DISPLAY" \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     --volume="$XAUTHORITY:/root/.Xauthority" \
     --volume="$(pwd):/ros2_ws/src/mobile_sensor" \
     mobile_sensor_bridge
   ```

## Development Workflow

For development, you can mount the local repository into the container and make changes in real-time:

1. Start the container in development mode:
   ```bash
   docker-compose up
   ```

2. Make changes to the code on your local machine. The changes will be reflected in the container.

3. If you need to rebuild the ROS package after making changes, you can run:
   ```bash
   docker exec -it mobile_sensor_bridge bash -c "cd /ros2_ws && colcon build --packages-select mobile_sensor && source install/setup.bash"
   ```

## Troubleshooting

### X11 Display Issues

If you encounter issues with visualization tools like RViz2, make sure your host allows X11 connections:

```bash
xhost +local:docker
```

### SSL Certificate Issues

If the SSL certificate generation fails, you can manually generate certificates:

```bash
docker exec -it mobile_sensor_bridge bash -c "cd /ros2_ws/src/mobile_sensor/src && ./generate_ssl_cert.sh"
```

### Node.js Dependency Issues

If you encounter Node.js dependency issues, you can manually install dependencies:

```bash
docker exec -it mobile_sensor_bridge bash -c "cd /ros2_ws/src/mobile_sensor && npm install"
```