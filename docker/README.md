# Docker Image

To build the Docker images for Nvidia Jetson Xavier NX, you simply need to run `./docker_build.sh` on this folder, and it will build the images automatically. Make sure you setup SSH access to GitHub, as the submodules are linked through SSH (not HTTP).

After building the image, run `./docker_run.sh` to automatically start the Docker container.

To use ROS remotely (from your computer to the Jetson Container), make sure to follow the [Network Setup](http://wiki.ros.org/ROS/NetworkSetup) guide.