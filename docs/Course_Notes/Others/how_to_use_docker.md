## Simple introduction about docker
Following tutorial: [link](https://docs.docker.com/get-started/docker-overview/)

> Official description: Docker provides the ability to package and run an application in a loosely isolated environment called a container. The isolation and security lets you run many containers simultaneously on a given host. Containers are lightweight and contain everything needed to run the application, so you don't need to rely on what's installed on the host.

[Cited from "What is a container"](https://docs.docker.com/get-started/docker-concepts/the-basics/what-is-a-container/)
> Imagine you're developing a killer web app that has three main components - a React frontend, a Python API, and a PostgreSQL database. If you wanted to work on this project, you'd have to install Node, Python, and PostgreSQL.
> How do you make sure you have the same versions as the other developers on your team? Or your CI/CD system? Or what's used in production?
> How do you ensure the version of Python (or Node or the database) your app needs isn't affected by what's already on your machine? How do you manage potential conflicts?
> Enter containers!
> What is a container? Simply put, containers are isolated processes for each of your app's components. Each component - the frontend React app, the Python API engine, and the database - runs in its own isolated environment, completely isolated from everything else on your machine.
> Here's what makes them awesome. Containers are: 
> Self-contained. Each container has everything it needs to function with no reliance on any pre-installed dependencies on the host machine.
Isolated. Since containers are run in isolation, they have minimal influence on the host and other containers, increasing the security of your applications.
Independent. Each container is independently managed. Deleting one container won't affect any others.
Portable. Containers can run anywhere! The container that runs on your development machine will work the same way in a data center or anywhere in the cloud!

## Example tutorial of using ROS2, Docker, SSH
Summary: Running ROS2 hello_world node inside docker container on InDro robot

### SSH connection
```bash
ssh username@indro-ip
```

### ROS2 workspace and package creation (ROS2 basic operations)
Step1: Initialize the ROS2 workspace and package
```bash
# initialize the ROS2 workspace
mkdir -p ~/hello_indro_ws/src
cd ~/hello_indro_ws
colcon build

# initialize the ROS2 package
cd ~/hello_indro_ws/src
ros2 pkg create --build-type ament_python hello_indro
```

Step2: Write the source code for the hello_indro node
``` bash
cd ~/hello_indro_ws/src/hello_indro/hello_indro
vim hello_indro_node.py # use any preferred editor

```
edit the python source code
```py
import rclpy
from rclpy.node import Node

class HelloInDroNode(Node):
    def __init__(self):
        super().__init__('hello_indro_node')
        self.get_logger().info('Hello, InDro!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloInDroNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Step3: Make the script executable
```bash
chmod +x hello_indro_node.py
```

Step4: Edit the `setup.py` file to modify the `entry_points`

```bash
vim ~/hello_indro_ws/src/hello_indro/setup.py # use any preferred editor
```

```py
entry_points={
    'console_scripts': [
        'hello_indro = hello_indro.hello_indro_node:main',
    ],
}
```

Step4: Build the ROS2 package and Source the environmental variable
```bash
cd ~/hello_indro_ws
colcon build
source install/setup.bash

# check the created package
ros2 pkg list | grep hello_indro
```

### Docker Container Setup
Step1: Create the Dockerfile
```bash
cd ~/hello_indro_ws
vim Dockerfile # use any preferred editor
```

```dockerfile
# include the ROS2 base image, which includes a minimal ROS2 installation
FROM ros:foxy

# Set working directory
WORKDIR /hello_indro_ws

# Copy the workspace into the container
COPY . /hello_indro_ws

# Install dependencies, build, and source the local environmental variable
# if additional ROS2 packages are required, they should be listed here to install them
# source the local environmental variable may not be needed here, to be further tested
RUN apt update && apt install -y python3-pip && \
    colcon build && \
    echo "source /hello_indro_ws/install/setup.bash" >> /root/.bashrc

# Default command to run if no extra command is provided
CMD ["ros2", "run", "hello_world", "hello_world"]

# Set entrypoint to source both the base ROS2 and workspace environments before running the above CMD
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && source /hello_indro_ws/install/setup.bash && exec \"$@\"", "--"]
```

Step2: Build the Docker image
```bash
docker build -t hello_world_image .
```
Note: If this is the first time building, Docker needs to download the ROS2 foxy base image, which can take a few mintues. If the base image is already cached, it will be much faster.

### Run the hello_indro Node in Docker Container
```bash
docker run --rm --net=host hello_world_image
```

### Debug & Test
Check the Docker container information
```bash
docker ps # list the running Docker containers

docker logs container_id # check the output logs inside the Docker container

docker exec -it container_id /bin/bash # interactively access the terminal of the running container

docker stats <container_id_or_name> # shows live resources usage (CPU, memory, network, etc.) of the container

```

Run a Docker container with an interactive shell:
```bash
docker run --rm -it --net=host hello_indro_image bash
```

Rebuild the ROS2 package after modification
```bash
colcon build --packages-select <your_package_name>
```

Rebuild the Docker image
```bash
# with the --no-cache option to do a clean build, ignoring cached layers
docker build --no-cache -t hello_world_image .
```
