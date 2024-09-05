# ROS2 Container ID
ROS2_CONTAINER_ID=""

# Alias to build the Docker image
alias ws-build="docker build -t stewart_platform_ros2 -f docker/Dockerfile ."

# Function to run the Docker container interactively with a mounted volume
function ws-start {
    local workspace_dir=$(pwd)
    ROS2_CONTAINER_ID=$(docker run -d -it --runtime=nvidia --gpus all -v $workspace_dir/ros2_ws:/ros2_ws stewart_platform_ros2)
    echo "Started container with ID: $ROS2_CONTAINER_ID"
}

# Function to attach to the running container using
function ws-exec {    
    if [ -n "$ROS2_CONTAINER_ID" ]; then
        docker exec -it "$ROS2_CONTAINER_ID" bash
    else
        echo "No container is currently running. Start a container first."
    fi    
}

# Function to stop and remove the running container
function ws-stop {
    if [ -n "$ROS2_CONTAINER_ID" ]; then
        docker stop "$ROS2_CONTAINER_ID" && docker rm "$ROS2_CONTAINER_ID"
        echo "Stopped and removed container with ID: $ROS2_CONTAINER_ID"
        ROS2_CONTAINER_ID=""
    else
        echo "No container is currently running."
    fi
}

