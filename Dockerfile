# Use the official ROS base image for ARM architecture
FROM arm32v7/ros:noetic-ros-base

# Set the working directory in the container
WORKDIR /app

# Copy the project files to the container
COPY . /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-opencv \
    python-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages (replace with your package names)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-package1 \
    ros-noetic-package2 \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies
RUN pip install --no-cache-dir \
    rospy

# Set the entry point command
CMD ["python", "ball_gest_detection.py"]

