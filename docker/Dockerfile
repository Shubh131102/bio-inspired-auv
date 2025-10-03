# Base image depends on your ROS2 distro; tweak as needed
FROM ros:humble

RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /ws
COPY . /ws
# You can uncomment these lines when your packages are real:
# RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
# RUN colcon build
