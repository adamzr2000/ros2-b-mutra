# Running the Gazebo Robot Simulation

## Installation

1. Clone the repository:
```bash
git clone git@github.com:adamzr2000/turtlebot3-service-remote-attestaition.git
```

2. Build Docker Images:
Navigate to the [dockerfiles](./dockerfiles) directory and run the `./build.sh` scripts for each image.


## Start the Simulation
To launch the Gazebo robot simulator along with two spawned robots, run:

```bash
docker compose up -d
```

This will:
- Start the Gazebo server.
- Spawn two robots in the simulation.
- Launch a VNC container with the Gazebo client for graphical access via a web browser.

## Access the Gazebo Client
1. Environment variables are defined in the [.env](./env) file.
2. Open a browser and navigate to: [http://127.0.0.1:6080](http://127.0.0.1:6080).
3. Open a terminal inside the VNC session and execute:

   ```bash
   ./run_gazebo_client.sh
   ```

## Stop the Simulation
To shut down the simulation and related containers, run:

```bash
docker compose down
```

## ROS Node for Remote Attestation
The C++ executable responsible for robot state publishing can be found at:

[robot_state_publisher.cpp](./dockerfiles/turtlebot3/ros2_ws/src/robot_state_publisher/src/robot_state_publisher.cpp)

