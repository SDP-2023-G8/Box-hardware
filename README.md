# Box-hardware
Software for the secure parcel box.
## Deploy on Raspberry Pi
The software is run in  a docker container containing all the dependencies. Run<bt/>
`./run_docker.sh`<br/>
to run an interactive shell in the container. You should be in /inbox/Box-hardware after running the command.
## Setup dependencies and build the project
1. If you use docker, all the dependencies should be installed. Otherwise, run <br/>
`rosdep install -i --from-path src --rosdistro galactic -y`<br/>
You only need to do it once.<br/>
2. To build the project, run <br/>
`colcon build` <br/>
3. If the build was successful, run <br/>
`.install/setup.bash` <br/>
Then, run the node of your choice using `ros2 run` or `ros2 launch`
## Run the state machine
1. Open 3 terminals (run ./run_docker.sh, colcon build and . install/setup.bash before) <br/>
1.1 Terminal 1: ros2 run door_lock door_lock <br/>
1.2 Terminal 2: ros2 launch state_machine state_machine.launch.py <br/>
1.3 Terminal 3: ros2 launch qr_scanner_pipeline qr_scanner.launch.py <br/>