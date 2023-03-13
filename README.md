# Box-hardware
Software for the secure parcel box.
## Setup dependencies and build the project
1. If you use the Rasbperry Pi, all the dependencies should be installed. Otherwise, run <br/>
`rosdep install -i --from-path src --rosdistro galactic -y`<br/>
You only need to do it once.<br/>
2. To build the project, run <br/>
`colcon build` <br/>
3. If the build was successful, run <br/>
`.install/setup.bash` <br/>
Then, run the node of your choice using `ros2 run` or `ros2 launch`
## Run the state machine
1. Open a terminal <br/>
2. Run `colcon build` and `. install/setup.bash` <br/>
3. Run `ros2 launch launch/box.launch.py` <br/>
