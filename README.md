# lil-ros-lm

- python3 -m venv venv
- source venv/bin/activate
- pip3 install --upgrade setuptools==58.0.2
- pip3 install -r requirements.txt

- rm -rf build/ install/
- colcon build --packages-select roslm
- source install/setup.bash
OR
- echo "source install/setup.bash" >> ~/.bashrc
- source ~/.bashrc

## Usage

two packages: rosgpt and turtlesim.

four nodes:

rosgpt has three: rosgpt, rosgptparser_turtlesim, rosgpt_client_node.

turtlesim has one: turtlesim_node.


* Run the ROSGPT Flask server

```sh
ros run rosgpt rosgpt
```

* Run the turtlesim node

``` sh
ros2 run turtlesim turtlesim_node
```

* Run rosgptparser_turtlesim.py

```sh
 ros2 run rosgpt rosgptparser_turtlesim
```

4. Run rosgpt_client_node.py

```sh
ros2 run rosgpt rosgpt_client_node
```

In the terminal of rosgpt_client_node, type English commands to the robot.  For example, “i want you to move 1 meter speed 0.8”.
