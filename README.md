# lil-ros-lm

- python3 -m venv venv
- source venv/bin/activate
- pip3 install --upgrade setuptools==58.0.2
- pip3 install -r requirements.txt
- colcon build --packages-select roslm
- source install/setup.bash