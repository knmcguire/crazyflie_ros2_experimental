clean:
	rm -rf ../../build ../../install ../../log
	@echo "Workspace cleaned"

package:
	cd ..; colcon build
	@echo "Workspace built"

.PHONY devel:
devel:
	pip install pre-commit pip-tools
	@echo "Developer packages installed"

.PHONY requirements:
requirements:
	pip-compile requirements.in
	@echo "Python dependencies requirements.txt generated"

.PHONY python:
python:
	pip install -r requirements.txt

.PHONY dependencies:
dependencies:
	sudo apt-get install -y ros-galactic-slam-toolbox \
		ros-galactic-tf-transformations \
		ros-galactic-xacro \
		ros-galactic-nav2-bringup \
		ros-galactic-webots-ros2-driver
	@echo "ROS 2 dependencies packages installed"

all: clean python dependencies package
