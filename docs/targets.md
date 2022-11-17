# Makefile targets

- `make clean` - It will clean the package build folders
- `make package` - This command is building the ROS workspace
- `make devel` - Install developer tools such as `pre-commit` and `pip-tools`
- `make requirements` - Compiles the requirements.txt from requirements.in (do this only when you know what you do, both file are in version control and they pin the Python modules version)
- `make python` - Install all Python dependencies
- `make dependencies` - Install all ROS 2 based dependencies
- `make all` - Install everything and it should be ready to be used afterward
