**Table of Contents**
- [Installing](#installing)
  - [Downloading the code](#downloading-the-code)
  - [Useful bashrc aliases](#useful-bashrc-aliases)
  - [Creating a virtual environment](#creating-a-virtual-environment)
    - [Using Alias](#using-alias)
    - [Manual creation of virtual environment](#manual-creation-of-virtual-environment)
  - [Installing dependencies](#installing-dependencies)
    - [Using alias](#using-alias-1)
    - [Manual installation of dependencies](#manual-installation-of-dependencies)
  - [Building the code](#building-the-code)
    - [Using alias](#using-alias-2)
    - [Manual build instructions](#manual-build-instructions)
  - [Sourcing the workspace](#sourcing-the-workspace)
    - [Using alias](#using-alias-3)
    - [Manually sourcing the workspace](#manually-sourcing-the-workspace)

# Installing
## Downloading the code
This repository is dependent on three other repositories found here: https://github.com/james-swedeen/advanced_path_planning, https://github.com/james-swedeen/matplotlibcpp, and https://github.com/james-swedeen/kalman_filter.

## Useful bashrc aliases
The commands below assume that you install the following aliases in your `~/.bashrc` file. For details on installing and compiling without these aliases, see [Install.md](documentation/install.md).

Note that all of the following aliases **must be run from the workspace root**.
* *make_mav_venv*: Creates a virtual environment, installs ros dependencies, and installs the mav_sim package
    ```
    alias make_mav_venv='python3 -m venv --system-site-packages venv && source venv/bin/activate && touch venv/COLCON_IGNORE && rosdep install --from-paths src --ignore-src -r -y && pip install transforms3d && pip install shapely && pip install pandas && pip install -e src/muav_soln/mavsim_python/'
    ```
* *sd*: Used to source the workspace directory correctly when a build is not required.
    ```
    alias sd='source /opt/ros/humble/setup.bash && source venv/bin/activate && . install/setup.bash'
    ```
* *build*: Used to source the ROS2 root, activate the virtual environment (assumed to be *venv*), build, and then source the install
    ```
    alias build='source /opt/ros/humble/setup.bash && source venv/bin/activate && python3 -m colcon build --symlink-install && . install/local_setup.bash'

## Creating a virtual environment
### Using Alias
If you defined the aliases above in your .bashrc file, the creation of the virtual environment and installation of dependencies be performed with the following command (executed from the workspace base folder)
```bash
make_mav_venv
```

### Manual creation of virtual environment
You must take care when using a virtual environment with ROS2, especially in workspaces that include custom ROS2 packages. The setup must be done in a way that additional Python packages (e.g., the `mav_sim` package) can also be installed within the virtual environment.

ROS relies on a couple of Python packages that are installed in the global Python installation for parts of its build process. For that reason, we'll actually have our virtual environment be set up to have access to the system installed Python packages by including the `--system-site-packages` flag when creating it. Assuming that the virtual environment is named `venv`, the following command will create a valid virtual environment.
```bash
python3 -m venv --system-site-packages venv
```

Next, you will want to add a `COLCON_IGNORE` file in your virtual environment folder to ensure that the colcon build system does not try to compile your virtual environment. This can be done with
```bash
touch venv/COLCON_IGNORE
```


## Installing dependencies
### Using alias
If you defined the aliases above in your .bashrc file, the creation of the virtual environment and installation of dependencies be performed with the following command (executed from the workspace base folder). Note that this does *not* need to be run a second time.
```bash
make_mav_venv
```

### Manual installation of dependencies
You must also install all of the ros dependencies. This can be done using [rosdep](http://wiki.ros.org/rosdep), running the following command from the workspace root directory with the virtual environment activated.
```bash
source venv/bin/activate
rosdep install --from-paths src --ignore-src -r -y
```

Rosdep misses two packages that you must manually install. With your virtual environment activated, run the following
```bash
source venv/bin/activate
pip install transforms3d
pip install shapley
```

## Building the code
### Using alias
From the workspace root folder, run the following command.
```bash
build
```

### Manual build instructions
Prior to compiling the workspace, you must build the `mav_sim` python package. For more details, see the instructions in the [`mav_sim` README.md](../muav_soln/mavsim_python/README.md) located in the "muav_soln/mavsim_python" directory. From the root directory, the mav_sim package can be installed as
```bash
source venv/bin/activate
pip install -e src/muav_soln/mavsim_python/
```
Note that the *-e* option makes the install editable so that you can change the code without rebuilding.

You can now compile the code. From the workspace root directory, run the following
```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
python3 -m colcon build --symlink-install
colcon build --symlink-install
. install/local_setup.bash
```
Using `python3 -m colcon build ...` is important as leaving off `python3 -m` will result in *colcon* using the system's python interpreter instead of the virtual environment's interpretter. The `--symlink-install` option allows the python code to be editable without rebuild.

## Sourcing the workspace
**Each time** you open a terminal, you must source the workspace from the workspace root directory.

### Using alias
From the workspace root folder, run the following command.
```bash
sd
```

### Manually sourcing the workspace
Three things must be sources:
1. The ROS install
2. The virtual environment
3. The workspace

This can be done by executing the following commands from the workspace root folder.
```
source /opt/ros/humble/setup.bash
source venv/bin/activate
. install/setup.bash'
```
