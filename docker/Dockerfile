FROM osrf/ros:humble-desktop

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Setup the image
RUN apt update && apt upgrade -y && apt install nano gedit python3.10-venv -y

# Copy files over to the image
WORKDIR /home/uav_sim/src
COPY muav_soln muav_soln
COPY rapidly_exploring_random_tree rapidly_exploring_random_tree
COPY scenario scenario
COPY suas_launch suas_launch

# Copy over the bashrc commands
RUN echo "alias make_mav_venv='python3 -m venv --system-site-packages venv && source venv/bin/activate && touch venv/COLCON_IGNORE && rosdep install --from-paths src --ignore-src -r -y && pip install transforms3d && pip install shapely && pip install pandas && pip install -e src/muav_soln/mavsim_python/'" >> ~/.bashrc
RUN echo "alias sd='source /opt/ros/humble/setup.bash && source venv/bin/activate && . install/setup.bash'" >> ~/.bashrc
RUN echo "alias build='source /opt/ros/humble/setup.bash && source venv/bin/activate && python3 -m colcon build --symlink-install && . install/local_setup.bash'" >> ~/.bashrc

# Build the workspace
WORKDIR /home/uav_sim
RUN python3 -m venv --system-site-packages venv && source venv/bin/activate && touch venv/COLCON_IGNORE && rosdep install --from-paths src --ignore-src -r -y && pip install transforms3d && pip install shapely && pip install -e src/muav_soln/mavsim_python/
RUN source /opt/ros/humble/setup.bash && source venv/bin/activate && python3 -m colcon build --symlink-install && . install/local_setup.bash

# Prepare workspace for immediately executing files
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source venv/bin/activate" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc