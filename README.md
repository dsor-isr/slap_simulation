# SLAP Simulation
This repository contains the ROS code USED to generate the results in the paper entitled **Target localization and pursuit with networked robotic vehicles: theory and experiments**. It holds the code stack for simulating surface and underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System and Robotics), making use of the [FAROL code base](https://github.com/dsor-isr/farol/) and the [SLAP algorithm](https://github.com/dsor-isr/slap/).

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation
- Clone this repository and its submodules to a catkin workspace:
```bash
mkdir -p ~/catkin_ws_slap_sim/src
git clone --recursive git@github.com:dsor-isr/slap_simulation.git ~/catkin_ws_slap_sim/src/.
```

- Run the installation script (note: you will require administrator priviledges)
```bash
roscd farol/farol_addons/farol_docker/
chmod u+x install_requirements.sh
./install_requirements.sh
```

### Using Farol Scripts and Alias
In order to make use of the scripts and alias created to make the development of code easier, please add the following lines to your ~/.bashrc file.
NOTE: replace '/<path_to_workspace>' with the folder where you put your catkin_ws inside. If you put in your home folder, then this variable should be left empty!

```bash
# Function to change between different catkin workspaces on the fly - this is not compulsory, but it is a nice addition 🤓

# Create a file to store the latest catkin workspace (if it does not exist) and put in the first line the default name, i.e. catkin_ws
if [ ! -f ~/.catkin_ws_config ]; then touch ~/.catkin_ws_config && echo catkin_ws > ~/.catkin_ws_config ;fi

# Set the variable CATKIN_PACKAGE with the workspace in the catkin_ws_config file
export CATKIN_PACKAGE=$(head -n 1 ~/.catkin_ws_config)

# Function to update the default catkin workspace variable and store the last setting in the file
set_catkin_ws_function() {
    #set CATKIN_PACKAGE according the an input parameter
    export CATKIN_PACKAGE=catkin_ws_$1
    echo CATKIN_PACKAGE = ${CATKIN_PACKAGE}
    
    # save into a hidden file the catkin workspace setting
    echo $CATKIN_PACKAGE > ~/.catkin_ws_config
    source ~/.bashrc
}

# This is required (to source the ROS and farol files)
source /opt/ros/noetic/setup.bash
export CATKIN_ROOT=${HOME}/<path_to_workspace>
export ROS_WORKSPACE=${CATKIN_ROOT}/${CATKIN_PACKAGE}
export FAROL_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname farol_scripts | head -n 1)
source ${FAROL_SCRIPTS}/farol_easy_alias/farol_permanent_alias/alias.sh
```

### Compile the code
- Compile the code
```bash
set_catkin_ws_function slap_sim
cd ~/<path_to_workspace>/catkin_ws_slap_sim/
catkin build
```

### Running the code
- Launch the mission scenario and all 3 vehicles (one target, mvector, and two trackers, mblack and mred). Run these commands once in different terminal kernels:
```bash
roslaunch slap_bringup start_scenario.launch gui:=false
roslaunch slap_bringup start_vehicle.launch name:=mvector
roslaunch slap_bringup start_vehicle.launch name:=mblack
roslaunch slap_bringup start_vehicle.launch name:=mred
```

### Using the PONTE Console
- Clone PONTE repo to home directory:
```bash
git clone git@github.com:dsor-isr/ponte.git ~/
```
- Open web console on `~/ponte/index.html`.
- **Import configs:** On the top selection bar, choose `Vehicle Configurations`, click `Import` and choose `ponte/configs/console_conf_slap_30.conf`.
- **Sending waypoints:** All 3 vehicles should now be visible in the webpage. By selecting one vehicle, it is possible to right-click and send waypoints to easily move the vehicles around to more suitable positions.
- **Starting PF:** On the top selection bar, select `Draw Mission -> Design New Mission` to create a Path Following mission. Select a vehicle and click `Upload mission to the vehicles` under the `Design Tools` menu.
- **Monitoring:** On the right side of the web console, loads of data is shown about the currently selected vehicle.

By now, you should have spawned all 3 vehicles. Test by sending waypoints for each of them.

### Starting SLAP Algorithm

Firstly, the mvector vehicle should perform a basic path following, such as a "lawn-mower", for example. While it starts, make sure to stop the other two vehicles (mred and mblack) on the console (by clicking the `Stop` button on the right-side of the web console) before continuing. Then, you can start running the `slap_bringup/Slap.sh` file, after which a series of commands must be inserted, as the bash script will exit and you have to execute it again for each one of them:

- start
- start_dekf
- start_cpf

This will start each action for both vehicles, and they should start performing loops around the target, the mvector vehicle. During the mission, you can also test other pursuit configurations, such as the `behind target`. Just run the `slap_bringup/Slap.sh` script again with the following command:

- set_formation
- behind_target
- 10
- 135

The value `10` represents the distance from the target and `135` is the formation angle between the two tracker vehicles. Another option of testing is setting different values for the threshold on the communication triggering errors (check paper for clarification). Either for the DEKF or the CPF itself:

- set_etc_dekf
- 10

- set_etc_cpf
- 0.1

These values are representative; try different ones for higher or lower communication rates (0/10/100 for DEKF, and 0/0.01/0.1 for CPF, for instance).

### Reference
If you find the code useful and would like to cite it, please REFERENCE the following paper:

**Target localization and pursuit with networked robotic vehicles: theory and experiments**, 

Authors: Nguyen Hung, Eduardo Cunha, Francisco Branco, Antonio Pascoal, Institute for Systems and Robotics, IST, Lisbon

The paper was accepted for publication at the Journal of Field Robotics

More information:
https://nt-hung.github.io/research/Range-based-target-localization/