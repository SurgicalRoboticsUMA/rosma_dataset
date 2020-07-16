# Using the data of the ROSMA dataset
Open software developed under the European TERRINet program.

*Author:* Irene Rivas-Blanco

*Contact info:* irivas@uma.es

*Affiliation:* University of Malaga, [Medical Robotics Lab](https://www.uma.es/medical-robotics/cms/base/ver/base/basecontent/75284/proyectos/)

## MATLAB GUI
The MATLAB GUI for easily using the data of the ROSMA datset has been created with the tool App Designer in MATLAB R2019. 
To run the gui:
```bash
run('gui_rosma')
```

First, browse a data file from the [dataset](https://zenodo.org/record/3932964#.XwTNX3UzakA). This will generate a mat file named *data.mat* containing a struct variable with all the data of the csv file. Note: make sure that the csv file is in the MATLAB working folder.

To run the file press the play button. The cartesian position of the Master Tool Manipulators and the Patient Side Manipulators is displayed in real-time. You can pause the simulation and play it again, or stop it. 

![screenshots](https://github.com/SurgicalRoboticsUMA/dataPaper/blob/master/gui.png)

## ROS
If the checkbox *ROS* is on, the joint position of the four manipulators is published in the corresponding ROS topics to move them. Thus, if the dVRK package is running (real hardware or simulation) the manipulators will replicate the motion stored in the csv file. 

You can download the dVRK package from [https://github.com/jhu-dvrk/dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros). Then, use the configuration files of the folder *config* of this repository:
- Copy file *full_system.rviz* into /dvrk-ros/dvrk_model/rviz_config.
- Copy file *dvrk_full_rviz.launch* into /dvrk-ros/dvrk_robot/launch.
- Copy files *sawControllersPID-PSM_modified.xml* and *full_teleop_SIMULATED.json* into /cisst-saw/sawIntuitiveResearchKit/share.

Then, launch the package as:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch dvrk_robot dvrk_full_rviz.launch slave1:=PSM1 slave2:=PSM2 master1:=MTMR master2:=MTML config:=<catkin_ws path>/src/cisst-saw/sawIntuitiveResearchKit/share/full_teleop_modified_SIMULATED.json
```

![screenshots](https://github.com/SurgicalRoboticsUMA/dataPaper/blob/master/rviz.png)

