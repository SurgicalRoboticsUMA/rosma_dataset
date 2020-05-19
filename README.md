# dVRK Dataset GUI - ROSMA Project
Open software developed under the European TERRINet program.

*Author:* Irene Rivas-Blanco
*Contact info:* irivas@uma.es
*Affiliation:* University of Malaga, [Medical Robotics Lab](https://www.uma.es/medical-robotics/cms/base/ver/base/basecontent/75284/proyectos/)

## Overview
This package contains a MATLAB GUI that collects data of the dVRK and a ros node that bridges between the dVRK node and the GUI. 

## Prerequisites
* ROS - Tested with Kinetic and Melodic.
* MATLAB - Tested with R2019b.
* da Vinci Research Kit (dVRK) - Tested with the dVRK in The BioRobotics Institute, Scuola Superiore Sant'Anna, Pontedera, Italy.

![screenshots](https://github.com/irivas-uma/ROSMA/blob/master/Resources/dvrk.png)


## To build

## To launch
To launch the ROS node (assuming the package is in folder *catkin_ws* in the home directory:
```bash
cd catkin_ws/
source devel/setup.bash
rosrun rosma_dataset rosma_dataset
```

To launch the GUI to manage the recording during the experiments, run the MATLAB file *rosma_gui.mlapp*.

## Recording node
This node subscribes to the kinematic topics of the dVRK, running with the [dvrk_robot package](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_robot).When a message is sent to
 */rosma/gui/filename*, all the dVRK data is stored in csv format in the path sent to the topic. The recording is stopped using the topic */rosma/gui/stop/*.

## GUI
Created with MATLAB APP Designer Toolbox (R2019b).

### Enter a new user

The *New User* option opens a new panel to enter the profile of a new user.  The data that will be stored is: alias, name, gender, age and skill level (Thechnician or surgeon).  When the user info is saved, the data is stored into the file *UsersInfo.mat*.  This file contains a table variable called UsersInfo, which contains the the following fieldsfor each user of the experiment:
* Id: numerical identifier for each user. 
* Name, Surname, Gender and Age.
* The last fields correspond with the labels of each task of the experiment.  These fields contains thenumber of repetitions the user has performed for each task.  Thus, these fields are initialize to zero andare updated each time a user performs a task.

### Perform the experiment

When you select a particular user, first, the experiment history is displayed, showing the number of trials the user has performed for each task. Then, to start the experiment, you have to choose a task, and then press *Start Recording*. The time spent in the task is shown until the end of the experiment. The number of erros made by the user during the performance is entered by hand. The behavior of the buttons of this experiment panel is as follows:

* Start Recording: Publishes the path to save the data in the topic /rosma/gui/filename. The filename is as follows: *userID_task_trial* (data is stored in an external disk, so path to this disk is added at the beggining of the filename).
For example, *U01_Clipping_05* corresponds to the fifth trial of the clipping task for the user with ID U01. 

* Stop Recording: Publishes in the topic /rosma/guide/stop to stop de data recording. 

* Save: Button to save the current data. Consequently, the variable *UsersData* is updated with the new trial of the current task, and the variable *performanceData* is updated with the performance time, number of errors and score. The score is computed as *time in seconds + penalty points*.

* Delete: Press this button if you don't want to save the current trial. Then, *UsersData* is not updated and the next trial will overwrite the csv. 
