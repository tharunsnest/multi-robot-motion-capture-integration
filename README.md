# multi-robot-motion-detection


"Clone This: https://github.com/tharunsnest/multi-robot-motion-capture-integration/tree/64ebf44d220f20aa666bf3746882a89ef674ec0f"


# Instructions:
    Create 2 Drive for ros: https://github.com/AutonomyLab/create_autonomy

    copy the launch file folder into your ros package.

    copy the source,waypoint and destination_robotX to scripts folder of your ros package and make them executable

    clone the create 2 drive to your workspace

    modify the launch files depending on the location of the scripts.

    destination_robotX.py has the destination coordinates for robotX. Modify it to specify it's path

    source.py updates the robotX/s_pose from gazebo.

    excecute the following on different terminals.

    roslaunch [your-package-name] robot_main.launch

    rosrun [your-package-name] source.py

    rosrun [your-package-name] destination_robotX.py

