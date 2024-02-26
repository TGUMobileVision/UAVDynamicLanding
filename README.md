# UAV Dynamic Landing
This package is the source code of UAV Dynamic Platform Landing.

Description: 

In this project, using an onboard camera, an autonomous dynamic landing system is designed with respect to a UAV platform, which contains a comprehensive framework and is integrated in an onboard computer. Specifically, pose estimation for the monocular camera is conducted based on visual-inertial localization algorithms, and target detection algorithms are utilized for landing site recognition. A pose transformation algorithm is designed and target information is conversed, and landing trajectories are generated even regenerated with respect to moving targets by motion planning with provided state information. An outerloop geometric tracking controller for the UAV is integrated to complete landing tasks by calculating required thrust and attitude. To construct a capable and extendible experimental platform, an integrated flight system is designed above the robot operating system (ROS) including the landing objective, and feasibility of the proposed method is verified by experiments for both static and dynamic landing targets.

The demonstration of experimental results is as follows：

![demonstration of experimental results](https://github.com/TGUMobileVision/UAVDynamicLanding/blob/main/FlightRecorders_48849282.gif)


We put the relevant experiment demonstration video in the paper on Youku website:https://v.youku.com/v_show/id_XNTg3NTgyNDYwOA==.html


For more technical details, please refer to:

C. An, B. Li*, W. Shi, and X. Zhang, ``Autonomous quadrotor UAV systems for dynamic platform landing with onboard sensors,” International Journal of Robotics and Automation, vol. 38, no. 4, pp. 296-305, 2023.