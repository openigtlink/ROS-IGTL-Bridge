# ROS-IGTL-Bridge

Author:Tobias Frank

This ROS-Node was implemented to enable the communication between a Lego Mindstorms EV3 robot and the planning software 3D Slicer using an OpenIGTLink interface. For further information see the following websites:

- http://www.ev3dev.org/
- https://www.slicer.org/
- http://openigtlink.org/
- http://wiki.ros.org/fuerte

----------------------------------------------------------------------------------------------------------------------------------------

This ROS-Node can be used as an interface between ROS and 3D Slicer for the following workflow in a possible clinical scenario:

- obtain preoperative imaging of the patient and define the target/registration landmarks in 3D Slicer
- set up the patient on the OR table
- move the robot's end effector to the defined landmarks 
- import the measured landmarks to 3D Slicer using ROS-IGTL-Bridge
- perform a fiducial registration in 3D Slicer and transform the target points into the robot's coordinate system
- send the target points to ROS via ROS-IGTL-Bridge
- get visual feedback of the robot's movement by sending the continously refreshed robot position as a transform to 3D Slicer via ROS-IGTL-Bridge

![alt tag](https://lh3.googleusercontent.com/kCCpy9qH8fAiacAV-f6IYP3z_1epPB__bn3gF-BE_8-Qsr4jP-sL6ZsYb79v1OxkVOqunu4vY_l6I_8BWOHs0fPc8Sq70nvKx8-xG4w12chG5OfgPpX5zGGoOEINiG9RBIySIjV1U_MPj-ckborV9x4lHinX9jpSND_-2swh_XrB5wH7yuWwC5H00cTmlIHZbQ1zkA-3XFBPa_rWAGnyecysxVL9K4wGfC2uE3bKdXwmHao6qofZ1KPq-7PqhRHTzcCp6nbx7VDAhbTxSQj5u1JHLO85Uc2aQjbMQ5YRN8eKq7_-n8kNU8gXIFOr_xOxjYiBYZGOCpVIB8M17VrMzbhjPMWnZKBscNnJ4YCIC-II7q1Fc0mmtfiRoav7TE3-p-u2hzZNdJbNFIDaokmWWk8MDN2Or-JECOqoTHV5kmDApRibvrHBWpMgCgtjnUQginM7wZOMKPLdU0k723UVlnLfWWRJWdRYvYiCo_4v_k_spqkHLeCHnj0TMKMWwUNdIlau5XLzyiROxfU20hOG3dNj9BoLOGnKHwcOb8nw7d-V9gWjgIPcFXVsMHOxXM0XNBYe=w847-h635-no)

-------------------------------------------------------------------------------------------------------------------------------------

The node subsribes to the topic "reached" which provides the final position of the robot's end effector published as a two digit std_msgs::String message. The data is forwarded to 3D Slicer as igtl::PointMessage and imported by the OpenIGTLinkIF Module as a fiducial list. Received target positions from 3D Slicer are published to the topic "cmd_xyz" subsribed by the robot. The measured position of the end effector can be accessed by subscribing to the topic "msr_xyz" and is sent as a continously refreshed igtl::TransformMessage to 3D Slicer.




