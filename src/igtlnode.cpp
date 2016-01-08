#include <igtlnode.h>
//----------------------------------------------------------------------
igtlnode_class::igtlnode_class(int argc, char *argv[], const char* node_name)
{
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle;	
	
    CreateIGTLServer();
   	ROS_INFO("[igtlnode] igtlnode up and Running.");	
   	
	point_pub = nh->advertise<std_msgs::String>("cmd_xyz", 1000);  // command new robot position
	sub_point = nh->subscribe("reached", 1000, &igtlnode_class::robotCallback,this);  // robot reached final position
	sub_posi = nh->subscribe("msr_xyz", 1000, &igtlnode_class::robotPositionCallback,this);  // continously measured robot position
	
	points_to_slicer = vtkSmartPointer<vtkPoints>::New();
	boost::thread* thr = new boost::thread(boost::bind(&igtlnode_class::PointReceiverThread, this));  // thread to receive data from slicer
}
//----------------------------------------------------------------------
igtlnode_class::~igtlnode_class()
{
	this->socket->CloseSocket();
	delete igtlserver;
}
//----------------------------------------------------------------------
void igtlnode_class::Run()
{
    ros::spin();
}
//----------------------------------------------------------------------
void igtlnode_class::CreateIGTLServer()
{
	this->igtlserver = new IGTL_Server((this->socket));
}
//----------------------------------------------------------------------
void igtlnode_class::robotPositionCallback(const std_msgs::String::ConstPtr& msg)
{
	std::istringstream parser (msg->data);	
	igtlFloat32 point[3];
	parser 	>> point[0] >> point[1] ;
	point[2] = 0;   // 2D -> z = 0
	
	igtl::Matrix4x4 Transformation;
	igtl::IdentityMatrix(Transformation);
	
	Transformation[0][3] = point [0];
	Transformation[1][3] = point [1];
	Transformation[2][3] = point [2];
	
	igtlserver->SendTransform("RobotPosition",Transformation);
}
//----------------------------------------------------------------------
void igtlnode_class::robotCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("[igtlnode] Received Point from Robot: [%s]", msg->data.c_str());
	
	std::istringstream parser (msg->data);	
	igtlFloat32 point[3];
	parser 	>> point[0] >> point[1] ;
	point[2] = 0; // 2D -> z = 0
	
	points_to_slicer->InsertNextPoint(point);  // add point to list
	
	// Send list of 3 points for registration
	int number_of_points = 3;
	if(points_to_slicer->GetNumberOfPoints () == number_of_points)        
	{
		ROS_INFO("[igtlnode] Sending [%d] points for fiducial registration to 3D Slicer",number_of_points );
		igtlserver->SendPoints("Points",points_to_slicer);
		points_to_slicer = vtkSmartPointer<vtkPoints>::New(); // reset pointlist
	}
}
//----------------------------------------------------------------------
void igtlnode_class::PointReceiverThread()
{
	std::string datatype;

	// Get Points from Slicer
	datatype = "POINT";   
	while(1)
	{
		vtkSmartPointer<vtkPoints>  points_from_slicer = vtkSmartPointer<vtkPoints>::New();
		datatype = igtlserver->ReceiveFromSlicer(points_from_slicer,datatype);	

		// Publish Points to topic
		double *point;
		for (unsigned int i = 0; i <points_from_slicer->GetNumberOfPoints(); i ++)
		{
			point = points_from_slicer->GetPoint(i);
			std_msgs::String msg;

			std::stringstream ss;
			ss << point[0] <<" " << point[1];
			msg.data = ss.str();

			ROS_INFO("[igtlnode] Published Point from 3D Slicer: %s", msg.data.c_str());

			point_pub.publish(msg);
		}
		if(!ros::ok()) break;
	}
}
//----------------------------------------------------------------------

