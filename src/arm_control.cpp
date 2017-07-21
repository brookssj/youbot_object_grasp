#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

#include "youbot_object_grasp/arm_interface_gazebo.h"
#include "youbot_object_grasp/arm_interface_youbot.h"
#include "youbot_object_grasp/arm_interface.h"
#include "youbot_object_grasp/base_control.h"
#include "youbot_object_grasp/block_info.h"

#include <iostream>
#include <vector>


enum ProcessState
{
	Initializing,
	WaitingForBlock,
	NavigatingToBlock,
	MovingArmToSearchPose,
	AligningToBlock,
	GraspingBlock,
	PuttingArmInCarryPose,
	//InitingReturnToStart,
	//ReturningToStart,
	PrepareForDrop,
	DropBlock,
	//ReturnToPickupPoint,
	//MoveToFinish,
	Finished
};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh("~");
    ros::Publisher mStatePub = nh.advertise<std_msgs::Int32>( "/control_current_state", 1 );
    int controllerState = 0;


	// --- Constants --- //

	const double adjustmentBaseSpeed = 0.0073;

	//--Initial Grasp Position Values--//
	/*tf::Transform mG_RightHomePose_05;
	tf::Transform mG_LeftHomePose_05;
	tf::Transform mG_RightGraspPose_05;
	tf::Transform mG_LeftGraspPose_05;
	tf::Transform translate;
	tf::Vector3 t;
	std::vector<double> mArmLeft90DegSeedVals;
	std::vector<double> mArmRight90DegSeedVals;

	translate.setIdentity();
	translate.getOrigin().setZ( 0.084 );

	tf::Quaternion q( 0.692, 0.687, -0.16, 0.154 );
	t.setValue( 0.017, -0.331, 0.059 );
	mG_RightHomePose_05.setRotation(q);
	mG_RightHomePose_05.setOrigin(t);
	mArmRight90DegSeedVals[0] =  4.56;
	mArmRight90DegSeedVals[1] = 2.04427;
	mArmRight90DegSeedVals[2]-1.51891;
	mArmRight90DegSeedVals[3] = 2.54343;
	mArmRight90DegSeedVals[4] = 2.93883;
	mG_RightGraspPose_05 = mG_RightHomePose_05 * translate;

	q.setValue( 0.704, -0.675, -0.158, -0.156 );
	t.setValue( 0.015, 0.331, 0.059 );
	mG_LeftHomePose_05.setRotation(q);
	mG_LeftHomePose_05.setOrigin(t);
	mArmLeft90DegSeedVals[0] = 1.37;
	mArmLeft90DegSeedVals[1] = 2.04427;
	mArmLeft90DegSeedVals[2] = -1.51891;
	mArmLeft90DegSeedVals[3] = 2.54343;
	mArmLeft90DegSeedVals[4] = 2.93883;
	mG_LeftGraspPose_05 = mG_LeftHomePose_05 * translate;*/
	
	
	// --- Parameters --- //
	
	bool usingGazebo = false;
	ros::param::get("/using_gazebo", usingGazebo);


	// --- TF --- //
	
	tf::TransformListener* pListener = new tf::TransformListener();
	std::cout << "Wait for 2 seconds to allow tfs to buffer" << std::endl;
	ros::Duration(2).sleep();

	tf::StampedTransform g_arm0_to_base_link;
	pListener->lookupTransform("arm_link_0", "base_link", ros::Time(0), g_arm0_to_base_link);
	

	// --- Arm Interface object --- //
	
	cArmInterface* pArmInterface;
	std::cout << "Creating arm interface." << std::endl;
	if( usingGazebo )
	{
		std::cout << "\tUsing Gazebo interface" << std::endl;
		pArmInterface = new cArmInterfaceGazebo(g_arm0_to_base_link, nh);
	}
	else
	{
		std::cout << "\tUsing youBot interface" << std::endl;
		pArmInterface = new cArmInterfaceYoubot(g_arm0_to_base_link, nh);
	}


	// --- Base Controller --- //

	cBaseControl* pBaseController = new cBaseControl(nh);


	// --- Initialization --- //

	if( usingGazebo )
	{
		// This is because sometimes when using Gazebo all of the extra stuff
		// takes a while to start up.
		
		std::cout << "Waiting 5 seconds to allow everything to start up." << std::endl;
		for( int i = 5; i > 0; --i )
		{
			std::cout << i << std::endl;
			ros::Duration(1).sleep();
		}
	}


	// --- Position Arm --- //

	std::cout << "Driving arm to camera position." << std::endl;
	pArmInterface->GoToCameraSearchPose();
	ros::Duration(3).sleep();  // Wait for the arm to get to the position.


	// --- Block Info --- //
	
	cBlockInfo* pBlockInfo = new cBlockInfo(nh);


	// --- Begin --- //

	bool graspingLeft = false;
	ProcessState currentState;
	if (controllerState == 6)
		{
			currentState = PrepareForDrop;
		} 
	else
		{	currentState = WaitingForBlock;
		}

	tf::Transform g_StartingPose_w;
	tf::Transform pickupGoal;
	int stopBaseCounter = 0;
	while(ros::ok())
	{
		switch(currentState)
		{
		case WaitingForBlock:
		{
			if( pBlockInfo->BlockFound() )
			{
				std::cout << "Found the block" << std::endl;
				// First let's save our starting position so we can return to it.
				g_StartingPose_w = pBaseController->GetCurrentWorldPosition();
				
				// Drive the youBot next to the block.  Make sure we don't rotate
				// based on the rotation matrix of the block itself.
				tf::StampedTransform g_arm0_to_arm5;
				pListener->lookupTransform("arm_link_0", "arm_link_5", ros::Time(0), g_arm0_to_arm5);
				pickupGoal = g_arm0_to_arm5 * pBlockInfo->GetTransformA5ToBlock();
				if( pickupGoal.getOrigin().getY() < 0.0 )
				{
					graspingLeft = false;
					pickupGoal.getOrigin().setY( pickupGoal.getOrigin().getY() + 0.27 );
				}
				else
				{
					graspingLeft = true;
					pickupGoal.getOrigin().setY( pickupGoal.getOrigin().getY() - 0.45 );
				}

				pickupGoal.setBasis( tf::Matrix3x3::getIdentity() );

				std::cout << "Publishing move goal" << std::endl;
				pBaseController->MoveRelativeToArmLink0( pickupGoal );
				currentState = NavigatingToBlock;
				std::cout << "Exiting the WaitingForBlock state" << std::endl;
				std::cout << std::endl;
				std::cout << "Entering NavigatingToBlock state" << std::endl;
			}
			break;
		}

			
		case NavigatingToBlock:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				currentState = MovingArmToSearchPose;
				std::cout << "Reached navigation goal." << std::endl;
				std::cout << "Exiting NavigatingToBlock state" << std::endl;
			}
			break;
		}

			
		case MovingArmToSearchPose:
		{
			std::cout << std::endl;
			std::cout << "Entering MovingArmToSearchPose state" << std::endl;
			std::cout << "Moving arm to search pose." << std::endl;

			if( graspingLeft )
			{
				pArmInterface->GoToLeftAlignPose();
			}
			else
			{
				pArmInterface->GoToRightAlignPose();
			}
			
			ros::Duration(4).sleep();  // Wait for the arm to get to the position.

			controllerState = 4;
			mStatePub.publish(controllerState);
			ros::Duration(2).sleep(); //Wait for camera to adjust to lighting
			currentState = AligningToBlock;
			std::cout << "Exiting MovingArmToSearchPose state" << std::endl;
			break;
		}

		
		case AligningToBlock:
		{
			// Target is x: 355 +/- 10, y: 375 +/- 10
			double xCmdVel = 0;
			double yCmdVel = 0;
			const tf::Vector3& finalBlockLoc = pBlockInfo->GetBlockAlignmentPosition();
			if( finalBlockLoc.getX() > 374.0 + 9.0 )
			{
				// Move towards front of base when grasping right, opposite when left
				xCmdVel = graspingLeft ? -adjustmentBaseSpeed : adjustmentBaseSpeed;
			}
			else if( finalBlockLoc.getX() < 376.0 - 3.0 )
			{
				// Move towards rear of base when grasping right, opposite when left
				xCmdVel = graspingLeft ? adjustmentBaseSpeed : -adjustmentBaseSpeed;
			}

			if( finalBlockLoc.getY() > 410.0 + 10.0 )
			{
				// Move to the left when grasping right, opposite when left
				yCmdVel = graspingLeft ? adjustmentBaseSpeed : -adjustmentBaseSpeed;
			}
			else if( finalBlockLoc.getY() < 410.0 - 10.0 )
			{
				// Move to the right when grasping right, opposite when left
				yCmdVel = graspingLeft ? -adjustmentBaseSpeed : adjustmentBaseSpeed;
			}

			bool adjustmentMade = (xCmdVel != 0) || (yCmdVel != 0);
			
			// If no adjustments were made, this should be all zeros which should stop the base.
			pBaseController->CommandBaseVelocity(xCmdVel, yCmdVel, 0);

			// This weird counter is here to force the base to stop.  Due to the face that things
			// are happening asynchronously and the base might still be moving when we send this
			// command, we are going to have a counter that makes sure we don't drift away from
			// our target by accident while we are trying to stop.
			if( !adjustmentMade && (stopBaseCounter < 10) )
			{
				// TODO:  Use a timer instead of just a simple counter.
				++stopBaseCounter;
				std::cout << finalBlockLoc.getX() << std::endl;
				std::cout << finalBlockLoc.getY() << std::endl;				
				currentState = GraspingBlock;
				std::cout << "Exiting AligningToBlock state" << std::endl;
			}
			else
			{
				stopBaseCounter = 0;
			}
			
			break;
		}

		
		case GraspingBlock:
		{
			std::cout << std::endl;
			std::cout << "Entering GraspingBlock state" << std::endl;
			std::cout << "Waiting 2 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(2).sleep();

			std::cout << "Opening grippers" << std::endl;
			pArmInterface->OpenGrippers();
			std::cout << "Waiting 4 seconds to allow grippers to open." << std::endl;
			ros::Duration(4).sleep();

			//--Rotating Grippers--//
			const float finalBlockRot = pBlockInfo->GetBlockAlignmentRotation();

			// --- Establish Goal Position --- //

			std::cout << "Reaching to grasp." << std::endl;
			if( graspingLeft )
			{
				//mArmRight90DegSeedVals[4] = finalBlockRot;
				//pArmInterface->PositionArm( mG_LeftGraspPose_05, mArmLeft90DegSeedVals );
				pArmInterface->SetRightSeedVal(4, finalBlockRot);
				std::cout << finalBlockRot << std::cout;
				pArmInterface->GoToLeftGraspPose();
			}
			else
			{
				//mArmRight90DegSeedVals[4] = finalBlockRot;
				//pArmInterface->PositionArm( mG_RightGraspPose_05, mArmRight90DegSeedVals );
				pArmInterface->SetLeftSeedVal(4, finalBlockRot);
				std::cout <<finalBlockRot << std::cout;
				pArmInterface->GoToRightGraspPose();
			}
			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3.0).sleep();

			std::cout << "Closing grippers" << std::endl;
			pArmInterface->CloseGrippers();
			std::cout << "Waiting 4 seconds to allow grippers to close." << std::endl;
			ros::Duration(4).sleep();
			
			controllerState = 0;
			mStatePub.publish(controllerState);
			currentState = PuttingArmInCarryPose;
			std::cout << "Exiting GraspingBlock state" << std::endl;
			break;
		}

		
		case PuttingArmInCarryPose:
		{
			std::cout << std::endl;
			std::cout << "Entered PuttingArmInCarryPose state" << std::endl;
			std::cout << "Putting arm back into search pose" << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftHomePose();
			}
			else
			{
				pArmInterface->GoToRightHomePose();
			}

			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3.0).sleep();
			
			
			std::cout << "Driving arm to carry pose." << std::endl;
			pArmInterface->GoToCameraSearchPose();
			std::cout << "Waiting 3 seconds to allow arm to reach pose" << std::endl;
			ros::Duration(3.0).sleep();  // Allow the arm to reach the pose.

			//CurrentState = InitiatingReturnToStart;
			controllerState=0;
			mStatePub.publish(controllerState);
			currentState = Finished;
			std::cout << "Exiting PuttingArmInCarryPose state" << std::endl;
			break;
		}

		/*
		case InitiatingReturnToStart:
		{
			std::cout << std::endl;
			std::cout << "Entering InitiatingReturnToStart state" << std::endl;
			std::cout << "Publishing goal and waiting for status to change" << std::endl;

			pBaseController->MoveToWorlPosition( g_StartingPose_w );

			currentState = ReturningToStart;
			std::cout << "Exiting InitiatingReturnToStart" << std::endl;
			std::cout << std::endl;
			std::cout << "Entering ReturningToStart state" << std::endl;

			break;
		}

		
		case ReturningToStart:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				currentState = PrepareForDrop;
				std::cout << "Exiting the ReturningToStart state" << std::endl;
				std::cout << "Entering PrepareForDrop state" << std::endl;
				std::cout << std::endl;
			}
			break;
		}
		*/
		
		case PrepareForDrop:
		{
			std::cout << "Putting arm into drop pose." << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftGraspPose();
			}
			else
			{
				pArmInterface->GoToRightGraspPose();
			}
			
			std::cout << "Waiting 3 seconds to allow arm to finish moving." << std::endl;
			ros::Duration(3).sleep();  // Wait for the arm to get to the position.

			std::cout << "Exiting PrepareForDrop state" << std::endl;
			std::cout << "Entering DropBlock state" << std::endl << std::endl;
			currentState = DropBlock;
			break;
		}

		
		case DropBlock:
		{
			std::cout << "Opening grippers" << std::endl;
			pArmInterface->OpenGrippers();
			std::cout << "Waiting 2 seconds to allow grippers to open." << std::endl;
			ros::Duration(2).sleep();			

			std::cout << "Moving arm back to get out of the way." << std::endl;
			if( graspingLeft )
			{
				pArmInterface->GoToLeftAlignPose();
			}
			else
			{
				pArmInterface->GoToRightAlignPose();
			}
			ros::Duration(2).sleep();			
			
			std::cout << "Driving arm to carry pose." << std::endl;
			pArmInterface->GoToCameraSearchPose();
			ros::Duration(2.0).sleep();  // Allow the arm to reach the pose.

			pArmInterface->CloseGrippers();
			ros::Duration(1.0).sleep();  // Allow the arm to reach the pose.

			/*std::cout << "Publishing nav goal to go back to the pickup location" << std::endl;
			tf::Transform rotate;
			rotate.setIdentity();
			rotate.getBasis().setRPY(0, 0, 3.14);
			pBaseController->MoveRelativeToArmLink0( pickupGoal * rotate );*/

			currentState = Finished;
			std::cout << "Exiting DropBlock state" << std::endl;
			//std::cout << "Entering ReturnToPickupPoint state" << std::endl;
			std::cout << std::endl;
			break;
		}

		/*
		case ReturnToPickupPoint:
		{
			if( !pBaseController->MovingToMoveBaseGoal() )
			{
				std::cout << "Exiting ReturnToPickupPoint state" << std::endl;
				currentState = Finished;
			}
			
			break;
		}
		
		*/

		default:
			break;
		}

		ros::spinOnce();
	}

	delete pListener;
	pListener = 0;

	delete pArmInterface;
	pArmInterface = 0;
	
	delete pBaseController;
	pBaseController = 0;
	
	return 0;
}

