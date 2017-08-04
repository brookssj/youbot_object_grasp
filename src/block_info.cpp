#include "youbot_object_grasp/block_info.h"
#include <iostream>
#include <std_msgs/Float32.h>


////////////////////////////////////////////////////////////////////////////////
//  Construction / Destruction
////////////////////////////////////////////////////////////////////////////////

cBlockInfo::cBlockInfo(ros::NodeHandle& nh)
{
    mBlockPoseSub = nh.subscribe( "/block_pose", 1, &cBlockInfo::BlockCallback, this );
	mFloorNormalSub = nh.subscribe( "/floor_normal", 1, &cBlockInfo::FloorNormalCallback, this );
	mRgbBlockLocSub = nh.subscribe( "/rgb_seg/block_location", 1, &cBlockInfo::BlockAlignLocationCallback, this );
	mRgbBlockRotSub = nh.subscribe( "/rgb_seg/block_rotation", 1, &cBlockInfo::BlockAlignRotationCallback, this);
	mRgbBlockDimSub = nh.subscribe( "/rgb_seg/block_dimension", 1, &cBlockInfo::BlockDimensionCallback, this);
	mRgbFinalRotSub = nh.subscribe( "/final_block_rotation", 1, &cBlockInfo::FinalBlockRotationCallback, this);
	//mStateControllerSub= nh.subscribe( "/control_current_state", 1, &cBlockInfo::StateCallback, this);
	//controllerState = 0;

	// Create the transform listener and then pause 2 seconds to allow tfs to buffer.
	mpListener = new tf::TransformListener();
	ros::Duration(2).sleep();
	
	mCameraCalibrated = true;
	mBlockFound = false;

	mG_AsusCorrection.setIdentity();

	
	// --- arm_link_5 to ASUS center --- //

	tf::Matrix3x3 rot;
	tf::Vector3 t;
	
	// Initialize the transformation for arm_link_5 -> ASUS.  The rotation will
	// later be automatically calibrated.
	rot.setValue(0.0, -1.0, 0.0,
				 1.0, 0.0, 0.0,
				 0.0, 0.0, 1.0);
	t.setValue(0.088, 0.0, 0.022);
		
	mG_A5ToAsus.setBasis(rot);
	mG_A5ToAsus.setOrigin(t);
}

////////////////////////////////////////////////////////////////////////////////

cBlockInfo::~cBlockInfo()
{
	delete mpListener;
	mpListener = 0;
}

////////////////////////////////////////////////////////////////////////////////
//  Interface Functions
////////////////////////////////////////////////////////////////////////////////

bool cBlockInfo::BlockFound() const
{
	return mBlockFound;
}

////////////////////////////////////////////////////////////////////////////////

tf::Transform cBlockInfo::GetTransformA5ToBlock() const
{
	if( !(mBlockFound && mCameraCalibrated) )
	{
		tf::Transform ret;
		ret.setIdentity();
		return ret;
	}

	std::cout << "Block pose relative to ASUS" << std::endl;
	tf::Matrix3x3 rot = mG_AsusToBlock.getBasis();
	tf::Vector3 row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	tf::Vector3 t = mG_AsusToBlock.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl;

	std::cout << std::endl;
	
	return mG_AsusCorrection * mG_AsusToBlock;
}

////////////////////////////////////////////////////////////////////////////////

const tf::Vector3& cBlockInfo::GetBlockAlignmentPosition() const
{
	return mBlockAlignmentLocation;
}

////////////////////////////////////////////////////////////////////////////////

const float cBlockInfo::GetBlockAlignmentRotation() const
{
	return mBlockAlignmentRotation;
}

const float cBlockInfo::GetFinalRotation() const
{
	return mFinalRotation;
}

////////////////////////////////////////////////////////////////////////////////
//  Subscriber Functions
////////////////////////////////////////////////////////////////////////////////

/*void cBlockInfo::StateCallback(const std_msgs::Int32& state)
{
	controllerState = state.data;
}
*/
void cBlockInfo::BlockCallback(const geometry_msgs::Pose& pose_ASUStoBlock)
{
	if( mBlockFound )
		return;
	
	std::cout << "Received block pose!" << std::endl;

	if( !mCameraCalibrated )
	{
		std::cout << "Camera not calibrated yet.  Throwing away block pose." << std::endl;
		std::cout << std::endl;
		return;
	}

	mG_AsusToBlock = GetTransformFromPose(pose_ASUStoBlock);	

	/*
	// Build a transformation matrix
	g_A0ToBlock = getArm0ToBlockTransform(pose_ASUStoBlock);

	tf::Matrix3x3 rot = g_A0ToBlock.getBasis();
	std::cout << "Block pose relative to arm_link_0" << std::endl;
	tf::Vector3 row = rot.getRow(0);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(1);
	std::cout << "R = | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	row = rot.getRow(2);
	std::cout << "    | " << row.getX() << " " << row.getY() << " " << row.getZ() << " | " << std::endl;
	std::cout << std::endl;
	tf::Vector3 t = g_A0ToBlock.getOrigin();
	std::cout << "t = < " << t.getX() << ", " << t.getY() << ", " << t.getZ() << " >" << std::endl;

	std::cout << std::endl;
	*/

	mBlockFound = true;
}

////////////////////////////////////////////////////////////////////////////////

void cBlockInfo::FloorNormalCallback( const geometry_msgs::Vector3& norm )
{
	if( mCameraCalibrated )
		return;

	// --- First Transform Everything To Base Frame --- //

	tf::Vector3 floorNormal(norm.x, norm.y, norm.z);
	std::cout << "Received floor normal" << std::endl;
	
	// --- Get Rotation Error --- //
	
	// Ultimately we want to find a quaternion such that we can transform the
	// floor normal so that it aligns with the base z axis.  To do that we
	// will find a quaternion that aligns the normal to the y-axis of the camera.
	tf::Vector3 cameraYAxis(0, 1, 0);
	if( floorNormal.getY() < 0.0 )
		cameraYAxis.setY(-1.0);

	tfScalar angle = cameraYAxis.angle(floorNormal);
	tf::Vector3 rotAxis = cameraYAxis.cross(floorNormal);
	tf::Quaternion q(rotAxis, angle);
	
	// --- Now Apply It As A Correction --- //

	mG_AsusCorrection.setIdentity();
	mG_AsusCorrection.setRotation(q);
	mG_AsusCorrection = mG_AsusCorrection.inverse();

	mCameraCalibrated = true;
}

////////////////////////////////////////////////////////////////////////////////
	
void cBlockInfo::BlockAlignLocationCallback( const geometry_msgs::Point& loc )
{
	mBlockAlignmentLocation.setX(loc.x);
	mBlockAlignmentLocation.setY(loc.y);
	mBlockAlignmentLocation.setZ(loc.z);
}

////////////////////////////////////////////////////////////////////////////////

void cBlockInfo::BlockAlignRotationCallback( const std_msgs::Float32& rot)
{
	//mBlockAlignmentRotation = rot.data;
	bool dims = GetBlockDimension();
	float pi = 3.1415926535897931;
	if (rot.data*-1 < 15 and dims == false)
	{
		mBlockAlignmentRotation = 1.40;
	}
	else if (rot.data*-1 > 80 and dims == true)
	{
		mBlockAlignmentRotation = 1.40;
	}
	else if (rot.data*-1 < 15 or rot.data*-1 > 80)
	{
		mBlockAlignmentRotation = 2.93883;
	}
	else
	{
		mBlockAlignmentRotation = (rot.data*(-1)*pi)/180;
	}
	if (mBlockAlignmentRotation < .11 or mBlockAlignmentRotation > 8)
	{
		std::cout << "Uh oh, out of range. Reported " << mBlockAlignmentRotation << " as rotation." << std::endl;
	}

}

////////////////////////////////////////////////////////////////////////////////
void cBlockInfo::FinalBlockRotationCallback(const std_msgs::Float32& rot)
{
	mFinalRotation = rot.data;
}

const bool cBlockInfo::GetBlockDimension() const
{
	return blockDimension;
}

////////////////////////////////////////////////////////////////////////////////

void cBlockInfo::BlockDimensionCallback(const std_msgs::Bool& dim)
{
	blockDimension = dim.data;
}
////////////////////////////////////////////////////////////////////////////////
//  Helper Functions
////////////////////////////////////////////////////////////////////////////////

tf::Transform cBlockInfo::GetTransformFromPose(const geometry_msgs::Pose& pose)
{
	tf::Quaternion q( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
	tf::Vector3 t( pose.position.x, pose.position.y, pose.position.z );
	
	return tf::Transform( q, t );
}

