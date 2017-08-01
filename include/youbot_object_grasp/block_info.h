#ifndef INCLUDED_BLOCK_INFO_H
#define INCLUDED_BLOCK_INFO_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>


class cBlockInfo
{
public:

	//--------------------------------------------------------------------------//
	//----------------------  CONSTRUCTION / DESTRUCTION  ----------------------//
	//--------------------------------------------------------------------------//
	
	cBlockInfo(ros::NodeHandle& nh);

	~cBlockInfo();


	//--------------------------------------------------------------------------//
	//-------------------------  INTERFACE FUNCTIONS  --------------------------//
	//--------------------------------------------------------------------------//

	bool BlockFound() const;
	bool blockDimension;

	tf::Transform GetTransformA5ToBlock() const;

	const tf::Vector3& GetBlockAlignmentPosition() const;
	const float GetBlockAlignmentRotation() const;
	const bool GetBlockDimension() const;
	const float GetFinalRotation() const;

	

private:

	//--------------------------------------------------------------------------//
	//-------------------------  SUBSCRIBER FUNCTIONS  -------------------------//
	//--------------------------------------------------------------------------//

	void BlockCallback(const geometry_msgs::Pose& pose_ASUStoBlock);
	void FloorNormalCallback( const geometry_msgs::Vector3& norm );
	void BlockAlignLocationCallback( const geometry_msgs::Point& loc );
	void BlockAlignRotationCallback ( const std_msgs::Float32& rot );
	void BlockDimensionCallback ( const std_msgs::Bool& dim );
	void FinalBlockRotationCallback(const std_msgs::Float32& rot);




	//--------------------------------------------------------------------------//
	//---------------------------  HELPER FUNCTIONS  ---------------------------//
	//--------------------------------------------------------------------------//

	tf::Transform GetTransformFromPose(const geometry_msgs::Pose& pose);

	
	//--------------------------------------------------------------------------//
	//-----------------------------  DATA MEMBERS  -----------------------------//
	//--------------------------------------------------------------------------//
	
	ros::Subscriber mBlockPoseSub;
	ros::Subscriber mFloorNormalSub;
	ros::Subscriber mRgbBlockLocSub;
	ros::Subscriber mRgbBlockRotSub;
	ros::Subscriber mRgbBlockDimSub;
	ros::Subscriber mRgbFinalRotSub;

	tf::Transform mG_A5ToAsus;
	tf::Transform mG_AsusCorrection;
	tf::Transform mG_AsusToBlock;

	tf::Vector3 mBlockAlignmentLocation;
	float mBlockAlignmentRotation;
	float mFinalRotation;

	bool mCameraCalibrated;

	bool mBlockFound;

	tf::TransformListener* mpListener;
};

#endif  // INCLUDED_BLOCK_INFO_H

