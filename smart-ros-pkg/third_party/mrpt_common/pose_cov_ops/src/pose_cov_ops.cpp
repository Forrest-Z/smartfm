/*
 * pose_cov_ops.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: JLBC
 *
 */

#include "pose_cov_ops/pose_cov_ops.h"

#include <mrpt/poses.h>

using namespace mrpt::poses;
using namespace mrpt::math;

void pose_cov_ops::compose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out)
{
	CPose3D A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE),OUT(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	OUT.composeFrom(A,B);
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}
void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian  A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	const CPose3DPDFGaussian OUT = A + B;
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}
void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::Pose& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian A(UNINITIALIZED_POSE);
	CPose3D            B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	A+=B;
	mrpt_bridge::poses::mrpt2ros(A,out);
}
void pose_cov_ops::compose(const geometry_msgs::Pose& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3D            A(UNINITIALIZED_POSE);
	CPose3DPDFGaussian B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	B.changeCoordinatesReference(A);// b = a (+) b
	mrpt_bridge::poses::mrpt2ros(B,out);
}



void pose_cov_ops::inverseCompose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out)
{
	CPose3D A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE),OUT(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	OUT.inverseComposeFrom(A,B);
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian  A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B);

	const CPose3DPDFGaussian OUT = A - B;
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::Pose& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian A(UNINITIALIZED_POSE);
	CPose3D            B_mean(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A);
	mrpt_bridge::poses::ros2mrpt(b,B_mean);

	const CPose3DPDFGaussian B(B_mean, CMatrixDouble66() ); // Cov=zeros

	const CPose3DPDFGaussian OUT = A - B;
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::Pose& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3D            A_mean(UNINITIALIZED_POSE);
	CPose3DPDFGaussian B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::ros2mrpt(a,A_mean);
	mrpt_bridge::poses::ros2mrpt(b,B);

	const CPose3DPDFGaussian A(A_mean, CMatrixDouble66() ); // Cov=zeros

	const CPose3DPDFGaussian OUT = A - B;
	mrpt_bridge::poses::mrpt2ros(OUT,out);
}
