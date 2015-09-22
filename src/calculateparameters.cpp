//============================================================================
// Name        : 510Assign3.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <string>

#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
//#include <armadillo>
#include "opencv2/core/core.hpp"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"

#define PI 3.141592

using namespace cv;
using namespace std;
//using namespace arma;



//********* boost::tuple<float, float> height_equation_calculator(double tanA, double tanB, float zA, float zB)
float height_equation_calculator(double tanA, double tanB, float zA, float zB)
		{
			float a = ((tanA * zB) - (tanB * zA));
			float b = ((tanA * tanB * (zA*zA)) - (tanA * tanB * (zB*zB)));
			float c = ((tanA * (zA*zA) * zB) - (tanB * zA * (zB*zB)));

			float determinant = b*b - 4*a*c;

			if (determinant > 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);
				   float x2 = (-b - sqrt(determinant)) / (2*a);
//				   cout << "Both heights are: "<< x1 <<" and "<<x2<<endl;
//				   cout << "Taking only +ve height..."<<endl;
				   // ****** return boost::make_tuple(x1, x2);
				   if (x1 > 0 and x2 < 0){
					   return x1;
				   }
				   else if (x2 > 0 and x1 < 0){
					   return x2;
				   }
				   else{
					   cout << "\n Problem with height roots. Both -ve or both +ve !!! \n" <<endl;
				   }
			}

			else if (determinant == 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);

				   return x1;
				}
			else {
				cout << "Height Roots are Complex !!"<<endl;
			}

//			return boost::make_tuple(a+b, a-b);

		}


boost::tuple<float, float> zA_equation_calculator(double tanA, double tanB, float zB, float height)
		{
			float a = ((tanA * zB) + (tanA * tanB * height));
			float b = ((-tanB * (zB*zB)) - (tanB * (height*height)));
			float c = ((tanA * (height*height) * zB) - (tanA * tanB * height * (zB*zB)));

			float determinant = b*b - 4*a*c;

			if (determinant > 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);
				   float x2 = (-b - sqrt(determinant)) / (2*a);

				   return boost::make_tuple(x1, x2);
			}

			else if (determinant == 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);

				   return x1;
				}
			else {
				cout << "Z_A Roots are Complex !!"<<endl;
			}
}


boost::tuple<float, float> zB_equation_calculator(double tanA, double tanB, float zA, float height)
		{
			float a = ((tanB * zA) + (tanA * tanB * height));
			float b = ((-tanA * (zA*zA)) - (tanA * (height*height)));
			float c = ((tanB * (height*height) * zA) - (tanA * tanB * height * (zA*zA)));

			float determinant = b*b - 4*a*c;

			if (determinant > 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);
				   float x2 = (-b - sqrt(determinant)) / (2*a);

				   return boost::make_tuple(x1, x2);
			}

			else if (determinant == 0) {
				   float x1 = (-b + sqrt(determinant)) / (2*a);

				   return x1;
				}
			else {
				cout << "Z_B Roots are Complex !!"<<endl;
			}
}



// Function for generating random floating point numbers
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


// Function for generating Tan(Theta)
float tan_theta(float focal_len, float head, float foot)
{
	return (focal_len * (foot - head)) / (pow(focal_len, 2) + (foot*head));
}


//Function for calculating Rotation Matrix
Mat rotation_matrix(double theta)
{
	/*
	 * Calculating rotation matrix
	 */

	// VUP matrix Points opposite of Gravity
	Mat vup = Mat::zeros(1, 3, CV_32F);
	vup.at<float>(1) = 1;
//	cout << "VUP: "<< vup << endl;

	// Look-At Point ..... Where the camera is pointing (VPN)
	Mat look_at_point = Mat::zeros(1, 3, CV_32F);
	look_at_point.at<float>(1) = -1;
	look_at_point.at<float>(2) = -1;
//	cout << "Look-At Point: "<<look_at_point<<endl;
	look_at_point = look_at_point / norm(look_at_point, NORM_L2);
//	cout << "Normalized Look-At point (n): "<< look_at_point<<endl;

	// The u_axis vector
	Mat u_axis = vup.cross(look_at_point);
//	cout << "U axis: "<< u_axis<<endl;
	u_axis = u_axis / norm(u_axis, NORM_L2);
//	cout << "Normalized U axis: "<< u_axis<<endl;

	// The v_axis vector
	Mat v_axis = look_at_point.cross(u_axis);
//	cout << "V_axis (should be normalized): "<<v_axis<<endl<<endl;


//	// Where the camera is pointing (VPN)
//	Mat newZaxis = Mat::zeros(1, 3, CV_32F);
//	newZaxis.at<float>(1) = std::tan(theta * PI /180.0);
//	newZaxis.at<float>(2) = -(std::tan(theta * PI /180.0));
//	//cout << "VPN: "<<newZaxis<<endl;
//	newZaxis = newZaxis / norm(newZaxis, NORM_L2);
//	//cout << "VPN: "<<newZaxis<<endl;
//
//	// The VUP vector
//	Mat newXaxis = vup.cross(newZaxis);
//	newXaxis = newXaxis / norm(newXaxis, NORM_L2);
////	cout << ux<<endl ;
//
//	// Third axis
//	Mat newYaxis = newZaxis.cross(newXaxis);

	/*
	 * Rotation Matrix for the camera to get the new axis
	 */
	Mat rotationMatrix;
	rotationMatrix.push_back(u_axis);
	rotationMatrix.push_back(v_axis);
	rotationMatrix.push_back(look_at_point);
//	cout << "Rotation matrix: "<<endl<< rotationMatrix<<endl<<endl;

	/*
	 * Converting the Rotation Matrix into a Homogeneous Rotation Matrix
	 */
	Mat homogeneousRotationMatrix; //= Mat::eye(4,3, CV_32F);
	homogeneousRotationMatrix.push_back(u_axis);
	homogeneousRotationMatrix.push_back(v_axis);
	homogeneousRotationMatrix.push_back(look_at_point);

	Mat justarow = Mat::zeros(1, 3, CV_32F);
	homogeneousRotationMatrix.push_back(justarow);

	Mat justacolumn = Mat::zeros(4, 1, CV_32F);
	justacolumn.at<float>(3,0) = 1;

	hconcat(homogeneousRotationMatrix, justacolumn, homogeneousRotationMatrix);
//	cout << "Homogeneous Rotation matrix:"<<endl<<homogeneousRotationMatrix<<endl<<endl;
	return homogeneousRotationMatrix;
}


//Function for generating PRT Matrix
Mat prt_matrix_func(Mat homogeneousRotationMatrix, float actualCameraHeight, float focalLength)
{
	/*
		 * 4x4 matrix PRT Matrix
		 * (Projection Matrix dot product Homogeneous Rotation Matrix) dot product Translation Matrix.
		 */

		Mat PRTMatrix = Mat::zeros(4,4, CV_32F);

		PRTMatrix.at<float>(0,0) = homogeneousRotationMatrix.at<float>(0,0);

		PRTMatrix.at<float>(1,1) = homogeneousRotationMatrix.at<float>(1,1);
		PRTMatrix.at<float>(1,2) = homogeneousRotationMatrix.at<float>(1,2);
		PRTMatrix.at<float>(1,3) = (homogeneousRotationMatrix.at<float>(1,1)) * (-(actualCameraHeight));

		PRTMatrix.at<float>(2,1) = homogeneousRotationMatrix.at<float>(2,1);
		PRTMatrix.at<float>(2,2) = homogeneousRotationMatrix.at<float>(2,2);
		PRTMatrix.at<float>(2,3) = (homogeneousRotationMatrix.at<float>(2,1)) * (-(actualCameraHeight));

		PRTMatrix.at<float>(3,1) = homogeneousRotationMatrix.at<float>(2,1) / focalLength;
		PRTMatrix.at<float>(3,2) = homogeneousRotationMatrix.at<float>(2,2) / focalLength;
		PRTMatrix.at<float>(3,3) = ((homogeneousRotationMatrix.at<float>(2,1)) * (-(actualCameraHeight))) / focalLength;

//		cout << "PRT Matrix: "<< endl<<PRTMatrix<<endl<<endl;
		return PRTMatrix;
}


//Function for generating coordinates of a person's HEAD
Mat head_location(float personHeight, float distance)
{
	/*
	 * Actual location of the HEAD of a person at FIRST DISTANCE
	 */
	Mat head_location = Mat::ones(1,4, CV_32F);
	head_location.at<float>(0,0) = 0;
	head_location.at<float>(0,1) = personHeight;
	head_location.at<float>(0,2) = distance;

	return head_location;
}


//Function for generating coordinates of a person's FOOT
Mat foot_location(float distance)
{
	/*
	 * Actual location of the FOOT of a person at FIRST DISTANCE
	 */
	Mat foot_location = Mat::ones(1,4, CV_32F);
	foot_location.at<float>(0,0) = 0;
	foot_location.at<float>(0,1) = 0;
	foot_location.at<float>(0,2) = distance;

	return foot_location;
}


/*
 * Multiplying PRT matrix with Actual locations to get the
 * projected locations on the image plane
*/
//Function for generating coordinates of a person's PROJECTED HEAD location on the image plane
Mat projected_head_location(Mat prt_matrix, Mat headX_distX)
{
/*
 * Projected location of HEAD at FIRST distance
 */
Mat projected_head_location = Mat::ones(1,4, CV_32F);
projected_head_location.at<float>(0,0) = (prt_matrix.row(0)).dot(headX_distX);
projected_head_location.at<float>(0,1) = (prt_matrix.row(1)).dot(headX_distX);
projected_head_location.at<float>(0,2) = (prt_matrix.row(2)).dot(headX_distX);
projected_head_location.at<float>(0,3) = (prt_matrix.row(3)).dot(headX_distX);
projected_head_location = projected_head_location / projected_head_location.at<float>(0,3);

return projected_head_location;
}


//Function for generating coordinates of a person's PROJECTED FOOT location on the image plane
Mat projected_foot_location(Mat prt_matrix, Mat footX_distX)
{
/*
 * Projected location of FOOT at FIRST distance
 */
Mat projected_foot_location = Mat::ones(1,4, CV_32F);
projected_foot_location.at<float>(0,0) = (prt_matrix.row(0)).dot(footX_distX);
projected_foot_location.at<float>(0,1) = (prt_matrix.row(1)).dot(footX_distX);
projected_foot_location.at<float>(0,2) = (prt_matrix.row(2)).dot(footX_distX);
projected_foot_location.at<float>(0,3) = (prt_matrix.row(3)).dot(footX_distX);
projected_foot_location = projected_foot_location / projected_foot_location.at<float>(0,3);

return projected_foot_location;
}

boost::tuple<float, float> person_height(float height, float tan_theta_A, float tan_theta_B, float za, float zb)
{
	float person_height_at_za = (tan_theta_A*((za*za) + (height*height))) / (za + (tan_theta_A * height));
	float person_height_at_zb = (tan_theta_B*((zb*zb) + (height*height))) / (zb + (tan_theta_B * height));

//	cout << za<<" "<<zb<<" "<< person_height_at_za<<" "<<person_height_at_zb<<endl;
	return boost::make_tuple(person_height_at_za, person_height_at_zb);
}

int main()
{
//*********
//88888888888

//	float focalLength = 0.035;
//	float actualCameraHeight = 10;
//	float firstDistance = -0.5;
//	float secondDistance = -1.0;
//	double theta = -45;
//
//		double firstPersonHeight = 1.58; // Height 5.2 feet
//		double secondPersonHeight = 1.82; // height 6 feet
//		double thirdPersonHeight = 1.79; // height 5.9 feet
//
//	Mat rotation_matirx = rotation_matrix(theta);
//	Mat prt_matrix = prt_matrix_func(rotation_matirx, actualCameraHeight, focalLength);
//
//	std::fstream syntheticdata;
//	syntheticdata.open("src/Synthetic_Data.txt", std::fstream::out | std::fstream::trunc);
//
//	//syntheticdata << "Cam_Ht" <<"\t"<< "Opti_Ax" <<"\t"<< "Z_a" <<"\t"<< "Z_b" <<"\t"<< "1st_ht"
//	//		<<"\t"<< "2nd_Ht" <<"\t"<< "3rd_ht" <<"\t"<< "H1_Za" <<"\t"<< "F1_Za"
//	//		<<"\t"<< "H1_Zb" <<"\t"<< "F1_Zb"<<"\t"<< "H2_Za"
//	//		<<"\t"<< "F2_Za"<<"\t"<< "H2_Zb"<<"\t"<< "F2_Zb" << endl;
//			//<<" "<< "H3_Za"<<" "<< "F3_Za"<<" "<< "H3_Zb"
//			//<<" "<< "F3_Zb"<<endl;
//
//	for (int i=0; i<100; i++)
//	{
//
////	float actualCameraHeight = rand() % 14 + 2;
////	float firstDistance = rand() % 16 + 2;
////	float secondDistance = rand() % 14 + 12;
//
////	float firstPersonHeight = 1.85, secondPersonHeight = 2.20;
//
//		// Fixed variables
////		float focalLength = 0.035;
////		float actualCameraHeight = 8;
////		float firstDistance = -10;
////		float secondDistance = -15;
////		double theta = 45;
//
//
//		firstDistance += -0.5;
//		secondDistance += -0.5;
//
//
////	cout << fRand(1,3)<<endl;
////	double firstPersonHeight = fRand(1,1.5);
//
////!!!	double firstPersonHeight = 1.42;
////!!!	double secondPersonHeight = fRand(1,2);
////!!!	double thirdPersonHeight = fRand(1.5,2.5);
//
////	cout << firstPersonHeight <<endl<< secondPersonHeight<<endl;
//
//	// Camera is rotated 45 degrees so that it is pointed towards the ground.
////	double theta = rand() % 56 + 25;
////
////	firstDistance = 0 - firstDistance;
////	secondDistance = 0 - secondDistance;
//
//
////	cout << actualCameraHeight <<","<< firstDistance <<","<< secondDistance <<","<< firstPersonHeight <<","<< secondPersonHeight <<","<< theta <<endl;
////	cout << "This program will estimate the height of the camera and the distance of the person from that camera."<<endl;
////
////	cout << "Assumptions:"
////			"\n1) Camera will be rotated anti-clock wise by 45 degrees along -ve x-axis."
////			"\n2) Height of the person is 2 meters."
////			"\n3) Focal length of camera is 35mm(or 0.035 m)."<<endl;
////
////	cout << "Enter height of camera (always +ve): ";
////	cin >> actualCameraHeight;
////
////	cout << "Enter 1st distance in front of camera (always -ve): ";
////	cin >> firstDistance;
////	firstDistance = -firstDistance;
//
////	cout <<"Enter 2nd distance in front of camera (always -ve): ";
////	cin >> secondDistance;
////	secondDistance = -secondDistance;
//
//
//
////			/*
////			 * Calculating rotation matrix
////			 */
////
////			// VUP matrix Points opposite of Gravity
////			Mat antiGravityVector = Mat::zeros(1, 3, CV_32F);
////			antiGravityVector.at<float>(1) = 1;
////			//cout << VUP <<endl;
////
////			// Where the camera is pointing (VPN)
////			Mat newZaxis = Mat::zeros(1, 3, CV_32F);
////			//VPN.at(0) = 0;
////			newZaxis.at<float>(1) = std::tan(theta * PI /180.0);
////			newZaxis.at<float>(2) = -(std::tan(theta * PI /180.0));
////			newZaxis = newZaxis / norm(newZaxis, NORM_L2);
////		//	cout << nx << endl;
////
////			// The VUP vector
////			//Mat ux = Mat::zeros(1, 3, CV_32F);
////			Mat newXaxis = antiGravityVector.cross(newZaxis);
////			newXaxis = newXaxis / norm(newXaxis, NORM_L2);
////		//	cout << ux<<endl ;
////
////			// Third axis
////			Mat newYaxis = newZaxis.cross(newXaxis);
////		//	cout << vx<<endl;
////		//	cout << norm(vx, NORM_L2);
////		//	cout << ux<<endl<<vx<<endl<<nx;
////
////			/*
////			 * Rotation Matrix for the camera to get the new axis
////			 */
////			Mat rotationMatrix;
////			rotationMatrix.push_back(newXaxis);
////			rotationMatrix.push_back(newYaxis);
////			rotationMatrix.push_back(newZaxis);
////
////			/*
////			 * Converting the Rotation Matrix into a Homogeneous Rotation Matrix
////			 */
////			Mat homogeneousRotationMatrix; //= Mat::eye(4,3, CV_32F);
////			homogeneousRotationMatrix.push_back(newXaxis);
////			homogeneousRotationMatrix.push_back(newYaxis);
////			homogeneousRotationMatrix.push_back(newZaxis);
////
////			Mat justarow = Mat::zeros(1, 3, CV_32F);
////			homogeneousRotationMatrix.push_back(justarow);
////
////			Mat justacolumn = Mat::zeros(4, 1, CV_32F);
////			justacolumn.at<float>(3,0) = 1;
////
////			hconcat(homogeneousRotationMatrix, justacolumn, homogeneousRotationMatrix);
//
////	cout << homogeneousRotationMatrix<<endl;
//
////				/*
////				 * 4x4 matrix PRT Matrix
////				 * (Projection Matrix dot product Homogeneous Rotation Matrix) dot product Translation Matrix.
////				 */
////
////				Mat PRTMatrix = Mat::zeros(4,4, CV_32F);
////
////				PRTMatrix.at<float>(0,0) = homogeneousRotationMatrix.at<float>(0,0);
////				PRTMatrix.at<float>(1,1) = homogeneousRotationMatrix.at<float>(1,1);
////				PRTMatrix.at<float>(1,2) = homogeneousRotationMatrix.at<float>(1,2);
////				PRTMatrix.at<float>(1,3) = (homogeneousRotationMatrix.at<float>(1,1)) * (-(actualCameraHeight));
////
////				PRTMatrix.at<float>(2,1) = homogeneousRotationMatrix.at<float>(2,1);
////				PRTMatrix.at<float>(2,2) = homogeneousRotationMatrix.at<float>(2,2);
////				PRTMatrix.at<float>(2,3) = (homogeneousRotationMatrix.at<float>(2,1)) * (-(actualCameraHeight));
////
////				PRTMatrix.at<float>(3,1) = homogeneousRotationMatrix.at<float>(2,1) / focalLength;
////				PRTMatrix.at<float>(3,2) = homogeneousRotationMatrix.at<float>(2,2) / focalLength;
////				PRTMatrix.at<float>(3,3) = ((homogeneousRotationMatrix.at<float>(2,1)) * (-(actualCameraHeight))) / focalLength;
//
////	Mat homogeneousPRTMatrix = PRTMatrix.clone();
////	homogeneousPRTMatrix.row(1) = homogeneousPRTMatrix.row(1) / homogeneousPRTMatrix.at<float>(1,3);
////	homogeneousPRTMatrix.row(2) = homogeneousPRTMatrix.row(2) / homogeneousPRTMatrix.at<float>(2,3);
////	homogeneousPRTMatrix.row(3) = homogeneousPRTMatrix.row(3) / homogeneousPRTMatrix.at<float>(3,3);
////	cout << PRTMatrix <<endl;
//
//
//	/*
//	 * Actual location of the HEAD of a person at FIRST DISTANCE
//	 */
//	//			Mat head1_dist1 = Mat::ones(1,4, CV_32F);
//	//			head1_dist1.at<float>(0,0) = 0;
//	//			head1_dist1.at<float>(0,1) = firstPersonHeight;
//	//			head1_dist1.at<float>(0,2) = firstDistance;
//
//	/*
//	 * Actual location of the FOOT of a person at FIRST DISTANCE
//	 */
////				Mat foot1_dist1 = Mat::ones(1,4, CV_32F);
////				foot1_dist1.at<float>(0,0) = 0;
////				foot1_dist1.at<float>(0,1) = 0;
////				foot1_dist1.at<float>(0,2) = firstDistance;
//
//
//	Mat head1_dist1 = head_location(firstPersonHeight, firstDistance);
//	Mat foot1_dist1 = foot_location(firstDistance);
//
////	if (firstDistance == -8)
////	{
////		cout << head1_dist1<< "   "<<foot1_dist1<<endl;
////	}
//
////	cout << head1_dist1.size();
//
//	/*
//	 * Actual location of the HEAD of a person at SECOND DISTANCE
//	 */
////					Mat head1_dist2 = Mat::ones(1,4, CV_32F);
////					head1_dist2.at<float>(0,0) = 0;
////					head1_dist2.at<float>(0,1) = firstPersonHeight;
////					head1_dist2.at<float>(0,2) = secondDistance;
//
//	/*
//	 * Actual location of the FOOT of a person at SECOND DISTANCE
//	 */
////					Mat foot1_dist2 = Mat::ones(1,4, CV_32F);
////					foot1_dist2.at<float>(0,0) = 0;
////					foot1_dist2.at<float>(0,1) = 0;
////					foot1_dist2.at<float>(0,2) = secondDistance;
//
//	Mat head1_dist2 = head_location(firstPersonHeight, secondDistance);
//	Mat foot1_dist2 = foot_location(secondDistance);
//
////	cout << head1_dist1<<endl;
////	cout << foot1_dist1<<endl;
////	cout << head1_dist2<<endl;
////	cout << foot1_dist2<<endl;
//
//	/*
//		 * Actual location of the HEAD of a person at FIRST DISTANCE
//		 */
////						Mat head2_dist1 = Mat::ones(1,4, CV_32F);
////						head2_dist1.at<float>(0,0) = 0;
////						head2_dist1.at<float>(0,1) = secondPersonHeight;
////						head2_dist1.at<float>(0,2) = firstDistance;
//
//		/*
//		 * Actual location of the FOOT of a person at FIRST DISTANCE
//		 */
////						Mat foot2_dist1 = Mat::ones(1,4, CV_32F);
////						foot2_dist1.at<float>(0,0) = 0;
////						foot2_dist1.at<float>(0,1) = 0;
////						foot2_dist1.at<float>(0,2) = firstDistance;
//
//		Mat head2_dist1 = head_location(secondPersonHeight, firstDistance);
//		Mat foot2_dist1 = foot_location(firstDistance);
//
//		/*
//		 * Actual location of the HEAD of a person at SECOND DISTANCE
//		 */
////						Mat head2_dist2 = Mat::ones(1,4, CV_32F);
////						head2_dist2.at<float>(0,0) = 0;
////						head2_dist2.at<float>(0,1) = secondPersonHeight;
////						head2_dist2.at<float>(0,2) = secondDistance;
//
//		/*
//		 * Actual location of the FOOT of a person at SECOND DISTANCE
//		 */
////						Mat foot2_dist2 = Mat::ones(1,4, CV_32F);
////						foot2_dist2.at<float>(0,0) = 0;
////						foot2_dist2.at<float>(0,1) = 0;
////						foot2_dist2.at<float>(0,2) = secondDistance;
//
//			Mat head2_dist2 = head_location(secondPersonHeight, secondDistance);
//			Mat foot2_dist2 = foot_location(secondDistance);
//
//
//			/*
//			 * Actual location of the HEAD and FOOT of a person at First and Second DISTANCE
//			 */
//			Mat head3_dist1 = head_location(thirdPersonHeight, firstDistance);
//			Mat foot3_dist1 = foot_location(firstDistance);
//			Mat head3_dist2 = head_location(thirdPersonHeight, secondDistance);
//			Mat foot3_dist2 = foot_location(secondDistance);
//	/*
//	 * Multiplying PRT matrix with Actual locations to get the
//	 * projected locations on the image plane
//	*/
//
//	/*
//	 * Projected location of HEAD at FIRST distance
//	 */
////					Mat head1_dist1_projection = Mat::ones(1,4, CV_32F);
////					head1_dist1_projection.at<float>(0,0) = (prt_matrix.row(0)).dot(head1_dist1);
////					head1_dist1_projection.at<float>(0,1) = (prt_matrix.row(1)).dot(head1_dist1);
////					head1_dist1_projection.at<float>(0,2) = (prt_matrix.row(2)).dot(head1_dist1);
////					head1_dist1_projection.at<float>(0,3) = (prt_matrix.row(3)).dot(head1_dist1);
////
////					head1_dist1_projection = head1_dist1_projection / head1_dist1_projection.at<float>(0,3);
//
////	cout << projectedFirstPersonHead1<<endl;
//
//
//			Mat head1_dist1_projection = projected_head_location(prt_matrix, head1_dist1);
//			Mat foot1_dist1_projection = projected_foot_location(prt_matrix, foot1_dist1);
//
////			if (firstDistance == -8)
////			{
////				cout << head1_dist1_projection<< "   "<<foot1_dist1_projection<<endl;
////				cout << prt_matrix<<endl;
////			}
//
//
//	/*
//	 * Projected location of FOOT at FIRST distance
//	 */
////				Mat projectedFirstPersonFoot1 = Mat::ones(1,4, CV_32F);
////				projectedFirstPersonFoot1.at<float>(0,0) = (prt_matrix.row(0)).dot(foot1_dist1);
////				projectedFirstPersonFoot1.at<float>(0,1) = (prt_matrix.row(1)).dot(foot1_dist1);
////				projectedFirstPersonFoot1.at<float>(0,2) = (prt_matrix.row(2)).dot(foot1_dist1);
////				projectedFirstPersonFoot1.at<float>(0,3) = (prt_matrix.row(3)).dot(foot1_dist1);
////
////				projectedFirstPersonFoot1 = projectedFirstPersonFoot1 / projectedFirstPersonFoot1.at<float>(0,3);
//
////	cout << projectedFirstPersonFoot1<<endl;
//
//
//	/*
//	 * Projected location of HEAD at SECOND distance
//	 */
////				Mat projectedFirstPersonHead2 = Mat::ones(1,4, CV_32F);
////				projectedFirstPersonHead2.at<float>(0,0) = (prt_matrix.row(0)).dot(head1_dist2);
////				projectedFirstPersonHead2.at<float>(0,1) = (prt_matrix.row(1)).dot(head1_dist2);
////				projectedFirstPersonHead2.at<float>(0,2) = (prt_matrix.row(2)).dot(head1_dist2);
////				projectedFirstPersonHead2.at<float>(0,3) = (prt_matrix.row(3)).dot(head1_dist2);
////
////				projectedFirstPersonHead2 = projectedFirstPersonHead2 / projectedFirstPersonHead2.at<float>(0,3);
//
////	cout << projectedFirstPersonHead2<<endl;
//
//
//			Mat head1_dist2_projection = projected_head_location(prt_matrix, head1_dist2);
//			Mat foot1_dist2_projection = projected_foot_location(prt_matrix, foot1_dist2);
//	/*
//	 * Projected location of FOOT at SECOND distance
//	 */
////				Mat projectedFirstPersonFoot2 = Mat::ones(1,4, CV_32F);
////				projectedFirstPersonFoot2.at<float>(0,0) = (prt_matrix.row(0)).dot(foot1_dist2);
////				projectedFirstPersonFoot2.at<float>(0,1) = (prt_matrix.row(1)).dot(foot1_dist2);
////				projectedFirstPersonFoot2.at<float>(0,2) = (prt_matrix.row(2)).dot(foot1_dist2);
////				projectedFirstPersonFoot2.at<float>(0,3) = (prt_matrix.row(3)).dot(foot1_dist2);
////
////				projectedFirstPersonFoot2 = projectedFirstPersonFoot2 / projectedFirstPersonFoot2.at<float>(0,3);
//
////	cout << projectedFirstPersonFoot2 <<endl;
//
//
//			Mat head2_dist1_projection = projected_head_location(prt_matrix, head2_dist1);
//			Mat foot2_dist1_projection = projected_foot_location(prt_matrix, foot2_dist1);
//
//			Mat head2_dist2_projection = projected_head_location(prt_matrix, head2_dist2);
//			Mat foot2_dist2_projection = projected_foot_location(prt_matrix, foot2_dist2);
//
//
//			Mat head3_dist1_projection = projected_head_location(prt_matrix, head3_dist1);
//			Mat foot3_dist1_projection = projected_foot_location(prt_matrix, foot3_dist1);
//
//			Mat head3_dist2_projection = projected_head_location(prt_matrix, head3_dist2);
//			Mat foot3_dist2_projection = projected_foot_location(prt_matrix, foot3_dist2);
//
////	cout << projectedFirstPersonHead1<<endl;
////	cout << projectedFirstPersonFoot1<<endl;
////	cout << projectedFirstPersonHead2<<endl;
////	cout << projectedFirstPersonFoot2<<endl;
////
////	/*
////	 * Calculating tan(Theta) {the angle suspended between the HEAD and the FOOT of a person} for both distances.
////	 */
////	float tanTheta1 = focalLength();
//
//	syntheticdata <<focalLength<<" "<<actualCameraHeight <<"  "<< theta <<"  "<< firstDistance <<"  "<< secondDistance <<"  "<< firstPersonHeight
//			<<"  "<< secondPersonHeight <<"  "<< thirdPersonHeight <<"  "<< head1_dist1_projection.at<float>(0,1) <<"  "<< foot1_dist1_projection.at<float>(0,1)
//			<<"  "<< head1_dist2_projection.at<float>(0,1) <<"  "<< foot1_dist2_projection.at<float>(0,1)<<"  "<< head2_dist1_projection.at<float>(0,1)
//			<<"  "<< foot2_dist1_projection.at<float>(0,1)<<"  "<< head2_dist2_projection.at<float>(0,1)<<"  "<< foot2_dist2_projection.at<float>(0,1)
//			<<" "<< head3_dist1_projection.at<float>(0,1)<<" "<< foot3_dist1_projection.at<float>(0,1)<<" "<< head3_dist2_projection.at<float>(0,1)
//			<<" "<< foot3_dist2_projection.at<float>(0,1)<<endl;
////	 syntheticdata <<
////	syntheticdata<<endl;
//
//	}
//	syntheticdata.close();

//********//
//888888888

	/*
	 * Now we go forward and use projections of head and foot as the starting point.
	 */

	std::fstream height_za_zb;
	height_za_zb.open("src/height_za_zb.txt", std::fstream::out | std::fstream::trunc);

	std::ifstream synthetic_data_file("src/Synthetic_Data.txt");
	std::string line;

		while(std::getline(synthetic_data_file,line))
		{
			stringstream instr(line);
//			vector<string> fileContents;
			float focal_length, camera_height, theta,
					z_A, z_B,
					height_1, height_2, height_3,
					head1_dist1, foot1_dist1, head1_dist2, foot1_dist2,
					head2_dist1, foot2_dist1, head2_dist2, foot2_dist2,
					head3_dist1, foot3_dist1, head3_dist2, foot3_dist2;

			instr >> focal_length; instr >> camera_height; instr>> theta;
			instr >> z_A; instr >> z_B;
			instr >> height_1; instr >> height_2; instr >> height_3;
			instr >> head1_dist1; instr >> foot1_dist1; instr >> head1_dist2; instr >> foot1_dist2;
			instr >> head2_dist1; instr >> foot2_dist1; instr >> head2_dist2; instr >> foot2_dist2;
			instr >> head3_dist1; instr >> foot3_dist1; instr >> head3_dist2; instr >> foot3_dist2;


//			cout <<focal_length << " "<<camera_height<<" "<<theta<<" "<<z_A<<" "<<z_B<<" "<<height_1<<" "<<head1_dist1<<" "<<foot1_dist1<<" "<<head1_dist2<<" "<<foot1_dist2<<endl<<endl;

			// For 1st person
			float tan_theta_1A = tan_theta(focal_length, head1_dist1, foot1_dist1);
			float tan_theta_1B = tan_theta(focal_length, head1_dist2, foot1_dist2);
//			cout << "Focal: "<<focal_length<<", head1 dist1: "<< head1_dist1<<", foot1 dist1: "<< foot1_dist1<<", theta b/w them: "<< tan_theta_1A <<endl;

			// For 2nd person
			double tan_theta_2A = tan_theta(focal_length, head2_dist1, foot2_dist1);
			double tan_theta_2B = tan_theta(focal_length, head2_dist2, foot2_dist2);

			// For 3rd person
			double tan_theta_3A = tan_theta(focal_length, head3_dist1, foot3_dist1);
			double tan_theta_3B = tan_theta(focal_length, head3_dist2, foot3_dist2);

//							double a = ((tan_theta_1A * z_B) - (tan_theta_1B * z_A));
//							double b = ((tan_theta_1A * tan_theta_1B * (z_A*z_A)) - (tan_theta_1A * tan_theta_1B * (z_B*z_B)));
//							double c = ((tan_theta_1A * (z_A*z_A) * z_B) - (tan_theta_1B * z_A * (z_B*z_B)));
//
//							double determinant = b*b - 4*a*c;
//
//							if (determinant > 0) {
//								   double x1 = (-b + sqrt(determinant)) / (2*a);
//								   double x2 = (-b - sqrt(determinant)) / (2*a);
//							}
//
//							else if (determinant == 0) {
//								   double x1 = (-b + sqrt(determinant)) / (2*a);
//								}
//							else {
//								cout << "Roots are Complex !!"<<endl;
//							}


//							cout << "Ta = "<< tan_theta_person_1_dist_1<<endl;
//							cout << "Tb = "<< tan_theta_person_1_dist_2<<endl;
//							cout << "a = "<< a <<endl;
//							cout << "b = "<< b <<endl;
//							cout << "c = "<< c <<endl;
//
							// Height roots
							//float height1, height2;// = (-b + sqrt(pow(b,2) - 4 * a * c)) / 2 * a;
//							float person2_height1, person2_height2;// = (-b - sqrt(pow(b,2) - 4 * a * c)) / 2 * a;

							// Two distance roots for each height


//
//							cout << "h1 = "<<h1<<endl;
//							cout << "h2 = "<<h2<<endl;

			// Heights from 1st person at distance 'A' and 'B'
			// New and Bad estimate of z_A and z_B
			//z_A = -12; z_B = -5;
			float positive_height;
			positive_height = height_equation_calculator(tan_theta_1A, tan_theta_1B, z_A, z_B);
//			cout << "For bad estimated z_A "<< z_A << " and estimated z_B "<< z_B <<endl;
//			cout << "Positive Height: "<< positive_height<<endl;
//			cout << endl;
/******
			//boost::tie(height1,height2) = height_equation_calculator(tan_theta_1A, tan_theta_1B, z_A, z_B);
			//cout << "h1 root from 1st person= "<<height1<<endl;
			//cout << "h2 root from 1st person= "<<height2<<endl;
			cout <<endl;
*******/

//			// Heights from 2nd person at distance 'A' and 'B'
//			boost::tie(height1,height2) = height_equation_calculator(tan_theta_2A, tan_theta_2B, z_A, z_B);
//										cout << "h1 root from 2nd person= "<<height1<<endl;
//										cout << "h2 root from 2nd person= "<<height2<<endl;
//										cout <<endl;

			// ------------------------------------------------------------
			float height_DistA_root1,height_DistA_root2; //Height2_DistA_root1,Height2_DistA_root2;
			// For height1, calculating 2 roots of Dist 'A'
			//boost::tie(Height1_DistA_root1, Height1_DistA_root2) = zA_equation_calculator(tan_theta_1A, tan_theta_1B, z_B, height1);
			//cout << "For h1 = "<<height1<<" :"<<endl;
//			cout << "DistA_Height1_root1 = "<<Height1_DistA_root1<<endl;
//			cout << "DistA_Height1_root2 = "<<Height1_DistA_root2<<endl;
			boost::tie(height_DistA_root1, height_DistA_root2) = zA_equation_calculator(tan_theta_1A, tan_theta_1B, z_B, positive_height);
//			cout << "For Height(+ve) = "<<positive_height<<" :"<<endl;
//			cout << "Distance A root 1(height_DistA_root1) = "<<height_DistA_root1<<endl;
//			cout << "Distance A root 2(height_DistA_root2) = "<<height_DistA_root2<<endl;
//			cout <<endl;


/*******	// For height2, calculating 2 roots of Dist 'A'
			boost::tie(Height2_DistA_root1, Height2_DistA_root2) = zA_equation_calculator(tan_theta_1A, tan_theta_1B, z_B, height2);
										cout << "For h2 = "<<height2<<" :"<<endl;
										cout << "DistA_Height2_root1 = "<<Height2_DistA_root1<<endl;
										cout << "DistA_Height2_root2 = "<<Height2_DistA_root2<<endl;
										cout << endl;
			// ------------------------------------------------------------------
********/

/*******	// For each height and each root of Dist 'A', get Dist 'B'
			boost::tie(Height1_DistA_1_root1, Height1_DistA_1_root2) = zB_equation_calculator(tan_theta_1A, tan_theta_1B, Height1_DistA_root1, height1);
			cout << "For h1 = "<<height1<<" and Height1_DistA_root1 = "<<Height1_DistA_root1<<endl;
			cout << "Height1_DistA_1_root1 = "<<Height1_DistA_1_root1<<endl;
			cout << "Height1_DistA_1_root2 = "<<Height1_DistA_1_root2<<endl;
			cout <<endl;
********/
			float heightandDistA1_DistB_root1,heightandDistA1_DistB_root2;
			boost::tie(heightandDistA1_DistB_root1, heightandDistA1_DistB_root2) = zB_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root1, positive_height);
//			cout << "Height(+ve) = "<<positive_height<<" and with Distance A root 1 = "<<height_DistA_root1<<endl;
//			cout << "Height with Distance A root 1, Distance B root1 (heightandDistA1_DistB_root1) = "<<heightandDistA1_DistB_root1<<endl;
//			cout << "Height with Distance A root 1, Distance B root2 (heightandDistA1_DistB_root2) = "<<heightandDistA1_DistB_root2<<endl;
//			cout << endl;

/*******
			float heightandDistA2_DistB_root1,heightandDistA2_DistB_root2;
			boost::tie(Height1_DistA_2_root1, Height1_DistA_2_root2) = zB_equation_calculator(tan_theta_1A, tan_theta_1B, Height1_DistA_root2, height1);
			cout << "For h1 = "<<height1<<" and DistA_Height1_root2 = "<<Height1_DistA_root2<<endl;
			cout << "Height1_DistA_2_root1 = "<<Height1_DistA_2_root1<<endl;
			cout << "Height1_DistA_2_root2 = "<<Height1_DistA_2_root2<<endl;
			cout <<endl;
********/
			float heightandDistA2_DistB_root1,heightandDistA2_DistB_root2;
			boost::tie(heightandDistA2_DistB_root1, heightandDistA2_DistB_root2) = zB_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root2, positive_height);
//			cout << "Height(+ve) = "<<positive_height<<" and with Distance A root 2 = "<<height_DistA_root2<<endl;
//			cout << "Height with Distance A root 2, Distance B root2 (heightandDistA2_DistB_root1) = "<<heightandDistA2_DistB_root1<<endl;
//			cout << "Height with Distance A root 2, Distance B root2 (heightandDistA2_DistB_root2) = "<<heightandDistA2_DistB_root2<<endl;
//			cout <<endl;


			float bad_estimate_height_1 = height_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root1, heightandDistA1_DistB_root1);
//			cout << "For bad estimated z-A: "<< height_DistA_root1 << " and z_B: "<< heightandDistA1_DistB_root1<<endl;
//			cout << "Bad estimate height 1: " << bad_estimate_height_1<<endl;
//			cout <<endl;

			float bad_estimate_height_2 = height_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root1, heightandDistA1_DistB_root2);
//			cout << "For bad estimated z-A: "<< height_DistA_root1 << " and z_B: "<< heightandDistA1_DistB_root2<<endl;
//			cout << "Bad estimate height 2: " << bad_estimate_height_2<<endl;
//			cout <<endl;

			float bad_estimate_height_3 = height_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root2, heightandDistA2_DistB_root1);
//			cout << "For bad estimated z-A: "<< height_DistA_root2 << " and z_B: "<< heightandDistA2_DistB_root1<<endl;
//			cout << "Bad estimate height 3: " << bad_estimate_height_3<<endl;
//			cout <<endl;

			float bad_estimate_height_4 = height_equation_calculator(tan_theta_1A, tan_theta_1B, height_DistA_root2, heightandDistA2_DistB_root2);
//			cout << "For bad estimated z-A: "<< height_DistA_root2 << " and z_B: "<< heightandDistA2_DistB_root2<<endl;
//			cout << "Bad estimate height 4: " << bad_estimate_height_4<<endl;
//			cout <<endl;


			float estimated_height = (bad_estimate_height_1 + bad_estimate_height_2 + bad_estimate_height_3 + bad_estimate_height_4)/4;

	float person_height_1_1,person_height_1_2;
	boost::tie(person_height_1_1, person_height_1_2) = person_height(estimated_height,tan_theta_1A,tan_theta_1B, height_DistA_root1,heightandDistA1_DistB_root1);

	float person_height_2_1,person_height_2_2;
	boost::tie(person_height_2_1,person_height_2_2) = person_height(estimated_height,tan_theta_1A,tan_theta_1B, height_DistA_root1,heightandDistA2_DistB_root2);

	float person_height_3_1,person_height_3_2;
	boost::tie(person_height_3_1,person_height_3_2) = person_height(estimated_height,tan_theta_1A,tan_theta_1B, height_DistA_root2,heightandDistA1_DistB_root1);

	float person_height_4_1,person_height_4_2;
	boost::tie(person_height_4_1,person_height_4_2) = person_height(estimated_height,tan_theta_1A,tan_theta_1B, height_DistA_root2,heightandDistA2_DistB_root2);

	float avg_person_height = (person_height_1_1+person_height_1_2 + person_height_2_1+person_height_2_2 + person_height_2_1+person_height_2_2
			+ person_height_4_1+person_height_4_2) / 8;

	cout << estimated_height<<" "<<height_DistA_root1<<" "<<height_DistA_root2<<" "<<heightandDistA1_DistB_root1<<" "<< heightandDistA1_DistB_root2<<" "<<avg_person_height<<endl;

//	cout << endl;


//			float bad_z_B;
//			if (height_DistA_root1 > height_DistA_root2){
//				if ()
//			}


//double q,w;
//boost::tie(q,w) = testing(3,4);
//cout << q <<w;
			//break;
		}
	synthetic_data_file.close();

//8888888
//*******/


	return 0;
}
