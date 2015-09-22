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

	//	return boost::make_tuple(a+b, a-b);

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


/*
 * 4x4 matrix PRT Matrix
 * (Projection Matrix dot product Homogeneous Rotation Matrix) dot product Translation Matrix.
 * Function for generating PRT Matrix
 */
Mat prt_matrix_func(Mat homogeneousRotationMatrix, float actualCameraHeight, float focalLength)
{
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

	//	cout << "PRT Matrix: "<< endl<<PRTMatrix<<endl<<endl;
	return PRTMatrix;
}


/*
 * Actual location of the HEAD of a person at FIRST DISTANCE.
 * Function for generating coordinates of a person's HEAD in real world.
 */
Mat head_location(float personHeight, float distance)
{
	Mat head_location = Mat::ones(1,4, CV_32F);
	head_location.at<float>(0,0) = 0;
	head_location.at<float>(0,1) = personHeight;
	head_location.at<float>(0,2) = distance;
	return head_location;
}


/*
 * Actual location of the FOOT of a person at FIRST DISTANCE.
 * Function for generating coordinates of a person's FOOT in real world.
 */
Mat foot_location(float distance)
{
	Mat foot_location = Mat::ones(1,4, CV_32F);
	foot_location.at<float>(0,0) = 0;
	foot_location.at<float>(0,1) = 0;
	foot_location.at<float>(0,2) = distance;
	return foot_location;
}


/*
 * Multiplying PRT matrix with Actual locations to get the projected locations on the image plane
 */

/*
 * Projected location of HEAD at FIRST distance.
 * This function for generating coordinates of a person's
 * PROJECTED HEAD location on the image plane.
 */
Mat projected_head_location(Mat prt_matrix, Mat headX_distX)
{
	Mat projected_head_location = Mat::ones(1,4, CV_32F);
	projected_head_location.at<float>(0,0) = (prt_matrix.row(0)).dot(headX_distX);
	projected_head_location.at<float>(0,1) = (prt_matrix.row(1)).dot(headX_distX);
	projected_head_location.at<float>(0,2) = (prt_matrix.row(2)).dot(headX_distX);
	projected_head_location.at<float>(0,3) = (prt_matrix.row(3)).dot(headX_distX);
	projected_head_location = projected_head_location / projected_head_location.at<float>(0,3);
	return projected_head_location;
}


/*
 * Projected location of FOOT at FIRST distance.
 * Function for generating coordinates of a person's
 * PROJECTED FOOT location on the image plane.
 */
Mat projected_foot_location(Mat prt_matrix, Mat footX_distX)
{
	Mat projected_foot_location = Mat::ones(1,4, CV_32F);
	projected_foot_location.at<float>(0,0) = (prt_matrix.row(0)).dot(footX_distX);
	projected_foot_location.at<float>(0,1) = (prt_matrix.row(1)).dot(footX_distX);
	projected_foot_location.at<float>(0,2) = (prt_matrix.row(2)).dot(footX_distX);
	projected_foot_location.at<float>(0,3) = (prt_matrix.row(3)).dot(footX_distX);
	projected_foot_location = projected_foot_location / projected_foot_location.at<float>(0,3);
	return projected_foot_location;
}


/*
 * Function for generating Height of a person.
 * This function calculates the height of the same person at distance Za and Zb
 * and returns it as a tuple.
 */
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
//	double firstPersonHeight = 1.58; // Height 5.2 feet
//	double secondPersonHeight = 1.82; // height 6 feet
//	double thirdPersonHeight = 1.79; // height 5.9 feet
//
//	Mat rotation_matirx = rotation_matrix(theta);
//	Mat prt_matrix = prt_matrix_func(rotation_matirx, actualCameraHeight, focalLength);
//
//	std::fstream syntheticdata;
//	syntheticdata.open("src/Synthetic_Data.txt", std::fstream::out | std::fstream::trunc);
//
//	for (int i=0; i<100; i++)
//	{
//
//		firstDistance += -0.5;
//		secondDistance += -0.5;
//
//		//!!!	double firstPersonHeight = 1.42;
//		//!!!	double secondPersonHeight = fRand(1,2);
//		//!!!	double thirdPersonHeight = fRand(1.5,2.5);
//
//
//		// Head and Foot of 1st person at Za and Zb
//		Mat head1_dist1 = head_location(firstPersonHeight, firstDistance);
//		Mat foot1_dist1 = foot_location(firstDistance);
//
//		Mat head1_dist2 = head_location(firstPersonHeight, secondDistance);
//		Mat foot1_dist2 = foot_location(secondDistance);
//
//		// Head and Foot of 2nd person at Za and Zb
//		Mat head2_dist1 = head_location(secondPersonHeight, firstDistance);
//		Mat foot2_dist1 = foot_location(firstDistance);
//
//		Mat head2_dist2 = head_location(secondPersonHeight, secondDistance);
//		Mat foot2_dist2 = foot_location(secondDistance);
//
//		// Head and Foot of 3rd person at Za and Zb
//		Mat head3_dist1 = head_location(thirdPersonHeight, firstDistance);
//		Mat foot3_dist1 = foot_location(firstDistance);
//
//		Mat head3_dist2 = head_location(thirdPersonHeight, secondDistance);
//		Mat foot3_dist2 = foot_location(secondDistance);
//
//
//		/*
//		 *   PROJECTIONS ON IMAGE PLANE
//		 */
//
//		// Projected Head and Foot of 1st person at Za and Zb on the image plane
//		Mat head1_dist1_projection = projected_head_location(prt_matrix, head1_dist1);
//		Mat foot1_dist1_projection = projected_foot_location(prt_matrix, foot1_dist1);
//
//		Mat head1_dist2_projection = projected_head_location(prt_matrix, head1_dist2);
//		Mat foot1_dist2_projection = projected_foot_location(prt_matrix, foot1_dist2);
//
//
//		// Projected Head and Foot of 2nd person at Za and Zb on the image plane
//		Mat head2_dist1_projection = projected_head_location(prt_matrix, head2_dist1);
//		Mat foot2_dist1_projection = projected_foot_location(prt_matrix, foot2_dist1);
//
//		Mat head2_dist2_projection = projected_head_location(prt_matrix, head2_dist2);
//		Mat foot2_dist2_projection = projected_foot_location(prt_matrix, foot2_dist2);
//
//
//		// Projected Head and Foot of 3rd person at Za and Zb on the image plane
//		Mat head3_dist1_projection = projected_head_location(prt_matrix, head3_dist1);
//		Mat foot3_dist1_projection = projected_foot_location(prt_matrix, foot3_dist1);
//
//		Mat head3_dist2_projection = projected_head_location(prt_matrix, head3_dist2);
//		Mat foot3_dist2_projection = projected_foot_location(prt_matrix, foot3_dist2);
//
//
//		/*
//		 * Writing all the information in a file
//		 */
//		syntheticdata <<focalLength<<" "<<actualCameraHeight <<"  "<< theta <<"  "<< firstDistance <<"  "<< secondDistance <<"  "<< firstPersonHeight
//				<<"  "<< secondPersonHeight <<"  "<< thirdPersonHeight <<"  "<< head1_dist1_projection.at<float>(0,1) <<"  "<< foot1_dist1_projection.at<float>(0,1)
//				<<"  "<< head1_dist2_projection.at<float>(0,1) <<"  "<< foot1_dist2_projection.at<float>(0,1)<<"  "<< head2_dist1_projection.at<float>(0,1)
//				<<"  "<< foot2_dist1_projection.at<float>(0,1)<<"  "<< head2_dist2_projection.at<float>(0,1)<<"  "<< foot2_dist2_projection.at<float>(0,1)
//				<<" "<< head3_dist1_projection.at<float>(0,1)<<" "<< foot3_dist1_projection.at<float>(0,1)<<" "<< head3_dist2_projection.at<float>(0,1)
//				<<" "<< foot3_dist2_projection.at<float>(0,1)<<endl;
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
		//vector<string> fileContents;
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


		//cout <<focal_length << " "<<camera_height<<" "<<theta<<" "<<z_A<<" "<<z_B<<" "<<height_1<<" "<<head1_dist1<<" "<<foot1_dist1<<" "<<head1_dist2<<" "<<foot1_dist2<<endl<<endl;

		// For 1st person
		float tan_theta_1A = tan_theta(focal_length, head1_dist1, foot1_dist1);
		float tan_theta_1B = tan_theta(focal_length, head1_dist2, foot1_dist2);
		//cout << "Focal: "<<focal_length<<", head1 dist1: "<< head1_dist1<<", foot1 dist1: "<< foot1_dist1<<", theta b/w them: "<< tan_theta_1A <<endl;


		// For 2nd person
		double tan_theta_2A = tan_theta(focal_length, head2_dist1, foot2_dist1);
		double tan_theta_2B = tan_theta(focal_length, head2_dist2, foot2_dist2);


		// For 3rd person
		double tan_theta_3A = tan_theta(focal_length, head3_dist1, foot3_dist1);
		double tan_theta_3B = tan_theta(focal_length, head3_dist2, foot3_dist2);


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


	}
	synthetic_data_file.close();

	//8888888
	//*******/

	return 0;
}
