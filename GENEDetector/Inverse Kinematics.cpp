#include <iostream>
#include "Point.cpp"
#include "PIDController.cpp"
#include <math.h>

using namespace std;

void inverseKinematics(Point end_point, double& new_ang1, double& new_ang2);

double PointDistance(Point p1, Point p2); 
double toRadians(double angle); 
double toDegrees(double angle); 

const double LEN_A = 5.0; 
const double LEN_B = 4.0; 
const double M_PI = 3.141592653589793238463; 

/*
int main(int argc, char** argv)
{
	double curr_angOne = 25; 
	double curr_angTwo = 25; 
	double new_x = 0, new_y = 0, new_z = 0;

	Point end_point(0, 0); 
	while (true)
	{
		cout << "Enter New Point" << endl; 
		cout << "X: "; 
		cin >> new_x;
		cout << endl << "Y: ";
		cin >> new_y; 
		cout << endl << "Z: ";
		cin >> new_z;

		end_point.setX(new_x); 
		end_point.setY(new_y); 

		inverseKinematics(end_point, curr_angOne, curr_angTwo); 

		cout << "Angle One: " << curr_angOne << "Angle Two: " << curr_angTwo << endl;

		//Angle to the left or to the right
		if(new_z > 0)
			cout << "Angle Turret: " << toDegrees(atan(new_z / new_x)) << endl; 
		else 
			cout << "Angle Turret: " << -toDegrees(atan(new_z / new_x)) << endl;

	}
}
*/
//InverseKinematics calculates the joint angles given an endpoint
//Point p - endpoint in Cartesian space
//double curr_ang1 - current angle of first joint
//double curr_ang2 - current angle of second joint
//return - new first and second joint angles

void inverseKinematics(Point end_point, double & new_ang1, double & new_ang2){

	double radius = PointDistance(Point(0, 0), end_point); //distance from origin to point
	cout << "Distance: " << radius << endl; 
	double angle_theta = atan2(end_point.y, end_point.x);  //angle counterclockwise from reference x-axis to point
	cout << "Angle Theta: " << angle_theta << endl;

	double quad1_angB = acos((radius * radius - LEN_A * LEN_A - LEN_B * LEN_B) / (2 * LEN_A * LEN_B)); //second joint angle
	double quad1_angA = angle_theta + fabs(asin((sin(quad1_angB) * LEN_B)/radius)); //first joint angle
	quad1_angB = quad1_angB - quad1_angA; 
	
	double quad2_angB = acos((radius * radius - LEN_A * LEN_A - LEN_B * LEN_B) / (2 * LEN_A * LEN_B)); //second joint angle
	double quad2_angA = angle_theta - fabs(atan((LEN_B * sin(quad2_angB)) / (LEN_A + LEN_B * cos(quad2_angB)))); //first joint angle
	 
	//Servo position is +ve for -ve second joint angle
		new_ang1 = toDegrees(quad1_angA);
		new_ang2 = toDegrees(quad1_angB);
	
}//End

// PointDistance calculates the length of the hypotenuse formed by two points
//Point p1 - first point
//Point p2 - second point
double PointDistance(Point p1, Point p2){
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); 
} //end PointDistance


double toRadians(double angle)
{
	return angle * ((2 * M_PI) / 360.0); 
}

double toDegrees(double angle)
{
	return angle * (360.0 / (2 * M_PI));
}