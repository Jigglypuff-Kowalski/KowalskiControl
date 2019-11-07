//Class to represent a point in cartesian space

//A point in cartesian space
class Point
{

public:
	double x; //x coordinate of the point
	double y; //y coordinate of the point

	Point(double coord_x, double coord_y)
	{
		x = coord_x; 
		y = coord_y; 
	}

	//set the x position of the point
	void setX(double new_x)
	{
		x = new_x; 
	}

	void setY(double new_y)
	{
		y = new_y; 
	}
}; 