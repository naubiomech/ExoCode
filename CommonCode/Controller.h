/*
 * 
 * 
 * P. Stegall Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

typedef enum controllers{
	hipControllerName
	ankleControllerName
} controllers

struct controllerData {
	int setPoint;
	int parameter1;
	int parameter2;
};

class Controller
{
	public:
		
		void setController(int joint, int controller); // Changes the controller for an individual joint
		int calcMotorCMD();
		
		int setPoint;
		int parameter1;
		int parameter2;
		
		
	private:
		
};

#endif