#include "joyDrivers.h"


joyStruct driverXboxOne(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.seq = msg.header.seq;
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	if(msg.axes[6] == 1){
		joy.buttonLeft = 1;
	    joy.buttonRight = 0;
	}
	else if(msg.axes[6] == -1){
		joy.buttonLeft = 0;
	    joy.buttonRight = 1;
	}
	else{
		joy.buttonLeft = 0;
	    joy.buttonRight = 0;
	}

	if(msg.axes[7] == 1){
		joy.buttonUp = 1;
	    joy.buttonDown = 0;
	}
	else if(msg.axes[7] == -1){
		joy.buttonUp = 0;
	    joy.buttonDown = 1;
	}
	else{
		joy.buttonUp = 0;
	    joy.buttonDown = 0;
	}

	return joy;
}

joyStruct driverXbox360(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.seq = msg.header.seq;
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];
	joy.buttonLeft = msg.buttons[11];
    joy.buttonRight = msg.buttons[12];
	joy.buttonUp = 0;
	joy.buttonDown = 0;

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	return joy;
}

joyStruct driverXbox360Wired(sensor_msgs::Joy msg){
	joyStruct joy;

	joy.seq = msg.header.seq;
	joy.buttonA = msg.buttons[0];
	joy.buttonB = msg.buttons[1];
	joy.buttonX = msg.buttons[2];
	joy.buttonY = msg.buttons[3];
	joy.buttonL1 = msg.buttons[4];
	joy.buttonR1 = msg.buttons[5];
	joy.buttonSelect = msg.buttons[6];
	joy.buttonStart = msg.buttons[7];

	joy.LstickHor = msg.axes[0];
	joy.LstickVer = msg.axes[1];
	joy.L2 = msg.axes[2];
	joy.RstickHor = msg.axes[3];
	joy.RstickVer = msg.axes[4];
	joy.R2 = msg.axes[5];

	if(msg.axes[6] == 1){
		joy.buttonLeft = 1;
	    joy.buttonRight = 0;
	}
	else if(msg.axes[6] == -1){
		joy.buttonLeft = 0;
	    joy.buttonRight = 1;
	}
	else{
		joy.buttonLeft = 0;
	    joy.buttonRight = 0;
	}

	if(msg.axes[7] == 1){
		joy.buttonUp = 1;
	    joy.buttonDown = 0;
	}
	else if(msg.axes[7] == -1){
		joy.buttonUp = 0;
	    joy.buttonDown = 1;
	}
	else{
	joy.buttonUp = 0;
	    joy.buttonDown = 0;
	}

		return joy;
}

PVA_structure filterJoy(PVA_structure PVA_ref,
                        geometry_msgs::Vector3 Vel_ref,
                        double dt){
	const double zeta = 1.0;	//Critically damped
	const double wn = 5.0;		//Time constant = 1/5
	Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity(3,3);

	//Low pass filter continuous model
	Eigen::Matrix3d A;
	Eigen::Vector3d B;

	A << 0,      1,          0,
	     0,      0,          1,
	     0, -wn*wn, -2*zeta*wn;
	B << 0,
	     0,
	     wn*wn;

	//Current states in x,y,z direction
	Eigen::Vector3d Xk_x, Xk_y, Xk_z;
	Xk_x << PVA_ref.Pos.pose.position.x,
	        PVA_ref.Vel.twist.linear.x,
	        PVA_ref.Acc.accel.linear.x;
	Xk_y << PVA_ref.Pos.pose.position.y,
	        PVA_ref.Vel.twist.linear.y,
	        PVA_ref.Acc.accel.linear.y;
	Xk_z << PVA_ref.Pos.pose.position.z,
	        PVA_ref.Vel.twist.linear.z,
	        PVA_ref.Acc.accel.linear.z;

	//Inputs to the system
	Eigen::Vector3d U;
	U << Vel_ref.x,
	     Vel_ref.y,
	     Vel_ref.z;

	//Propagate states (crude euler integration)
	Eigen::Vector3d Xk1_x, Xk1_y, Xk1_z;
	Xk1_x = (I_3x3 + A*dt)*Xk_x + dt*B*U[0];
	Xk1_y = (I_3x3 + A*dt)*Xk_y + dt*B*U[1];
	Xk1_z = (I_3x3 + A*dt)*Xk_z + dt*B*U[2];

	//Update PVA structure
	PVA_structure PVA_ref_new;
	PVA_ref_new.Pos.pose.position = SetPoint(Xk1_x[0], Xk1_y[0], Xk1_z[0]);
	PVA_ref_new.Vel.twist.linear  = SetVector3(Xk1_x[1], Xk1_y[1], Xk1_z[1]);
	PVA_ref_new.Acc.accel.linear  = SetVector3(Xk1_x[2], Xk1_y[2], Xk1_z[2]);

	return PVA_ref_new;

}

void printJoyValues(joyStruct joy){

	std::cout << "Buttons" << "\t" << joy.seq << "\n";
	std::cout << "Buttons" << "\t";
	std::cout << joy.buttonA << " " << joy.buttonB << " "
	          << joy.buttonX << " "<< joy.buttonY << " "
			  << joy.buttonR1 << " " << joy.buttonL1 << " "
	          << joy.buttonSelect << " "<< joy.buttonStart << " "
			  << joy.buttonLeft << " " << joy.buttonRight << " "
	          << joy.buttonUp << " "<< joy.buttonDown;
	std::cout << std::setprecision(3) << "\nAxes" << "\t";
	std::cout << joy.LstickHor << "\t" << joy.LstickVer << "\t"
	          << joy.RstickHor << "\t"<< joy.RstickVer << "\t"
			  << joy.L2 << "\t" << joy.R2 << "\n";
}

