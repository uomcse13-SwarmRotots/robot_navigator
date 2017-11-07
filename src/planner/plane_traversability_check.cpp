bool planeTraversabilityCheck(double a,double b,double c,double e, double x0,double y0,double z0, double d){
	
	double c2_ = c/(sqrt(a*a+b*b+c*c));
	double angle = acos(c2_);
	//Print angle	

	double acceptedAngle 	= 0.349066; // 20 degree
	double acceptedDistance = (d/2)*sin(angle);

	//checking the traversability
	//Angle check
	if(angle<acceptedAngle){ 
		//distance calculation and check
		//double upper = a*x0 + b*y0 + c*z0 + d;
		//double lower = sqrt(a*a+b*b+c*c);
		double distance = (a*x0 + b*y0 + c*z0 + e)/sqrt(a*a+b*b+c*c);
		//Print distance
		
		//take evaluationg decision based on values
		return true;
		
	}
	else return false;

}