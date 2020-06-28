#pragma once
#include "Method_CGAL.hpp"
#include "Method_CVT.hpp"
#include "VD.hpp"
#include "cvDrawing.hpp"

class GSTool {		

public:
	vector<vector<double>> simReturn(int simNumber, BallRegion br, int methodSign){//1:CGAL 2:CVT
		
		vector<vector<double>> point_sim;		
		clock_t t1;
		clock_t t2;

		if (methodSign == 1) {
			simplification_Method_CGAL smc;
			smc.simplification_Method_CGAL_init(br.pointCloudData, br.pointNormal, br.radius);
			t1 = clock();
			//point_sim = smc.simplification_Method_CGAL_Grid(0.033,pathFile10000);
			point_sim = smc.simplification_Method_CGAL_WLOP(simNumber);
			//point_sim = smc.simplification_Method_CGAL_Hierarchy(1000, 0.045);
			t2 = clock();
			std::cout << "simplification running time:" << (t2 - t1) / 1000.0 << "s" << endl;
			return point_sim;
		}
		else if (methodSign == 2) {
			simplification_Method_CVT smCVT;
			smCVT.simplification_Method_CVT_init(br);
			t1 = clock();
			point_sim = smCVT.CVT_Simplification(simNumber);
			t2 = clock();
			std::cout << "simplification running time:" << (t2 - t1) / 1000.0 << "s" << endl;
			return point_sim;
			
		}
		else if (methodSign == 3) {
		
		}
		else {
		
		}
	}
	   	  
};
