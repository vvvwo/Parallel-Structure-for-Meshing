#pragma once
#include "Method_AIVS_Remesh.hpp"
#include "ballRegionCompute.hpp"


class GRemeshTool {

public:

	void Remesh(BallRegion br,int simplificationNum, string fileName) {//1:CGAL_PoissonRemesh 2:CVT
		
		vector<vector<double>> point_sim;
		clock_t t1;
		clock_t t2;
		cout << "AIVS meshing start." << endl;
		AIVS_Reconstruct ar;
		ar.AIVS_Reconstruct_init(br);		
		t1 = clock();
		ar.AIVS_Reconstruct_Remesh(simplificationNum, fileName);
		t2 = clock();
		std::cout << "Running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		cout << "AIVS Remesh end." << endl;	

	}

};
