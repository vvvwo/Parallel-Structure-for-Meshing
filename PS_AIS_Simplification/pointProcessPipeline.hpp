#pragma once

#include "LoadPointCloud.hpp"
#include "normalCompute.hpp"
#include "ballRegionCompute.hpp"

class pointProcessPipeline {

public:
	LoadPointCloud lpc;//load different kinds of files to achieve point cloud	
	NormalEstimation ne;//normal estimation for point cloud
	BallRegion br;//neibor structure construct
	BallRegion br_fix;//add points to fix neibor structure construct
	vector<int> fixBox;//
	int theorld;

public:

	void pointProcessPipeline_init(string filepath, bool brj) {
		//ne.test();
		cout << "pointProcessPipeline_init run!"<< endl;
		lpc.PointCloud_Load(filepath);//support obj, off, ply, xyz, txt
		//lpc.PointCloud_Load_Fast(filepath);
		
		ne.estimateNormal_init(lpc.FileNormal);

		//if (filepath.find(".obj") >= 0) {
			//char* pfile = new char[strlen(filepath.c_str()) + 1];
			//strcpy(pfile, filepath.c_str());
			//GLMmodel* pModel = glmReadOBJ(pfile);		
		//}		
		if (ne.normalLoad()) {
			cout << "pointProcessPipeline_init: normal file exist and the data are loaded." << endl;
		}
		/*
		else if (filepath.find(".obj") >= 0) {
			cout << "obj normal start." << endl;
			char* pfile = new char[strlen(filepath.c_str()) + 1];
			strcpy(pfile, filepath.c_str());
			GLMmodel* pModel = glmReadOBJ(pfile);	
			ne.estimateNormal(pModel);		
		}*/
		else {
			cout << "pcl normal start." << endl;
			//ne.estimateNormal(lpc.pointSet_uniform, 'j');	
			ne.estimateNormal_PCL_MP(lpc.pointSet_uniform);
		}			

		if (brj) {
			vector<vector<double>> pdata = lpc.pointSet_uniform;
			vector<vector<double>> ndata = ne.normalVector;
			vector<int> borderdata = lpc.indexBorder;
			br.BallRegion_init(pdata, ndata, borderdata);		
		}	

	}
		
		 
};
