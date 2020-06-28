#pragma once

#include "View.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h>   // OpenGL GLUT Library Header
#include "FileProcess/LoadFileDlg.h"
#include "trackball.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "pointProcessPipeline.hpp"
#include "GenerateRemesh.cpp"

using namespace std;

int main()
{
	cout << "start!" << endl;
	//input point cloud, obj, xyz, ply, txt, off
	string pathObj = "Bunny";
	string pathFile = "data//" + pathObj + ".obj"; //"3DModel/bunny.obj";//T164.obj
	int simplificationNum = 10000;
	//**Init point cloud**
	pointProcessPipeline ppp;	
	ppp.pointProcessPipeline_init(pathFile, true);
	
	//**Remesh Process**
	GRemeshTool gt;
	gt.Remesh(ppp.br, simplificationNum, pathObj);
	
	cout << "Finish!" << endl;

}






