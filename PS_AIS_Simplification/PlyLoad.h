#pragma once
#ifndef PLYREADER_H_
#define PLYREADER_H_

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <iostream>
using namespace std;

struct SModelData
{
	
	vector <float> vecFaceTriangles; // = face * 9
	vector <float> vecFaceTriangleColors; // = face * 9
	vector <float> vecNormals; // = face * 9
	int iTotalConnectedTriangles;
};

class CPLYLoader
{
public:
	
	vector<vector<double>> points;
	vector<vector<double>> normals;
	vector<vector<double>> colors;
	vector<vector<int>> vecFaceIndex;	
	CPLYLoader();
	int LoadModel(char* filename);
	void Draw();	

private:

	SModelData m_ModelData;
	float* mp_vertexXYZ;
	float* mp_vertexNorm;
	float* mp_vertexRGB;
	int m_totalConnectedQuads;
	int m_totalConnectedPoints;
	int m_totalFaces;	
};

#endif
