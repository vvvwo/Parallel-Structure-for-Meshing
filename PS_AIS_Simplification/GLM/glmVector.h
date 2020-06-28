#pragma once
#include <GL/freeglut.h>
#include <vector>
using namespace std;


vector<int> glmBox();
int glmNosePoint(GLMmodel* model);
vector<vector <int>> glmPointNeibor(GLMmodel* model);//计算点云中每一点的相邻点

int glmIndexPoint(GLMmodel* model,double x,double y,double z);

void glmTurnNorm2Trangular(GLMmodel* model);