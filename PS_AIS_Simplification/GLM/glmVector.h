#pragma once
#include <GL/freeglut.h>
#include <vector>
using namespace std;


vector<int> glmBox();
int glmNosePoint(GLMmodel* model);
vector<vector <int>> glmPointNeibor(GLMmodel* model);//���������ÿһ������ڵ�

int glmIndexPoint(GLMmodel* model,double x,double y,double z);

void glmTurnNorm2Trangular(GLMmodel* model);