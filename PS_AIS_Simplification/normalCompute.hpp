#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h> 
#include "GLM/glm.h"
#include "GLM/glmVector.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
//PCL
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include "ballRegionCompute.hpp"

class NormalEstimation {

public:

	vector<vector<double>> normalVector;
	string fileNormal;


public:

	void estimateNormal_init(string fileNormal_input) {

		fileNormal = fileNormal_input;

	}

	//default normal computation
	void estimateNormal(GLMmodel* pModel)
	{
		
		if (pModel->numnormals == pModel->numvertices) {

			for (int i = 1; i <= pModel->numvertices; i++) {
				double nx_i = pModel->normals[3 * i];
				double ny_i = pModel->normals[3 * i + 1];
				double nz_i = pModel->normals[3 * i + 2];
				if (nx_i == 0 && ny_i == 0 && nz_i == 0) {
					vector<double> nv_i;
					nv_i.push_back(0);
					nv_i.push_back(1);
					nv_i.push_back(0);
					normalVector.push_back(nv_i);
				}
				else {
					double normalength = sqrt(nx_i * nx_i + ny_i * ny_i + nz_i * nz_i);
					vector<double> nv_i;
					nv_i.push_back(nx_i / normalength);
					nv_i.push_back(ny_i / normalength);
					nv_i.push_back(nz_i / normalength);
					normalVector.push_back(nv_i);
				}
			}
			//nv_i.push_back(pModel); 
			//normalVector.push_back(nv_i);	
			normalSave(normalVector);
			return;
		}
		

		if (normalVector.size() > 0) {

			normalVector.clear();

		}

		if (pModel->numfacetnorms <= 2) {
			glmFacetNormals(pModel);
		}

		vector<vector<double>> pn;
		for (int i = 0; i <= pModel->numvertices; i++) {

			vector<double> empty_i;
			pn.push_back(empty_i);

		}

		cout << "numtriangles:" << pModel->numtriangles << endl;
		cout << "numfaceNormal" << pModel->numfacetnorms << endl;

		for (int i = 0; i < pModel->numtriangles; i++) {

			int b0 = pModel->triangles[i].vindices[0];
			int b1 = pModel->triangles[i].vindices[1];
			int b2 = pModel->triangles[i].vindices[2];
			double xn = pModel->facetnorms[3 * (i + 1)];
			double yn = pModel->facetnorms[3 * (i + 1) + 1];
			double zn = pModel->facetnorms[3 * (i + 1) + 2];
			pn[b0].push_back(xn);
			pn[b0].push_back(yn);
			pn[b0].push_back(zn);
			pn[b1].push_back(xn);
			pn[b1].push_back(yn);
			pn[b1].push_back(zn);
			pn[b2].push_back(xn);
			pn[b2].push_back(yn);
			pn[b2].push_back(zn);

		}

		for (int i = 1; i < pn.size(); i++) {

			vector<double> pn_i = pn[i];

			double x_sum = 0;
			double y_sum = 0;
			double z_sum = 0;

			if (pn_i.size() == 0) {
				vector<double> empty_i;
				empty_i.push_back(0);
				empty_i.push_back(0);
				empty_i.push_back(0);
				normalVector.push_back(empty_i);

			}
			else {
				for (int j = 0; j < pn_i.size() / 3; j++) {

					x_sum = x_sum + pn_i[3 * j];
					y_sum = y_sum + pn_i[3 * j + 1];
					z_sum = z_sum + pn_i[3 * j + 2];

				}

				vector<double> nv_i;
				nv_i.push_back(x_sum / (double)pn_i.size());
				nv_i.push_back(y_sum / (double)pn_i.size());
				nv_i.push_back(z_sum / (double)pn_i.size());

				double normalLength = sqrt(nv_i[0] * nv_i[0] + nv_i[1] * nv_i[1] + nv_i[2] * nv_i[2]);
				nv_i[0] = nv_i[0] / normalLength;
				nv_i[1] = nv_i[1] / normalLength;
				nv_i[2] = nv_i[2] / normalLength;

				normalVector.push_back(nv_i);

			}
		}
		normalSave(normalVector);
	}

	//weight based normal computation
	void estimateNormal(GLMmodel * pModel, bool accurate) {

		if (normalVector.size() > 0) {
			normalVector.clear();
		}

		if (pModel->numfacetnorms <= 2) {

			glmFacetNormals(pModel);

		}

		//achieve the trangular for different points.
		vector<vector<int>> neiborTrans;//neibor tragular index
		vector<vector<int>> neiborTrans_index;//neibor tragular index
		vector<vector<double>> neiborTrans_Norm;//neibor trangular norm
		vector<vector<double>> neiborTrans_Area;//bneibor tragular area

		//add first item to regular data of norm and area
		vector<double> empty_int;
		vector<double> empty_double;
		neiborTrans_Norm.push_back(empty_int);
		neiborTrans_Area.push_back(empty_double);

		for (int i = 0; i <= pModel->numvertices; i++) {
			vector<int> t_i;
			neiborTrans.push_back(t_i);
			neiborTrans_index.push_back(t_i);
		}

		for (int i = 0; i < pModel->numtriangles; i++) {
			int b1 = pModel->triangles[i].vindices[0];
			int b2 = pModel->triangles[i].vindices[1];
			int b3 = pModel->triangles[i].vindices[2];
			neiborTrans[b1].push_back(b1);
			neiborTrans[b1].push_back(b2);
			neiborTrans[b1].push_back(b3);
			neiborTrans_index[b1].push_back(i);
			neiborTrans[b2].push_back(b1);
			neiborTrans[b2].push_back(b2);
			neiborTrans[b2].push_back(b3);
			neiborTrans_index[b2].push_back(i);
			neiborTrans[b3].push_back(b1);
			neiborTrans[b3].push_back(b2);
			neiborTrans[b3].push_back(b3);
			neiborTrans_index[b3].push_back(i);
		}

		for (int i = 1; i < neiborTrans.size(); i++) {
			vector<int> tragular_index = neiborTrans[i];
			vector<int> tragular_index_index = neiborTrans_index[i];
			vector<double> norm_i;
			vector<double> area_i;
			for (int j = 0; j < tragular_index.size() / 3; j++) {
				int b1 = tragular_index[3 * j];
				int b2 = tragular_index[3 * j + 1];
				int b3 = tragular_index[3 * j + 2];
				int trans_i = tragular_index_index[j];
				double xb1 = pModel->vertices[3 * b1];
				double yb1 = pModel->vertices[3 * b1 + 1];
				double zb1 = pModel->vertices[3 * b1 + 2];
				double xb2 = pModel->vertices[3 * b2];
				double yb2 = pModel->vertices[3 * b2 + 1];
				double zb2 = pModel->vertices[3 * b2 + 2];
				double xb3 = pModel->vertices[3 * b3];
				double yb3 = pModel->vertices[3 * b3 + 1];
				double zb3 = pModel->vertices[3 * b3 + 2];
				double v12x = xb2 - xb1;//ax
				double v12y = yb2 - yb1;//ay
				double v12z = zb2 - zb1;//az
				double v23x = xb3 - xb2;//bx
				double v23y = yb3 - yb2;//by
				double v23z = zb3 - zb2;//bz
				double v31x = xb1 - xb3;
				double v31y = yb1 - yb3;
				double v31z = zb1 - zb3;
				double nx = v12y * v23z - v12z * v23y;
				double ny = v12z * v23x - v12x * v23z;
				double nz = v12x * v23y - v12y * v23x;
				double nx1 = v23y * v31z - v23z * v31y;
				double ny1 = v23z * v31x - v23x * v31z;
				double nz1 = v23x * v31y - v23y * v31x;

				int b11 = pModel->triangles[trans_i].vindices[0];
				int b12 = pModel->triangles[trans_i].vindices[1];
				int b13 = pModel->triangles[trans_i].vindices[2];
				double nx_1 = pModel->facetnorms[3 * (trans_i + 1)];
				double ny_1 = pModel->facetnorms[3 * (trans_i + 1) + 1];
				double nz_1 = pModel->facetnorms[3 * (trans_i + 1) + 2];

				double l = sqrt(nx_1 * nx_1 + ny_1 * ny_1 + nz_1 * nz_1);
				double unitN = sqrt(nx * nx + ny * ny + nz * nz);

				nx = nx / unitN;
				ny = ny / unitN;
				nz = nz / unitN;
				norm_i.push_back(nx);
				norm_i.push_back(ny);
				norm_i.push_back(nz);

				double v12_length = sqrt(v12x * v12x + v12y * v12y + v12z * v12z);
				double v23_length = sqrt(v23x * v23x + v23y * v23y + v23z * v23z);
				double v31_length = sqrt(v31x * v31x + v31y * v31y + v31z * v31z);

				double p = (v12_length + v23_length + v31_length) / 2;
				double area_ij = sqrt(p * (p - v12_length) * (p - v23_length) * (p - v31_length));
				area_i.push_back(area_ij);
			}

			neiborTrans_Norm.push_back(norm_i);
			neiborTrans_Area.push_back(area_i);
		}

		for (int i = 1; i <= pModel->numvertices; i++) {
			vector<double> norm_vi;
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			norm_vi.push_back(0);
			vector<double> neiborTrans_Norm_i = neiborTrans_Norm[i];
			vector<double> neiborTrans_Area_i = neiborTrans_Area[i];
			double sum_area_i = 0;
			if (neiborTrans_Norm_i.size() == 0 ||
				neiborTrans_Area_i.size() == 0) {
				vector<double> empty_i;
				empty_i.push_back(0);
				empty_i.push_back(0);
				empty_i.push_back(0);
				normalVector.push_back(empty_i);
			}
			else {
				for (int j = 0; j < neiborTrans_Area_i.size(); j++) {
					sum_area_i = sum_area_i + neiborTrans_Area_i[j];
				}
				for (int j = 0; j < neiborTrans_Norm_i.size() / 3; j++) {
					double x_n = neiborTrans_Norm_i[3 * j];
					double y_n = neiborTrans_Norm_i[3 * j + 1];
					double z_n = neiborTrans_Norm_i[3 * j + 2];
					double weight_j;
					if (accurate == true) {
						weight_j = neiborTrans_Area_i[j] / sum_area_i;
					}
					else {
						weight_j = 1 / (double)neiborTrans_Area_i.size();
					}
					norm_vi[0] = norm_vi[0] + x_n * weight_j;
					norm_vi[1] = norm_vi[1] + y_n * weight_j;
					norm_vi[2] = norm_vi[2] + z_n * weight_j;
				}
				double n_length = sqrt(norm_vi[0] * norm_vi[0] + norm_vi[1] * norm_vi[1] + norm_vi[2] * norm_vi[2]);
				norm_vi[0] = norm_vi[0] / n_length;
				norm_vi[1] = norm_vi[1] / n_length;
				norm_vi[2] = norm_vi[2] / n_length;
				normalVector.push_back(norm_vi);
			}
		}
		normalSave(normalVector);
	}

	vector<vector<double>> estimateNormal_PCL_MP_return(vector<vector<double>> pointsVector) {

		vector<vector<double>> normalVector2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// fills a PointCloud with random data
		for (int i = 0; i < pointsVector.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointsVector[i][0];
			cloud_i.y = pointsVector[i][1];
			cloud_i.z = pointsVector[i][2];
			cloud->push_back(cloud_i);
		}

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNumberOfThreads(12);
		ne.setInputCloud(cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setKSearch(20);
		//ne.setRadiusSearch(0.03);

		// Compute the features
		ne.compute(*cloud_normals);

		for (int i = 0; i < cloud_normals->size(); i++) {

			vector<double> normal_i;
			double dis_i = sqrt((cloud_normals->at(i).normal_x) * (cloud_normals->at(i).normal_x) +
				(cloud_normals->at(i).normal_y) * (cloud_normals->at(i).normal_y) +
				(cloud_normals->at(i).normal_z) * (cloud_normals->at(i).normal_z));
			normal_i.push_back(cloud_normals->at(i).normal_x / dis_i);
			normal_i.push_back(cloud_normals->at(i).normal_y / dis_i);
			normal_i.push_back(cloud_normals->at(i).normal_z / dis_i);
			normalVector2.push_back(normal_i);

		}
		return normalVector2;

	}

	void estimateNormal_PCL_MP(vector<vector<double>> pointsVector) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// fills a PointCloud with random data
		for (int i = 0; i < pointsVector.size(); i++)
		{
			pcl::PointXYZ cloud_i;
			cloud_i.x = pointsVector[i][0];
			cloud_i.y = pointsVector[i][1];
			cloud_i.z = pointsVector[i][2];
			cloud->push_back(cloud_i);
		}

		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNumberOfThreads(12);
		ne.setInputCloud(cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		// Use all neighbors in a sphere of radius 3cm
		ne.setKSearch(20);
		//ne.setRadiusSearch(0.03);
		// Compute the features
		ne.compute(*cloud_normals);
		for (int i = 0; i < cloud_normals->size(); i++) {
			vector<double> normal_i;
			double dis_i = sqrt((cloud_normals->at(i).normal_x) * (cloud_normals->at(i).normal_x) +
				(cloud_normals->at(i).normal_y) * (cloud_normals->at(i).normal_y) +
				(cloud_normals->at(i).normal_z) * (cloud_normals->at(i).normal_z));
			normal_i.push_back(cloud_normals->at(i).normal_x / dis_i);
			normal_i.push_back(cloud_normals->at(i).normal_y / dis_i);
			normal_i.push_back(cloud_normals->at(i).normal_z / dis_i);
			normalVector.push_back(normal_i);
		}

		vector<vector<double>> nR = estimateNormal_RegularNormal(pointsVector, normalVector);
		normalVector.clear();
		normalVector = nR;
		normalSave(normalVector);
	}
	
	bool normalLoad() {

		ifstream fin(fileNormal);
		if (fin)
		{
			fin.close();
			freopen(fileNormal.c_str(), "r", stdin);
			if (normalVector.size() > 0) {
				normalVector.clear();
			}
			int numSum;
			std::cin >> numSum;
			for (int i = 0; i < numSum; i++) {
				double x_i;
				double y_i;
				double z_i;
				std::cin >> x_i >> y_i >> z_i;
				vector<double> p_n_i;
				p_n_i.push_back(x_i);
				p_n_i.push_back(y_i);
				p_n_i.push_back(z_i);
				normalVector.push_back(p_n_i);
			}
			return true;
		}
		else {
			fin.close();
			return false;
		}

	}

private:

	double maxErrorNormalMeasure(vector<vector<double>> normalVector1, vector<vector<double>> normalVector2) {

		if (normalVector1.size() != normalVector2.size()) {

			cout << "maxErrorNormalMeasure error! size not equal." << endl;

		}

		int singlePointNum = 0;

		double max = -9999;
		for (int i = 0; i < normalVector1.size(); i++) {

			vector<double> nv1 = normalVector1[i];
			vector<double> nv2 = normalVector2[i];
			vector<double> nv2_neg;
			nv2_neg.push_back(-nv2[0]);
			nv2_neg.push_back(-nv2[1]);
			nv2_neg.push_back(-nv2[2]);

			if ((nv1[0] == 0 && nv1[1] == 0 && nv1[2] == 0) || (nv2[0] == 0 && nv2[1] == 0 && nv2[2] == 0)) {
				singlePointNum++;
				continue;
			}

			double dotProduct = nv1[0] * nv2[0] + nv1[1] * nv2[1] + nv1[2] * nv2[2];
			double dotProduct_neg = nv1[0] * nv2_neg[0] + nv1[1] * nv2_neg[1] + nv1[2] * nv2_neg[2];
			double disProduct = sqrt(nv1[0] * nv1[0] + nv1[1] * nv1[1] + nv1[2] * nv1[2]) *
				sqrt(nv2[0] * nv2[0] + nv2[1] * nv2[1] + nv2[2] * nv2[2]);
			double h_neg = dotProduct_neg / disProduct;
			double h = dotProduct / disProduct;
			if (h_neg > 1) {
				h_neg = 1;
			}
			if (h_neg < -1) {
				h_neg = -1;
			}
			if (h > 1) {
				h = 1;
			}
			if (h < -1) {
				h = -1;
			}

			double angle_i_neg = acos(h_neg);
			double angle_i = acos(h);

			if (angle_i_neg < angle_i) {
				angle_i = angle_i_neg;
			}

			if (angle_i > max) {
				max = angle_i;
			}

		}
		cout << "Single Point Num:" << singlePointNum << endl;
		return max;

	}

	double aveErrorNormalMeasure(vector<vector<double>> normalVector1, vector<vector<double>> normalVector2) {

		if (normalVector1.size() != normalVector2.size()) {

			cout << "maxErrorNormalMeasure error! size not equal." << endl;

		}

		int singlePointNum = 0;

		double sum = 0;
		for (int i = 0; i < normalVector1.size(); i++) {

			vector<double> nv1 = normalVector1[i];
			vector<double> nv2 = normalVector2[i];
			vector<double> nv2_neg;
			nv2_neg.push_back(-nv2[0]);
			nv2_neg.push_back(-nv2[1]);
			nv2_neg.push_back(-nv2[2]);

			if ((nv1[0] == 0 && nv1[1] == 0 && nv1[2] == 0) || (nv2[0] == 0 && nv2[1] == 0 && nv2[2] == 0)) {
				//cout << i << endl;
				singlePointNum++;
				continue;
			}


			double dotProduct = nv1[0] * nv2[0] + nv1[1] * nv2[1] + nv1[2] * nv2[2];
			double dotProduct_neg = nv1[0] * nv2_neg[0] + nv1[1] * nv2_neg[1] + nv1[2] * nv2_neg[2];
			double disProduct = sqrt(nv1[0] * nv1[0] + nv1[1] * nv1[1] + nv1[2] * nv1[2]) *
				sqrt(nv2[0] * nv2[0] + nv2[1] * nv2[1] + nv2[2] * nv2[2]);

			double h_neg = dotProduct_neg / disProduct;
			double h = dotProduct / disProduct;
			if (h_neg > 1) {
				h_neg = 1;
			}
			if (h_neg < -1) {
				h_neg = -1;
			}
			if (h > 1) {
				h = 1;
			}
			if (h < -1) {
				h = -1;
			}

			double angle_i_neg = acos(h_neg);
			double angle_i = acos(h);
			if (angle_i_neg < angle_i) {
				angle_i = angle_i_neg;
			}
			if (!(angle_i > -4 && angle_i < 4)) {
				cout << "hello!" << endl;

			}
			sum = sum + angle_i;
		}

		cout << "Single Point Num:" << singlePointNum << endl;
		double ave = sum / (double)(normalVector1.size() - singlePointNum);

		return ave;

	}

	bool normalLoad_store() {

		ifstream fin(fileNormal);
		if (fin)
		{
			if (normalVector.size() > 0) {
				normalVector.clear();
			}
			int numSum;
			fin >> numSum;
			for (int i = 0; i < numSum; i++) {
				double x_i;
				double y_i;
				double z_i;
				fin >> x_i >> y_i >> z_i;
				vector<double> p_n_i;
				p_n_i.push_back(x_i);
				p_n_i.push_back(y_i);
				p_n_i.push_back(z_i);
				normalVector.push_back(p_n_i);
			}
			fin.close();
			return true;
		}
		else {
			fin.close();
			return false;
		}

	}
	
	void normalSave(vector<vector<double>> n) {

		if (fileNormal.size() == 2) {
			cout << "normal file name is empty!" << endl;
		}
		else {
			ofstream fout(fileNormal, ios::app);
			fout << n.size() << endl;
			for (int i = 0; i < n.size(); i++) {
				fout << n[i][0] << " " << n[i][1] << " " << n[i][2] << endl;
			}
			fout << endl;
			fout.close();
		}

	}

	vector<vector<double>> estimateNormal_RegularNormal(vector<vector<double>> pointCloudData, vector<vector<double>> pointNormal) {

		int start = 0;
		std::cout << "RegularNormal start:" << std::endl;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudData.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudData.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudData[i][0];
			cloud->points[i].y = pointCloudData[i][1];
			cloud->points[i].z = pointCloudData[i][2];

		}
		kdtree.setInputCloud(cloud);

		std::vector<std::vector<int>> pointNeibor(pointCloudData.size());//neibor points in Ball for every points

		int Kn = 8;

#pragma omp parallel for
		for (int i = 0; i < pointCloudData.size(); i++) {
			if (i % 10000 == 0) {
				std::cout << (pointCloudData.size() - i) / 10000 << ",";
			}
			std::vector<int> pointIdxNKNSearch(Kn);
			std::vector<float> pointNKNSquaredDistance(Kn);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = pointCloudData[i][0];
			searchPoint.y = pointCloudData[i][1];
			searchPoint.z = pointCloudData[i][2];
			kdtree.nearestKSearch(searchPoint, Kn, pointIdxNKNSearch, pointNKNSquaredDistance);
			double x1 = pointCloudData[pointIdxNKNSearch[1]][0];
			double y1 = pointCloudData[pointIdxNKNSearch[1]][1];
			double z1 = pointCloudData[pointIdxNKNSearch[1]][2];
			vector<double> ppp;
			ppp.push_back(x1);
			ppp.push_back(y1);
			ppp.push_back(z1);
			if (pointNKNSquaredDistance[0] == 0) {
				pointNeibor_i.insert(pointNeibor_i.end(), pointIdxNKNSearch.begin() + 1, pointIdxNKNSearch.end());
			}
			else {
				pointNeibor_i.insert(pointNeibor_i.end(), pointIdxNKNSearch.begin(), pointIdxNKNSearch.end() - 1);
			}
			pointNeibor[i].insert(pointNeibor[i].end(), pointNeibor_i.begin(), pointNeibor_i.end());
		}
		std::cout << endl;
		vector<vector<Point_Index>> normal_store;
		vector<bool> normalJudge(pointCloudData.size());
		for (int i = 0; i < normalJudge.size(); i++) {
			normalJudge[i] = false;
		}
		normalJudge[start] = true;
		vector<Point_Index> normal_start;
		//normal_start.push_back(pointNormal[start][0]);
		//normal_start.push_back(pointNormal[start][1]);
		//normal_start.push_back(pointNormal[start][2]);
		Point_Index pi(start, start);
		normal_start.push_back(pi);//parent point index		
		normal_store.push_back(normal_start);
		while (1) {
			// 1. achieve source points
			vector<Point_Index> norProcessNow = normal_store[normal_store.size() - 1];
			vector<Point_Index> norProcessNow_Neibor;

			// 2. achieve neibor of source points
			for (int i = 0; i < norProcessNow.size(); i++) {
				vector<int> neibor_i = pointNeibor[norProcessNow[i].index];
				for (int j = 0; j < neibor_i.size(); j++) {
					if (normalJudge[neibor_i[j]] == false) {
						Point_Index pij(norProcessNow[i].index, neibor_i[j]);
						//remove repeat
						bool repeatJudge = false;
						for (int k = 0; k < norProcessNow_Neibor.size(); k++) {
							if (norProcessNow_Neibor[k].index == neibor_i[j]) {
								repeatJudge = true;
							}
						}
						if (!repeatJudge) {
							norProcessNow_Neibor.push_back(pij);
						}
					}
				}
			}

			// 3. update normal
			if (norProcessNow_Neibor.size() == 0) {
				break;
			}
			else {
				normal_store.push_back(norProcessNow_Neibor);
			}
			for (int i = 0; i < norProcessNow_Neibor.size(); i++) {
				vector<double> n_parent = pointNormal[norProcessNow_Neibor[i].parentIndex];
				vector<double> n_itself = pointNormal[norProcessNow_Neibor[i].index];
				double a1 = n_parent[0] * n_itself[0] + n_parent[1] * n_itself[1] + n_parent[2] * n_itself[2];
				double a2 = -n_parent[0] * n_itself[0] - n_parent[1] * n_itself[1] - n_parent[2] * n_itself[2];
				if (a1 > 1) {
					a1 = 1;
				}
				if (a1 < -1) {
					a1 = -1;
				}
				if (a2 > 1) {
					a2 = 1;
				}
				if (a2 < -1) {
					a2 = -1;
				}
				if (acos(a1) > acos(a2)) {
					pointNormal[norProcessNow_Neibor[i].index][0] = -pointNormal[norProcessNow_Neibor[i].index][0];
					pointNormal[norProcessNow_Neibor[i].index][1] = -pointNormal[norProcessNow_Neibor[i].index][1];
					pointNormal[norProcessNow_Neibor[i].index][2] = -pointNormal[norProcessNow_Neibor[i].index][2];
				}
				normalJudge[norProcessNow_Neibor[i].index] = true;
			}
			normal_store.push_back(norProcessNow_Neibor);
		}
		//cout << endl;	
		std::cout << "RegularNormal end:" << std::endl;
		return pointNormal;
	}

};

