#pragma once

#include <iostream>
#include "Method_AIVS_OM.hpp"
#include "Method_CGAL_Advancing.hpp"
#include "VD.hpp"
using namespace std;


class AIVS_Reconstruct {

private:

	BallRegion br;
	double h;
	double weightPointUpdata = 0.6;//update weight for seedpoints

public:

	void AIVS_Reconstruct_init(BallRegion brinput) {

		br = brinput;
		h = brinput.radius;

	}

	void AIVS_Reconstruct_Remesh(int pointNum, string fileName) {

		cout << "AIVS_Reconstruct start." << endl;
		vector<vector<double>> seedPoints;
		simplification_Method_AIVS_OM sma;

		//0. simplification
		if (br.pointCloudData.size()< (pointNum * 3)) {
			cout << "0. AIVS_Reconstruct up-sampling:" << endl;
			vector<vector<vector<double>>> pN = AIVS_Reconstruct_UpPointsTran(br.pointCloudData);
			vector<int> borderIndex = AIVS_Reconstruct_BorderReturn(pN[0]);
			BallRegion brUp;
			brUp.BallRegion_init(pN[0], pN[1], borderIndex);
			cout << "1. AIVS_Reconstruct Simplification:" << endl;
			sma.simplification_Method_AIVS_init(brUp);
			seedPoints = sma.AIVS_simplification(pointNum);		
		}
		else {
			cout << "1. AIVS_Reconstruct Simplification:" << endl;
			sma.simplification_Method_AIVS_init(br);
			seedPoints = sma.AIVS_simplification(pointNum);		
		}

		//1. Simplification		
		
		//vector<vector<double>> seedPoints = AIVS_Reconstruct_Optimization(seedPoints1, pointNum, 20);

		//2. Meshing
		cout << "2. AIVS_Reconstruct Meshing:" << endl;
		CGAL_Advancing_Reconstruct cr;
		cr.CGAL_Advancing_Remesh_init(seedPoints);
		vector<vector<int>> faceInfor = cr.CGAL_Advancing_Remesh_FaceInfor();		

		//3. Check the wrong trangular		
		cout << "3. AIVS_Reconstruct Face Detect:" << endl;
		vector<vector<int>> faceInforN = AIVS_Reconstruct_FaceRemove(faceInfor, seedPoints);

		//3. Store FIle	
		cout << "4. AIVS_Reconstruct Store File:" << endl;
		string offName = "Remesh\\" + fileName + ".off";
		string objName = "Remesh\\" + fileName + ".obj";
		AIVS_SaveOFF(offName, seedPoints, faceInforN);
		AIVS_SaveOBJ(objName, seedPoints, faceInforN);
		cout << "AIVS_Reconstruct finish." << endl;
	}

private:

	//******************point cloud increase********************

	vector<vector<vector<double>>> AIVS_Reconstruct_IncreasePoints(vector<vector<double>> p, int Upara) {
		
		//Upara is the increase parameter		
		double UparaStep = (double)1 / (double)(Upara * 2 + 1);
		int IterNum0 = p.size();
		int GlobalIter = 2;
		vector<vector<double>> pNN = p;
		while (GlobalIter > 0) {
			GlobalIter--;
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed = AIVS_KdTreeConstruct(p);//kdtree
			vector<vector<double>> pAdd;
			int kn = 8;
			for (int i = 0; i < pNN.size(); i++) {
				if (i % 1000 == 0) {
					cout << (pNN.size() - i) / 1000 << " ";
				}
				vector<double> seedPoints_i = pNN[i];
				vector<int> pointIdxNKNSearch(kn);
				vector<float> pointNKNSquaredDistance(kn);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = seedPoints_i[0];
				searchPoint.y = seedPoints_i[1];
				searchPoint.z = seedPoints_i[2];
				kdtreeSeed.nearestKSearch(searchPoint, kn, pointIdxNKNSearch, pointNKNSquaredDistance);

				
				for (int j = 1; j < pointIdxNKNSearch.size(); j++) {
					int indexij = pointIdxNKNSearch[j];
					vector<double> pNeiborj = p[indexij];
					for (int k = 0; k < Upara; k++) {
						vector<double> pNew_ijk(3);
						double weight = UparaStep * (k + 1);
						pNew_ijk[0] = pNeiborj[0] * weight + seedPoints_i[0] * (1 - weight);
						pNew_ijk[1] = pNeiborj[1] * weight + seedPoints_i[1] * (1 - weight);
						pNew_ijk[2] = pNeiborj[2] * weight + seedPoints_i[2] * (1 - weight);
						pAdd.push_back(pNew_ijk);
					}
				}
			}
			pNN.clear();			
			pNN = pAdd;
			p.insert(p.end(), pAdd.begin(), pAdd.end());		
		}		

		vector<vector<double>> pNormal(p.size());

#pragma omp parallel for
		for (int i = 0; i < p.size(); i++) {
			if (i % 1000 == 0) {
				cout << (p.size() - i) / 1000 << " ";
			}
			vector<double> pNewi =  AIVS_pointUpdate(p[i]);
			p[i][0] = pNewi[0];
			p[i][1] = pNewi[1];
			p[i][2] = pNewi[2];
			vector<double> pNi(3);
			pNi[0] = pNewi[3];
			pNi[1] = pNewi[4];
			pNi[2] = pNewi[5];
			pNormal[i] = pNi;		
		}

		//remove repeat points
		vector<vector<double>> pRemoveRepeat;
		vector<vector<double>> pRemoveRepeatN;

		int kn = 2;		
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed = AIVS_KdTreeConstruct(p);//kdtree
		for (int i = 0; i < p.size(); i++) {

			vector<double> seedPoints_i = p[i];
			vector<int> pointIdxNKNSearch(kn);
			vector<float> pointNKNSquaredDistance(kn);
			std::vector<int> pointNeibor_i;
			pcl::PointXYZ searchPoint;
			searchPoint.x = seedPoints_i[0];
			searchPoint.y = seedPoints_i[1];
			searchPoint.z = seedPoints_i[2];
			kdtreeSeed.nearestKSearch(searchPoint, kn, pointIdxNKNSearch, pointNKNSquaredDistance);

			if (pointNKNSquaredDistance[1] != 0) {
				pRemoveRepeat.push_back(p[i]);
				pRemoveRepeatN.push_back(pNormal[i]);
				
			}
		}

		vector<vector<vector<double>>> result;
		result.push_back(pRemoveRepeat);
		result.push_back(pRemoveRepeatN);

		return result;
	}

	vector<vector<vector<double>>> AIVS_Reconstruct_UpPointsTran(vector<vector<double>> p) {

		cout << "increase the points number from the point cloud" << endl;

		//1. achieve mesh
		CGAL_Advancing_Reconstruct crt;
		crt.CGAL_Advancing_Remesh_init(p);
		vector<vector<int>> faceInfor = crt.CGAL_Advancing_Remesh_FaceInfor();

		//2. achieve regular area
		vector<int> faceInfor1 = faceInfor[1];

		vector<vector<double>> b1;
		b1.push_back(p[faceInfor1[0]]);
		b1.push_back(p[faceInfor1[1]]);
		b1.push_back(p[faceInfor1[2]]);

		vector<int> faceInfor2 = faceInfor[2];
		vector<vector<double>> b2;
		b2.push_back(p[faceInfor2[0]]);
		b2.push_back(p[faceInfor2[1]]);
		b2.push_back(p[faceInfor2[2]]);

		vector<int> faceInfor3 = faceInfor[3];
		vector<vector<double>> b3;
		b3.push_back(p[faceInfor3[0]]);
		b3.push_back(p[faceInfor3[1]]);
		b3.push_back(p[faceInfor3[2]]);

		double avergeN = (AIVS_Reconstruct_AchieveArea(b1) + 
			AIVS_Reconstruct_AchieveArea(b2) + AIVS_Reconstruct_AchieveArea(b3))/3;
		double points = 6.0;

		//3. achieve regular area
		for (int i = 0; i < faceInfor.size(); i++) {
			vector<int> faceInfor1 = faceInfor[i];
			vector<vector<double>> bi;
			bi.push_back(p[faceInfor1[0]]);
			bi.push_back(p[faceInfor1[1]]);
			bi.push_back(p[faceInfor1[2]]);	
			double areai = AIVS_Reconstruct_AchieveArea(bi);
			int N = 6.0 * areai / avergeN;
			vector<vector<double>> pnewPoint = AIVS_Reconstruct_NewPoint(bi, N);
			p.insert(p.end(), pnewPoint.begin(), pnewPoint.end());
		}

		//4. update points positions 
		vector<vector<double>> pNormal(p.size());

#pragma omp parallel for
		for (int i = 0; i < p.size(); i++) {
			if (i % 1000 == 0) {
				cout << (p.size() - i) / 1000 << " ";
			}
			vector<double> pNewi = AIVS_pointUpdate(p[i]);
			p[i][0] = pNewi[0];
			p[i][1] = pNewi[1];
			p[i][2] = pNewi[2];
			vector<double> pNi(3);
			pNi[0] = pNewi[3];
			pNi[1] = pNewi[4];
			pNi[2] = pNewi[5];
			pNormal[i] = pNi;
		}

		//remove repeat points
		vector<vector<double>> pRemoveRepeat;
		vector<vector<double>> pRemoveRepeatN;

		int kn = 14;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed = AIVS_KdTreeConstruct(p);//kdtree
		for (int i = 0; i < p.size(); i++) {
			if ((p.size() - i) % 1000 == 0) {
				cout << (p.size() - i) / 1000 << " ";			
			}
			if (p[i][0] == -9999) {
				continue;			
			}
			else {
				vector<double> seedPoints_i = p[i];
				vector<int> pointIdxNKNSearch(kn);
				vector<float> pointNKNSquaredDistance(kn);
				std::vector<int> pointNeibor_i;
				pcl::PointXYZ searchPoint;
				searchPoint.x = seedPoints_i[0];
				searchPoint.y = seedPoints_i[1];
				searchPoint.z = seedPoints_i[2];
				kdtreeSeed.nearestKSearch(searchPoint, kn, pointIdxNKNSearch, pointNKNSquaredDistance);

				if (pointNKNSquaredDistance[1] != 0) {
					pRemoveRepeat.push_back(p[i]);
					pRemoveRepeatN.push_back(pNormal[i]);
				}
				else {
					int index = pointIdxNKNSearch[1];
					if (i > index) {
						//cout << "Error! i = " << i << endl;					
					}
					else {
						pRemoveRepeat.push_back(p[i]);
						pRemoveRepeatN.push_back(pNormal[i]);
						p[index][0] = -9999;
					}				
				}			
			}			
		}

		vector<vector<vector<double>>> result;
		result.push_back(pRemoveRepeat);
		result.push_back(pRemoveRepeatN);

		return result;

	}

	double AIVS_Reconstruct_AchieveArea(vector<vector<double>> vPoint) {
		
		double x0 = vPoint[0][0];
		double y0 = vPoint[0][1];
		double z0 = vPoint[0][2];

		double x1 = vPoint[1][0];
		double y1 = vPoint[1][1];
		double z1 = vPoint[1][2];

		double x2 = vPoint[2][0];
		double y2 = vPoint[2][1];
		double z2 = vPoint[2][2];

		double b1 = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1)* (z0 - z1));
		double b2 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2) + (z0 - z2) * (z0 - z2));
		double b3 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));

		double p = (b1 + b2 + b3) / 2;
		double tArea = sqrt(p * (p - b1) * (p - b2) * (p - b3));	
		return tArea;	
	}

	//New sampling point from tragular
	vector<vector<double>> AIVS_Reconstruct_NewPoint(vector<vector<double>> vPoint, int N) {
		//N is area parameter
		vector<vector<double>> newPoint;

		//init three points in trangular 
		double x0 = vPoint[0][0];
		double y0 = vPoint[0][1];
		double z0 = vPoint[0][2];

		double x1 = vPoint[1][0];
		double y1 = vPoint[1][1];
		double z1 = vPoint[1][2];

		double x2 = vPoint[2][0];
		double y2 = vPoint[2][1];
		double z2 = vPoint[2][2];

		//compute border point 
		int Q = sqrt(2 * N + 0.5) - 1.414 / 2 + 1;
		
		vector<vector<double>> b1;
		vector<vector<double>> b2;

		for (int i = 1; i < Q; i++) {

			vector<double> b1i(3);
			b1i[0] = x0 * (1 - (double)i / (double)Q) + x1 * ((double)i / (double)Q);
			b1i[1] = y0 * (1 - (double)i / (double)Q) + y1 * ((double)i / (double)Q);
			b1i[2] = z0 * (1 - (double)i / (double)Q) + z1 * ((double)i / (double)Q);
			b1.push_back(b1i);

			vector<double> b2i(3);
			b2i[0] = x0 * (1 - (double)i / (double)Q) + x2 * ((double)i / (double)Q);
			b2i[1] = y0 * (1 - (double)i / (double)Q) + y2 * ((double)i / (double)Q);
			b2i[2] = z0 * (1 - (double)i / (double)Q) + z2 * ((double)i / (double)Q);
			b2.push_back(b2i);
		
		}
		vector<vector<double>> pMiddlePoints;
		for (int i = 0; i < b1.size(); i++) {
			int seg = i + 2;
			vector<double> b1i = b1[i];
			vector<double> b2i = b2[i];
			for (int j = 0; j < i + 1; j++) {
				double weightij = (double)(j + 1) / (double)seg;
				vector<double> newij(3);
				newij[0] = b1i[0] * (1 - weightij) + b2i[0] * weightij;
				newij[1] = b1i[1] * (1 - weightij) + b2i[1] * weightij;
				newij[2] = b1i[2] * (1 - weightij) + b2i[2] * weightij;
				newPoint.push_back(newij);
			}		
		}
		newPoint.insert(newPoint.end(), b1.begin(), b1.end());
		newPoint.insert(newPoint.end(), b2.begin(), b2.end());
		return newPoint;
	}

	vector<int> AIVS_Reconstruct_BorderReturn(vector<vector<double>> p) {
		
		vector<int> indexBorder;

		double maxx, maxy, maxz, minx, miny, minz;

		maxx = minx = p[0][0];
		maxy = miny = p[0][1];
		maxz = minz = p[0][2];

		int indexMaxX = -1;
		int indexMaxY = -1;
		int indexMaxZ = -1;
		int indexMinX = -1;
		int indexMinY = -1;
		int indexMinZ = -1;

		for (int i = 0; i < p.size(); i++) {
			double xi = p[i][0];
			double yi = p[i][1];
			double zi = p[i][2];
			if (xi < minx) {
				minx = xi;
				indexMinX = i;
			}
			if (xi > maxx) {
				maxx = xi;
				indexMaxX = i;
			}
			if (yi < miny) {
				miny = yi;
				indexMinY = i;
			}
			if (yi > maxy) {
				maxy = yi;
				indexMaxY = i;
			}
			if (zi < minz) {
				minz = zi;
				indexMinZ = i;
			}
			if (zi > maxz) {
				maxz = zi;
				indexMaxZ = i;
			}
		}
		indexBorder.push_back(indexMinX);
		indexBorder.push_back(indexMinY);
		indexBorder.push_back(indexMinZ);
		indexBorder.push_back(indexMaxX);
		indexBorder.push_back(indexMaxY);
		indexBorder.push_back(indexMaxZ);
		return indexBorder;	
	}
	
	//****************point cloud increase end********************
	
	vector<vector<int>> AIVS_Reconstruct_FaceRemove(vector<vector<int>> faceInfor, vector<vector<double>> seedPoints) {

		vector<vector<int>> faceInforN;
		for (int i = 0; i < faceInfor.size(); i++) {	

			vector<double> b0(3);
			b0[0] = seedPoints[faceInfor[i][0]][0];
			b0[1] = seedPoints[faceInfor[i][0]][1];
			b0[2] = seedPoints[faceInfor[i][0]][2];

			vector<double> b1(3);
			b1[0] = seedPoints[faceInfor[i][1]][0];
			b1[1] = seedPoints[faceInfor[i][1]][1];
			b1[2] = seedPoints[faceInfor[i][1]][2];

			vector<double> b2(3);
			b2[0] = seedPoints[faceInfor[i][2]][0];
			b2[1] = seedPoints[faceInfor[i][2]][1];
			b2[2] = seedPoints[faceInfor[i][2]][2];

			double b01 = sqrt((b0[0] - b1[0]) * (b0[0] - b1[0])
				+ (b0[1] - b1[1]) * (b0[1] - b1[1])
				+ (b0[2] - b1[2]) * (b0[2] - b1[2]));

			double b12 = sqrt((b2[0] - b1[0]) * (b2[0] - b1[0])
				+ (b2[1] - b1[1]) * (b2[1] - b1[1])
				+ (b2[2] - b1[2]) * (b2[2] - b1[2]));

			double b02 = sqrt((b0[0] - b2[0]) * (b0[0] - b2[0])
				+ (b0[1] - b2[1]) * (b0[1] - b2[1])
				+ (b0[2] - b2[2]) * (b0[2] - b2[2]));

			double scale = br.unitSize * 1.5;
			if (b01 > scale || b12 > scale|| b02 > scale)
            {
				continue;
			}
			else {
				faceInforN.push_back(faceInfor[i]);			
			}
		}
		return faceInforN;

	
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZ> AIVS_KdTreeConstruct(vector<vector<double>> seedpoints) {

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSeed;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = seedpoints.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < seedpoints.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = seedpoints[i][0];
			cloud->points[i].y = seedpoints[i][1];
			cloud->points[i].z = seedpoints[i][2];

		}
		kdtreeSeed.setInputCloud(cloud);
		return kdtreeSeed;
	}

	vector<double> AIVS_pointUpdate(vector<double> point_i) {

		int iter = 10;


#pragma region Achieve Neibor
		//Achieve Neibor
		int K = br.pointNumEsti + 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		std::vector<int> pointNeior;
		//double r_i = pointNKNSquaredDistance[pointNKNSquaredDistance.size() - 1];
		pcl::PointXYZ searchPoint;
		searchPoint.x = point_i[0];
		searchPoint.y = point_i[1];
		searchPoint.z = point_i[2];
		br.kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		if (pointNKNSquaredDistance[0] == 0) {
			point_i.push_back(br.pointNormal[pointIdxNKNSearch[0]][0]);
			point_i.push_back(br.pointNormal[pointIdxNKNSearch[0]][1]);
			point_i.push_back(br.pointNormal[pointIdxNKNSearch[0]][2]);
			return point_i;
		}
		else {
			pointNeior.insert(pointNeior.end(), pointIdxNKNSearch.begin(), pointIdxNKNSearch.end() - 1);
		}


		//vector<int> pointNeior = br.pointNeibor[i];
		vector<vector<double>> pointNormal_i(pointNeior.size());
		pointNormal_i[0] = br.pointNormal[pointNeior[0]];
		for (int j = 1; j < pointNeior.size(); j++) {
			//vector<double> n_j = br.pointNormal[pointNeior[j]];
			pointNormal_i[j] = br.pointNormal[pointNeior[j]];
		}

#pragma endregion

#pragma region MLS error
		//++++++++++++++++++++interater start+++++++++++++++++++++++++++
		double errorExist = 0.0001;
		vector<double> px;
		px.insert(px.end(), point_i.begin(), point_i.end());
		//vector<int> p_neibor = br.pointNeibor[i];			
		//regularNoraml

		vector<double> px_store(3);
		vector<double> nx_store(3);
		vector<double> ax;//new point position
		ax.push_back(0);
		ax.push_back(0);
		ax.push_back(0);
		vector<double> nx;//new point normal
		nx.push_back(0);
		nx.push_back(0);
		nx.push_back(0);
		double errorEndTem;//record new 
		double errorStore = 9999;
		double weight;
		while (iter) {
			//vector<double> ax = simMeasurement_cop_a(px, pointNeiborRegualrNum);
			//vector<double> nx = simMeasurement_cop_n(px, pointNeiborRegualrNum, pointNeiborNormalRegualrNum);
			double fenmu = 0;
			for (int j = 0; j < pointNeior.size(); j++) {
				double dis_i = sqrt((br.pointCloudData[pointNeior[j]][0] - px[0]) * (br.pointCloudData[pointNeior[j]][0] - px[0]) +
					(br.pointCloudData[pointNeior[j]][1] - px[1]) * (br.pointCloudData[pointNeior[j]][1] - px[1]) +
					(br.pointCloudData[pointNeior[j]][2] - px[2]) * (br.pointCloudData[pointNeior[j]][2] - px[2]));
				if (dis_i == 0) {
					continue;
				}
				double eData = -((dis_i / h) * (dis_i / h));
				eData = exp(eData);
				ax[0] = ax[0] + br.pointCloudData[pointNeior[j]][0] * eData;
				ax[1] = ax[1] + br.pointCloudData[pointNeior[j]][1] * eData;
				ax[2] = ax[2] + br.pointCloudData[pointNeior[j]][2] * eData;
				nx[0] = nx[0] + br.pointNormal[pointNeior[j]][0] * eData;
				nx[1] = nx[1] + br.pointNormal[pointNeior[j]][1] * eData;
				nx[2] = nx[2] + br.pointNormal[pointNeior[j]][2] * eData;
				fenmu = fenmu + eData;
			}
			if (fenmu != 0) {
				ax[0] = ax[0] / fenmu;
				ax[1] = ax[1] / fenmu;
				ax[2] = ax[2] / fenmu;
				nx[0] = nx[0] / fenmu;
				nx[1] = nx[1] / fenmu;
				nx[2] = nx[2] / fenmu;
			}
			fenmu = 0;

			//5.3 Set x' = x - n(x')T(a(x')-x)n(x'), weight = n(x')T(a(x')-x)
			weight = nx[0] * (point_i[0] - ax[0]) +
				nx[1] * (point_i[1] - ax[1]) + nx[2] * (point_i[2] - ax[2]);
			px_store[0] = px[0];
			px_store[1] = px[1];
			px_store[2] = px[2];
			nx_store[0] = nx[0];
			nx_store[1] = nx[1];
			nx_store[2] = nx[2];
			px[0] = point_i[0] - weight * nx[0];
			px[1] = point_i[1] - weight * nx[1];
			px[2] = point_i[2] - weight * nx[2];
			//5.4 ||n(x')T(a(x')-x)n(x')||>errorEndTem
			ax[0] = 0;
			ax[1] = 0;
			ax[2] = 0;
			nx[0] = 0;
			nx[1] = 0;
			nx[2] = 0;

			//vector<double> axnew = simMeasurement_cop_a(px, pointNeiborRegualrNum);
			//vector<double> nxnew = simMeasurement_cop_n(px, pointNeiborRegualrNum, pointNeiborNormalRegualrNum);

			for (int j = 0; j < pointNeior.size(); j++) {
				double dis_i = sqrt((br.pointCloudData[pointNeior[j]][0] - px[0]) * (br.pointCloudData[pointNeior[j]][0] - px[0]) +
					(br.pointCloudData[pointNeior[j]][1] - px[1]) * (br.pointCloudData[pointNeior[j]][1] - px[1]) +
					(br.pointCloudData[pointNeior[j]][2] - px[2]) * (br.pointCloudData[pointNeior[j]][2] - px[2]));
				if (dis_i == 0) {
					continue;
				}
				double eData = -((dis_i / h) * (dis_i / h));
				eData = exp(eData);
				ax[0] = ax[0] + br.pointCloudData[pointNeior[j]][0] * eData;
				ax[1] = ax[1] + br.pointCloudData[pointNeior[j]][1] * eData;
				ax[2] = ax[2] + br.pointCloudData[pointNeior[j]][2] * eData;
				nx[0] = nx[0] + br.pointNormal[pointNeior[j]][0] * eData;
				nx[1] = nx[1] + br.pointNormal[pointNeior[j]][1] * eData;
				nx[2] = nx[2] + br.pointNormal[pointNeior[j]][2] * eData;
				fenmu = fenmu + eData;
			}
			if (fenmu != 0) {
				ax[0] = ax[0] / fenmu;
				ax[1] = ax[1] / fenmu;
				ax[2] = ax[2] / fenmu;
				nx[0] = nx[0] / fenmu;
				nx[1] = nx[1] / fenmu;
				nx[2] = nx[2] / fenmu;
			}

			weight = nx[0] * (point_i[0] - ax[0]) +
				nx[1] * (point_i[1] - ax[1]) + nx[2] * (point_i[2] - ax[2]);

			ax[0] = 0;
			ax[1] = 0;
			ax[2] = 0;
			nx[0] = 0;
			nx[1] = 0;
			nx[2] = 0;

			errorEndTem = abs(weight);
			if (errorEndTem < errorStore) {
				errorStore = errorEndTem;
			}
			else {
				px[0] = px_store[0];
				px[1] = px_store[1];
				px[2] = px_store[2];
				break;
			}
			if (errorEndTem < errorExist) {//|| errorEndTem > errorstore
				break;
			}
			iter--;
		}
#pragma endregion 

		vector<double> finalResult(6);
		finalResult[0] = px[0];
		finalResult[1] = px[1];
		finalResult[2] = px[2];
		finalResult[3] = nx_store[0];
		finalResult[4] = nx_store[1];
		finalResult[5] = nx_store[2];
		return finalResult;

	}

	vector<double> AIVS_Centorid(vector<double> p, vector<vector<double>> pn) {

		vector<double> pT = CVT_Delaunay_Return(p, pn);
		vector<double> pPullBack = AIVS_pointUpdate(pT);
		return pPullBack;	
	}


	//CVT coordinate transfer
	vector<double> CVT_NormalVector(vector<double> v) {

		double l = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		v[0] = v[0] / l;
		v[1] = v[1] / l;
		v[2] = v[2] / l;
		return v;

	}

	vector<double> CVT_Global2local(vector<vector<double>> localAxis,
		vector<double> localCenter, vector<double> pG) {

		vector<double> xAxis = localAxis[0];
		vector<double> yAxis = localAxis[1];
		vector<double> zAxis = localAxis[2];

		vector<double> o(3);
		o[0] = -localCenter[0] * xAxis[0] - localCenter[1] * xAxis[1] - localCenter[2] * xAxis[2];
		o[1] = -localCenter[0] * yAxis[0] - localCenter[1] * yAxis[1] - localCenter[2] * yAxis[2];
		o[2] = -localCenter[0] * zAxis[0] - localCenter[1] * zAxis[1] - localCenter[2] * zAxis[2];


		double plx = pG[0] * xAxis[0] + pG[1] * xAxis[1] + pG[2] * xAxis[2] + o[0];
		double ply = pG[0] * yAxis[0] + pG[1] * yAxis[1] + pG[2] * yAxis[2] + o[1];
		double plz = pG[0] * zAxis[0] + pG[1] * zAxis[1] + pG[2] * zAxis[2] + o[2];

		vector<double> pl;
		pl.push_back(plx);
		pl.push_back(ply);
		pl.push_back(plz);

		return pl;

	}

	vector<double> CVT_Local2Global(vector<vector<double>> localAxis,
		vector<double> localCenter, vector<double> pl) {

		vector<double> xAxis = localAxis[0];
		vector<double> yAxis = localAxis[1];
		vector<double> zAxis = localAxis[2];

		double pGx = pl[0] * xAxis[0] + pl[1] * yAxis[0] + pl[2] * zAxis[0] + localCenter[0];
		double pGy = pl[0] * xAxis[1] + pl[1] * yAxis[1] + pl[2] * zAxis[1] + localCenter[1];
		double pGz = pl[0] * xAxis[2] + pl[1] * yAxis[2] + pl[2] * zAxis[2] + localCenter[2];

		vector<double> pG;
		pG.push_back(pGx);
		pG.push_back(pGy);
		pG.push_back(pGz);

		return pG;


	}

	vector<double> CVT_Point2Plane(vector<double> p, vector<double> pn,
		vector<double> sourceP) {//map sourceP to the plane

		double A = pn[0];
		double B = pn[1];
		double C = pn[2];
		double D = -A * p[0] - B * p[1] - C * p[2];

		double Q = A * sourceP[0] + B * sourceP[1] + C * sourceP[2];

		double landa = (Q + D) / (A * A + B * B + C * C);

		double xmap = sourceP[0] - A * landa;
		double ymap = sourceP[1] - B * landa;
		double zmap = sourceP[2] - C * landa;

		vector<double> targetP(3);
		targetP[0] = xmap;
		targetP[1] = xmap;
		targetP[2] = xmap;

	}

	vector<double> CVT_Delaunay_Return(vector<double> p, vector<vector<double>> p_neibor) {

		vector<double> p_c(3);
		vector<double> p_n(3);
		p_c[0] = p[0];
		p_c[1] = p[1];
		p_c[2] = p[2];
		p_n[0] = p[3];
		p_n[1] = p[4];
		p_n[2] = p[5];

		//1. tangent plane, local region
		vector<double> zLocal = CVT_NormalVector(p_n);
		vector<double> yDirection(3);
		yDirection[0] = p_neibor[p_neibor.size() - 1][0];
		yDirection[0] = p_neibor[p_neibor.size() - 1][1];
		yDirection[0] = p_neibor[p_neibor.size() - 1][2];
		vector<double> yLocal = CVT_NormalVector(yDirection);
		vector<double> xLocal(3);
		xLocal[0] = yLocal[1] * zLocal[2] - yLocal[2] * zLocal[1];
		xLocal[1] = zLocal[0] * yLocal[2] - yLocal[0] * zLocal[2];
		xLocal[2] = yLocal[0] * zLocal[1] - yLocal[1] * zLocal[0];
		yLocal[0] = xLocal[1] * zLocal[2] - xLocal[2] * zLocal[1];
		yLocal[1] = xLocal[2] * zLocal[0] - xLocal[0] * zLocal[2];
		yLocal[2] = xLocal[0] * zLocal[1] - xLocal[1] * zLocal[0];

		vector<vector<double>> localAxis;
		localAxis.push_back(xLocal);
		localAxis.push_back(yLocal);
		localAxis.push_back(zLocal);

		//2. map the point into the plane
		vector<vector<double>> planeData;
		for (int i = 0; i < p_neibor.size(); i++) {
			vector<double> p_neibor_i = p_neibor[i];
			vector<double> p_neibor_iN = CVT_Global2local(localAxis, p, p_neibor_i);
			vector<double> p_neibor_iN2D(2);
			p_neibor_iN2D[0] = p_neibor_iN[0];
			p_neibor_iN2D[1] = p_neibor_iN[1];
			planeData.push_back(p_neibor_iN2D);
		}
		double r = sqrt((planeData[0][0] - planeData[planeData.size() - 1][0]) *
			(planeData[0][0] - planeData[planeData.size() - 1][0])
			+ (planeData[0][1] - planeData[planeData.size() - 1][1]) *
			(planeData[0][1] - planeData[planeData.size() - 1][1]));

		vector<vector<double>> planeDataStore = planeData;

		if (true) {
			vector<double> p1; p1.push_back(0);  p1.push_back(r);
			vector<double> p2; p2.push_back(-r); p2.push_back(r);
			vector<double> p3; p3.push_back(-r); p3.push_back(0);
			vector<double> p4; p4.push_back(-r); p4.push_back(-r);
			vector<double> p5; p5.push_back(0);	 p5.push_back(-r);
			vector<double> p6; p6.push_back(r);  p6.push_back(-r);
			vector<double> p7; p7.push_back(r);  p7.push_back(0);
			vector<double> p8; p8.push_back(r);  p8.push_back(r);
			planeData.push_back(p1); planeData.push_back(p2); planeData.push_back(p3);
			planeData.push_back(p4); planeData.push_back(p5); planeData.push_back(p6);
			planeData.push_back(p7); planeData.push_back(p8);
		}
		VoronoiD vd;
		vd.VoronoiD_init(planeData);
		vector<double> center = vd.VoronoiD_Return_Delaunay_Mean(planeData[0], planeDataStore);
		center.push_back(0);
		vector<double> centerReal = CVT_Local2Global(localAxis, p, center);
		return centerReal;
	}

	//file store
	void AIVS_SaveOFF(string fileName, vector<vector<double>> points, vector<vector<int>> facet) {

		ofstream f1(fileName, ios::app);
		f1 << "OFF" << endl;

		f1 << points.size() << " " << facet.size() << " " << 0 << endl;

		for (int i = 0; i < points.size(); i++) {
			f1 << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << endl;
		}

		for (int i = 0; i < facet.size(); i++) {
			f1 << 3 << " " << facet[i][0] << " " << facet[i][1] << " " << facet[i][2] << endl;
		}

		f1.close();


	}

	void AIVS_SaveOBJ(string fileName, vector<vector<double>> points, vector<vector<int>> facet) {

		ofstream f1(fileName, ios::app);

		//f1 << points.size() << " " << facet.size() << " " << 0 << endl;

		for (int i = 0; i < points.size(); i++) {
			f1 << "v " << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << endl;
		}

		for (int i = 0; i < facet.size(); i++) {
			f1 << "f " << facet[i][0] + 1 << " " << facet[i][1] + 1 << " " << facet[i][2] + 1 << endl;
		}

		f1.close();


	}

};

	