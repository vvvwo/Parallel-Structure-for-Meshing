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
#include <time.h>
#include "ballRegionCompute.hpp"

class simplification_Method_AIVS_OM {

public:

	BallRegion br;	
	vector<vector<int>> boxIndexNumberCu;
	vector<int> labelG;

private:

	int iter = 10;

public:

	void simplification_Method_AIVS_init(BallRegion br_i) {
		br = br_i;		
		AIVS_initBoxIndexNumber();
	}
	   	 
	vector<vector<double>> AIVS_simplification(int pointNum) {		

		//achieve the point rate
		double rate = (double)pointNum / (double)br.pointCloudData.size();

		//1. 8 box list (vector<vector<int>>)
		boxIndexNumberCu;

		//2. box center (vector<vector<double>>)
		vector<vector<double>> boxcenter = br.squareBoxesCReal;
		vector<int> boxcenterIndex = br.squareBoxesCenter;
		//3. box scale (double)
		double boxScale = br.unitSize;

		//4. neibor box link (vector<vector<int>>)
		vector<vector<int>> neiborBox(br.squareBoxes.size());
		for (int i = 1; i < br.squareBoxes.size(); i++) {			
			vector<int> neiborBox_i = br.BallRegion_ReturnNeiborBox_Box(i);
			neiborBox[i] = neiborBox_i;
		}

		//5. the simplification number for each box
		vector<int> boxSimiNumer(br.squareBoxes.size());
		for (int i = 0; i < boxSimiNumer.size(); i++) {
			int numberb = br.squareBoxes[i].size();
			if (numberb == 0) {
				boxSimiNumer[i] = 0;
			}
			else {
				double numberT = (double)numberb * rate;
				int simnumber = numberT;
				if (numberT - (double)simnumber > 0.2) {
					simnumber = simnumber + 1;
				}
				boxSimiNumer[i] = simnumber;
			}
		}

		//6. the points infor in each box
		br.squareBoxes;

		//7. global label (vector<int>)		
		labelG.resize(br.pointCloudData.size(), 1);

		//8. point cloud data (vector<vector<double>>)
		br.pointCloudData;		
		
		//Voroni_OpenMP Start!
		
		vector<vector<int>> SimResult = AIVS_Voroni_OpenMP(//AIVS_Voroni_OpenMP( ; AIVS_Voroni_Cuda(
			boxIndexNumberCu,
			boxcenter,
			boxcenterIndex,
			boxScale,
			neiborBox,
			boxSimiNumer,
			br.squareBoxes,
			labelG,
			br.pointCloudData
		);//store the data

		//Final Step, to achieve the accurate points number

		vector<vector<double>> finalResult = AIVS_AccurateCut(SimResult, pointNum);//final simplification result
		    
		return finalResult;
	}	

private:

	void AIVS_initBoxIndexNumber() {

		int boxNum = br.squareBoxes.size();
		vector<int> xyzNum = br.XYZNumber;

		int simin = 9999;
		int simax = 0;


		vector<vector<int>> box(8);
		for (int i = 1; i <= xyzNum[0]; i++) {
			for (int j = 1; j <= xyzNum[1]; j++) {
				for (int k = 1; k <= xyzNum[2]; k++) {
					int index_Num = i + xyzNum[0] * (j - 1) + (xyzNum[0] * xyzNum[1] * (k - 1));
					if (br.squareBoxes[index_Num].size() == 0) {
						continue;					
					}
					//if (index_Num == 325) {
						//cout << "i:" << i << "j:" << j << "k:" << k << endl;
					//}
					//if (index_Num < simin) {
						//simin = index_Num;
					//}
					//if (index_Num > simax) {
						//simax = index_Num;
					//}
					if (i % 2 == 1 && j % 2 == 1 && k % 2 == 1) {
						box[0].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 1 && k % 2 == 1) {
						box[1].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 0 && k % 2 == 1) {
						box[2].push_back(index_Num);
					}
					else if (i % 2 == 1 && j % 2 == 0 && k % 2 == 1) {
						box[3].push_back(index_Num);
					}
					else if (i % 2 == 1 && j % 2 == 1 && k % 2 == 0) {
						box[4].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 1 && k % 2 == 0) {
						box[5].push_back(index_Num);
					}
					else if (i % 2 == 0 && j % 2 == 0 && k % 2 == 0) {
						box[6].push_back(index_Num);
					}
					else {
						box[7].push_back(index_Num);
					}
				}
			}
		}

		boxIndexNumberCu = box;

	}

	bool AIVS_searching(int index, vector<int> indexV) {

		for (int i = 0; i < indexV.size(); i++) {
			if (index == indexV[i]) {
				return true;
			}
		}
		return false;
	}

	//achieve the final accurate point number
	vector<vector<double>> AIVS_AccurateCut(vector<vector<int>> t, int pointNum) {

		vector<vector<double>> finalResult;

		int numberSum = 0;
		//achieve the number of block with max number
		int numberMax = 0;
		//achieve the number of block with min number
		int numberMin = 99999;
		vector<vector<int>> blockIndex;
		for (int i = 0; i < br.squareBoxes.size(); i++) {
			vector<int> blockIndexi;
			for (int j = 0; j < t[i].size(); j++) {
				int index = t[i][j];
				if (index != -1) {
					blockIndexi.push_back(index);
				}
				else {
					break;
				}
			}
			blockIndex.push_back(blockIndexi);
			numberSum = numberSum + blockIndexi.size();
			if (blockIndexi.size() < numberMin && blockIndexi.size() > 0) {
				numberMin = blockIndexi.size();
			}
			if (blockIndexi.size() > numberMax) {
				numberMax = blockIndexi.size();
			}
		}

		int numberMiddle = (numberMax - numberMin) / 3;
		int dTiff = numberSum - pointNum;


		if (dTiff <= 0) {
			for (int i = 0; i < blockIndex.size(); i++) {
				for (int j = 0; j < blockIndex[i].size(); j++) {
					vector<double> p = br.pointCloudData[blockIndex[i][j]];
					finalResult.push_back(p);
				}
			}
			return finalResult;
		}

		else {
			int iii = 0;
			vector<int> blockIndexNum(blockIndex.size());
			for (int i = 0; i < blockIndex.size(); i++) {
				blockIndexNum[i] = blockIndex[i].size();
			}
			bool singleJudge = false;
			int diffStore = dTiff;
			while (1) {				
				if (dTiff == 0) {
					break;
				}
				if (iii == blockIndex.size()) {
					iii = 0;
					singleJudge = !singleJudge;
					if (dTiff == diffStore) {
						numberMiddle--;					
					}
					else {
						diffStore = dTiff;					
					}
				}
				if (!singleJudge) {
					if (blockIndexNum[iii] > numberMiddle && iii % 2 == 0) {
						blockIndexNum[iii] --;
						dTiff--;
					}
				}
				else {
					if (blockIndexNum[iii] > numberMiddle && iii % 2 != 0) {
						blockIndexNum[iii] --;
						dTiff--;
					}
				}
				iii++;
			}
			for (int i = 0; i < blockIndex.size(); i++) {
				for (int j = 0; j < blockIndexNum[i]; j++) {
					vector<double> p = br.pointCloudData[blockIndex[i][j]];
					finalResult.push_back(p);
				}
			}
			return finalResult;
		}

	}
	
#pragma region AIVS_Voroni_Box
	//Simplification Core
	vector<vector<int>> AIVS_Voroni(
		vector<vector<int>> boxList_1,//8 box list for parallel computation
		vector<vector<double>> boxCenter_2,//the center points for each box 
		vector<int> boxCenterIndex_2,//the center index point in the box
		double boxScale_3,//the scle of the box
		vector<vector<int>> boxNeibor_4,
		vector<int> boxSimNum_5,
		vector<vector<int>> boxPoints_6,//the point index in the box
		vector<int> labelG_7,
		vector<vector<double>> pointData_8
	) {

		//init simiT for simplification result store
		vector<vector<int>> simiT(boxPoints_6.size());//store the raw simplification result
		for (int i = 0; i < simiT.size(); i++) {
			vector<int> simiT_i(boxPoints_6[i].size(), -1);
			simiT[i] = simiT_i;
		}

		double searchBoxRadius = boxScale_3 * 3.0 / 4.0;

		for (int i = 0; i < boxPoints_6.size(); i++){				
			if ((boxPoints_6.size() - i) % 1000 == 0) {
				cout << "block:" << (boxPoints_6.size() - i)/1000 << endl;
			}
			int boxIndex = i;
			int simNum = boxSimNum_5[boxIndex];
			if (simNum == 0) {
				continue;
			}	
			
			//int boxNeibirIndex = boxNeibor_4[boxIndex]
			vector<double> pCenter = boxCenter_2[boxIndex];
			//achieve center point infor

			vector<int> pointTemp = boxPoints_6[boxIndex];
			vector<int> labelTemp(pointTemp.size(), 1);
			if (boxCenterIndex_2[boxIndex] >= -1 && boxCenterIndex_2[boxIndex] < pointTemp.size()) {
				labelTemp[boxCenterIndex_2[boxIndex]] = 0;
			}

			//search the neibor points into the block
			for (int k = 0; k < boxNeibor_4[boxIndex].size(); k++) {
				vector<int> pN = boxPoints_6[boxNeibor_4[boxIndex][k]];// achieve the points from neibor box
				for (int l = 0; l < pN.size(); l++) {
					vector<double> pn_l = pointData_8[pN[l]];
					if (pn_l[0] <= pCenter[0] + searchBoxRadius && pn_l[0] >= pCenter[0] - searchBoxRadius
						&& pn_l[1] <= pCenter[1] + searchBoxRadius && pn_l[1] >= pCenter[1] - searchBoxRadius
						&& pn_l[2] <= pCenter[2] + searchBoxRadius && pn_l[2] >= pCenter[2] - searchBoxRadius
						&& labelG_7[pN[l]] == 0) {
						pointTemp.push_back(pN[l]);
						labelTemp.push_back(2);
					}
				}
			}
			//start Voroni: //1. pointTemp  2. labelTemp 3. simNum
			int samplieIndex = 0;//record the  
			vector<double> mindistance(pointTemp.size(), -1);
			for (int k = 0; k < mindistance.size(); k++) {
				if (labelTemp[k] == 0) {
					mindistance[k] = 0;
					simiT[boxIndex][samplieIndex] = pointTemp[k];
					labelG_7[pointTemp[k]] = 0;
					samplieIndex++;
				}
				else if (labelTemp[k] == 2) {
					mindistance[k] = 0;
				}
				else {
					double minTemp = 9999;
					vector<double> pTemp = pointData_8[pointTemp[k]];					
					for (int l = 0; l < mindistance.size(); l++) {
						if (labelTemp[l] == 0 || labelTemp[l] == 2) {
							vector<double> pTemp_l = pointData_8[pointTemp[l]];
							
							double dis_Temp = sqrt(
								(pTemp[0] - pTemp_l[0]) * (pTemp[0] - pTemp_l[0]) +
								(pTemp[1] - pTemp_l[1]) * (pTemp[1] - pTemp_l[1]) +
								(pTemp[2] - pTemp_l[2]) * (pTemp[2] - pTemp_l[2])
							);
							if (dis_Temp < minTemp) {
								minTemp = dis_Temp;
							}
						}
					}
					mindistance[k] = minTemp;
				}
			}

			while (1) {				
				if (samplieIndex >= simNum) {
					break;
				}
				else {
					//select the point with the longest mindis
					int indexSelect = -1;
					double max_mindistance = 0;
					for (int k = 0; k < mindistance.size(); k++) {
						if (labelTemp[k] == 1 && mindistance[k] > max_mindistance) {
							indexSelect = k;
							max_mindistance = mindistance[k];
						}
					}

					//achieve new simplification point.
					mindistance[indexSelect] = 0;
					labelG_7[pointTemp[indexSelect]] = 0;
					simiT[boxIndex][samplieIndex] = pointTemp[indexSelect];
					vector<double> pTemp = pointData_8[pointTemp[indexSelect]];
					samplieIndex++;

					//updata mindistance.
					for (int k = 0; k < mindistance.size(); k++) {
						if (labelTemp[k] == 1) {
							vector<double> pTemp_k = pointData_8[pointTemp[k]];
							double dis_Temp = sqrt((pTemp_k[0] - pTemp[0]) * (pTemp_k[0] - pTemp[0]) +
								(pTemp_k[1] - pTemp[1]) * (pTemp_k[1] - pTemp[1]) +
								(pTemp_k[2] - pTemp[2]) * (pTemp_k[2] - pTemp[2]));
							if (dis_Temp < mindistance[k]) {
								mindistance[k] = dis_Temp;
							}
						}
					}


				}
			}
				//End Voroni


			//block end		
		}

		return simiT;

	}

	vector<vector<int>> AIVS_Voroni_OpenMP(
		vector<vector<int>> boxList_1,//8 box list for parallel computation
		vector<vector<double>> boxCenter_2,//the center points for each box 
		vector<int> boxCenterIndex_2,//the center index point in the box
		double boxScale_3,//the scle of the box
		vector<vector<int>> boxNeibor_4,
		vector<int> boxSimNum_5,
		vector<vector<int>> boxPoints_6,//the point index in the box
		vector<int> labelG_7,
		vector<vector<double>> pointData_8

	) {
		//init simiT for simplification result store
		vector<vector<int>> simiT(boxPoints_6.size());//store the raw simplification result
		for (int i = 0; i < simiT.size(); i++) {
			vector<int> simiT_i(boxPoints_6[i].size(), -1);
			simiT[i] = simiT_i;
		}
		double searchBoxRadius = boxScale_3 * 3.0 / 4.0;
		for (int i = 0; i < boxList_1.size(); i++) {			
			cout << "blockList:" << i << endl;
#pragma omp parallel for
			for (int j = 0; j < boxList_1[i].size(); j++) {
				int boxIndex = boxList_1[i][j];
				int simNum = boxSimNum_5[boxIndex];
				if (simNum == 0) {
					continue;
				}				
				//int boxNeibirIndex = boxNeibor_4[boxIndex]
				vector<double> pCenter = boxCenter_2[boxIndex];
				//achieve center point infor
				vector<int> pointTemp = boxPoints_6[boxIndex];
				vector<int> labelTemp(pointTemp.size(), 1);
				if (boxCenterIndex_2[boxIndex] >= -1 && boxCenterIndex_2[boxIndex] < pointTemp.size()) {
					labelTemp[boxCenterIndex_2[boxIndex]] = 0;
				}
				//search the neibor points into the block
				for (int k = 0; k < boxNeibor_4[boxIndex].size(); k++) {
					vector<int> pN = boxPoints_6[boxNeibor_4[boxIndex][k]];// achieve the points from neibor box
					for (int l = 0; l < pN.size(); l++) {
						vector<double> pn_l = pointData_8[pN[l]];
						if (pn_l[0] <= pCenter[0] + searchBoxRadius && pn_l[0] >= pCenter[0] - searchBoxRadius
							&& pn_l[1] <= pCenter[1] + searchBoxRadius && pn_l[1] >= pCenter[1] - searchBoxRadius
							&& pn_l[2] <= pCenter[2] + searchBoxRadius && pn_l[2] >= pCenter[2] - searchBoxRadius
							&& labelG_7[pN[l]] == 0) {
							pointTemp.push_back(pN[l]);
							labelTemp.push_back(2);
						}
					}
				}
				//start Voroni: //1. pointTemp  2. labelTemp 3. simNum
				int samplieIndex = 0;//record the  
				vector<double> mindistance(pointTemp.size(), -1);
				for (int k = 0; k < mindistance.size(); k++) {
					if (labelTemp[k] == 0) {
						mindistance[k] = 0;
						simiT[boxIndex][samplieIndex] = pointTemp[k];
						labelG_7[pointTemp[k]] = 0;
						samplieIndex++;
					}
					else if (labelTemp[k] == 2) {
						mindistance[k] = 0;
					}
					else {
						double minTemp = 9999;
						vector<double> pTemp = pointData_8[pointTemp[k]];
						for (int l = 0; l < mindistance.size(); l++) {
							if (labelTemp[l] == 0 || labelTemp[l] == 2) {
								vector<double> pTemp_l = pointData_8[pointTemp[l]];
								double dis_Temp = sqrt(
									(pTemp[0] - pTemp_l[0]) * (pTemp[0] - pTemp_l[0]) +
									(pTemp[1] - pTemp_l[1]) * (pTemp[1] - pTemp_l[1]) +
									(pTemp[2] - pTemp_l[2]) * (pTemp[2] - pTemp_l[2])
								);
								if (dis_Temp < minTemp) {
									minTemp = dis_Temp;
								}
							}
						}
						mindistance[k] = minTemp;
					}
				}
				while (1) {
					if (samplieIndex >= simNum) {
						break;
					}
					else {
						//select the point with the longest mindis
						int indexSelect = -1;
						double max_mindistance = 0;
						for (int k = 0; k < mindistance.size(); k++) {
							if (labelTemp[k] == 1 && mindistance[k] > max_mindistance) {
								indexSelect = k;
								max_mindistance = mindistance[k];
							}
						}
						//achieve new simplification point.
						mindistance[indexSelect] = 0;
						labelG_7[pointTemp[indexSelect]] = 0;
						simiT[boxIndex][samplieIndex] = pointTemp[indexSelect];
						vector<double> pTemp = pointData_8[pointTemp[indexSelect]];
						samplieIndex++;

						//updata mindistance.
						for (int k = 0; k < mindistance.size(); k++) {
							if (labelTemp[k] == 1) {
								vector<double> pTemp_k = pointData_8[pointTemp[k]];
								double dis_Temp = sqrt((pTemp_k[0] - pTemp[0]) * (pTemp_k[0] - pTemp[0]) +
									(pTemp_k[1] - pTemp[1]) * (pTemp_k[1] - pTemp[1]) +
									(pTemp_k[2] - pTemp[2]) * (pTemp_k[2] - pTemp[2]));
								if (dis_Temp < mindistance[k]) {
									mindistance[k] = dis_Temp;
								}
							}
						}
					}
				}
				//End Voroni
			}//block end		
		}
		return simiT;
	}
	
#pragma endregion

};
