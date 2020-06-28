#pragma once
// standard includes
#include <iostream>
#include <fstream>
#include <cassert>
// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP>                                  VD;
// typedef for the result type of the point location
typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;
typedef VD::Locate_result             Locate_result;
typedef VD::Vertex_handle             Vertex_handle;
typedef VD::Face_handle               Face_handle;
typedef VD::Halfedge_handle           Halfedge_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

class VoronoiD {

public:

	VD vd;
	vector<vector<double>> pOrignal;  //original points
	vector<vector<double>> halfEdage; //half Edage
	vector<vector<double>> rayEdage;  //ray Edage
	vector<vector<double>> halfEdage_r; //half Edage
	vector<vector<double>> rayEdage_r;  //ray Edage	
	vector<vector<double>> rayEdageBound; //ray Edage to bounding box
	vector<vector<double>> DelaunayTg; //Delaunay_triangulation
	
public:

	void VoronoiD_init(vector<vector<double>> pInput) {
		//cout << "VoronoiD init run:" << endl;
		pOrignal = pInput;
		//cout << "VoronoiD computation" << endl;
		VoronoiD_Computation();
		//cout << "VoronoiD init finished." << endl;
	}	

	vector<vector<double>> VoronoiD_Bounding(vector<vector<double>> pi, double r) {
		cout << "VoronoiD bounding run:" << endl;
		//store halfEdage and rayEdage
		vector<vector<double>> he1 = halfEdage;
		vector<vector<double>> re1 = rayEdage;
		if (rayEdageBound.size() > 0) {
			rayEdageBound.clear();		
		}	
		if (halfEdage_r.size() > 0) {
			halfEdage_r.clear();		
		}
		if (rayEdage_r.size() > 0) {
			rayEdage_r.clear();		
		}

		VoronoiD_UpdateEdge(r);//update halfedge

		//First: get the ray crossing point
		for (int i = 0; i < rayEdage.size(); i++) {
			Point_2 ps(rayEdage[i][0], rayEdage[i][1]);
			Point_2 pt(rayEdage[i][2], rayEdage[i][3]);
			Point_2 ptn = VoronoiD_CrossPoint(ps, pt, r);
			vector<double> rayEdge_i;
			rayEdge_i.push_back(ps[0]);
			rayEdge_i.push_back(ps[1]);
			rayEdge_i.push_back(ptn[0]);
			rayEdge_i.push_back(ptn[1]);
			rayEdageBound.push_back(rayEdge_i);
		}
		//Second: get the accurate edge;
		vector<vector<double>> finalResult;
		for (int i = 0; i < pi.size(); i++) {
			//cout << "processing:index" << i << "point." << endl;			
			vector<double> pi_i = pi[i];
			vector<vector<double>> pE = VoronoiD_Bounding_Single(pi_i, r);
			vector<double> finalResult_i;
			for (int j = 0; j < pE.size(); j++) {
				finalResult_i.push_back(pE[j][0]);
				finalResult_i.push_back(pE[j][1]);			
			}
			finalResult.push_back(finalResult_i);
		}	

		halfEdage.clear(); //half Edage
		rayEdage.clear();  //ray Edage
		halfEdage = he1;
		rayEdage = re1;
		cout << "VoronoiD bounding finished." << endl;
		return finalResult;		
	}
	
	vector<vector<double>> VoronoiD_Centroid(vector<vector<double>> bounding) {

		vector<vector<double>> result;

		for (int i = 0; i < bounding.size(); i++) {
			double x0 = pOrignal[i][0];
			double y0 = pOrignal[i][1];
			int size_i = bounding[i].size() / 2;
			vector<double> psite;
			vector<double> psiteWight;
			double Area = 0;
			//1. computing the trangulation position			
			for (int j = 0; j < size_i; j++) {
				int k = j + 1;
				if (k == size_i) {
					k = 0;				
				}
				double x1 = bounding[i][2 * j];
				double y1 = bounding[i][2 * j + 1];
				double x2 = bounding[i][2 * k];
				double y2 = bounding[i][2 * k + 1];
				double b1 = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
				double b2 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
				double b3 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
				double p = (b1 + b2 + b3) / 2;
				double tArea = sqrt(p * (p - b1) * (p - b2) * (p - b3));				
				Area = Area + tArea;
				psite.push_back((x0 + x1 + x2) / 3);
				psite.push_back((y0 + y1 + y2) / 3);
				psiteWight.push_back(tArea);
				
			}	
			double x_new = 0;
			double y_new = 0;

			for (int j = 0; j < psiteWight.size(); j++) {
				x_new = x_new + psite[2 * j] * psiteWight[j]/ Area;
				y_new = y_new + psite[2 * j + 1] * psiteWight[j]/ Area;
			}

			vector<double>pNew_i(2);
			pNew_i[0] = x_new;
			pNew_i[1] = y_new;
			result.push_back(pNew_i);
		}

		return result;
			
	}

	//The method is used for point cloud CVT 
	vector<double> VoronoiD_Centroid_Single(vector<double> pointC, double r) {

		vector<vector<double>> pointCB = VoronoiD_Bounding_Single(pointC, r);
		double x0 = pointC[0];
		double y0 = pointC[1];		
		vector<double> psite;
		vector<double> psiteWight;
		double Area = 0;
		//1. computing the trangulation position			
		for (int i = 0; i < pointCB.size(); i++) {			
			double x1 = pointCB[i][0];
			double y1 = pointCB[i][1];
			double x2 = pointCB[i][2];
			double y2 = pointCB[i][3];
			double b1 = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
			double b2 = sqrt((x0 - x2) * (x0 - x2) + (y0 - y2) * (y0 - y2));
			double b3 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
			double p = (b1 + b2 + b3) / 2;
			double tArea = sqrt(p * (p - b1) * (p - b2) * (p - b3));
			Area = Area + tArea;
			psite.push_back((x0 + x1 + x2) / 3);
			psite.push_back((y0 + y1 + y2) / 3);
			psiteWight.push_back(tArea);
		}
		double x_new = 0;
		double y_new = 0;
		for (int j = 0; j < psiteWight.size(); j++) {
			x_new = x_new + psite[2 * j] * psiteWight[j] / Area;
			y_new = y_new + psite[2 * j + 1] * psiteWight[j] / Area;
		}
		vector<double>pNew_i(2);
		pNew_i[0] = x_new;
		pNew_i[1] = y_new;
		return pNew_i;
	}
	
	vector<double> VoronoiD_Centroid_Single_meanPoints(vector<double> pointC, double r) {

		vector<vector<double>> pointCB = VoronoiD_Bounding_Single(pointC, r);
		double x_new = 0;
		double y_new = 0;
		//1. computing the trangulation position			
		for (int i = 0; i < pointCB.size(); i++) {
			double x1 = pointCB[i][0];
			double y1 = pointCB[i][1];
			double x2 = pointCB[i][2];
			double y2 = pointCB[i][3];	
			x_new = x_new + x1 + x2;
			y_new = y_new + y1 + y2;
		}
		x_new = x_new / pointCB.size() / 2;
		y_new = y_new / pointCB.size() / 2;
		vector<double>pNew_i(2);
		pNew_i[0] = x_new;
		pNew_i[1] = y_new;
		return pNew_i;
	}

	vector<int> VoronoiD_Return_Delaunay(vector<double> pointC, vector<vector<double>> pointCN) {
		//pointCN dont include pointC

		vector<vector<double>> border;
		
		for (int i = 0; i < DelaunayTg.size(); i++) {

			double x1 = DelaunayTg[i][0];
			double y1 = DelaunayTg[i][1];
			double x2 = DelaunayTg[i][2];
			double y2 = DelaunayTg[i][3];
			double x3 = DelaunayTg[i][4];
			double y3 = DelaunayTg[i][5];
			vector<double> border_i;
			if (x1 == pointC[0] && y1 == pointC[1]) {
				border_i.push_back(x2);
				border_i.push_back(y2);
				border_i.push_back(x3);
				border_i.push_back(y3);	
				border.push_back(border_i);
			}
			else if (x2 == pointC[0] && y2 == pointC[1]) {
				border_i.push_back(x1);
				border_i.push_back(y1);
				border_i.push_back(x3);
				border_i.push_back(y3);	
				border.push_back(border_i);
			}
			else if (x3 == pointC[0] && y3 == pointC[1]) {
				border_i.push_back(x1);
				border_i.push_back(y1);
				border_i.push_back(x2);
				border_i.push_back(y2);	
				border.push_back(border_i);
			}else{
				continue;			
			}		

		}

		vector<int> borderInt;

		for (int i = 0; i < border.size(); i++) {

			vector<double> border_p1;
			border_p1.push_back(border[i][0]);
			border_p1.push_back(border[i][1]);
			vector<double> border_p2;
			border_p2.push_back(border[i][2]);
			border_p2.push_back(border[i][3]);	

			int b1 = VoronoiD_search(border_p1, pointCN);
			int b2 = VoronoiD_search(border_p2, pointCN);
			
			if (b1 >= 0 && b2 >= 0) {				
				borderInt.push_back(b1);
				borderInt.push_back(b2);
			}		
		}
		return borderInt;	
	}
	
	vector<double> VoronoiD_Return_Delaunay_Mean(vector<double> pointC, vector<vector<double>> pointCN) {

		vector<vector<double>> border;

		for (int i = 0; i < DelaunayTg.size(); i++) {

			double x1 = DelaunayTg[i][0];
			double y1 = DelaunayTg[i][1];
			double x2 = DelaunayTg[i][2];
			double y2 = DelaunayTg[i][3];
			double x3 = DelaunayTg[i][4];
			double y3 = DelaunayTg[i][5];
			vector<double> border_i;
			if (x1 == pointC[0] && y1 == pointC[1]) {
				border_i.push_back(x2);
				border_i.push_back(y2);
				border_i.push_back(x3);
				border_i.push_back(y3);
				border.push_back(border_i);
			}
			else if (x2 == pointC[0] && y2 == pointC[1]) {
				border_i.push_back(x1);
				border_i.push_back(y1);
				border_i.push_back(x3);
				border_i.push_back(y3);
				border.push_back(border_i);
			}
			else if (x3 == pointC[0] && y3 == pointC[1]) {
				border_i.push_back(x1);
				border_i.push_back(y1);
				border_i.push_back(x2);
				border_i.push_back(y2);
				border.push_back(border_i);
			}
			else {
				continue;
			}

			vector<double> center(2);
			center[0] = 0;
			center[1] = 0;
			for (int i = 0; i < border.size(); i++) {

				center[0] = center[0] + border[i][0] + border[i][2];
				center[1] = center[1] + border[i][1] + border[i][3];
			
			}
			center[0] = center[0] / border.size() / 2;
			center[1] = center[1] / border.size() / 2;
			return center;
		}
	
	}

private:

	void VoronoiD_Computation() {

		//VD vd;
		for (int i = 0; i < pOrignal.size(); i++) {
			Site_2 t(pOrignal[i][0], pOrignal[i][1]);
			vd.insert(t);
		}

		DT dt = vd.dual();

		for (DT::Finite_faces_iterator fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); fit++)
		{
			vector<double> tri;
			tri.push_back(fit->vertex(0)->point().hx());
			tri.push_back(fit->vertex(0)->point().hy());
			tri.push_back(fit->vertex(1)->point().hx());
			tri.push_back(fit->vertex(1)->point().hy());
			tri.push_back(fit->vertex(2)->point().hx());
			tri.push_back(fit->vertex(2)->point().hy());
			DelaunayTg.push_back(tri);
		}

		for (DT::Edge_iterator eit = dt.edges_begin(); eit != dt.edges_end(); eit++) {
			CGAL::Object o = dt.dual(eit);
			vector<double> epi;
			if (CGAL::object_cast<K::Segment_2>(&o)) //如果这条边是线段，则绘制线段
			{
				//cout << "it is a segment." << endl;
				//cout << "The source vertex is:" << CGAL::object_cast<K::Segment_2>(&o)->source().hx() << " " << CGAL::object_cast<K::Segment_2>(&o)->source().hy() << endl;
				//cout << "The target vertex is:" << CGAL::object_cast<K::Segment_2>(&o)->target().hx() << " " << CGAL::object_cast<K::Segment_2>(&o)->target().hy() << endl;
				epi.push_back(CGAL::object_cast<K::Segment_2>(&o)->source().hx());
				epi.push_back(CGAL::object_cast<K::Segment_2>(&o)->source().hy());
				epi.push_back(CGAL::object_cast<K::Segment_2>(&o)->target().hx());
				epi.push_back(CGAL::object_cast<K::Segment_2>(&o)->target().hy());
				halfEdage.push_back(epi);
			}
			else if (CGAL::object_cast<K::Ray_2>(&o))//如果这条边是射线，则绘制射线
			{
				//cout << "it is a ray." << endl;
				//cout << "The source vertex is:" << CGAL::object_cast<K::Ray_2>(&o)->source().hx() << " " << CGAL::object_cast<K::Ray_2>(&o)->source().hy() << endl;
				//cout << "The target vertex is:" << CGAL::object_cast<K::Ray_2>(&o)->point(1).hx() << " " << CGAL::object_cast<K::Ray_2>(&o)->point(1).hy() << endl;
				epi.push_back(CGAL::object_cast<K::Ray_2>(&o)->source().hx());
				epi.push_back(CGAL::object_cast<K::Ray_2>(&o)->source().hy());
				epi.push_back(CGAL::object_cast<K::Ray_2>(&o)->point(1).hx());
				epi.push_back(CGAL::object_cast<K::Ray_2>(&o)->point(1).hy());
				rayEdage.push_back(epi);
			}
		}
	}
	
	void VoronoiD_UpdateEdge(double r) {		

		for (int i = 0; i < halfEdage.size(); i++) {
			double x1 = halfEdage[i][0];
			double y1 = halfEdage[i][1];
			double x2 = halfEdage[i][2];
			double y2 = halfEdage[i][3];
			vector<double> halfEdage_i;			
			if (abs(x1) < r && abs(y1) < r && abs(x2) < r && abs(y2) < r) {
				halfEdage_i.insert(halfEdage_i.end(), halfEdage[i].begin(), halfEdage[i].end());
				halfEdage_r.push_back(halfEdage_i);
			}
			else if (abs(x1) < r && abs(y1) < r) {
				halfEdage_i.push_back(x1);
				halfEdage_i.push_back(y1);
				halfEdage_i.push_back(x2);
				halfEdage_i.push_back(y2);
				rayEdage_r.push_back(halfEdage_i);			
			}
			else if (abs(x2) < r && abs(y2) < r) {
				halfEdage_i.push_back(x2);
				halfEdage_i.push_back(y2);
				halfEdage_i.push_back(x1);
				halfEdage_i.push_back(y1);				
				rayEdage_r.push_back(halfEdage_i);			
			}
			else {
				continue;			
			}	
		}

		for (int i = 0; i < rayEdage.size(); i++) {
			if (abs(rayEdage[i][0]) < r && abs(rayEdage[i][1]) < r) {
				rayEdage_r.push_back(rayEdage[i]);
			}
			else {
				continue;
			}
		}
		
		halfEdage.clear(); //half Edage
		rayEdage.clear();  //ray Edage
		halfEdage = halfEdage_r;
		rayEdage = rayEdage_r;
	}

	vector<vector<double>> VoronoiD_Bounding_Single(vector<double> pi, double r) {

		vector<vector<double>> result;

		Point_2 p(pi[0], pi[1]);
		Locate_result lr = vd.locate(p);
		Face_handle* f = boost::get<Face_handle>(&lr);
		//std::cout << "face." << std::endl;
		//std::cout << "The vertices of the Voronoi face are"
			//<< " (in counterclockwise order):" << std::endl;
		Ccb_halfedge_circulator ec_start = (*f)->ccb();
		Ccb_halfedge_circulator ec = ec_start;
		
		//cout << "原始:" << endl;
		//do {
			//if (ec->has_source()) {
				//std::cout << ec->source()->point() << std::endl;				
			//}
			//else {
				//std::cout << "point at infinity" << std::endl;				
			//}
			//if (ec->has_target()) {
				//std::cout << ec->target()->point() << std::endl;				
			//}
			//else {
				//std::cout << "point at infinity" << std::endl;				
			//}
		
		//} while (++ec != ec_start);

		//cout <<"修正:"<< endl;
		do {
			vector<double> halfedgei;	
			if (ec->is_ray()) {
				//if the source point is in the region
				if (ec->has_source() && abs(ec->source()->point()[0]) < r && abs(ec->source()->point()[1]) < r) {
					//std::cout << ec->source()->point() << std::endl;
					halfedgei.push_back(ec->source()->point()[0]);
					halfedgei.push_back(ec->source()->point()[1]);
					//std::cout << "point at infinity" << std::endl;
					halfedgei.push_back(-9999);
					halfedgei.push_back(-9999);				
				}
				else if (ec->has_target() && abs(ec->target()->point()[0]) < r && abs(ec->target()->point()[1]) < r) {
					//std::cout << "point at infinity" << std::endl;
					halfedgei.push_back(-9999);
					halfedgei.push_back(-9999);
					//std::cout << ec->target()->point() << std::endl;
					halfedgei.push_back(ec->target()->point()[0]);
					halfedgei.push_back(ec->target()->point()[1]);									
				}
				else {
					//not in the region
					continue;				
				}
				result.push_back(halfedgei);
			}
			else {
				double xs = ec->source()->point()[0];
				double ys = ec->source()->point()[1];
				double xt = ec->target()->point()[0];
				double yt = ec->target()->point()[1];
				if (abs(xs) < r && abs(ys) < r && abs(xt) < r && abs(yt) < r) {
					halfedgei.push_back(ec->source()->point()[0]);
					halfedgei.push_back(ec->source()->point()[1]);
					halfedgei.push_back(ec->target()->point()[0]);
					halfedgei.push_back(ec->target()->point()[1]);	
					//std::cout << ec->source()->point() << std::endl;
					//std::cout << ec->target()->point() << std::endl;
				}
				else if(abs(xs) < r && abs(ys) < r){
					halfedgei.push_back(ec->source()->point()[0]);
					halfedgei.push_back(ec->source()->point()[1]);
					halfedgei.push_back(-9999);
					halfedgei.push_back(-9999);	
					//std::cout << ec->source()->point() << std::endl;					
					//std::cout << "point at infinity" << std::endl;
				}
				else if(abs(xt) < r && abs(yt) < r){					
					halfedgei.push_back(-9999);
					halfedgei.push_back(-9999);	
					halfedgei.push_back(ec->target()->point()[0]);
					halfedgei.push_back(ec->target()->point()[1]);
					//std::cout << "point at infinity" << std::endl;
					//std::cout << ec->target()->point() << std::endl;
				}
				else {
					continue;				
				}
				result.push_back(halfedgei);
			}
			/*
			if (ec->has_source()) {
				std::cout << ec->source()->point() << std::endl;				
				halfedgei.push_back(ec->source()->point()[0]);
				halfedgei.push_back(ec->source()->point()[1]);
			}
			else {
				std::cout << "point at infinity" << std::endl;
				halfedgei.push_back(-9999);
				halfedgei.push_back(-9999);
			}
			if (ec->has_target()) {
				std::cout << ec->target()->point() << std::endl;
				halfedgei.push_back(ec->target()->point()[0]);
				halfedgei.push_back(ec->target()->point()[1]);
			}
			else {
				std::cout << "point at infinity" << std::endl;
				halfedgei.push_back(-9999);
				halfedgei.push_back(-9999);
			}
			result.push_back(halfedgei);*/
		} while (++ec != ec_start);
		
		for (int i = 0; i < result.size(); i++) {
			vector<double> result_i = result[i];
			if (i == 0 && result_i[0] == -9999) {
				vector<double> raystart_i;
				raystart_i.push_back(result[result.size()-1][0]);
				raystart_i.push_back(result[result.size()-1][1]);
				vector<double> raycontext_i;
				raycontext_i.push_back(result[0][2]);
				raycontext_i.push_back(result[0][3]);
				vector<vector<double>> rayedge_i = VoronoiD_Edge(pi, raystart_i, raycontext_i, r);
				result[0][1] = rayedge_i[rayedge_i.size() - 1][1];
				result[0][0] = rayedge_i[rayedge_i.size() - 1][0];
				result[result.size() - 1].clear();
				for (int j = 0; j < rayedge_i.size() - 1; j++) {
					result[result.size() - 1].push_back(rayedge_i[j][0]);
					result[result.size() - 1].push_back(rayedge_i[j][1]);
					result[result.size() - 1].push_back(rayedge_i[j][2]);
					result[result.size() - 1].push_back(rayedge_i[j][3]);
				}
			}
			else if (result_i[2] == -9999){
				vector<double> raystart_i;
				raystart_i.push_back(result_i[0]);
				raystart_i.push_back(result_i[1]);
				vector<double> raycontext_i;
				raycontext_i.push_back(result[i+1][2]);
				raycontext_i.push_back(result[i+1][3]);
				vector<vector<double>> rayedge_i = VoronoiD_Edge(pi, raystart_i, raycontext_i, r);
				result[i+1][0] = rayedge_i[rayedge_i.size() - 1][0];
				result[i+1][1] = rayedge_i[rayedge_i.size() - 1][1];
				result[i].clear();
				for (int j = 0; j < rayedge_i.size() - 1; j++) {
					result[i].push_back(rayedge_i[j][0]);
					result[i].push_back(rayedge_i[j][1]);
					result[i].push_back(rayedge_i[j][2]);
					result[i].push_back(rayedge_i[j][3]);
				}			
			}			
			//cout <<"start:"<<result[i][0]<<","<< result[i][1] << endl;
			//cout <<"end:"<< result[i][2] << "," << result[i][3] << endl;			
		}

		vector<vector<double>> finalResult;
		for (int i = 0; i < result.size(); i++) {			
			for (int j = 0; j < result[i].size() / 4; j++) {
				vector<double> edge_ij;
				edge_ij.push_back(result[i][4 * j]);
				edge_ij.push_back(result[i][4 * j + 1]);
				edge_ij.push_back(result[i][4 * j + 2]);
				edge_ij.push_back(result[i][4 * j + 3]);
				finalResult.push_back(edge_ij);
			}	
		}		
		//after fix
		//cout << "after fix:" << endl;
		//for (int i = 0; i < finalResult.size(); i++) {
			//cout << "s:" << finalResult[i][0]<<","<< finalResult[i][1] << endl;
			//cout << "t:" << finalResult[i][2] << "," << finalResult[i][3] << endl;
		//}
		return finalResult;
	}

	vector<vector<double>> VoronoiD_Edge(vector<double> pointO, vector<double> rayStart,
		vector<double> rayContext, double r) {//achieve the next point of the edge
		//pointO: original point; rayStart: start point of ray; rayBefore: point have been selected
 		//computation the rayLine
		
		vector<double> rayStartStore;
		double min = 99999;
		for (int i = 0; i < rayEdageBound.size(); i++) {
			vector<double> rayStart_i = rayEdageBound[i];
			if (rayStart_i[0] == rayStart[0] && rayStart_i[1] == rayStart[1]) {
				vector<double> raySource_i;
				raySource_i.push_back(rayStart_i[2]);
				raySource_i.push_back(rayStart_i[3]);
				double dis = sqrt((pointO[0] - raySource_i[0]) * (pointO[0] - raySource_i[0]) +
					(pointO[1] - raySource_i[1]) * (pointO[1] - raySource_i[1]));
				if (dis < min) {
					rayStartStore = raySource_i;
					min = dis;
				}				
			}
		}

		vector<double> ray_repeat;
		if (rayStart[0] == rayContext[0] && rayStart[1] == rayContext[1]) {
			ray_repeat = rayStartStore;
		}
		else {
			ray_repeat.push_back(-9999);
			ray_repeat.push_back(-9999);		
		}

		min = 99999;
		vector<double> rayContextEndStore;
		for (int i = 0; i < rayEdageBound.size(); i++) {
			vector<double> rayStart_i = rayEdageBound[i];
			if (rayStart_i[0] == rayContext[0] && rayStart_i[1] == rayContext[1]) {
				vector<double> raySource_i;
				raySource_i.push_back(rayStart_i[2]);
				raySource_i.push_back(rayStart_i[3]);

				if (ray_repeat[0] == raySource_i[0] && ray_repeat[1] == raySource_i[1]) {
					continue;				
				}

				double dis = sqrt((pointO[0] - raySource_i[0]) * (pointO[0] - raySource_i[0]) +
					(pointO[1] - raySource_i[1]) * (pointO[1] - raySource_i[1]));
				if (dis < min) {
					rayContextEndStore = raySource_i;
					min = dis;
				}
			}
		}

		if (rayContextEndStore.size() == 0 || (rayContextEndStore[0] == rayStartStore[0] &&
			rayContextEndStore[1] == rayStartStore[1])) {

			cout << "Error! the ray endpoint searching wrong!"<<endl;

		}

		vector<vector<double>> finalResult;
		vector<double> p1;
		vector<double> p12;
		vector<double> p2;
		vector<double> p23;
		vector<double> p3;
		p1.insert(p1.end(), rayStart.begin(), rayStart.end());
		p1.insert(p1.end(), rayStartStore.begin(), rayStartStore.end());
		p2.insert(p2.end(), rayStartStore.begin(), rayStartStore.end());
		p2.insert(p2.end(), rayContextEndStore.begin(), rayContextEndStore.end());
		p3.insert(p3.end(), rayContextEndStore.begin(), rayContextEndStore.end());
		p3.insert(p3.end(), rayContext.begin(), rayContext.end());

		if (rayStartStore[0] == rayContextEndStore[0] || rayStartStore[1] == rayContextEndStore[1]) {			
			
			finalResult.push_back(p1);
			finalResult.push_back(p2);
			finalResult.push_back(p3);		
		}
		else {
			double xCrossRay = 0;
			double yCrossRay = 0;
			if (abs(rayStartStore[0]) == r) {
				xCrossRay = rayStartStore[0];			
			}
			if (abs(rayContextEndStore[0]) == r) {
				xCrossRay = rayContextEndStore[0];
			}
			if (abs(rayStartStore[1]) == r) {
				yCrossRay = rayStartStore[1];
			}
			if (abs(rayContextEndStore[1]) == r) {
				yCrossRay = rayContextEndStore[1];
			}

			if (xCrossRay == 0 || yCrossRay == 0) {
				cout << "Error! crossing point missing!" << endl;			
			}
			p12.insert(p12.end(), rayStartStore.begin(), rayStartStore.end());
			p12.push_back(xCrossRay);
			p12.push_back(yCrossRay);
			p23.push_back(xCrossRay);
			p23.push_back(yCrossRay);
			p23.insert(p23.end(), rayContextEndStore.begin(), rayContextEndStore.end());
			finalResult.push_back(p1);
			finalResult.push_back(p12);
			finalResult.push_back(p23);
			finalResult.push_back(p3);		
		}

		return finalResult;		
	}

	Point_2 VoronoiD_CrossPoint(Point_2 ps, Point_2 pt, int r) {//ps is the source of the ray, pt is the target of the ray

		int lineX = r;
		//corssing 4 Axis
		double v1x = pt[0] - ps[0];
		double v1y = pt[1] - ps[1];
		//1 200, y
		double landa1 = (lineX - ps[0]) / v1x;
		double x1 = lineX;
		double y1 = landa1 * v1y + ps[1];
		if (landa1 > 0 && y1 > -lineX && y1 < lineX) {
			Point_2 result(x1, y1);
			return result;
		}
		//2 -200, y
		double landa2 = (-lineX - ps[0]) / v1x;
		double x2 = -lineX;
		double y2 = landa2 * v1y + ps[1];
		if (landa2 > 0 && y2 > -lineX && y2 < lineX) {
			Point_2 result(x2, y2);
			return result;
		}
		//3 x, 200
		double landa3 = (lineX - ps[1]) / v1y;
		double x3 = landa3 * v1x + ps[0];
		double y3 = lineX;
		if (landa3 > 0 && x3 > -lineX && x3 < lineX) {
			Point_2 result(x3, y3);
			return result;
		}
		//4 x, -200
		double landa4 = (-lineX - ps[1]) / v1y;
		double x4 = landa4 * v1x + ps[0];
		double y4 = -lineX;
		if (landa4 > 0 && x4 > -lineX && x4 < lineX) {
			Point_2 result(x4, y4);
			return result;
		}

		cout << "Error!no cross point!";
		Point_2 result(0, 0);
		return result;

	}
	
	int VoronoiD_search(vector<double> p, vector<vector<double>> pl) {

		for (int i = 0; i < pl.size(); i++) {
			vector<double> pli = pl[i];
			if (p[0] == pl[i][0] && p[1] == pl[i][1]) {				
				return i;			
			}		
		}	
		return -1;	

	}

};



/*
if (result[0][0] != result[result.size() - 1][2] &&
			result[0][1] != result[result.size() - 1][3]) {

			//same line
			if (result[0][0] == result[result.size() - 1][2] ||
				result[0][1] == result[result.size() - 1][3]) {
				vector<double> result_i;
				result_i.push_back(result[result.size() - 1][2]);
				result_i.push_back(result[result.size() - 1][3]);
				result_i.push_back(result[0][0]);
				result_i.push_back(result[0][1]);
				result.push_back(result_i);
			}
			//not same line
			else {
				double xcrossAxis;
				double ycrossAxis;
				if (abs(result[0][0]) == (int)r) {
					xcrossAxis = result[0][0];
				}
				else {
					ycrossAxis = result[0][1];
				}

				if (abs(result[result.size() - 1][2]) == (int)r) {
					xcrossAxis = result[result.size() - 1][2];
				}
				else {
					ycrossAxis = result[result.size() - 1][3];
				}
				vector<double> result_i1;
				vector<double> result_i2;
				result_i1.push_back(result[result.size() - 1][2]);
				result_i1.push_back(result[result.size() - 1][3]);
				result_i1.push_back(xcrossAxis);
				result_i1.push_back(ycrossAxis);
				result_i2.push_back(xcrossAxis);
				result_i2.push_back(ycrossAxis);
				result_i2.push_back(result[0][0]);
				result_i2.push_back(result[0][1]);
				result.push_back(result_i1);
				result.push_back(result_i2);
			}
		}
		cout << endl;

vector<double> VoronoiD_Edge(vector<double> pointO, vector<double> rayStart,
		vector<double> rayBefore, double r) {//achieve the next point of the edge
		//pointO: original point; rayStart: start point of ray; rayBefore: point have been selected
		//computation the rayLine

		vector<vector<double>> raySourceStore;
		for (int i = 0; i < rayEdageBound.size(); i++) {
			vector<double> rayStart_i = rayEdageBound[i];
			if (rayStart_i[0] == rayStart[0] && rayStart_i[1] == rayStart[1]) {
				vector<double> raySource_i;
				raySource_i.push_back(rayStart_i[2]);
				raySource_i.push_back(rayStart_i[3]);
				raySourceStore.push_back(raySource_i);
			}
		}

		if (raySourceStore.size() == 1) {

			return raySourceStore[0];

		}
		else {
			vector<vector<double>> raySourceStore2;
			for (int i = 0; i < raySourceStore.size(); i++) {
				vector<double> raySourceStore_i = raySourceStore[i];
				bool judge = true;
				for (int j = 0; j < rayBefore.size()/2; j++) {
					if (raySourceStore_i[0] == rayBefore[2 * j] &&
						raySourceStore_i[1] == rayBefore[2 * j + 1]) {
						judge = false;
						break;
					}
				}
				if (judge) {
					raySourceStore2.push_back(raySourceStore_i);
				}
			}
			if (raySourceStore2.size() == 1) {
				return raySourceStore2[0];
			}
			else {
				double min = 9999;
				vector<double> result(2);
				for (int i = 0; i < raySourceStore2.size(); i++) {
					double distance_i = sqrt((pointO[0] - raySourceStore2[i][0]) * (pointO[0] - raySourceStore2[i][0]) +
						(pointO[1] - raySourceStore2[i][1]) * (pointO[1] - raySourceStore2[i][1]));
					if (distance_i < min) {
						min = distance_i;
						result[0] = raySourceStore2[i][0];
						result[1] = raySourceStore2[i][1];
					}
				}
				return result;
			}

		}
		vector<double> resultFinal;
		resultFinal.push_back(-9999);
		resultFinal.push_back(-9999);
		return resultFinal;
	}
//function statement
void print_endpoint(Halfedge_handle e, bool is_src);
void VoronoiD();
void Voronoi2D(vector<vector<double>> points);

void print_endpoint(Halfedge_handle e, bool is_src) {
	std::cout << "\t";
	if (is_src) {
		if (e->has_source())  std::cout << e->source()->point() << std::endl;
		else  std::cout << "point at infinity" << std::endl;
	}
	else {
		if (e->has_target())  std::cout << e->target()->point() << std::endl;
		else  std::cout << "point at infinity" << std::endl;
	}
}

void VoronoiD()
{
	
	vector<double> seedpoints;
	seedpoints.push_back(-10);
	seedpoints.push_back(10);
	seedpoints.push_back(-150);
	seedpoints.push_back(160);
	seedpoints.push_back(-170);
	seedpoints.push_back(-80);
	seedpoints.push_back(-20);
	seedpoints.push_back(-150);
	seedpoints.push_back(180);
	seedpoints.push_back(-30);
	seedpoints.push_back(150);
	seedpoints.push_back(70);
	seedpoints.push_back(-190);
	seedpoints.push_back(10);

	vector<vector<double>> pList;
	for (int i = 0; i < seedpoints.size() / 2; i++) {
		double xi = seedpoints[2 * i];
		double yi = seedpoints[2 * i+1];
		vector<double> pi;
		pi.push_back(xi);
		pi.push_back(yi);
		pList.push_back(pi);	
	}
	
	Voronoi2D(pList);
	
}

void Voronoi2D(vector<vector<double>> points) {

	VD vd;
	for (int i = 0; i < points.size(); i++) {
		Site_2 t(points[i][0], points[i][1]);		
		vd.insert(t);	
	}

	
	DT dt = vd.dual();
	//for (int i = 0; i < points.size(); i++) {
		//Point_2 t(points[i][0], points[i][1]);
		//dt.insert(t);
	//}

	for (DT::Edge_iterator eit = dt.edges_begin(); eit != dt.edges_end(); eit++) {
		CGAL::Object o = dt.dual(eit);
		if (CGAL::object_cast<K::Segment_2>(&o)) //如果这条边是线段，则绘制线段
		{
			cout << "it is a segment." << endl;
			cout << "The source vertex is:" << CGAL::object_cast<K::Segment_2>(&o)->source().hx() << " " << CGAL::object_cast<K::Segment_2>(&o)->source().hy() << endl;
			cout << "The target vertex is:" << CGAL::object_cast<K::Segment_2>(&o)->target().hx() << " " << CGAL::object_cast<K::Segment_2>(&o)->target().hy() << endl;
		}
		else if (CGAL::object_cast<K::Ray_2>(&o))//如果这条边是射线，则绘制射线
		{
			cout << "it is a ray." << endl;
			cout << "The source vertex is:" << CGAL::object_cast<K::Ray_2>(&o)->source().hx() << " " << CGAL::object_cast<K::Ray_2>(&o)->source().hy() << endl;
			cout << "The target vertex is:" << CGAL::object_cast<K::Ray_2>(&o)->point(1).hx() << " " << CGAL::object_cast<K::Ray_2>(&o)->point(1).hy() << endl;
		}	
	}
	


	for (VD::Edge_iterator e = vd.edges_begin(); e != vd.edges_end(); e++) {				

		if (e->has_source()) {
			VD::Vertex_handle s_i = e->source();			
			cout << "The source vertex is:" << s_i->point().hx() <<" "<< s_i->point().hy()<< endl;			
		}
		else {
			std::cout << "The source vertex is infinity" << std::endl;				
		}
		if (e->has_target()) {
			VD::Vertex_handle t_i = e->target();
			cout << "The target vertex is:" << t_i->point() << endl;
		}
		else {
			std::cout << "The target vertex is infinity" << std::endl;
		}	
	}	

	assert(vd.is_valid());
	for(int i = 0;i< points.size(); i++){

		Point_2 p(points[i][0], points[i][1]);		
		Locate_result lr = vd.locate(p);
		if (Vertex_handle * v = boost::get<Vertex_handle>(&lr)) {
			std::cout << "vertex." << std::endl;
			std::cout << "The Voronoi vertex is:" << std::endl;
			std::cout << "\t" << (*v)->point() << std::endl;
		}
		else if (Halfedge_handle * e = boost::get<Halfedge_handle>(&lr)) {
			std::cout << "edge." << std::endl;
			std::cout << "The source and target vertices "
				<< "of the Voronoi edge are:" << std::endl;
			print_endpoint(*e, true);
			print_endpoint(*e, false);
		}
		else if (Face_handle * f = boost::get<Face_handle>(&lr)) {
			std::cout << "face." << std::endl;
			std::cout << "The vertices of the Voronoi face are"
				<< " (in counterclockwise order):" << std::endl;
			Ccb_halfedge_circulator ec_start = (*f)->ccb();
			Ccb_halfedge_circulator ec = ec_start;
			do {
				print_endpoint(ec, false);
			} while (++ec != ec_start);
		}
		std::cout << std::endl;
	
	}
}
for (Face_iterator f = VD.faces_begin(); f != VD.faces_end(); f++)
{
	Ccb_halfedge_circulator ec_start = (f)->ccb();
	Ccb_halfedge_circulator ec = ec_start;
	do {
		if (!ec->has_source())
		{
		}
		else
			QpolyF << QPointF(((Halfedge_handle)ec)->source()->point().x(), ((Halfedge_handle)ec)->source()->point().y());
	} while (++ec != ec_start);
	VectPolygon.push_back(QpolyF);
	QpolyF.clear();
}
*/

