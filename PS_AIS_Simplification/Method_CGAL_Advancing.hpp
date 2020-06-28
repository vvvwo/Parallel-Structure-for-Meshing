#pragma once
#include <iostream>
#include <fstream>
#include <algorithm>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/array.h>
#include <CGAL/disable_warnings.h>
typedef std::array<std::size_t, 3> Facet;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3  Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;

using namespace std;

struct Construct {
	Mesh& mesh;
	template < typename PointIterator>
	Construct(Mesh& mesh, PointIterator b, PointIterator e)
		: mesh(mesh)
	{
		for (; b != e; ++b) {
			boost::graph_traits<Mesh>::vertex_descriptor v;
			v = add_vertex(mesh);
			mesh.point(v) = *b;
		}
	}
	Construct& operator=(const Facet f)
	{
		typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<Mesh>::vertices_size_type size_type;
		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
			vertex_descriptor(static_cast<size_type>(f[1])),
			vertex_descriptor(static_cast<size_type>(f[2])));
		return *this;
	}
	Construct&
		operator*() { return *this; }
	Construct&
		operator++() { return *this; }
	Construct
		operator++(int) { return *this; }
};

class CGAL_Advancing_Reconstruct {

private:

	vector<vector<double>> pointsR;

public:

	void CGAL_Advancing_Remesh_init(vector<vector<double>> point_input) {

		pointsR = point_input;
	
	}

	void CGAL_Advancing_Remesh(string filename) {

		std::vector<Point_3> points;
		std::vector<Facet> facets;
		Mesh m;
		
		for (int i = 0; i < pointsR.size(); i++) {
			Point_3 pi(pointsR[i][0], pointsR[i][1], pointsR[i][2]);
			points.push_back(pi);		
		}

		Construct construct(m, points.begin(), points.end());
		CGAL::advancing_front_surface_reconstruction(points.begin(),
			points.end(),
			construct);
		
		const std::string& output_file("Remesh\\Advancing\\"+filename+".off");
		std::ofstream output_stream(output_file.c_str());
		if (output_stream && CGAL::write_off(output_stream, m)) {
			cout << "Advancing Remesh finish!" << endl;		
		}
		else {
			cout << "Advancing Remesh failed!" << endl;		
		}
	
		vector<vector<int>> indices;

		for (Mesh::Face_index face_index : m.faces()) {
			CGAL::Vertex_around_face_circulator<Mesh> vcirc(m.halfedge(face_index), m), done(vcirc);
			vector<int> ti;
			do {
				ti.push_back(*vcirc++);
			} while (vcirc != done);
			indices.push_back(ti);
		}

		//for (int i = 0; i < indices.size(); i++) {
			//for (int j = 0; j < indices[i].size(); j++) {
				//cout << indices[i][j] << " ";			
			//}
			//cout << endl;
		//}
	
	}
	 
	void CGAL_Advancing_Remesh_Out(string filename) {

		std::vector<Point_3> points;
		std::vector<Facet> facets;
		Mesh m;

		for (int i = 0; i < pointsR.size(); i++) {
			Point_3 pi(pointsR[i][0], pointsR[i][1], pointsR[i][2]);
			points.push_back(pi);
		}

		Construct construct(m, points.begin(), points.end());
		CGAL::advancing_front_surface_reconstruction(points.begin(),
			points.end(),
			construct);

		const std::string& output_file(filename);
		std::ofstream output_stream(output_file.c_str());
		if (output_stream && CGAL::write_off(output_stream, m)) {
			cout << "Advancing Remesh finish!" << endl;
		}
		else {
			cout << "Advancing Remesh failed!" << endl;
		}

		vector<vector<int>> indices;

		for (Mesh::Face_index face_index : m.faces()) {
			CGAL::Vertex_around_face_circulator<Mesh> vcirc(m.halfedge(face_index), m), done(vcirc);
			vector<int> ti;
			do {
				ti.push_back(*vcirc++);
			} while (vcirc != done);
			indices.push_back(ti);
		}	

	}

	vector<vector<int>> CGAL_Advancing_Remesh_FaceInfor() {

		std::vector<Point_3> points;
		std::vector<Facet> facets;
		Mesh m;

		for (int i = 0; i < pointsR.size(); i++) {
			Point_3 pi(pointsR[i][0], pointsR[i][1], pointsR[i][2]);
			points.push_back(pi);
		}

		Construct construct(m, points.begin(), points.end());
		CGAL::advancing_front_surface_reconstruction(points.begin(),
			points.end(),
			construct);		

		vector<vector<int>> indices;

		for (Mesh::Face_index face_index : m.faces()) {
			CGAL::Vertex_around_face_circulator<Mesh> vcirc(m.halfedge(face_index), m), done(vcirc);
			vector<int> ti;
			do {
				ti.push_back(*vcirc++);
			} while (vcirc != done);
			indices.push_back(ti);
		}
		return indices;	
	}

};


