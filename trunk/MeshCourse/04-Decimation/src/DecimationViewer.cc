//=============================================================================
//                                                
//   Code framework for the lecture
//
//   "Surface Representation and Geometric Modeling"
//
//   Mark Pauly, Mario Botsch, Balint Miklos, and Hao Li
//
//   Copyright (C) 2007 by  Applied Geometry Group and 
//							Computer Graphics Laboratory, ETH Zurich
//                                                                         
//-----------------------------------------------------------------------------
//                                                                            
//                                License                                     
//                                                                            
//   This program is free software; you can redistribute it and/or
//   modify it under the terms of the GNU General Public License
//   as published by the Free Software Foundation; either version 2
//   of the License, or (at your option) any later version.
//   
//   This program is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU General Public License for more details.
//   
//   You should have received a copy of the GNU General Public License
//   along with this program; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin Street, Fifth Floor, 
//   Boston, MA  02110-1301, USA.
//                                                                            
//=============================================================================
//=============================================================================
//
//  CLASS ReconViewer - IMPLEMENTATION
//
//=============================================================================

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>
#include <vector>
#include <float.h>
#include <windows.h>
#include "DecimationViewer.hh"


DecimationViewer::DecimationViewer(const char* _title, int _width, int _height) :
MeshViewer(_title, _width, _height),
percentage_(80),
useEdgeCollapse(false)
{
	mesh_.add_property(vquadric);
	mesh_.add_property(vprio);
	mesh_.add_property(vtarget);
};

void DecimationViewer::keyboard(int key, int x, int y) 
{
	switch (key)
	{
	case 'o': {
			OPENFILENAME ofn={0};
			char szFileName[MAX_PATH]={0};
			ofn.lStructSize=sizeof(OPENFILENAME);
			ofn.Flags=OFN_ALLOWMULTISELECT|OFN_EXPLORER;
			ofn.lpstrFilter="All Files (*.*)\0*.*\0";
			ofn.lpstrFile=szFileName;
			ofn.nMaxFile=MAX_PATH;
			if(GetOpenFileName(&ofn)) {
				mesh_.clear();
				if (open_mesh(szFileName)) {
					std::cout << "#vertices after open: " << mesh_.n_vertices() << std::endl;
				}
				else {
					std::cout << "Failed to read the file" << std::endl;
				}
			}
		}
		break;
	case 'z':  //up percentage by 5%
		percentage_=percentage_+5;
		if (percentage_>95) percentage_=95;
		std::cout<<"Percentage is %"<<percentage_<<"\n";
		break;
	case 'x':  //up percentage by 5%
		percentage_=percentage_-5;
		if (percentage_<5) percentage_=5;
		std::cout<<"Percentage is %"<<percentage_<<"\n";
		break;
	case 'd': //decimate
		// compute normals & quadrics
		std::cout << "#vertices before init: " << mesh_.n_vertices() << std::endl;
		init();
		// decimate
		std::cout << "#vertices before decimate: " << mesh_.n_vertices() << std::endl;
		decimate( ((double)percentage_/100.0)*mesh_.n_vertices() );
		std::cout << "#vertices after decimate: " << mesh_.n_vertices() << std::endl;
		break;
	case 'm':
		useEdgeCollapse = !useEdgeCollapse;
		std::cout << "Edge collapse mode is ";
		if (useEdgeCollapse) {
			std::cout << "ON" << std::endl;
		}
		else {
			std::cout << "OFF" << std::endl;
		}
		break;
	default:
		GlutExaminer::keyboard(key, x, y);
		break;
	}
}

void DecimationViewer::init()
{
	// compute face normals
	mesh_.update_face_normals();

	for (Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		Quadricd		Q;
		Mesh::Point		p;
		double			d;
		p = mesh_.point(vit.handle());
		priority(vit) = -1.0;
		quadric(vit).clear();
		
		// Exercise 4.1 --------------------------------------
		// INSERT CODE:
		// calc vertex quadrics from incident triangles
		// ---------------------------------------------------
		Q.clear();
		for(Mesh::VertexFaceIter fit = mesh_.vf_begin(vit); fit; ++fit) {
			Mesh::Normal n = mesh_.normal(fit.handle());
			d = -( p|n );
			Q += Quadricd(n[0], n[1], n[2], d);
		}
		quadric(vit.handle()) = Q;
	}
}

bool DecimationViewer::is_collapse_legal(Mesh::HalfedgeHandle _hh)
{
	// collect vertices
	Mesh::VertexHandle v0, v1;
	v0 = mesh_.from_vertex_handle(_hh);
	v1 = mesh_.to_vertex_handle(_hh);

	// collect faces
	Mesh::FaceHandle fl = mesh_.face_handle(_hh);
	Mesh::FaceHandle fr = mesh_.face_handle(mesh_.opposite_halfedge_handle(_hh));

	// backup point positions
	Mesh::Point p0 = mesh_.point(v0);
	Mesh::Point p1 = mesh_.point(v1);

	// topological test
	if (!mesh_.is_collapse_ok(_hh))
		return false;

	// test boundary stuff
	if (mesh_.is_boundary(v0) && !mesh_.is_boundary(v1))
		return false;

	// Exercise 4.2 -----------------------------------------------
	// INSERT CODE:
	// test normal flipping:
	//   if normal vector of a (non-degenerate) triangle changes by 
	//   more than pi/4 degrees, return false.
	// ------------------------------------------------------------




	Mesh::Point newFacePts[3];

	for(Mesh::VertexFaceIter fit = mesh_.vf_begin(v0); fit; ++fit) {
		if (fit.handle() == fl || fit.handle() == fr) {
			continue;
		}
		Mesh::Normal oldN = mesh_.normal(fit.handle());
		oldN = oldN.normalize();
		VertexHandle v;
		int i = 0;
		for (Mesh::FaceHalfedgeIter heit = mesh_.fh_begin(fit); heit; ++heit, i++) {
			v = mesh_.from_vertex_handle(heit.handle());
			if (v == v0) {
				newFacePts[i] = mesh_.point(v1);
			}
			else {
				newFacePts[i] = mesh_.point(v);
			}
		}
		Vec3f a = newFacePts[1] - newFacePts[0];
		Vec3f b = newFacePts[2] - newFacePts[0];
		Vec3f newN = (a % b).normalize();
		double dprod = (oldN | newN);
		double alpha = acos( std::max(-0.99, dprod ) );
		if (abs(alpha) >= (M_PI / 4)) {
			return false;
		}
	}

	// collapse passed all tests -> ok
	return true;
}

float DecimationViewer::priority(Mesh::HalfedgeHandle _heh)
{
	// Exercise 4.3 ----------------------------------------------
	// INSERT CODE:
	// return priority: the smaller the better
	// use quadrics to estimate approximation error
	// -----------------------------------------------------------
	Mesh::VertexHandle		from	=	mesh_.from_vertex_handle(_heh),
							to		=	mesh_.to_vertex_handle(_heh);
	return quadric(from)(mesh_.point(from)) + quadric(to)(mesh_.point(to));
}

void DecimationViewer::enqueue_vertex(Mesh::VertexHandle _vh)
{
	float						prio, min_prio(FLT_MAX);
	Mesh::HalfedgeHandle		min_hh;
	// find best out-going halfedge
	for (Mesh::VOHIter vh_it(mesh_, _vh); vh_it; ++vh_it) {
		if (is_collapse_legal(vh_it)) {
			prio = priority(vh_it);
			if (prio >= -1.0 && prio < min_prio) {
				min_prio = prio;
				min_hh   = vh_it.handle();
			}
		}
	}
	// update queue
	QueueVertex					qv;
	qv.v=_vh; 
	qv.prio=priority(_vh);
	if (priority(_vh) != -1.0) {
		queue.erase(qv);
		priority(_vh) = -1.0;
	}
	if (min_hh.is_valid()) {
		priority(_vh) = min_prio;
		target(_vh)   = min_hh;
		qv.prio = min_prio;
		queue.insert(qv);
	}
}

void DecimationViewer::decimate(unsigned int _n_vertices)
{
	unsigned int nv(mesh_.n_vertices());
	// build priority queue
	queue.clear();
	for (Mesh::VertexIter v_it  = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {
		enqueue_vertex(v_it.handle());
	}
	while (nv > _n_vertices && !queue.empty())
	{
		// Exercise 4.3 ----------------------------------------------
		// INSERT CODE:
		// Decimate using priority queue:
		//   1) take 1st element of queue
		//   2) collapse this halfedge
		//   3) update queue
		// -----------------------------------------------------------
		// take 1st element of queue
		queue_iterator it = queue.begin();
		QueueVertex qv = *(it);
		queue.erase(it);
		Mesh::VertexHandle v = qv.v;
		Mesh::HalfedgeHandle hh = target(v);
		Mesh::VertexHandle t = mesh_.to_vertex_handle(hh);

		if (useEdgeCollapse) {
			if ( !is_collapse_legal(hh) || !is_collapse_legal(mesh_.opposite_halfedge_handle(hh)) ) {
				continue;
			}
			Vec3f r = new_vertex_location(v, t);
			mesh_.set_point(t, r);
		}
		else {
			if ( !is_collapse_legal(hh) ) {
				continue;
			}
		}
		// collapse the v->t halfedge
		// This next line took me a day to figure out :(
		quadric(t) += quadric(v);
		mesh_.collapse(hh);
		mesh_.delete_vertex(v, true);
		// update queue
		for (Mesh::VVIter vvit = mesh_.vv_begin(t); vvit != mesh_.vv_end(t); ++vvit) {
			if (vvit.handle() != v) {
				enqueue_vertex(vvit.handle());
			}
		}
		enqueue_vertex(t);
		nv--;
	}
	// clean up
	queue.clear();
	// now, delete the items marked to be deleted
	mesh_.garbage_collection();
	// re-compute face & vertex normals
	mesh_.update_normals();
	// re-update face indices for faster rendering
	update_face_indices();
}

// From HW2
void DecimationViewer::solve_linear_system( gmmMatrix& _M, gmmVector& _b, gmmVector& _x)
{
	// solve linear system by gmm's LU factorization
	unsigned int N = _b.size();
	_x.resize(N);
	std::vector< size_t >  ipvt(N);
	gmm::lu_factor( _M, ipvt );
	gmm::lu_solve( _M, ipvt, _x, _b );
}

// Like in the book: minimize the new quadric error
Vec3f DecimationViewer::new_vertex_location(Mesh::VertexHandle v, Mesh::VertexHandle t)
{
	Quadricd Q;
	Q += quadric(v);
	Q += quadric(t);
	gmmMatrix M = Q.matrix();
	gmmVector b(4);
	b[0] = b[1] = b[2] = 0;
	b[3] = 1;
	gmmVector x(4);
	solve_linear_system(M, b, x);
	// just to be safe
	if (abs(x[3] - 0) < 0.001) {
		return mesh_.point(t);
	}
	return Vec3f(x[0]/x[3], x[1]/x[3], x[2]/x[3]);
}
