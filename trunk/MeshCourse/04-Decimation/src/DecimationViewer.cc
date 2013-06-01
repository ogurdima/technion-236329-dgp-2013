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


//== INCLUDES =================================================================


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Tools/Utils/Timer.hh>

#include <vector>
#include <float.h>

#include <windows.h>
#include "DecimationViewer.hh"


//== IMPLEMENTATION ========================================================== 

DecimationViewer::
	DecimationViewer(const char* _title, int _width, int _height) : MeshViewer(_title, _width, _height) {};
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------


void DecimationViewer::keyboard(int key, int x, int y) 
{
	switch (key)
	{
	case 'o':
		{
			OPENFILENAME ofn={0};
			char szFileName[MAX_PATH]={0};
			ofn.lStructSize=sizeof(OPENFILENAME);
			ofn.Flags=OFN_ALLOWMULTISELECT|OFN_EXPLORER;
			ofn.lpstrFilter="All Files (*.*)\0*.*\0";
			ofn.lpstrFile=szFileName;
			ofn.nMaxFile=MAX_PATH;
			if(GetOpenFileName(&ofn))
			{
				mesh_.clear();
				MeshViewer::open_mesh(szFileName);
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
		init();

		// decimate
		decimate((percentage_/100.0)*mesh_.n_vertices());
		std::cout << "#vertices: " << mesh_.n_vertices() << std::endl;

	default:
		GlutExaminer::keyboard(key, x, y);
		break;
	}
}


//=============================================================================
//DECIMATION IMPLEMENTATION FUNCTIONS
//==============================================================================




void DecimationViewer::init()
{
	// compute face normals
	mesh_.update_face_normals();


	Mesh::VertexIter  v_it, v_end = mesh_.vertices_end();
	Mesh::Point       n;
	double              a, b, c, d;

	for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
	{
		priority(v_it) = -1.0;
		quadric(v_it).clear();


		// Exercise 4.1 --------------------------------------
		// INSERT CODE:
		// calc vertex quadrics from incident triangles
		// ---------------------------------------------------

	}
}


//-----------------------------------------------------------------------------


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


	// collapse passed all tests -> ok
	return true;
}


//-----------------------------------------------------------------------------


float DecimationViewer::priority(Mesh::HalfedgeHandle _heh)
{
	// Exercise 4.3 ----------------------------------------------
	// INSERT CODE:
	// return priority: the smaller the better
	// use quadrics to estimate approximation error
	// -----------------------------------------------------------
	return 0; //this is here just to make sure it compiles until function is implemented
}


//-----------------------------------------------------------------------------


void DecimationViewer::enqueue_vertex(Mesh::VertexHandle _vh)
{
	float                   prio, min_prio(FLT_MAX);
	Mesh::HalfedgeHandle  min_hh;


	// find best out-going halfedge
	for (Mesh::VOHIter vh_it(mesh_, _vh); vh_it; ++vh_it)
	{
		if (is_collapse_legal(vh_it))
		{
			prio = priority(vh_it);
			if (prio >= -1.0 && prio < min_prio)
			{
				min_prio = prio;
				min_hh   = vh_it.handle();
			}
		}
	}


	// update queue
	QueueVertex qv;
	qv.v=_vh; qv.prio=priority(_vh);
	if (priority(_vh) != -1.0) 
	{
		queue.erase(qv);
		priority(_vh) = -1.0;
	}

	if (min_hh.is_valid()) 
	{
		priority(_vh) = min_prio;
		target(_vh)   = min_hh;
		qv.prio=min_prio;
		queue.insert(qv);
	}
}


//-----------------------------------------------------------------------------


void DecimationViewer::decimate(unsigned int _n_vertices)
{
	unsigned int nv(mesh_.n_vertices());

	Mesh::HalfedgeHandle hh;
	Mesh::VertexHandle   to, from;
	Mesh::VVIter         vv_it;

	std::vector<Mesh::VertexHandle>            one_ring;
	std::vector<Mesh::VertexHandle>::iterator  or_it, or_end;



	// build priority queue
	Mesh::VertexIter  v_it  = mesh_.vertices_begin(), 
		v_end = mesh_.vertices_end();

	queue.clear();
	for (; v_it!=v_end; ++v_it)
		enqueue_vertex(v_it.handle());



	while (nv > _n_vertices && !queue.empty())
	{

		// Exercise 4.3 ----------------------------------------------
		// INSERT CODE:
		// Decimate using priority queue:
		//   1) take 1st element of queue
		//   2) collapse this halfedge
		//   3) update queue
		// -----------------------------------------------------------

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