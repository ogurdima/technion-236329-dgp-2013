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

#include <IsoEx/Grids/ScalarGridT.hh>
#include <IsoEx/Extractors/MarchingCubesT.hh>

#include "ReconViewer.hh"
#include "ImplicitRBF.hh"
#include "ImplicitMLS.hh"
#include <vector>
#include <float.h>

#include <windows.h>



//== IMPLEMENTATION ========================================================== 


ReconViewer::
	ReconViewer(const char* _title, int _width, int _height)
	: MeshViewer(_title, _width, _height)
{ 
	mesh_.request_vertex_colors();
	epsilon=0.01;
	betha = 1;
	rbf_to_use = TRIHARMONIC;
	add_draw_mode("Point Cloud");

}


//-----------------------------------------------------------------------------


ReconViewer::
	~ReconViewer()
{
}

//-----------------------------------------------------------------------------

bool
	ReconViewer::
	open_mesh(const char* _filename)
{
	// load mesh
	if (MeshViewer::open_mesh(_filename))
	{
		glutPostRedisplay();
		return true;
	}
	return false;
}


//-----------------------------------------------------------------------------




void 
	ReconViewer::
	draw(const std::string& _draw_mode)
{

	if (_draw_mode == "Point Cloud")
	{

		//drawing point cloud
		glDisable(GL_LIGHTING);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		for (std::vector<Point>::iterator pi=Points.begin();pi!=Points.end();pi++)
			glVertex3d((*pi)[0], (*pi)[1],(*pi)[2]);
		glEnd();
	} else {

		if (indices_.empty())
		{
			MeshViewer::draw(_draw_mode);
			return;
		} 
		else MeshViewer::draw(_draw_mode);
	}

	/*if (_draw_mode == "Vertex Valences")
	{

	glDisable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	GL::glVertexPointer(mesh_.points());
	GL::glNormalPointer(mesh_.vertex_normals());
	GL::glColorPointer(mesh_.vertex_colors());
	glDepthRange(0.01, 1.0);
	glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	glColor3f(0.1, 0.1, 0.1);
	glEnableClientState(GL_VERTEX_ARRAY);
	GL::glVertexPointer(mesh_.points());
	glDrawBuffer(GL_BACK);
	glDepthRange(0.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDepthFunc(GL_LEQUAL);
	glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	glDisableClientState(GL_VERTEX_ARRAY);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDepthFunc(GL_LESS);
	} */


}


//=============================================================================

void ReconViewer::MeshFromFunction(Implicit* ImpFunc)
{
	std::cout << "Bounding Box\n" << std::flush;
	Point bb_min( Points[0]), bb_max( Points[0]);

	for (std::vector<Point>::iterator pi=Points.begin(); pi!=Points.end();pi++)
	{
		bb_min.minimize(*pi);
		bb_max.maximize(*pi);
	}

	Point  bb_center = (bb_max+bb_min)*0.5f;
	OpenMesh::Vec3d VecDiag(bb_max[0]-bb_min[0], bb_max[1]-bb_min[1], bb_max[2]-bb_min[2]);
	bb_min = bb_center - 0.6f * Point(VecDiag[0], VecDiag[1], VecDiag[2]);
	bb_max = bb_center + 0.6f * Point(VecDiag[0], VecDiag[1], VecDiag[2]);

	// setup Marching Cubes grid by sampling RBF
	std::cout << "Setup grid\n" << std::flush;

	float MeanSize = VecDiag.mean();
	int res[3];
	int idir;
	for (idir=0; idir<3; idir++)
	{
		res[idir] = (int)(MC_RESOLUTION * VecDiag[0]/MeanSize + 0.5); 
	}
	IsoEx::ScalarGridT<Scalar>  grid(bb_min,
		Point(bb_max[0]-bb_min[0], 0, 0),
		Point(0, bb_max[1]-bb_min[1], 0),
		Point(0, 0, bb_max[2]-bb_min[2]),
		res[0], res[1], res[2]);

	for (unsigned int x=0; x<res[0]; ++x)
		for (unsigned int y=0; y<res[1]; ++y)
			for (unsigned int z=0; z<res[2]; ++z)
				grid.value(x,y,z) = (*ImpFunc)( grid.point(x,y,z) );


	// isosurface extraction by Marching Cubes
	std::cout << "Marching Cubes\n" << std::flush;
	marching_cubes(grid, mesh_); 

	mesh_.update_normals();

	// update face indices for faster rendering
	update_face_indices();

	// info
	std::cerr << mesh_.n_vertices() << " vertices, "
		<< mesh_.n_faces()    << " faces\n";
}

void 
	ReconViewer::keyboard(int key, int x, int y) 
{
	switch (key)
	{
	case 'o':
		{
			//.pts (point cloud) file opening
			//CFileDialog dlg(TRUE, "pts", "*.pts");
			//if (dlg.DoModal() == IDOK)
			OPENFILENAME ofn={0};
			char szFileName[MAX_PATH]={0};
			ofn.lStructSize=sizeof(OPENFILENAME);
			ofn.Flags=OFN_ALLOWMULTISELECT|OFN_EXPLORER;
			ofn.lpstrFilter="All Files (*.*)\0*.*\0";
			ofn.lpstrFile=szFileName;
			ofn.nMaxFile=MAX_PATH;
			if(GetOpenFileName(&ofn));
			{
				Points.clear();
				Normals.clear();
				//std::ifstream ifs(dlg.GetPathName());
				std::ifstream ifs(szFileName);
				if (!ifs) 
				{
					std::cerr << "Cannot open file\n";
					exit(1);
				}

				Point                p, n;

				while (ifs && !ifs.eof())
				{
					ifs >> p[0] >> p[1] >> p[2];
					ifs >> n[0] >> n[1] >> n[2];
					Points.push_back(p);
					Normals.push_back(n);
				}
				std::cout << Points.size() << " sample points\n";
				ifs.close();

				//establishing bounding box for drawing
				Mesh::Point            bbMin, bbMax;
				bbMin=*(Points.begin());
				bbMax=*(Points.begin());
				for (std::vector<Point>::iterator pi=Points.begin();pi!=Points.end();pi++){
					bbMin.minimize(*pi);
					bbMax.maximize(*pi);
				}
				set_scene( (Vec3f)(bbMin + bbMax)*0.5, 0.5*(bbMin - bbMax).norm());
			}
			break;
		}


		//RBF interpolation
	case 'r':{
		std::cout << "Fit RBF\n" << std::flush;
		RBF* _rbf;
		switch (rbf_to_use) {
		case TRIHARMONIC:
			_rbf = new TriharmonicRbf();
			break;
		case BSPLINE:
			_rbf = new CubicBSplineRbf(betha);
			break;
		}

		ImplicitRBF  implicitRBF_( Points, Normals, epsilon, *_rbf);
		MeshFromFunction((Implicit*)&implicitRBF_);
		delete _rbf;
		break;
			 }

	case 'm':{
		std::cout << "Fit MLS\n" << std::flush;
		ImplicitMLS  implicitMLS_( Points, Normals);
		MeshFromFunction((Implicit*)&implicitMLS_);
		break;
			 }

	case GLUT_KEY_UP:
		epsilon+=0.01;
		std::cout <<"Epsilon ="<<epsilon<<"\n"<<std::flush;
		break;

	case GLUT_KEY_DOWN:
		if (epsilon>0.015){
			epsilon-=0.01;
			std::cout <<"Epsilon ="<<epsilon<<"\n"<<std::flush;
		}
		break;
	case '[':
		if (betha > 0.2) {
			betha -= 0.1;
			std::cout <<"Betha ="<<betha<<"\n"<<std::flush;
		}
		break;
	case ']':
		betha += 0.1;
		std::cout <<"Betha ="<<betha<<"\n"<<std::flush;
		break;
	case 't':
		rbf_to_use = (ReconRBF)(((int)rbf_to_use + 1) % 2);
		if (rbf_to_use == TRIHARMONIC) {
			std::cout << "Triharmonic RBF"<< std::endl << std::flush;
		}
		if (rbf_to_use == BSPLINE) {
			std::cout << "Cubic B-Spline RBF"<< std::endl << std::flush;
		}
		break;

	default:
		{
			GlutExaminer::keyboard(key, x, y);
			break;
		}
	}
}
