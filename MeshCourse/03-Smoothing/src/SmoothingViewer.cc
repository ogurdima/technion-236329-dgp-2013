//=============================================================================
//                                                                            
//   Example code for the full-day course
//
//   M. Botsch, M. Pauly, C. Roessl, S. Bischoff, L. Kobbelt,
//   "Geometric Modeling Based on Triangle Meshes"
//   held at SIGGRAPH 2006, Boston, and Eurographics 2006, Vienna.
//
//   Copyright (C) 2006 by  Computer Graphics Laboratory, ETH Zurich, 
//                      and Computer Graphics Group,      RWTH Aachen
//
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


#include "SmoothingViewer.hh"
#include "UniformLaplacian.h"
#include "LaplaceBeltrami.h"
#include <windows.h>


SmoothingViewer::SmoothingViewer(const char* _title, int _width, int _height): 
QualityViewer(_title, _width, _height)
{ 
	mesh_.add_property(vpos_);
	mesh_.add_property(vnorm_);
	
}

bool SmoothingViewer::open_mesh(const char* _filename)
{
	bool res = QualityViewer::open_mesh(_filename);
	if (!res) {
		return false;
	}
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		Mesh::Normal nrm = mesh_.normal(vit.handle());
		mesh_.property(vnorm_, vit.handle()) = nrm.normalize();
	}
	return true;
}

void SmoothingViewer::keyboard(int key, int x, int y)
{
	switch (toupper(key))
	{
	case 'O':
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
	case 'N': {
			std::cout << "10 Laplace-Beltrami smoothing iterations: " << std::flush;
			smooth(10);
			break;
		}
	case 'U': {
			std::cout << "10 uniform smoothing iterations: " << std::flush;
			uniform_smooth(10);
			break;
		}
	case 'T': {
			std::cout << "10 tangential smoothing iterations: " << std::flush;
			tangential_smooth(10);
			break;
		}
	default: {
			QualityViewer::keyboard(key, x, y);
			return;
		}
	}
	calc_weights();
	calc_mean_curvature();
	calc_uniform_mean_curvature();
	calc_gauss_curvature();
	calc_triangle_quality();
	face_color_coding();
	glutPostRedisplay();
	std::cout << "done\n";
}

void SmoothingViewer::smooth(unsigned int _iters)
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 3.3.b Smoothing using the Laplace-Beltrami.
	// Use eweight_ properties for the individual edge weights
	// and their sum for the normalization term.
	// ------------- IMPLEMENT HERE ---------
	for (int i = 0; i < _iters; i++) {
		LaplaceBeltrami l(mesh_, eweight_);
		generic_smooth_iter(&l);
	}
}

void SmoothingViewer::uniform_smooth(unsigned int _iters)
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 3.1.b Smoothing using the uniform Laplacian approximation
	// ------------- IMPLEMENT HERE ---------
	for (int i = 0; i < _iters; i++) {
		UniformLaplacian l(mesh_);
		generic_smooth_iter(&l);
	}
}

void SmoothingViewer::tangential_smooth(unsigned int _iters)
{
	for (int i = 0; i < _iters; i++) {
		UniformLaplacian l(mesh_);
		tangential_smooth_iter(&l);
	}
}

void SmoothingViewer::generic_smooth_iter(Laplacian* l)
{
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		Mesh::Point p = mesh_.point(vit.handle());
		Vec3f dir = l->operator()(vit);
		mesh_.property(vpos_, vit) = p + (dir * 0.5);
	}
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		mesh_.set_point( vit.handle(), mesh_.property(vpos_, vit) );
	}
	mesh_.update_normals();
}

void SmoothingViewer::tangential_smooth_iter(Laplacian* l)
{
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		Mesh::Point p = mesh_.point(vit.handle());
		Vec3f dir = l->operator()(vit);
		Vec3f norm = mesh_.property(vnorm_, vit.handle());
		double len = (norm | dir);
		Vec3f ortho = norm * len;
		Vec3f tangent = dir - ortho;
		mesh_.property(vpos_, vit) = p + (tangent * 0.5);
	}
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		mesh_.set_point( vit.handle(), mesh_.property(vpos_, vit) );
	}
	mesh_.update_normals();
}