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
//=============================================================================
//
//  CLASS SmoothingViewer - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include "SmoothingViewer.hh"



//== IMPLEMENTATION ========================================================== 


SmoothingViewer::
SmoothingViewer(const char* _title, int _width, int _height)
  : QualityViewer(_title, _width, _height)
{ 
  mesh_.add_property(vpos_);
}


//-----------------------------------------------------------------------------


void
SmoothingViewer::
keyboard(int key, int x, int y)
{
	switch (toupper(key))
	{
	case 'N':
		{
			std::cout << "10 Laplace-Beltrami smoothing iterations: " << std::flush;
			smooth(10);
			calc_weights();
			calc_mean_curvature();
			calc_uniform_mean_curvature();
			calc_gauss_curvature();
			calc_triangle_quality();
			face_color_coding();

			glutPostRedisplay();
			std::cout << "done\n";
			break;
		}
	case 'U':
		{
			std::cout << "10 uniform smoothing iterations: " << std::flush;
			uniform_smooth(10);
			calc_weights();
			calc_mean_curvature();
			calc_uniform_mean_curvature();
			calc_gauss_curvature();
			calc_triangle_quality();
			face_color_coding();

			glutPostRedisplay();
			std::cout << "done\n";
			break;
		}


	default:
		{
			QualityViewer::keyboard(key, x, y);
			break;
		}
	}
}


//-----------------------------------------------------------------------------


void 
SmoothingViewer::
smooth(unsigned int _iters)
{
	Mesh::VertexIter        v_it, v_end(mesh_.vertices_end());
	Mesh::HalfedgeHandle    h;
	Mesh::EdgeHandle        e;
	Mesh::VertexVertexIter  vv_it;
  Mesh::VertexOHalfedgeIter  vhe_it;
	
	Mesh::Scalar            w, ww;

	// ------------- IMPLEMENT HERE ---------
	// TASK 3.3.b Smoothing using the Laplace-Beltrami.
	// Use eweight_ properties for the individual edge weights
	// and their sum for the normalization term.
	// ------------- IMPLEMENT HERE ---------
 unsigned int iter;
 for (iter=0; iter<_iters; iter++)
 {
   std::vector<Mesh::Point> NewPoints;
   std::vector<Vec3f> LaplaceVec;
   for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
   {
     Mesh::Point VertexPoint = mesh_.point(v_it);
     ww = 0.;
     OpenMesh::Vec3f            laplace(0.0, 0.0, 0.0);

     for (vhe_it=mesh_.voh_iter(v_it); vhe_it; ++vhe_it)
     {
       h = mesh_.halfedge_handle(vhe_it);
       Mesh::VertexHandle CurrentVertex = mesh_.to_vertex_handle(vhe_it);
       Mesh::Point CurrentPoint = mesh_.point(CurrentVertex);
       w = mesh_.property(eweight_,mesh_.edge_handle(vhe_it));
       OpenMesh::Vec3f Vec = mesh_.property(eweight_,mesh_.edge_handle(vhe_it)) * (CurrentPoint - VertexPoint);

       laplace += Vec;
       ww += w;
     }

     laplace = laplace*1./ww;
     //laplace = laplace*mesh_.property(vweight_,v_it);
     Mesh::Point NewPosition = (VertexPoint+laplace*0.5);
     NewPoints.push_back(NewPosition); 
     LaplaceVec.push_back(laplace);
   }
   int ind=0;
   for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
   {
     if (!mesh_.is_boundary(v_it))
     {
      mesh_.set_point(v_it, NewPoints[ind]);
     }
     else
     {
       Vec3f Laplace(0.,0.,0.);
       int NbPoints = 0;
       for (vv_it=mesh_.vv_iter(v_it); vv_it; ++vv_it)
       {
         if (!mesh_.is_boundary(vv_it))
         {
           Laplace += LaplaceVec[ind];
           NbPoints++;
         }
       }
     }
     ind++;
   }
   mesh_.update_normals();
 }
}

//-----------------------------------------------------------------------------

static int TangentialSmoothing = 0;

void 
SmoothingViewer::
uniform_smooth(unsigned int _iters)
{
	Mesh::VertexIter        v_it, v_end(mesh_.vertices_end());
	Mesh::VertexVertexIter  vv_it;

	// ------------- IMPLEMENT HERE ---------
	// TASK 3.1.b Smoothing using the uniform Laplacian approximation
	// ------------- IMPLEMENT HERE ---------
  std::vector<OpenMesh::Vec3f> Normals;
  for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
  {
    Vec3f Normal = mesh_.normal(v_it);
    Normals.push_back(Normal);
  }

  int iter;
  for (iter=0; iter<_iters; iter++)
  {
    std::vector<Mesh::Point> NewPoints;
    int ind=0;
    for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
    {
      //if (!mesh_.is_boundary(v_it))
      {
        int n_vertices = 0;
        Mesh::Point VertexPoint = mesh_.point(v_it);
        Mesh::Point centroid(0.0, 0.0, 0.0);

        for (vv_it=mesh_.vv_iter(v_it); vv_it; ++vv_it)
        {
          centroid += mesh_.point(vv_it);
          n_vertices++;
          if (mesh_.is_boundary(v_it))
          {
            Vec3f Normal = mesh_.normal(v_it);
            Vec3f Vec1 = mesh_.point(vv_it)-VertexPoint;
            Vec3f NormalComponent = (Vec1|Normal)*Normal;
            Vec3f TangentComponent = Vec1-NormalComponent;
            Vec3f Vec2 = NormalComponent-TangentComponent;
            Mesh::Point OppositePoint = VertexPoint+Vec2;
            centroid += OppositePoint;
            n_vertices++;
          }
        }
        centroid = centroid*(1./n_vertices);
        OpenMesh::Vec3f Laplace = (centroid-VertexPoint)*0.5;
        Mesh::Point NewPosition = VertexPoint+Laplace;//(centroid+VertexPoint)*0.5;
        if (TangentialSmoothing)
        {
          OpenMesh::Vec3f Vec = Laplace-(Laplace|Normals[ind])*Normals[ind];
          NewPosition = VertexPoint+Vec;
        }
        NewPoints.push_back(NewPosition);   
      }
      ind++;
    }
    ind=0;
    for (v_it=mesh_.vertices_begin(); v_it != v_end; ++v_it)
    {
      //if (!mesh_.is_boundary(v_it))
      {
        mesh_.set_point(v_it, NewPoints[ind]);
        ind++;
      }
    }
    mesh_.update_normals();
  }
}

//=============================================================================
