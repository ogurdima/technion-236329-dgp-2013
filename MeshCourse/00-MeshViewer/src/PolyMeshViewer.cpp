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
//  CLASS PolyMeshViewer - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================


#include <OpenMesh/Core/IO/MeshIO.hh>
#include "PolyMeshViewer.hh"
#include "gl.hh"
#include <iostream>
#include <fstream>


//== IMPLEMENTATION ========================================================== 


PolyMeshViewer::
PolyMeshViewer(const char* _title, int _width, int _height)
  : GlutExaminer(_title, _width, _height)
{
  mesh_.request_face_normals();
  mesh_.request_vertex_normals();

  clear_draw_modes();
  add_draw_mode("Wireframe");
  add_draw_mode("Hidden Line");
  add_draw_mode("Solid Flat");
  add_draw_mode("Solid Smooth");
  set_draw_mode(3);
}


//-----------------------------------------------------------------------------


bool
PolyMeshViewer::
open_mesh(const char* _filename)
{
  // load mesh
  if (OpenMesh::IO::read_mesh(mesh_, _filename))
  {
    // set center and radius
    Mesh::ConstVertexIter  v_it(mesh_.vertices_begin()), 
                           v_end(mesh_.vertices_end());
    Mesh::Point            bbMin, bbMax;

    bbMin = bbMax = mesh_.point(v_it);
    for (; v_it!=v_end; ++v_it)
    {
      bbMin.minimize(mesh_.point(v_it));
      bbMax.maximize(mesh_.point(v_it));
    }
    set_scene( (Vec3f)(bbMin + bbMax)*0.5, 0.5*(bbMin - bbMax).norm());

    // compute face & vertex normals
    mesh_.update_normals();


    // update face indices for faster rendering
    update_face_indices();

    // info
    std::cerr << mesh_.n_vertices() << " vertices, "
	      << mesh_.n_faces()    << " faces\n";

    return true;
  }

  return false;
}


//-----------------------------------------------------------------------------


void
PolyMeshViewer::
update_face_indices()
{
  Mesh::ConstFaceIter        f_it(mesh_.faces_sbegin()), 
                             f_end(mesh_.faces_end());
  Mesh::ConstFaceVertexIter  fv_it;

  indices_.clear();
  //counting number of vertices per face
 Mesh::FaceVertexIter fvi_it = mesh_.fv_iter(f_it);
 NumVerticesPerFace=mesh_.valence(f_it);
 //for(; fvi_it; ++fvi_it) 
    //NumVerticesPerFace++;

  indices_.reserve(mesh_.n_faces()*NumVerticesPerFace);
  std::cout << "mesh indices updated" << std::endl;
  

  for (; f_it!=f_end; ++f_it)
  {
    indices_.push_back((fv_it=mesh_.cfv_iter(f_it)).handle().idx());
	for (int i=1;i<NumVerticesPerFace;i++)
		indices_.push_back((++fv_it).handle().idx());
    //indices_.push_back((++fv_it).handle().idx());
  }
}


//-----------------------------------------------------------------------------


void 
PolyMeshViewer::
draw(const std::string& _draw_mode)
{
  if (indices_.empty())
  {
    GlutExaminer::draw(_draw_mode);
    return;
  }



  if (_draw_mode == "Wireframe")
  {
    glDisable(GL_LIGHTING);
    glColor3f(1.0, 1.0, 1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glEnableClientState(GL_VERTEX_ARRAY);
    GL::glVertexPointer(mesh_.points());

    if (NumVerticesPerFace==4)
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	else
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);

    glDisableClientState(GL_VERTEX_ARRAY);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }


  else if (_draw_mode == "Hidden Line")
  {

	  glDisable(GL_LIGHTING);
	  glShadeModel(GL_SMOOTH);
	  glColor3f(0.0, 0.0, 0.0);

	  glEnableClientState(GL_VERTEX_ARRAY);
	  GL::glVertexPointer(mesh_.points());

	  glDepthRange(0.01, 1.0);
	  if (NumVerticesPerFace==4)
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	else
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	  glDisableClientState(GL_VERTEX_ARRAY);
	  glColor3f(1.0, 1.0, 1.0);

	  glEnableClientState(GL_VERTEX_ARRAY);
	  GL::glVertexPointer(mesh_.points());

	  glDrawBuffer(GL_BACK);
	  glDepthRange(0.0, 1.0);
	  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	  glDepthFunc(GL_LEQUAL);
	  if (NumVerticesPerFace==4)
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	else
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	  
	  glDisableClientState(GL_VERTEX_ARRAY);
	  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	  glDepthFunc(GL_LESS);

  }


  else if (_draw_mode == "Solid Flat")
  {
    Mesh::ConstFaceIter        f_it(mesh_.faces_begin()), 
                               f_end(mesh_.faces_end());
    Mesh::ConstFaceVertexIter  fv_it;

    glEnable(GL_LIGHTING);
    glShadeModel(GL_FLAT);

    if (NumVerticesPerFace==3)
		glBegin(GL_TRIANGLES);
	else
		glBegin(GL_QUADS);
    for (; f_it!=f_end; ++f_it)
    {
      GL::glNormal(mesh_.normal(f_it));
      fv_it = mesh_.cfv_iter(f_it.handle()); 
	  for (int i=0;i<NumVerticesPerFace;i++){
		GL::glVertex(mesh_.point(fv_it));
		++fv_it;
	  }
      /*GL::glVertex(mesh_.point(fv_it));
      ++fv_it;
      GL::glVertex(mesh_.point(fv_it));*/
    }
    glEnd();
  }


  else if (_draw_mode == "Solid Smooth")
  {
    glEnable(GL_LIGHTING);
    glShadeModel(GL_SMOOTH);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    GL::glVertexPointer(mesh_.points());
    GL::glNormalPointer(mesh_.vertex_normals());
	
    if (NumVerticesPerFace==4)
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
	else
		glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
  }
}


//=============================================================================
