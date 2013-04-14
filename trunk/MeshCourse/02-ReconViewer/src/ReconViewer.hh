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
//  CLASS ReconViewer
//
//=============================================================================


#ifndef RECONVIEWERWIDGET_HH
#define RECONVIEWERWIDGET_HH



//== INCLUDES =================================================================



#include <MeshViewer.hh>
#include "Implicit.h"
#include <iostream>
#include <fstream>
#include "RBF.h"



//== CLASS DEFINITION =========================================================

typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;
typedef Mesh::Point                       Point;
typedef Mesh::Scalar                      Scalar;
typedef OpenMesh::Vec3d                   Vec3d;

#define MC_RESOLUTION  50


typedef enum {TRIHARMONIC, BSPLINE} ReconRBF; 


class ReconViewer : public MeshViewer
{
public:
   
  /// default constructor
  ReconViewer(const char* _title, int _width, int _height);

  // destructor
  ~ReconViewer();

  /// open mesh
  virtual bool open_mesh(const char* _filename);

  //overloading GLUT keyboard
  virtual void keyboard(int key, int x, int y);

  //construcing mesh from grid
  void MeshFromFunction(Implicit* ImpFunc);



protected:

  virtual void draw(const std::string& _draw_mode);
  std::vector<Point>   Points, Normals;
  float epsilon;
  float betha;
  ReconRBF rbf_to_use;

private:

};


//=============================================================================
#endif // RECONVIEWERWIDGET_HH defined
//=============================================================================

