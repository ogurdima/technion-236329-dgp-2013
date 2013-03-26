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
//  CLASS ValenceViewer - IMPLEMENTATION
//
//=============================================================================


//== INCLUDES =================================================================

#include <afxdlgs.h>

#include "ValenceViewer.hh"
#include <vector>
#include <float.h>



//== IMPLEMENTATION ========================================================== 


#define MAX_VALENCE 10000


ValenceViewer::ValenceViewer(const char* _title, int _width, int _height) : 
MeshViewer(_title, _width, _height), 
maxValence(1),
minValence(0)
{ 
	mesh_.request_vertex_colors();

	add_draw_mode("Vertex Valences");

	// Add custom menu entries here
	glutAddMenuEntry("Load Geometry", LOAD_GEOMETRY);


	// Adding custom property: 
	mesh_.add_property(valence_vprop);

}

// Overriden virtual method - fetch class specific IDs here
void ValenceViewer::processmenu(int i) 
{
	switch (i) 
	{
		case LOAD_GEOMETRY: 
		{
			CFileDialog dlg(TRUE,_T(".off"),NULL,NULL,_T("*.off|*.*"));
			if(dlg.DoModal()==IDOK)
			{
				bool ok = true;
				open_mesh(dlg.GetPathName());
			}
		}
		break;
		default: // default - let superclass handle this
		{
			MeshViewer::processmenu(i);
		}
	}
}

//-----------------------------------------------------------------------------


ValenceViewer::
	~ValenceViewer()
{
}

//-----------------------------------------------------------------------------

bool
	ValenceViewer::
	open_mesh(const char* _filename)
{
	// load mesh
	if (MeshViewer::open_mesh(_filename))
	{
		// reset extreme valence values
		maxValence = 1;
		minValence = 0;
		// compute vertex valence and color coding
		calc_valences();
		color_coding();

		glutPostRedisplay();
		return true;
	}
	return false;
}


//-----------------------------------------------------------------------------


void 
	ValenceViewer::
	calc_valences()
{
	// EXERCISE 1.2 /////////////////////////////////////////////////////////////
	// Compute valence of every vertex of "mesh_" and store them in each vertex
	// using for example custom attributes via dynamic customization
	// (hint: use the Mesh::VertexIter iterator)
	
	// Dont want to do it, so let's leave it linear as it is now
	int valenceBucket[MAX_VALENCE] = {};

	// Iterate over all vertices one-by-one
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		int valence = 0;
		// iterate over 1-ring neighbor vertices, basically just count them
		for(Mesh::VertexVertexIter vvit = mesh_.vv_iter(vit); vvit; ++vvit) {
			valence++;
		}
		// store this value as a custom property of the vertex
		mesh_.property(valence_vprop, vit) = valence;
		// storing maxValence and minValence for O(1) access in future color matching
		if ( valence > maxValence ) {
			maxValence = valence;
		}
		if ( minValence == 0 || minValence > valence ) {
			minValence = valence;
		}
	}

	/////////////////////////////////////////////////////////////////////////////
}


//-----------------------------------------------------------------------------


void 
	ValenceViewer::
	color_coding()
{
	// EXERCISE 1.3 /////////////////////////////////////////////////////////////
	// Implement a color visualization of your choice that shows the valence of
	// veach ertex of "mesh_". 
	// (hint: use Mesh::Color color type)

	int dValence = maxValence - minValence;
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		int curValence = mesh_.property(valence_vprop, vit);
		int r = (255 * (curValence - minValence)) / dValence;
		int g = (255 - r);
		// color change is linear on vetex valence: max is red, min is green.
		// Looks a bit weird on allmost-regular models with a singular high-valence vertex.
		// Can be fixed using lerp until median valence value but whatever.
		mesh_.set_color(vit, Mesh::Color(r, g, 0));
	}

	/////////////////////////////////////////////////////////////////////////////
}


//-----------------------------------------------------------------------------


void 
	ValenceViewer::
	draw(const std::string& _draw_mode)
{

	if (indices_.empty())
	{
		MeshViewer::draw(_draw_mode);
		return;
	}

	if (_draw_mode == "Vertex Valences")
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
	}

	else MeshViewer::draw(_draw_mode);
}


//=============================================================================
