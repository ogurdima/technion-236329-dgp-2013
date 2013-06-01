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
//  CLASS QualityViewer
//
//=============================================================================


#ifndef QUALVIEWERWIDGET_HH
#define QUALVIEWERWIDGET_HH

#include <MeshViewer.hh>

class QualityViewer : public MeshViewer
{
public:

													QualityViewer(const char* _title, int _width, int _height);
													~QualityViewer();
	virtual bool									open_mesh(const char* _filename);

protected:
	typedef OpenMesh::VPropHandleT<Mesh::Scalar>	ScalarVpropT;
	typedef OpenMesh::EPropHandleT<Mesh::Scalar>	ScalarEpropT;
	typedef OpenMesh::FPropHandleT<Mesh::Scalar>	ScalarFpropT;

	virtual void									init();
	virtual void									draw(const std::string& _draw_mode);
	void											calc_weights();
	double											calc_he_weight(Mesh::HalfedgeHandle he1);
	void											calc_mean_curvature();
	void											calc_uniform_mean_curvature();
	void											calc_gauss_curvature();
	void											calc_triangle_quality();
	void											face_color_coding();
	void											find_min_max(ScalarVpropT prop, Mesh::Scalar& min, Mesh::Scalar& max);
	Mesh::Color										value_to_color(float value, float min, float max);
	void											color_coding(ScalarVpropT prop);
	
	
	std::vector<float>								face_colors_;
	OpenMesh::VPropHandleT<Mesh::Scalar>			vweight_, 
													vunicurvature_, 
													vcurvature_, 
													vgausscurvature_;
	
	OpenMesh::EPropHandleT<Mesh::Scalar>			eweight_;
	OpenMesh::FPropHandleT<Mesh::Scalar>			tshape_;
	GLuint											textureID_;
};


#endif // QUALVIEWERWIDGET_HH defined

