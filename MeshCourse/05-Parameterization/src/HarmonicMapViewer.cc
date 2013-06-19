#include "HarmonicMapViewer.hh"
#include <windows.h>
#include <iostream>
#include <fstream>

static bool	_BoundingBox2DComputed			=	false;

HarmonicMapViewer::HarmonicMapViewer(const char* _title, int _width, int _height, int iRepeat) :
MeshViewer(_title, _width, _height)
{ 
	_Repeat						=	iRepeat;
	_TextureCoordinates_u		=	NULL;
	_TextureCoordinates_h		=	NULL;
	_TextureCoordinates_m		=	NULL;
	_ParameterizationMode_		=	NoParameterization;
	mesh_.request_vertex_colors();

	mesh_.add_property(vpos_);
	mesh_.add_property(vparam_u_);
	mesh_.add_property(vparam_h_);
	mesh_.add_property(texcoord_u_);
	mesh_.add_property(texcoord_h_);

	mesh_.add_property(vparam_m_);
	mesh_.add_property(texcoord_m_);

	mesh_.add_property(cotangentWeight);
	mesh_.add_property(meanValueWeight);
	mesh_.add_property(uniformWeight);
	mesh_.add_property(fixedBoundParametrization);

	add_draw_mode("UV Domain");
	add_draw_mode("Textured mesh");
	init();
}

HarmonicMapViewer::~HarmonicMapViewer()
{ 
	if (glIsTexture(textureID_)) {
		glDeleteTextures( 1, &textureID_);
	}
	if (_TextureCoordinates_u) {
		delete [] _TextureCoordinates_u;
	}
	if (_TextureCoordinates_h) {
		delete [] _TextureCoordinates_h;
	}
}

void HarmonicMapViewer::init()
{
	MeshViewer::init();
	update3DboundBox();
	calc_fixed_curcular_boundary_parametrization();
	calc_cotangent_weights();
	calc_uniform_weights();
	calc_mean_value_weights();
	

	// generate checkerboard-like image
	GLubyte tex[256][256][3];
	int index=0;
	for (int x=0; x<256; ++x) {
		for (int y=0; y<256; ++y) {
			if ((x<128&&y<128) ||(x>128&&y>128)) {
				tex[x][y][0] = 0;
				tex[x][y][1] = 255;
				tex[x][y][2] = 0;
			}
			else {
				tex[x][y][0] = 255;
				tex[x][y][1] = 255;
				tex[x][y][2] = 255;
			}
		}
	}
	// generate texture
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1); 
	if (!glIsTexture(textureID_)) {
		glGenTextures(1, &textureID_);
	}
	glBindTexture(GL_TEXTURE_2D, textureID_);
	// copy texture to GL
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
}

void HarmonicMapViewer::calc_fixed_curcular_boundary_parametrization()
{
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		mesh_.property(fixedBoundParametrization, vit.handle()) =	OpenMesh::Vec2f(0,0);
		mesh_.property(vparam_u_, vit.handle())					=	OpenMesh::Vec2f(0,0);
		mesh_.property(vparam_h_, vit.handle())					=	OpenMesh::Vec2f(0,0);
		mesh_.property(texcoord_u_, vit.handle())				=	OpenMesh::Vec2f(0,0);
		mesh_.property(texcoord_h_, vit.handle())				=	OpenMesh::Vec2f(0,0);
	}

	HalfedgeHandle firstOnBoundary;
	for (Mesh::HalfedgeIter eit = mesh_.halfedges_begin(); eit != mesh_.halfedges_end(); ++eit) {
		if (mesh_.is_boundary(eit.handle())) {
			firstOnBoundary = eit.handle();
			break;
		}
	}
	if (!mesh_.is_valid_handle(firstOnBoundary)) {
		return; // cannot do it at this point
	}
	std::vector<VertexHandle>	boundaryDisc;
	std::vector<double>			lengthUptoVertex;
	double						totalLength				= 0;
	VertexHandle				v						= mesh_.from_vertex_handle(firstOnBoundary);
	Mesh::Point					prevP					= mesh_.point(v); 
	lengthUptoVertex.push_back(totalLength);
	boundaryDisc.push_back(v);
	
	for (	HalfedgeHandle	he = mesh_.next_halfedge_handle(firstOnBoundary); 
							he != firstOnBoundary; he = mesh_.next_halfedge_handle(he)) 
	{
		v = mesh_.from_vertex_handle(he);
		lengthUptoVertex.push_back(totalLength);
		boundaryDisc.push_back(v);
		Mesh::Point p = mesh_.point(v); 
		totalLength += (p - prevP).length();
		prevP = p;
	}
	for (int i = 0; i < boundaryDisc.size(); i++) {
		double angleSoFar = 2 * M_PI * (lengthUptoVertex[i] / totalLength);
		mesh_.property(fixedBoundParametrization ,boundaryDisc[i]) = Vec2f(cos(angleSoFar), sin(angleSoFar));
	}
}

void HarmonicMapViewer::calc_uniform_parameterization()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.1 Uniform map computation:
	// Search and order boundary vertices
	// Compute boundary parameters
	// Solve the linear system for internal vertex parameters using solve_linear_system()
	// Store the parameters in the vparam_u_ vertex property
	// ------------- IMPLEMENT HERE ---------
	cout << "Computing uniform parametrization..." << endl;
	calc_weighted_parameterization(uniformWeight, vparam_u_);
}

void HarmonicMapViewer::calc_harmonic_parameterization()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.2 harmonic map computation:
	// Search and order boundary vertices
	// Compute boundary parameters
	// Solve the linear system for internal vertex parameters using solve_linear_system()
	// Store the parameters in the vparam_h_ vertex property
	// ------------- IMPLEMENT HERE ---------
	cout << "Computing harmonic parametrization..." << endl;
	calc_weighted_parameterization(cotangentWeight, vparam_h_);
}

void HarmonicMapViewer::calc_mean_value_parameterization()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.5 mean value map computation:
	// Search and order boundary vertices
	// Compute boundary parameters
	// Solve the linear system for internal vertex parameters using solve_linear_system()
	// Store the parameters in the vparam_m_ vertex property
	// ------------- IMPLEMENT HERE ---------
	cout << "Computing mean-value parametrization..." << endl;
	calc_weighted_parameterization(meanValueWeight, vparam_m_);
}

void HarmonicMapViewer::ComputeTextureCoordinates(int iTextureWidth, int iTextureHeight, int iRepeats, ParameterizationMode imode)
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.3 Compute texture coordinates for textured mesh
	// rendering using a texture image of dimension iTextureWidth*iTextureHeights and iRepeats repeats
	// and store them in a mesh property.
	// If imode is equals to Uniform, compute the texture coordinates using the
	// parameters stored in vparam_u_ and store the result in texcoord_u_.
	// If imode is equals to Harmonic, compute the texture coordinates using the
	// parameters stored in vparam_h_ and store the result in texcoord_h_.
	// ------------- IMPLEMENT HERE ---------
	double xLen = _bbMax2D[0] - _bbMin2D[0];
	double yLen = _bbMax2D[1] - _bbMin2D[1];
	OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop;
	OpenMesh::VPropHandleT<OpenMesh::Vec2f> vtex;
	switch (imode) {
	case Uniform:
		vprop = vparam_u_;
		vtex = texcoord_u_;
		break;
	case Harmonic:
		vprop = vparam_h_;
		vtex = texcoord_h_;
		break;
	case MeanValue:
		vprop = vparam_m_;
		vtex = texcoord_m_;
		break;
	default:
		return;
	}
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		Vec2f param = mesh_.property(vprop, vit.handle());

		double x = ((param[0] - _bbMin2D[0]) / xLen) * iRepeats;
		double y = ((param[1] - _bbMin2D[1]) / yLen) * iRepeats;
		mesh_.property(vtex, vit) = Vec2f(x,y);
		// cout << x << " " << y << endl;
	}
	float *& TextureCoord = (_ParameterizationMode_ == Uniform? _TextureCoordinates_u: 
				(_ParameterizationMode_ == MeanValue) ? _TextureCoordinates_m : _TextureCoordinates_h);
	if (NULL != TextureCoord) {
		delete[] TextureCoord;
		TextureCoord = NULL;
	}
}

void HarmonicMapViewer::calc_distortion(ParameterizationMode imode)
{
	float angle_distortion = 0.0, area_distortion = 0.0;
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.4 Compute distortion of triangle areas and angles
	// and print it in the output window.
	// If imode is equals to Uniform, uniform map distortion has to be 
	// computed.
	// If imode is equals to Harmonic, harmonic map distortion has to be
	// computed.
	// ------------- IMPLEMENT HERE ---------

	OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop;
	switch (imode) {
	case Uniform:
		vprop = vparam_u_;
		break;
	case Harmonic:
		vprop = vparam_h_;
		break;
	case MeanValue:
		vprop = vparam_m_;
		break;
	default:
		return;
	}

	double total2d = getTotal2dArea(vprop);
	double total3d = getTotal3dArea();

	for (Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		angle_distortion += getFaceAngleDistortion(fit.handle(), vprop);
		double face3dArea = getFace3dArea(fit.handle());
		double face2dArea = getFace2dArea(fit.handle(), vprop);
		double distElem = ((face3dArea / total3d) + (face2dArea / total2d));
		area_distortion += distElem * distElem;
	}

	cout << "Parameterization distortion: " <<endl;
	cout << (imode==Uniform? " * Uniform map: ": " * Harmonic map: ") <<endl;
	cout << " ---- Angle distortion: " << angle_distortion << " Area distortion: " << area_distortion << endl;
}

// My solution helpers
void HarmonicMapViewer::calc_weighted_parameterization(OpenMesh::EPropHandleT<Mesh::Scalar> eweight, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vcoord) 
{
	int									n = mesh_.n_vertices();
	gmmMatrix							Mx(n,n);
	gmmMatrix							My(n,n);
	gmmVector							bx(n);
	gmmVector							by(n);
	gmmVector							x(n);
	gmmVector							y(n);
	std::vector<VertexHandle>			vertexIndex;
	//std::map<VertexHandle, int>			reverseIndex;
	int cnt = 0;
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit, ++cnt) {
		vertexIndex.push_back(vit.handle());
		//reverseIndex.insert(std::pair<VertexHandle, int>(vit.handle(), cnt));
	}
	for (int i = 0; i < n; i++) {
		double sum = 0;
		for (int j = 0; j < n; j++) {
			Mx(i,j) = calc_matrix_coefficient(vertexIndex[i], vertexIndex[j], eweight);
			My(i,j) = Mx(i,j);
			sum += Mx(i,j);
		}
		// sanity check
		if ( mesh_.is_boundary(vertexIndex[i]) ) {
			assert(abs(sum - 1.0) < 0.0000001);
		}
		else {
			assert(abs(sum-0.0) < 0.0000001);
		}
		if (mesh_.is_boundary(vertexIndex[i])) {
			bx[i] = mesh_.property(fixedBoundParametrization, vertexIndex[i])[0];
			by[i] = mesh_.property(fixedBoundParametrization, vertexIndex[i])[1];
		}
		else {
			bx[i] = 0;
			by[i] = 0;
		}
	}
	// printMatrix(Mx, n);
	cout << "Solving 2 linear systems with " << n << " variables..." << endl;
	solve_linear_system(Mx, bx, x);
	solve_linear_system(My, by, y);
	for (int i = 0; i < n; i++) {
		mesh_.property(vcoord, vertexIndex[i]) = Vec2f(x[i], y[i]);
	}
	update2DboundBox(vcoord);
}

double HarmonicMapViewer::calc_matrix_coefficient(VertexHandle v1, VertexHandle v2, OpenMesh::EPropHandleT<Mesh::Scalar> prop) 
{
	if ( mesh_.is_boundary(v1) ) {
		if (v1 == v2) {
			return 1;
		}
		return 0;
	}
	if (v1 == v2) {
		return -sum_one_ring_scalar_eprop(v1, prop);
	}
	else {
		for (Mesh::VertexOHalfedgeIter vheit = mesh_.voh_begin(v1); vheit; vheit++) {
			if (mesh_.to_vertex_handle(vheit.handle()) == v2) {
				EdgeHandle eh = mesh_.edge_handle(vheit.handle());
				return mesh_.property(prop, eh);
			}			
		}
		return 0;
	}
}

bool HarmonicMapViewer::vertices_are_neighbors(VertexHandle v1, VertexHandle v2)
{
	for (Mesh::VertexVertexIter vvit = mesh_.vv_begin(v1); vvit; vvit++) {
		if (vvit.handle() == v2) {
			return true;
		}
	}
	return false;
}

double HarmonicMapViewer::sum_one_ring_scalar_eprop(VertexHandle v, OpenMesh::EPropHandleT<Mesh::Scalar> prop) 
{
	double sum = 0;
	for (Mesh::VertexEdgeIter veit = mesh_.ve_begin(v); veit; veit++) {
		sum += mesh_.property(prop, veit.handle());
	}
	return sum;
}

double HarmonicMapViewer::getFaceAngleDistortion(Mesh::FaceHandle f, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop)
{
	HalfedgeHandle he1 = mesh_.halfedge_handle(f);
	HalfedgeHandle he2 = mesh_.next_halfedge_handle(he1);
	HalfedgeHandle he3 = mesh_.next_halfedge_handle(he2);

	Vec2f p1 = mesh_.property(vprop, mesh_.from_vertex_handle(he1));
	Vec2f p2 = mesh_.property(vprop, mesh_.from_vertex_handle(he2));
	Vec2f p3 = mesh_.property(vprop, mesh_.from_vertex_handle(he3));

	double dist = 0;
	double dprod;
	double alpha;
	double betha;
	Vec2f v1;
	Vec2f v2;

	// he1 and he2
	v1 = (p2 - p1).normalize();
	v2 = (p3 - p2).normalize();
	dprod = (v1 | v2);
	betha = acos( std::min( 0.99, std::max(-0.99, dprod ) ) );
	alpha = calc_angle_between_he(he1, he2);
	dist += (alpha - betha)*(alpha - betha);
	// he2 and he3
	v1 = (p3 - p2).normalize();
	v2 = (p1 - p3).normalize();
	dprod = (v1 | v2);
	betha = acos( std::min( 0.99, std::max(-0.99, dprod ) ) );
	alpha = calc_angle_between_he(he2, he3);
	dist += (alpha - betha)*(alpha - betha);
	// he3 and he1
	v1 = (p1 - p3).normalize();
	v2 = (p2 - p1).normalize();
	dprod = (v1 | v2);
	betha = acos( std::min( 0.99, std::max(-0.99, dprod ) ) );
	alpha = calc_angle_between_he(he3, he1);
	dist += (alpha - betha)*(alpha - betha);
	return dist;
}

double HarmonicMapViewer::getTotal3dArea()
{
	double area = 0;
	for (Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		area += getFace3dArea(fit.handle());	
	}
	return area;
}

double HarmonicMapViewer::getTotal2dArea(OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop)
{
	double area = 0;
	for (Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		area += getFace2dArea(fit.handle(), vprop);	
	}
	return area;
}

double HarmonicMapViewer::getFace3dArea(Mesh::FaceHandle f)
{
	HalfedgeHandle he1 = mesh_.halfedge_handle(f);
	HalfedgeHandle he2 = mesh_.next_halfedge_handle(he1);
	Vec3f a = ( mesh_.point(mesh_.to_vertex_handle(he2)) - mesh_.point(mesh_.from_vertex_handle(he2)) );
	Vec3f b = ( mesh_.point(mesh_.from_vertex_handle(he1)) - mesh_.point(mesh_.to_vertex_handle(he1)) );
	double _3dArea = abs( (a%b).length() );
	return _3dArea;
}

double HarmonicMapViewer::getFace2dArea(Mesh::FaceHandle f, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop)
{
	HalfedgeHandle he1 = mesh_.halfedge_handle(f);
	HalfedgeHandle he2 = mesh_.next_halfedge_handle(he1);
	HalfedgeHandle he3 = mesh_.next_halfedge_handle(he2);
	Vec2f p1 = mesh_.property(vprop, mesh_.from_vertex_handle(he1));
	Vec2f p2 = mesh_.property(vprop, mesh_.from_vertex_handle(he2));
	Vec2f p3 = mesh_.property(vprop, mesh_.from_vertex_handle(he3));
	Vec3f v1 = Vec3f(p1[0],p1[1],0);
	Vec3f v2 = Vec3f(p2[0],p2[1],0);
	Vec3f v3 = Vec3f(p3[0],p3[1],0);
	double _2dArea = abs( ((v2-v1)%(v3-v2)).length() );
	return _2dArea;
}

// Weights computation
void HarmonicMapViewer::calc_uniform_weights()
{
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); ++eit) {
		mesh_.property(uniformWeight, eit) = 1;
	}
}

void HarmonicMapViewer::calc_mean_value_weights() {
	// IMPLEMENT HERE TODO: compute harmonic weights and store in meanValueWeight eprop 
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); ++eit) {
		Mesh::Scalar weight = 0;
		Mesh::HalfedgeHandle he = mesh_.halfedge_handle(eit.handle(), 0);
		Mesh::HalfedgeHandle the = mesh_.opposite_halfedge_handle(he);

		double angle1 = calc_angle_between_he(the, mesh_.next_halfedge_handle(he));
		double angle2 = calc_angle_between_he(mesh_.prev_halfedge_handle(the), he);
		double dist = (mesh_.point(mesh_.to_vertex_handle(he)) - mesh_.point(mesh_.from_vertex_handle(he))).length();
		weight = (tan(angle1 / 2) + tan(angle2 / 2)) / (2 * dist);
		mesh_.property(meanValueWeight, eit) = weight;
	}
}

double HarmonicMapViewer::calc_angle_between_he(Mesh::HalfedgeHandle he1, Mesh::HalfedgeHandle he2)
{
	Vec3f v2 = ( mesh_.point(mesh_.to_vertex_handle(he2)) - mesh_.point(mesh_.from_vertex_handle(he2)) ).normalize();
	Vec3f v1 = ( mesh_.point(mesh_.to_vertex_handle(he1)) - mesh_.point(mesh_.from_vertex_handle(he1)) ).normalize();
	double dprod = (v1 | v2);
	double alpha = acos( std::min( 0.99, std::max(-0.99, dprod ) ) );
	return alpha;
}

void HarmonicMapViewer::calc_cotangent_weights()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 3.3.a Compute cotangent weights for laplacian, and produce them in the mesh edge property eweight_
	// Use the weights from calc_weights(): eweight_
	// ------------- IMPLEMENT HERE ---------
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); ++eit) {
		Mesh::Scalar weight = 0;
		double w1, w2;
		// Left triangle
		Mesh::HalfedgeHandle he1 = mesh_.halfedge_handle(eit.handle(), 0);
		w1 = calc_he_cotangent_weight(he1);
		// Right triangle
		Mesh::HalfedgeHandle the1 = mesh_.opposite_halfedge_handle(he1);
		w2 = calc_he_cotangent_weight(the1);

		weight = 0.5 * ( w1 + w2 );
		weight = std::max(0.0f, weight);
		mesh_.property(cotangentWeight, eit) = weight;
	}
}

double HarmonicMapViewer::calc_he_cotangent_weight(Mesh::HalfedgeHandle he1)
{
	if ( mesh_.is_boundary(he1) ) {
		return 0;
	}
	Mesh::HalfedgeHandle he2 = mesh_.next_halfedge_handle(he1);
	Mesh::HalfedgeHandle he3 = mesh_.next_halfedge_handle(he2);
	Mesh::VertexHandle v1 = mesh_.from_vertex_handle(he1);
	Mesh::VertexHandle v2 = mesh_.from_vertex_handle(he2);
	Mesh::VertexHandle v3 = mesh_.from_vertex_handle(he3);

	Vec3f vec1 = (mesh_.point(v2) - mesh_.point(v3)).normalize();
	Vec3f vec2 = (mesh_.point(v1) - mesh_.point(v3)).normalize();
	double dprod = (vec1 | vec2);
	double alpha = acos( std::min( 0.99, std::max(-0.99, dprod ) ) );
	return (1.0 / tan(alpha));
}

// Misc
bool HarmonicMapViewer::open_mesh(const char* _meshfilename)
{
	if ( MeshViewer::open_mesh(_meshfilename) ) {
		// store vertex initial positions and 3D mesh bounding box
		return true;
	}
	return false;
}

void HarmonicMapViewer::solve_linear_system( gmmMatrix& _M, gmmVector& _b, gmmVector& _x)
{
	unsigned int N = _b.size();
	_x.resize(N);
	std::vector< size_t >  ipvt(N);
	gmm::lu_factor( _M, ipvt );
	gmm::lu_solve( _M, ipvt, _x, _b );
} 

void HarmonicMapViewer::draw(const std::string& _draw_mode)
{
	if (indices_.empty()) {
		MeshViewer::draw(_draw_mode);
		return;
	}
	if (_draw_mode == "UV Domain") {
		if (_ParameterizationMode_!=NoParameterization) {
			OpenMesh::VPropHandleT<OpenMesh::Vec2f> & TexCoordHandle = 
				(_ParameterizationMode_ == Uniform? vparam_u_: (_ParameterizationMode_ == MeanValue) ? vparam_m_ : vparam_h_);
			Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
			for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it) {
				OpenMesh::Vec2f UVCoord = mesh_.property(TexCoordHandle,v_it);
				mesh_.set_point(v_it, Mesh::Point(-UVCoord[0], UVCoord[1], 0.));
			}
			mesh_.update_normals();
			Vec3f bbMaxIn3d = Vec3f(_bbMax2D[0], _bbMax2D[1], 0);
			Vec3f bbMinIn3d = Vec3f(_bbMin2D[0], _bbMin2D[1], 0);
			set_scene( (Vec3f)(bbMinIn3d + bbMaxIn3d)*0.5, 0.5*(bbMinIn3d - bbMaxIn3d).norm());
			MeshViewer::draw("Wireframe");
		}
	}
	else {
		Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
		for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it) {
			mesh_.set_point(v_it, mesh_.property(vpos_,v_it));
		}
		mesh_.update_normals();
		set_scene( (Vec3f)(_bbMin3D + _bbMax3D)*0.5, 0.5*(_bbMin3D - _bbMax3D).norm());

		if (_draw_mode == "Textured mesh" && _ParameterizationMode_!=NoParameterization) {
			float *& TextureCoord = (_ParameterizationMode_ == Uniform? _TextureCoordinates_u: 
				(_ParameterizationMode_ == MeanValue) ? _TextureCoordinates_m : _TextureCoordinates_h);
			OpenMesh::VPropHandleT<OpenMesh::Vec2f> & TexCoordHandle = (_ParameterizationMode_ == Uniform? texcoord_u_: 
				(_ParameterizationMode_ == MeanValue) ? texcoord_m_ : texcoord_h_);
			if (NULL == TextureCoord) {
				int nvertices = mesh_.n_vertices();
				TextureCoord = new float[2*nvertices];
				Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
				int index=0;
				for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it) {
					OpenMesh::Vec2f UVParam = mesh_.property(TexCoordHandle,v_it);
					TextureCoord[index++] = UVParam[0];
					TextureCoord[index++] = UVParam[1];
				}
			}
			

			glEnable( GL_TEXTURE_2D ); 
			glEnable(GL_LIGHTING);
			glShadeModel(GL_SMOOTH);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			GL::glVertexPointer(mesh_.points());
			GL::glNormalPointer(mesh_.vertex_normals());
			GL::glTexCoordPointer(2, GL_FLOAT, 0, TextureCoord);
			glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, &indices_[0]);
			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glDisableClientState(GL_TEXTURE_COORD_ARRAY);
			glDisable( GL_TEXTURE_2D );
		}
		else {
			MeshViewer::draw(_draw_mode);
		}
	}
}

void HarmonicMapViewer::keyboard(int key, int x, int y)
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
			if(GetOpenFileName(&ofn)) {
				open_mesh(szFileName);
				glutPostRedisplay();
				init();
			}
		}
		break;
	case 'U':
		_ParameterizationMode_ = Uniform;
		_BoundingBox2DComputed = false;
		calc_uniform_parameterization();
		ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
		calc_distortion(_ParameterizationMode_);
		glutPostRedisplay();
		break;
	case 'H':
		_ParameterizationMode_ = Harmonic;
		_BoundingBox2DComputed = false;
		calc_harmonic_parameterization();
		ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
		calc_distortion(_ParameterizationMode_);
		glutPostRedisplay();
		break;
	case 'M':
		_ParameterizationMode_ = MeanValue;
		_BoundingBox2DComputed = false;
		calc_mean_value_parameterization();
		ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
		calc_distortion(_ParameterizationMode_);
		glutPostRedisplay();
		break;
	case 'R': 
		_Repeat++;
		ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
		printf("Number of repeats: %d\n",_Repeat);
		glutPostRedisplay();
		break;
	case 'E': 
		if (_Repeat>1) _Repeat--;
		ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
		printf("Number of repeats: %d\n",_Repeat);
		glutPostRedisplay();
		break;
	default:
		MeshViewer::keyboard(key, x, y);
		break;
	}
}

void HarmonicMapViewer::printMatrix(gmmMatrix M, int n) {
	std::ofstream myfile;
	myfile.open("example.txt");
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			myfile << M(i,j) << "  ";
		}
		myfile << std::endl;
	}
	myfile.close();
}

void HarmonicMapViewer::update3DboundBox()
{
	Mesh::VertexIter v_it = mesh_.vertices_begin();
	if (!mesh_.is_valid_handle(v_it)) {
		return;
	}
	_bbMin3D = _bbMax3D = mesh_.point(v_it);
	for (v_it; v_it != mesh_.vertices_end(); ++v_it) {
		mesh_.property(vpos_,v_it) = mesh_.point(v_it);
		_bbMin3D.minimize(mesh_.point(v_it));
		_bbMax3D.maximize(mesh_.point(v_it));
	}
}

void HarmonicMapViewer::update2DboundBox(OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop)
{
	if (_BoundingBox2DComputed) {
		return;
	}
	Mesh::ConstVertexIter	vit(mesh_.vertices_begin()), 
							v_end(mesh_.vertices_end());
	_bbMin2D = _bbMax2D = mesh_.property(vprop, vit);
	for (; vit != v_end; ++vit) {
		_bbMin2D.minimize( mesh_.property(vprop, vit) );
		_bbMax2D.maximize( mesh_.property(vprop, vit) );
	}
	_BoundingBox2DComputed = true;
}