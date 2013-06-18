#include "HarmonicMapViewer.hh"
#include <windows.h>

static bool			_ParameterizationComputed_u		=	false, 
					_ParameterizationComputed_h		=	false,
					_BoundingBox2DComputed			=	false;

HarmonicMapViewer::HarmonicMapViewer(const char* _title, int _width, int _height, int iRepeat) :
MeshViewer(_title, _width, _height)
{ 
	_Repeat						=	iRepeat;
	_TextureCoordinates_u		=	NULL;
	_TextureCoordinates_h		=	NULL;
	_ParameterizationMode_		=	NoParameterization;
	mesh_.request_vertex_colors();

	mesh_.add_property(vpos_);
	mesh_.add_property(vparam_u_);
	mesh_.add_property(vparam_h_);
	mesh_.add_property(texcoord_u_);
	mesh_.add_property(texcoord_h_);

	mesh_.add_property(cotangentWeight);
	mesh_.add_property(harmonicWeight);
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
	calc_fixed_curcular_boundary_parametrization();
	calc_cotangent_weights();
	calc_uniform_weights();
	calc_harmonic_weights();
	

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

bool HarmonicMapViewer::open_mesh(const char* _meshfilename)
{
	if ( MeshViewer::open_mesh(_meshfilename) ) {
		// store vertex initial positions and 3D mesh bounding box
		Mesh::VertexIter v_it = mesh_.vertices_begin();
		_bbMin3D = _bbMax3D = mesh_.point(v_it);
		for (v_it; v_it != mesh_.vertices_end(); ++v_it) {
			mesh_.property(vpos_,v_it) = mesh_.point(v_it);
			_bbMin3D.minimize(mesh_.point(v_it));
			_bbMax3D.maximize(mesh_.point(v_it));
		}
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

void HarmonicMapViewer::calc_uniform_weights()
{
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); ++eit) {
		mesh_.property(uniformWeight, eit) = 1;
	}
}

void HarmonicMapViewer::calc_harmonic_weights() {
	// IMPLEMENT HERE TODO: compute harmonic weights and store in harmonicWeight eprop 
}

void HarmonicMapViewer::calc_cotangent_weights()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 3.3.a Compute cotangent weights for laplacian, and produce them in the mesh edge property eweight_
	// Use the weights from calc_weights(): eweight_
	// ------------- IMPLEMENT HERE ---------
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); ++eit)
	{
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

void HarmonicMapViewer::calc_uniform_parameterization()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.1 Uniform map computation:
	// Search and order boundary vertices
	// Compute boundary parameters
	// Solve the linear system for internal vertex parameters using solve_linear_system()
	// Store the parameters in the vparam_u_ vertex property
	// ------------- IMPLEMENT HERE ---------
	int n = mesh_.n_vertices();
	gmmMatrix Mx(n,n);
	gmmMatrix My(n,n);
	gmmVector bx(n);
	gmmVector by(n);
	gmmVector x(n);
	gmmVector y(n);
	std::vector<VertexHandle> vertexIndex;
	for(Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); ++vit) {
		vertexIndex.push_back(vit.handle());
	}

	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			Mx(i,j) = calc_matrix_coefficient(vertexIndex[i], vertexIndex[j], uniformWeight);
			My(i,j) = Mx(i,j);
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
	solve_linear_system(Mx, bx, x);
	solve_linear_system(My, by, y);
	for (int i = 0; i < n; i++) {
		mesh_.property(vparam_u_, vertexIndex[i]) = Vec2f(x[i], y[i]);
	}

}

double HarmonicMapViewer::calc_matrix_coefficient(VertexHandle v1, VertexHandle v2, OpenMesh::EPropHandleT<Mesh::Scalar> prop) 
{
	if (v1 == v2) {
		if ( mesh_.is_boundary(v1) ) {
			return 1;
		}
		return -sum_one_ring_scalar_eprop(v1, prop);
	}
	else {
		for (Mesh::VertexOHalfedgeIter vheit = mesh_.voh_begin(v1); vheit; vheit++) {
			if (mesh_.to_vertex_handle(vheit.handle()) == v2) {
				EdgeHandle eh = mesh_.edge_handle(vheit.handle());
				return mesh_.property(prop, eh);
			}
			return 0;
		}
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

void HarmonicMapViewer::calc_harmonic_parameterization()
{
	// ------------- IMPLEMENT HERE ---------
	// TASK 5.2 harmonic map computation:
	// Search and order boundary vertices
	// Compute boundary parameters
	// Solve the linear system for internal vertex parameters using solve_linear_system()
	// Store the parameters in the vparam_h_ vertex property
	// ------------- IMPLEMENT HERE ---------
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
	cout << "Parameterization distortion: " <<endl;
	cout << (imode==Uniform? " * Uniform map: ": " * Harmonic map: ") <<endl;
	cout << " ---- Angle distortion: " << angle_distortion << " Area distortion: " << area_distortion << endl;
}

void HarmonicMapViewer::draw(const std::string& _draw_mode)
{
	if (indices_.empty()) {
		MeshViewer::draw(_draw_mode);
		return;
	}
	if (_draw_mode == "UV Domain") {
		if (_ParameterizationMode_!=NoParameterization) {
			OpenMesh::VPropHandleT<OpenMesh::Vec2f> & TexCoordHandle = (_ParameterizationMode_ == Uniform? vparam_u_: vparam_h_);
			Mesh::VertexIter v_it, v_end(mesh_.vertices_end());
			for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it) {
				OpenMesh::Vec2f UVCoord = mesh_.property(TexCoordHandle,v_it);
				mesh_.set_point(v_it, Mesh::Point(-UVCoord[0], UVCoord[1], 0.));
			}
			mesh_.update_normals();
			if (!_BoundingBox2DComputed) {
				Mesh::ConstVertexIter  v_it(mesh_.vertices_begin()), 
					v_end(mesh_.vertices_end());
				_bbMin2D = _bbMax2D = mesh_.point(v_it);
				for (; v_it!=v_end; ++v_it) {
					_bbMin2D.minimize(mesh_.point(v_it));
					_bbMax2D.maximize(mesh_.point(v_it));
				}
			}
			set_scene( (Vec3f)(_bbMin2D + _bbMax2D)*0.5, 0.5*(_bbMin2D - _bbMax2D).norm());
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
			float *& TextureCoord = (_ParameterizationMode_ == Uniform? _TextureCoordinates_u: _TextureCoordinates_h);
			OpenMesh::VPropHandleT<OpenMesh::Vec2f> & TexCoordHandle = (_ParameterizationMode_ == Uniform? texcoord_u_: texcoord_h_);
			if (!TextureCoord) {
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
		if (!_ParameterizationComputed_u) {
			calc_uniform_parameterization();
			ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
			calc_distortion(_ParameterizationMode_);
			_ParameterizationComputed_u = true;      
		}
		glutPostRedisplay();
		break;
	case 'H':
		_ParameterizationMode_ = Harmonic;
		if (!_ParameterizationComputed_h) {
			calc_harmonic_parameterization();
			ComputeTextureCoordinates(256, 256, _Repeat, _ParameterizationMode_);
			calc_distortion(_ParameterizationMode_);
			_ParameterizationComputed_h = true;       
		}
		glutPostRedisplay();
		break;
	case 'R': 
		_Repeat++;
		ComputeTextureCoordinates(256, 256, _Repeat, Uniform);
		ComputeTextureCoordinates(256, 256, _Repeat, Harmonic);
		printf("Number of repeats: %d\n",_Repeat);
		glutPostRedisplay();
		break;
	case 'E': 
		if (_Repeat>1) _Repeat--;
		ComputeTextureCoordinates(256, 256, _Repeat, Uniform);
		ComputeTextureCoordinates(256, 256, _Repeat, Harmonic);
		printf("Number of repeats: %d\n",_Repeat);
		glutPostRedisplay();
		break;
	default:
		MeshViewer::keyboard(key, x, y);
		break;
	}
}
