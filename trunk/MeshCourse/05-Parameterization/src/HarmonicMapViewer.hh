#ifndef HARMONIC_MAP_VIEWER_HH
#define HARMONIC_MAP_VIEWER_HH

#include <gmm.h>
#include "MeshViewer.hh"

class HarmonicMapViewer : public MeshViewer
{
public:
	enum ParameterizationMode				{	NoParameterization, Uniform, Harmonic, MeanValue	}; 
	typedef gmm::dense_matrix<double>		gmmMatrix;
	typedef std::vector<double>				gmmVector;

											HarmonicMapViewer(const char* _title, int _width, int _height, int iRepeat);
											~HarmonicMapViewer();
	virtual bool							open_mesh(const char* _meshfilename);
private:
	void									calc_uniform_parameterization();
	void									calc_harmonic_parameterization();
	void									ComputeTextureCoordinates(int iTextureWidth, int iTextureHeight, int iReapeats, ParameterizationMode imode);
	void									calc_distortion(ParameterizationMode imode);
	void									solve_linear_system( gmmMatrix& _A, gmmVector& _b, gmmVector& _x );
	virtual void							init();
	virtual void							draw(const std::string& _draw_mode);
	virtual void							keyboard(int key, int x, int y);

	void									calc_cotangent_weights();
	void									calc_uniform_weights();
	void									calc_mean_value_weights();
	double									calc_he_cotangent_weight(Mesh::HalfedgeHandle he1);
	void									calc_fixed_curcular_boundary_parametrization();
	bool									vertices_are_neighbors(VertexHandle v1, VertexHandle v2);
	double									sum_one_ring_scalar_eprop(VertexHandle v, OpenMesh::EPropHandleT<Mesh::Scalar> prop);
	double									calc_matrix_coefficient(VertexHandle v1, VertexHandle v2, OpenMesh::EPropHandleT<Mesh::Scalar> prop);
	void									printMatrix(gmmMatrix M, int n);
	void									calc_weighted_parameterization(OpenMesh::EPropHandleT<Mesh::Scalar> eweight, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vcoord);
	void									calc_mean_value_parameterization();
	double									calc_angle_between_he(Mesh::HalfedgeHandle he1, Mesh::HalfedgeHandle he2);
	void									update3DboundBox();
	void									update2DboundBox(OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop);
	double									getFaceAngleDistortion(Mesh::FaceHandle f, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop);
	double									getFace3dArea(Mesh::FaceHandle f);
	double									getFace2dArea(Mesh::FaceHandle f, OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop);
	double									getTotal3dArea();
	double									getTotal2dArea(OpenMesh::VPropHandleT<OpenMesh::Vec2f> vprop);

private:
	int _Repeat;
	OpenMesh::VPropHandleT<OpenMesh::Vec2f>	vparam_u_, vparam_h_, texcoord_u_, texcoord_h_ , vparam_m_ , texcoord_m_;

	OpenMesh::VPropHandleT<Mesh::Point>		vpos_;
	GLuint									textureID_;
	float									*_TextureCoordinates_u,
											*_TextureCoordinates_h,
											*_TextureCoordinates_m;
	Mesh::Point								_bbMin3D, 
											_bbMax3D;
	OpenMesh::Vec2f							_bbMin2D, 
											_bbMax2D;
	ParameterizationMode					_ParameterizationMode_;
	OpenMesh::EPropHandleT<Mesh::Scalar>	cotangentWeight;
	OpenMesh::EPropHandleT<Mesh::Scalar>	meanValueWeight;
	OpenMesh::EPropHandleT<Mesh::Scalar>	uniformWeight; //wat? :)
	OpenMesh::VPropHandleT<OpenMesh::Vec2f>	fixedBoundParametrization;	
};

#endif // HARMONIC_MAP_VIEWER_HH defined
