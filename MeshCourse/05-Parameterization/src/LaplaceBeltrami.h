#pragma once
#include "Laplacian.h"
class LaplaceBeltrami : public Laplacian
{
public:
	LaplaceBeltrami(Mesh _m, OpenMesh::EPropHandleT<Mesh::Scalar> _edgeWeightProp);
	~LaplaceBeltrami(void);
	virtual OpenMesh::Vec3f operator()(Mesh::VertexIter);

protected:
	OpenMesh::EPropHandleT<Mesh::Scalar> edgeWeight;
};

