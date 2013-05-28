#pragma once
#include "Laplacian.h"
class UniformLaplacian : public Laplacian
{
public:
	UniformLaplacian(Mesh _m);
	virtual ~UniformLaplacian(void);
	virtual OpenMesh::Vec3f operator()(Mesh::VertexIter);
};

