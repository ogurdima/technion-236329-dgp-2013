#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


class Laplacian
{
public:

	typedef OpenMesh::TriMesh_ArrayKernelT<>	Mesh;
	typedef Mesh::Point							Point;
	typedef OpenMesh::Vec3f						Vec3f;

	Laplacian(Mesh _m): m(_m) {}
	virtual ~Laplacian() {}
	virtual float operator()(Mesh::VertexIter) = 0;
protected:
	Mesh m;
};

