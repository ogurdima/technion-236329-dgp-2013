#include "UniformLaplacian.h"


UniformLaplacian::UniformLaplacian(Mesh _m) :
Laplacian(_m)
{

}


UniformLaplacian::~UniformLaplacian(void)
{
}


OpenMesh::Vec3f UniformLaplacian::operator()(Mesh::VertexIter vit)
{
	if (m.is_boundary(vit)) {
		return Vec3f(0,0,0);
	}
	Mesh::VertexVertexIter vvit;
	int				neighbors	= 0;
	Point			curVertex	= m.point(vit);
	Point			midPoint	= Mesh::Point(0.0, 0.0, 0.0);

	for (vvit = m.vv_iter(vit); vvit; ++vvit)
	{
		midPoint += m.point(vvit);
		neighbors++;
	}
	midPoint = midPoint * (1.0/neighbors);
	return (midPoint - curVertex);
}