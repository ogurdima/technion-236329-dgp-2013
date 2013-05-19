#include "UniformLaplacian.h"


UniformLaplacian::UniformLaplacian(Mesh _m) :
Laplacian(_m)
{

}


UniformLaplacian::~UniformLaplacian(void)
{
}


float UniformLaplacian::operator()(Mesh::VertexIter vit)
{
	Mesh::VertexVertexIter vvit;
	int				neighbors	= 0;
	Point			curVertex	= m.point(vit);
	Point			midPoint	= Mesh::Point(0.0, 0.0, 0.0);

	for (vvit = m.vv_iter(vit); vvit; ++vvit)
	{
		midPoint += m.point(vvit);
		neighbors++;
		if (m.is_boundary(vit))
		{
			Vec3f n = m.normal(vit);
			Vec3f offset = m.point(vvit) - curVertex;
			Vec3f normalOffset = (offset | n) * n;
			Vec3f tangentOffset = offset - normalOffset;
			Point tangentCompensator = curVertex + normalOffset - tangentOffset;
			midPoint += tangentCompensator;
			neighbors++;
		}
	}
	midPoint = midPoint * (1.0/neighbors);
	return (midPoint - curVertex).norm();
}