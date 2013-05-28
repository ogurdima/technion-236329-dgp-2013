#include "LaplaceBeltrami.h"


LaplaceBeltrami::LaplaceBeltrami(Mesh _m, OpenMesh::EPropHandleT<Mesh::Scalar> _edgeWeightProp) :
Laplacian(_m),
edgeWeight(_edgeWeightProp)
{

}


LaplaceBeltrami::~LaplaceBeltrami(void)
{
}


OpenMesh::Vec3f LaplaceBeltrami::operator()(Mesh::VertexIter vit)
{
	Point origPoint = m.point(vit);
	Point midPoint = Point(0,0,0);
	float totalWeight;
	for (VertexOHalfedgeIter hit = m.voh_iter(vit); hit; ++hit)
	{
		Point neighbor = m.point( m.to_vertex_handle( hit.handle() ) );
		float weight = m.property( edgeWeight, m.edge_handle(hit) );
		midPoint += weight * neighbor;
		totalWeight += weight;
	}
	if (totalWeight < 0.0001) {
		midPoint = origPoint;
	}
	else {
		midPoint = midPoint * (1.0 / totalWeight);
	}
	return (midPoint - origPoint);
}