#ifndef IMPLICIT_HEADER_FILE
#define IMPLICIT_HEADER_FILE

#include <OpenMesh/Core/Geometry/VectorT.hh>

class Implicit{
public:

	typedef OpenMesh::Vec3f Vec3f;

	Implicit(){}
	~Implicit(){}

	virtual float operator()(const Vec3f& _p) const = 0;
};


#endif