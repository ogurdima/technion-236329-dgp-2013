//=============================================================================


#ifndef MLS_HH
#define MLS_HH


//=============================================================================


#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <vector>
#include <float.h>


//=============================================================================


class ImplicitMLS
{
public:

	typedef OpenMesh::Vec3f Vec3f;

	
  // fit RBF to given constraints
  ImplicitMLS( const std::vector<Vec3f>& _points, 
               const std::vector<Vec3f>& _normals );

  // evaluate implicit at position _p
  float operator()(const Vec3f& _p) const;


private:

    const std::vector<Vec3f>&  points_;
    const std::vector<Vec3f>&  normals_;
    float                      InvBetaSquare_;
};


//=============================================================================
#endif // RBF_HH defined
//=============================================================================

