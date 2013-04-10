//=============================================================================


#include "ImplicitMLS.hh"


//== IMPLEMENTATION ==========================================================
ImplicitMLS::ImplicitMLS( const std::vector<Vec3f>& _points, 
                          const std::vector<Vec3f>& _normals )
: points_(_points), normals_(_normals)
{
  //////////////////////////////////////////////////////////////////////
  // INSERT CODE:
  // 1) compute beta for the gaussian functions
  // 2) store the inverse of beta square for further use in operator ()
  //////////////////////////////////////////////////////////////////////
}


float ImplicitMLS::operator()(const Vec3f& _p) const
{
  float dist(0);
  //////////////////////////////////////////////////////////////////////
  // INSERT CODE:
  // 1) compute MLS distance to tangent planes for point p_
  //////////////////////////////////////////////////////////////////////
  return dist;
}

//=============================================================================
