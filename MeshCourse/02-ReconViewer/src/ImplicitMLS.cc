//=============================================================================


#include "ImplicitMLS.hh"


//== IMPLEMENTATION ==========================================================
ImplicitMLS::ImplicitMLS( const std::vector<Vec3f>& _points, 
						 const std::vector<Vec3f>& _normals )
						 : points_(_points), normals_(_normals), InvBetaSquare_(-1)
{
	//////////////////////////////////////////////////////////////////////
	// INSERT CODE:
	// 1) compute beta for the gaussian functions
	// 2) store the inverse of beta square for further use in operator ()
	//////////////////////////////////////////////////////////////////////
	float betha = calculateBetha();
	assert(betha != 0 );
	InvBetaSquare_ = 1.0 / (betha * betha);
}


float ImplicitMLS::operator()(const Vec3f& _p) const
{
	float dist(0);
	float d_i, phi_i, dist_i;
	//////////////////////////////////////////////////////////////////////
	// INSERT CODE:
	// 1) compute MLS distance to tangent planes for point p_
	//////////////////////////////////////////////////////////////////////
	for (int i = 0; i < points_.size(); i++) {
		dist_i = (_p - points_[i]).length();
		d_i = normals_[i] | (_p - points_[i]);
		phi_i = expf( -( (dist_i*dist_i) * InvBetaSquare_ ) );
		dist += d_i * phi_i;
	}

	return dist;
}

//=============================================================================

int ImplicitMLS::getEuclidClosestNeighbor(const int ptidx) const
{
	float curdist = -1;
	float mindist = -1;
	float closestNeighborIdx = -1;
	const Vec3f& pt = points_[ptidx];
	for (int i = 0; i < points_.size(); i++) {
		if (ptidx == i) {
			continue;
		}
		curdist = (points_[i] - pt).length();
		if (closestNeighborIdx < 0 || curdist < mindist) {
			closestNeighborIdx = i;
			mindist = curdist;
		}
	}
	assert(closestNeighborIdx > -1);
	return closestNeighborIdx;
}

float ImplicitMLS::calculateBetha() const
{
	float sum = 0;
	int num = 0;
	Vec3f closest;
	for(int i = 0; i < points_.size(); i++) {
		int j = getEuclidClosestNeighbor(i);
		closest = points_[j];
		sum += (closest - points_[i]).length();
		num++;
	}
	if (num == 0) {
		return 0;
	}
	float avg = sum / (float) num;
	return 2*avg;
}