//=============================================================================


#include "ImplicitRBF.hh"


//== IMPLEMENTATION ==========================================================


ImplicitRBF::ImplicitRBF(	const std::vector<Vec3f>& _points, 
						 const std::vector<Vec3f>& _normals, 
						 float& epsilon, 
						 RBF& _rbf
						 ) :
rbf_(_rbf),
	epsilon_(epsilon)
{
	//////////////////////////////////////////////////////////////////////
	// INSERT CODE:
	// 1) collect constraints (on-surface and off-surface)
	// 2) setup matrix
	// 3) solve linear system for weights_
	//////////////////////////////////////////////////////////////////////
	std::cout << "Collecting constraints" << std::endl << std::flush;
	std::vector<float> vals;
	int n = _points.size();
	for (int i = 0; i < n; i++) {
		centers_.push_back( _points[i] );
		vals.push_back(0);
	}
	for (int i = 0; i < n; i++) {
		centers_.push_back( _points[i] + (_normals[i] * epsilon) );
		vals.push_back(epsilon);
	}

	std::cout << "Filling the matrix" << std::endl << std::flush;
	gmmMatrix M(2*n,2*n);
	for (int row = 0; row < 2*n; row++) {
		Vec3f p_i = centers_[row];
		for (int col = 0; col < 2*n; col++) {
			Vec3f c_i = centers_[col];
			M(row,col) = kernel(p_i, c_i);
		}
	}
	gmmVector B(2*n);
	for (int i = 0; i < B.size(); i++) {
		B[i] = vals[i];
	}
	gmmVector X(2*n);

	std::cout << "Solving the linear system" << std::endl << std::flush;
	solve_linear_system(M, B, X);

	weights_.clear();
	weights_.resize(X.size());
	for (int i = 0; i < B.size(); i++) {
		weights_[i] = X[i];
	}

}


//-----------------------------------------------------------------------------


void
	ImplicitRBF::solve_linear_system( gmmMatrix& _M, 
	gmmVector& _b, 
	gmmVector& _x)
{
	// solve linear system by gmm's LU factorization
	unsigned int N = _b.size();
	_x.resize(N);
	std::vector< size_t >  ipvt(N);
	gmm::lu_factor( _M, ipvt );
	gmm::lu_solve( _M, ipvt, _x, _b );
}


//-----------------------------------------------------------------------------


float ImplicitRBF::operator()(const Vec3f& _p) const
{
	std::vector<Vec3f>::const_iterator  
		c_it(centers_.begin()),
		c_end(centers_.end());

	std::vector<double>::const_iterator   
		w_it(weights_.begin());

	double f(0);

	for (; c_it!=c_end; ++c_it, ++w_it)
		f += *w_it * kernel(*c_it, _p);

	return f;
}


//=============================================================================
