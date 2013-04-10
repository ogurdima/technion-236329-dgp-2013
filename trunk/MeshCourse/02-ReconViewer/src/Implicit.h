#ifndef IMPLICIT_HEADER_FILE
#define IMPLICIT_HEADER_FILE

class Implicit{
public:
	Implicit(){}
	~Implicit(){}

	virtual double operator()(const Vec3f& _p) const;
};


#endif