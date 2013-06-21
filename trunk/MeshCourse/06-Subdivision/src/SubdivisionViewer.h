#ifndef SUBDIVISION_VIEWER_HH
#define SUBDIVISION_VIEWER_HH

#include "MeshViewer.hh"
#include <gmm.h>

class SubdivisionViewer : public MeshViewer
{
public:
																SubdivisionViewer(const char* _title, int _width, int _height);
																~SubdivisionViewer();
	virtual bool												open_mesh(const char* _meshfilename);

private:
	virtual void												init();
	virtual void												draw(const std::string& _draw_mode);
	virtual void												keyboard(int key, int x, int y);
	void														Perform_CatmullClark();
	void														Perform_Loop();

	OpenMesh::VPropHandleT<Mesh::Point>							vpos_;
	OpenMesh::FPropHandleT<Mesh::Point>							fcentroid_;
	OpenMesh::EPropHandleT<Mesh::Point>							enewpoint_;
	OpenMesh::EPropHandleT<VertexHandle>						enewvertex_;
	Mesh::Point													_bbMin3D, 
																_bbMax3D;
};

#endif // SUBDIVISION_VIEWER_HH defined

