#ifndef SUBDIVISION_VIEWER_HH
#define SUBDIVISION_VIEWER_HH

#include "PolyMeshViewer.hh"
#include <gmm.h>

class SubdivisionViewer : public PolyMeshViewer
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

	

	Mesh::Point													getFaceCentroid(Mesh::FaceHandle fh);
	Mesh::Point													getEdgeNewPoint(Mesh::EdgeHandle eh);
	Mesh::Point													getCatmullClarkOrigVertexNewLocation(Mesh::VertexHandle vh);
	void														addNewCatmullClarkConnectivity(Mesh::FaceHandle fh);
	Mesh::Point													getIncidentFacesAverageMidpoint(Mesh::VertexHandle vh);
	Mesh::Point													getIncidentEdgesAverageMidpoint(Mesh::VertexHandle vh);
	Mesh::Point													getIncidentBoundaryEdgesAverageMidpoint(Mesh::VertexHandle vh);
	Mesh::Point													getEdgeMidpoint(Mesh::EdgeHandle eh);
	int															getVertexValence(Mesh::VertexHandle vh);

	OpenMesh::VPropHandleT<Mesh::Point>							vpos_;
	OpenMesh::FPropHandleT<Mesh::Point>							fcentroid_;
	OpenMesh::EPropHandleT<Mesh::Point>							enewpoint_;
	OpenMesh::EPropHandleT<VertexHandle>						enewvertex_;
	Mesh::Point													_bbMin3D, 
																_bbMax3D;

	struct QuadFaceCorners
	{
		Mesh::VertexHandle v0;
		Mesh::VertexHandle v1;
		Mesh::VertexHandle v2;
		Mesh::VertexHandle v3;

		QuadFaceCorners(Mesh::VertexHandle _v0, Mesh::VertexHandle _v1, Mesh::VertexHandle _v2, Mesh::VertexHandle _v3) :
			v0(_v0),
			v1(_v1),
			v2(_v2),
			v3(_v3)
		{}
	};

};

#endif // SUBDIVISION_VIEWER_HH defined

