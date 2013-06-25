#include "SubdivisionViewer.h"
#include <windows.h>

static bool _SubdivisioneterizationComputed_u = false, 
			_SubdivisioneterizationComputed_h = false;
static bool _BoundingBox2DComputed = false;

// I don't know why but I need these typedefs for the code to compile
typedef OpenMesh::PolyMesh_ArrayKernelT<>					Mesh;
typedef OpenMesh::TriMesh_ArrayKernelT<>					TriMesh;

SubdivisionViewer::SubdivisionViewer(const char* _title, int _width, int _height): 
PolyMeshViewer(_title, _width, _height)
{ 
	mesh_.request_face_status();
	mesh_.request_halfedge_status();
	mesh_.request_edge_status();
	mesh_.request_vertex_status();

	init();
}



SubdivisionViewer::~SubdivisionViewer()
{ 

}

void SubdivisionViewer::init()
{
	PolyMeshViewer::init();
}

void SubdivisionViewer::Perform_CatmullClark()
{
	mesh_.add_property(fcentroid_);
	mesh_.add_property(enewpoint_);
	mesh_.add_property(enewvertex_);
	/***********Catmull-Clark Subdivision************
	1. Find face centroids and fill them in fcentroid_
	2. Find edge newpoints and fill them in enewpoint_
	3. Allocate new edge vertices in enewvertex_
	4. Delete the old faces, and enter the new faces
	NOTICE: make sure to call mesh_.garbage_collection after deleting new faces, or you
	might get mixed up when running over all faces\halfedges\vertices
	**********************************************************/
	for(Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		mesh_.property(fcentroid_, fit.handle()) = getFaceCentroid(fit.handle());
	}
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); eit++) {
		mesh_.property(enewpoint_, eit.handle()) = getEdgeNewCatmullClarkPoint(eit.handle());
	}
	for (Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); vit++) {
		mesh_.set_point(vit.handle(), getCatmullClarkOrigVertexNewLocation( vit.handle() ));
	}
	// Allocating vertices for new edge points. Face vertex is allocated in addNewCatmullClarkConnectivity
	// New faces are created and old faces are deleted in addNewCatmullClarkConnectivity
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); eit++) {
		mesh_.property(enewvertex_, eit.handle()) = mesh_.add_vertex( mesh_.property(enewpoint_, eit.handle()) );
		assert( mesh_.is_valid_handle( mesh_.property(enewvertex_, eit.handle()) ) );
	}
	std::vector<Mesh::FaceHandle> oldFaces;
	for(Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		oldFaces.push_back(fit.handle());
	}
	for (int i = 0; i < oldFaces.size(); i++) {
		addNewCatmullClarkConnectivity( oldFaces[i] );
	}

	mesh_.garbage_collection();
	mesh_.remove_property(fcentroid_);
	mesh_.remove_property(enewpoint_);
	mesh_.remove_property(enewvertex_);
	update_face_indices();
	mesh_.update_normals();
	std::cerr << mesh_.n_vertices() << " vertices, " << mesh_.n_faces() << " faces\n";
}


void SubdivisionViewer::Perform_Loop()
{
	mesh_.add_property(enewpoint_);
	mesh_.add_property(enewvertex_);
	/************Loop subdivision***************
	1. Find New Edge midpoints, and enter them in enewpoint_
	2. Allocate new edge vertices in enewvertex_
	3. Delete the old faces, and enter the new faces
	**********************************************/

	triTesselation();

	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); eit++) {
		mesh_.property(enewpoint_, eit.handle()) = getEdgeNewLoopPoint(eit.handle());
	}
	for (Mesh::VertexIter vit = mesh_.vertices_begin(); vit != mesh_.vertices_end(); vit++) {
		mesh_.set_point(vit.handle(), getLoopOrigVertexNewLocation( vit.handle() ));
	}
	// Allocating vertices for new edge points. Face vertex is allocated in addNewCatmullClarkConnectivity
	// New faces are created and old faces are deleted in addNewCatmullClarkConnectivity
	for (Mesh::EdgeIter eit = mesh_.edges_begin(); eit != mesh_.edges_end(); eit++) {
		mesh_.property(enewvertex_, eit.handle()) = mesh_.add_vertex( mesh_.property(enewpoint_, eit.handle()) );
		assert( mesh_.is_valid_handle( mesh_.property(enewvertex_, eit.handle()) ) );
	}
	std::vector<Mesh::FaceHandle> oldFaces;
	for(Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		oldFaces.push_back(fit.handle());
	}
	for (int i = 0; i < oldFaces.size(); i++) {
		addNewLoopConnectivity( oldFaces[i] );
	}

	mesh_.garbage_collection();
	mesh_.remove_property(enewpoint_);
	mesh_.remove_property(enewvertex_);
	update_face_indices();
	mesh_.update_normals();
	std::cerr << mesh_.n_vertices() << " vertices, " << mesh_.n_faces() << " faces\n";
}

// Loop
Mesh::Point SubdivisionViewer::getLoopOrigVertexNewLocation(Mesh::VertexHandle vh)
{
	if (mesh_.is_boundary(vh)) {
		return getLoopBoundaryVertexNewLocation(vh);
	}
	else {
		return getLoopInternalVertexNewLocation(vh);
	}
}

Mesh::Point SubdivisionViewer::getLoopBoundaryVertexNewLocation(Mesh::VertexHandle vh)
{
	Mesh::Point		P		=	mesh_.point(vh);
	Mesh::Point		sum		=	(P * 6);
	int				count	=	8;
	for (Mesh::VertexOHalfedgeIter heit = mesh_.voh_begin(vh); heit != mesh_.voh_end(vh); heit++) {
		EdgeHandle eh = mesh_.edge_handle(heit.handle());
		if ( mesh_.is_boundary(eh) ) {
			VertexHandle v = mesh_.to_vertex_handle(heit.handle());
			sum += (mesh_.point(v) * 1);
		}
	}
	return (sum / count);
}

Mesh::Point SubdivisionViewer::getLoopInternalVertexNewLocation(Mesh::VertexHandle vh)
{
	Mesh::Point		P		=	mesh_.point(vh);
	double			betha	=	loopBetha(vh);
	Mesh::Point		sum		=	Mesh::Point(0,0,0);
	int				k		=	0;
	for (Mesh::VertexVertexIter vit = mesh_.vv_begin(vh); vit != mesh_.vv_end(vh); vit++) {
		sum += (mesh_.point(vit.handle()) * betha);
		k++;
	}
	sum += P * (1 - (k*betha));
	/*cout << "Old: ( "<< P[0] << "," << P[1] << "," << P[2] <<") ";
	cout << "New: ( "<< sum[0] << "," << sum[1] << "," << sum[2] <<") ";
	cout << endl;*/
	return sum;
}

Mesh::Point SubdivisionViewer::getEdgeNewLoopPoint(Mesh::EdgeHandle eh)
{
	if (mesh_.is_boundary(eh)) {
		return getEdgeMidpoint(eh); // we want new point to remain on boundary
	}
	int count = 8;
	Mesh::Point sum(0,0,0);
	HalfedgeHandle he	= mesh_.halfedge_handle(eh, 0);
	HalfedgeHandle ohe	= mesh_.opposite_halfedge_handle(he);
	sum += (mesh_.point( mesh_.from_vertex_handle(he) ) * 3);
	sum += (mesh_.point( mesh_.to_vertex_handle(he) ) * 3);

	HalfedgeHandle nhe	= mesh_.next_halfedge_handle(he);
	HalfedgeHandle onhe	= mesh_.next_halfedge_handle(ohe);
	sum += (mesh_.point( mesh_.to_vertex_handle(nhe) ) * 1);
	sum += (mesh_.point( mesh_.to_vertex_handle(onhe) ) * 1);
	return sum / count;
}

double SubdivisionViewer::loopBetha(Mesh::VertexHandle vh)
{
	// 5/8 = 0.625, 3/8 = 0.375, 1/4 = 0.25
	int			n			=	getVertexValence(vh);
	double		nInv		=	1.0/((double)n);
	double		_cos		=	cos( 2 * nInv * M_PI );
	double		toSquare	=	(0.375) + ((0.25) * _cos);
	double		betha		=	nInv * ( (0.625) - (toSquare*toSquare) );
	return betha;
}

void SubdivisionViewer::addNewLoopConnectivity(Mesh::FaceHandle fh)
{
	Mesh::HalfedgeHandle			firstHe		=	mesh_.halfedge_handle(fh);
	Mesh::HalfedgeHandle			he			=	firstHe;
	std::vector<TriFaceCorners>		facesToAdd;
	std::vector<Mesh::VertexHandle>	newVertices;
	do {
		Mesh::HalfedgeHandle	prevHe	=	mesh_.prev_halfedge_handle(he);	
		Mesh::EdgeHandle		e		=	mesh_.edge_handle(he);
		Mesh::EdgeHandle		prevE	=	mesh_.edge_handle(prevHe);
		Mesh::VertexHandle		v		=	mesh_.from_vertex_handle(he);
		Mesh::VertexHandle		eV		=	mesh_.property(enewvertex_ ,e);
		Mesh::VertexHandle		prevEV	=	mesh_.property(enewvertex_ ,prevE);
		facesToAdd.push_back( TriFaceCorners(v, eV, prevEV) );
		newVertices.push_back(eV);

		he = mesh_.next_halfedge_handle(he);
	} while (he != firstHe);
	// Now we can delete the old face
	mesh_.delete_face(fh);
	// Now we can add the new faces
	mesh_.add_face(newVertices);
	for (int i = 0; i < facesToAdd.size(); i++) {
		mesh_.add_face(facesToAdd[i].v0, facesToAdd[i].v1, facesToAdd[i].v2);
	}
}

// Catmull clark
Mesh::Point SubdivisionViewer::getEdgeNewCatmullClarkPoint(Mesh::EdgeHandle eh)
{
	if (mesh_.is_boundary(eh)) {
		return getEdgeMidpoint(eh); // we want new point to remain on boundary
	}
	int count = 4;
	Mesh::Point sum(0,0,0);
	HalfedgeHandle he	= mesh_.halfedge_handle(eh, 0);
	HalfedgeHandle ohe	= mesh_.opposite_halfedge_handle(he);
	sum += mesh_.point( mesh_.from_vertex_handle(he) );
	sum += mesh_.point( mesh_.to_vertex_handle(he) );
	if (!mesh_.is_boundary(he)) {
		sum += mesh_.property(fcentroid_, mesh_.face_handle(he));
	}
	if (!mesh_.is_boundary(ohe)) {
		sum += mesh_.property(fcentroid_, mesh_.face_handle(ohe));
	}
	return sum / count;
}

Mesh::Point	SubdivisionViewer::getCatmullClarkOrigVertexNewLocation(Mesh::VertexHandle vh)
{
	
	Mesh::Point		P = mesh_.point(vh);
	Mesh::Point		F = getIncidentFacesAverageMidpoint(vh);
	Mesh::Point		R;
	if (mesh_.is_boundary(vh)) { // we want new point to be on the "boundary plane"
					R = getIncidentBoundaryEdgesAverageMidpoint(vh);
	}
	else {
					R = getIncidentEdgesAverageMidpoint(vh);;
	}
	int				n = getVertexValence(vh);
	if (mesh_.is_boundary(vh)) {
		return ( ((R*2) + (P*(n-2)) ) / (n)  ); // disable face influence on boundary vertex
	}
	return ( (F + (R*2) + (P*(n-3)) ) / (n)  );
}

void SubdivisionViewer::addNewCatmullClarkConnectivity(Mesh::FaceHandle fh)
{
	Mesh::Point						facePoint	=	mesh_.property(fcentroid_, fh);
	Mesh::VertexHandle				faceVertex	=	mesh_.add_vertex(facePoint);
	Mesh::HalfedgeHandle			firstHe		=	mesh_.halfedge_handle(fh);
	Mesh::HalfedgeHandle			he			=	mesh_.halfedge_handle(fh);
	std::vector<QuadFaceCorners>	facesToAdd;
	do {
		Mesh::HalfedgeHandle	prev			=	mesh_.prev_halfedge_handle(he);
		Mesh::VertexHandle		src				=	mesh_.from_vertex_handle(he);
		Mesh::EdgeHandle		eh				=	mesh_.edge_handle(he);
		Mesh::VertexHandle		edgeVertex		=	mesh_.property(enewvertex_, mesh_.edge_handle(he));
		Mesh::VertexHandle		prevEdgeVertex	=	mesh_.property(enewvertex_, mesh_.edge_handle(prev));

		facesToAdd.push_back( QuadFaceCorners(src, edgeVertex, faceVertex, prevEdgeVertex) );
		he = mesh_.next_halfedge_handle(he);
	} while (he != firstHe);
	// Now we can delete the old face
	mesh_.delete_face(fh);
	// Now we can add the new faces
	for (int i = 0; i < facesToAdd.size(); i++) {
		Mesh::FaceHandle nf = mesh_.add_face(facesToAdd[i].v0, facesToAdd[i].v1, facesToAdd[i].v2, facesToAdd[i].v3);
	}
}

// Helpers
Mesh::Point SubdivisionViewer::getFaceCentroid(Mesh::FaceHandle fh)
{
	Mesh::Point sum(0,0,0);
	int count = 0;
	for(Mesh::FaceVertexIter vit = mesh_.fv_begin(fh); vit != mesh_.fv_end(fh); vit++) {
		sum += mesh_.point(vit.handle());
		count++;
	}
	return sum / count;
}

Mesh::Point SubdivisionViewer::getEdgeMidpoint(Mesh::EdgeHandle eh)
{
	int count = 2;
	Mesh::Point sum(0,0,0);
	HalfedgeHandle he	= mesh_.halfedge_handle(eh, 0);
	HalfedgeHandle ohe	= mesh_.opposite_halfedge_handle(he);
	sum += mesh_.point( mesh_.from_vertex_handle(he) );
	sum += mesh_.point( mesh_.to_vertex_handle(he) );
	return sum / count;
}

Mesh::Point	SubdivisionViewer::getIncidentFacesAverageMidpoint(Mesh::VertexHandle vh)
{
	Mesh::Point sum(0,0,0);
	int count = 0;
	for (Mesh::VertexFaceIter fit = mesh_.vf_begin(vh); fit != mesh_.vf_end(vh); fit++) {
		sum += mesh_.property(fcentroid_, fit.handle());
		count++;
	}
	return sum / count;
}

Mesh::Point SubdivisionViewer::getIncidentEdgesAverageMidpoint(Mesh::VertexHandle vh)
{
	Mesh::Point sum(0,0,0);
	int count = 0;
	for (Mesh::VertexEdgeIter eit = mesh_.ve_begin(vh); eit != mesh_.ve_end(vh); eit++) {
		sum += getEdgeMidpoint(eit.handle());
		count++;
	}
	return sum / count;
}

Mesh::Point SubdivisionViewer::getIncidentBoundaryEdgesAverageMidpoint(Mesh::VertexHandle vh)
{
	Mesh::Point sum(0,0,0);
	int count = 0;
	for (Mesh::VertexEdgeIter eit = mesh_.ve_begin(vh); eit != mesh_.ve_end(vh); eit++) {
		if (mesh_.is_boundary(eit.handle())) {
			sum += getEdgeMidpoint(eit.handle());
			count++;
		}
	}
	return sum / count;
}

int SubdivisionViewer::getVertexValence(Mesh::VertexHandle vh)
{
	int count = 0;
	for (Mesh::VertexEdgeIter eit = mesh_.ve_begin(vh); eit != mesh_.ve_end(vh); eit++) {
		count++;
	}
	return count;
}

int SubdivisionViewer::getNumOfFaceVertices(Mesh::FaceHandle fh)
{
	int count = 0;
	for(Mesh::FaceVertexIter vit = mesh_.fv_begin(fh); vit != mesh_.fv_end(fh); vit++) {
		count++;
	}
	return count;
}

// Tesselation
void SubdivisionViewer::triTesselation()
{
	std::vector<FaceHandle>	badFaces;
	for(Mesh::FaceIter fit = mesh_.faces_begin(); fit != mesh_.faces_end(); fit++) {
		if (3 != getNumOfFaceVertices(fit.handle())) {
			badFaces.push_back(fit.handle());
		}
	}
	if (badFaces.size() == 0) {
		return;
	}
	for(int i = 0; i < badFaces.size(); i++) {
		if (mesh_.is_boundary(badFaces[i])) {
			bool ok = false;
			cout << "Hole: ";
		}
		faceTriTesselation(badFaces[i]);
	}
	mesh_.garbage_collection();
	update_face_indices();
	mesh_.update_normals();
}

void SubdivisionViewer::faceTriTesselation(Mesh::FaceHandle fh)
{
	Mesh::Point						facePoint	=	getFaceCentroid(fh);
	Mesh::VertexHandle				faceVertex	=	mesh_.add_vertex(facePoint);
	Mesh::HalfedgeHandle			firstHe		=	mesh_.halfedge_handle(fh);
	Mesh::HalfedgeHandle			he			=	firstHe;
	std::vector<TriFaceCorners>		facesToAdd;
	do {
		Mesh::VertexHandle		v0		=	mesh_.to_vertex_handle(he);
		Mesh::VertexHandle		v2		=	mesh_.from_vertex_handle(he);
		facesToAdd.push_back( TriFaceCorners(v0, faceVertex, v2) );

		he = mesh_.next_halfedge_handle(he);
	} while (he != firstHe);
	// Now we can delete the old face
	mesh_.delete_face(fh);
	// Now we can add the new faces
	for (int i = 0; i < facesToAdd.size(); i++) {
		mesh_.add_face(facesToAdd[i].v0, facesToAdd[i].v1, facesToAdd[i].v2);
	}
	cout << facesToAdd.size() << endl;
}


// Common
bool SubdivisionViewer::open_mesh(const char* _meshfilename)
{
	if (PolyMeshViewer::open_mesh(_meshfilename)) {
		// store vertex initial positions and 3D mesh bounding box
		Mesh::VertexIter v_it=mesh_.vertices_begin(), v_end(mesh_.vertices_end());
		_bbMin3D = _bbMax3D = mesh_.point(v_it);
		for (v_it=mesh_.vertices_begin(); v_it!=v_end; ++v_it) {
			_bbMin3D.minimize(mesh_.point(v_it));
			_bbMax3D.maximize(mesh_.point(v_it));
		}
		return true;
	}
	return false;
}

void SubdivisionViewer::draw(const std::string& _draw_mode)
{
	PolyMeshViewer::draw(_draw_mode);
}

void SubdivisionViewer::keyboard(int key, int x, int y)
{
	switch (toupper(key)) { 
	case 'O':
		{
			OPENFILENAME ofn={0};
			char szFileName[MAX_PATH]={0};
			ofn.lStructSize=sizeof(OPENFILENAME);
			ofn.Flags=OFN_ALLOWMULTISELECT|OFN_EXPLORER;
			ofn.lpstrFilter="All Files (*.*)\0*.*\0";
			ofn.lpstrFile=szFileName;
			ofn.nMaxFile=MAX_PATH;
			if(GetOpenFileName(&ofn))
				open_mesh(szFileName);
		}
		break;
	case 'C':
		Perform_CatmullClark();
		glutPostRedisplay();
		break;
	case 'L':
		Perform_Loop();
		glutPostRedisplay();
		break;
	default:
		PolyMeshViewer::keyboard(key, x, y);
		break;
	}
}
