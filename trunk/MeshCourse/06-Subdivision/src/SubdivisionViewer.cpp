#include "SubdivisionViewer.h"
#include <windows.h>

static bool _SubdivisioneterizationComputed_u = false, _SubdivisioneterizationComputed_h = false;
static bool _BoundingBox2DComputed = false;

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

	mesh_.remove_property(fcentroid_);
	mesh_.remove_property(enewpoint_);
	mesh_.remove_property(enewvertex_);
	mesh_.update_normals();
	update_face_indices();
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
	mesh_.remove_property(enewpoint_);
	mesh_.remove_property(enewvertex_);
	mesh_.update_normals();
	update_face_indices();
	std::cerr << mesh_.n_vertices() << " vertices, " << mesh_.n_faces() << " faces\n";
}

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
