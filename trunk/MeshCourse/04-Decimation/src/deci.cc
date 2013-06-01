//== INCLUDES =================================================================

#include "QuadricT.hh"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Types/TriMesh_ArrayKernelT.hh>



//== IMPLEMENTATION ===========================================================





//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------


int main(int argc, char **argv)
{
	if (argc < 4) 
	{
		std::cerr << "Usage: \n" 
			<< argv[0] << " percentage  in.off  out.off\n\n";
		exit(1);
	}


	


	// read mesh
	OpenMesh::IO::read_mesh(mesh, argv[2]);
	std::cout << "#vertices: " << mesh.n_vertices() << std::endl;

	// compute normals & quadrics
	init();

	// decimate
	decimate((int)(atof(argv[1])*mesh.n_vertices()));
	std::cout << "#vertices: " << mesh.n_vertices() << std::endl;

	// write mesh
	OpenMesh::IO::write_mesh(mesh, argv[3]);
}


//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
