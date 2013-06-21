#include "SubdivisionViewer.h"

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	SubdivisionViewer window("Subdivision", 512, 512);
	glutMainLoop();
}
