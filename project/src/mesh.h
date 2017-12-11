#ifndef MESH_H
#define MESH_H

#include <vector>
#include <vecmath.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include "tuple.h"

typedef tuple< unsigned, 3 > Tuple3u;

struct Mesh
{
	// list of vertices from the OBJ file
	// in the "bind pose"
	std::vector< Vector3f > bindVertices;

	// each face has 3 indices
	// referencing 3 vertices
	std::vector< Tuple3u > faces;

	// current vertex positions after animation
	std::vector< Vector3f > currentVertices;

    // current visible vertex positions after animation
    std::vector< Vector3f > visibleVertices;

    // current visible vertex positions after animation
    std::vector< Vector3f > larrysVertices;


	// current vertex velocities after animation
	std::vector< Vector3f > currentVelocities;

    // springs between visible vertex positions, and vertex positions
    std::vector<Vector4f> particleSprings;

	// list of vertex to joint attachments
	// each element of attachments is a vector< float > containing
	// one attachment weight per joint
	std::vector< std::vector< float > > attachments;

	// 2.1.1. load() should populate bindVertices, currentVertices, and faces
	void load(const char *filename);

	// 2.1.2. draw the current mesh.
	void draw();
    std::vector<Vector3f> getVelocities();

    std::vector<Vector3f> getVisState();

    std::vector<Vector3f> evalF(std::vector<Vector3f> visState);

    void setState(std::vector<Vector3f> visState, std::vector<Vector3f> velocities);

	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	void loadAttachments( const char* filename, int numJoints );
};

#endif
