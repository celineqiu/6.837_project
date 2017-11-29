#include "mesh.h"

#include "vertexrecorder.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 4.1. load() should populate bindVertices, currentVertices, and faces
	ifstream in(filename);

	while (in) {
		string s;
		in >> s;

		if (s == "v") {
			Vector3f v;
			in >> v[0] >> v[1] >> v[2];
			bindVertices.push_back(v);
		} else if (s == "f") {
			Tuple3u f;
			in >> f[0] >> f[1] >> f[2];
			faces.push_back(f);
		}
	};

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;

    // initialize 0 velocities?
	currentVelocities.assign(currentVertices.size(), Vector3f(0,0,0));
}

void Mesh::draw()
{

	VertexRecorder rec;
	// 4.2 Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

	for (int i = 0; i < (int) faces.size(); ++i) {

		Vector3f v0 = currentVertices[faces[i][0]-1];
		Vector3f v1 = currentVertices[faces[i][1]-1];
		Vector3f v2 = currentVertices[faces[i][2]-1];

		Vector3f N = Vector3f::cross(v1-v0, v2-v0).normalized();

		rec.record(v0, N);
		rec.record(v1, N);
		rec.record(v2, N);

	}

    rec.draw();


}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 4.3. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
    ifstream in(filename);

    float w;
    while (in >> w) {
        vector<float> vec;
        vec.push_back(w);
        for (int i=0; i < numJoints-2; ++i) {
            in >> w;
            vec.push_back(w);
        }

        attachments.push_back(vec);
    };
    in.close();

}
