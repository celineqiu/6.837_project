#include "mesh.h"

#include "vertexrecorder.h"

using namespace std;

const float springConstant = 8.0f;
const float springLength = 0.01f;
const float dragConstant = 1.0f;
float groundHeight = 1.0f;

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
            if (v[1] < groundHeight) {
                groundHeight = v[1];
            }
		} else if (s == "f") {
			Tuple3u f;
			in >> f[0] >> f[1] >> f[2];
			faces.push_back(f);
		}
	};

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
    visibleVertices = bindVertices;

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

		Vector3f v0 = visibleVertices[faces[i][0]-1];
		Vector3f v1 = visibleVertices[faces[i][1]-1];
		Vector3f v2 = visibleVertices[faces[i][2]-1];

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

    // create springs
    for (int i=0; i < currentVertices.size(); i++) {
        vector<float> attachment = attachments[i];
        float sum = 0;
        for (int j=0; j < attachment.size(); j++) {
            sum += pow(attachment[j],2);
        }

        particleSprings.push_back(Vector4f(i, i, springConstant/sum, springLength/sum));
        currentVelocities.push_back(Vector3f());
    }

}

std::vector<Vector3f> Mesh::getVelocities() {
    return currentVelocities;
}

std::vector<Vector3f> Mesh::getVisState() {
    return visibleVertices;
};

Vector3f evalSpr(Vector3f pos, Vector3f pos2, float springConstant, float length) {
    Vector3f d = pos-pos2;
    // k(||d|| - r) * unit(d);
    if (springConstant == 0) {
        return Vector3f();
    }

    if (d.abs()-length > 2*length) {
        return -springConstant*(2*length)*d.normalized();
    } else {
        return -springConstant*(d.abs()-length)*d.normalized();
    }
}

Vector3f evalDra(Vector3f vel) {
    Vector3f f = -vel*dragConstant;
    return f;
}

std::vector<Vector3f> Mesh::evalF(std::vector<Vector3f> visState) {

    std::vector<Vector3f> f;

    for (int i =0; i < currentVertices.size(); i++ ) {
        Vector3f acc = evalSpr(visState[i], currentVertices[i], particleSprings[i].z(), particleSprings[i].w());
        acc += evalDra(currentVelocities[i]);

        f.push_back(acc);

    }

    return f;
};

void Mesh::setState(std::vector<Vector3f> state, std::vector<Vector3f> velocities){
    visibleVertices = state;
    currentVelocities = velocities;

    for (int i=0; i < visibleVertices.size(); i++) {
        Vector3f d = (visibleVertices[i]-currentVertices[i]);
        if (d.abs() > 2*springLength) {
            visibleVertices[i] = currentVertices[i] + d.normalized()*2*springLength;
        }

        if (visibleVertices[i].y() < groundHeight) {
            visibleVertices[i] = Vector3f(visibleVertices[i].x(), groundHeight, visibleVertices[i].z());
        }
    }
};