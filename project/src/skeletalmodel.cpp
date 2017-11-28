#include "skeletalmodel.h"
#include <cassert>

#include "starter_util.h"
#include "vertexrecorder.h"
#include <math.h>

using namespace std;

SkeletalModel::SkeletalModel() {
    program = compileProgram(c_vertexshader, c_fragmentshader_light);
    if (!program) {
        printf("Cannot compile program\n");
        assert(false);
    }
}

SkeletalModel::~SkeletalModel() {
    // destructor will release memory when SkeletalModel is deleted
    while (m_joints.size()) {
        delete m_joints.back();
        m_joints.pop_back();
    }

    glDeleteProgram(program);
}

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
    loadSkeleton(skeletonFile);

    m_mesh.load(meshFile);
    m_mesh.loadAttachments(attachmentsFile, (int)m_joints.size());

    computeBindWorldToJointTransforms();
    updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(const Camera& camera, bool skeletonVisible)
{
    // draw() gets called whenever a redraw is required
    // (after an update() occurs, when the camera moves, the window is resized, etc)

    m_matrixStack.clear();

    glUseProgram(program);
    updateShadingUniforms();
    if (skeletonVisible)
    {
        drawJoints(camera);
        drawSkeleton(camera);
    }
    else
    {
        // Tell the mesh to draw itself.
        // Since we transform mesh vertices on the CPU,
        // There is no need to set a Model matrix as uniform
        camera.SetUniforms(program, Matrix4f::identity());
        m_mesh.draw();
    }
    glUseProgram(0);
}

void SkeletalModel::updateShadingUniforms() {
    // UPDATE MATERIAL UNIFORMS
    GLfloat diffColor[] = { 0.4f, 0.4f, 0.4f, 1 };
    GLfloat specColor[] = { 0.9f, 0.9f, 0.9f, 1 };
    GLfloat shininess[] = { 50.0f };
    int loc = glGetUniformLocation(program, "diffColor");
    glUniform4fv(loc, 1, diffColor);
    loc = glGetUniformLocation(program, "specColor");
    glUniform4fv(loc, 1, specColor);
    loc = glGetUniformLocation(program, "shininess");
    glUniform1f(loc, shininess[0]);

    // UPDATE LIGHT UNIFORMS
    GLfloat lightPos[] = { 3.0f, 3.0f, 5.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightPos");
    glUniform4fv(loc, 1, lightPos);

    GLfloat lightDiff[] = { 120.0f, 120.0f, 120.0f, 1.0f };
    loc = glGetUniformLocation(program, "lightDiff");
    glUniform4fv(loc, 1, lightDiff);
}

void SkeletalModel::loadSkeleton(const char* filename)
{
    cout << filename << endl;
    // Load the skeleton from file here.
    ifstream in(filename);

    float x;
    float y;
    float z;

    while (in >> x) {

        in >> y >> z;
        Matrix4f translation = Matrix4f::translation(x, y, z);

        Joint* joint = new Joint;
        joint->transform = translation;

        //add joint to list
        m_joints.push_back(joint);

        // find children
        int joint_index;
        in >> joint_index;
        if (joint_index == -1) {
            //root
            m_rootJoint = joint;
        } else {
            m_joints[joint_index]->children.push_back(joint);
        }
    }
    in.close();
}


void SkeletalModel::drawJoint(Joint* root, const Camera& camera) {
    // add to the stack (does the transformation);
    m_matrixStack.push(root->transform);

    Matrix4f M = m_matrixStack.top();

    // update transformation uniforms
    camera.SetUniforms(program, M);
    // draw
    drawSphere(0.025f, 12, 12);

    for (int i=0; i < root->children.size(); ++i) {
        drawJoint(root->children[i], camera);
    }
    m_matrixStack.pop();
}

void SkeletalModel::drawJoints(const Camera& camera)
{
    // Draw a sphere at each joint. You will need to add a recursive
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawSphere( 0.025f, 12, 12 )
    // to draw a sphere of reasonable size.
    //
    // You should use your MatrixStack class. A function
    // should push it's changes onto the stack, and
    // use stack.pop() to revert the stack to the original
    // state.

    // this is just for illustration:

    // translate from top of stack, but doesn't push, since that's not
    // implemented yet.

    drawJoint(m_rootJoint, camera);

    // didn't push to stack, so no pop() needed
}

void SkeletalModel::drawBones(Joint* root, const Camera& camera) {
    // add to the stack (does the transformation);

    m_matrixStack.push(root->transform);
    Matrix4f M = m_matrixStack.top();

    for (int i=0; i < root->children.size(); ++i) {

        //bisector between y and translation
        Vector3f xyz = (root->children[i]->transform.getCol(3).xyz().normalized() + Vector3f(0, 1, 0))/2;


        //then rotate 180 around this bisector
        Matrix4f R = Matrix4f::rotation(xyz, 3.14159);
        camera.SetUniforms(program, M*R);

        // draw bone
        float length = (root->children[i]->transform.getCol(3)).xyz().abs();
        drawCylinder(6, 0.02f, length);

        //draw rest of joints
        drawBones(root->children[i], camera);
    }

    m_matrixStack.pop();


}


void SkeletalModel::drawSkeleton(const Camera& camera)
{
    // Draw cylinders between the joints. You will need to add a recursive 
    // helper function to traverse the joint hierarchy.
    //
    // We recommend using drawCylinder(6, 0.02f, <height>);
    // to draw a cylinder of reasonable diameter.

    drawBones(m_rootJoint, camera);

}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
    // Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
    Matrix4f R = Matrix4f::rotateX(rX)*Matrix4f::rotateY(rY)*Matrix4f::rotateZ(rZ);
    R.setCol(3, m_joints[jointIndex]->transform.getCol(3));
    m_joints[jointIndex]->transform = R;
}

void SkeletalModel::computeBindWorldToJointTransforms()
{
    // 2.3.1. Implement this method to compute a per-joint transform from
    // world-space to joint space in the BIND POSE.
    //
    // Note that this needs to be computed only once since there is only
    // a single bind pose.
    //
    // This method should update each joint's bindWorldToJointTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.

    m_rootJoint->bindWorldToJointTransform = m_rootJoint->transform;
    computeBW2J(m_rootJoint);
}

void SkeletalModel::computeBW2J(Joint* root) {
    for (int i=0; i < root->children.size(); ++i) {
        root->children[i]->bindWorldToJointTransform = root->bindWorldToJointTransform*root->children[i]->transform;

        //compute rest of joints
        computeBW2J(root->children[i]);
    }
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
    // 2.3.2. Implement this method to compute a per-joint transform from
    // joint space to world space in the CURRENT POSE.
    //
    // The current pose is defined by the rotations you've applied to the
    // joints and hence needs to be *updated* every time the joint angles change.
    //
    // This method should update each joint's currentJointToWorldTransform.
    // You will need to add a recursive helper function to traverse the joint hierarchy.

    m_rootJoint->currentJointToWorldTransform = m_rootJoint->transform;
    updateCJ2W(m_rootJoint);
}

void SkeletalModel::updateCJ2W(Joint* root) {
    for (int i=0; i < root->children.size(); ++i) {
        root->children[i]->currentJointToWorldTransform = root->currentJointToWorldTransform
                                                          *root->children[i]->transform;
        updateCJ2W(root->children[i]);
    }
}




void SkeletalModel::updateMesh()
{
    // 2.3.2. This is the core of SSD.
    // Implement this method to update the vertices of the mesh
    // given the current state of the skeleton.
    // You will need both the bind pose world --> joint transforms.
    // and the current joint --> world transforms.

    for (int i=0; i < (int) m_mesh.currentVertices.size(); ++i) {
        Vector3f xyz = m_mesh.bindVertices[i];
        Vector3f sum;
        for (int j=0; j < (int) m_joints.size()-1; ++j) {
            float w = (m_mesh.attachments[i])[j];
            sum += w*(m_joints[j+1]->currentJointToWorldTransform
                                             *m_joints[j+1]->bindWorldToJointTransform.inverse()
                                             *Vector4f(xyz, 1)).xyz();
        }

        m_mesh.currentVertices[i] = sum;
    }

}

