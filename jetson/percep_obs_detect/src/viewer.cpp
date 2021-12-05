#include "viewer.hpp"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

using namespace std;

// Took the shaders from the ZED code :) 
GLchar* OBJ_VERTEX_SHADER =
"#version 130\n"
//"#version 330 core\n"
"in vec3 in_Vertex;\n"
"in vec3 in_Color;\n"
"uniform mat4 u_mvpMatrix;\n"
"out vec3 b_color;\n"
"void main() {\n"
"   b_color = in_Color;\n"
"	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
"}"; 

GLchar* OBJ_FRAGMENT_SHADER =
"#version 130\n"
//"#version 330 core\n"
"in vec3 b_color;\n"
"out vec4 out_Color;\n"
"void main() {\n"
"   out_Color = vec4(b_color, 1);\n"
"}";

GLchar* PC_VERTEX_SHADER =
"#version 130\n"
//"#version 330 core\n"
"in vec3 in_Vertex;\n"
"in uint in_Color;\n"
"uniform mat4 u_mvpMatrix;\n"
"out vec4 b_color;\n"
"void main() {\n"
"   uint q = in_Color;\n"
//"   uint q = uint(0xFF0000FF);\n"
"   float r = float(q & uint(0x000000FF))/255.0f;\n"
"   float g = float( (q & uint(0x0000FF00)) >> 8 )/255.0f;\n"
"   float b = float( (q & uint(0x00FF0000)) >> 16)/255.0f;\n"
"   b_color = vec4(r, g, b, 1.f);\n"
//"   b_color = vec4(0.0f, 1.0f, 0.0f, 1.f);\n"
"	gl_Position = u_mvpMatrix * vec4(in_Vertex.xyz, 1);\n"
"}";

GLchar* PC_FRAGMENT_SHADER =
"#version 130\n"
//"#version 330 core\n"
"in vec4 b_color;\n"
"out vec4 out_Color;\n"
"void main() {\n"
"   out_Color = b_color;\n"
"}";

/*
 * Shader 
 */

Shader::Shader(GLchar* vs, GLchar* fs) {
    if (!compile(verterxId_, GL_VERTEX_SHADER, vs)) {
        cout << "ERROR: while compiling vertex shader" << endl;
    }
    if (!compile(fragmentId_, GL_FRAGMENT_SHADER, fs)) {
        cout << "ERROR: while compiling vertex shader" << endl;
    }

    programId_ = glCreateProgram();

    glAttachShader(programId_, verterxId_);
    glAttachShader(programId_, fragmentId_);

    glBindAttribLocation(programId_, ATTRIB_VERTICES_POS, "in_Vertex");
    glBindAttribLocation(programId_, ATTRIB_COLOR_POS, "in_Color");

    glLinkProgram(programId_);

    GLint errorlk(0);
    glGetProgramiv(programId_, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        cout << "ERROR: while linking shader" << endl;
        GLint errorSize(0);
        glGetProgramiv(programId_, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(programId_, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programId_);
    }
}

Shader::~Shader() {
    if (verterxId_ != 0)
        glDeleteShader(verterxId_);
    if (fragmentId_ != 0)
        glDeleteShader(fragmentId_);
    if (programId_ != 0)
        glDeleteShader(programId_);
}

GLuint Shader::getProgramId() {
    return programId_;
}

bool Shader::compile(GLuint &shaderId, GLenum type, GLchar* src) {
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        return false;
    }
    glShaderSource(shaderId, 1, (const char**) &src, 0);
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        cout << "ERROR: while compiling shader" << endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}


/* 
 * 3D Object
 */

Object3D::Object3D() {
    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &pointsGPU);
    glGenBuffers(1, &colorsGPU);
    glGenBuffers(1, &indiciesGPU);
}

Object3D::Object3D(std::vector<vec3> &pts, std::vector<vec3> &colors, std::vector<int> &idcs) {
    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &pointsGPU);
    glGenBuffers(1, &colorsGPU);
    glGenBuffers(1, &indiciesGPU);
    update(pts, colors, idcs);
}


void Object3D::draw() {
    glBindVertexArray(vaoID);
    //std::cout << indicies.size() << std::endl;
    if(wireframe) {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glLineWidth(3);
    }
        
    glDrawElements(GL_TRIANGLES, (GLsizei) indicies.size(), GL_UNSIGNED_INT, 0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBindVertexArray(0);
}

void Object3D::update(std::vector<vec3> &pts, std::vector<vec3> &colors, std::vector<int> &idcs) {
    // Update internal CPU representations 
    // We might not actually even need this 
    points = pts;
    colors = colors;
    indicies = idcs;
    
    //Provide default initialization with: ??
    //std::iota(indicies.begin(), indicies.end(), 0);

    // Update GPU data for rendering 
    glBindVertexArray(vaoID);
    // Points
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glBufferData(GL_ARRAY_BUFFER, points.size()*sizeof(vec3), &points[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    // Colors
    glBindBuffer(GL_ARRAY_BUFFER, colorsGPU);
    glBufferData(GL_ARRAY_BUFFER, colors.size()*sizeof(vec3), &colors[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);
    // Indicies
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indiciesGPU);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicies.size()*sizeof(unsigned int), &indicies[0], GL_DYNAMIC_DRAW);
    // Unbind 
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    //std::cout << vaoID << std::endl;
}

Object3D::~Object3D() {
    //glDeleteVertexArrays(1, &vaoID);
    glDeleteBuffers(1, &pointsGPU);
    glDeleteBuffers(1, &colorsGPU);
    glDeleteBuffers(1, &indiciesGPU);
}

/*
 * Point Cloud
 */

PointCloud::PointCloud() {
    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &pointsGPU);
}

PointCloud::~PointCloud() {
    glDeleteVertexArrays(1, &vaoID);
    glDeleteBuffers(1, &pointsGPU);
}

void PointCloud::update(std::vector<vec4> &pts) {
    update(&pts[0], pts.size());
}

void PointCloud::update(vec4* pts, int size) {
    this->size = size;

    // Update GPU data for rendering 
    glBindVertexArray(vaoID);
    // Points
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glBufferData(GL_ARRAY_BUFFER, size*sizeof(vec4), pts, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec4), 0);
    glEnableVertexAttribArray(0);
    // Color
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(vec4), (void*)offsetof(vec4, w));
    glEnableVertexAttribArray(1);
    // Unbind 
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

}

void PointCloud::draw() {
    glBindVertexArray(vaoID);
    glDrawArrays(GL_POINTS, 0, size);
    glBindVertexArray(0);
}

/* 
 * Camera
 */


/* 
 * Viewer
 */

Viewer* curInstance; 

Viewer::Viewer() : camera(glm::perspective(glm::radians(35.0f), 1920.0f/1080.0f, 0.01f, 100000.0f)) {
}

void Viewer::init(int argc, char **argv) {
    // Window stuff
    glutInit(&argc, argv);
    int wnd_w = 1920*0.7;//glutGet(GLUT_SCREEN_WIDTH);
    int wnd_h = 1080*0.7;//glutGet(GLUT_SCREEN_HEIGHT);
    glutInitWindowSize(1920*0.7, 1080*0.7);
    glutInitWindowPosition(wnd_w*0.05, wnd_h*0.05);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("Display");

    // Glew (loads opengl api backend)
    GLenum err = glewInit();
    if (GLEW_OK != err)
        cout << "Error w/ viewer";

    // Options
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    //glEnable(GL_DEPTH_TEST | GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    // Callbacks
    cout << "creating callbacks" << endl;
    glutMouseFunc(Viewer::mouseButtonCallback);
    glutMotionFunc(Viewer::mouseMotionCallback);
    glutKeyboardFunc(Viewer::keyPressedCallback);
    glutKeyboardUpFunc(Viewer::keyReleasedCallback);
    cout << "callbacks created" << endl;
    curInstance = this;

    // Shader
    objectShader = Shader(OBJ_VERTEX_SHADER, OBJ_FRAGMENT_SHADER);
    pcShader = Shader(PC_VERTEX_SHADER, PC_FRAGMENT_SHADER);

    curInstance->frame = 0;
    curInstance->framePlay = true;
}

// Viewer tick
void Viewer::update() {
    // Basic drawing setup 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.7, 0.7, 0.7, 1.f);
    //glLineWidth(2.f);
    glPointSize(6.f); // 1, 3
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Draw 3D Objects
    glUseProgram(objectShader.getProgramId());
    for(auto &object : objects) {
        glm::mat4 mvp_mat = camera.projection * camera.view;
        glUniformMatrix4fv(glGetUniformLocation(objectShader.getProgramId(), "u_mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp_mat));
        object.draw();
    }
    viewer_mutex.lock();
    for(auto &object : ephemeralObjects) {
        glm::mat4 mvp_mat = camera.projection * camera.view;
        glUniformMatrix4fv(glGetUniformLocation(objectShader.getProgramId(), "u_mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp_mat));
        object.draw();
    }
    viewer_mutex.unlock();

    glUseProgram(pcShader.getProgramId());
    pc_mutex.lock();
    for (auto &pc : pointClouds) {
        glm::mat4 mvp_mat = camera.projection * camera.view;
        glUniformMatrix4fv(glGetUniformLocation(objectShader.getProgramId(), "u_mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp_mat));
        pc.draw();
    }

    //std::cout << pointClouds.size() << std::endl; 
    //std::cout << ephemeralObjects.size() << std::endl;
    pc_mutex.unlock();

    // Update display
    glutMainLoopEvent();
    glutSwapBuffers();
    glutPostRedisplay();
}

void Viewer::addObject(Object3D &obj, bool ephemeral) {
    viewer_mutex.lock();
    if(ephemeral) ephemeralObjects.push_back(obj);
    else objects.push_back(obj);
    viewer_mutex.unlock();
}

void Viewer::clearEphemerals() {
    viewer_mutex.lock();
    ephemeralObjects.clear();
    viewer_mutex.unlock();
}

void Viewer::updatePointCloud(int idx, vec4* pts, int size) {
    pc_mutex.lock();
    // Calculate
    float maxX = numeric_limits<float>::min(), maxZ = numeric_limits<float>::min();
    float minX = numeric_limits<float>::max(), minZ = numeric_limits<float>::max();
    for(int i = 0; i < size; ++i) {
        maxX = max(maxX, pts[i].x);
        maxZ = max(maxZ, pts[i].z);
        minX = min(minX, pts[i].x);
        minZ = min(minZ, pts[i].z);
    }
    pcCenter = glm::vec3((minX + maxX) / 2, 0.0f, (minZ + maxZ) / 2);
    pointClouds[idx].update(pts, size);
    pc_mutex.unlock();
}

#ifndef VIEWER_ONLY
void Viewer::updatePointCloud(GPU_Cloud pc) {
    glm::vec4* pc_cpu = new glm::vec4[pc.size];
    cudaMemcpy(pc_cpu, pc.data, sizeof(float4)*pc.size, cudaMemcpyDeviceToHost);
    updatePointCloud(0, pc_cpu, pc.size);
    delete[] pc_cpu;
}
#endif

void Viewer::addPointCloud() {
    pc_mutex.lock();
    pointClouds.push_back(PointCloud());
    pc_mutex.unlock();
}

void Viewer::setTarget() {
    curInstance->camera.setTarget(pcCenter);
}

void Viewer::setTarget(vec3 target) {
    curInstance->camera.setTarget(target);
}

void Viewer::mouseButtonCallback(int button, int state, int x, int y) {
    // We need to reset the previous mouse X on the click to avoid a jump in look dir
    curInstance->prevMouseX = x;
    curInstance->prevMouseY = y;


    // wheel up
    if(button == 3) {
        // curInstance->camera.setEye(lookDir * 0.75f + curInstance->camera.getTarget());
        curInstance->camera.scaleEye(-0.15f);
    }
        // wheel down
    else if(button == 4) {
        //curInstance->camera.setEye(lookDir * 1.25f + curInstance->camera.getTarget());
        curInstance->camera.scaleEye(0.15f);

    }

    glutPostRedisplay();

}

void Viewer::mouseMotionCallback(int x, int y) {

    int deltaX = x - curInstance->prevMouseX;
    int deltaY = y - curInstance->prevMouseY;

    curInstance->camera.rotateX(deltaY);
    curInstance->camera.rotateY(deltaX);
    
    curInstance->prevMouseX = x;
    curInstance->prevMouseY = y;

}

// Responds to key presses, TODO: more features here :)
void Viewer::keyPressedCallback(unsigned char c, int x, int y) {
    cout << "key press" << endl;

    //if(c == " ") framePlay = 
}

void Viewer::keyReleasedCallback(unsigned char c, int x, int y) {
    if(c == ' ') {
        curInstance->framePlay = !curInstance->framePlay;
    } if(c =='d') {
        curInstance->frame++;
    } 
    if(c =='a') {
        curInstance->frame--;
    }
}   


#ifdef VIEWER_ONLY
int main(int argc, char** argv) {
    Viewer viewer;
    viewer.init(argc, argv);
    PCDReader reader;
    std::string dir = ROOT_DIR;
    dir += "/data/";
    std::cout << dir << std::endl;
    reader.open(dir);
    std::vector<vec4> cloud = reader.readCloudCPU(dir+"pcl50.pcd");
    viewer.addPointCloud();
    viewer.updatePointCloud(0, &cloud[0], cloud.size());
    viewer.setTarget();
    while(true) {
        viewer.update();
    }
    return 0;
}
#endif
