#include "viewer.hpp"
#include <iostream>
#include <exception>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/rotate_vector.hpp>

using namespace std;

// Took the shaders from the ZED code :) 
GLchar const* OBJ_VERTEX_SHADER =
        "#version 330 core\n"
        "in vec3 in_Vertex;\n"
        "in vec3 in_Color;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec3 b_color;\n"
        "void main() {\n"
        "   b_color = in_Color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "}";

GLchar const* OBJ_FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec3 b_color;\n"
        "out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = vec4(b_color, 1);\n"
        "}";

GLchar const* PC_VERTEX_SHADER =
        "#version 330 core\n"
        "in vec4 in_Vertex;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec4 b_color;\n"
        "void main() {\n"
        "   uint q = floatBitsToUint(in_Vertex.w);\n"
        "   float r = float(q & uint(0x000000FF))/255.0f;\n"
        "   float g = float( (q & uint(0x0000FF00)) >> 8 )/255.0f;\n"
        "   float b = float( (q & uint(0x00FF0000)) >> 16)/255.0f;\n"
        "   b_color = vec4(r, g, b, 1.f);\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex.xyz, 1);\n"
        "}";

GLchar const* PC_FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec4 b_color;\n"
        "out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = b_color;\n"
        "}";

/*
 * Shader 
 */

Shader::Shader(GLchar const* vs, GLchar const* fs) {
    if (!compile(vertexShaderId, GL_VERTEX_SHADER, vs)) {
        cout << "ERROR: while compiling vertex shader" << endl;
    }
    if (!compile(fragmentShaderId, GL_FRAGMENT_SHADER, fs)) {
        cout << "ERROR: while compiling vertex shader" << endl;
    }

    programShaderId = glCreateProgram();

    glAttachShader(programShaderId, vertexShaderId);
    glAttachShader(programShaderId, fragmentShaderId);

    glBindAttribLocation(programShaderId, ATTRIB_VERTICES_POS, "in_Vertex");
    glBindAttribLocation(programShaderId, ATTRIB_COLOR_POS, "in_Color");

    glLinkProgram(programShaderId);

    GLint errorlk(0);
    glGetProgramiv(programShaderId, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        cout << "ERROR: while linking shader" << endl;
        GLint errorSize(0);
        glGetProgramiv(programShaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char* error = new char[errorSize + 1];
        glGetShaderInfoLog(programShaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programShaderId);
        throw runtime_error("Shader error");
    }
}

Shader::~Shader() {
    if (vertexShaderId != 0)
        glDeleteShader(vertexShaderId);
    if (fragmentShaderId != 0)
        glDeleteShader(fragmentShaderId);
    if (programShaderId != 0)
        glDeleteShader(programShaderId);
}

GLuint Shader::getProgramId() {
    return programShaderId;
}

bool Shader::compile(GLuint& shaderId, GLenum type, GLchar const* src) {
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        return false;
    }
    glShaderSource(shaderId, 1, (const char**) &src, nullptr);
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        cout << "ERROR: while compiling shader" << endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char* error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}

Shader& Shader::operator=(Shader other) {
    swap(other);
    return *this;
}

Shader::Shader(Shader&& other) noexcept: Shader() { swap(other); }

void Shader::swap(Shader& other) {
    std::swap(vertexShaderId, other.vertexShaderId);
    std::swap(fragmentShaderId, other.fragmentShaderId);
    std::swap(programShaderId, other.programShaderId);
}


/* 
 * 3D Object
 */

Object3D::Object3D() {
    glGenVertexArrays(1, &vaoID);
    glGenBuffers(1, &pointsGPU);
    glGenBuffers(1, &colorsGPU);
    glGenBuffers(1, &indicesGPU);
}

Object3D::Object3D(std::vector<vec3>& points, std::vector<vec3>& colors, std::vector<int>& indices) : Object3D() {
    update(points, colors, indices);
}


void Object3D::draw() const {
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(3);
    }
    glBindVertexArray(vaoID);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, colorsGPU);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesGPU);
    glDrawElements(GL_TRIANGLES, size, GL_UNSIGNED_INT, nullptr);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
}

void Object3D::update(std::vector<vec3>& points, std::vector<vec3>& colors, std::vector<int>& indices) {
    size = indices.size();
    // Update GPU data for rendering
    glBindVertexArray(vaoID);
    // Points
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(vec3), points.data(), GL_DYNAMIC_DRAW);
    // Colors
    glBindBuffer(GL_ARRAY_BUFFER, colorsGPU);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(vec3), colors.data(), GL_DYNAMIC_DRAW);
    // Indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesGPU);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_DYNAMIC_DRAW);
    // Unbind 
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Object3D::~Object3D() {
    glDeleteVertexArrays(1, &vaoID);
    glDeleteBuffers(1, &pointsGPU);
    glDeleteBuffers(1, &colorsGPU);
    glDeleteBuffers(1, &indicesGPU);
}

Object3D::Object3D(Object3D&& other) noexcept {
    swap(other);
}

Object3D& Object3D::operator=(Object3D other) {
    swap(other);
    return *this;
}

void Object3D::swap(Object3D& other) {
    std::swap(size, other.size);
    std::swap(vaoID, other.vaoID);
    std::swap(pointsGPU, other.pointsGPU);
    std::swap(colorsGPU, other.colorsGPU);
    std::swap(indicesGPU, other.indicesGPU);
}


/*
 * Point Cloud
 */

PointCloud::PointCloud() {
    glGenVertexArrays(1, &vaoId);
    glGenBuffers(1, &pointsGPU);
}

PointCloud::~PointCloud() {
    glDeleteVertexArrays(1, &vaoId);
    glDeleteBuffers(1, &pointsGPU);
}

void PointCloud::update(std::vector<vec4>& pts) {
    update(pts.data(), pts.size());
}

void PointCloud::update(vec4* pts, int size) {
    this->size = size;
    // Update GPU data for rendering
    glBindVertexArray(vaoId);
    // Points
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glBufferData(GL_ARRAY_BUFFER, size * sizeof(vec4), pts, GL_DYNAMIC_DRAW);
}

void PointCloud::draw() {
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, pointsGPU);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);
    glDrawArrays(GL_POINTS, 0, size);
    glDisableVertexAttribArray(0);
}

PointCloud::PointCloud(PointCloud&& other) noexcept {
    swap(other);
}

PointCloud& PointCloud::operator=(PointCloud other) {
    swap(other);
    return *this;
}

void PointCloud::swap(PointCloud& other) {
    std::swap(size, other.size);
    std::swap(vaoId, other.vaoId);
    std::swap(pointsGPU, other.pointsGPU);
}

/*
 * Camera
 */


/* 
 * Viewer
 */

Viewer::Viewer()
        : camera(glm::perspectiveFov(glm::radians(35.0f), 1920.0f, 1080.0f, 0.1f, 100000.0f)) {
    if (!glfwInit()) {
        throw runtime_error("GLFW init failed");
    }
    glfwSetErrorCallback([](int error_code, const char* description) {
        cout << "GLFW error: " << error_code << ", " << description << endl;
    });
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWvidmode const* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    window = glfwCreateWindow(static_cast<int>(mode->width * 0.7), static_cast<int>(mode->height * 0.7), "Viewer", nullptr, nullptr);
    if (!window) {
        throw runtime_error("Window init failed");
    }
    glfwMakeContextCurrent(window);
    if (glewInit()) {
        throw runtime_error("Failed to init glew");
    }

    glfwSwapInterval(1); // V-SYNC to avoid excessive framerate

    // Options
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Shader
    objectShader = Shader(OBJ_VERTEX_SHADER, OBJ_FRAGMENT_SHADER);
    pcShader = Shader(PC_VERTEX_SHADER, PC_FRAGMENT_SHADER);

    glfwSetWindowUserPointer(window, this);

    int major, minor, rev;
    std::cout << glfwGetVersionString() << std::endl;
//    if (glfwRawMouseMotionSupported())
//        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    ImGui::StyleColorsDark();
}

Viewer::~Viewer() {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Viewer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    auto viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    viewer->camera.zoom(static_cast<float>(yoffset * -500.0));
}

void Viewer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_LEFT_BRACKET: {
                viewer->frame = std::max(0, viewer->frame - 1);
                break;
            }
            case GLFW_KEY_RIGHT_BRACKET: {
                viewer->frame = std::min(viewer->maxFrame - 1, viewer->frame + 1);
                break;
            }
            case GLFW_KEY_SPACE: {
                viewer->framePlay = !viewer->framePlay;
                break;
            }
            case GLFW_KEY_1: {
                viewer->procStage = ProcStage::RAW;
                break;
            }
            case GLFW_KEY_2: {
                viewer->procStage = ProcStage::POSTPASS;
                break;
            }
            case GLFW_KEY_3: {
                viewer->procStage = ProcStage::POSTRANSAC;
                break;
            }
            case GLFW_KEY_4: {
                viewer->procStage = ProcStage::POSTECE;
                break;
            }
            case GLFW_KEY_5: {
                viewer->procStage = ProcStage::POSTBOUNDING;
                break;
            }
            case GLFW_KEY_6: {
                viewer->procStage = ProcStage::POSTBEARING;
                break;
            }
            case GLFW_KEY_ESCAPE: {
                viewer->inMenu = !viewer->inMenu;
                break;
            }
            case GLFW_KEY_R: {
                viewer->record = !viewer->record;
                break;
            }
        }
    }
}

void Viewer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    auto viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if (viewer->inMenu) {
        viewer->prevFocused = false;
        return;
    }
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool focused = glfwGetWindowAttrib(window, GLFW_FOCUSED);
    if (focused == viewer->prevFocused) { // check to avoid look jumping when tabbing in and out
        auto deltaX = static_cast<float>((xpos - width / 2) * viewer->mouseSensitivity * 0.01);
        auto deltaY = static_cast<float>((ypos - height / 2) * viewer->mouseSensitivity * 0.01);
//        std::cout << "dx: " << xpos << "\n";
//        std::cout << "dy: " << ypos << "\n";
        viewer->camera.rotateX(-deltaY);
        viewer->camera.rotateY(-deltaX);
        glfwSetCursorPos(window, width / 2, height / 2);
    }
    viewer->prevFocused = focused;
}

// Viewer tick
void Viewer::update() {
    // Basic drawing setup
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (record) glClearColor(0.25f, 0.2f, 0.2f, 0.0f);
    else glClearColor(0.2f, 0.2f, 0.2f, 0.0f);
    glPointSize(6.0f); // 1, 3
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Draw 3D Objects
    glm::mat4 mvp_mat = camera.projection * camera.getView();
    glUseProgram(objectShader.getProgramId());
    glUniformMatrix4fv(glGetUniformLocation(objectShader.getProgramId(), "u_mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp_mat));
    for (auto& object: objects) {
        object.draw();
    }
    viewer_mutex.lock();
    for (auto& object: ephemeralObjects) {
        object.draw();
    }
    viewer_mutex.unlock();

    glUseProgram(pcShader.getProgramId());
    glUniformMatrix4fv(glGetUniformLocation(pcShader.getProgramId(), "u_mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp_mat));
    pc_mutex.lock();
    for (auto& pc: pointClouds) {
        pc.draw();
    }
    pc_mutex.unlock();

    constexpr float speed = 300.0f;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camera.move({speed, 0.0f, 0.0f});
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camera.move({-speed, 0.0f, 0.0f});
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camera.move({0.0f, 0.0f, speed});
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camera.move({0.0f, 0.0f, -speed});
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) camera.rotateX(mouseSensitivity * 0.05f);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) camera.rotateX(-mouseSensitivity * 0.05f);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) camera.rotateY(-mouseSensitivity * 0.05f);
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) camera.rotateY(mouseSensitivity * 0.05f);

    glfwSetInputMode(window, GLFW_CURSOR, inMenu ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_HIDDEN);
    drawUI();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void Viewer::drawUI() {
    // Create a window called "My First Tool", with a menu bar.
    ImGui::Begin("Layers");
    if (ImGui::RadioButton("Raw", procStage == ProcStage::RAW)) procStage = ProcStage::RAW;
    if (ImGui::RadioButton("Pass", procStage == ProcStage::POSTPASS)) procStage = ProcStage::POSTPASS;
    if (ImGui::RadioButton("RANSAC", procStage == ProcStage::POSTRANSAC)) procStage = ProcStage::POSTRANSAC;
    if (ImGui::RadioButton("ECE", procStage == ProcStage::POSTECE)) procStage = ProcStage::POSTECE;
    if (ImGui::RadioButton("Bounding", procStage == ProcStage::POSTBOUNDING)) procStage = ProcStage::POSTBOUNDING;
    if (ImGui::RadioButton("Bearing", procStage == ProcStage::POSTBEARING)) procStage = ProcStage::POSTBEARING;
    ImGui::End();

#ifndef VIEWER_ONLY
    ImGui::Begin("Parameters");
    ImGui::SliderFloat("Epsilon", &epsilon, 0.0f, 20.0f);
    ImGui::SliderInt("Iterations", &iterations, 1, 2000);
    ImGui::SliderFloat("Threshold", &threshold, 0.0f, 500.0f);
    ImGui::SliderFloat("Removal Radius", &removalRadius, 0.0f, 500.0f);
    ImGui::Checkbox("Remove Ground", &removeGround);
    ImGui::Separator();
    ImGui::SliderFloat("Tolerance", &tolerance, 0.0f, 500.0f);
    ImGui::SliderFloat("Minimum Size", &minSize, 0.0f, 1000.0f);
    ImGui::End();
#endif

    ImGui::Begin("Playback");
    ImGui::SliderInt("Frame", &frame, 0, maxFrame);
    ImGui::Checkbox("Record", &record);
    ImGui::End();

    ImGui::Begin("Controls");
    ImGui::Text("Escape: Open UI\n"
                "[/]: Next/previous frame\n"
                "Space: Pause/resume playback\n"
                "R: Record (requires ZED DataSource)\n"
                "WASD: First-person move\n"
                "Arrow keys: Camera move\n"
                "1-6: Choose layers");
    ImGui::SliderFloat("Mouse", &mouseSensitivity, 0.1f, 10.0f);
    ImGui::End();
}

void Viewer::addObject(Object3D&& obj, bool ephemeral) {
    viewer_mutex.lock();
    if (ephemeral) ephemeralObjects.push_back(std::move(obj));
    else objects.push_back(std::move(obj));
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
    for (int i = 0; i < size; ++i) {
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
    auto* pc_cpu = new glm::vec4[pc.size];
    cudaMemcpy(pc_cpu, pc.data, sizeof(float4) * pc.size, cudaMemcpyDeviceToHost);
    updatePointCloud(0, pc_cpu, pc.size);
    delete[] pc_cpu;
}

#endif

void Viewer::addPointCloud() {
    pc_mutex.lock();
    pointClouds.emplace_back();
    pc_mutex.unlock();
}

void Viewer::setCenter() {
    setCenter(pcCenter);
}

void Viewer::setCenter(vec3 center) {
    camera.setCenter(center);
}

bool Viewer::open() {
    return !glfwWindowShouldClose(window);
}

#ifdef VIEWER_ONLY

int main(int argc, char** argv) {
    Viewer viewer;
    PCDReader reader;
    std::string dir = ROOT_DIR;
    dir += "/data/";
    std::cout << dir << std::endl;
    reader.open(dir);
    std::vector<vec4> cloud = reader.readCloudCPU(dir + "pcl50.pcd");
    viewer.addPointCloud();
    viewer.updatePointCloud(0, cloud.data(), cloud.size());
    viewer.setCenter();
    while (viewer.open()) {
        viewer.update();
    }
    return 0;
}

#endif
