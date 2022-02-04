#include <vector>
#include <mutex>
#include <utility>
#include <numeric>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/mat4x4.hpp>
#include "reader.h"

#ifndef VIEWER_ONLY

#include "common.hpp"

#endif

// change this to cuda floatX types
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/rotate_vector.hpp>

typedef glm::vec3 vec3;
typedef glm::vec4 vec4;

//class Drawable {}
//class ClearPath

// Point Cloud Graphics Object
class PointCloud {
    public:
        PointCloud();

        ~PointCloud();

        PointCloud(PointCloud&& other) noexcept;

        PointCloud& operator=(PointCloud other);

        void swap(PointCloud& other);

        void update(std::vector<vec4>&& pts);

        void draw();

        // control p/r/y with mouse??
//    void setRotation(float pitch, float roll, float yaw);

    private:
        GLsizei size{};
        GLuint vaoId{};
        GLuint pointsGPU{};
};

// 3D Object
class Object3D {
    public:
        Object3D();

        Object3D(std::vector<vec3>& points, std::vector<vec3>& colors, std::vector<int>& indices);

        ~Object3D();

        Object3D(Object3D&& other) noexcept;

        Object3D& operator=(Object3D other);

        void swap(Object3D& other);

        // Change the underlying model
        void update(std::vector<vec3>& points, std::vector<vec3>& colors, std::vector<int>& indices);

        void draw() const;

        // Allow rotation and translation of the underlying model
//    void setTranslation(float x, float y, float z);
//    void setRotation(float pitch, float roll, float yaw);
//    glm::mat4 getModelMat();

        bool wireframe = true;

    private:
        // State variables
//    glm::mat4 translation;
//    glm::mat4 rotation;

        // GPU representation
        // https://gamedev.stackexchange.com/questions/8042/whats-the-purpose-of-opengls-vertex-array-objects

        GLsizei size{};
        GLuint vaoID{};
        GLuint pointsGPU{};
        GLuint colorsGPU{};
        GLuint indicesGPU{};
};

class Camera {
    public:
        explicit Camera(glm::mat4 projection) : projection(projection) {
        }

        void setCenter(glm::vec3 inCenter) {
            center = inCenter;
        }

        void rotateY(float deltaY) {
            eulerOrientation.y += deltaY;
            eulerOrientation.y = glm::mod(eulerOrientation.y, glm::radians(360.0f));
        }

        void rotateX(float deltaX) {
            eulerOrientation.x += deltaX;
            float limit = glm::radians(80.0f);
            if (eulerOrientation.x < -limit) eulerOrientation.x = -limit;
            if (eulerOrientation.x > +limit) eulerOrientation.x = +limit;
        }

        void zoom(float scale) {
            distance += scale;
            distance = std::max(distance, 1.0f);
        }

        glm::quat getRotation() const {
            return glm::angleAxis(eulerOrientation.x, glm::vec3{1.0f, 0.0f, 0.0f}) *
                   glm::angleAxis(eulerOrientation.y, glm::vec3{0.0f, 1.0f, 0.0f});
        }

        void move(glm::vec3 displacement) {
            offset += displacement * getRotation();
        }

        glm::mat4 getView() {
            // rotate around y-axis then x-axis (order is reversed since it's essentially matrix multiplication)
            glm::quat rotation = getRotation();
            glm::vec3 target = center - offset;
            glm::vec3 eye = target + glm::vec3{0.0f, 0.0f, distance} * rotation;
            glm::vec3 up{0.0f, -1.0f, 0.0f}; // TODO: check logic on why inverted
            return glm::lookAt(eye, target, up);
        }

        glm::mat4 projection;
        float distance{5000.0f};
        glm::vec3 center{}, offset{};
        glm::vec3 eulerOrientation{};
};

// Vertex and Frag shader, stole this straight from ZED example 
// but this is basically the same in every OpenGL program so 
// it will work even without ZED 
class Shader {
    public:

        Shader() = default;

        Shader(GLchar const* vs, GLchar const* fs);

        Shader(Shader&& other) noexcept;

        ~Shader();

        Shader& operator=(Shader other);

        void swap(Shader& other);

        GLuint getProgramId();

        static const GLint ATTRIB_VERTICES_POS = 0;
        static const GLint ATTRIB_COLOR_POS = 1;
    private:
        bool compile(GLuint& shaderId, GLenum type, GLchar const* src);

        GLuint vertexShaderId{};
        GLuint fragmentShaderId{};
        GLuint programShaderId{};
};

/*
 *** Define which stage of the filtering process will be shown in the viewer
 */
enum class ProcStage {
    RAW, POSTPASS, POSTRANSAC, POSTECE, POSTBOUNDING, POSTBEARING
};

class Viewer {
    public:
        int frame = 0;
        int maxFrame = -1;
        bool framePlay = false;
        bool inMenu = false;
        bool record = false;
        ProcStage procStage = ProcStage::POSTBEARING;

        // Creates a window
        Viewer();

        ~Viewer();

        // Updates the window and draws graphics (graphics thread)
        void update();

        void drawUI();

        bool open();

        // Add an object, either ephemeral or permanent
        void addObject(Object3D&& obj, bool ephemeral);

        // Adds a point cloud
        void addPointCloud();

        // Empty the ephemeral objects array
        void clearEphemerals();

        // Sets the viewer target
        void setCenter();

        void setCenter(vec3 center);

        // need to provide thread safe ways to update viewer internals
        void updatePointCloud(int idx, std::vector<vec4>&& pts);

#ifndef VIEWER_ONLY

        void updatePointCloud(GPU_Cloud pc);

        bool doParameterInit = true;
        float epsilon;
        int iterations;
        float threshold;
        float removalRadius;
        bool removeGround;
        float tolerance;
        float minSize;

#endif

        void updateObjectModel(int idx, glm::mat4 rotation, glm::mat4 translation);

        glm::vec3 getCenter() {
            return pcCenter;
        }

    private:
        // Internals
        Camera camera;
        std::vector<Object3D> objects;
        std::vector<Object3D> ephemeralObjects;
        Shader objectShader;
        Shader pcShader;
        std::vector<PointCloud> pointClouds;
        GLFWwindow* window;

        glm::vec3 pcCenter;
        float mouseSensitivity = 1.0f;

        // which ps is being used
        int active_pc = -1;

        // Callbacks
        static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

        static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

        static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);

        // states
        double prevMouseX, prevMouseY;
        bool prevFocused = false;
};
