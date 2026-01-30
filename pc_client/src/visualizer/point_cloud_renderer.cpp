#include "visualizer/point_cloud_renderer.hpp"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <GLFW/glfw3.h>
#include <iostream>
#include <cstring>

namespace vi_slam {
namespace visualizer {

namespace {
// Vertex shader source code
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProjection;
uniform vec3 uDefaultColor;
uniform bool uUseVertexColors;

out vec3 fragColor;

void main() {
    gl_Position = uProjection * uView * vec4(aPos, 1.0);
    fragColor = uUseVertexColors ? aColor : uDefaultColor;
}
)";

// Fragment shader source code
const char* fragmentShaderSource = R"(
#version 330 core
in vec3 fragColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(fragColor, 1.0);
}
)";

// Compile a shader
unsigned int compileShader(unsigned int type, const char* source) {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    // Check for compilation errors
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation failed: " << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

} // anonymous namespace

PointCloudRenderer::PointCloudRenderer()
    : initialized_(false)
    , vbo_(0)
    , colorVbo_(0)
    , shaderProgram_(0)
    , pointCount_(0)
    , hasColors_(false)
{
}

PointCloudRenderer::~PointCloudRenderer() {
    shutdown();
}

bool PointCloudRenderer::initialize(const Config& config) {
    if (initialized_) {
        std::cerr << "PointCloudRenderer already initialized" << std::endl;
        return false;
    }

    config_ = config;

    // Create shaders
    if (!createShaders()) {
        std::cerr << "Failed to create shaders" << std::endl;
        return false;
    }

    // Create buffers
    createBuffers();

    initialized_ = true;
    std::cout << "PointCloudRenderer initialized successfully" << std::endl;

    return true;
}

void PointCloudRenderer::shutdown() {
    if (!initialized_) {
        return;
    }

    deleteBuffers();

    if (shaderProgram_ != 0) {
        glDeleteProgram(shaderProgram_);
        shaderProgram_ = 0;
    }

    initialized_ = false;
    std::cout << "PointCloudRenderer shutdown complete" << std::endl;
}

void PointCloudRenderer::updatePointCloud(const std::vector<Eigen::Vector3d>& points,
                                           const std::vector<Eigen::Vector3f>& colors) {
    if (!initialized_) {
        std::cerr << "PointCloudRenderer not initialized" << std::endl;
        return;
    }

    pointCount_ = points.size();
    hasColors_ = !colors.empty();

    if (pointCount_ == 0) {
        return;
    }

    // Validate colors size if provided
    if (hasColors_ && colors.size() != points.size()) {
        std::cerr << "Warning: Colors size (" << colors.size()
                  << ") doesn't match points size (" << points.size()
                  << "). Ignoring colors." << std::endl;
        hasColors_ = false;
    }

    // Convert Eigen::Vector3d to float array for OpenGL
    std::vector<float> positionData(pointCount_ * 3);
    for (size_t i = 0; i < pointCount_; ++i) {
        positionData[i * 3 + 0] = static_cast<float>(points[i].x());
        positionData[i * 3 + 1] = static_cast<float>(points[i].y());
        positionData[i * 3 + 2] = static_cast<float>(points[i].z());
    }

    // Update position buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, positionData.size() * sizeof(float),
                 positionData.data(), GL_DYNAMIC_DRAW);

    // Update color buffer if colors are provided
    if (hasColors_) {
        std::vector<float> colorData(pointCount_ * 3);
        for (size_t i = 0; i < pointCount_; ++i) {
            colorData[i * 3 + 0] = colors[i].x();
            colorData[i * 3 + 1] = colors[i].y();
            colorData[i * 3 + 2] = colors[i].z();
        }

        glBindBuffer(GL_ARRAY_BUFFER, colorVbo_);
        glBufferData(GL_ARRAY_BUFFER, colorData.size() * sizeof(float),
                     colorData.data(), GL_DYNAMIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointCloudRenderer::render(const Eigen::Matrix4f& viewMatrix,
                                 const Eigen::Matrix4f& projMatrix) {
    if (!initialized_ || pointCount_ == 0) {
        return;
    }

    // Use shader program
    glUseProgram(shaderProgram_);

    // Set uniforms
    unsigned int viewLoc = glGetUniformLocation(shaderProgram_, "uView");
    unsigned int projLoc = glGetUniformLocation(shaderProgram_, "uProjection");
    unsigned int colorLoc = glGetUniformLocation(shaderProgram_, "uDefaultColor");
    unsigned int useColorsLoc = glGetUniformLocation(shaderProgram_, "uUseVertexColors");

    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, viewMatrix.data());
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, projMatrix.data());
    glUniform3f(colorLoc, config_.defaultColor.x(), config_.defaultColor.y(), config_.defaultColor.z());
    glUniform1i(useColorsLoc, hasColors_ && config_.useVertexColors ? 1 : 0);

    // Set point size
    glPointSize(config_.pointSize);

    // Bind and setup position attribute
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Bind and setup color attribute if available
    if (hasColors_ && config_.useVertexColors) {
        glBindBuffer(GL_ARRAY_BUFFER, colorVbo_);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
    }

    // Draw points
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pointCount_));

    // Cleanup
    glDisableVertexAttribArray(0);
    if (hasColors_ && config_.useVertexColors) {
        glDisableVertexAttribArray(1);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(0);
}

void PointCloudRenderer::clear() {
    pointCount_ = 0;
    hasColors_ = false;
}

void PointCloudRenderer::setPointSize(float size) {
    config_.pointSize = size;
}

void PointCloudRenderer::setDefaultColor(const Eigen::Vector3f& color) {
    config_.defaultColor = color;
}

bool PointCloudRenderer::createShaders() {
    // Compile vertex shader
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    if (vertexShader == 0) {
        return false;
    }

    // Compile fragment shader
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return false;
    }

    // Link shaders into program
    shaderProgram_ = glCreateProgram();
    glAttachShader(shaderProgram_, vertexShader);
    glAttachShader(shaderProgram_, fragmentShader);
    glLinkProgram(shaderProgram_);

    // Check for linking errors
    int success;
    glGetProgramiv(shaderProgram_, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram_, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(shaderProgram_);
        shaderProgram_ = 0;
        return false;
    }

    // Delete shaders (they're linked into the program now)
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void PointCloudRenderer::createBuffers() {
    // Generate VBOs
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &colorVbo_);
}

void PointCloudRenderer::deleteBuffers() {
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }

    if (colorVbo_ != 0) {
        glDeleteBuffers(1, &colorVbo_);
        colorVbo_ = 0;
    }
}

} // namespace visualizer
} // namespace vi_slam
