#include "visualizer/trajectory_renderer.hpp"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <GLFW/glfw3.h>
#include <iostream>
#include <cstring>
#include <cmath>

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

out vec3 fragColor;

void main() {
    gl_Position = uProjection * uView * vec4(aPos, 1.0);
    fragColor = aColor;
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

TrajectoryRenderer::TrajectoryRenderer()
    : initialized_(false)
    , lineVbo_(0)
    , lineColorVbo_(0)
    , frustumVbo_(0)
    , frustumColorVbo_(0)
    , shaderProgram_(0)
    , poseCount_(0)
    , lineVertexCount_(0)
    , frustumVertexCount_(0)
{
}

TrajectoryRenderer::~TrajectoryRenderer() {
    shutdown();
}

bool TrajectoryRenderer::initialize(const Config& config) {
    if (initialized_) {
        std::cerr << "TrajectoryRenderer already initialized" << std::endl;
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
    std::cout << "TrajectoryRenderer initialized successfully" << std::endl;

    return true;
}

void TrajectoryRenderer::shutdown() {
    if (!initialized_) {
        return;
    }

    deleteBuffers();

    if (shaderProgram_ != 0) {
        glDeleteProgram(shaderProgram_);
        shaderProgram_ = 0;
    }

    initialized_ = false;
    poseCount_ = 0;
    lineVertexCount_ = 0;
    frustumVertexCount_ = 0;

    std::cout << "TrajectoryRenderer shutdown complete" << std::endl;
}

void TrajectoryRenderer::updateTrajectory(const std::vector<Pose>& poses) {
    if (!initialized_) {
        std::cerr << "TrajectoryRenderer not initialized" << std::endl;
        return;
    }

    if (poses.empty()) {
        clear();
        return;
    }

    poseCount_ = poses.size();

    // Build line strip vertices and colors
    std::vector<Eigen::Vector3f> lineVertices;
    std::vector<Eigen::Vector3f> lineColors;
    lineVertices.reserve(poses.size());
    lineColors.reserve(poses.size());

    for (const auto& pose : poses) {
        Eigen::Vector3d position = pose.transform.translation();
        lineVertices.push_back(position.cast<float>());
        lineColors.push_back(interpolateColor(pose.trackingQuality));
    }

    // Update line VBO
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 lineVertices.size() * sizeof(Eigen::Vector3f),
                 lineVertices.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, lineColorVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 lineColors.size() * sizeof(Eigen::Vector3f),
                 lineColors.data(),
                 GL_DYNAMIC_DRAW);

    lineVertexCount_ = lineVertices.size();

    // Build frustum vertices
    std::vector<Eigen::Vector3f> frustumVertices;
    std::vector<Eigen::Vector3f> frustumColors;

    for (size_t i = 0; i < poses.size(); i += config_.frustumInterval) {
        Eigen::Vector3f color = interpolateColor(poses[i].trackingQuality);
        generateFrustum(poses[i].transform, frustumVertices, frustumColors, color);
    }

    // Update frustum VBO
    if (!frustumVertices.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, frustumVbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     frustumVertices.size() * sizeof(Eigen::Vector3f),
                     frustumVertices.data(),
                     GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, frustumColorVbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     frustumColors.size() * sizeof(Eigen::Vector3f),
                     frustumColors.data(),
                     GL_DYNAMIC_DRAW);

        frustumVertexCount_ = frustumVertices.size();
    } else {
        frustumVertexCount_ = 0;
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void TrajectoryRenderer::render(const Eigen::Matrix4f& viewMatrix, const Eigen::Matrix4f& projMatrix) {
    if (!initialized_ || poseCount_ == 0) {
        return;
    }

    glUseProgram(shaderProgram_);

    // Set uniforms
    int viewLoc = glGetUniformLocation(shaderProgram_, "uView");
    int projLoc = glGetUniformLocation(shaderProgram_, "uProjection");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, viewMatrix.data());
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, projMatrix.data());

    // Render trajectory line
    if (lineVertexCount_ > 0) {
        glLineWidth(config_.lineWidth);

        glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, lineColorVbo_);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glDrawArrays(GL_LINE_STRIP, 0, static_cast<int>(lineVertexCount_));

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }

    // Render camera frustums
    if (frustumVertexCount_ > 0) {
        glLineWidth(1.0f);

        glBindBuffer(GL_ARRAY_BUFFER, frustumVbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, frustumColorVbo_);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glDrawArrays(GL_LINES, 0, static_cast<int>(frustumVertexCount_));

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

void TrajectoryRenderer::clear() {
    poseCount_ = 0;
    lineVertexCount_ = 0;
    frustumVertexCount_ = 0;
}

void TrajectoryRenderer::setLineWidth(float width) {
    config_.lineWidth = width;
}

void TrajectoryRenderer::setFrustumSize(float size) {
    config_.frustumSize = size;
}

bool TrajectoryRenderer::createShaders() {
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

    // Link shader program
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

    // Clean up individual shaders (no longer needed after linking)
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void TrajectoryRenderer::createBuffers() {
    // Create line VBOs
    glGenBuffers(1, &lineVbo_);
    glGenBuffers(1, &lineColorVbo_);

    // Create frustum VBOs
    glGenBuffers(1, &frustumVbo_);
    glGenBuffers(1, &frustumColorVbo_);
}

void TrajectoryRenderer::deleteBuffers() {
    if (lineVbo_ != 0) {
        glDeleteBuffers(1, &lineVbo_);
        lineVbo_ = 0;
    }
    if (lineColorVbo_ != 0) {
        glDeleteBuffers(1, &lineColorVbo_);
        lineColorVbo_ = 0;
    }
    if (frustumVbo_ != 0) {
        glDeleteBuffers(1, &frustumVbo_);
        frustumVbo_ = 0;
    }
    if (frustumColorVbo_ != 0) {
        glDeleteBuffers(1, &frustumColorVbo_);
        frustumColorVbo_ = 0;
    }
}

void TrajectoryRenderer::generateFrustum(const Eigen::Isometry3d& pose,
                                        std::vector<Eigen::Vector3f>& vertices,
                                        std::vector<Eigen::Vector3f>& colors,
                                        const Eigen::Vector3f& color) const {
    // Camera frustum is a pyramid with 5 vertices (apex + 4 corners)
    // Scale frustum by config_.frustumSize
    float s = config_.frustumSize;

    // Define frustum corners in camera frame (apex at origin, looking along +Z)
    Eigen::Vector3d apex(0.0, 0.0, 0.0);
    Eigen::Vector3d tl(-s, -s, 2*s);  // Top-left
    Eigen::Vector3d tr(s, -s, 2*s);   // Top-right
    Eigen::Vector3d bl(-s, s, 2*s);   // Bottom-left
    Eigen::Vector3d br(s, s, 2*s);    // Bottom-right

    // Transform to world frame
    auto transform = [&pose](const Eigen::Vector3d& p) {
        return (pose * p).cast<float>();
    };

    Eigen::Vector3f apexW = transform(apex);
    Eigen::Vector3f tlW = transform(tl);
    Eigen::Vector3f trW = transform(tr);
    Eigen::Vector3f blW = transform(bl);
    Eigen::Vector3f brW = transform(br);

    // Add edges (12 edges total)
    // Pyramid edges (apex to corners)
    vertices.push_back(apexW); vertices.push_back(tlW);
    vertices.push_back(apexW); vertices.push_back(trW);
    vertices.push_back(apexW); vertices.push_back(blW);
    vertices.push_back(apexW); vertices.push_back(brW);

    // Far plane edges
    vertices.push_back(tlW); vertices.push_back(trW);
    vertices.push_back(trW); vertices.push_back(brW);
    vertices.push_back(brW); vertices.push_back(blW);
    vertices.push_back(blW); vertices.push_back(tlW);

    // Add colors for all vertices (2 per edge, 8 edges = 16 vertices)
    for (int i = 0; i < 16; ++i) {
        colors.push_back(color);
    }
}

Eigen::Vector3f TrajectoryRenderer::interpolateColor(float quality) const {
    // Clamp quality to [0, 1]
    quality = std::max(0.0f, std::min(1.0f, quality));

    // Linear interpolation between poor (red) and good (green)
    return config_.poorTrackColor * (1.0f - quality) + config_.goodTrackColor * quality;
}

} // namespace visualizer
} // namespace vi_slam
