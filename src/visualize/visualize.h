#pragma once

#include <small_point_lio/pch.h>
#include <pangolin/pangolin.h>

namespace visualize {

    class GridRenderer {
    private:
        std::vector<float> vertices;
        std::vector<unsigned int> indices;
        pangolin::GlBuffer vbo;
        pangolin::GlBuffer ibo;
        pangolin::GlSlProgram shader;

    public:
        GridRenderer();

    private:
        void init();

        void compileShader();

    public:
        void render(const pangolin::OpenGlMatrix &mvp);
    };

    class Visualize {
    public:
        bool is_running = true;
        std::mutex mutex;
        std::vector<Eigen::Vector3f> pointcloud_map;
        std::vector<Eigen::Vector3f> pointcloud_realtime;
        std::vector<Eigen::Vector3f> path;

        void loop();
    };

}// namespace visualize
