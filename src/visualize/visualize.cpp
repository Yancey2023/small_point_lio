#include "visualize.h"

namespace visualize {

    GridRenderer::GridRenderer() {
        init();
        compileShader();
    }

    void GridRenderer::init() {
        vbo = pangolin::GlBuffer(pangolin::GlArrayBuffer, 0, GL_FLOAT, 3, GL_STATIC_DRAW);
        ibo = pangolin::GlBuffer(pangolin::GlElementArrayBuffer, 0, GL_UNSIGNED_INT, 1, GL_STATIC_DRAW);

        const int grid_size = 50;
        const float square_size = 1.0f;
        const float total_size = grid_size * square_size;
        const float half_size = total_size / 2.0f;

        // 生成顶点数据 - 只存储网格交点
        vertices.reserve((grid_size + 1) * (grid_size + 1) * 3);

        for (int i = 0; i <= grid_size; ++i) {
            for (int j = 0; j <= grid_size; ++j) {
                vertices.push_back(j * square_size - half_size);// x
                vertices.push_back(i * square_size - half_size);// y
                vertices.push_back(0.0f);                       // z
            }
        }

        // 生成索引数据 - 水平线
        indices.reserve((grid_size + 1) * grid_size * 2 + grid_size * (grid_size + 1) * 2);

        // 水平线
        for (int i = 0; i <= grid_size; ++i) {
            for (int j = 0; j < grid_size; ++j) {
                int start = i * (grid_size + 1) + j;
                indices.push_back(start);
                indices.push_back(start + 1);
            }
        }

        // 垂直线
        for (int j = 0; j <= grid_size; ++j) {
            for (int i = 0; i < grid_size; ++i) {
                int start = i * (grid_size + 1) + j;
                indices.push_back(start);
                indices.push_back(start + grid_size + 1);
            }
        }

        // 上传数据到GPU
        vbo.Bind();
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
                     vertices.data(), GL_STATIC_DRAW);

        ibo.Bind();
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                     indices.data(), GL_STATIC_DRAW);
    }

    void GridRenderer::compileShader() {
        const char *vertex_shader = R"(
            #version 330 core
            layout(location = 0) in vec3 aPos;
            uniform mat4 MVP;
            void main() {
                gl_Position = MVP * vec4(aPos, 1.0);
            })";

        const char *fragment_shader = R"(
            #version 330 core
            out vec4 FragColor;
            uniform vec3 color;
            void main() {
                FragColor = vec4(color, 1.0);
            })";

        shader.AddShader(pangolin::GlSlVertexShader, vertex_shader);
        shader.AddShader(pangolin::GlSlFragmentShader, fragment_shader);
        shader.Link();
    }

    void GridRenderer::render(const pangolin::OpenGlMatrix &mvp) {
        shader.Bind();
        shader.SetUniform("MVP", mvp);
        shader.SetUniform("color", 0.7f, 0.7f, 0.7f);// 灰色网格

        vbo.Bind();
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);

        ibo.Bind();

        glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);

        glDisableVertexAttribArray(0);
        shader.Unbind();
    }

    void Visualize::loop() {
        pangolin::CreateWindowAndBind("Navigation Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);

        // 定义相机投影和模型视图矩阵
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -5, 10, 0, 0, 0, pangolin::AxisZ));

        // 创建交互视图
        pangolin::Handler3D handler(s_cam);
        pangolin::View &d_cam = pangolin::CreateDisplay()
                                        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                                        .SetHandler(&handler);

        std::unique_ptr<GridRenderer> grid_renderer = std::make_unique<GridRenderer>();

        while (!pangolin::ShouldQuit()) {
            if (!mutex.try_lock()) {
                continue;
            }
            // if (!is_running) {
            //     break;
            // }

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            glEnable(GL_DEPTH_TEST);

            if (!is_running) {
                glLineWidth(2);
                glColor3f(1.0, 1.0, 1.0);
                glBegin(GL_POINTS);
                for (const auto &point: pointcloud_map) {
                    glVertex3f(point.x(), point.y(), point.z());
                }
                glEnd();
            }

            glDisable(GL_DEPTH_TEST);

            grid_renderer->render(s_cam.GetProjectionModelViewMatrix());

            glLineWidth(3);
            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_POINTS);
            for (const auto& point : pointcloud_realtime) {
                glVertex3f(point.x(), point.y(), point.z());
            }
            glEnd();

            glLineWidth(3);
            glColor3f(1.0, 0.0, 1.0);
            glBegin(GL_LINE_STRIP);
            for (const auto &position: path) {
                glVertex3f(position.x(), position.y(), position.z());
            }
            glEnd();

            mutex.unlock();

            pangolin::FinishFrame();
        }
        grid_renderer = nullptr;
        pangolin::DestroyWindow("Navigation Viewer");
    }

}// namespace visualize
