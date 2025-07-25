#include <GL/glew.h>
#include <pangolin/pangolin.h>
#include <thread>
#include <iostream>

/*
g++ test_pangolin.cpp -o test_pangolin \
  -std=c++17 \
  -I/usr/include/GL \
  -lGLEW -lGL -lpangolin -pthread
*/

class MinimalViewer {
public:
    void Run() {
        // 1. Create Window
        pangolin::CreateWindowAndBind("Pangolin Test", 1024, 768);
        
        // 2. Check if OpenGL context is valid
        if (glewInit() != GLEW_OK) {
            std::cerr << "Failed to initialize GLEW!" << std::endl;
            return;
        }

        // 3. Enable OpenGL features
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // 4. Create UI panel and variables (matching your code)
        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);

        // 5. Main loop
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // Print variable states (for debugging)
            std::cout << "Follow Camera: " << menuFollowCamera.Get() 
                      << " | Show Points: " << menuShowPoints.Get() << std::endl;
            
            pangolin::FinishFrame();
        }
    }
};

int main() {
    MinimalViewer viewer;
    std::thread viewerThread(&MinimalViewer::Run, &viewer);
    viewerThread.join();  // Wait for thread to finish
    return 0;
}
