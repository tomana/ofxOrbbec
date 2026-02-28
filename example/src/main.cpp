#include "ofMain.h"
#include "ofApp.h"
#include <GLFW/glfw3.h>

//========================================================================
int main( ){

#if defined(GLFW_VERSION_MAJOR) && (GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR==3 && GLFW_VERSION_MINOR>=4))
    // Force Wayland (alternatively: GLFW_PLATFORM_X11)
        glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#else
    // Older GLFW: no init hint available — use env var instead:
    // putenv(const_cast<char*>
#endif

	//Use ofGLFWWindowSettings for more options like multi-monitor fullscreen
	ofGLWindowSettings settings;
	settings.setSize(1024, 768);
	settings.windowMode = OF_WINDOW; //can also be OF_FULLSCREEN

	auto window = ofCreateWindow(settings);

	ofRunApp(window, std::make_shared<ofApp>());
	ofRunMainLoop();

}
