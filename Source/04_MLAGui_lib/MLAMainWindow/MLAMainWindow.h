#ifndef __MLA_MAINWINDOW_H__
#define __MLA_MAINWINDOW_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"
#include "MLAGui_lib.h"

class MainWindow {

public:
	MainWindow();
	MainWindow(std::string, int, int);
	~MainWindow();

	int WindowInit();
	bool GLinit();
	bool LoadShader(std::string const, std::string const);
	void MainLoop(Motion*);

	void DrawStaticMotion(Motion*);
	void Animate(Motion*, float, float);

	void DisplayLine(glm::mat4 &projection, glm::mat4 &modelview, double *line_vertices, double *line_colour);
	void DisplayPoint(glm::mat4 &projection, glm::mat4 &modelview, double *point, double *point_color);

private:
	std::string m_windowTitle;
	int m_windowWidth;
	int m_windowHeight;

	SDL_Window* m_window;
	SDL_GLContext m_openGLContext;
	SDL_Event m_events;
	Input m_input;

	int m_state;

	Shader m_shader;

	glm::mat4 m_modelview;
	glm::mat4 m_projection;

	Camera m_camera;
};
#endif //__MLA_MAINWINDOW_H__