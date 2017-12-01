/**
MainWindow.h

@author Quentin Couland
@version 0.1

*/

#ifndef __MLA_MAINWINDOW_H__
#define __MLA_MAINWINDOW_H__

#include "MLATools_lib.h"
#include "MLAMotion_lib.h"
#include "MLAParser_lib.h"
#include "MLAGui_lib.h"
#include "MLANeuron_lib.h"
#include "MLARender_lib.h"

class MainWindow {

public:
	MainWindow();
	MainWindow(const std::string, const int, const int);
	~MainWindow();

	int WindowInit();
	bool GLinit();
	bool LoadShader(const std::string, const std::string);
	void MainLoop(Motion*);

	void DrawStaticMotion(Motion*);
	void Animate(Motion*, const double, const double);
	

private:
	std::string m_window_title;
	int m_window_width;
	int m_window_height;

	SDL_Window* m_window;
	SDL_GLContext m_openGL_context;
	SDL_Event m_events;
	Input m_input;

	int m_state;

	Shader m_shader;

	glm::dmat4 m_modelview;
	glm::dmat4 m_projection;

	Camera m_camera;

	NeuronConnection m_neuron;
};
#endif //__MLA_MAINWINDOW_H__