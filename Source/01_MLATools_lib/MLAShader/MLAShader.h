#ifndef __DEF_SHADER__
#define __DEF_SHADER__

#include <glew.h>

#include <iostream>
#include <string>
#include <fstream>

class Shader {

public:
	Shader();
	Shader(Shader const &shaderACopier);
	Shader(std::string vertexSource, std::string fragmentSource);
	~Shader();

	Shader& operator=(Shader const &shaderACopier);

	void setVertexSource(const std::string&);
	void setFragmentSource(const std::string&);

	bool Load();
	bool CompileShader(GLuint &shader, GLenum type, std::string const &fichierSource);
	const GLuint& getProgramID() const;


private:
	GLuint m_vertexID;
	GLuint m_fragmentID;
	GLuint m_programID;

	std::string m_vertexSource;
	std::string m_fragmentSource;
};

#endif //__DEF_SHADER__