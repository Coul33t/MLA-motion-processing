#ifndef __MLA_SHADER_H__
#define __MLA_SHADER_H__

#include "MLACommonInclude.h"

class MlaShader {

public:
	MlaShader();
	MlaShader(const MlaShader& shaderACopier);
	MlaShader(const std::string& vertexSource, const std::string& fragmentSource);
	~MlaShader();

	MlaShader& operator=(const MlaShader& shaderACopier);


	//bool load();
	void load();

	const GLuint& getProgramID() const;


private:
	void deleteShaderIfExist();
	bool compileShader(GLuint& shader, const GLenum& type, const std::string& fichierSource);

private:
	GLuint m_vertexID;
	GLuint m_fragmentID;
	GLuint m_programID;

	std::string m_vertexSource;
	std::string m_fragmentSource;
};

#endif //__MLA_SHADER_H__
