#include "MLAShader.h"


// Constructors
MlaShader::MlaShader() : m_vertexID(0), m_fragmentID(0), m_programID(0), m_vertexSource(""), m_fragmentSource("")
{
}




MlaShader::MlaShader(MlaShader const &shaderToCopy) {

	// Source file copy
	m_vertexSource = shaderToCopy.m_vertexSource;
	m_fragmentSource = shaderToCopy.m_fragmentSource;


	// Shader loading
	load();
}

//why is there no loading ? to make a test in the Character::character
MlaShader::MlaShader(const std::string& vertexSource, const std::string& fragmentSource) : m_vertexID(0), m_fragmentID(0), m_programID(0),
m_vertexSource(vertexSource), m_fragmentSource(fragmentSource) {

	// Shader loading
	load();
}

// Destructor
//need to check if the shader exists
//MlaShader::~MlaShader() {
//    glDeleteShader(m_vertexID);
//    glDeleteShader(m_fragmentID);
//    glDeleteProgram(m_programID);
//}

// Destructor
MlaShader::~MlaShader() {
	// Destruction of a potential old shader
	deleteShaderIfExist();

}


// Methods

MlaShader& MlaShader::operator=(const MlaShader& shaderToCopy) {

	// Source file copy
	m_vertexSource = shaderToCopy.m_vertexSource;
	m_fragmentSource = shaderToCopy.m_fragmentSource;

	// Shader loading
	load();

	return *this;
}


//add this function to not repeat code
void MlaShader::deleteShaderIfExist(){
	// Destruction of a potential old shader
	if (m_vertexID && (glIsShader(m_vertexID) == GL_TRUE))
		glDeleteShader(m_vertexID);

	if (m_fragmentID && (glIsShader(m_fragmentID) == GL_TRUE))
		glDeleteShader(m_fragmentID);

	if (m_programID && (glIsProgram(m_programID) == GL_TRUE))
		glDeleteProgram(m_programID);

}

//bool MlaShader::load() {
void MlaShader::load() {

	// Destruction of a potential old shader
	deleteShaderIfExist();


	// Compilation of the new shaders
	if (!compileShader(m_vertexID, GL_VERTEX_SHADER, m_vertexSource)){
		MlaECoutLine("Fail compile vertex shader");
		return;
	}
	if (!compileShader(m_fragmentID, GL_FRAGMENT_SHADER, m_fragmentSource)){
		MlaECoutLine("Fail compile fragment shader");
		return;
	}


	// Program creation
	m_programID = glCreateProgram();


	// Shaders binding
	glAttachShader(m_programID, m_vertexID);
	glAttachShader(m_programID, m_fragmentID);


	// Locking of shaders input
	glBindAttribLocation(m_programID, 0, "in_Vertex");
	glBindAttribLocation(m_programID, 1, "in_Color");
	glBindAttribLocation(m_programID, 2, "in_TexCoord0");


	// Program linking
	glLinkProgram(m_programID);


	// Linking check
	GLint linkError(0);
	glGetProgramiv(m_programID, GL_LINK_STATUS, &linkError);


	if (linkError != GL_TRUE) {
		// Error size
		GLint errorSize(0);
		glGetProgramiv(m_programID, GL_INFO_LOG_LENGTH, &errorSize);

		// Memory allocation
		char *error = new char[errorSize + 1];

		// Error recovering
		glGetShaderInfoLog(m_programID, errorSize, &errorSize, error);
		error[errorSize] = '\0';

		// Error display
		MlaECoutLine("Link error: " + std::string(error));
		// std::cout << error << std::endl;


		// Freeing memory and returning false
		delete[] error;
		glDeleteProgram(m_programID);

		// return false;
	}



	// Else, everybody went good

	//else
	//  return true;
}


//this is a private function
bool MlaShader::compileShader(GLuint &shader, const GLenum& type, const std::string &sourceFile) {

	// Shader creation
	shader = glCreateShader(type);

	// Shader checking
	if (shader == 0) {
		MlaECoutLine("Error, shader type (" + std::to_string(type) + ") does not exist");
		//std::cout << "Error, shader type (" << type << ") does not exist" << std::endl;
		return false;
	}

	// reading stream
	std::ifstream file(sourceFile.c_str());

	// Opening test
	if (!file) {

		MlaECoutLine("Error, file " + sourceFile + " not found");
		//std::cout << "Error, file " << sourceFile << " not found" << std::endl;
		glDeleteShader(shader);
		return false;
	}

	// String to read the source code
	std::string line;
	std::string sourceCode;

	// Reading
	while (getline(file, line))
		sourceCode += line + '\n';

	// File closing
	file.close();

	// Retrieving of the C string
	const GLchar* sourceCodeChain = sourceCode.c_str();

	// Sending the source code to the shader
	glShaderSource(shader, 1, &sourceCodeChain, 0);

	// Shader compilation
	glCompileShader(shader);

	// Compilation checking
	GLint compilationError(0);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compilationError);


	if (compilationError != GL_TRUE) {
		// Error size retrieving
		GLint errorSize(0);
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &errorSize);

		// Memory allocation
		char *error = new char[errorSize + 1];

		// Error retrieving
		glGetShaderInfoLog(shader, errorSize, &errorSize, error);
		error[errorSize] = '\0';

		// Error display
		MlaECoutLine("Compilation error: " + std::string(error));
		std::cout << error << std::endl;

		// Freeing memory and returning false
		delete[] error;
		glDeleteShader(shader);

		return false;
	}

	else
		return true;
}


// Getter
const GLuint& MlaShader::getProgramID() const {
	return m_programID;
}