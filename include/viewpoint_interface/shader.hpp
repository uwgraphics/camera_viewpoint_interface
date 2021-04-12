#ifndef __SHADER_H__
#define __SHADER_H__

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader
{
public:
    Shader() {}

    Shader(std::string vertexPath, std::string fragmentPath)
    {
        // Retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
        
        try 
        {
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            vShaderFile.close();
            fShaderFile.close();
            vertexCode   = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure& e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }

        const char* vShaderCode = vertexCode.c_str();
        const char * fShaderCode = fragmentCode.c_str();
        
        // Compile shaders
        unsigned int vertex, fragment;
        
        // Vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        
        // Fragment shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        
        // Shader program
        id_ = glCreateProgram();
        glAttachShader(id_, vertex);
        glAttachShader(id_, fragment);
        glLinkProgram(id_);
        checkCompileErrors(id_, "PROGRAM");

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void use() 
    { 
        glUseProgram(id_); 
    }
    
    // Uniform setting functions
    void setBool(const std::string& name, bool value) const
    {         
        glUniform1i(glGetUniformLocation(id_, name.c_str()), (int)value); 
    }

    void setInt(const std::string& name, int value) const
    { 
        glUniform1i(glGetUniformLocation(id_, name.c_str()), value); 
    }

    void setFloat(const std::string& name, float value) const
    { 
        glUniform1f(glGetUniformLocation(id_, name.c_str()), value); 
    }

    void setVec3(const std::string& name, glm::vec3 value) const
    {
        glUniform3fv(glGetUniformLocation(id_, name.c_str()), 1, glm::value_ptr(value));
    }

    void setMat4(const std::string& name, glm::mat4 value) const
    {
        glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, false, glm::value_ptr(value));
    }

private:
    unsigned int id_;

    void checkCompileErrors(unsigned int shader, std::string type)
    {
        int success;
        char infoLog[1024];
        if (type != "PROGRAM") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};

#endif // __SHADER_H__
