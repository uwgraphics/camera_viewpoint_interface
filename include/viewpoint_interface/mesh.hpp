#ifndef __MESH_HPP__
#define __MESH_HPP__

#include <string>
#include <vector>

#include <glad/glad.h> 

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "shader.hpp"


namespace viewpoint_interface
{

struct Vertex 
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 tex_coords;
    glm::vec3 tangent;
    glm::vec3 bitangent;
};

struct Texture 
{
    uint id;
    std::string name;
    std::string type;
    std::string path;
};

class Mesh 
{
public:
    Mesh(std::vector<Vertex> vertices, std::vector<uint> indices, std::vector<Texture> textures,
            aiMaterial* material) : initialized_(false), vertices_(std::move(vertices)), 
            indices_(std::move(indices)), textures_(std::move(textures)) 
    {
        aiColor3D color(0.0f, 0.0f, 0.0f);
        if(AI_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE, color)) {
            diff_color_ = { color.r, color.b, color.g };
        }
    }

    void draw(const Shader& shader) 
    {
        if (!initialized_) {
            return;
        }

        shader.setVec3("diffuse", diff_color_);

        // Bind appropriate textures
        for (uint i = 0; i < textures_.size(); i++) {
            glActiveTexture(GL_TEXTURE0 + i);
            std::string name(textures_[i].name);

            shader.setInt(name, i);
            glBindTexture(GL_TEXTURE_2D, textures_[i].id);
        }
        
        // Draw mesh
        glBindVertexArray(VAO_);
        glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        glActiveTexture(GL_TEXTURE0);
    }

    void initializeMesh()
    {
        glGenVertexArrays(1, &VAO_);
        glGenBuffers(1, &VBO_);
        glGenBuffers(1, &EBO_);

        glBindVertexArray(VAO_);

        // Load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO_);
        glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), &vertices_[0], GL_STATIC_DRAW);  

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(uint), &indices_[0], GL_STATIC_DRAW);

        // Vertex Positions
        glEnableVertexAttribArray(0);	
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        
        // Vertex normals
        glEnableVertexAttribArray(1);	
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        
        // Vertex texture coords
        glEnableVertexAttribArray(2);	
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tex_coords));
        
        // Vertex tangent
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tangent));
        
        // Vertex bitangent
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, bitangent));

        glBindVertexArray(0);
        initialized_  = true;
    }

private:
    std::vector<Vertex> vertices_;
    std::vector<uint> indices_;
    std::vector<Texture> textures_;
    glm::vec3 diff_color_;
    uint VAO_, VBO_, EBO_;
    bool initialized_;
};

} // viewpoint_interface

#endif // __MESH_HPP__