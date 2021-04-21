#ifndef __MODEL_HPP__
#define __MODEL_HPP__

#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "helpers.hpp"
#include "mesh.hpp"
#include "shader.hpp"
#include "instance_info.hpp"
#include "stb_image.h"


namespace viewpoint_interface
{

class Model
{
public:
    Model(const std::string& model_path, const std::string& frag_path, const std::string& vert_path,
            glm::vec2 anchor_pos=glm::vec2(0.0), float anchor_range=0.2, glm::vec3 pos=glm::vec3(0.0), 
            glm::vec3 angles=glm::vec3(0.0), float scale=1.0) :
            frag_path_(frag_path), vert_path_(vert_path), anchor_position_(anchor_pos), 
            anchor_range_(anchor_range), default_position_(pos), default_angles_(angles), 
            default_scale_(scale), valid_(false), initialized_(false) 
    {
        loadModel(model_path);
    }

    std::string getName() const { return name_; }
    glm::vec3 getDefaultPosition() const { return default_position_; }
    glm::vec3 getDefaultOrientation() const { return default_angles_; }
    glm::vec3 getDefaultScale() const { return default_scale_; }
    const glm::vec2& getAnchorPosition() const { return anchor_position_; }
    const float& getAnchorRange() const { return anchor_range_; }

    void initializeModel()
    {
        if (initialized_) { return; }

        for (auto& mesh : meshes_) {
            mesh.initializeMesh();
        }

        shader_ = Shader(vert_path_, frag_path_);

        // TODO: Load textures
        for (auto& texture : textures_) {
            std::cout << texture.path << std::endl;
        }

        initialized_ = true;
    }

    void draw(glm::mat4 model_mat, InstanceState& state)
    {
        if (!valid_ || !initialized_) {
            return;
        }

        shader_.use();
        shader_.setMat4("model_mat", model_mat);

        if (state.selection_ == SelectionState::Hovered ||
                state.selection_ == SelectionState::Selected) {
            shader_.setBool("highlight", true);
        }
        else {
            shader_.setBool("highlight", false);
        }

        for (auto& mesh : meshes_) {
            mesh.draw(shader_);
        }
    }

private:
    Shader shader_;
    glm::vec3 default_position_;
    glm::vec3 default_angles_;
    glm::vec3 default_scale_;
    glm::vec2 anchor_position_;
    float anchor_range_;
    std::string directory_;
    std::string name_;
    std::string frag_path_;
    std::string vert_path_;
    std::vector<Texture> textures_; // Stores all unique loaded textures
    std::vector<Mesh> meshes_;
    bool valid_, initialized_;

    void loadModel(std::string const &path)
    {
        Assimp::Importer importer;
        const aiScene* scene(importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenSmoothNormals | 
            aiProcess_FlipUVs | aiProcess_CalcTangentSpace));
        
        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            printText("Error loading model: " + std::string(importer.GetErrorString()));
            valid_ = false;
            return;
        }

        size_t dir_end(path.find_last_of('/'));
        directory_ = path.substr(0, dir_end);
        name_ = path.substr(dir_end+1, path.find_first_of(".") - (dir_end+1));

        // Process the scene nodes recursively
        processNode(scene->mRootNode, scene);

        valid_ = true;
    }

    void processNode(aiNode *node, const aiScene *scene)
    {
        for (uint i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* mesh(scene->mMeshes[node->mMeshes[i]]);
            meshes_.push_back(processMesh(mesh, scene));
        }

        // Process children nodes
        for (uint i = 0; i < node->mNumChildren; ++i) {
            processNode(node->mChildren[i], scene);
        }
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene)
    {
        std::vector<Vertex> vertices;
        std::vector<uint> indices;
        std::vector<Texture> textures;

        // Get mesh vertices
        for (uint i = 0; i < mesh->mNumVertices; ++i) {
            Vertex vertex;
            glm::vec3 vector;

            // Position
            vector.x = mesh->mVertices[i].x;
            vector.y = mesh->mVertices[i].y;
            vector.z = mesh->mVertices[i].z;
            vertex.position = vector;
            
            // Normals
            if (mesh->HasNormals()) {
                vector.x = mesh->mNormals[i].x;
                vector.y = mesh->mNormals[i].y;
                vector.z = mesh->mNormals[i].z;
                vertex.normal = vector;
            }

            // UV's
            if (mesh->mTextureCoords[0]) {
                glm::vec2 vec;
                // Vertex can contain up to 8 different texture coordinates, but we only take first set
                vec.x = mesh->mTextureCoords[0][i].x; 
                vec.y = mesh->mTextureCoords[0][i].y;
                vertex.tex_coords = vec;

                // Tangent
                vector.x = mesh->mTangents[i].x;
                vector.y = mesh->mTangents[i].y;
                vector.z = mesh->mTangents[i].z;
                vertex.tangent = vector;

                // Bitangent
                vector.x = mesh->mBitangents[i].x;
                vector.y = mesh->mBitangents[i].y;
                vector.z = mesh->mBitangents[i].z;
                vertex.bitangent = vector;
            }
            else {
                vertex.tex_coords = glm::vec2(0.0f, 0.0f);
            }

            vertices.push_back(vertex);
        }

        // Get face indices
        for (uint i = 0; i < mesh->mNumFaces; ++i) {
            aiFace face(mesh->mFaces[i]);
            
            for (uint j = 0; j < face.mNumIndices; ++j) {
                indices.push_back(face.mIndices[j]);        
            }
        }

        // Process materials and textures
        aiMaterial* material(scene->mMaterials[mesh->mMaterialIndex]);

        // Diffuse maps
        std::vector<Texture> diffuse_maps(loadMaterialTextureInfo(material, aiTextureType_DIFFUSE, "texture_diffuse"));
        textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());
        
        // Specular maps
        std::vector<Texture> specular_maps(loadMaterialTextureInfo(material, aiTextureType_SPECULAR, "texture_specular"));
        textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());
        
        // Normal maps
        std::vector<Texture> normal_maps(loadMaterialTextureInfo(material, aiTextureType_HEIGHT, "texture_normal"));
        textures.insert(textures.end(), normal_maps.begin(), normal_maps.end());
        
        // Height maps
        std::vector<Texture> height_maps(loadMaterialTextureInfo(material, aiTextureType_AMBIENT, "texture_height"));
        textures.insert(textures.end(), height_maps.begin(), height_maps.end());
        
        return Mesh(std::move(vertices), std::move(indices), std::move(textures), material);
    }

    std::vector<Texture> loadMaterialTextureInfo(aiMaterial *mat, aiTextureType type, std::string type_name)
    {
        std::vector<Texture> load_textures;
        for (uint i = 0; i < mat->GetTextureCount(type); ++i) {
            aiString tex_path;
            mat->GetTexture(type, i, &tex_path);

            // Check if texture was loaded before and, if so, skip loading
            bool skip(false);
            for (auto texture : textures_)
            {
                if (std::strcmp(texture.path.data(), tex_path.C_Str()) == 0) {
                    load_textures.push_back(texture);
                    skip = true;
                    break;
                }
            }

            if (!skip) {   
                Texture texture;
                std::string path(tex_path.C_Str());
                std::string filename;
                if (path.find_last_of("/\\") == std::string::npos) { // No slashes found
                    filename = path;
                }
                else {
                    filename = path.substr(path.find_last_of("/\\"));
                }

                // texture.id = TextureFromFile(tex_path.C_Str(), directory_);
                texture.name = filename.substr(0, filename.find_first_of("."));
                texture.type = type_name;
                texture.path = path;
                load_textures.push_back(texture);
                textures_.push_back(texture);
            }
        }

        return load_textures;
    }
};

uint loadTextureFromFile(const std::string& path, const std::string &directory)
{
    std::string full_path(directory + '/' + path);

    uint tex_id;
    glGenTextures(1, &tex_id);

    int width, height, num_components;
    unsigned char *data = stbi_load(full_path.c_str(), &width, &height, &num_components, 0);
    if (data) {
        GLenum format;
        if (num_components == 1) {
            format = GL_RED;
        }
        else if (num_components == 3) {
            format = GL_RGB;
        }
        else if (num_components == 4) {
            format = GL_RGBA;
        }

        glBindTexture(GL_TEXTURE_2D, tex_id);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        printText("Texture failed to load at path: " + path);
        stbi_image_free(data);
    }

    return tex_id;
}

} // viewpoint_interface

#endif // __MODEL_HPP__