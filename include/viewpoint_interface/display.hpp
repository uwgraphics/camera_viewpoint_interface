#ifndef __DISPLAYS_HPP__
#define __DISPLAYS_HPP__

#include <string>
#include <vector>

namespace viewpoint_interface
{

    struct DisplayDims
    {
        uint width;
        uint height;
        uint channels;

        DisplayDims(uint w, uint h, uint c) : int_size(w * h * c) {}

        inline uint size() const { return int_size; }

    private:
        uint int_size;
    };


    // struct DisplayBuffers
    // {
    //     DisplayDims dimensions;
    //     std::vector<uint> data;

    //     DisplayBuffers(DisplayDims dims) : dimensions(dims)
    //     {
    //         data = std::vector<uint>(dimensions.size());
    //     }
    // };


    class Display
    {
    public:

        Display(std::string &internal, std::string &external, std::string &topic, const DisplayDims &dims) :
                int_name(internal), ext_name(external), topic_name(topic), dimensions(dims) {}

        inline std::string getInternalName() const { return int_name; }
        inline std::string getExternalName() const { return ext_name; }
        inline std::string getTopicName() const { return topic_name; }

        const DisplayDims& dims() const { return dimensions; }

    private:
        std::string int_name, ext_name, topic_name;
        DisplayDims dimensions;
    };


    class DisplayManager
    {
    public:
        static const uint max_buffers = 9;

        DisplayManager(uint num = max_buffers, DisplayDims dims = DisplayDims{1024, 1024, 3})
        {
            for (int i = 0; i < num; i++) {
                buffers.push_back(std::vector<uint>(dims.size()));
            }
        }

        void addDisplay(const Display &disp)
        {
            displays.push_back(disp);
        }

        uint size() { return displays.size(); }

        Display operator [](uint ix)
        {
            return displays.at(ix);
        }

    private:
        std::vector<Display> displays;
        std::vector<uint> active_buffers;
        std::vector<std::vector<uint>> buffers;
    };

} // viewpoint_interface

#endif // __DISPLAYS_HPP__