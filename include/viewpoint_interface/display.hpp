#ifndef __DISPLAYS_HPP__
#define __DISPLAYS_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>


namespace viewpoint_interface
{
    class DisplayManager;

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

    struct DisplayInfo
    {
        uint id;
        std::string internal, external, topic;
        std::vector<uchar> data;

        DisplayInfo(std::string &int_name, std::string &ext_name, std::string &topic_name) : 
                internal(int_name), external(ext_name), topic(topic_name) {}

        void setDataSize(uint size)
        {
            data.resize(size);
        }
    };


    class Display
    {
    public:

        Display(std::string &internal, std::string &external, std::string &topic, const DisplayDims &dims) :
                info(internal, external, topic), dimensions(dims), active(true)
        {
            info.id = getNextId();
            info.setDataSize(dimensions.size());
        }

        inline uint getId() const { return info.id; }
        inline std::string getInternalName() const { return info.internal; }
        inline std::string getExternalName() const { return info.external; }
        inline std::string getTopicName() const { return info.topic; }
        inline const DisplayInfo& getDisplayInfo() const { return info; }

        const DisplayDims& dims() const { return dimensions; }

    private:
        bool active;
        DisplayInfo info;
        DisplayDims dimensions;

        uint getNextId() 
        { 
            static uint current_id = 0;
            return current_id++;
        }

        bool isActive() const { return active; }
        void activate() { active = true; }
        void deactivate() { active = false; }
        bool flipState() 
        { 
            bool new_state = !active;
            active = new_state; 
            return new_state;
        }

        void copyImage(const cv::Mat &image)
        {
            info.data.assign(image.data, image.data + image.total()*image.channels());
        }

        friend class DisplayManager;
    };


    class DisplayManager
    {
    public:
        DisplayManager() {}

        uint size() const { return displays.size(); }

        void addDisplay(const Display &disp)
        {
            uint ix = displays.size();

            displays.push_back(disp);
            num_active_displays++;
        }

        bool isDisplayActive(uint ix) const { return displays[ix].isActive(); }
        void activateDisplay(uint ix) 
        { 
            if (displays[ix].isActive()) {
                return;
            }

            displays[ix].activate(); 
            num_active_displays++; 
        }

        void deactivateDisplay(uint ix) 
        { 
            displays[ix].deactivate();
            num_active_displays--;
        }

        void flipDisplayState(uint ix) 
        {
            if (displays[ix].flipState()) {
                num_active_displays++;
            }
            else {
                num_active_displays--;
            }
        }

        uint getDisplayId(uint ix) const
        {
            return displays[ix].getId();
        }

        std::string getDisplayExternalName(uint ix) const 
        { 
            return displays[ix].getExternalName(); 
        }

        std::string getDisplayInternalName(uint ix) const 
        { 
            return displays[ix].getInternalName(); 
        }

        std::string getDisplayTopicName(uint ix) const 
        { 
            return displays[ix].getTopicName(); 
        }

        const DisplayInfo& getDisplayInfo(uint ix) const
        { 
            return displays[ix].getDisplayInfo(); 
        }

        uint getNumActiveDisplays() const { return num_active_displays; }

        uint getNumTotalDisplays() const { return displays.size(); }

        uint getDisplayIxById(uint id) const
        {
            for (int i = 0; i < displays.size(); i++) {
                if (displays[i].getId() == id) {
                    return i;
                }
            }

            return 0;
        }

        uint getNextActiveDisplayIx(int ix) const
        {
            if (num_active_displays == 0) { return 0; }

            uint next_ix = nextIx(ix, displays.size());
            while (!isDisplayActive(next_ix))
            {
                next_ix = nextIx(next_ix, displays.size());
            }
            
            return next_ix;
        }

        uint getNextActiveDisplayId(int ix) const
        {
            return getDisplayId(getNextActiveDisplayIx(ix));
        }

        uint getPrevActiveDisplayIx(int ix) const
        {
            if (num_active_displays == 0) { return 0; }

            uint prev_ix = prevIx(ix, displays.size());
            while (!isDisplayActive(prev_ix))
            {
                prev_ix = prevIx(prev_ix, displays.size());
            }
            
            return prev_ix;
        }

        uint getPrevActiveDisplayId(int ix) const
        {
            return getDisplayId(getPrevActiveDisplayIx(ix));
        }

        void swapDisplays(uint ix1, uint ix2)
        {
            std::vector<Display> &vec(displays);
            std::iter_swap(vec.begin() + ix1, vec.begin() + ix2);
        }

        void copyImageToDisplay(uint ix, const cv::Mat &image)
        {
            displays[ix].copyImage(image);
        }


    private:       
        uint num_active_displays = 0;
        std::vector<Display> displays;


        uint nextIx(uint ix, uint size) const
        {
            return (ix + 1) % size;
        }

        uint prevIx(uint ix, uint size) const
        {
            return (ix == 0) ? (size - 1) : (ix - 1);
        }

    };

} // viewpoint_interface

#endif // __DISPLAYS_HPP__