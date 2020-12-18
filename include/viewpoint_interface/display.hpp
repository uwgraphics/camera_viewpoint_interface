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


    struct DisplayBuffers
    {
        DisplayDims dimensions;
        std::vector<uchar> data;

        DisplayBuffers(DisplayDims dims=DisplayDims{1024, 1024, 3}) : dimensions(dims)
        {
            data = std::vector<uchar>(dimensions.size());
        }
    };


    class Display
    {
    public:

        Display(std::string &internal, std::string &external, std::string &topic, const DisplayDims &dims) :
                int_name(internal), ext_name(external), topic_name(topic), dimensions(dims), active(true) {}

        inline std::string getInternalName() const { return int_name; }
        inline std::string getExternalName() const { return ext_name; }
        inline std::string getTopicName() const { return topic_name; }

        const DisplayDims& dims() const { return dimensions; }

    private:
        std::string int_name, ext_name, topic_name;
        DisplayDims dimensions;
        bool active;

        bool isActive() const { return active; }
        void activate() { active = true; }
        void deactivate() { active = false; }
        bool flipState() 
        { 
            bool new_state = !active;
            active = new_state; 
            return new_state;
        }

        friend class DisplayManager;
    };


    enum class DisplayLoadHint
    {
        DisplayHint_LoadOne,    // Load only one display--unable to predict next one
        DisplayHint_Neighbors   // Load previous and next displays in ring
    };

    enum DisplayRole
    {
        DisplayRole_Primary,
        DisplayRole_Secondary1
    };

    class DisplayManager
    {
    public:
        static const uint max_buffers = 9;

        DisplayManager(uint num=max_buffers)
        {
            for (int i = 0; i < num; i++) {
                buffers.push_back(DisplayBuffers());
            }
        }

        uint size() const { return displays.size(); }

        Display &operator [](uint ix)
        {
            return displays.at(ix);
        }

        void addDisplay(const Display &disp)
        {
            uint ix = displays.size();

            active_displays.push_back(ix);
            displays.push_back(disp);
            num_active_displays++;

            if (ix == primary_display_ix) {
                bindDisplayToBufferIx(ix, primary_buffer_ix);
            }
            else {
                // TODO: Handle secondary displays
                bindDisplayToBufferIx(ix, UNBOUND_BUFFER);
            }
        }

        bool isDisplayActive(uint ix) const { return displays[ix].isActive(); }
        void activateDisplay(uint ix) { displays[ix].activate(); num_active_displays++; }
        void deactivateDisplay(uint ix) 
        { 
            displays[ix].deactivate(); 
            num_active_displays--;
            
            if (primary_display_ix == ix) {
                setPrimaryDisplay(nextActiveDisplayIx());
            }
        }

        void flipDisplayState(uint ix) 
        {
            if (displays[ix].flipState()) {
                num_active_displays++;
            }
            else {
                num_active_displays--;

                if (primary_display_ix == ix) {
                    setPrimaryDisplay(nextActiveDisplayIx());
                }
            }
        }

        uint getNumActiveDisplays() const { return num_active_displays; }

        uint getPrimaryDisplayIx() const { return primary_display_ix; }
        void setPrimaryDisplay(uint ix)
        {
            if (ix == primary_display_ix) {
                return;
            }

            unbindDisplayFromBufferIx(primary_display_ix);
            primary_display_ix = ix;
            bindDisplayToBufferIx(ix, primary_buffer_ix);
        }

        const Display &getPrimaryDisplay() const { return displays.at(primary_display_ix); }
        ImVec4 getPrimaryColorActive() const { return primary_color_active; }
        ImVec4 getPrimaryColorHovered() const { return primary_color_hovered; }
        ImVec4 getPrimaryColorBase() const { return primary_color_base; }


        void swapDisplays(uint ix1, uint ix2)
        {
            if (ix1 == primary_display_ix) {
                setPrimaryDisplay(ix2);
            }
            else if (ix2 == primary_display_ix) {
                setPrimaryDisplay(ix1);
            }

            std::vector<Display> &vec(displays);
            std::iter_swap(vec.begin() + ix1, vec.begin() + ix2);
        }

        void toNextPrimaryDisplay() { setPrimaryDisplay(nextActiveDisplayIx()); }
        void toPrevPrimaryDisplay() { setPrimaryDisplay(prevActiveDisplayIx()); }

        void fillBufferForDisplayIx(uint ix, const cv::Mat &image)
        {
            // TODO: Figure out what to do if image size != buffer size

            int buffer_ix = getBufferIxForDisplay(ix);
            // TODO: Verify whether buffer will ever be unbound at this point
            // Bind it if so

            DisplayBuffers &buffer(getBuffer(buffer_ix));
            buffer.data.assign(image.data, image.data + image.total()*image.channels());
        }

        const std::vector<uchar> &getDataVectorForDisplayIx(uint ix)
        {
            return buffers.at(getBufferIxForDisplay(ix)).data;
        }

        const std::vector<uchar> &getDataVectorForRole(DisplayRole role)
        {
            switch(role)
            {
                case DisplayRole::DisplayRole_Primary:
                {                    
                    return buffers.at(primary_buffer_ix).data;
                }   break;

                default:
                {
                    return buffers.at(primary_buffer_ix).data;
                }
            }
        }

        void bufferDisplay()
        {
            // TODO: Allow manually buffering
        }

    private:
        uint primary_display_ix = 0;
        
        const ImVec4 primary_color_active{10.0/255, 190.0/255, 10.0/255, 255.0/255};
        const ImVec4 primary_color_hovered{10.0/255, 190.0/255, 10.0/255, 200.0/255};
        const ImVec4 primary_color_base{10.0/255, 190.0/255, 10.0/255, 150.0/255};
       
        uint num_active_displays = 0;
        std::vector<uint> active_displays;
        std::vector<Display> displays;
        std::vector<uint> display_ring; // TODO: Implement functions to handle ring
        // We should not disrupt displays vector since we need it for the buffer assignments
        // Right now, we can't rearrange displays w/o affecting the match of active to buffer

        uint primary_buffer_ix = 0;
        const int UNBOUND_BUFFER = -1;
        std::map<uint, int> displays_to_buffers;
        std::vector<DisplayBuffers> buffers;

        uint nextIx(uint ix, uint size) const
        {
            return (ix + 1) % size;
        }

        uint prevIx(uint ix, uint size) const
        {
            return (ix == 0) ? (size - 1) : (ix - 1);
        }

        uint nextActiveDisplayIx() const
        {
            if (active_displays.size() == 0) { return 0; }
            if (active_displays.size() == 1) { return primary_display_ix; }

            uint next_ix = nextIx(primary_display_ix, displays.size());
            while (!isDisplayActive(next_ix))
            {
                next_ix = nextIx(next_ix, displays.size());
            }
            
            return next_ix;
        }

        uint prevActiveDisplayIx() const
        {
            if (active_displays.size() == 0) { return 0; }
            if (active_displays.size() == 1) { return primary_display_ix; }

            uint prev_ix = prevIx(primary_display_ix, displays.size());
            while (!isDisplayActive(prev_ix))
            {
                prev_ix = prevIx(prev_ix, displays.size());
            }
            
            return prev_ix;
        }

        bool isDisplayBoundToBuffer(uint ix)
        {
            return displays_to_buffers[ix] != UNBOUND_BUFFER;
        }

        void bindDisplayToBufferIx(uint disp_ix, int buffer_ix)
        {
            displays_to_buffers[disp_ix] = buffer_ix;
        }

        void unbindDisplayFromBufferIx(uint disp_ix)
        {
            displays_to_buffers[disp_ix] = UNBOUND_BUFFER;
        }

        inline int getBufferIxForDisplay(uint ix)
        {
            return displays_to_buffers[ix];
        }

        DisplayBuffers &getBuffer(uint ix)
        {
            return buffers.at(ix);
        }

        // uint numBuffersUsed() const
        // {
        //     return active_buffers.size();
        // }

        // bool isBufferAvailable() const
        // {
        //     return numBuffersUsed() < max_buffers;
        // }
    };

} // viewpoint_interface

#endif // __DISPLAYS_HPP__