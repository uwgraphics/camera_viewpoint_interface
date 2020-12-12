#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <string>

namespace viewpoint_interface
{

    enum LayoutType
    {
        NONE = -1,
        MOVIE,
        GRID,
        SPLIT,
        TWINNED,
        PIP,
        CAROUSEL
    };
    
    // TODO: Consider adding a layout manager to cache parameters for
    // layouts that have been activated previously in this session

    class Layout
    {
    public:        
        const std::string& getLayoutName(LayoutType type) const
        {
            return layout_names[(uint)type];
        }

    protected:
        Layout(LayoutType type) : layout_type(type) {}

    private:
        static const uint num_layout_types = 6;
        const std::string layout_names[num_layout_types] = {
            "Movie", "Grid", "Split Screen", "Twinned",
            "Picture-in-Picture", "Carousel"
        };

        LayoutType layout_type;

    };
    

    class NoneLayout : public Layout
    {
    public:
        NoneLayout() : Layout(LayoutType::NONE) {}

        const std::string getLayoutName(LayoutType type) const
        {
            return "Invalid layout";
        }

    };

    class MovieLayout : public Layout
    {
    public:
        MovieLayout() : Layout(LayoutType::MOVIE) {}
    };

    class GridLayout : public Layout
    {
    public:
        GridLayout() : Layout(LayoutType::GRID) {}
    };

    class SplitLayout : public Layout
    {
    public:
        SplitLayout() : Layout(LayoutType::SPLIT) {}
    };

    class TwinnedLayout : public Layout
    {
    public:
        TwinnedLayout() : Layout(LayoutType::TWINNED) {}
    };

    class PipLayout : public Layout
    {
    public:
        PipLayout() : Layout(LayoutType::PIP) {}
    };

    class CarouselLayout : public Layout
    {
    public:
        CarouselLayout() : Layout(LayoutType::CAROUSEL) {}
    };


} // viewpoint_interface

#endif // __LAYOUT_HPP__