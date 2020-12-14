#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <string>

namespace viewpoint_interface
{

    class LayoutManager;

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
        LayoutType getLayoutType() const { return layout_type; } 

        const std::vector<std::string> getLayoutList() const 
        { 
            return layout_names; 
        }

        const std::string getLayoutName() const
        {
            if (layout_type == LayoutType::NONE) {
                return "Layouts Inactive";
            }

            return layout_names[(uint)layout_type];
        }


    protected:
        Layout(LayoutType type) : layout_type(type) {}

    private:
        const std::vector<std::string> layout_names = {
            "Movie", "Grid", "Split Screen", "Twinned",
            "Picture-in-Picture", "Carousel"
        };

        LayoutType layout_type;
    };
    

    class NoneLayout : public Layout
    {
    public:
        NoneLayout() : Layout(LayoutType::NONE) {}
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


    class LayoutManager
    {
    public:
        LayoutManager() : active_layout(new NoneLayout()) {}

        static LayoutType intToLayoutType(int ix) {
            if (ix >= 0 && ix < num_layout_types) {
                return (LayoutType)ix;
            }

            return LayoutType::NONE;
        }

        const std::vector<std::string> getLayoutList() const 
        { 
            return active_layout->getLayoutList(); 
        }

        std::shared_ptr<Layout> getActiveLayout() const
        {
            return active_layout;
        }

        bool isLayoutActive(LayoutType type) const
        {
            if (active_layout->getLayoutType() == type) {
                return true;
            }

            return false;
        }

        void activateLayout(LayoutType type)
        {
            // Already active
            if (active_layout->getLayoutType() == type) {
                return;
            }

            // We cache previously active layouts so that their params are not reset
            if (!isInCache(active_layout->getLayoutType())) {
                layouts_cache.push_back(active_layout);
            }

            if (isInCache(type)) {
                active_layout = getLayoutFromCache(type);
            }
            else {
                active_layout = newLayout(type);
            }
        }

    private:
        static const uint num_layout_types = 6;

        std::vector<std::shared_ptr<Layout>> layouts_cache;
        std::shared_ptr<Layout> active_layout;

        std::shared_ptr<Layout> newLayout(LayoutType type)
        {
            std::shared_ptr<Layout> layout;
            switch(type) {
                case LayoutType::MOVIE:
                {
                    layout = std::shared_ptr<Layout>(new MovieLayout());
                } break;

                case LayoutType::GRID:
                {
                    layout = std::shared_ptr<Layout>(new GridLayout());
                } break;

                case LayoutType::SPLIT:
                {
                    layout = std::shared_ptr<Layout>(new SplitLayout());
                } break;

                case LayoutType::TWINNED:
                {
                    layout = std::shared_ptr<Layout>(new TwinnedLayout());
                } break;

                case LayoutType::PIP:
                {
                    layout = std::shared_ptr<Layout>(new PipLayout());
                } break;

                case LayoutType::CAROUSEL:
                {
                    layout = std::shared_ptr<Layout>(new CarouselLayout());
                } break;

                default: 
                {
                    layout = std::shared_ptr<Layout>(new NoneLayout());
                } break;
            }
            
            return layout;
        }

        bool isInCache(LayoutType type) const
        {
            for (const std::shared_ptr<Layout> layout : layouts_cache) {
                if (layout->getLayoutType() == type) {
                    return true;
                }
            }

            return false;
        }

        std::shared_ptr<Layout> getLayoutFromCache(LayoutType type) const
        {
            for (const std::shared_ptr<Layout> layout : layouts_cache) {
                if (layout->getLayoutType() == type) {
                    return layout;
                }
            }

            return std::shared_ptr<Layout>(new NoneLayout());
        }
    };

} // viewpoint_interface

#endif // __LAYOUT_HPP__