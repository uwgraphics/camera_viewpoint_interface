#ifndef __DISPLAY_STATE_CACHE_HPP__
#define __DISPLAY_STATE_CACHE_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

void Layout::DisplayStateCache::touchDisplay(uint id)
{
    cache_.remove(id);
    cache_.emplace_front(id);
}

void Layout::DisplayStateCache::reverseCache() { cache_.reverse(); }

void Layout::DisplayStateCache::printCache()
{
    std::cout << "Cache: ";
    for (auto& elem : cache_) {
        std::cout << elem << " ";
    }

    std::cout << std::endl;
}


} // viewpoint_interface

#endif // __DISPLAY_STATE_CACHE_HPP__