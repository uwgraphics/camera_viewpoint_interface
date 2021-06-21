#ifndef __DISPLAY_RING_HPP__
#define __DISPLAY_RING_HPP__

#include <vector>

#include "display.hpp"

namespace viewpoint_interface
{

class DisplayRing
{
public:
    bool empty() const
    {
        return ring_.empty();
    }

    uint size() const
    {
        ring_.size();
    }

    uint getDisplayIdAt(uint ix) const
    {
        if (ix >= ring_.size()) {
            return 0;
        }

        return ring_.at(ix);
    }

    uint getTotalDisplaysNumber() const
    {
        return ring_.size() + inactive_displays_.size();
    }

    void addDisplayId(uint id)
    {
        ring_.emplace_back(id);
    }

    bool isDisplayInRing(uint id) const
    {
        for (uint disp_id : ring_) {
            if (disp_id == id) {
                return true;
            }
        }

        return false;
    }

    void markDisplayActive(uint id)
    {
        if (isDisplayInactive(id) && !isDisplayInRing(id)) {
            addDisplayId(id);
            uint inactive_ix(getIndexForInactiveDisplayId(id));
            inactive_displays_.erase(std::next(inactive_displays_.begin(), inactive_ix));
        }
    }

    void markDisplayInactive(uint id)
    {
        if (isDisplayInRing(id) && !isDisplayInactive(id)) {
            inactive_displays_.emplace_back(id);
            uint active_ix(getIndexForDisplayId(id));
            ring_.erase(std::next(ring_.begin(), active_ix));

            if (active_frame_ == ring_.size()) {
                // Handle inactivation of last display when it's the active frame
                active_frame_ = 0;
            }
        }
    }

    void switchActiveDisplay(uint new_id)
    {
        uint disp_ix(getIndexForDisplayId(new_id));
        if (disp_ix != -1) { // Display already active in ring
            active_frame_ = disp_ix; // Just move along the ring to the right display
        }
        else {
            if (isDisplayInactive(new_id)) { // When display previously inactivated
                markDisplayActive(new_id); // Activate
                active_frame_ = size()-1; // And switch frame to this display
            }
            else { // When display has never been in ring
                ring_[active_frame_] = new_id; // Swap out active display
            }
        }
    }

    void switchDisplayAtIx(uint ix, uint new_id)
    {
        if (ix >= ring_.size()) {
            return;
        }

        if (isDisplayInactive(new_id)) {
            uint inactive_ix(getIndexForInactiveDisplayId(new_id));
            inactive_displays_.erase(std::next(inactive_displays_.begin(), inactive_ix));
        }

        ring_[ix] = new_id;
    }

    void swapDisplayPositionsInRing(uint id, float delta)
    {
        uint first_ix(getIndexForDisplayId(id));
        if (first_ix == -1) {
            return;
        }

        uint second_ix;
        if (delta < 0 && first_ix > 0) {
            second_ix = first_ix-1;
        }
        else if (delta > 0 && first_ix < size()-1) {
            second_ix = first_ix+1;
        }
        else {
            return;
        }

        auto ring_first(std::next(ring_.begin(), first_ix));
        auto ring_second(std::next(ring_.begin(), second_ix));
        std::iter_swap(ring_first, ring_second);
    }

    uint getActiveFrameDisplayId() const
    {
        if (ring_.empty()) {
            return 0;
        }

        return ring_[active_frame_];
    }

    uint getNextActiveFrameDisplayId() const
    {
        if (ring_.empty()) {
            return 0;
        }
        else if (active_frame_ == size() - 1) {
            return ring_[0];
        }

        return ring_[active_frame_ + 1];
    }

    void setActiveFrameByIndex(uint ix)
    {
        active_frame_ = ix;
    }

    void toNextActiveFrame()
    {
        ++active_frame_;
        if (active_frame_ == size()) {
            active_frame_ = 0;
        }
    }
    
    void toPrevActiveFrame()
    {
        if (active_frame_ == 0) {
            active_frame_ = size()-1;
        }
        else {
            --active_frame_;
        }
    }

    void addImageResponseForId(uint display_id, uint gl_id)
    {
        // NOTE: It may be worth doing some more validation to prune id's for displays
        // that are inactive, but the images will generally update fast enough that
        // it doesn't matter if an old image hangs around longer than necessary
        gl_ids_[display_id] = gl_id;
    }

    uint getImageIdForDisplayId(uint id) const
    {
        auto entry(gl_ids_.find(id));
        if (entry != gl_ids_.end()) {
            return entry->second;
        }

        return 0;
    }

private:
    std::vector<uint> ring_;
    std::vector<uint> inactive_displays_;
    uint active_frame_ = 0; // Active frame points to an index position within ring_
    std::map<uint, uint> gl_ids_; // Stores OpenGL ID for displays in ring

    int getIndexForDisplayId(uint id)
    {
        uint index(0);
        for (uint disp_id : ring_) {
            if (disp_id == id) {
                return index;
            }
            ++index;
        }

        return -1;
    }

    int getIndexForInactiveDisplayId(uint id)
    {
        uint index(0);
        for (uint disp_id : inactive_displays_) {
            if (disp_id == id) {
                return index;
            }
            ++index;
        }

        return -1;
    }

    bool isDisplayInactive(uint id) const
    {
        for (uint disp_id : inactive_displays_) {
            if (disp_id == id) {
                return true;
            }
        }

        return false;
    }
};

} // viewpoint_interface

#endif // __DISPLAY_RING_HPP__