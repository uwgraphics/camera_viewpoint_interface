#ifndef __DISPLAY_RING_HPP__
#define __DISPLAY_RING_HPP__

#include <vector>

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

Layout::DisplayRing::DisplayRing() : active_frame_(0) {}

void Layout::DisplayRing::pushDisplay(uint id) { ring_.emplace_back(id); }

void Layout::DisplayRing::removeDisplay(uint id)
{
    if (isPrimaryDisplay(id)) {
        uint next_prim(getNextIdWithoutRole(id, LayoutDisplayRole::Primary));
        setPrimaryDisplay(next_prim);
    }

    if (isSecondaryDisplay(id)) {
        uint next_sec(getNextIdWithoutRole(id, LayoutDisplayRole::Secondary));
        setSecondaryDisplay(next_sec);
    }

    unsetRoles(id);

    uint ix(getIndexForDisplayId(id));
    ring_.erase(ring_.begin() + ix);

    if (getActiveFrameIndex() >= ring_.size()) {
        setActiveFrameByIndex(0);
    }
}

void Layout::DisplayRing::swapDisplays(uint id, float delta)
{
        int first_ix(getIndexForDisplayId(id));
        if (first_ix == -1) {
            return;
        }

        int second_ix(-1);
        if (delta < 0 && first_ix > 0) {
            second_ix = first_ix-1;
        }
        else if (delta > 0 && first_ix < ring_.size()-1) {
            second_ix = first_ix+1;
        }
        else {
            return;
        }

        uint temp = ring_[first_ix];
        ring_[first_ix] = ring_[second_ix];
        ring_[second_ix] = temp;
}

void Layout::DisplayRing::toNextDisplay(LayoutDisplayRole role)
{
    if (ring_.size() < 2) {
        return;
    }

    if (getNumForRole(role) == 0) {
        return;
    }

    uint cur_frame_id(getActiveFrameDisplayId());
    if (!isDisplayRole(cur_frame_id, role)) {
        auto role_vec(getDisplayRoleList(role));

        if (role_vec.size() > 1) {
            return;
        }
        cur_frame_id = role_vec.at(0);
    }

    uint next_id(getNextIdWithoutRole(cur_frame_id, role));
    if (cur_frame_id != next_id) {
        unsetDisplayRole(cur_frame_id, role);
        setDisplayRole(next_id, role);
        setActiveFrameById(next_id);
    }
}

void Layout::DisplayRing::toPrevDisplay(LayoutDisplayRole role)
{
    if (ring_.size() < 2) {
        return;
    }

    if (getNumForRole(role) == 0) {
        return;
    }

    uint cur_frame_id(getActiveFrameDisplayId());
    if (!isDisplayRole(cur_frame_id, role)) {
        auto role_vec(getDisplayRoleList(role));

        if (role_vec.size() > 1) {
            return;
        }
        cur_frame_id = role_vec.at(0);
    }

    uint prev_id(getPrevIdWithoutRole(cur_frame_id, role));
    if (cur_frame_id != prev_id) {
        unsetDisplayRole(cur_frame_id, role);
        setDisplayRole(prev_id, role);
        setActiveFrameById(prev_id);
    }
}

void Layout::DisplayRing::setDisplayRole(uint id, LayoutDisplayRole role)
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            setPrimaryDisplay(id);
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            setSecondaryDisplay(id);
        }   break;
    }
}

void Layout::DisplayRing::unsetDisplayRole(uint id, LayoutDisplayRole role)
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            unsetPrimaryDisplay(id);
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            unsetSecondaryDisplay(id);
        }   break;
    }
}

void Layout::DisplayRing::unsetRoles(uint id)
{
    primary_displays_[id] = false;
    secondary_displays_[id] = false;
}

void Layout::DisplayRing::unsetAllForRole(LayoutDisplayRole role)
{
    std::map<uint, bool>::iterator it;
    std::map<uint, bool>::iterator end_it;    
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            it = primary_displays_.begin();
            end_it = primary_displays_.end();
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            it = secondary_displays_.begin();
            end_it = secondary_displays_.end();
        }   break;
    }

    for (; it != end_it; ++it) {
        it->second = false;
    }
}

bool Layout::DisplayRing::isPrimaryDisplay(uint id) const
{
    if (primary_displays_.find(id) == primary_displays_.end()) {
        return false;
    }

    return primary_displays_.at(id); 
}

bool Layout::DisplayRing::isSecondaryDisplay(uint id) const
{
    if (secondary_displays_.find(id) == secondary_displays_.end()) {
        return false;
    }

    return secondary_displays_.at(id); 
}

bool Layout::DisplayRing::isDisplayRole(uint id, LayoutDisplayRole role) const
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            return isPrimaryDisplay(id);
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            return isSecondaryDisplay(id);
        }   break;
    }
}

std::vector<uint> Layout::DisplayRing::getDisplayRoleList(LayoutDisplayRole role) const
{
    std::vector<uint> list;
   
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            for (uint display_id : ring_) {
                if (isPrimaryDisplay(display_id)) {
                    list.emplace_back(display_id);
                }
            }
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            for (uint display_id : ring_) {
                if (isSecondaryDisplay(display_id)) {
                    list.emplace_back(display_id);
                }
            }
        }   break;
    }

    return list;
}

uint Layout::DisplayRing::getNumPrimaryDisplays() const
{
    uint count(0);
    auto it(primary_displays_.begin());
    for (; it != primary_displays_.end(); ++it) {
        if (it->second) {
            ++count;
        }
    }

    return count;
}

uint Layout::DisplayRing::getNumSecondaryDisplays() const
{
    uint count(0);
    auto it(secondary_displays_.begin());
    for (; it != secondary_displays_.end(); ++it) {
        if (it->second) {
            ++count;
        }
    }

    return count;
}

uint Layout::DisplayRing::getNumForRole(LayoutDisplayRole role) const
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            return getNumPrimaryDisplays();
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            return getNumSecondaryDisplays();
        }   break;
    }
}

uint Layout::DisplayRing::getDisplayIdByIx(uint ix) const
{
    ring_.at(ix);
}

void Layout::DisplayRing::setActiveFrameByIndex(uint ix)
{
    if (ix > ring_.size()) {
        return;
    }

    active_frame_ = ix;
}

void Layout::DisplayRing::setActiveFrameById(uint id)
{
    auto prim_vec(getDisplayRoleList(LayoutDisplayRole::Primary));
    for (int i(0); i < prim_vec.size(); ++i) {
        uint cur_id(prim_vec.at(i));
        if (cur_id == id) {
            setActiveFrameByIndex(i);
            return;
        }
    }
}

uint Layout::DisplayRing::getActiveFrameIndex() const { return active_frame_; }

uint Layout::DisplayRing::getActiveFrameDisplayId() const
{
    if (ring_.empty()) {
        return 0;
    }

    return getDisplayRoleList(LayoutDisplayRole::Primary).at(active_frame_);
}

void Layout::DisplayRing::toNextActiveFrame()
{
    ++active_frame_;
    if (active_frame_ == getNumPrimaryDisplays()) {
        active_frame_ = 0;
    }
}

void Layout::DisplayRing::toPrevActiveFrame()
{
    if (active_frame_ == 0) {
        active_frame_ = getNumPrimaryDisplays()-1;
    }
    else {
        --active_frame_;
    }
}

// TODO: Should these two functions below be part of the DisplayManager instead?

void Layout::DisplayRing::addImageResponseForId(uint display_id, uint gl_id)
{
    // NOTE: It may be worth doing some more validation to prune id's for displays
    // that are inactive, but the images will generally update fast enough that
    // it doesn't matter if an old image hangs around longer than necessary
    gl_ids_[display_id] = gl_id;
}

uint Layout::DisplayRing::getImageIdForDisplayId(uint id) const
{
    auto entry(gl_ids_.find(id));
    if (entry != gl_ids_.end()) {
        return entry->second;
    }

    return 0;
}


void Layout::DisplayRing::setPrimaryDisplay(uint id) { primary_displays_[id] = true; }
void Layout::DisplayRing::setSecondaryDisplay(uint id) { secondary_displays_[id] = true; }
void Layout::DisplayRing::unsetPrimaryDisplay(uint id) { primary_displays_[id] = false; }
void Layout::DisplayRing::unsetSecondaryDisplay(uint id) { secondary_displays_[id] = false; }

uint Layout::DisplayRing::getNextIdWithoutRole(uint start_id, LayoutDisplayRole role)
{
    uint cur_ix(getIndexForDisplayId(start_id));

    // Loop from cur_ix through end of ring
    for (int i(cur_ix+1); i < ring_.size(); ++i) {
        uint cur_id(ring_.at(i));
        switch (role)
        {
            case LayoutDisplayRole::Primary:
            {
                if (!primary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                if (!secondary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;
        }
    }

    // If valid ID not found, loop back to beginning of ring
    for (int i(0); i < cur_ix; ++i) {
        uint cur_id(ring_.at(i));
        switch (role)
        {
            case LayoutDisplayRole::Primary:
            {
                if (!primary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                if (!secondary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;
        }
    }

    return start_id;
}

uint Layout::DisplayRing::getPrevIdWithoutRole(uint start_id, LayoutDisplayRole role)
{
    uint cur_ix(getIndexForDisplayId(start_id));

    // Loop back from cur_ix through beginning of ring
    for (int i(cur_ix-1); i >= 0; --i) {
        uint cur_id(ring_.at(i));
        switch (role)
        {
            case LayoutDisplayRole::Primary:
            {
                if (!primary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                if (!secondary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;
        }
    }

    // If valid ID not found, loop back from end of ring
    for (int i(ring_.size()-1); i > cur_ix; --i) {
        uint cur_id(ring_.at(i));
        switch (role)
        {
            case LayoutDisplayRole::Primary:
            {
                if (!primary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                if (!secondary_displays_[cur_id]) {
                    return cur_id;
                }
            }   break;
        }
    }

    return start_id;
}

uint Layout::DisplayRing::getIndexForDisplayId(uint id)
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


} // viewpoint_interface

#endif // __DISPLAY_RING_HPP__