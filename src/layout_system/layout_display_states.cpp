#ifndef __LAYOUT_DISPLAY_STATES_HPP__
#define __LAYOUT_DISPLAY_STATES_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

Layout::LayoutDisplayStates::LayoutDisplayStates(DisplayManager &displays) : num_active_displays_(0), active_limit_(0),
        num_primary_(0), num_secondary_(0)
{
    for (int i(0); i < displays.getNumTotalDisplays(); ++i) {
        uint id(displays.getDisplayId(i));
        activateDisplay(id);
    }

    display_cache_.reverseCache();
}

uint Layout::LayoutDisplayStates::size() const { return states_.size(); }
bool Layout::LayoutDisplayStates::empty() const { return size() == 0; }
uint Layout::LayoutDisplayStates::getNumActiveDisplays() const { return num_active_displays_; }
bool Layout::LayoutDisplayStates::noDisplaysActive() const { return getNumActiveDisplays() == 0; }
uint Layout::LayoutDisplayStates::setActiveLimit(uint limit) { active_limit_ = limit; }

uint Layout::LayoutDisplayStates::setNumDisplaysForRole(int num, LayoutDisplayRole role)
{
    uint role_count(0);
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            num_primary_ = num;
            role_count = display_ring_.getNumPrimaryDisplays();
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            num_secondary_ = num;
            role_count = display_ring_.getNumSecondaryDisplays();
        }   break;
    }

    // -1 indicates all displays have the role--this is the default
    if (num == -1) {
        num = size();
    }

    // If too many displays have a specific role, reset them and set the role
    // back for num displays
    if (role_count > num) {
        display_ring_.unsetAllForRole(role);
        role_count = 0;
    }

    auto it(display_ring_.loopStart());
    while (role_count < num && it != display_ring_.loopEnd()) {
        if (!display_ring_.isDisplayRole(*it, role)) {
            display_ring_.setDisplayRole(*it, role);
            ++role_count;
        }

        ++it;
    }
}

std::map<uint, bool>::const_iterator Layout::LayoutDisplayStates::loopStart() const { return states_.begin(); }
std::map<uint, bool>::const_iterator Layout::LayoutDisplayStates::loopEnd() const { return states_.end(); }
bool Layout::LayoutDisplayStates::isDisplayActive(uint id) { return states_[id]; }

uint Layout::LayoutDisplayStates::getOldestActiveDisplay()
{
    int last_active_id;
    auto it(display_cache_.loopStart());
    for (; it != display_cache_.loopEnd(); ++it) {
        uint cur_id(*it);
        if (states_[cur_id]) {
            last_active_id = cur_id;
        }
    }

    return last_active_id;
}

void Layout::LayoutDisplayStates::activateDisplay(uint id) 
{ 
    states_[id] = true; 
    ++num_active_displays_;
    display_cache_.touchDisplay(id);
    display_ring_.pushDisplay(id);

    if (active_limit_ > 0 && num_active_displays_ > active_limit_) {
        deactivateDisplay(getOldestActiveDisplay());
    }
}

void Layout::LayoutDisplayStates::deactivateDisplay(uint id)
{
    uint num_active(getNumActiveDisplays());
    uint prim_limit(getPrimaryLimitNum());
    uint sec_limit(getSecondaryLimitNum());
    if ((num_active == prim_limit && prim_limit > 0) ||
            (num_active == sec_limit && sec_limit > 0)) {
        return;
    }

    if (getNumActiveDisplays() == 1) {
        return;
    }

    states_[id] = false;
    --num_active_displays_;
    display_cache_.touchDisplay(id);
    display_ring_.removeDisplay(id);
}

uint Layout::LayoutDisplayStates::getDisplayIxById(uint id)
{
    uint ix(0);
    auto it(states_.begin());
    for (; it != states_.end(); ++it, ++ix) {
        uint cur_id(it->first);
        if (cur_id == id) {
            break;
        }
    }

    return ix;
}

bool Layout::LayoutDisplayStates::isDisplayIxActive(uint ix)
{
    uint cur_ix(0);
    auto it(states_.begin());
    for (; it != states_.end(); ++it, ++cur_ix) {
        bool active(it->second);
        if (ix == cur_ix) {
            if (active) {
                return true;
            }
            else {
                break;
            }
        }
    }

    return false;
}

uint Layout::LayoutDisplayStates::getNextActiveDisplayIx(int ix)
{
    if (num_active_displays_ == 0) { return 0; }

    uint next_ix(nextIx(ix, states_.size()));
    while (!isDisplayIxActive(next_ix))
    {
        next_ix = nextIx(next_ix, states_.size());
    }
    
    return next_ix;
}

uint Layout::LayoutDisplayStates::getPrevActiveDisplayIx(int ix)
{
    if (num_active_displays_ == 0) { return 0; }

    uint prev_ix = prevIx(ix, states_.size());
    while (!isDisplayIxActive(prev_ix))
    {
        prev_ix = prevIx(prev_ix, states_.size());
    }
    
    return prev_ix;
}

uint Layout::LayoutDisplayStates::getCacheFront() { return display_cache_.getFront(); }
uint Layout::LayoutDisplayStates::getCacheBack() { return display_cache_.getBack(); }


// --- Ring Functions ---

Layout::DisplayRing& Layout::LayoutDisplayStates::getDisplayRing()
{
    return display_ring_;
}

void Layout::LayoutDisplayStates::setDisplayRole(uint id, LayoutDisplayRole role)
{
    display_ring_.setDisplayRole(id, role);
}

void Layout::LayoutDisplayStates::toNextDisplay(LayoutDisplayRole role)
{
    display_ring_.toNextDisplay(role);
}

void Layout::LayoutDisplayStates::toPrevDisplay(LayoutDisplayRole role)
{
    display_ring_.toPrevDisplay(role);
}

void Layout::LayoutDisplayStates::setActiveFrameByIndex(uint ix)
{
    if (ix >= getNumActiveDisplays()) {
        ix = 0;
    }

    display_ring_.setActiveFrameByIndex(ix); 
}

void Layout::LayoutDisplayStates::setActiveFrameById(uint id)
{
    display_ring_.setActiveFrameById(id);
}

uint Layout::LayoutDisplayStates::getActiveFrameDisplayId() const { return display_ring_.getActiveFrameDisplayId(); }

void Layout::LayoutDisplayStates::toNextActiveFrame() { display_ring_.toNextActiveFrame(); }
void Layout::LayoutDisplayStates::toPrevActiveFrame() { display_ring_.toPrevActiveFrame(); }

void Layout::LayoutDisplayStates::handleActiveFrameDirectionInput(LayoutCommand command)
{
    int cur_ix(display_ring_.getActiveFrameIndex());

    int factor;
    switch (command)
    {
        case LayoutCommand::ACTIVE_FRAME_UP:
        {
            factor = -1;

            if (cur_ix % 2 == 0) {
                factor = 0;
            }
        }   break;
        
        case LayoutCommand::ACTIVE_FRAME_DOWN:
        {
            factor = 1;

            if (cur_ix % 2 == 1) {
                factor = 0;
            }

            if (cur_ix == display_ring_.getNumPrimaryDisplays()-1) {
                factor = -1;
            }

        }   break;
        
        case LayoutCommand::ACTIVE_FRAME_LEFT:
        {
            factor = -2;
        }   break;
        
        case LayoutCommand::ACTIVE_FRAME_RIGHT:
        {
            factor = 2;
        }   break;
    }

    if (factor == 0) {
        return;
    }

    int ix_to_set(cur_ix + factor);
    if (ix_to_set < display_ring_.getNumPrimaryDisplays() && ix_to_set >= 0) {
        display_ring_.setActiveFrameByIndex(ix_to_set);
    }
}

void Layout::LayoutDisplayStates::addImageResponseForId(uint display_id, uint gl_id)
{ 
    display_ring_.addImageResponseForId(display_id, gl_id); 
}

uint Layout::LayoutDisplayStates::getImageIdForDisplayId(uint id) const
{
    display_ring_.getImageIdForDisplayId(id);
}


// --- Private Functions ---

uint Layout::LayoutDisplayStates::nextIx(uint ix, uint size) const
{
    return (ix + 1) % size;
}

uint Layout::LayoutDisplayStates::prevIx(uint ix, uint size) const
{
    return (ix == 0) ? (size - 1) : (ix - 1);
}


} // viewpoint_interface

#endif // __LAYOUT_DISPLAY_STATES_HPP__