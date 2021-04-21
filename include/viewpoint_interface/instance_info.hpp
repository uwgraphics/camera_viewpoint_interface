#ifndef __INSTANCE_INFO_HPP__
#define __INSTANCE_INFO_HPP__

namespace viewpoint_interface
{

enum class SelectionState
{
    None,
    Hovered,
    Selected
};

struct InstanceState
{
    SelectionState selection_;

    InstanceState() : selection_(SelectionState::None) {}
};

} // viewpoint_interface

#endif // __INSTANCE_INFO_HPP__