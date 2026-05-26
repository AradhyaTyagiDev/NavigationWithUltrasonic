// high-level movement intent

#pragma once

enum class NavigationAction
{
    None,
    Stop,
    MoveForward,
    MoveBackward,
    RotateLeft,
    RotateRight,
    CurveLeft,
    CurveRight
};