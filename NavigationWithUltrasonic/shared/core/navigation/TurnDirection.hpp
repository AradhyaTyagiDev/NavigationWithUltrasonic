/*
Defines:
- left/right preference
- escape direction
- alternating behavior
 */

#pragma once

/*
| Concept   | Responsibility |
| --------- | -------------- |
| direction | Left / Right   |
| magnitude | turn intensity |
| geometry  | motion planner |
*/
enum class TurnDirection
{
    None,
    Left,
    Right,
    Random
};