
/*
Conservative Mode
    - larger danger distances
    - smoother EMA
    - slower motion
    - more cautious
Aggressive Mode
    - smaller safety margins
    - faster EMA
    - higher speeds
    - faster reactions

*/
enum class RobotBehaviorMode
{
    Conservative,
    Balanced,
    Aggressive
};