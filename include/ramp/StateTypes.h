#ifndef RASP_STATE_TYPES_H_
#define RASP_STATE_TYPES_H_

namespace RAMP
{

enum class StateType
{
    FREE,
    FORB, // obstacle
    RISK, // risk region
    SAFE  // safe region
};

}

#endif // RASP_STATE_TYPES_H_
