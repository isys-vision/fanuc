#ifndef MIKADO_TYPES_H
#define MIKADO_TYPES_H

#ifndef FLATHEADERS
#include "simple_message/simple_serialize.h"
#include "simple_message/byte_array.h"
#include "simple_message/shared_types.h"
#include <unordered_set>
#else
#include "simple_serialize.h"
#include "byte_array.h"
#include "shared_types.h"
#endif


namespace industrial
{

namespace simple_message
{

namespace mikado_messages
{

  const int EoM_bytes = 0xFFFFFFFF; //end of message sequence

/**
 * \brief Enumeration of the Mikado message types
 */
namespace MikadoMessageTypes
{
  enum MikadoMessageType
  {
    MIKADO_ROBOT_STATUS_MSG          = 65007,
    MIKADO_CONNECTION_INFO_MSG       = 65008,
    MIKADO_DYNAMIC_JOINTS_MSG        = 65009,
    MIKADO_DYNAMIC_JOINTS_TRAJ_PT_MSG = 65010
  };
}
typedef MikadoMessageTypes::MikadoMessageType MikadoMessageType;

}
}
}


#endif