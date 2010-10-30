INCLUDE_DIRECTORIES( "." ".." ) 

SET(AGENT_SRCS
   libagent/libagent.cpp
   libagent/libagent.hpp
   libagent/AgentData.hpp
   utils/options.cpp
)

configure_src_module(agent ${AGENT_SRCS})

use_lib(agent ALCOMMON)

