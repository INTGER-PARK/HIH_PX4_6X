uint64 timestamp                 # [us] sender time; freshness uses local uORB reception time
uint32 sequence                  # heartbeat sequence
uint32 reset_counter             # edge-triggered reset request
bool enable                      # enable request, not an ACTIVE command
