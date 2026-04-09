# DM broadcast downstream (topics), re-published hop-by-hop.
uint64 msg_id
string src_agent
string dst            # reuse as "group" later for predicated multicast
string path_json
string payload_json
uint8  ttl
uint8  priority

