// Thin adapter around the netrace trace reader to feed packets to BookSim.
#ifndef _NETRACE_ADAPTER_HPP_
#define _NETRACE_ADAPTER_HPP_

#include <deque>
#include <string>
#include <vector>

extern "C" {
#include "netrace.h"
}

struct NetracePacket {
  nt_packet_t *packet;
  long long cycle;
};

class NetraceAdapter {
public:
  NetraceAdapter(const std::string &filename, int nodes, bool ignore_deps,
                 int region, int scale);
  ~NetraceAdapter();

  void AdvanceTo(long long time);
  bool Ready(int src, long long time) const;
  NetracePacket PopReady(int src, long long time);
  bool Done() const;

  // Caller clears dependencies on ejection to unblock dependents.
  void OnEject(nt_packet_t *pkt);

  int nodes() const { return _nodes; }

private:
  void _MaybePromoteWaiting();
  void _Close();
  void _FreeQueue(std::vector<std::deque<NetracePacket>> &queues);

  nt_context_t *_ctx;
  nt_packet_t *_next_packet;
  int _nodes;
  bool _ignore_deps;
  int _scale;

  std::vector<std::deque<NetracePacket>> _ready;
  std::vector<std::deque<NetracePacket>> _waiting;
};

#endif
