// Thin adapter around the netrace trace reader to feed packets to BookSim.
#include "netrace_adapter.hpp"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>

NetraceAdapter::NetraceAdapter(const std::string &filename, int nodes,
                               bool ignore_deps, int region, int scale)
    : _ctx(nullptr), _next_packet(nullptr), _nodes(nodes),
      _ignore_deps(ignore_deps), _scale(scale ? scale : 1),
      _ready(nodes), _waiting(nodes) {
  _ctx = static_cast<nt_context_t *>(calloc(1, sizeof(nt_context_t)));
  assert(_ctx);
  nt_open_trfile(_ctx, filename.c_str());
  if (_ignore_deps) {
    nt_disable_dependencies(_ctx);
  }
  nt_header_t *header = nt_get_trheader(_ctx);
  if (header && header->num_nodes != _nodes) {
    std::cerr << "warning: netrace nodes (" << static_cast<int>(header->num_nodes)
              << ") do not match config nodes (" << _nodes << ")" << std::endl;
  }
  if (region > 0 && header && region < static_cast<int>(header->num_regions)) {
    nt_seek_region(_ctx, &header->regions[region]);
  }
  _next_packet = nt_read_packet(_ctx);
}

NetraceAdapter::~NetraceAdapter() {
  _FreeQueue(_ready);
  _FreeQueue(_waiting);
  if (_next_packet) {
    nt_clear_dependencies_free_packet(_ctx, _next_packet);
    _next_packet = nullptr;
  }
  _Close();
}

void NetraceAdapter::_FreeQueue(std::vector<std::deque<NetracePacket>> &queues) {
  for (auto &q : queues) {
    while (!q.empty()) {
      nt_packet_t *pkt = q.front().packet;
      q.pop_front();
      if (pkt) {
        nt_clear_dependencies_free_packet(_ctx, pkt);
      }
    }
  }
}

void NetraceAdapter::_Close() {
  if (_ctx) {
    nt_close_trfile(_ctx);
    free(_ctx);
    _ctx = nullptr;
  }
}

void NetraceAdapter::_MaybePromoteWaiting() {
  if (_ignore_deps) {
    return;
  }
  for (int src = 0; src < _nodes; ++src) {
    auto &q = _waiting[src];
    for (auto it = q.begin(); it != q.end();) {
      if (nt_dependencies_cleared(_ctx, it->packet)) {
        _ready[src].push_back(*it);
        it = q.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void NetraceAdapter::AdvanceTo(long long time) {
  while (_next_packet &&
         static_cast<long long>(_next_packet->cycle / _scale) <= time) {
    nt_packet_t *pkt = _next_packet;
    long long inject_cycle =
        static_cast<long long>(_next_packet->cycle / _scale);
    _next_packet = nt_read_packet(_ctx);
    if ((pkt->src >= static_cast<unsigned>(_nodes)) ||
        (pkt->dst >= static_cast<unsigned>(_nodes))) {
      std::cerr << "warning: skipping netrace packet with src/dst out of range: "
                << static_cast<int>(pkt->src) << "->"
                << static_cast<int>(pkt->dst) << " (nodes=" << _nodes << ")"
                << std::endl;
      nt_clear_dependencies_free_packet(_ctx, pkt);
      continue;
    }
    NetracePacket wrapped{pkt, inject_cycle};
    if (_ignore_deps || nt_dependencies_cleared(_ctx, pkt)) {
      _ready[pkt->src].push_back(wrapped);
    } else {
      _waiting[pkt->src].push_back(wrapped);
    }
  }
  _MaybePromoteWaiting();
}

bool NetraceAdapter::Ready(int src, long long time) const {
  if (src < 0 || src >= _nodes) return false;
  if (_ready[src].empty()) return false;
  return _ready[src].front().cycle <= time;
}

NetracePacket NetraceAdapter::PopReady(int src, long long time) {
  if (!Ready(src, time)) {
    return {nullptr, 0};
  }
  NetracePacket pkt = _ready[src].front();
  _ready[src].pop_front();
  return pkt;
}

bool NetraceAdapter::Done() const {
  bool no_ready = std::all_of(_ready.begin(), _ready.end(),
                              [](const std::deque<NetracePacket> &q) {
                                return q.empty();
                              });
  bool no_waiting = std::all_of(_waiting.begin(), _waiting.end(),
                                [](const std::deque<NetracePacket> &q) {
                                  return q.empty();
                                });
  return !_next_packet && no_ready && no_waiting;
}

void NetraceAdapter::OnEject(nt_packet_t *pkt) {
  if (!pkt) return;
  nt_clear_dependencies_free_packet(_ctx, pkt);
  _MaybePromoteWaiting();
}
