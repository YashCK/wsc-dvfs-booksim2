// $Id$

/*
  Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimer.
  Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sstream>
#include <cmath>
#include <fstream>
#include <limits>
#include <cstdlib>
#include <ctime>

#include "booksim.hpp"
#include "booksim_config.hpp"
#include "trafficmanager.hpp"
#include "batchtrafficmanager.hpp"
#include "random_utils.hpp" 
#include "vc.hpp"
#include "packet_reply_info.hpp"
#include "policy/policy_factory.hpp"
#include <cmath>
#include "routers/iq_router.hpp"
#include <sys/stat.h>
#include <sys/types.h>

TrafficManager * TrafficManager::New(Configuration const & config,
				     vector<Network *> const & net)
{
    TrafficManager * result = NULL;
    string sim_type = config.GetStr("sim_type");
    if((sim_type == "latency") || (sim_type == "throughput")) {
        result = new TrafficManager(config, net);
    } else if(sim_type == "batch") {
        result = new BatchTrafficManager(config, net);
    } else {
        cerr << "Unknown simulation type: " << sim_type << endl;
    } 
    return result;
}

TrafficManager::TrafficManager( const Configuration &config, const vector<Network *> & net )
    : Module( 0, "traffic_manager" ), _net(net), _empty_network(false), _deadlock_timer(0), _reset_time(0), _drain_time(-1), _cur_id(0), _cur_pid(0), _time(0)
{

    _nodes = _net[0]->NumNodes( );
    _routers = _net[0]->NumRouters( );

    _vcs = config.GetInt("num_vcs");
    _subnets = config.GetInt("subnets");
 
    _subnet.resize(Flit::NUM_FLIT_TYPES);
    _subnet[Flit::READ_REQUEST] = config.GetInt("read_request_subnet");
    _subnet[Flit::READ_REPLY] = config.GetInt("read_reply_subnet");
    _subnet[Flit::WRITE_REQUEST] = config.GetInt("write_request_subnet");
    _subnet[Flit::WRITE_REPLY] = config.GetInt("write_reply_subnet");

    // ============ Message priorities ============ 

    string priority = config.GetStr( "priority" );

    if ( priority == "class" ) {
        _pri_type = class_based;
    } else if ( priority == "age" ) {
        _pri_type = age_based;
    } else if ( priority == "network_age" ) {
        _pri_type = network_age_based;
    } else if ( priority == "local_age" ) {
        _pri_type = local_age_based;
    } else if ( priority == "queue_length" ) {
        _pri_type = queue_length_based;
    } else if ( priority == "hop_count" ) {
        _pri_type = hop_count_based;
    } else if ( priority == "sequence" ) {
        _pri_type = sequence_based;
    } else if ( priority == "none" ) {
        _pri_type = none;
    } else {
        Error( "Unkown priority value: " + priority );
    }

    // ============ Routing ============ 

    string rf = config.GetStr("routing_function") + "_" + config.GetStr("topology");
    map<string, tRoutingFunction>::const_iterator rf_iter = gRoutingFunctionMap.find(rf);
    if(rf_iter == gRoutingFunctionMap.end()) {
        Error("Invalid routing function: " + rf);
    }
    _rf = rf_iter->second;
  
    _lookahead_routing = !config.GetInt("routing_delay");
    _noq = config.GetInt("noq");
    if(_noq) {
        if(!_lookahead_routing) {
            Error("NOQ requires lookahead routing to be enabled.");
        }
    }

    // ============ Traffic ============ 

    _classes = config.GetInt("classes");

    _use_read_write = config.GetIntArray("use_read_write");
    if(_use_read_write.empty()) {
        _use_read_write.push_back(config.GetInt("use_read_write"));
    }
    _use_read_write.resize(_classes, _use_read_write.back());

    _write_fraction = config.GetFloatArray("write_fraction");
    if(_write_fraction.empty()) {
        _write_fraction.push_back(config.GetFloat("write_fraction"));
    }
    _write_fraction.resize(_classes, _write_fraction.back());

    _read_request_size = config.GetIntArray("read_request_size");
    if(_read_request_size.empty()) {
        _read_request_size.push_back(config.GetInt("read_request_size"));
    }
    _read_request_size.resize(_classes, _read_request_size.back());

    _read_reply_size = config.GetIntArray("read_reply_size");
    if(_read_reply_size.empty()) {
        _read_reply_size.push_back(config.GetInt("read_reply_size"));
    }
    _read_reply_size.resize(_classes, _read_reply_size.back());

    _write_request_size = config.GetIntArray("write_request_size");
    if(_write_request_size.empty()) {
        _write_request_size.push_back(config.GetInt("write_request_size"));
    }
    _write_request_size.resize(_classes, _write_request_size.back());

    _write_reply_size = config.GetIntArray("write_reply_size");
    if(_write_reply_size.empty()) {
        _write_reply_size.push_back(config.GetInt("write_reply_size"));
    }
    _write_reply_size.resize(_classes, _write_reply_size.back());

    string packet_size_str = config.GetStr("packet_size");
    if(packet_size_str.empty()) {
        _packet_size.push_back(vector<int>(1, config.GetInt("packet_size")));
    } else {
        vector<string> packet_size_strings = tokenize_str(packet_size_str);
        for(size_t i = 0; i < packet_size_strings.size(); ++i) {
            _packet_size.push_back(tokenize_int(packet_size_strings[i]));
        }
    }
    _packet_size.resize(_classes, _packet_size.back());

    string packet_size_rate_str = config.GetStr("packet_size_rate");
    if(packet_size_rate_str.empty()) {
        int rate = config.GetInt("packet_size_rate");
        assert(rate >= 0);
        for(int c = 0; c < _classes; ++c) {
            int size = _packet_size[c].size();
            _packet_size_rate.push_back(vector<int>(size, rate));
            _packet_size_max_val.push_back(size * rate - 1);
        }
    } else {
        vector<string> packet_size_rate_strings = tokenize_str(packet_size_rate_str);
        packet_size_rate_strings.resize(_classes, packet_size_rate_strings.back());
        for(int c = 0; c < _classes; ++c) {
            vector<int> rates = tokenize_int(packet_size_rate_strings[c]);
            rates.resize(_packet_size[c].size(), rates.back());
            _packet_size_rate.push_back(rates);
            int size = rates.size();
            int max_val = -1;
            for(int i = 0; i < size; ++i) {
                int rate = rates[i];
                assert(rate >= 0);
                max_val += rate;
            }
            _packet_size_max_val.push_back(max_val);
        }
    }
  
    for(int c = 0; c < _classes; ++c) {
        if(_use_read_write[c]) {
            _packet_size[c] = 
                vector<int>(1, (_read_request_size[c] + _read_reply_size[c] +
                                _write_request_size[c] + _write_reply_size[c]) / 2);
            _packet_size_rate[c] = vector<int>(1, 1);
            _packet_size_max_val[c] = 0;
        }
    }

    _load = config.GetFloatArray("injection_rate"); 
    if(_load.empty()) {
        _load.push_back(config.GetFloat("injection_rate"));
    }
    _load.resize(_classes, _load.back());

    if(config.GetInt("injection_rate_uses_flits")) {
        for(int c = 0; c < _classes; ++c)
            _load[c] /= _GetAveragePacketSize(c);
    }

    _traffic = config.GetStrArray("traffic");
    _traffic.resize(_classes, _traffic.back());

    _traffic_pattern.resize(_classes);

    _class_priority = config.GetIntArray("class_priority"); 
    if(_class_priority.empty()) {
        _class_priority.push_back(config.GetInt("class_priority"));
    }
    _class_priority.resize(_classes, _class_priority.back());
    _class_cfg = ParseClassConfig(config, _classes);
    _policy_telemetry.class_latency_p99.assign(_classes, 0.0);

    _class_assigner = MakeClassAssigner(config, _classes);
    _priority_policy = MakePriorityPolicy(config);
    _dvfs_policy = MakeDVFSPolicy(config);
    _channel_width = config.GetInt("channel_width");
    _use_netrace = (config.GetInt("use_netrace") > 0);
    _netrace_class = config.GetInt("netrace_class");
    if(_netrace_class < 0) _netrace_class = 0;
    if(_netrace_class >= _classes) _netrace_class = _classes - 1;
    _netrace_use_addr_size = (config.GetInt("netrace_use_addr_size") > 0);
    _netrace_class_from_node_types =
        (config.GetInt("netrace_class_from_node_types") > 0);
    if(_use_netrace) {
        string netrace_file = config.GetStr("netrace_file");
        if(netrace_file.empty()) {
            Error("use_netrace set but netrace_file not provided.");
        }
        int netrace_region = config.GetInt("netrace_region");
        bool netrace_ignore_deps = (config.GetInt("netrace_ignore_deps") > 0);
        int netrace_scale = config.GetInt("netrace_scale");
        _netrace_adapter.reset(
            new NetraceAdapter(netrace_file, _nodes, netrace_ignore_deps,
                               netrace_region, netrace_scale));
    }
    _dvfs_epoch = config.GetInt("dvfs_epoch");
    _last_dvfs_epoch = 0;
    _power_telemetry.router_power.assign(_routers, 0.0);
    _dvfs_freqs = config.GetFloatArray("dvfs_freqs");
    _dvfs_voltages = config.GetFloatArray("dvfs_voltages");
    if(_dvfs_freqs.empty()) {
        _dvfs_freqs.push_back(1.0);
    }
    if(_dvfs_voltages.empty()) {
        _dvfs_voltages.push_back(1.0);
    }
    _dvfs_voltages.resize(_dvfs_freqs.size(), _dvfs_voltages.back());
    _power_dyn_base = config.GetFloat("power_dyn_base");
    _power_leak_base = config.GetFloat("power_leak_base");
    _use_orion = config.GetInt("use_orion") > 0;
    _output_dir = config.GetStr("output_dir");
    _run_name = config.GetStr("run_name");
    _latency_csv_name = config.GetStr("latency_csv");
    _stall_csv_name = config.GetStr("stall_csv");
    _throughput_csv_name = config.GetStr("throughput_csv");
    _summary_csv_name = config.GetStr("summary_csv");
    _power_cap = config.GetFloat("power_cap");
    _dvfs_log_interval = config.GetInt("dvfs_log_interval");
    if(_dvfs_log_interval <= 0) _dvfs_log_interval = _dvfs_epoch;
    _dvfs_log_last = 0;
    _dvfs_power_avg_sum = 0.0;
    _dvfs_power_avg_count = 0;
    _dvfs_log_out = NULL;
    _dvfs_log_name = config.GetStr("dvfs_log");
    auto ensure_dir = [](const string &path) {
        if(path.empty()) return;
        size_t pos = 0;
        while(true) {
            pos = path.find('/', pos + 1);
            string sub = path.substr(0, pos);
            if(sub.empty()) continue;
            mkdir(sub.c_str(), 0755);
            if(pos == string::npos) break;
        }
    };
    string base_dir = _output_dir.empty() ? "sims" : _output_dir;
    if(!_run_name.empty()) {
        base_dir += "/" + _run_name;
    }
    ensure_dir(base_dir);
    _base_output_dir = base_dir;
    string dvfs_path;
    if(!_dvfs_log_name.empty()) {
        if(_dvfs_log_name == "-") {
            dvfs_path = "-";
        } else if(!_dvfs_log_name.empty() && _dvfs_log_name[0] == '/') {
            dvfs_path = _dvfs_log_name;
        } else {
            dvfs_path = base_dir + "/" + _dvfs_log_name;
        }
    } else {
        dvfs_path = base_dir + "/power_log";
    }
    if(dvfs_path == "-") {
        _dvfs_log_out = &cout;
    } else {
        std::ofstream *out = new ofstream(dvfs_path.c_str());
        if(out->good()) {
            _dvfs_log_out = out;
        } else {
            delete out;
            _dvfs_log_out = NULL;
        }
    }

    vector<string> injection_process = config.GetStrArray("injection_process");
    injection_process.resize(_classes, injection_process.back());

    _injection_process.resize(_classes);

    for(int c = 0; c < _classes; ++c) {
        _traffic_pattern[c] = TrafficPattern::New(_traffic[c], _nodes, &config);
        _injection_process[c] = InjectionProcess::New(injection_process[c], _nodes, _load[c], &config);
    }

    // ============ Injection VC states  ============ 

    _buf_states.resize(_nodes);
    _last_vc.resize(_nodes);
    _last_class.resize(_nodes);

    for ( int source = 0; source < _nodes; ++source ) {
        _buf_states[source].resize(_subnets);
        _last_class[source].resize(_subnets, 0);
        _last_vc[source].resize(_subnets);
        for ( int subnet = 0; subnet < _subnets; ++subnet ) {
            ostringstream tmp_name;
            tmp_name << "terminal_buf_state_" << source << "_" << subnet;
            BufferState * bs = new BufferState( config, this, tmp_name.str( ) );
            int vc_alloc_delay = config.GetInt("vc_alloc_delay");
            int sw_alloc_delay = config.GetInt("sw_alloc_delay");
            int router_latency = config.GetInt("routing_delay") + (config.GetInt("speculative") ? max(vc_alloc_delay, sw_alloc_delay) : (vc_alloc_delay + sw_alloc_delay));
            int min_latency = 1 + _net[subnet]->GetInject(source)->GetLatency() + router_latency + _net[subnet]->GetInjectCred(source)->GetLatency();
            bs->SetMinLatency(min_latency);
            _buf_states[source][subnet] = bs;
            _last_vc[source][subnet].resize(_classes, -1);
        }
    }

#ifdef TRACK_FLOWS
    _outstanding_credits.resize(_classes);
    for(int c = 0; c < _classes; ++c) {
        _outstanding_credits[c].resize(_subnets, vector<int>(_nodes, 0));
    }
    _outstanding_classes.resize(_nodes);
    for(int n = 0; n < _nodes; ++n) {
        _outstanding_classes[n].resize(_subnets, vector<queue<int> >(_vcs));
    }
#endif

    // ============ Injection queues ============ 

    _qtime.resize(_nodes);
    _qdrained.resize(_nodes);
    _partial_packets.resize(_nodes);

    for ( int s = 0; s < _nodes; ++s ) {
        _qtime[s].resize(_classes);
        _qdrained[s].resize(_classes);
        _partial_packets[s].resize(_classes);
    }

    _total_in_flight_flits.resize(_classes);
    _measured_in_flight_flits.resize(_classes);
    _retired_packets.resize(_classes);

    _packet_seq_no.resize(_nodes);
    _repliesPending.resize(_nodes);
    _requestsOutstanding.resize(_nodes);

    _hold_switch_for_packet = config.GetInt("hold_switch_for_packet");

    // ============ Simulation parameters ============ 

    _total_sims = config.GetInt( "sim_count" );

    _router.resize(_subnets);
    for (int i=0; i < _subnets; ++i) {
        _router[i] = _net[i]->GetRouters();
    }
    _router_domains = config.GetIntArray("router_domains");
    if(_router_domains.empty()) {
        _router_domains.assign(_routers, 0);
    }
    _router_domains.resize(_routers, _router_domains.back());
    for(int subnet = 0; subnet < _subnets; ++subnet) {
        for(int r = 0; r < _routers; ++r) {
            _router[subnet][r]->SetFrequencyDomain(_router_domains[r]);
        }
    }
    _router_freq_scale.assign(_routers, 1.0);

    //seed the network
    int seed;
    if(config.GetStr("seed") == "time") {
      seed = int(time(NULL));
      cout << "SEED: seed=" << seed << endl;
    } else {
      seed = config.GetInt("seed");
    }
    RandomSeed(seed);

    _measure_latency = (config.GetStr("sim_type") == "latency");

    _sample_period = config.GetInt( "sample_period" );
    _max_samples    = config.GetInt( "max_samples" );
    _warmup_periods = config.GetInt( "warmup_periods" );

    _measure_stats = config.GetIntArray( "measure_stats" );
    if(_measure_stats.empty()) {
        _measure_stats.push_back(config.GetInt("measure_stats"));
    }
    _measure_stats.resize(_classes, _measure_stats.back());
    _pair_stats = (config.GetInt("pair_stats")==1);

    _latency_thres = config.GetFloatArray( "latency_thres" );
    if(_latency_thres.empty()) {
        _latency_thres.push_back(config.GetFloat("latency_thres"));
    }
    _latency_thres.resize(_classes, _latency_thres.back());

    _warmup_threshold = config.GetFloatArray( "warmup_thres" );
    if(_warmup_threshold.empty()) {
        _warmup_threshold.push_back(config.GetFloat("warmup_thres"));
    }
    _warmup_threshold.resize(_classes, _warmup_threshold.back());

    _acc_warmup_threshold = config.GetFloatArray( "acc_warmup_thres" );
    if(_acc_warmup_threshold.empty()) {
        _acc_warmup_threshold.push_back(config.GetFloat("acc_warmup_thres"));
    }
    _acc_warmup_threshold.resize(_classes, _acc_warmup_threshold.back());

    _stopping_threshold = config.GetFloatArray( "stopping_thres" );
    if(_stopping_threshold.empty()) {
        _stopping_threshold.push_back(config.GetFloat("stopping_thres"));
    }
    _stopping_threshold.resize(_classes, _stopping_threshold.back());

    _acc_stopping_threshold = config.GetFloatArray( "acc_stopping_thres" );
    if(_acc_stopping_threshold.empty()) {
        _acc_stopping_threshold.push_back(config.GetFloat("acc_stopping_thres"));
    }
    _acc_stopping_threshold.resize(_classes, _acc_stopping_threshold.back());

    _include_queuing = config.GetInt( "include_queuing" );

    _print_csv_results = config.GetInt( "print_csv_results" );
    _deadlock_warn_timeout = config.GetInt( "deadlock_warn_timeout" );

    string watch_file = config.GetStr( "watch_file" );
    if((watch_file != "") && (watch_file != "-")) {
        _LoadWatchList(watch_file);
    }

    vector<int> watch_flits = config.GetIntArray("watch_flits");
    for(size_t i = 0; i < watch_flits.size(); ++i) {
        _flits_to_watch.insert(watch_flits[i]);
    }
  
    vector<int> watch_packets = config.GetIntArray("watch_packets");
    for(size_t i = 0; i < watch_packets.size(); ++i) {
        _packets_to_watch.insert(watch_packets[i]);
    }

    string stats_out_file = config.GetStr( "stats_out" );
    if(stats_out_file == "") {
        _stats_out = NULL;
    } else if(stats_out_file == "-") {
        _stats_out = &cout;
    } else {
        _stats_out = new ofstream(stats_out_file.c_str());
        config.WriteMatlabFile(_stats_out);
    }
  
#ifdef TRACK_FLOWS
    _injected_flits.resize(_classes, vector<int>(_nodes, 0));
    _ejected_flits.resize(_classes, vector<int>(_nodes, 0));
    string injected_flits_out_file = config.GetStr( "injected_flits_out" );
    if(injected_flits_out_file == "") {
        _injected_flits_out = NULL;
    } else {
        _injected_flits_out = new ofstream(injected_flits_out_file.c_str());
    }
    string received_flits_out_file = config.GetStr( "received_flits_out" );
    if(received_flits_out_file == "") {
        _received_flits_out = NULL;
    } else {
        _received_flits_out = new ofstream(received_flits_out_file.c_str());
    }
    string stored_flits_out_file = config.GetStr( "stored_flits_out" );
    if(stored_flits_out_file == "") {
        _stored_flits_out = NULL;
    } else {
        _stored_flits_out = new ofstream(stored_flits_out_file.c_str());
    }
    string sent_flits_out_file = config.GetStr( "sent_flits_out" );
    if(sent_flits_out_file == "") {
        _sent_flits_out = NULL;
    } else {
        _sent_flits_out = new ofstream(sent_flits_out_file.c_str());
    }
    string outstanding_credits_out_file = config.GetStr( "outstanding_credits_out" );
    if(outstanding_credits_out_file == "") {
        _outstanding_credits_out = NULL;
    } else {
        _outstanding_credits_out = new ofstream(outstanding_credits_out_file.c_str());
    }
    string ejected_flits_out_file = config.GetStr( "ejected_flits_out" );
    if(ejected_flits_out_file == "") {
        _ejected_flits_out = NULL;
    } else {
        _ejected_flits_out = new ofstream(ejected_flits_out_file.c_str());
    }
    string active_packets_out_file = config.GetStr( "active_packets_out" );
    if(active_packets_out_file == "") {
        _active_packets_out = NULL;
    } else {
        _active_packets_out = new ofstream(active_packets_out_file.c_str());
    }
#endif

#ifdef TRACK_CREDITS
    string used_credits_out_file = config.GetStr( "used_credits_out" );
    if(used_credits_out_file == "") {
        _used_credits_out = NULL;
    } else {
        _used_credits_out = new ofstream(used_credits_out_file.c_str());
    }
    string free_credits_out_file = config.GetStr( "free_credits_out" );
    if(free_credits_out_file == "") {
        _free_credits_out = NULL;
    } else {
        _free_credits_out = new ofstream(free_credits_out_file.c_str());
    }
    string max_credits_out_file = config.GetStr( "max_credits_out" );
    if(max_credits_out_file == "") {
        _max_credits_out = NULL;
    } else {
        _max_credits_out = new ofstream(max_credits_out_file.c_str());
    }
#endif

    // ============ Statistics ============ 

    _plat_stats.resize(_classes);
    _overall_min_plat.resize(_classes, 0.0);
    _overall_avg_plat.resize(_classes, 0.0);
    _overall_max_plat.resize(_classes, 0.0);
    _plat_pcnt_stats.resize(_classes, static_cast<PercentileStats *>(NULL));

    _nlat_stats.resize(_classes);
    _overall_min_nlat.resize(_classes, 0.0);
    _overall_avg_nlat.resize(_classes, 0.0);
    _overall_max_nlat.resize(_classes, 0.0);
    _nlat_pcnt_stats.resize(_classes, static_cast<PercentileStats *>(NULL));

    _flat_stats.resize(_classes);
    _overall_min_flat.resize(_classes, 0.0);
    _overall_avg_flat.resize(_classes, 0.0);
    _overall_max_flat.resize(_classes, 0.0);
    _flat_pcnt_stats.resize(_classes, static_cast<PercentileStats *>(NULL));
    _qdel_pcnt_stats.resize(_classes, static_cast<PercentileStats *>(NULL));

    _frag_stats.resize(_classes);
    _overall_min_frag.resize(_classes, 0.0);
    _overall_avg_frag.resize(_classes, 0.0);
    _overall_max_frag.resize(_classes, 0.0);

    if(_pair_stats){
        _pair_plat.resize(_classes);
        _pair_nlat.resize(_classes);
        _pair_flat.resize(_classes);
    }
  
    _hop_stats.resize(_classes);
    _overall_hop_stats.resize(_classes, 0.0);
  
    _sent_packets.resize(_classes);
    _overall_min_sent_packets.resize(_classes, 0.0);
    _overall_avg_sent_packets.resize(_classes, 0.0);
    _overall_max_sent_packets.resize(_classes, 0.0);
    _accepted_packets.resize(_classes);
    _overall_min_accepted_packets.resize(_classes, 0.0);
    _overall_avg_accepted_packets.resize(_classes, 0.0);
    _overall_max_accepted_packets.resize(_classes, 0.0);

    _sent_flits.resize(_classes);
    _overall_min_sent.resize(_classes, 0.0);
    _overall_avg_sent.resize(_classes, 0.0);
    _overall_max_sent.resize(_classes, 0.0);
    _accepted_flits.resize(_classes);
    _overall_min_accepted.resize(_classes, 0.0);
    _overall_avg_accepted.resize(_classes, 0.0);
    _overall_max_accepted.resize(_classes, 0.0);

#ifdef TRACK_STALLS
    _buffer_busy_stalls.resize(_classes);
    _buffer_conflict_stalls.resize(_classes);
    _buffer_full_stalls.resize(_classes);
    _buffer_reserved_stalls.resize(_classes);
    _crossbar_conflict_stalls.resize(_classes);
    _overall_buffer_busy_stalls.resize(_classes, 0);
    _overall_buffer_conflict_stalls.resize(_classes, 0);
    _overall_buffer_full_stalls.resize(_classes, 0);
    _overall_buffer_reserved_stalls.resize(_classes, 0);
    _overall_crossbar_conflict_stalls.resize(_classes, 0);
#endif

    for ( int c = 0; c < _classes; ++c ) {
        ostringstream tmp_name;

        tmp_name << "plat_stat_" << c;
        _plat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _plat_stats[c];
        _plat_pcnt_stats[c] = new PercentileStats();
        tmp_name.str("");
  
        tmp_name << "nlat_stat_" << c;
        _nlat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _nlat_stats[c];
        _nlat_pcnt_stats[c] = new PercentileStats();
        tmp_name.str("");
  
        tmp_name << "flat_stat_" << c;
        _flat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _flat_stats[c];
        _flat_pcnt_stats[c] = new PercentileStats();
        tmp_name.str("");

        tmp_name << "qdel_stat_" << c;
        _qdel_pcnt_stats[c] = new PercentileStats();
        tmp_name.str("");

        tmp_name << "frag_stat_" << c;
        _frag_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 100 );
        _stats[tmp_name.str()] = _frag_stats[c];
        tmp_name.str("");

        tmp_name << "hop_stat_" << c;
        _hop_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 20 );
        _stats[tmp_name.str()] = _hop_stats[c];
        tmp_name.str("");

        if(_pair_stats){
            _pair_plat[c].resize(_nodes*_nodes);
            _pair_nlat[c].resize(_nodes*_nodes);
            _pair_flat[c].resize(_nodes*_nodes);
        }

        _sent_packets[c].resize(_nodes, 0);
        _accepted_packets[c].resize(_nodes, 0);
        _sent_flits[c].resize(_nodes, 0);
        _accepted_flits[c].resize(_nodes, 0);

#ifdef TRACK_STALLS
        _buffer_busy_stalls[c].resize(_subnets*_routers, 0);
        _buffer_conflict_stalls[c].resize(_subnets*_routers, 0);
        _buffer_full_stalls[c].resize(_subnets*_routers, 0);
        _buffer_reserved_stalls[c].resize(_subnets*_routers, 0);
        _crossbar_conflict_stalls[c].resize(_subnets*_routers, 0);
#endif
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    tmp_name << "pair_plat_stat_" << c << "_" << i << "_" << j;
                    _pair_plat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_plat[c][i*_nodes+j];
                    tmp_name.str("");
	  
                    tmp_name << "pair_nlat_stat_" << c << "_" << i << "_" << j;
                    _pair_nlat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_nlat[c][i*_nodes+j];
                    tmp_name.str("");
	  
                    tmp_name << "pair_flat_stat_" << c << "_" << i << "_" << j;
                    _pair_flat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_flat[c][i*_nodes+j];
                    tmp_name.str("");
                }
            }
        }
    }

    _slowest_flit.resize(_classes, -1);
    _slowest_packet.resize(_classes, -1);

 

}

TrafficManager::~TrafficManager( )
{

    if(_netrace_adapter) {
        for(auto &entry : _netrace_inflight) {
            _netrace_adapter->OnEject(entry.second);
        }
        _netrace_inflight.clear();
    }

    for ( int source = 0; source < _nodes; ++source ) {
        for ( int subnet = 0; subnet < _subnets; ++subnet ) {
            delete _buf_states[source][subnet];
        }
    }
  
    for ( int c = 0; c < _classes; ++c ) {
        delete _plat_stats[c];
        delete _nlat_stats[c];
        delete _flat_stats[c];
        delete _frag_stats[c];
        delete _plat_pcnt_stats[c];
        delete _nlat_pcnt_stats[c];
        delete _flat_pcnt_stats[c];
        delete _qdel_pcnt_stats[c];
        delete _hop_stats[c];

        delete _traffic_pattern[c];
        delete _injection_process[c];
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    delete _pair_plat[c][i*_nodes+j];
                    delete _pair_nlat[c][i*_nodes+j];
                    delete _pair_flat[c][i*_nodes+j];
                }
            }
        }
    }
  
    if(gWatchOut && (gWatchOut != &cout)) delete gWatchOut;
    if(_stats_out && (_stats_out != &cout)) delete _stats_out;
    if(_dvfs_log_out && (_dvfs_log_out != &cout)) {
        delete static_cast<std::ofstream*>(_dvfs_log_out);
    }

#ifdef TRACK_FLOWS
    if(_injected_flits_out) delete _injected_flits_out;
    if(_received_flits_out) delete _received_flits_out;
    if(_stored_flits_out) delete _stored_flits_out;
    if(_sent_flits_out) delete _sent_flits_out;
    if(_outstanding_credits_out) delete _outstanding_credits_out;
    if(_ejected_flits_out) delete _ejected_flits_out;
    if(_active_packets_out) delete _active_packets_out;
#endif

#ifdef TRACK_CREDITS
    if(_used_credits_out) delete _used_credits_out;
    if(_free_credits_out) delete _free_credits_out;
    if(_max_credits_out) delete _max_credits_out;
#endif

    PacketReplyInfo::FreeAll();
    Flit::FreeAll();
    Credit::FreeAll();
}


void TrafficManager::_RetireFlit( Flit *f, int dest )
{
    _deadlock_timer = 0;

    assert(_total_in_flight_flits[f->cl].count(f->id) > 0);
    _total_in_flight_flits[f->cl].erase(f->id);
  
    if(f->record) {
        assert(_measured_in_flight_flits[f->cl].count(f->id) > 0);
        _measured_in_flight_flits[f->cl].erase(f->id);
    }

    if ( f->watch ) { 
        *gWatchOut << GetSimTime() << " | "
                   << "node" << dest << " | "
                   << "Retiring flit " << f->id 
                   << " (packet " << f->pid
                   << ", src = " << f->src 
                   << ", dest = " << f->dest
                   << ", hops = " << f->hops
                   << ", flat = " << f->atime - f->itime
                   << ")." << endl;
    }

    if ( f->head && ( f->dest != dest ) ) {
        ostringstream err;
        err << "Flit " << f->id << " arrived at incorrect output " << dest;
        Error( err.str( ) );
    }
  
    if((_slowest_flit[f->cl] < 0) ||
       (_flat_stats[f->cl]->Max() < (f->atime - f->itime)))
        _slowest_flit[f->cl] = f->id;
    _flat_stats[f->cl]->AddSample( f->atime - f->itime);
    _flat_pcnt_stats[f->cl]->AddSample( f->atime - f->itime);
    if(_pair_stats){
        _pair_flat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - f->itime );
    }
      
        if ( f->tail ) {
            Flit * head;
            if(f->head) {
                head = f;
            } else {
            map<int, Flit *>::iterator iter = _retired_packets[f->cl].find(f->pid);
            assert(iter != _retired_packets[f->cl].end());
            head = iter->second;
            _retired_packets[f->cl].erase(iter);
            assert(head->head);
            assert(f->pid == head->pid);
        }
        if ( f->watch ) { 
            *gWatchOut << GetSimTime() << " | "
                       << "node" << dest << " | "
                       << "Retiring packet " << f->pid 
                       << " (plat = " << f->atime - head->ctime
                       << ", nlat = " << f->atime - head->itime
                       << ", frag = " << (f->atime - head->atime) - (f->id - head->id) // NB: In the spirit of solving problems using ugly hacks, we compute the packet length by taking advantage of the fact that the IDs of flits within a packet are contiguous.
                       << ", src = " << head->src 
                       << ", dest = " << head->dest
                       << ")." << endl;
        }

        //code the source of request, look carefully, its tricky ;)
        if (f->type == Flit::READ_REQUEST || f->type == Flit::WRITE_REQUEST) {
            PacketReplyInfo* rinfo = PacketReplyInfo::New();
            rinfo->source = f->src;
            rinfo->time = f->atime;
            rinfo->record = f->record;
            rinfo->type = f->type;
            _repliesPending[dest].push_back(rinfo);
        } else {
            if(f->type == Flit::READ_REPLY || f->type == Flit::WRITE_REPLY  ){
                _requestsOutstanding[dest]--;
            } else if(f->type == Flit::ANY_TYPE) {
                _requestsOutstanding[f->src]--;
            }
      
        }

        // Only record statistics once per packet (at tail)
        // and based on the simulation state
        if ( ( _sim_state == warming_up ) || f->record ) {
      
            _hop_stats[f->cl]->AddSample( f->hops );

        if((_slowest_packet[f->cl] < 0) ||
           (_plat_stats[f->cl]->Max() < (f->atime - head->itime)))
            _slowest_packet[f->cl] = f->pid;
        _plat_stats[f->cl]->AddSample( f->atime - head->ctime);
        _nlat_stats[f->cl]->AddSample( f->atime - head->itime);
        _plat_pcnt_stats[f->cl]->AddSample( f->atime - head->ctime);
        _nlat_pcnt_stats[f->cl]->AddSample( f->atime - head->itime);
        _frag_stats[f->cl]->AddSample( (f->atime - head->atime) - (f->id - head->id) );
        int qdel = head->itime - head->ctime;
        if(qdel >= 0) {
            _qdel_pcnt_stats[f->cl]->AddSample(qdel);
        }
   
            if(_pair_stats){
                _pair_plat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->ctime );
                _pair_nlat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->itime );
            }
        }
    
        if(f != head) {
            head->Free();
        }

        if(_use_netrace && _netrace_adapter) {
            auto it = _netrace_inflight.find(f->pid);
            if(it != _netrace_inflight.end()) {
                _netrace_adapter->OnEject(it->second);
                _netrace_inflight.erase(it);
            }
        }
    
    }
  
    if(f->head && !f->tail) {
        _retired_packets[f->cl].insert(make_pair(f->pid, f));
    } else {
        f->Free();
    }
}

int TrafficManager::_IssuePacket( int source, int cl )
{
    int result = 0;
    if(_use_read_write[cl]){ //use read and write
        //check queue for waiting replies.
        //check to make sure it is on time yet
        if (!_repliesPending[source].empty()) {
            if(_repliesPending[source].front()->time <= _time) {
                result = -1;
            }
        } else {
      
            //produce a packet
            if(_injection_process[cl]->test(source)) {
	
                //coin toss to determine request type.
                result = (RandomFloat() < _write_fraction[cl]) ? 2 : 1;
	
                _requestsOutstanding[source]++;
            }
        }
    } else { //normal mode
        result = _injection_process[cl]->test(source) ? 1 : 0;
        _requestsOutstanding[source]++;
    } 
    if(result != 0) {
        _packet_seq_no[source]++;
    }
    return result;
}

void TrafficManager::_GeneratePacket( int source, int stype, 
                                      int cl, int time,
                                      int destination_override,
                                      int size_override,
                                      Flit::FlitType packet_type_override )
{
    assert(stype!=0);

    Flit::FlitType packet_type = Flit::ANY_TYPE;
    int assigned_cl = cl;
    if(_class_assigner) {
        assigned_cl = _class_assigner->Assign(source, -1, cl, _policy_telemetry);
        if((assigned_cl < 0) || (assigned_cl >= _classes)) {
            assigned_cl = cl;
        }
    }
    cl = assigned_cl;
    int size = _GetNextPacketSize(cl); //input size 
    int pid = _cur_pid++;
    assert(_cur_pid);
    int packet_destination = _traffic_pattern[cl]->dest(source);
    bool record = false;
    bool watch = gWatchOut && (_packets_to_watch.count(pid) > 0);
    if(_use_read_write[cl]){
        if(stype > 0) {
            if (stype == 1) {
                packet_type = Flit::READ_REQUEST;
                size = _read_request_size[cl];
            } else if (stype == 2) {
                packet_type = Flit::WRITE_REQUEST;
                size = _write_request_size[cl];
            } else {
                ostringstream err;
                err << "Invalid packet type: " << packet_type;
                Error( err.str( ) );
            }
        } else {
            PacketReplyInfo* rinfo = _repliesPending[source].front();
            if (rinfo->type == Flit::READ_REQUEST) {//read reply
                size = _read_reply_size[cl];
                packet_type = Flit::READ_REPLY;
            } else if(rinfo->type == Flit::WRITE_REQUEST) {  //write reply
                size = _write_reply_size[cl];
                packet_type = Flit::WRITE_REPLY;
            } else {
                ostringstream err;
                err << "Invalid packet type: " << rinfo->type;
                Error( err.str( ) );
            }
            packet_destination = rinfo->source;
            time = rinfo->time;
            record = rinfo->record;
            _repliesPending[source].pop_front();
            rinfo->Free();
        }
    }

    if(destination_override >= 0) {
        packet_destination = destination_override;
    }
    if(size_override > 0) {
        size = size_override;
    }
    if(packet_type_override != Flit::ANY_TYPE) {
        packet_type = packet_type_override;
    }

    if ((packet_destination <0) || (packet_destination >= _nodes)) {
        ostringstream err;
        err << "Incorrect packet destination " << packet_destination
            << " for stype " << packet_type;
        Error( err.str( ) );
    }

    if ( ( _sim_state == running ) ||
         ( ( _sim_state == draining ) && ( time < _drain_time ) ) ) {
        record = _measure_stats[cl];
    }

    int subnetwork = ((packet_type == Flit::ANY_TYPE) ? 
                      RandomInt(_subnets-1) :
                      _subnet[packet_type]);
  
    if ( watch ) { 
        *gWatchOut << GetSimTime() << " | "
                   << "node" << source << " | "
                   << "Enqueuing packet " << pid
                   << " at time " << time
                   << "." << endl;
    }
  
    for ( int i = 0; i < size; ++i ) {
        Flit * f  = Flit::New();
        f->id     = _cur_id++;
        assert(_cur_id);
        f->pid    = pid;
        f->watch  = watch | (gWatchOut && (_flits_to_watch.count(f->id) > 0));
        f->subnetwork = subnetwork;
        f->src    = source;
        f->ctime  = time;
        f->record = record;
        f->cl     = cl;

        _total_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        if(record) {
            _measured_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        }
    
        if(gTrace){
            cout<<"New Flit "<<f->src<<endl;
        }
        f->type = packet_type;

        if ( i == 0 ) { // Head flit
            f->head = true;
            //packets are only generated to nodes smaller or equal to limit
            f->dest = packet_destination;
        } else {
            f->head = false;
            f->dest = -1;
        }
        switch( _pri_type ) {
        case class_based:
            f->pri = _class_priority[cl];
            assert(f->pri >= 0);
            break;
        case age_based:
            f->pri = numeric_limits<int>::max() - time;
            assert(f->pri >= 0);
            break;
        case sequence_based:
            f->pri = numeric_limits<int>::max() - _packet_seq_no[source];
            assert(f->pri >= 0);
            break;
        default:
            f->pri = 0;
        }
        if(_priority_policy) {
            _priority_policy->OnEnqueue(*f, _class_cfg[f->cl], _policy_telemetry);
        }
        if ( i == ( size - 1 ) ) { // Tail flit
            f->tail = true;
        } else {
            f->tail = false;
        }
    
        f->vc  = -1;

        if ( f->watch ) { 
            *gWatchOut << GetSimTime() << " | "
                       << "node" << source << " | "
                       << "Enqueuing flit " << f->id
                       << " (packet " << f->pid
                       << ") at time " << time
                       << "." << endl;
        }

        _partial_packets[source][cl].push_back( f );
    }
}

void TrafficManager::_Inject(){

    if(_use_netrace && _netrace_adapter) {
        _netrace_adapter->AdvanceTo(_time);
        for (int input = 0; input < _nodes; ++input) {
            // Use default class unless overridden per-packet
            int default_cl = _netrace_class;
            if (_partial_packets[input][default_cl].empty() &&
                _netrace_adapter->Ready(input, _time)) {
                NetracePacket pkt = _netrace_adapter->PopReady(input, _time);
                if (pkt.packet) {
                    int bytes = nt_get_packet_size(pkt.packet);
                    if (_netrace_use_addr_size && pkt.packet->addr > 0) {
                        bytes = pkt.packet->addr;
                    }
                    int size_flits = 1;
                    if (bytes > 0) {
                        size_flits =
                            (bytes * 8 + _channel_width - 1) / _channel_width;
                        if (size_flits < 1) size_flits = 1;
                    }
                    Flit::FlitType packet_type = Flit::ANY_TYPE;
                    switch(pkt.packet->type) {
                    case 1: packet_type = Flit::READ_REQUEST; break;
                    case 2: // ReadResp
                    case 3: // ReadRespWithInvalidate
                        packet_type = Flit::READ_REPLY; break;
                    case 4: packet_type = Flit::WRITE_REQUEST; break;
                    case 5: packet_type = Flit::WRITE_REPLY; break;
                    default: packet_type = Flit::ANY_TYPE; break;
                    }
                    _packet_seq_no[input]++;
                    _requestsOutstanding[input]++;
                    int assigned_pid = _cur_pid;
                    long long inject_cycle = pkt.cycle;
                    if (inject_cycle < _time) inject_cycle = _time;
                    if (inject_cycle > std::numeric_limits<int>::max()) {
                        inject_cycle = std::numeric_limits<int>::max();
                    }
                    int cl = default_cl;
                    if (_netrace_class_from_node_types) {
                        cl = pkt.packet->node_types;
                        if (cl < 0 || cl >= _classes) {
                            cl = default_cl;
                        }
                    }
                    _GeneratePacket(input, 1, cl,
                                    static_cast<int>(inject_cycle),
                                    static_cast<int>(pkt.packet->dst),
                                    size_flits, packet_type);
                    _netrace_inflight[assigned_pid] = pkt.packet;
                }
            }
            if ((_sim_state == draining) && _netrace_adapter->Done() &&
                _partial_packets[input][default_cl].empty()) {
                _qdrained[input][default_cl] = true;
            }
        }
        return;
    }

    for ( int input = 0; input < _nodes; ++input ) {
        for ( int c = 0; c < _classes; ++c ) {
            // Potentially generate packets for any (input,class)
            // that is currently empty
            if ( _partial_packets[input][c].empty() ) {
                bool generated = false;
                while( !generated && ( _qtime[input][c] <= _time ) ) {
                    int stype = _IssuePacket( input, c );
	  
                    if ( stype != 0 ) { //generate a packet
                        _GeneratePacket( input, stype, c, 
                                         _include_queuing==1 ? 
                                         _qtime[input][c] : _time );
                        generated = true;
                    }
                    // only advance time if this is not a reply packet
                    if(!_use_read_write[c] || (stype >= 0)){
                        ++_qtime[input][c];
                    }
                }
	
                if ( ( _sim_state == draining ) && 
                     ( _qtime[input][c] > _drain_time ) ) {
                    _qdrained[input][c] = true;
                }
            }
        }
    }
}

void TrafficManager::_Step( )
{
    _policy_telemetry.time = _time;
    bool flits_in_flight = false;
    for(int c = 0; c < _classes; ++c) {
        flits_in_flight |= !_total_in_flight_flits[c].empty();
    }
    if(flits_in_flight && (_deadlock_timer++ >= _deadlock_warn_timeout)){
        _deadlock_timer = 0;
        cout << "WARNING: Possible network deadlock.\n";
    }

    vector<map<int, Flit *> > flits(_subnets);
  
    for ( int subnet = 0; subnet < _subnets; ++subnet ) {
        for ( int n = 0; n < _nodes; ++n ) {
            Flit * const f = _net[subnet]->ReadFlit( n );
            if ( f ) {
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Ejecting flit " << f->id
                               << " (packet " << f->pid << ")"
                               << " from VC " << f->vc
                               << "." << endl;
                }
                flits[subnet].insert(make_pair(n, f));
                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_accepted_flits[f->cl][n];
                    if(f->tail) {
                        ++_accepted_packets[f->cl][n];
                    }
                }
            }

            Credit * const c = _net[subnet]->ReadCredit( n );
            if ( c ) {
#ifdef TRACK_FLOWS
                for(set<int>::const_iterator iter = c->vc.begin(); iter != c->vc.end(); ++iter) {
                    int const vc = *iter;
                    assert(!_outstanding_classes[n][subnet][vc].empty());
                    int cl = _outstanding_classes[n][subnet][vc].front();
                    _outstanding_classes[n][subnet][vc].pop();
                    assert(_outstanding_credits[cl][subnet][n] > 0);
                    --_outstanding_credits[cl][subnet][n];
                }
#endif
                _buf_states[n][subnet]->ProcessCredit(c);
                c->Free();
            }
        }
        _net[subnet]->ReadInputs( );
    }
  
    if ( !_empty_network ) {
        _Inject();
    }

    for(int subnet = 0; subnet < _subnets; ++subnet) {

        for(int n = 0; n < _nodes; ++n) {

            Flit * f = NULL;

            BufferState * const dest_buf = _buf_states[n][subnet];

            int const last_class = _last_class[n][subnet];

            int class_limit = _classes;

            if(_hold_switch_for_packet) {
                list<Flit *> const & pp = _partial_packets[n][last_class];
                if(!pp.empty() && !pp.front()->head && 
                   !dest_buf->IsFullFor(pp.front()->vc)) {
                    f = pp.front();
                    assert(f->vc == _last_vc[n][subnet][last_class]);

                    // if we're holding the connection, we don't need to check that class 
                    // again in the for loop
                    --class_limit;
                }
            }

            for(int i = 1; i <= class_limit; ++i) {

                int const c = (last_class + i) % _classes;

                list<Flit *> const & pp = _partial_packets[n][c];

                if(pp.empty()) {
                    continue;
                }

                Flit * const cf = pp.front();
                assert(cf);
                assert(cf->cl == c);
	
                if(cf->subnetwork != subnet) {
                    continue;
                }

                if(f && (f->pri >= cf->pri)) {
                    continue;
                }

                if(cf->head && cf->vc == -1) { // Find first available VC
	  
                    OutputSet route_set;
                    _rf(NULL, cf, -1, &route_set, true);
                    set<OutputSet::sSetElement> const & os = route_set.GetSet();
                    assert(os.size() == 1);
                    OutputSet::sSetElement const & se = *os.begin();
                    assert(se.output_port == -1);
                    int vc_start = se.vc_start;
                    int vc_end = se.vc_end;
                    int vc_count = vc_end - vc_start + 1;
                    if(_noq) {
                        assert(_lookahead_routing);
                        const FlitChannel * inject = _net[subnet]->GetInject(n);
                        const Router * router = inject->GetSink();
                        assert(router);
                        int in_channel = inject->GetSinkPort();

                        // NOTE: Because the lookahead is not for injection, but for the 
                        // first hop, we have to temporarily set cf's VC to be non-negative 
                        // in order to avoid seting of an assertion in the routing function.
                        cf->vc = vc_start;
                        _rf(router, cf, in_channel, &cf->la_route_set, false);
                        cf->vc = -1;

                        if(cf->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Generating lookahead routing info for flit " << cf->id
                                       << " (NOQ)." << endl;
                        }
                        set<OutputSet::sSetElement> const sl = cf->la_route_set.GetSet();
                        assert(sl.size() == 1);
                        int next_output = sl.begin()->output_port;
                        vc_count /= router->NumOutputs();
                        vc_start += next_output * vc_count;
                        vc_end = vc_start + vc_count - 1;
                        assert(vc_start >= se.vc_start && vc_start <= se.vc_end);
                        assert(vc_end >= se.vc_start && vc_end <= se.vc_end);
                        assert(vc_start <= vc_end);
                    }
                    if(cf->watch) {
                        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                   << "Finding output VC for flit " << cf->id
                                   << ":" << endl;
                    }
                    for(int i = 1; i <= vc_count; ++i) {
                        int const lvc = _last_vc[n][subnet][c];
                        int const vc =
                            (lvc < vc_start || lvc > vc_end) ?
                            vc_start :
                            (vc_start + (lvc - vc_start + i) % vc_count);
                        assert((vc >= vc_start) && (vc <= vc_end));
                        if(!dest_buf->IsAvailableFor(vc)) {
                            if(cf->watch) {
                                *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                           << "  Output VC " << vc << " is busy." << endl;
                            }
                        } else {
                            if(dest_buf->IsFullFor(vc)) {
                                if(cf->watch) {
                                    *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                               << "  Output VC " << vc << " is full." << endl;
                                }
                            } else {
                                if(cf->watch) {
                                    *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                               << "  Selected output VC " << vc << "." << endl;
                                }
                                cf->vc = vc;
                                break;
                            }
                        }
                    }
                }
	
                if(cf->vc == -1) {
                    if(cf->watch) {
                        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                   << "No output VC found for flit " << cf->id
                                   << "." << endl;
                    }
                } else {
                    if(dest_buf->IsFullFor(cf->vc)) {
                        if(cf->watch) {
                            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                       << "Selected output VC " << cf->vc
                                       << " is full for flit " << cf->id
                                       << "." << endl;
                        }
                    } else {
                        f = cf;
                    }
                }
            }

            if(f) {

                assert(f->subnetwork == subnet);

                int const c = f->cl;

                if(f->head) {
	  
                    if (_lookahead_routing) {
                        if(!_noq) {
                            const FlitChannel * inject = _net[subnet]->GetInject(n);
                            const Router * router = inject->GetSink();
                            assert(router);
                            int in_channel = inject->GetSinkPort();
                            _rf(router, f, in_channel, &f->la_route_set, false);
                            if(f->watch) {
                                *gWatchOut << GetSimTime() << " | "
                                           << "node" << n << " | "
                                           << "Generating lookahead routing info for flit " << f->id
                                           << "." << endl;
                            }
                        } else if(f->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Already generated lookahead routing info for flit " << f->id
                                       << " (NOQ)." << endl;
                        }
                    } else {
                        f->la_route_set.Clear();
                    }

                    dest_buf->TakeBuffer(f->vc);
                    _last_vc[n][subnet][c] = f->vc;
                }
	
                _last_class[n][subnet] = c;

                _partial_packets[n][c].pop_front();

#ifdef TRACK_FLOWS
                ++_outstanding_credits[c][subnet][n];
                _outstanding_classes[n][subnet][f->vc].push(c);
#endif

                dest_buf->SendingFlit(f);
	
                if(_pri_type == network_age_based) {
                    f->pri = numeric_limits<int>::max() - _time;
                    assert(f->pri >= 0);
                }
	
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Injecting flit " << f->id
                               << " into subnet " << subnet
                               << " at time " << _time
                               << " with priority " << f->pri
                               << "." << endl;
                }
                f->itime = _time;

                // Pass VC "back"
                if(!_partial_packets[n][c].empty() && !f->tail) {
                    Flit * const nf = _partial_packets[n][c].front();
                    nf->vc = f->vc;
                }
	
                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_sent_flits[c][n];
                    if(f->head) {
                        ++_sent_packets[c][n];
                    }
                }
	
#ifdef TRACK_FLOWS
                ++_injected_flits[c][n];
#endif
	
                _net[subnet]->WriteFlit(f, n);
	
            }
        }
    }

    for(int subnet = 0; subnet < _subnets; ++subnet) {
        for(int n = 0; n < _nodes; ++n) {
            map<int, Flit *>::const_iterator iter = flits[subnet].find(n);
            if(iter != flits[subnet].end()) {
                Flit * const f = iter->second;

                f->atime = _time;
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Injecting credit for VC " << f->vc 
                               << " into subnet " << subnet 
                               << "." << endl;
                }
                Credit * const c = Credit::New();
                c->vc.insert(f->vc);
                _net[subnet]->WriteCredit(c, n);
	
#ifdef TRACK_FLOWS
                ++_ejected_flits[f->cl][n];
#endif
	
                _RetireFlit(f, n);
            }
        }
        flits[subnet].clear();
        _net[subnet]->Evaluate( );
        _net[subnet]->WriteOutputs( );
    }

    _MaybeRunDVFS();

    ++_time;
    assert(_time);
    if(gTrace){
        cout<<"TIME "<<_time<<endl;
    }

}

void TrafficManager::_MaybeRunDVFS() {
    if(!_dvfs_policy || (_dvfs_epoch <= 0)) {
        return;
    }
    if((_time - _last_dvfs_epoch) < _dvfs_epoch) {
        return;
    }
    _last_dvfs_epoch = _time;

    _power_telemetry.total_power = 0.0;
    _power_telemetry.router_power.assign(_routers, 0.0);
    if(_use_orion) {
        for(int r = 0; r < _routers; ++r) {
            IQRouter *iqr = dynamic_cast<IQRouter *>(_router[0][r]);
            double freq_scale = _router_freq_scale[r];
            double p = 0.0;
            if(iqr) {
                p = iqr->GetOrionPower(freq_scale);
                iqr->ResetPowerMonitors();
            }
            _power_telemetry.router_power[r] = p;
            _power_telemetry.total_power += p;
        }
    } else {
        double base_freq = _dvfs_freqs.empty() ? 1.0 : _dvfs_freqs[0];
        auto nearest_voltage = [&](double freq)->double {
            double best_v = _dvfs_voltages.empty() ? 1.0 : _dvfs_voltages.back();
            if(_dvfs_freqs.empty() || _dvfs_voltages.empty()) return best_v;
            double best_err = fabs(freq - _dvfs_freqs[0]);
            best_v = _dvfs_voltages[0];
            for(size_t i = 0; i < _dvfs_freqs.size(); ++i) {
                double err = fabs(freq - _dvfs_freqs[i]);
                if(err < best_err) {
                    best_err = err;
                    best_v = _dvfs_voltages[i];
                }
            }
            return best_v;
        };
        for(int r = 0; r < _routers; ++r) {
            double freq = base_freq * _router_freq_scale[r];
            double v = nearest_voltage(freq);
            double dyn = _power_dyn_base * v * v * freq;
            double leak = _power_leak_base * v;
            double p = dyn + leak;
            _power_telemetry.router_power[r] = p;
            _power_telemetry.total_power += p;
        }
    }

    struct TMControl : public NetworkControl {
        explicit TMControl(TrafficManager* tm_in) : tm(tm_in) {}
        TrafficManager* tm;
        void SetRouterSpeed(int router_id, double freq_scale) override {
            if((router_id < 0) || (router_id >= tm->_routers)) return;
            tm->_router_freq_scale[router_id] = freq_scale;
            for(int subnet = 0; subnet < tm->_subnets; ++subnet) {
                tm->_router[subnet][router_id]->SetFrequencyScale(freq_scale);
            }
        }
        void SetDomainSpeed(int domain_id, double freq_scale) override {
            for(size_t r = 0; r < tm->_router_domains.size(); ++r) {
                if(tm->_router_domains[r] == domain_id) {
                    tm->_router_freq_scale[r] = freq_scale;
                    SetRouterSpeed(static_cast<int>(r), freq_scale);
                }
            }
        }
    } ctrl(this);

    _dvfs_power_avg_sum += _power_telemetry.total_power;
    _dvfs_power_avg_count++;

    bool do_log = (_time - _dvfs_log_last) >= _dvfs_log_interval;
    if(do_log && _dvfs_log_out) {
        std::map<int, double> domain_power;
        std::map<int, double> domain_freq;
        for(int r = 0; r < _routers; ++r) {
            int d = _router_domains[r];
            domain_power[d] += _power_telemetry.router_power[r];
            if(domain_freq.find(d) == domain_freq.end()) {
                domain_freq[d] = _router_freq_scale[r];
            }
        }
        std::ostringstream oss;
        oss << "DVFS epoch t=" << _time << " total_power=" << _power_telemetry.total_power;
        double headroom = _power_cap > 0 ? (_power_cap - _power_telemetry.total_power) : 0.0;
        double avg_power = (_dvfs_power_avg_count > 0) ? (_dvfs_power_avg_sum / static_cast<double>(_dvfs_power_avg_count)) : _power_telemetry.total_power;
        oss << " headroom=" << headroom << " avg_power=" << avg_power;
        oss << " domains{";
        bool first = true;
        for(std::map<int,double>::const_iterator it = domain_power.begin(); it != domain_power.end(); ++it) {
            if(!first) oss << ", ";
            first = false;
            int d = it->first;
            oss << d << ":freq=" << domain_freq[d] << ",power=" << it->second;
        }
        oss << "} routers{";
        for(int r = 0; r < _routers; ++r) {
            if(r) oss << ", ";
            oss << r << ":freq=" << _router_freq_scale[r] << ",power=" << _power_telemetry.router_power[r];
        }
        oss << "}";
        *_dvfs_log_out << oss.str() << endl;
        if(_dvfs_log_out != &cout) {
            cout << oss.str() << endl;
        }
        _dvfs_log_last = _time;
    }

    _dvfs_policy->Update(_power_telemetry, ctrl, (_dvfs_epoch ? (_time / _dvfs_epoch) : 0));
}
  
bool TrafficManager::_PacketsOutstanding( ) const
{
    for ( int c = 0; c < _classes; ++c ) {
        if ( _measure_stats[c] ) {
            if ( _measured_in_flight_flits[c].empty() ) {
	
                for ( int s = 0; s < _nodes; ++s ) {
                    if ( !_qdrained[s][c] ) {
#ifdef DEBUG_DRAIN
                        cout << "waiting on queue " << s << " class " << c;
                        cout << ", time = " << _time << " qtime = " << _qtime[s][c] << endl;
#endif
                        return true;
                    }
                }
            } else {
#ifdef DEBUG_DRAIN
                cout << "in flight = " << _measured_in_flight_flits[c].size() << endl;
#endif
                return true;
            }
        }
    }
    return false;
}

void TrafficManager::_ClearStats( )
{
    _slowest_flit.assign(_classes, -1);
    _slowest_packet.assign(_classes, -1);
    _policy_telemetry.class_latency_p99.assign(_classes, 0.0);

    for ( int c = 0; c < _classes; ++c ) {

        _plat_stats[c]->Clear( );
        _nlat_stats[c]->Clear( );
        _flat_stats[c]->Clear( );

        _frag_stats[c]->Clear( );

        _plat_pcnt_stats[c]->Clear();
        _nlat_pcnt_stats[c]->Clear();
        _flat_pcnt_stats[c]->Clear();
        _qdel_pcnt_stats[c]->Clear();

        _sent_packets[c].assign(_nodes, 0);
        _accepted_packets[c].assign(_nodes, 0);
        _sent_flits[c].assign(_nodes, 0);
        _accepted_flits[c].assign(_nodes, 0);

#ifdef TRACK_STALLS
        _buffer_busy_stalls[c].assign(_subnets*_routers, 0);
        _buffer_conflict_stalls[c].assign(_subnets*_routers, 0);
        _buffer_full_stalls[c].assign(_subnets*_routers, 0);
        _buffer_reserved_stalls[c].assign(_subnets*_routers, 0);
        _crossbar_conflict_stalls[c].assign(_subnets*_routers, 0);
#endif
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    _pair_plat[c][i*_nodes+j]->Clear( );
                    _pair_nlat[c][i*_nodes+j]->Clear( );
                    _pair_flat[c][i*_nodes+j]->Clear( );
                }
            }
        }
        _hop_stats[c]->Clear();

    }

    _reset_time = _time;
}

void TrafficManager::_ComputeStats( const vector<int> & stats, int *sum, int *min, int *max, int *min_pos, int *max_pos ) const 
{
    int const count = stats.size();
    assert(count > 0);

    if(min_pos) {
        *min_pos = 0;
    }
    if(max_pos) {
        *max_pos = 0;
    }

    if(min) {
        *min = stats[0];
    }
    if(max) {
        *max = stats[0];
    }

    *sum = stats[0];

    for ( int i = 1; i < count; ++i ) {
        int curr = stats[i];
        if ( min  && ( curr < *min ) ) {
            *min = curr;
            if ( min_pos ) {
                *min_pos = i;
            }
        }
        if ( max && ( curr > *max ) ) {
            *max = curr;
            if ( max_pos ) {
                *max_pos = i;
            }
        }
        *sum += curr;
    }
}

void TrafficManager::_DisplayRemaining( ostream & os ) const 
{
    for(int c = 0; c < _classes; ++c) {

        map<int, Flit *>::const_iterator iter;
        int i;

        os << "Class " << c << ":" << endl;

        os << "Remaining flits: ";
        for ( iter = _total_in_flight_flits[c].begin( ), i = 0;
              ( iter != _total_in_flight_flits[c].end( ) ) && ( i < 10 );
              iter++, i++ ) {
            os << iter->first << " ";
        }
        if(_total_in_flight_flits[c].size() > 10)
            os << "[...] ";
    
        os << "(" << _total_in_flight_flits[c].size() << " flits)" << endl;
    
        os << "Measured flits: ";
        for ( iter = _measured_in_flight_flits[c].begin( ), i = 0;
              ( iter != _measured_in_flight_flits[c].end( ) ) && ( i < 10 );
              iter++, i++ ) {
            os << iter->first << " ";
        }
        if(_measured_in_flight_flits[c].size() > 10)
            os << "[...] ";
    
        os << "(" << _measured_in_flight_flits[c].size() << " flits)" << endl;
    
    }
}

bool TrafficManager::_SingleSim( )
{
    int converged = 0;
  
    //once warmed up, we require 3 converging runs to end the simulation 
    vector<double> prev_latency(_classes, 0.0);
    vector<double> prev_accepted(_classes, 0.0);
    bool clear_last = false;
    int total_phases = 0;
    while( ( total_phases < _max_samples ) && 
           ( ( _sim_state != running ) || 
             ( converged < 3 ) ) ) {
    
        if ( clear_last || (( ( _sim_state == warming_up ) && ( ( total_phases % 2 ) == 0 ) )) ) {
            clear_last = false;
            _ClearStats( );
        }
    
    
        for ( int iter = 0; iter < _sample_period; ++iter )
            _Step( );
    
        //cout << _sim_state << endl;

        UpdateStats();
        DisplayStats();
    
        int lat_exc_class = -1;
        int lat_chg_exc_class = -1;
        int acc_chg_exc_class = -1;
    
        for(int c = 0; c < _classes; ++c) {
      
            if(_measure_stats[c] == 0) {
                continue;
            }

            double cur_latency = _plat_stats[c]->Average( );

            int total_accepted_count;
            _ComputeStats( _accepted_flits[c], &total_accepted_count );
            double total_accepted_rate = (double)total_accepted_count / (double)(_time - _reset_time);
            double cur_accepted = total_accepted_rate / (double)_nodes;

            double latency_change = fabs((cur_latency - prev_latency[c]) / cur_latency);
            prev_latency[c] = cur_latency;

            double accepted_change = fabs((cur_accepted - prev_accepted[c]) / cur_accepted);
            prev_accepted[c] = cur_accepted;

            double latency = (double)_plat_stats[c]->Sum();
            double count = (double)_plat_stats[c]->NumSamples();
      
            map<int, Flit *>::const_iterator iter;
            for(iter = _total_in_flight_flits[c].begin(); 
                iter != _total_in_flight_flits[c].end(); 
                iter++) {
                latency += (double)(_time - iter->second->ctime);
                count++;
            }
      
            if((lat_exc_class < 0) &&
               (_latency_thres[c] >= 0.0) &&
               ((latency / count) > _latency_thres[c])) {
                lat_exc_class = c;
            }
      
            cout << "latency change    = " << latency_change << endl;
            if(lat_chg_exc_class < 0) {
                if((_sim_state == warming_up) &&
                   (_warmup_threshold[c] >= 0.0) &&
                   (latency_change > _warmup_threshold[c])) {
                    lat_chg_exc_class = c;
                } else if((_sim_state == running) &&
                          (_stopping_threshold[c] >= 0.0) &&
                          (latency_change > _stopping_threshold[c])) {
                    lat_chg_exc_class = c;
                }
            }
      
            cout << "throughput change = " << accepted_change << endl;
            if(acc_chg_exc_class < 0) {
                if((_sim_state == warming_up) &&
                   (_acc_warmup_threshold[c] >= 0.0) &&
                   (accepted_change > _acc_warmup_threshold[c])) {
                    acc_chg_exc_class = c;
                } else if((_sim_state == running) &&
                          (_acc_stopping_threshold[c] >= 0.0) &&
                          (accepted_change > _acc_stopping_threshold[c])) {
                    acc_chg_exc_class = c;
                }
            }
      
        }
    
        // Fail safe for latency mode, throughput will ust continue
        if ( _measure_latency && ( lat_exc_class >= 0 ) ) {
      
            cout << "Average latency for class " << lat_exc_class << " exceeded " << _latency_thres[lat_exc_class] << " cycles. Aborting simulation." << endl;
            converged = 0; 
            _sim_state = draining;
            _drain_time = _time;
            if(_stats_out) {
                WriteStats(*_stats_out);
            }
            break;
      
        }
    
        if ( _sim_state == warming_up ) {
            if ( ( _warmup_periods > 0 ) ? 
                 ( total_phases + 1 >= _warmup_periods ) :
                 ( ( !_measure_latency || ( lat_chg_exc_class < 0 ) ) &&
                   ( acc_chg_exc_class < 0 ) ) ) {
                cout << "Warmed up ..." <<  "Time used is " << _time << " cycles" <<endl;
                clear_last = true;
                _sim_state = running;
            }
        } else if(_sim_state == running) {
            if ( ( !_measure_latency || ( lat_chg_exc_class < 0 ) ) &&
                 ( acc_chg_exc_class < 0 ) ) {
                ++converged;
            } else {
                converged = 0;
            }
        }
        ++total_phases;
    }
  
    if ( _sim_state == running ) {
        ++converged;
    
        _sim_state  = draining;
        _drain_time = _time;

        if ( _measure_latency ) {
            cout << "Draining all recorded packets ..." << endl;
            int empty_steps = 0;
            while( _PacketsOutstanding( ) ) { 
                _Step( ); 
	
                ++empty_steps;
	
                if ( empty_steps % 1000 == 0 ) {
	  
                    int lat_exc_class = -1;
	  
                    for(int c = 0; c < _classes; c++) {
	    
                        double threshold = _latency_thres[c];
	    
                        if(threshold < 0.0) {
                            continue;
                        }
	    
                        double acc_latency = _plat_stats[c]->Sum();
                        double acc_count = (double)_plat_stats[c]->NumSamples();
	    
                        map<int, Flit *>::const_iterator iter;
                        for(iter = _total_in_flight_flits[c].begin(); 
                            iter != _total_in_flight_flits[c].end(); 
                            iter++) {
                            acc_latency += (double)(_time - iter->second->ctime);
                            acc_count++;
                        }
	    
                        if((acc_latency / acc_count) > threshold) {
                            lat_exc_class = c;
                            break;
                        }
                    }
	  
                    if(lat_exc_class >= 0) {
                        cout << "Average latency for class " << lat_exc_class << " exceeded " << _latency_thres[lat_exc_class] << " cycles. Aborting simulation." << endl;
                        converged = 0; 
                        _sim_state = warming_up;
                        if(_stats_out) {
                            WriteStats(*_stats_out);
                        }
                        break;
                    }
	  
                    _DisplayRemaining( ); 
	  
                }
            }
        }
    } else {
        cout << "Too many sample periods needed to converge" << endl;
    }
  
    return ( converged > 0 );
}

bool TrafficManager::Run( )
{
    for ( int sim = 0; sim < _total_sims; ++sim ) {

        _time = 0;

        //remove any pending request from the previous simulations
        _requestsOutstanding.assign(_nodes, 0);
        for (int i=0;i<_nodes;i++) {
            while(!_repliesPending[i].empty()) {
                _repliesPending[i].front()->Free();
                _repliesPending[i].pop_front();
            }
        }

        //reset queuetime for all sources
        for ( int s = 0; s < _nodes; ++s ) {
            _qtime[s].assign(_classes, 0);
            _qdrained[s].assign(_classes, false);
        }

        // warm-up ...
        // reset stats, all packets after warmup_time marked
        // converge
        // draing, wait until all packets finish
        _sim_state    = warming_up;
  
        _ClearStats( );

        for(int c = 0; c < _classes; ++c) {
            _traffic_pattern[c]->reset();
            _injection_process[c]->reset();
        }

        if ( !_SingleSim( ) ) {
            cout << "Simulation unstable, ending ..." << endl;
            return false;
        }

        // Empty any remaining packets
        cout << "Draining remaining packets ..." << endl;
        _empty_network = true;
        int empty_steps = 0;

        bool packets_left = false;
        for(int c = 0; c < _classes; ++c) {
            packets_left |= !_total_in_flight_flits[c].empty();
        }

        while( packets_left ) { 
            _Step( ); 

            ++empty_steps;

            if ( empty_steps % 1000 == 0 ) {
                _DisplayRemaining( ); 
            }
      
            packets_left = false;
            for(int c = 0; c < _classes; ++c) {
                packets_left |= !_total_in_flight_flits[c].empty();
            }
        }
        //wait until all the credits are drained as well
        while(Credit::OutStanding()!=0){
            _Step();
        }
        _empty_network = false;

        //for the love of god don't ever say "Time taken" anywhere else
        //the power script depend on it
        cout << "Time taken is " << _time << " cycles" <<endl; 

        if(_stats_out) {
            WriteStats(*_stats_out);
        }
        _UpdateOverallStats();
    }
  
    DisplayOverallStats();
    if(_print_csv_results) {
        DisplayOverallStatsCSV();
    }

    _WriteCSVs();
  
    return true;
}

void TrafficManager::_UpdateOverallStats() {
    for ( int c = 0; c < _classes; ++c ) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        _overall_min_plat[c] += _plat_stats[c]->Min();
        _overall_avg_plat[c] += _plat_stats[c]->Average();
        _overall_max_plat[c] += _plat_stats[c]->Max();
        _overall_min_nlat[c] += _nlat_stats[c]->Min();
        _overall_avg_nlat[c] += _nlat_stats[c]->Average();
        _overall_max_nlat[c] += _nlat_stats[c]->Max();
        _overall_min_flat[c] += _flat_stats[c]->Min();
        _overall_avg_flat[c] += _flat_stats[c]->Average();
        _overall_max_flat[c] += _flat_stats[c]->Max();
    
        _overall_min_frag[c] += _frag_stats[c]->Min();
        _overall_avg_frag[c] += _frag_stats[c]->Average();
        _overall_max_frag[c] += _frag_stats[c]->Max();

        _overall_hop_stats[c] += _hop_stats[c]->Average();

        int count_min, count_sum, count_max;
        double rate_min, rate_sum, rate_max;
        double rate_avg;
        double time_delta = (double)(_drain_time - _reset_time);
        _ComputeStats( _sent_flits[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_sent[c] += rate_min;
        _overall_avg_sent[c] += rate_avg;
        _overall_max_sent[c] += rate_max;
        _ComputeStats( _sent_packets[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_sent_packets[c] += rate_min;
        _overall_avg_sent_packets[c] += rate_avg;
        _overall_max_sent_packets[c] += rate_max;
        _ComputeStats( _accepted_flits[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_accepted[c] += rate_min;
        _overall_avg_accepted[c] += rate_avg;
        _overall_max_accepted[c] += rate_max;
        _ComputeStats( _accepted_packets[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_accepted_packets[c] += rate_min;
        _overall_avg_accepted_packets[c] += rate_avg;
        _overall_max_accepted_packets[c] += rate_max;

#ifdef TRACK_STALLS
        _ComputeStats(_buffer_busy_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_busy_stalls[c] += rate_avg;
        _ComputeStats(_buffer_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_conflict_stalls[c] += rate_avg;
        _ComputeStats(_buffer_full_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_full_stalls[c] += rate_avg;
        _ComputeStats(_buffer_reserved_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_reserved_stalls[c] += rate_avg;
        _ComputeStats(_crossbar_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_crossbar_conflict_stalls[c] += rate_avg;
#endif

    }
}

void TrafficManager::WriteStats(ostream & os) const {
  
    os << "%=================================" << endl;

    for(int c = 0; c < _classes; ++c) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        //c+1 due to matlab array starting at 1
        os << "plat(" << c+1 << ") = " << _plat_stats[c]->Average() << ";" << endl
           << "plat_hist(" << c+1 << ",:) = " << *_plat_stats[c] << ";" << endl
           << "nlat(" << c+1 << ") = " << _nlat_stats[c]->Average() << ";" << endl
           << "nlat_hist(" << c+1 << ",:) = " << *_nlat_stats[c] << ";" << endl
           << "flat(" << c+1 << ") = " << _flat_stats[c]->Average() << ";" << endl
           << "flat_hist(" << c+1 << ",:) = " << *_flat_stats[c] << ";" << endl
           << "frag_hist(" << c+1 << ",:) = " << *_frag_stats[c] << ";" << endl
           << "hops(" << c+1 << ",:) = " << *_hop_stats[c] << ";" << endl;
        if(_pair_stats){
            os<< "pair_sent(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_plat[c][i*_nodes+j]->NumSamples() << " ";
                }
            }
            os << "];" << endl
               << "pair_plat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_plat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
            os << "];" << endl
               << "pair_nlat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_nlat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
            os << "];" << endl
               << "pair_flat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_flat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
        }

        double time_delta = (double)(_drain_time - _reset_time);

        os << "];" << endl
           << "sent_packets(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_packets[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "accepted_packets(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_packets[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "sent_flits(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_flits[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "accepted_flits(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_flits[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "sent_packet_size(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_flits[c][d] / (double)_sent_packets[c][d] << " ";
        }
        os << "];" << endl
           << "accepted_packet_size(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_flits[c][d] / (double)_accepted_packets[c][d] << " ";
        }
        os << "];" << endl;
#ifdef TRACK_STALLS
        os << "buffer_busy_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_busy_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_conflict_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_conflict_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_full_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_full_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_reserved_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_reserved_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "crossbar_conflict_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_crossbar_conflict_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl;
#endif
    }
}

void TrafficManager::UpdateStats() {
    _policy_telemetry.time = _time;
    _policy_telemetry.class_latency_p99.resize(_classes, 0.0);
    for(int c = 0; c < _classes; ++c) {
        if(_measure_stats[c]) {
            _policy_telemetry.class_latency_p99[c] = _nlat_pcnt_stats[c]->Percentile(0.99);
        }
    }
#if defined(TRACK_FLOWS) || defined(TRACK_STALLS)
    for(int c = 0; c < _classes; ++c) {
#ifdef TRACK_FLOWS
        {
            char trail_char = (c == _classes - 1) ? '\n' : ',';
            if(_injected_flits_out) *_injected_flits_out << _injected_flits[c] << trail_char;
            _injected_flits[c].assign(_nodes, 0);
            if(_ejected_flits_out) *_ejected_flits_out << _ejected_flits[c] << trail_char;
            _ejected_flits[c].assign(_nodes, 0);
        }
#endif
        for(int subnet = 0; subnet < _subnets; ++subnet) {
#ifdef TRACK_FLOWS
            if(_outstanding_credits_out) *_outstanding_credits_out << _outstanding_credits[c][subnet] << ',';
            if(_stored_flits_out) *_stored_flits_out << vector<int>(_nodes, 0) << ',';
#endif
            for(int router = 0; router < _routers; ++router) {
                Router * const r = _router[subnet][router];
#ifdef TRACK_FLOWS
                char trail_char = 
                    ((router == _routers - 1) && (subnet == _subnets - 1) && (c == _classes - 1)) ? '\n' : ',';
                if(_received_flits_out) *_received_flits_out << r->GetReceivedFlits(c) << trail_char;
                if(_stored_flits_out) *_stored_flits_out << r->GetStoredFlits(c) << trail_char;
                if(_sent_flits_out) *_sent_flits_out << r->GetSentFlits(c) << trail_char;
                if(_outstanding_credits_out) *_outstanding_credits_out << r->GetOutstandingCredits(c) << trail_char;
                if(_active_packets_out) *_active_packets_out << r->GetActivePackets(c) << trail_char;
                r->ResetFlowStats(c);
#endif
#ifdef TRACK_STALLS
                _buffer_busy_stalls[c][subnet*_routers+router] += r->GetBufferBusyStalls(c);
                _buffer_conflict_stalls[c][subnet*_routers+router] += r->GetBufferConflictStalls(c);
                _buffer_full_stalls[c][subnet*_routers+router] += r->GetBufferFullStalls(c);
                _buffer_reserved_stalls[c][subnet*_routers+router] += r->GetBufferReservedStalls(c);
                _crossbar_conflict_stalls[c][subnet*_routers+router] += r->GetCrossbarConflictStalls(c);
                r->ResetStallStats(c);
#endif
            }
        }
    }
#ifdef TRACK_FLOWS
    if(_injected_flits_out) *_injected_flits_out << flush;
    if(_received_flits_out) *_received_flits_out << flush;
    if(_stored_flits_out) *_stored_flits_out << flush;
    if(_sent_flits_out) *_sent_flits_out << flush;
    if(_outstanding_credits_out) *_outstanding_credits_out << flush;
    if(_ejected_flits_out) *_ejected_flits_out << flush;
    if(_active_packets_out) *_active_packets_out << flush;
#endif
#endif

#ifdef TRACK_CREDITS
    for(int s = 0; s < _subnets; ++s) {
        for(int n = 0; n < _nodes; ++n) {
            BufferState const * const bs = _buf_states[n][s];
            for(int v = 0; v < _vcs; ++v) {
                if(_used_credits_out) *_used_credits_out << bs->OccupancyFor(v) << ',';
                if(_free_credits_out) *_free_credits_out << bs->AvailableFor(v) << ',';
                if(_max_credits_out) *_max_credits_out << bs->LimitFor(v) << ',';
            }
        }
        for(int r = 0; r < _routers; ++r) {
            Router const * const rtr = _router[s][r];
            char trail_char = 
                ((r == _routers - 1) && (s == _subnets - 1)) ? '\n' : ',';
            if(_used_credits_out) *_used_credits_out << rtr->UsedCredits() << trail_char;
            if(_free_credits_out) *_free_credits_out << rtr->FreeCredits() << trail_char;
            if(_max_credits_out) *_max_credits_out << rtr->MaxCredits() << trail_char;
        }
    }
    if(_used_credits_out) *_used_credits_out << flush;
    if(_free_credits_out) *_free_credits_out << flush;
    if(_max_credits_out) *_max_credits_out << flush;
#endif

}

void TrafficManager::DisplayStats(ostream & os) const {
  
    for(int c = 0; c < _classes; ++c) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        cout << "Class " << c << ":" << endl;
    
        cout 
            << "Packet latency average = " << _plat_stats[c]->Average() << endl
            << "\tminimum = " << _plat_stats[c]->Min() << endl
            << "\tmaximum = " << _plat_stats[c]->Max() << endl
            << "Network latency average = " << _nlat_stats[c]->Average() << endl
            << "\tminimum = " << _nlat_stats[c]->Min() << endl
            << "\tmaximum = " << _nlat_stats[c]->Max() << endl
            << "Slowest packet = " << _slowest_packet[c] << endl
            << "Flit latency average = " << _flat_stats[c]->Average() << endl
            << "\tminimum = " << _flat_stats[c]->Min() << endl
            << "\tmaximum = " << _flat_stats[c]->Max() << endl
            << "Slowest flit = " << _slowest_flit[c] << endl
            << "Fragmentation average = " << _frag_stats[c]->Average() << endl
            << "\tminimum = " << _frag_stats[c]->Min() << endl
            << "\tmaximum = " << _frag_stats[c]->Max() << endl;
    
        int count_sum, count_min, count_max;
        double rate_sum, rate_min, rate_max;
        double rate_avg;
        int sent_packets, sent_flits, accepted_packets, accepted_flits;
        int min_pos, max_pos;
        double time_delta = (double)(_time - _reset_time);
        _ComputeStats(_sent_packets[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        sent_packets = count_sum;
        cout << "Injected packet rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_accepted_packets[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        accepted_packets = count_sum;
        cout << "Accepted packet rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_sent_flits[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        sent_flits = count_sum;
        cout << "Injected flit rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_accepted_flits[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        accepted_flits = count_sum;
        cout << "Accepted flit rate average= " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
    
        cout << "Injected packet length average = " << (double)sent_flits / (double)sent_packets << endl
             << "Accepted packet length average = " << (double)accepted_flits / (double)accepted_packets << endl;

        cout << "Total in-flight flits = " << _total_in_flight_flits[c].size()
             << " (" << _measured_in_flight_flits[c].size() << " measured)"
             << endl;
    
#ifdef TRACK_STALLS
        _ComputeStats(_buffer_busy_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer busy stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer conflict stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_full_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer full stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_reserved_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer reserved stall rate = " << rate_avg << endl;
        _ComputeStats(_crossbar_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Crossbar conflict stall rate = " << rate_avg << endl;
#endif
    
    }
}

void TrafficManager::DisplayOverallStats( ostream & os ) const {

    os << "====== Overall Traffic Statistics ======" << endl;
    for ( int c = 0; c < _classes; ++c ) {

        if(_measure_stats[c] == 0) {
            continue;
        }

        os << "====== Traffic class " << c << " ======" << endl;
    
        os << "Packet latency average = " << _overall_avg_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Network latency average = " << _overall_avg_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Flit latency average = " << _overall_avg_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Fragmentation average = " << _overall_avg_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Injected packet rate average = " << _overall_avg_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Accepted packet rate average = " << _overall_avg_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Injected flit rate average = " << _overall_avg_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Accepted flit rate average = " << _overall_avg_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Injected packet size average = " << _overall_avg_sent[c] / _overall_avg_sent_packets[c]
           << " (" << _total_sims << " samples)" << endl;

        os << "Accepted packet size average = " << _overall_avg_accepted[c] / _overall_avg_accepted_packets[c]
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Hops average = " << _overall_hop_stats[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
#ifdef TRACK_STALLS
        os << "Buffer busy stall rate = " << (double)_overall_buffer_busy_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer conflict stall rate = " << (double)_overall_buffer_conflict_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer full stall rate = " << (double)_overall_buffer_full_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer reserved stall rate = " << (double)_overall_buffer_reserved_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Crossbar conflict stall rate = " << (double)_overall_crossbar_conflict_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
#endif
    
    }
  
}

string TrafficManager::_OverallStatsCSV(int c) const
{
    ostringstream os;
    os << _traffic[c]
       << ',' << _use_read_write[c]
       << ',' << _load[c]
       << ',' << _overall_min_plat[c] / (double)_total_sims
       << ',' << _overall_avg_plat[c] / (double)_total_sims
       << ',' << _overall_max_plat[c] / (double)_total_sims
       << ',' << _overall_min_nlat[c] / (double)_total_sims
       << ',' << _overall_avg_nlat[c] / (double)_total_sims
       << ',' << _overall_max_nlat[c] / (double)_total_sims
       << ',' << _overall_min_flat[c] / (double)_total_sims
       << ',' << _overall_avg_flat[c] / (double)_total_sims
       << ',' << _overall_max_flat[c] / (double)_total_sims
       << ',' << _overall_min_frag[c] / (double)_total_sims
       << ',' << _overall_avg_frag[c] / (double)_total_sims
       << ',' << _overall_max_frag[c] / (double)_total_sims
       << ',' << _overall_min_sent_packets[c] / (double)_total_sims
       << ',' << _overall_avg_sent_packets[c] / (double)_total_sims
       << ',' << _overall_max_sent_packets[c] / (double)_total_sims
       << ',' << _overall_min_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_avg_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_max_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_min_sent[c] / (double)_total_sims
       << ',' << _overall_avg_sent[c] / (double)_total_sims
       << ',' << _overall_max_sent[c] / (double)_total_sims
       << ',' << _overall_min_accepted[c] / (double)_total_sims
       << ',' << _overall_avg_accepted[c] / (double)_total_sims
       << ',' << _overall_max_accepted[c] / (double)_total_sims
       << ',' << _overall_avg_sent[c] / _overall_avg_sent_packets[c]
       << ',' << _overall_avg_accepted[c] / _overall_avg_accepted_packets[c]
       << ',' << _overall_hop_stats[c] / (double)_total_sims;

#ifdef TRACK_STALLS
    os << ',' << (double)_overall_buffer_busy_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_conflict_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_full_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_reserved_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_crossbar_conflict_stalls[c] / (double)_total_sims;
#endif

    return os.str();
}

void TrafficManager::DisplayOverallStatsCSV(ostream & os) const {
    for(int c = 0; c < _classes; ++c) {
        os << "results:" << c << ',' << _OverallStatsCSV() << endl;
    }
}

void TrafficManager::_WriteCSVs() {
    string base_dir = _base_output_dir.empty() ? "sims" : _base_output_dir;
    auto make_path = [&](const string &fname)->string {
        if(fname.empty()) return "";
        if(fname == "-") return "-";
        if(!fname.empty() && fname[0] == '/') return fname;
        return base_dir + "/" + fname;
    };

    // Latency percentiles per class
    string lat_path = make_path(_latency_csv_name);
    if(!lat_path.empty()) {
        ostream *out = NULL;
        ofstream lat_file;
        if(lat_path == "-") {
            out = &cout;
        } else {
            lat_file.open(lat_path.c_str());
            if(lat_file.good()) out = &lat_file;
        }
        if(out) {
            *out << "class,plat_p50,plat_p95,plat_p99,nlat_p50,nlat_p95,nlat_p99,flat_p50,flat_p95,flat_p99\n";
            for(int c = 0; c < _classes; ++c) {
                double plat50 = _plat_pcnt_stats[c]->Percentile(0.50);
                double plat95 = _plat_pcnt_stats[c]->Percentile(0.95);
                double plat99 = _plat_pcnt_stats[c]->Percentile(0.99);
                double nlat50 = _nlat_pcnt_stats[c]->Percentile(0.50);
                double nlat95 = _nlat_pcnt_stats[c]->Percentile(0.95);
                double nlat99 = _nlat_pcnt_stats[c]->Percentile(0.99);
                double flat50 = _flat_pcnt_stats[c]->Percentile(0.50);
                double flat95 = _flat_pcnt_stats[c]->Percentile(0.95);
                double flat99 = _flat_pcnt_stats[c]->Percentile(0.99);
                *out << c << "," << plat50 << "," << plat95 << "," << plat99 << ","
                     << nlat50 << "," << nlat95 << "," << nlat99 << ","
                     << flat50 << "," << flat95 << "," << flat99 << "\n";
            }
        }
    }

#ifdef TRACK_STALLS
    string stall_path = make_path(_stall_csv_name);
    if(!stall_path.empty()) {
        ostream *out = NULL;
        ofstream stall_file;
        if(stall_path == "-") out = &cout;
        else {
            stall_file.open(stall_path.c_str());
            if(stall_file.good()) out = &stall_file;
        }
        if(out) {
            *out << "class,buffer_busy,buffer_conflict,buffer_full,buffer_reserved,crossbar_conflict\n";
            for(int c = 0; c < _classes; ++c) {
                *out << c << ","
                     << _overall_buffer_busy_stalls[c] << ","
                     << _overall_buffer_conflict_stalls[c] << ","
                     << _overall_buffer_full_stalls[c] << ","
                     << _overall_buffer_reserved_stalls[c] << ","
                     << _overall_crossbar_conflict_stalls[c] << "\n";
            }
        }
    }
#endif

    string thr_path = make_path(_throughput_csv_name);
    if(!thr_path.empty()) {
        ostream *out = NULL;
        ofstream thr_file;
        if(thr_path == "-") out = &cout;
        else {
            thr_file.open(thr_path.c_str());
            if(thr_file.good()) out = &thr_file;
        }
        if(out) {
            double sim_time = static_cast<double>(_time - _reset_time);
            *out << "class,offered_load,sent_packets,accepted_packets,sent_flits,accepted_flits,avg_throughput_flits_per_cycle\n";
            for(int c = 0; c < _classes; ++c) {
                int sentp = 0, accp = 0, sentf = 0, accf = 0;
                _ComputeStats(_sent_packets[c], &sentp);
                _ComputeStats(_accepted_packets[c], &accp);
                _ComputeStats(_sent_flits[c], &sentf);
                _ComputeStats(_accepted_flits[c], &accf);
                double thr = (sim_time > 0) ? (static_cast<double>(accf) / sim_time) : 0.0;
                double offered = (_load.size() > c) ? _load[c] : 0.0;
                *out << c << "," << offered << "," << sentp << "," << accp << "," << sentf << "," << accf << "," << thr << "\n";
            }
        }
    }

    // Summary CSV (one line)
    string sum_path = make_path(_summary_csv_name);
    if(!sum_path.empty()) {
        ostream *out = NULL;
        ofstream sum_file;
        if(sum_path == "-") out = &cout;
        else {
            sum_file.open(sum_path.c_str());
            if(sum_file.good()) out = &sum_file;
        }
        if(out) {
            *out << "policy,power_cap,control_p99,batch_p99,total_power_avg\n";
            double control_p99 = _nlat_pcnt_stats.size() > 0 ? _nlat_pcnt_stats[0]->Percentile(0.99) : 0.0;
            double batch_p99 = (_classes > 1 && _nlat_pcnt_stats.size() > 1) ? _nlat_pcnt_stats[1]->Percentile(0.99) : 0.0;
            double avg_power = (_dvfs_power_avg_count > 0) ? (_dvfs_power_avg_sum / static_cast<double>(_dvfs_power_avg_count)) : 0.0;
            *out << (_dvfs_policy ? _dvfs_policy->GetType() : "unknown") << ","
                 << _power_cap << ","
                 << control_p99 << ","
                 << batch_p99 << ","
                 << avg_power << "\n";
        }
    }
}

//read the watchlist
void TrafficManager::_LoadWatchList(const string & filename){
    ifstream watch_list;
    watch_list.open(filename.c_str());
  
    string line;
    if(watch_list.is_open()) {
        while(!watch_list.eof()) {
            getline(watch_list, line);
            if(line != "") {
                if(line[0] == 'p') {
                    _packets_to_watch.insert(atoi(line.c_str()+1));
                } else {
                    _flits_to_watch.insert(atoi(line.c_str()));
                }
            }
        }
    
    } else {
        Error("Unable to open flit watch file: " + filename);
    }
}

int TrafficManager::_GetNextPacketSize(int cl) const
{
    assert(cl >= 0 && cl < _classes);

    vector<int> const & psize = _packet_size[cl];
    int sizes = psize.size();

    if(sizes == 1) {
        return psize[0];
    }

    vector<int> const & prate = _packet_size_rate[cl];
    int max_val = _packet_size_max_val[cl];

    int pct = RandomInt(max_val);

    for(int i = 0; i < (sizes - 1); ++i) {
        int const limit = prate[i];
        if(limit > pct) {
            return psize[i];
        } else {
            pct -= limit;
        }
    }
    assert(prate.back() > pct);
    return psize.back();
}

double TrafficManager::_GetAveragePacketSize(int cl) const
{
    vector<int> const & psize = _packet_size[cl];
    int sizes = psize.size();
    if(sizes == 1) {
        return (double)psize[0];
    }
    vector<int> const & prate = _packet_size_rate[cl];
    int sum = 0;
    for(int i = 0; i < sizes; ++i) {
        sum += psize[i] * prate[i];
    }
    return (double)sum / (double)(_packet_size_max_val[cl] + 1);
}
