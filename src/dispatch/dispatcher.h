// dispatcher.h: Central message dispatcher.
// Polls decoder queues and routes data to aircraft list, feeders, and APIs.
// C++ implementation with C-callable interface.

#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "decoder_types.h"

#ifdef __cplusplus

#include "decoder_queue.h"
#include <vector>
#include <string>
#include <functional>

// Forward-declare modesMessage for the ADS-B raw queue
struct modesMessage;

// ======================== Decoder registration ========================

// Each decoder creates its queues and registers them here.
// Nullable pointers: a decoder only provides the queues it uses.

struct DecoderQueues {
    std::string                    name;       // "adsb", "flarm", "fanet", ...
    DecoderQueue<aircraft_update_t>* aircraft = nullptr;
    DecoderQueue<text_message_t>*    messages = nullptr;
    DecoderQueue<ground_track_t>*    ground   = nullptr;
    DecoderQueue<raw_modes_t>*       raw      = nullptr;
};

// ======================== Dispatcher ========================

class Dispatcher {
public:
    // Register a decoder's queues. Called once per decoder at startup.
    void registerDecoder(const DecoderQueues& dq);

    // Poll all queues and dispatch. Called from main thread (backgroundTasks).
    void poll();

    // Set callbacks for routing data
    using AircraftHandler = std::function<void(const aircraft_update_t&)>;
    using MessageHandler  = std::function<void(const text_message_t&)>;
    using GroundHandler   = std::function<void(const ground_track_t&)>;
    using RawHandler      = std::function<void(const raw_modes_t&)>;

    void onAircraft(AircraftHandler h) { aircraft_handler_ = std::move(h); }
    void onMessage(MessageHandler h)   { message_handler_  = std::move(h); }
    void onGround(GroundHandler h)     { ground_handler_   = std::move(h); }
    void onRaw(RawHandler h)           { raw_handler_      = std::move(h); }

    // Register an ADS-B modesMessage queue (one per source)
    DecoderQueue<struct modesMessage>* registerAdsbQueue(const std::string& name);

private:
    std::vector<DecoderQueues> decoders_;
    AircraftHandler aircraft_handler_;
    MessageHandler  message_handler_;
    GroundHandler   ground_handler_;
    RawHandler      raw_handler_;

    // Per-source ADS-B modesMessage queues
    struct AdsbSource {
        std::string name;
        DecoderQueue<struct modesMessage>* queue;
    };
    std::vector<AdsbSource> adsb_sources_;
};

// Global dispatcher instance
extern Dispatcher g_dispatcher;

#endif // __cplusplus

// ======================== C interface ========================

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handles for C code to push into queues
typedef void* aircraft_queue_handle_t;
typedef void* message_queue_handle_t;
typedef void* ground_queue_handle_t;
typedef void* raw_queue_handle_t;
typedef void* adsb_queue_handle_t;

// Push functions return 1 on success, 0 if queue full
int dispatcher_push_aircraft(aircraft_queue_handle_t q, const aircraft_update_t* upd);
int dispatcher_push_message(message_queue_handle_t q, const text_message_t* msg);
int dispatcher_push_ground(ground_queue_handle_t q, const ground_track_t* gt);
int dispatcher_push_raw(raw_queue_handle_t q, const raw_modes_t* raw);

// ADS-B: push a modesMessage into a registered per-source queue
struct modesMessage;
int dispatcher_push_adsb(adsb_queue_handle_t q, const struct modesMessage* mm);

// Called from main thread (backgroundTasks) to drain all queues
void dispatcher_poll(void);

// Called at startup to initialize the dispatcher and wire up handlers
void dispatcher_init(void);

// Register a new decoder with an aircraft queue. Returns handle for pushing.
aircraft_queue_handle_t dispatcher_register_aircraft_queue(const char *name);

// Register a new ADS-B source queue (demod, net, feeder). Returns handle.
adsb_queue_handle_t dispatcher_register_adsb_queue(const char *name);

#ifdef __cplusplus
}
#endif

#endif // DISPATCHER_H
