// dispatcher.cpp: Central message dispatcher implementation.
// Polls all registered decoder queues and routes data to the appropriate sinks.

#include "dispatcher.h"

// We cannot include dump1090.h directly (C11 stdatomic.h conflicts with C++).
// Instead, forward-declare only what we need from the C side.
extern "C" {
#include "dump1090_defs.h"
#include "dump1090_types.h"
#include "dump1090_message.h"

// From mode_s.h
void useModesMessage(struct modesMessage *mm);

// From track.h
struct aircraft *trackUpdateFromDecoder(const aircraft_update_t *upd);

// From dump1090_state.h / globals
extern pthread_rwlock_t aircraft_lock;
}

// Global dispatcher instance
Dispatcher g_dispatcher;

// ======================== Dispatcher methods ========================

void Dispatcher::registerDecoder(const DecoderQueues& dq) {
    decoders_.push_back(dq);
}

DecoderQueue<struct modesMessage>* Dispatcher::registerAdsbQueue(const std::string& name) {
    auto *q = new DecoderQueue<struct modesMessage>(8192);
    adsb_sources_.push_back({name, q});
    return q;
}

void Dispatcher::poll() {
    // --- ADS-B: drain all registered modesMessage queues ---
    for (auto& src : adsb_sources_) {
        struct modesMessage mm;
        while (src.queue->pop(mm)) {
            useModesMessage(&mm);
        }
    }

    // --- Other decoders: drain typed queues ---
    for (auto& dec : decoders_) {
        // Drain aircraft updates
        if (dec.aircraft) {
            aircraft_update_t upd;
            while (dec.aircraft->pop(upd)) {
                if (aircraft_handler_) aircraft_handler_(upd);
            }
        }

        // Drain text messages
        if (dec.messages) {
            text_message_t msg;
            while (dec.messages->pop(msg)) {
                if (message_handler_) message_handler_(msg);
            }
        }

        // Drain ground tracks
        if (dec.ground) {
            ground_track_t gt;
            while (dec.ground->pop(gt)) {
                if (ground_handler_) ground_handler_(gt);
            }
        }

        // Drain raw Mode-S (ADS-B only)
        if (dec.raw) {
            raw_modes_t raw;
            while (dec.raw->pop(raw)) {
                if (raw_handler_) raw_handler_(raw);
            }
        }
    }
}

// ======================== C interface ========================

extern "C" {

int dispatcher_push_aircraft(aircraft_queue_handle_t q, const aircraft_update_t* upd) {
    auto* queue = static_cast<DecoderQueue<aircraft_update_t>*>(q);
    return queue->push(*upd) ? 1 : 0;
}

int dispatcher_push_message(message_queue_handle_t q, const text_message_t* msg) {
    auto* queue = static_cast<DecoderQueue<text_message_t>*>(q);
    return queue->push(*msg) ? 1 : 0;
}

int dispatcher_push_ground(ground_queue_handle_t q, const ground_track_t* gt) {
    auto* queue = static_cast<DecoderQueue<ground_track_t>*>(q);
    return queue->push(*gt) ? 1 : 0;
}

int dispatcher_push_raw(raw_queue_handle_t q, const raw_modes_t* raw) {
    auto* queue = static_cast<DecoderQueue<raw_modes_t>*>(q);
    return queue->push(*raw) ? 1 : 0;
}

int dispatcher_push_adsb(adsb_queue_handle_t q, const struct modesMessage* mm) {
    auto* queue = static_cast<DecoderQueue<struct modesMessage>*>(q);
    return queue->push(*mm) ? 1 : 0;
}

void dispatcher_poll(void) {
    g_dispatcher.poll();
}

void dispatcher_init(void) {
    // Wire up handlers for other decoders (FLARM, FANET, etc.)
    // These will be connected in subsequent migration steps.

    g_dispatcher.onAircraft([](const aircraft_update_t& upd) {
        pthread_rwlock_wrlock(&aircraft_lock);
        trackUpdateFromDecoder(&upd);
        pthread_rwlock_unlock(&aircraft_lock);
    });

    g_dispatcher.onMessage([](const text_message_t& msg) {
        // TODO Step 4+: call panelLogMessage
        (void)msg;
    });

    g_dispatcher.onGround([](const ground_track_t& gt) {
        // TODO Step 4+: update ground cache
        (void)gt;
    });

    g_dispatcher.onRaw([](const raw_modes_t& raw) {
        // TODO: future use
        (void)raw;
    });
}

aircraft_queue_handle_t dispatcher_register_aircraft_queue(const char *name) {
    auto *q = new DecoderQueue<aircraft_update_t>(4096);
    DecoderQueues dq;
    dq.name = name;
    dq.aircraft = q;
    g_dispatcher.registerDecoder(dq);
    return static_cast<aircraft_queue_handle_t>(q);
}

adsb_queue_handle_t dispatcher_register_adsb_queue(const char *name) {
    return static_cast<adsb_queue_handle_t>(g_dispatcher.registerAdsbQueue(name));
}

} // extern "C"
