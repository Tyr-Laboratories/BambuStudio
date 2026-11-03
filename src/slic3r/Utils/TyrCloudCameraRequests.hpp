#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace Slic3r {
struct TyrCloudCameraState {
    std::recursive_mutex delivery;
    std::atomic<bool> stopping{false};
    std::atomic<uint64_t> generation{0};
};

// Retained by deferred UI work as well as the worker. Account changes invalidate
// it even after the worker callback has queued a result on the GUI thread.
class TyrCloudCameraGuard {
    friend class TyrCloudCameraRequests;
    std::shared_ptr<TyrCloudCameraState> state;
    uint64_t generation = 0;
    bool unbound = false;
    TyrCloudCameraGuard(std::shared_ptr<TyrCloudCameraState> s, uint64_t epoch)
        : state(std::move(s)), generation(epoch) {}
public:
    TyrCloudCameraGuard() = default;
    static TyrCloudCameraGuard unrestricted()
    {
        TyrCloudCameraGuard guard;
        guard.unbound = true;
        return guard;
    }
    bool cancelled() const
    {
        return state ? state->stopping || state->generation != generation : !unbound;
    }
    bool deliver(const std::function<void()> &consume) const
    {
        if (!state) {
            if (!unbound) return false;
            consume();
            return true;
        }
        std::lock_guard<std::recursive_mutex> lock(state->delivery);
        if (cancelled()) return false;
        consume();
        return true;
    }
};

class TyrCloudCameraRequests {
public:
    using Cancelled = std::function<bool()>;
    using Work = std::function<std::string(Cancelled)>;
    using Callback = std::function<void(std::string, TyrCloudCameraGuard)>;
private:
    struct Request { Work work; Callback callback; TyrCloudCameraGuard guard; };
    std::shared_ptr<TyrCloudCameraState> state = std::make_shared<TyrCloudCameraState>();
    std::recursive_mutex account;
    bool changing_account = false;
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Request> pending;
    std::thread worker;
    void run()
    {
        for (;;) {
            Request request;
            {
                std::unique_lock<std::mutex> lock(mutex);
                ready.wait(lock, [&] { return state->stopping || !pending.empty(); });
                if (state->stopping) return;
                request = std::move(pending.front());
                pending.pop_front();
            }
            std::string result;
            try { result = request.work([guard = request.guard] { return guard.cancelled(); }); }
            catch (...) { result = "Cloud camera connection failed. Check Docker Desktop and try again."; }
            request.guard.deliver([&] {
                try { request.callback(std::move(result), request.guard); } catch (...) {}
            });
        }
    }
public:
    TyrCloudCameraRequests() : worker([this] { run(); }) {}
    ~TyrCloudCameraRequests()
    {
        {
            std::lock_guard<std::recursive_mutex> delivery(state->delivery);
            std::lock_guard<std::recursive_mutex> admission(account);
            std::lock_guard<std::mutex> queue(mutex);
            state->stopping = true;
            ++state->generation;
            pending.clear();
        }
        ready.notify_one();
        worker.join();
    }
    bool submit_factory(const std::function<Work()> &factory, Callback callback)
    {
        std::lock_guard<std::recursive_mutex> admission(account);
        if (changing_account || state->stopping || !factory || !callback) return false;
        auto work = factory();
        if (!work) return false;
        std::lock_guard<std::mutex> queue(mutex);
        if (pending.size() >= 16) return false;
        pending.push_back({std::move(work), std::move(callback), {state, state->generation.load()}});
        ready.notify_one();
        return true;
    }
    bool submit(Work work, Callback callback)
    {
        return submit_factory([work = std::move(work)] { return work; }, std::move(callback));
    }
    void cancel_all()
    {
        std::lock_guard<std::recursive_mutex> delivery(state->delivery);
        std::lock_guard<std::mutex> queue(mutex);
        ++state->generation;
        pending.clear();
    }
    // Lock order is delivery -> admission -> queue. Callbacks can reenter the
    // same thread; admissions during the native account mutation are refused.
    int account_change(const std::function<int()> &mutate)
    {
        std::lock_guard<std::recursive_mutex> delivery(state->delivery);
        std::lock_guard<std::recursive_mutex> admission(account);
        changing_account = true;
        struct Restore { bool &flag; ~Restore() { flag = false; } } restore{changing_account};
        cancel_all();
        return mutate();
    }
};
}
