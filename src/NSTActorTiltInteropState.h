#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <mutex>


class NSTActorTiltInteropState final {
public:
    static constexpr std::size_t MaxActors = 5;
    bool Publish(std::uint32_t thread, std::uint64_t epoch,
                 const std::uint32_t* actors, std::size_t count) noexcept {
        std::scoped_lock lock(mutex_);
        active_ = false;
        actors_.fill(0);
        count_ = 0;
        if (!epoch || !actors || count < 1 || count > MaxActors) return false;
        for (std::size_t i=0;i<count;++i) {
            if (!actors[i]) return false;
            for (std::size_t j=0;j<i;++j) if (actors[i]==actors[j]) return false;
            actors_[i]=actors[i];
        }
        thread_=thread; epoch_=epoch; count_=count; active_=true;
        return true;
    }
    bool Revoke() noexcept {
        std::scoped_lock lock(mutex_);
        const bool was=active_; active_=false; return was;
    }
    bool Contains(std::uint32_t actorFormID) const noexcept {
        if (!actorFormID) return false;
        std::scoped_lock lock(mutex_);
        if (!active_) return false;
        for (std::size_t i=0;i<count_;++i) if (actors_[i]==actorFormID) return true;
        return false;
    }
private:
    mutable std::mutex mutex_{};
    std::array<std::uint32_t,MaxActors> actors_{};
    std::uint32_t thread_{};
    std::uint64_t epoch_{};
    std::size_t count_{};
    bool active_{};
};
