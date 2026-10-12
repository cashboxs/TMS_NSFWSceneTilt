#pragma once
// NST POC-02C: minimal, non-owning ABI declarations for OStim's legacy
// SKSE InterfaceExchangeMessage / "Threads" interface (not the newer
// RequestPluginAPI_Thread export).
// Interface ordering informed by the OStim public PluginInterface ABI and
// OStim-Size-Difference-Manager's OStimInterface.h:
// https://github.com/shonkthingy/OStim-Size-Difference-Manager/blob/master/skse/src/OStimAPI/OStimInterface.h
// KEEP virtual function order/signatures unchanged. Do NOT delete OStim objects.
#include <cstdint>

namespace OStimLegacy {
class PluginInterface {
public:
    virtual ~PluginInterface() = default;
    virtual std::uint32_t getVersion() = 0;
};
class ThreadActor {
public:
    virtual void* getGameActor() = 0;
};
class Thread {
public:
    virtual std::int32_t getThreadID() = 0;
    virtual bool isPlayerThread() = 0;
    virtual std::uint32_t getActorCount() = 0;
    virtual ThreadActor* getActor(std::uint32_t position) = 0;
    virtual void forEachThreadActor(void* visitor) = 0;
    virtual void* getCurrentNode() = 0;
    // Only call when Threads::getVersion() >= 3 (added at ABI v3).
    virtual void* getFurnitureType() = 0;
    virtual void* getFurnitureObject() = 0;
};
class ThreadEventListener {
public:
    virtual void threadRegistered(Thread* thread) = 0;
};
class ThreadInterface : public PluginInterface {
public:
    static constexpr const char* NAME = "Threads";
    virtual Thread* getThread(std::int32_t threadID) = 0;
    virtual void registerThreadStartListener(ThreadEventListener* listener) = 0;
    virtual void registerSpeedChangedListener(ThreadEventListener* listener) = 0;
    virtual void registerNodeChangedListener(ThreadEventListener* listener) = 0;
    virtual void registerClimaxListener(void* listener) = 0;
    virtual void registerThreadStopListener(ThreadEventListener* listener) = 0;
    virtual void* createThreadBuilder(std::uint32_t actorCount, void** actors) = 0;
};
class InterfaceMap {
public:
    virtual PluginInterface* queryInterface(const char* name) = 0;
    virtual bool addInterface(const char* name, PluginInterface* iface) = 0;
    virtual PluginInterface* removeInterface(const char* name) = 0;
};
struct InterfaceExchangeMessage {
    // Identical to the upstream multichar constant 'OST' without compiler
    // implementation-dependent multi-character literal warnings.
    static constexpr std::uint32_t MESSAGE_TYPE = (static_cast<std::uint32_t>('O') << 16) |
                                                   (static_cast<std::uint32_t>('S') << 8) |
                                                    static_cast<std::uint32_t>('T');
    InterfaceMap* interfaceMap = nullptr;
};
static_assert(InterfaceExchangeMessage::MESSAGE_TYPE == 0x4F5354u);
static_assert(sizeof(InterfaceExchangeMessage) == sizeof(void*));
} // namespace OStimLegacy
