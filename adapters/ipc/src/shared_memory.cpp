/**
 * @file shared_memory.cpp
 * @brief 实现 POSIX 共享内存的创建、映射、校验和释放。
 */

#include "quadruped/ipc/shared_memory.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <new>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace quadruped::ipc
{
namespace
{

std::string error_text(const char* operation)
{
    return std::string(operation) + ": " + std::strerror(errno);
}

bool valid_name(const std::string& name)
{
    return name.size() > 1 && name.front() == '/' && name.find('/', 1) == std::string::npos;
}

}  // 匿名命名空间

SharedMemory::OpenResult SharedMemory::create_owner(
    const std::string& name,
    const WireIdentity& identity)
{
    OpenResult result;
    if (!valid_name(name))
    {
        result.error_message = "shared memory name must start with one slash";
        return result;
    }

    shm_unlink(name.c_str());
    const int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0)
    {
        result.error_message = error_text("shm_open owner failed");
        return result;
    }
    if (ftruncate(fd, static_cast<off_t>(sizeof(SharedLayout))) != 0)
    {
        result.error_message = error_text("ftruncate failed");
        close(fd);
        shm_unlink(name.c_str());
        return result;
    }

    void* address = mmap(
        nullptr,
        sizeof(SharedLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);
    if (address == MAP_FAILED)
    {
        result.error_message = error_text("mmap owner failed");
        close(fd);
        shm_unlink(name.c_str());
        return result;
    }

    auto* layout = new (address) SharedLayout{};
    layout->identity = identity;
    layout->ready.store(1, std::memory_order_release);
    result.memory.reset(new SharedMemory(name, fd, layout, true));
    return result;
}

SharedMemory::OpenResult SharedMemory::open_existing(const std::string& name)
{
    OpenResult result;
    if (!valid_name(name))
    {
        result.error_message = "shared memory name must start with one slash";
        return result;
    }

    const int fd = shm_open(name.c_str(), O_RDWR, 0600);
    if (fd < 0)
    {
        result.error_message = error_text("shm_open existing failed");
        return result;
    }

    struct stat status{};
    if (fstat(fd, &status) != 0 || status.st_size != static_cast<off_t>(sizeof(SharedLayout)))
    {
        result.error_message = "shared memory size does not match wire schema";
        close(fd);
        return result;
    }

    void* address = mmap(
        nullptr,
        sizeof(SharedLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);
    if (address == MAP_FAILED)
    {
        result.error_message = error_text("mmap existing failed");
        close(fd);
        return result;
    }

    auto* layout = static_cast<SharedLayout*>(address);
    if (layout->ready.load(std::memory_order_acquire) != 1 ||
        layout->identity.magic != kWireMagic ||
        layout->identity.wire_schema_version != kWireSchemaVersion)
    {
        result.error_message = "shared memory identity is not ready or incompatible";
        munmap(address, sizeof(SharedLayout));
        close(fd);
        return result;
    }

    result.memory.reset(new SharedMemory(name, fd, layout, false));
    return result;
}

SharedMemory::~SharedMemory()
{
    if (layout_ != nullptr)
    {
        if (owner_)
        {
            layout_->ready.store(0, std::memory_order_release);
        }
        munmap(layout_, sizeof(SharedLayout));
    }
    if (fd_ >= 0)
    {
        close(fd_);
    }
    if (owner_)
    {
        shm_unlink(name_.c_str());
    }
}

std::int64_t monotonic_now_ns() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // 命名空间 quadruped::ipc
