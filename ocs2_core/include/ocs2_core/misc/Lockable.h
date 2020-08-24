#pragma once

#include <memory>
#include <mutex>

namespace ocs2 {

/**
 * Grants access to the object T and the unique_ptr that holds it.
 * Holds the SynchronizedPtr's mutex for the lifetime of the object.
 */
template <typename T>
class LockedPtr {
 public:
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m) : p_(p), lk_(m) {}

  /// Access
  T* operator->() { return p_.get(); }
  const T* operator->() const { return p_.get(); }
  T& operator*() { return *p_; }
  const T& operator*() const { return *p_; }

  /// Resets the unique ptr within the SynchronizedPtr.
  void reset(std::unique_ptr<T> p) noexcept { p_ = std::move(p); }

  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  std::unique_ptr<T>& p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * Grants const access to the wrapped object T.
 * Holds the SynchronizedPtr's mutex for the lifetime of the object.
 */
template <typename T>
class LockedConstPtr {
 public:
  LockedConstPtr(const T* p, std::mutex& m) : p_(p), lk_(m) {}

  /// const access
  const T* operator->() const { return p_; }
  const T& operator*() const { return *p_; }

  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  const T* p_;
  std::unique_lock<std::mutex> lk_;
};

template <typename T>
class Synchronized {
 public:
  /// Default constructor : Owns nothing
  Synchronized() = default;

  /// Construct from unique_ptr
  explicit Synchronized(std::unique_ptr<T> p) noexcept : p_(std::move(p)) {}

  /// Destructor : Obtains the lock and destroys the owned object.
  ~Synchronized() {
    std::unique_lock<std::mutex> lk(m_);
    p_.reset();
  }

  // Disable copy operations
  Synchronized(const Synchronized&) = delete;
  Synchronized& operator=(const Synchronized&) = delete;

  /// Move construction
  Synchronized(Synchronized&& other) noexcept {
    // Current object is being created, no need to lock this->m_
    std::unique_lock<std::mutex> lkOther(other.m_);
    p_ = std::move(other.p_);
  }

  /// Move assignment
  Synchronized& operator=(Synchronized&& other) noexcept {
    // Both other and this object already exist. Need to lock both
    std::unique_lock<std::mutex> lkThis(m_, std::defer_lock);
    std::unique_lock<std::mutex> lkOther(other.m_, std::defer_lock);
    std::lock(lkOther, lkThis);
    p_ = std::move(other.p_);
    return *this;
  }

  /// reset the wrapped object. The object is reseated while holding the lock.
  void reset(std::unique_ptr<T> p) noexcept {
    std::unique_lock<std::mutex> lk(m_);
    p_ = std::move(p);
  }

  /// Returns a pointer that holds the lock to the wrapped object. Lifetime of the LockedPtr<> determines the lifetime of the lock.
  LockedPtr<T> lock() { return {p_, m_}; }
  LockedConstPtr<T> lock() const { return {p_.get(), m_}; }

  /// Apply operations directly on the held object. Holds the lock for the duration of the call.
  LockedPtr<T> operator->() { return this->lock(); }
  LockedConstPtr<T> operator->() const { return this->lock(); }

  /// Get direct access to the wrapped value. NOT thread safe.
  T* getUnsafe() { return p_.get(); }

  /// Get direct access to the internal mutex.
  std::mutex& getMutex() { return m_; }

 private:
  mutable std::mutex m_;
  std::unique_ptr<T> p_;
};

}  // namespace ocs2
