#pragma once
#ifndef CATA_SRC_MEMORY_FAST_H
#define CATA_SRC_MEMORY_FAST_H

#include <memory>
#include <type_traits>

#if __GLIBCXX__
template<typename T> using shared_ptr_fast = std::__shared_ptr<T, __gnu_cxx::_S_single>;
template<typename T> using weak_ptr_fast = std::__weak_ptr<T, __gnu_cxx::_S_single>;
template<typename T, typename... Args> shared_ptr_fast<T> make_shared_fast(
    Args &&... args )
{
    return std::__make_shared<T, __gnu_cxx::_S_single>( args... );
}
#elif defined(_MSC_VER)

struct RefcountBase {
    long use_count = 0;
    long weak_count = 1;
    virtual ~RefcountBase() = default;
    virtual void delete_ptr() = 0;
};

template <typename Deleter>
struct Refcount : public RefcountBase {
    Deleter deleter;

    explicit Refcount(Deleter d) : deleter(std::move(d)) {}

    void delete_ptr() final {
        deleter();
    }
};

template <typename Deleter>
RefcountBase* make_refcount(Deleter d) {
    return new Refcount<Deleter>(std::move(d));
}

struct Marker {};

template <typename T>
class weak_ptr_fast;

template <typename T>
class shared_ptr_fast
{
    public:
        constexpr shared_ptr_fast() noexcept : ptr_( nullptr ), counts_( nullptr ) {}

        constexpr shared_ptr_fast( std::nullptr_t ) noexcept : shared_ptr_fast() {}

        template <typename U>
        explicit shared_ptr_fast(U* ptr) : shared_ptr_fast(ptr, std::default_delete<U>()) {}

        template< class U, class Deleter, typename = std::enable_if_t<std::is_invocable_v<Deleter, U*>> >
        shared_ptr_fast(U* ptr, Deleter d) : shared_ptr_fast(ptr, make_refcount([ptr, d]() { d(ptr); }), Marker{}) {}

        template <typename U>
        shared_ptr_fast(const shared_ptr_fast<U>& other, T* alias) noexcept : shared_ptr_fast(alias, other.counts_, Marker{}) {}

        shared_ptr_fast( const shared_ptr_fast &other ) noexcept : shared_ptr_fast( other.ptr_,
                    other.counts_, Marker{}) {}

        template <typename U>
        shared_ptr_fast( const shared_ptr_fast<U> &other ) noexcept : shared_ptr_fast( other.ptr_,
                    other.counts_, Marker{}) {}

        shared_ptr_fast( shared_ptr_fast &&other ) noexcept : shared_ptr_fast() {
            swap( other );
        }

        template <typename U>
        shared_ptr_fast( shared_ptr_fast<U> &&other ) noexcept : shared_ptr_fast( other.ptr_,
            other.counts_, Marker{}) {
        }

        ~shared_ptr_fast() {
            if (counts_ && --counts_->use_count == 0) {
                counts_->delete_ptr();
                if (--counts_->weak_count == 0) {
                    delete counts_;
                }
            }
        }

        explicit operator bool() const noexcept {
            return ptr_;
        }

        shared_ptr_fast& operator=(shared_ptr_fast other) noexcept {
            swap(other);
            return *this;
        }

        T *get() const {
            return ptr_;
        }

        T *operator->() const {
            return get();
        }

        T &operator*() const {
            return *get();
        }

        long use_count() const noexcept {
            if (counts_) {
                return counts_->use_count;
            }
            return 0;
        }

        void reset() noexcept {
            shared_ptr_fast().swap(*this);
        }

        template <typename U>
        void reset(U* other) noexcept {
            shared_ptr_fast(other).swap(*this);
        }

        void swap( shared_ptr_fast &other ) noexcept {
            std::swap( ptr_, other.ptr_ );
            std::swap( counts_, other.counts_ );
        }

    private:
        shared_ptr_fast(T* ptr, RefcountBase* counts, Marker) : ptr_(ptr), counts_(counts) {
            if (counts_) {
                ++counts_->use_count;
            }
        }

        T *ptr_;
        RefcountBase *counts_;

        template <typename U>
        friend class weak_ptr_fast;

        template <typename U>
        friend class shared_ptr_fast;
};

template< class T, class U >
bool operator==( const shared_ptr_fast<T> &lhs, const shared_ptr_fast<U> &rhs ) noexcept
{
    return lhs.get() == rhs.get();
}

template< class T, class U >
bool operator!=( const shared_ptr_fast<T> &lhs, const shared_ptr_fast<U> &rhs ) noexcept
{
    return !( lhs == rhs );
}

template< class T >
bool operator==(const shared_ptr_fast<T>& lhs, std::nullptr_t ) noexcept
{
    return !lhs.get();
}

template< class T >
bool operator==(std::nullptr_t, const shared_ptr_fast<T>& rhs) noexcept
{
    return rhs == nullptr;
}

template< class T >
bool operator!=(const shared_ptr_fast<T>& lhs, std::nullptr_t) noexcept
{
    return !(lhs == nullptr);
}

template< class T >
bool operator!=(std::nullptr_t, const shared_ptr_fast<T>& rhs) noexcept
{
    return !(nullptr == rhs);
}

template< class T, class U >
bool operator<(const shared_ptr_fast<T>& lhs, const shared_ptr_fast<U>& rhs) noexcept {
    return lhs.get() < rhs.get();
}
template< class T, class U >
bool operator>(const shared_ptr_fast<T>& lhs, const shared_ptr_fast<U>& rhs) noexcept  { return rhs < lhs; }
template< class T, class U >
bool operator<=(const shared_ptr_fast<T>& lhs, const shared_ptr_fast<U>& rhs) noexcept  { return !(lhs > rhs); }
template< class T, class U >
bool operator>=(const shared_ptr_fast<T>& lhs, const shared_ptr_fast<U>& rhs) noexcept  { return !(lhs < rhs); }

template <typename T>
void swap( shared_ptr_fast<T> &a, shared_ptr_fast<T> &b ) noexcept
{
    a.swap( b );
}

template <typename T>
class weak_ptr_fast
{
    public:
        constexpr weak_ptr_fast() noexcept : ptr_( nullptr ), counts_( nullptr ) {}

        constexpr weak_ptr_fast( std::nullptr_t ) noexcept : weak_ptr_fast() {}

        weak_ptr_fast( const weak_ptr_fast &other ) noexcept : weak_ptr_fast( other.ptr_, other.counts_ ) {}

        template <typename U>
        weak_ptr_fast(const weak_ptr_fast<U>& other) noexcept : weak_ptr_fast(other.ptr_, other.counts_) {}

        weak_ptr_fast( weak_ptr_fast &&other ) noexcept : weak_ptr_fast() {
            swap( other );
        }

        template <typename U>
        weak_ptr_fast(weak_ptr_fast<U>&& other) noexcept : weak_ptr_fast(other.ptr_, other.counts_) {}

        template <typename U>
        weak_ptr_fast(const shared_ptr_fast<U>& other) noexcept : weak_ptr_fast(other.ptr_,
            other.counts_) { }

        weak_ptr_fast &operator=( weak_ptr_fast other ) noexcept {
            swap( other );
            return *this;
        }

        ~weak_ptr_fast() {
            if (counts_ && --counts_->weak_count == 0) {
                delete counts_;
            }
        }

        void reset() noexcept {
            weak_ptr_fast().swap(*this);
        }

        bool expired() const noexcept {
            return !counts_ || counts_->use_count == 0;
        }

        shared_ptr_fast<T> lock() const noexcept {
            if( !expired() ) {
                return shared_ptr_fast<T>(ptr_, counts_, Marker{});
            }
            return shared_ptr_fast<T>();
        }

        void swap( weak_ptr_fast &other ) noexcept {
            std::swap( ptr_, other.ptr_ );
            std::swap( counts_, other.counts_ );
        }

    private:
        weak_ptr_fast( T *ptr, RefcountBase* counts ) : ptr_( ptr ), counts_( counts ) {
            if( counts_ ) {
                ++counts_->weak_count;
            }
        }

        T *ptr_;
        RefcountBase* counts_;

        template <typename U>
        friend class weak_ptr_fast;
};

template <typename T>
void swap( weak_ptr_fast<T> &a, weak_ptr_fast<T> &b ) noexcept
{
    a.swap( b );
}

template<class T, class U>
shared_ptr_fast<T> dynamic_pointer_cast(const shared_ptr_fast<U>& other) noexcept
{
    if (T* alias = dynamic_cast<T*>(other.get())) {
        return shared_ptr_fast<T>(other, alias);
    }
    return shared_ptr_fast<T>();
}

template<typename T, typename... Args>
shared_ptr_fast<T> make_shared_fast( Args &&... args )
{
    return shared_ptr_fast<T>( new T( std::forward<Args>( args )... ) );
}

#else
template<typename T> using shared_ptr_fast = std::shared_ptr<T>;
template<typename T> using weak_ptr_fast = std::weak_ptr<T>;
template<typename T, typename... Args> shared_ptr_fast<T> make_shared_fast(
    Args &&... args )
{
    return std::make_shared<T>( args... );
}
#endif

#endif // CATA_SRC_MEMORY_FAST_H
