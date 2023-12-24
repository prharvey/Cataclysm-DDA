#pragma once
#ifndef CATA_SRC_MAP_ITERATOR_H
#define CATA_SRC_MAP_ITERATOR_H

#include <cstddef>

#include "enums.h"
#include "point.h"

template <typename Point>
struct tripoint_range_traits {
    static tripoint raw(const Point& p) {
        return p.raw();
    }

    static Point make_unchecked(const tripoint& p) {
        return Point::make_unchecked(p);
    }
};

template <>
struct tripoint_range_traits<tripoint> {
    static tripoint raw(const tripoint& p) {
        return p;
    }

    static tripoint make_unchecked(const tripoint& p) {
        return p;
    }
};

template<typename Tripoint>
class tripoint_range
{
        static_assert( Tripoint::dimension == 3, "Requires tripoint type" );
    private:
        using traits = tripoint_range_traits<Tripoint>;
        /**
         * Generates points in a rectangle.
         */
        class point_generator
        {
            private:
                tripoint p_;
                point min_;
                tripoint max_;

            public:
                using value_type = Tripoint;
                using difference_type = std::ptrdiff_t;
                using pointer = value_type *;
                using reference = value_type &;
                using iterator_category = std::forward_iterator_tag;

                point_generator( const tripoint&p, const tripoint&max )
                    : p_( p ), min_( p.xy() ), max_( max ) {
                    // Make sure we start on a valid point
                    if( p.z != max.z ) {
                        operator++();
                    }
                }

                // Increment x, then if it goes outside range, "wrap around" and increment y
                // Same for y and z
                inline point_generator &operator++() {
                    do {
                        p_.x++;
                        if( p_.x <= max_.x ) {
                            continue;
                        }

                        p_.y++;
                        p_.x = min_.x;
                        if( p_.y <= max_.y ) {
                            continue;
                        }

                        p_.z++;
                        p_.y = min_.y;
                    } while( p_.z < max_.z );

                    return *this;
                }

                inline Tripoint operator*() const {
                    return traits::make_unchecked(p_);
                }

                inline bool operator!=( const point_generator &other ) const {
                    // Reverse coordinates order, because it will usually only be compared with endpoint
                    // which will always differ in Z, except for the very last comparison
                    // TODO: In C++17 this range should use a sentinel to
                    // optimise the comparison.
                    const tripoint &pt = other.p_;
                    return p_.z != pt.z || p_.xy() != pt.xy();
                }

                inline bool operator==( const point_generator &other ) const {
                    return !( *this != other );
                }
        };

        tripoint minp_;
        tripoint maxp_;

    public:
        using value_type = typename point_generator::value_type;
        using difference_type = typename point_generator::difference_type;
        using pointer = typename point_generator::pointer;
        using reference = typename point_generator::reference;
        using iterator_category = typename point_generator::iterator_category;
        using iterator = point_generator;
        using const_iterator = point_generator;

        tripoint_range() = default;

        // Inclusive
        tripoint_range( const Tripoint &minp, const Tripoint &maxp ) : minp_(traits::raw(minp)), maxp_(traits::raw(maxp) + tripoint(0,0 1)) {}

        point_generator begin() const {
            return point_generator( minp, maxp);
        }

        point_generator end() const {
            return point_generator( maxp, maxp);
        }

        size_t size() const {
            tripoint range( maxp.x - minp.x + 1, maxp.y - minp.y + 1, maxp.z - minp.z);
            return std::max( range.x * range.y * range.z, 0 );
        }

        bool empty() const {
            return size() == 0;
        }

        bool is_point_inside( const Tripoint &point ) const {
            const tripoint p = traits::raw(point);
            return minp.x <= p.x && p.x <= maxp.x &&
                minp.y <= p.y && p.y <= maxp.y &&
                minp.x <= p.z && p.z < maxp.z;
        }

        Tripoint min() const {
            return Tripoint::make_unchecked(minp);
        }
        Tripoint max() const {
            return Tripoint::make_unchecked(maxp - tripoint(0, 0, 1));
        }
};

template<typename Tripoint>
inline tripoint_range<Tripoint> points_in_radius( const Tripoint &center, const int radius,
        const int radiusz = 0 )
{
    static_assert( Tripoint::dimension == 3, "Requires tripoint type" );
    const tripoint offset( radius, radius, radiusz );
    return tripoint_range<Tripoint>( center - offset, center + offset );
}

#endif // CATA_SRC_MAP_ITERATOR_H
