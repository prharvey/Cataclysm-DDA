#pragma once
#ifndef CATA_SRC_PATHFINDING_H
#define CATA_SRC_PATHFINDING_H

#include "coordinates.h"
#include "game_constants.h"
#include "mapdata.h"
#include "mdarray.h"
#include "type_id.h"

class map;

template <typename State, typename Cost = int, typename VisitedSet = std::unordered_set<State>, typename BestStateMap = std::unordered_map<State, std::pair<Cost, State>>>
          class AStarPathfinder
{
public:
    template <typename NeighborsFn, typename CostFn, typename HeuristicFn>
    std::vector<State> find_path( const Cost &max, const State &from, const State &to,
                                  NeighborsFn neighbors_fn,
                                  CostFn cost_fn, HeuristicFn heuristic_fn );

private:
    VisitedSet visited_;
    BestStateMap best_state_;
};


enum class PathfindingFlag : uint8_t {
    Ground = 0,     // Can walk on
    Slow,           // Move cost > 2
    Swimmable,      // Can swim in
    Air,            // Empty air
    Unsheltered,    // Outside and above ground level
    Obstacle,       // Something stopping us, might be bashable.
    Impassable,     // Impassable obstacle.
    Vehicle,        // Vehicle tile (passable or not)
    DangerousField, // Dangerous field
    DangerousTrap,  // Dangerous trap (i.e. not flagged benign)
    GoesUp,         // Valid stairs up
    GoesDown,       // Valid stairs down
    RampUp,         // Valid ramp up
    RampDown,       // Valid ramp down
    Climmable,      // Impassable but can be climbed on examine
    Sharp,          // Sharp items (barbed wire, etc)
    Door,           // A door (any kind)
    InsideDoor,     // A door that can be opened from the inside only
    LockedDoor,     // A locked door
    Pit,            // A pit you can fall into / climb out of.
    DeepWater,      // Deep water.
    Burrowable,     // Can burrow into
    HardGround,     // Can not dig & burrow into
    SmallPassage,   // Small passage for a small creature.
    Lava,           // Lava terrain
};

class PathfindingFlags
{
    public:
        constexpr PathfindingFlags() = default;
        constexpr PathfindingFlags( PathfindingFlag flag ) : flags_( uint32_t{ 1 } << static_cast<uint8_t>
                    ( flag ) ) {}

        constexpr void set_union( PathfindingFlags flags ) {
            flags_ |= flags.flags_;
        }
        constexpr void set_intersect( PathfindingFlags flags ) {
            flags_ &= flags.flags_;
        }
        constexpr void set_clear( PathfindingFlags flags ) {
            flags_ &= ~flags.flags_;
        }

        constexpr bool is_set( PathfindingFlag flag ) const {
            return flags_ & ( uint32_t{ 1 } << static_cast<uint8_t>( flag ) );
        }
        constexpr bool is_set( PathfindingFlags flags ) const {
            return ( flags_ & flags.flags_ ) == flags.flags_;
        }
        constexpr bool is_any_set() const {
            return flags_;
        }

        constexpr operator bool() const {
            return is_any_set();
        }

        constexpr PathfindingFlags &operator|=( PathfindingFlags flags ) {
            set_union( flags );
            return *this;
        }

        constexpr PathfindingFlags &operator&=( PathfindingFlags flags ) {
            set_intersect( flags );
            return *this;
        }

    private:
        uint32_t flags_ = 0;
};

constexpr PathfindingFlags operator|( PathfindingFlags lhs, PathfindingFlags rhs )
{
    return lhs |= rhs;
}

constexpr PathfindingFlags operator&( PathfindingFlags lhs, PathfindingFlags rhs )
{
    return lhs &= rhs;
}

constexpr PathfindingFlags operator|( PathfindingFlags lhs, PathfindingFlag rhs )
{
    return lhs |= rhs;
}

constexpr PathfindingFlags operator&( PathfindingFlags lhs, PathfindingFlag rhs )
{
    return lhs &= rhs;
}

// Note that this is in reverse order for memory locality: z, y, x.
template <typename T>
using RealityBubbleArray =
std::array<std::array<std::array<T, MAPSIZE_X>, MAPSIZE_Y>, OVERMAP_LAYERS>;

class RealityBubblePathfindingCache
{
    public:
        RealityBubblePathfindingCache();

        PathfindingFlags flags( const tripoint_bub_ms &p ) const {
            return flag_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        int move_cost( const tripoint_bub_ms &p ) const {
            return move_cost_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        const std::pair<int, int> &bash_range( const tripoint_bub_ms &p ) const {
            return bash_range_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        const tripoint_bub_ms &up_destination( const tripoint_bub_ms &p ) const {
            return up_destinations_.find( p )->second;
        }

        const tripoint_bub_ms &down_destination( const tripoint_bub_ms &p ) const {
            return down_destinations_.find( p )->second;
        }

        void update( const map &here );

        void invalidate( int z ) {
            dirty_z_levels_.emplace( z );
        }

        void invalidate( const tripoint_bub_ms &p ) {
            dirty_positions_[p.z()].emplace( p.xy() );
            invalidate_dependants( p );
        }

    private:
        PathfindingFlags &flags_ref( const tripoint_bub_ms &p ) {
            return flag_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        char &move_cost_ref( const tripoint_bub_ms &p ) {
            return move_cost_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        std::pair<int, int> &bash_range_ref( const tripoint_bub_ms &p ) {
            return bash_range_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        bool vertical_move_destination( const map &here, ter_furn_flag flag, tripoint &t ) const;

        void invalidate_dependants( const tripoint_bub_ms &p );

        void update( const map &here, const tripoint_bub_ms &p );

        std::unordered_set<int> dirty_z_levels_;
        std::unordered_map<int, std::unordered_set<point_bub_ms>> dirty_positions_;
        std::unordered_map<tripoint_bub_ms, std::vector<tripoint_bub_ms>> dependants_by_position_;

        std::unordered_map<tripoint_bub_ms, tripoint_bub_ms> up_destinations_;
        std::unordered_map<tripoint_bub_ms, tripoint_bub_ms> down_destinations_;
        RealityBubbleArray<PathfindingFlags> flag_cache_;
        RealityBubbleArray<char> move_cost_cache_;
        RealityBubbleArray<std::pair<int, int>> bash_range_cache_;
};

class RealityBubblePathfindingSettings
{
    public:
        bool allow_flying() const {
            return allow_flying_;
        }
        void set_allow_flying( bool v = true ) {
            allow_flying_ = v;
        }

        bool allow_stairways() const {
            return allow_stairways_;
        }
        void set_allow_stairways( bool v = true ) {
            allow_stairways_ = v;
        }

        int max_cost() const {
            return max_cost_;
        }
        void set_max_cost( int v = 0 ) {
            max_cost_ = v;
        }

        PathfindingFlags &avoid_mask() {
            return avoid_mask_;
        }
        const PathfindingFlags &avoid_mask() const {
            return avoid_mask_;
        }

    private:
        bool allow_flying_ = false;
        bool allow_stairways_ = false;
        int max_cost_ = 0;
        PathfindingFlags avoid_mask_ = PathfindingFlag::Air | PathfindingFlag::Impassable;
};

class RealityBubblePathfinder
{
    public:
        // The class uses a lot of memory, and is safe to reuse as long as none of the provided
        // functors to find_path make use of it.
        explicit RealityBubblePathfinder( RealityBubblePathfindingCache *cache ) : cache_( cache ) {}

        template <typename CostFn, typename HeuristicFn>
        std::vector<tripoint_bub_ms> find_path( const RealityBubblePathfindingSettings &settings,
                                                const tripoint_bub_ms &from, const tripoint_bub_ms &to,
                                                CostFn cost_fn, HeuristicFn heuristic_fn );

    private:
        using X = unsigned char;
        using Y = unsigned char;
        using Z = char;

        struct PackedTripoint {
            X x;
            Y y;
            Z z;

            PackedTripoint() = default;

            constexpr PackedTripoint( X x, Y y, Z z ) : x( x ), y( y ), z( z ) {}

            constexpr PackedTripoint( const tripoint_bub_ms &p ) : x( p.x() ), y( p.y() ), z( p.z() ) {}

            constexpr operator tripoint_bub_ms() const {
                return tripoint_bub_ms( static_cast<int>( x ), static_cast<int>( y ), static_cast<int>( z ) );
            }

            constexpr bool operator==( PackedTripoint other ) const {
                return x == other.x && y == other.y && z == other.z;
            }

            constexpr bool operator!=( PackedTripoint other ) const {
                return !( *this == other );
            }
        };

        template <typename T, typename I, I kMax>
        struct FastIntSet {
            bool emplace( T p );

            void clear() {
                in.clear();
            }

            auto begin() const {
                return in.begin();
            }

            auto end() const {
                return in.end();
            }

            std::size_t count( T p ) const;

            std::vector<T> in;
            std::array<I, kMax> visited;
        };

        struct FastTripointSet {
            static constexpr int get_index( PackedTripoint p ) {
                constexpr int layer_size = MAPSIZE_X * MAPSIZE_Y;
                return ( p.z + OVERMAP_DEPTH ) * layer_size + p.y * MAPSIZE_X + p.x;
            }

            bool emplace( PackedTripoint p ) {
                const int i = get_index( p );
                return in_.emplace( i );
            }

            void clear() {
                in_.clear();
            }

            std::size_t count( PackedTripoint p ) const {
                const int i = get_index( p );
                return in_.count( i );
            }

            FastIntSet<int, std::uint32_t, MAPSIZE_X *MAPSIZE_Y *OVERMAP_LAYERS> in_;
        };

        struct FastTripointSet2 {
            static constexpr std::size_t kBits = MAPSIZE_X * MAPSIZE_Y * OVERMAP_LAYERS;
            static constexpr std::size_t kWords = ( kBits / 64 ) + 1;
            static constexpr std::uint64_t kBitMask = 64 - 1;

            static constexpr std::uint16_t get_word( std::uint64_t index ) {
                return index / 64;
            }

            static constexpr std::uint64_t get_bit( std::uint64_t index ) {
                return index & kBitMask;
            }

            static constexpr int get_index( PackedTripoint p ) {
                constexpr int layer_size = MAPSIZE_X * MAPSIZE_Y;
                return ( p.z + OVERMAP_DEPTH ) * layer_size + p.y * MAPSIZE_X + p.x;
            }

            bool emplace( PackedTripoint p ) {
                const std::uint64_t i = get_index( p );
                const std::uint16_t word = get_word( i );
                if( used_words_.emplace( word ) ) {
                    words_[word] = 0;
                }
                const std::uint64_t bit = get_bit( i );
                const std::uint64_t mask = std::uint64_t{ 1 } << bit;
                const bool has_bit = words_[word] & mask;
                words_[word] |= mask;
                return !has_bit;
            }

            void clear() {
                used_words_.clear();
            }

            std::size_t count( PackedTripoint p ) const {
                const std::uint64_t i = get_index( p );
                const std::uint16_t word = get_word( i );
                if( used_words_.count( word ) == 1 ) {
                    const std::uint64_t bit = get_bit( i );
                    const bool has_bit = words_[word] & ( std::uint64_t{ 1 } << bit );
                    return has_bit;
                }
                return 0;
            }

            FastIntSet<std::uint16_t, std::uint16_t, kWords> used_words_;
            std::array<std::uint64_t, kWords> words_;
        };

        struct FastTripointSet3 {
            static constexpr std::size_t kBits = MAPSIZE_X * MAPSIZE_Y * OVERMAP_LAYERS;
            static constexpr std::size_t kWords = ( kBits / 64 ) + 1;
            static constexpr std::uint64_t kBitMask = 64 - 1;

            static constexpr std::uint16_t get_word( std::uint64_t index ) {
                return index / 64;
            }

            static constexpr std::uint64_t get_bit( std::uint64_t index ) {
                return index & kBitMask;
            }

            static constexpr int get_index( PackedTripoint p ) {
                constexpr int layer_size = MAPSIZE_X * MAPSIZE_Y;
                return ( p.z + OVERMAP_DEPTH ) * layer_size + p.y * MAPSIZE_X + p.x;
            }

            bool emplace( PackedTripoint p ) {
                const std::uint64_t i = get_index( p );
                const std::uint16_t word = get_word( i );
                used_words_.emplace( word );
                const std::uint64_t bit = get_bit( i );
                const std::uint64_t mask = std::uint64_t{ 1 } << bit;
                const bool has_bit = words_[word] & mask;
                words_[word] |= mask;
                return !has_bit;
            }

            void clear() {
                for( std::uint16_t i : used_words_ ) {
                    words_[i] = 0;
                }
                used_words_.clear();
            }

            std::size_t count( PackedTripoint p ) const {
                const std::uint64_t i = get_index( p );
                const std::uint16_t word = get_word( i );
                const std::uint64_t bit = get_bit( i );
                const bool has_bit = words_[word] & ( std::uint64_t{ 1 } << bit );
                return has_bit;
            }

            FastIntSet<std::uint16_t, std::uint16_t, kWords> used_words_;
            std::array<std::uint64_t, kWords> words_;
        };

        struct FastTripointSet4 {
            static constexpr std::uint64_t kBitMask = 64 - 1;

            static constexpr std::size_t get_word( std::uint64_t index ) {
                return index / 64;
            }

            static constexpr std::uint64_t get_bit( std::uint64_t index ) {
                return index & kBitMask;
            }

            bool emplace( PackedTripoint p ) {
                const int z = p.z + OVERMAP_DEPTH;
                dirty_[z] = true;
                std::uint64_t &word = words_[z][p.y][get_word( p.x )];
                const std::uint64_t bit = get_bit( p.x );
                const std::uint64_t mask = std::uint64_t{ 1 } << bit;
                const bool has_bit = word & mask;
                word |= mask;
                return !has_bit;
            }

            void clear() {
                for( int z = 0; z < OVERMAP_HEIGHT; ++z ) {
                    if( !dirty_[z] ) {
                        continue;
                    }
                    dirty_[z] = false;
                    for( int y = 0; y < MAPSIZE_Y; ++y ) {
                        for( int x = 0; x < 3; ++x ) {
                            words_[z][y][x] = 0;
                        }
                    }
                }
            }

            std::size_t count( PackedTripoint p ) const {
                const int z = p.z + OVERMAP_DEPTH;
                const std::uint64_t word = words_[z][p.y][get_word( p.x )];
                const std::uint64_t bit = get_bit( p.x );
                const std::uint64_t mask = std::uint64_t{ 1 } << bit;
                return word & mask;
            }

            std::array<bool, OVERMAP_LAYERS> dirty_;
            std::array<std::array<std::array<std::uint64_t, 3>, MAPSIZE_Y>, OVERMAP_LAYERS> words_;
        };

        struct FastTripointSet5 {
            bool emplace( PackedTripoint p ) {
                const int z = p.z + OVERMAP_DEPTH;
                dirty_[z] = true;
                return !std::exchange( is_set_[z][p.y][p.x], true );
            }

            void clear() {
                for( int z = 0; z < OVERMAP_HEIGHT; ++z ) {
                    if( !dirty_[z] ) {
                        continue;
                    }
                    dirty_[z] = false;
                    for( int y = 0; y < MAPSIZE_Y; ++y ) {
                        for( int x = 0; x < MAPSIZE_X; ++x ) {
                            is_set_[z][y][x] = false;
                        }
                    }
                }
            }

            std::size_t count( PackedTripoint p ) const {
                return is_set_[p.z + OVERMAP_DEPTH][p.y][p.x];
            }

            std::array<bool, OVERMAP_LAYERS> dirty_;
            std::array<std::array<std::array<bool, MAPSIZE_X>, MAPSIZE_Y>, OVERMAP_LAYERS> is_set_;
        };

        struct FastBestStateMap {
            std::pair<std::pair<int, PackedTripoint>*, bool> try_emplace( PackedTripoint child,
                    int cost, PackedTripoint parent ) {
                std::pair<int, PackedTripoint> &result = best_states[child.z +
                        OVERMAP_DEPTH][child.y][child.x];
                if( in.emplace( child ) ) {
                    result.first = cost;
                    result.second = parent;
                    return std::make_pair( &result, true );
                }
                return std::make_pair( &result, false );
            }

            void clear() {
                in.clear();
            }

            std::pair<int, PackedTripoint> &operator[]( PackedTripoint child ) {
                return *try_emplace( child, 0, child ).first;
            }

            FastTripointSet4 in;
            RealityBubbleArray<std::pair<int, PackedTripoint>> best_states;
        };

        RealityBubblePathfindingCache *cache_;
        AStarPathfinder<PackedTripoint, int, FastTripointSet4, FastBestStateMap> astar_;
};

class PathfindingSettings
{
    public:
        static constexpr PathfindingFlags RoughTerrain = PathfindingFlag::Slow | PathfindingFlag::Obstacle |
                PathfindingFlag::Vehicle | PathfindingFlag::Sharp | PathfindingFlag::DangerousTrap;

        bool avoid_air() const {
            return is_set( PathfindingFlag::Air );
        }
        void set_avoid_air( bool v = true ) {
            set( PathfindingFlag::Air, v );
        }

        bool avoid_unsheltered() const {
            return is_set( PathfindingFlag::Unsheltered );
        }
        void set_avoid_unsheltered( bool v = true ) {
            set( PathfindingFlag::Unsheltered, v );
        }

        bool avoid_swimming() const {
            return is_set( PathfindingFlag::Swimmable );
        }
        void set_avoid_swimming( bool v = true ) {
            set( PathfindingFlag::Swimmable, v );
        }

        bool avoid_ground() const {
            return is_set( PathfindingFlag::Ground );
        }
        void set_avoid_ground( bool v = true ) {
            set( PathfindingFlag::Ground, v );
        }

        bool avoid_vehicle() const {
            return is_set( PathfindingFlag::Vehicle );
        }
        void set_avoid_vehicle( bool v = true ) {
            set( PathfindingFlag::Vehicle, v );
        }

        bool avoid_climbing() const {
            return avoid_climbing_;
        }
        void set_avoid_climbing( bool v = true ) {
            avoid_climbing_ = v;
            maybe_set_avoid_obstacle();
        }

        bool avoid_climb_stairway() const {
            return !rb_settings_.allow_stairways();
        }
        void set_avoid_climb_stairway( bool v = true ) {
            rb_settings_.set_allow_stairways( !v );
        }

        bool avoid_deep_water() const {
            return is_set( PathfindingFlag::DeepWater );
        }
        void set_avoid_deep_water( bool v = true ) {
            set( PathfindingFlag::DeepWater, v );
        }

        bool avoid_small_passages() const {
            return is_set( PathfindingFlag::SmallPassage );
        }
        void set_avoid_small_passages( bool v = true ) {
            set( PathfindingFlag::SmallPassage, v );
        }

        bool avoid_pits() const {
            return is_set( PathfindingFlag::Pit );
        }
        void set_avoid_pits( bool v = true ) {
            set( PathfindingFlag::Pit, v );
        }

        bool avoid_opening_doors() const {
            return avoid_opening_doors_;
        }
        void set_avoid_opening_doors( bool v = true ) {
            avoid_opening_doors_ = v;
            maybe_set_avoid_obstacle();
        }

        bool avoid_unlocking_doors() const {
            return avoid_unlocking_doors_;
        }
        void set_avoid_unlocking_doors( bool v = true ) {
            avoid_unlocking_doors_ = v;
        }

        bool avoid_rough_terrain() const {
            return is_set( RoughTerrain );
        }
        void set_avoid_rough_terrain( bool v = true ) {
            set( RoughTerrain, v );
        }

        bool avoid_dangerous_traps() const {
            return is_set( PathfindingFlag::DangerousTrap );
        }
        void set_avoid_dangerous_traps( bool v ) {
            set( PathfindingFlag::DangerousTrap, v );
        }

        bool avoid_sharp() const {
            return is_set( PathfindingFlag::Sharp );
        }
        void set_avoid_sharp( bool v = true ) {
            set( PathfindingFlag::Sharp, v );
        }

        bool avoid_lava() const {
            return is_set( PathfindingFlag::Lava );
        }
        void set_avoid_lava( bool v = true ) {
            set( PathfindingFlag::Lava, v );
        }

        bool avoid_hard_ground() const {
            return is_set( PathfindingFlag::HardGround );
        }
        void set_avoid_hard_ground( bool v = true ) {
            set( PathfindingFlag::HardGround, v );
        }

        const std::function<bool( const field_type_id & )> &maybe_avoid_dangerous_fields_fn() const {
            return maybe_avoid_dangerous_fields_fn_;
        }
        void set_maybe_avoid_dangerous_fields_fn( std::function<bool( const field_type_id & )> fn =
                    nullptr ) {
            maybe_avoid_dangerous_fields_fn_ = std::move( fn );
        }

        const std::function<bool( const tripoint_bub_ms & )> &maybe_avoid_fn() const {
            return maybe_avoid_fn_;
        }
        void set_maybe_avoid_fn( std::function<bool( const tripoint_bub_ms & )> fn = nullptr ) {
            maybe_avoid_fn_ = std::move( fn );
        }

        int max_distance() const {
            return max_distance_;
        }
        void set_max_distance( int v = 0 ) {
            max_distance_ = v;
        }

        int max_cost() const {
            return rb_settings_.max_cost();
        }
        void set_max_cost( int v = 0 ) {
            rb_settings_.set_max_cost( v );
        }

        int climb_cost() const {
            return climb_cost_;
        }
        void set_climb_cost( int v = 0 ) {
            climb_cost_ = v;
            maybe_set_avoid_obstacle();
        }

        bool is_digging() const {
            return is_digging_;
        }
        void set_is_digging( bool v = true ) {
            is_digging_ = v;
            maybe_set_avoid_obstacle();
        }

        bool is_flying() const {
            return rb_settings_.allow_flying();
        }
        void set_is_flying( bool v = true ) {
            rb_settings_.set_allow_flying( v );
        }

        PathfindingFlags avoid_mask() const {
            return rb_settings_.avoid_mask();
        }

        int bash_strength() const {
            return bash_strength_;
        }
        void set_bash_strength( int v = 0 ) {
            bash_strength_ = v;
            maybe_set_avoid_obstacle();
        }
        int bash_rating_from_range( int min, int max ) const;

    protected:
        const RealityBubblePathfindingSettings &rb_settings() const {
            return rb_settings_;
        }

        friend class map;

    private:
        bool is_set( PathfindingFlags flag ) const {
            return rb_settings_.avoid_mask().is_set( flag );
        }
        void set( PathfindingFlags flags, bool v ) {
            if( v ) {
                rb_settings_.avoid_mask().set_union( flags );
            } else {
                rb_settings_.avoid_mask().set_clear( flags );
            }
        }

        void maybe_set_avoid_obstacle() {
            // Check if we can short circuit checking obstacles. Significantly faster if so.
            set( PathfindingFlag::Obstacle, !is_digging() && climb_cost() <= 0 && avoid_climbing() &&
                 avoid_opening_doors() && bash_strength() <= 0 );
        }

        int max_distance_ = 0;

        int bash_strength_ = 0;

        // Expected terrain cost (2 is flat ground) of climbing a wire fence
        // 0 means no climbing
        int climb_cost_ = 0;
        bool is_digging_ = false;

        RealityBubblePathfindingSettings rb_settings_;

        bool avoid_climbing_ = false;
        bool avoid_opening_doors_ = false;
        bool avoid_unlocking_doors_ = false;
        std::function<bool( const field_type_id & )> maybe_avoid_dangerous_fields_fn_;
        std::function<bool( const tripoint_bub_ms & )> maybe_avoid_fn_;
};

// Implementation Details

struct FirstElementGreaterThan {
    template <typename T, typename... Ts>
    bool operator()( const std::tuple<T, Ts...> &lhs, const std::tuple<T, Ts...> &rhs ) const {
        return std::get<0>( lhs ) > std::get<0>( rhs );
    }
};

template <typename State, typename Cost, typename VisitedSet, typename BestStateMap>
template <typename NeighborsFn, typename CostFn, typename HeuristicFn>
std::vector<State> AStarPathfinder<State, Cost, VisitedSet, BestStateMap>::find_path(
    const Cost &max_cost, const State &from, const State &to, NeighborsFn neighbors_fn, CostFn cost_fn,
    HeuristicFn heuristic_fn )
{
    using FrontierNode = std::tuple<Cost, State>;
    std::priority_queue< FrontierNode, std::vector< FrontierNode>, FirstElementGreaterThan> frontier;
    std::vector<State> result;

    visited_.clear();
    best_state_.clear();

    best_state_.try_emplace( from, 0, from );
    frontier.emplace( heuristic_fn( from ), from );
    do {
        auto [estimated_cost, current_state] = frontier.top();
        frontier.pop();

        if( estimated_cost >= max_cost ) {
            break;
        }

        if( !visited_.emplace( current_state ) ) {
            continue;
        }

        if( current_state == to ) {
            while( current_state != from ) {
                result.push_back( current_state );
                current_state = best_state_[current_state].second;
            }
            std::reverse( result.begin(), result.end() );
            break;
        }

        const Cost current_cost = best_state_[current_state].first;
        neighbors_fn( current_state, [this, &frontier, &cost_fn, &heuristic_fn, &current_state,
              current_cost]( const State & neighbour ) {
            if( visited_.count( neighbour ) == 1 ) {
                return;
            }
            /*if (current_cost >= best_cost) {
                // Can't possibly do better than we've already seen, no matter what the cost
                // function says.
                return;
            }*/
            if( const std::optional<Cost> transition_cost = cost_fn( current_state, neighbour ) ) {
                const Cost new_cost = current_cost + *transition_cost;
                const auto& [iter, _] = best_state_.try_emplace( neighbour, std::numeric_limits<Cost>::max(),
                                        State() );
                auto& [best_cost, parent] = *iter;
                if( new_cost < best_cost ) {
                    best_cost = new_cost;
                    parent = current_state;
                    const Cost estimated_cost = new_cost + heuristic_fn( neighbour );
                    frontier.emplace( estimated_cost, neighbour );
                }
            }
        } );
    } while( !frontier.empty() );
    return result;
}

template <typename T, typename I, I kMax>
inline bool RealityBubblePathfinder::FastIntSet<T, I, kMax>::emplace( T i )
{
    const I test = visited[i];
    if( test >= in.size() || in[test] != i ) {
        visited[i] = in.size();
        in.push_back( i );
        return true;
    }
    return false;
}

template <typename T, typename I, I kMax>
inline std::size_t RealityBubblePathfinder::FastIntSet<T, I, kMax>::count( T i ) const
{
    const I test = visited[i];
    return test < in.size() && in[test] == i;
}

/*
inline std::pair<int, tripoint_bub_ms> &RealityBubblePathfinder::FastBestStateMap::operator[](
    const tripoint_bub_ms &child )
{
    std::pair<int, tripoint_bub_ms> &result = best_states[child.z() +
            OVERMAP_DEPTH][child.y()][child.x()];
    if( in.emplace( child ) ) {
        result.first = 0;
    }
    return result;
}*/

template <typename CostFn, typename HeuristicFn>
std::vector<tripoint_bub_ms> RealityBubblePathfinder::find_path( const
        RealityBubblePathfindingSettings &settings, const tripoint_bub_ms &from,
        const tripoint_bub_ms &to, CostFn cost_fn, HeuristicFn heuristic_fn )
{
    std::vector<PackedTripoint> path = astar_.find_path( settings.max_cost(), from, to, [this,
            &settings]( PackedTripoint current,
    auto &&emit_fn ) {
        const X cx = current.x;
        const Y cy = current.y;
        const Z cz = current.z;

        const X min_x = cx > 0 ? cx - 1 : 0;
        const X max_x = cx < MAPSIZE_X - 1 ? cx + 1 : MAPSIZE_X - 1;

        const Y min_y = cy > 0 ? cy - 1 : 0;
        const Y max_y = cy < MAPSIZE_Y - 1 ? cy + 1 : MAPSIZE_Y - 1;

        const PathfindingFlags avoid = settings.avoid_mask();

        if( settings.allow_flying() ) {
            for( Z z = std::max( cz - 1, -OVERMAP_DEPTH ); z <= std::min( cz + 1, OVERMAP_HEIGHT ); ++z ) {
                for( Y y = min_y; y <= max_y; ++y ) {
                    for( X x = min_x; x <= max_x; ++x ) {
                        if( x == cx && y == cy && z == cz ) {
                            continue;
                        }
                        const PackedTripoint next( x, y, z );
                        if( cache_->flags( next ) & avoid ) {
                            continue;
                        }
                        emit_fn( next );
                    }
                }
            }
            return;
        }

        const PathfindingFlags flags = cache_->flags( current );

        // If we're falling, we can only continue falling.
        if( cz > -OVERMAP_DEPTH && flags.is_set( PathfindingFlag::Air ) ) {
            const PackedTripoint down( cx, cy, cz - 1 );
            emit_fn( down );
            return;
        }

        for( Y y = min_y; y <= max_y; ++y ) {
            for( X x = min_x; x <= max_x; ++x ) {
                if( x == cx && y == cy ) {
                    continue;
                }
                const PackedTripoint next( x, y, cz );
                if( cache_->flags( next ) & avoid ) {
                    continue;
                }
                emit_fn( next );
            }
        }

        if( settings.allow_stairways() ) {
            if( flags.is_set( PathfindingFlag::GoesDown ) ) {
                emit_fn( cache_->down_destination( current ) );
            }
            if( flags.is_set( PathfindingFlag::GoesUp ) ) {
                emit_fn( cache_->up_destination( current ) );
            }
        }
        if( flags.is_set( PathfindingFlag::RampUp ) ) {
            for( Y y = min_y; y <= max_y; ++y ) {
                for( X x = min_x; x <= max_x; ++x ) {
                    if( x == cx && y == cy ) {
                        continue;
                    }
                    const PackedTripoint above( x, y, cz + 1 );
                    if( cache_->flags( above ) & avoid ) {
                        continue;
                    }
                    emit_fn( above );
                }
            }
        }
        if( flags.is_set( PathfindingFlag::RampDown ) ) {
            for( Y y = min_y; y <= max_y; ++y ) {
                for( X x = min_x; x <= max_x; ++x ) {
                    if( x == cx && y == cy ) {
                        continue;
                    }
                    const PackedTripoint below( x, y, cz - 1 );
                    if( cache_->flags( below ) & avoid ) {
                        continue;
                    }
                    emit_fn( below );
                }
            }
        }
    }, std::move( cost_fn ), std::move( heuristic_fn ) );

    std::vector<tripoint_bub_ms> tripath;
    tripath.reserve( path.size() );
    for( PackedTripoint p : path ) {
        tripath.push_back( p );
    }
    return tripath;
}

// Legacy Pathfinding

struct pathfinding_settings {
    int bash_strength = 0;
    int max_dist = 0;
    // At least 2 times the above, usually more
    int max_length = 0;

    // Expected terrain cost (2 is flat ground) of climbing a wire fence
    // 0 means no climbing
    int climb_cost = 0;

    bool allow_open_doors = false;
    bool allow_unlock_doors = false;
    bool avoid_traps = false;
    bool allow_climb_stairs = true;
    bool avoid_rough_terrain = false;
    bool avoid_sharp = false;

    pathfinding_settings() = default;
    pathfinding_settings( const pathfinding_settings & ) = default;

    pathfinding_settings( int bs, int md, int ml, int cc, bool aod, bool aud, bool at, bool acs,
                          bool art, bool as )
        : bash_strength( bs ), max_dist( md ), max_length( ml ), climb_cost( cc ),
          allow_open_doors( aod ), allow_unlock_doors( aud ), avoid_traps( at ), allow_climb_stairs( acs ),
          avoid_rough_terrain( art ), avoid_sharp( as ) {}

    pathfinding_settings &operator=( const pathfinding_settings & ) = default;

    // Converion to the new settings, while everything is being migrated.
    PathfindingSettings to_new_pathfinding_settings() const;
};

#endif // CATA_SRC_PATHFINDING_H
