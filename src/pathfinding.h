#pragma once
#ifndef CATA_SRC_PATHFINDING_H
#define CATA_SRC_PATHFINDING_H

#include "coordinates.h"
#include "game_constants.h"
#include "mapdata.h"
#include "mdarray.h"
#include "type_id.h"

class map;

template <typename State, typename Cost = int, typename VisitedSet = std::unordered_set<State>, typename ParentMap = std::unordered_map<State, State>>
class AStarPathfinder
{
    public:
        template <typename NeighborsFn, typename CostFn, typename HeuristicFn>
        std::vector<State> find_path( const State &from, const State &to, NeighborsFn neighbors_fn,
                                      CostFn cost_fn, HeuristicFn heuristic_fn );

    private:
        VisitedSet visited_;
        ParentMap parents_;
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

        const tripoint_bub_ms &stair_up_destination( const tripoint_bub_ms &p ) const {
            return up_stair_destinations_.find( p )->second;
        }

        const tripoint_bub_ms &stair_down_destination( const tripoint_bub_ms &p ) const {
            return down_stair_destinations_.find( p )->second;
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

        int &move_cost_ref( const tripoint_bub_ms &p ) {
            return move_cost_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        std::pair<int, int> &bash_range_ref( const tripoint_bub_ms &p ) {
            return bash_range_cache_[p.z() + OVERMAP_DEPTH][p.y()][p.x()];
        }

        bool vertical_move_destination( ter_furn_flag flag, tripoint &t ) const;

        void invalidate_dependants( const tripoint_bub_ms &p );

        void update( const map &here, const tripoint_bub_ms &p );

        std::unordered_set<int> dirty_z_levels_;
        std::unordered_map<int, std::unordered_set<point_bub_ms>> dirty_positions_;
        std::unordered_map<tripoint_bub_ms, std::vector<tripoint_bub_ms>> dependants_by_position_;

        // Note that this is in reverse order for memory locality: z, y, x.
        template <typename T>
        using RealityBubbleArray =
            std::array<std::array<std::array<T, MAPSIZE_X>, MAPSIZE_Y>, OVERMAP_LAYERS>;

        std::unordered_map<tripoint_bub_ms, tripoint_bub_ms> up_stair_destinations_;
        std::unordered_map<tripoint_bub_ms, tripoint_bub_ms> down_stair_destinations_;
        RealityBubbleArray<PathfindingFlags> flag_cache_;
        RealityBubbleArray<int> move_cost_cache_;
        RealityBubbleArray<std::pair<int, int>> bash_range_cache_;
};

class RealityBubblePathfindingSettings
{
    public:
        bool allow_flying() const {
            return allow_flying_;
        }
        void set_allow_flying( int v = true ) {
            allow_flying_ = v;
        }

        bool allow_stairways() const {
            return allow_stairways_;
        }
        void set_allow_stairways( int v = true ) {
            allow_stairways_ = v;
        }

    private:
        bool allow_flying_ = false;
        bool allow_stairways_ = false;
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
        /*
        struct IndexVisitedSet {
            void emplace( int index );
            void clear();
            std::size_t count( int index ) const;
            std::bitset<MAPSIZE_X *MAPSIZE_Y *OVERMAP_LAYERS> visited;
        };
        struct IndexParentsMap {
            void emplace( int child, int parent );
            int operator[]( int child ) const;
            std::array<int, MAPSIZE_X *MAPSIZE_Y *OVERMAP_LAYERS> parents;
        };

        static constexpr int get_index( const tripoint_bub_ms &p ) {
            constexpr int layer_size = MAPSIZE_X * MAPSIZE_Y;
            return ( p.z() + OVERMAP_DEPTH ) * layer_size + p.y() * MAPSIZE_X + p.x();
        }

        static constexpr tripoint_bub_ms get_tripoint( int index ) {
            constexpr int layer_size = MAPSIZE_X * MAPSIZE_Y;
            return tripoint_bub_ms( index / layer_size - OVERMAP_DEPTH, ( index % layer_size ) / MAPSIZE_X,
                                    index % MAPSIZE_X );
        }*/

        RealityBubblePathfindingCache *cache_;
        AStarPathfinder<tripoint_bub_ms/*int, int, IndexVisitedSet, IndexParentsMap*/> astar_;
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

        bool avoid_sunlight() const {
            return avoid_sunlight_;
        }
        void set_avoid_sunlight( bool v = true ) {
            avoid_sunlight_ = v;
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
        void set_max_distance( int v = true ) {
            max_distance_ = v;
        }

        int max_cost() const {
            return max_cost_;
        }
        void set_max_cost( int v = true ) {
            max_cost_ = v;
        }

        int climb_cost() const {
            return climb_cost_;
        }
        void set_climb_cost( int v = true ) {
            climb_cost_ = v;
            maybe_set_avoid_obstacle();
        }

        bool is_digging() const {
            return is_digging_;
        }
        void set_is_digging( int v = true ) {
            is_digging_ = v;
            maybe_set_avoid_obstacle();
        }

        bool is_flying() const {
            return rb_settings_.allow_flying();
        }
        void set_is_flying( int v = true ) {
            rb_settings_.set_allow_flying( v );
        }

        PathfindingFlags avoid_mask() const {
            return avoid_mask_;
        }

        int bash_strength() const {
            return bash_strength_;
        }
        void set_bash_strength( int v = true ) {
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
            return avoid_mask_.is_set( flag );
        }
        void set( PathfindingFlags flags, bool v ) {
            if( v ) {
                avoid_mask_.set_union( flags );
            } else {
                avoid_mask_.set_clear( flags );
            }
        }

        void maybe_set_avoid_obstacle() {
            // Check if we can short circuit checking obstacles. Significantly faster if so.
            set( PathfindingFlag::Obstacle, !is_digging() && climb_cost() <= 0 && avoid_climbing() &&
                 avoid_opening_doors() && bash_strength() <= 0 );
        }

        int max_distance_ = 0;
        int max_cost_ = 0;

        int bash_strength_ = 0;

        // Expected terrain cost (2 is flat ground) of climbing a wire fence
        // 0 means no climbing
        int climb_cost_ = 0;
        bool is_digging_ = false;

        RealityBubblePathfindingSettings rb_settings_;

        bool avoid_climbing_ = false;
        bool avoid_sunlight_ = false;
        bool avoid_opening_doors_ = false;
        bool avoid_unlocking_doors_ = false;
        std::function<bool( const field_type_id & )> maybe_avoid_dangerous_fields_fn_;
        std::function<bool( const tripoint_bub_ms & )> maybe_avoid_fn_;
        PathfindingFlags avoid_mask_ = PathfindingFlag::Air | PathfindingFlag::Impassable;
};

// Implementation Details

struct FirstElementGreaterThan {
    template <typename T, typename... Ts>
    bool operator()( const std::tuple<T, Ts...> &lhs, const std::tuple<T, Ts...> &rhs ) const {
        return std::get<0>( lhs ) > std::get<0>( rhs );
    }
};

template <typename State, typename Cost, typename VisitedSet, typename ParentMap>
template <typename NeighborsFn, typename CostFn, typename HeuristicFn>
std::vector<State> AStarPathfinder<State, Cost, VisitedSet, ParentMap>::find_path(
    const State &from, const State &to, NeighborsFn neighbors_fn, CostFn cost_fn,
    HeuristicFn heuristic_fn )
{
    using Node = std::tuple<Cost, Cost, State, State>;
    std::priority_queue< Node, std::vector< Node>, FirstElementGreaterThan> frontier;
    std::vector<State> result;

    visited_.clear();
    parents_.clear();
    // The first parameter should be heuristic_fn(start), but it is immediately popped
    // so there is no reason to waste the time.
    frontier.emplace( 0, 0, from, from );
    do {
        const auto [_, current_cost, current_state, current_parent] = frontier.top();
        frontier.pop();

        if( visited_.count( current_state ) == 1 ) {
            continue;
        }

        visited_.emplace( current_state );
        parents_.emplace( current_state, current_parent );

        if( current_state == to ) {
            State out = current_state;
            while( out != from ) {
                result.push_back( out );
                out = parents_[out];
            }
            std::reverse( result.begin(), result.end() );
            break;
        }

        neighbors_fn( current_state, [this, &frontier, &cost_fn, &heuristic_fn, &current_state,
              current_cost]( const State & neighbour ) {
            if( visited_.count( neighbour ) == 0 ) {
                if( const std::optional<Cost> transition_cost = cost_fn( current_state, neighbour ) ) {
                    const Cost new_cost = current_cost + *transition_cost;
                    const Cost estimated_cost = new_cost + heuristic_fn( neighbour );
                    frontier.emplace( estimated_cost, new_cost, neighbour, current_state );
                }
            }
        } );

    } while( !frontier.empty() );
    return result;
}

/*
inline void RealityBubblePathfinder::IndexVisitedSet::emplace( int index )
{
    visited.set( index );
}

inline void RealityBubblePathfinder::IndexVisitedSet::clear()
{
    visited.reset();
}

inline std::size_t RealityBubblePathfinder::IndexVisitedSet::count( int index ) const
{
    return visited.test( index );
}

inline void RealityBubblePathfinder::IndexParentsMap::emplace( int child, int parent )
{
    parents[child] = parent;
}

inline int RealityBubblePathfinder::IndexParentsMap::operator[]( int child ) const
{
    return parents[child];
}*/

template <typename CostFn, typename HeuristicFn>
std::vector<tripoint_bub_ms> RealityBubblePathfinder::find_path( const
        RealityBubblePathfindingSettings &settings, const tripoint_bub_ms &from,
        const tripoint_bub_ms &to, CostFn cost_fn, HeuristicFn heuristic_fn )
{
    std::vector<tripoint_bub_ms> path = astar_.find_path( from, to, [this,
                                        &settings]( const tripoint_bub_ms & current,
    auto &&emit_fn ) {
        for( int y = std::max( current.y() - 1, 0 ); y < std::min( current.y() + 1, MAPSIZE_Y ); ++y ) {
            for( int x = std::max( current.x() - 1, 0 ); x < std::min( current.x() + 1, MAPSIZE_X ); ++x ) {
                if( x == current.x() && y == current.y() ) {
                    continue;
                }
                const tripoint_bub_ms next = tripoint_bub_ms( x, y, current.z() );
                emit_fn( next );
            }
        }

        if( settings.allow_flying() ) {
            for( int z = std::max( current.z() - 1, -OVERMAP_DEPTH );
                 z <= std::min( current.z() + 1, OVERMAP_HEIGHT );
                 ++z ) {
                if( z == current.z() ) {
                    continue;
                }
                for( int y = std::max( current.y() - 1, 0 ); y < std::min( current.y() + 1, MAPSIZE_Y ); ++y ) {
                    for( int x = std::max( current.x() - 1, 0 ); x < std::min( current.x() + 1, MAPSIZE_X ); ++x ) {
                        const tripoint_bub_ms next( x, y, z );
                        emit_fn( next );
                    }
                }
            }
            return;
        }

        const PathfindingFlags flags = cache_->flags( current );
        if( settings.allow_stairways() ) {
            if( flags.is_set( PathfindingFlag::GoesDown ) ) {
                emit_fn( cache_->stair_down_destination( current ) );
            }
            if( flags.is_set( PathfindingFlag::GoesUp ) ) {
                emit_fn( cache_->stair_up_destination( current ) );
            }
        }
        if( flags.is_set( PathfindingFlag::RampUp ) ) {
            for( int y = std::max( current.y() - 1, 0 ); y < std::min( current.y() + 1, MAPSIZE_Y ); ++y ) {
                for( int x = std::max( current.x() - 1, 0 ); x < std::min( current.x() + 1, MAPSIZE_X ); ++x ) {
                    if( x == current.x() && y == current.y() ) {
                        continue;
                    }
                    const tripoint_bub_ms above( x, y, current.z() + 1 );
                    emit_fn( above );
                }
            }
        }
        if( flags.is_set( PathfindingFlag::RampDown ) ) {
            for( int y = std::max( current.y() - 1, 0 ); y < std::min( current.y() + 1, MAPSIZE_Y ); ++y ) {
                for( int x = std::max( current.x() - 1, 0 ); x < std::min( current.x() + 1, MAPSIZE_X ); ++x ) {
                    if( x == current.x() && y == current.y() ) {
                        continue;
                    }
                    const tripoint_bub_ms below( x, y, current.z() - 1 );
                    emit_fn( below );
                }
            }
        }
    }, std::move( cost_fn ), std::move( heuristic_fn ) );
    return path;
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
