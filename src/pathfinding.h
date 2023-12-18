#pragma once
#ifndef CATA_SRC_PATHFINDING_H
#define CATA_SRC_PATHFINDING_H

#include <queue>

#include "coordinates.h"
#include "game_constants.h"
#include "mapdata.h"
#include "mdarray.h"
#include "type_id.h"

class map;

template <typename Node, typename Cost = int, typename VisitedSet = std::unordered_set<Node>, typename BestPathMap = std::unordered_map<Node, std::pair<Cost, Node>>>
          class AStarState
{
public:
    void reset( const Node &start, Cost cost );

    std::optional<Node> get_next( Cost max );

    template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
    void generate_neighbors( const Node &current, StateCostFn &&s_cost_fn, TransitionCostFn &&t_cost_fn,
                             HeuristicFn &&heuristic_fn, NeighborsFn &&neighbors_fn );

    bool has_path_to( const Node &end ) const;

    Cost path_cost( const Node &end ) const;

    std::vector<Node> path_to( const Node &end ) const;

private:
    struct FirstElementGreaterThan {
        template <typename T, typename... Ts>
        bool operator()( const std::tuple<T, Ts...> &lhs, const std::tuple<T, Ts...> &rhs ) const {
            return std::get<0>( lhs ) > std::get<0>( rhs );
        }
    };

    using FrontierNode = std::tuple<Cost, Node>;
    using FrontierQueue =
        std::priority_queue<FrontierNode, std::vector<FrontierNode>, FirstElementGreaterThan>;

    VisitedSet visited_;
    BestPathMap best_paths_;
    FrontierQueue frontier_;
};

template <typename Node, typename Cost = int, typename VisitedSet = std::unordered_set<Node>, typename BestStateMap = std::unordered_map<Node, std::pair<Cost, Node>>>
          class AStarPathfinder
{
public:
    template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
    std::vector<Node> find_path( Cost max, const Node &from, const Node &to,
                                 StateCostFn &&s_cost_fn, TransitionCostFn &&t_cost_fn, HeuristicFn &&heuristic_fn,
                                 NeighborsFn &&neighbors_fn );

private:
    AStarState<Node, Cost, VisitedSet, BestStateMap> state_;
};

template <typename Node, typename Cost = int, typename VisitedSet = std::unordered_set<Node>, typename BestStateMap = std::unordered_map<Node, std::pair<Cost, Node>>>
          class BidirectionalAStarPathfinder
{
public:
    template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
    std::vector<Node> find_path( Cost max, const Node &from, const Node &to,
                                 StateCostFn &&s_cost_fn, TransitionCostFn &&t_cost_fn, HeuristicFn &&heuristic_fn,
                                 NeighborsFn &&neighbors_fn );

private:
    AStarState<Node, Cost, VisitedSet, BestStateMap> forward_;
    AStarState<Node, Cost, VisitedSet, BestStateMap> backward_;
};


enum class PathfindingFlag : uint8_t {
    Ground = 0,     // Can walk on
    Slow,           // Move cost > 2
    Swimmable,      // Can swim in
    Air,            // Empty air
    Unsheltered,    // Outside and above ground level
    Obstacle,       // Something stopping us, might be bashable.
    Bashable,       // Something bashable.
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

        // NOLINTNEXTLINE(google-explicit-constructor)
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

        constexpr explicit operator bool() const {
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

        bool allow_falling() const {
            return allow_falling_;
        }
        void set_allow_falling( bool v = true ) {
            allow_falling_ = v;
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

        const tripoint_rel_ms &padding() const {
            return padding_;
        }
        void set_padding( const tripoint_rel_ms &v ) {
            padding_ = v;
        }

    private:
        bool allow_flying_ = false;
        bool allow_falling_ = false;
        bool allow_stairways_ = false;
        int max_cost_ = 0;
        tripoint_rel_ms padding_ = tripoint_rel_ms( 16, 16, 1 );
        PathfindingFlags avoid_mask_ = PathfindingFlag::Air | PathfindingFlag::Impassable;
};

class RealityBubblePathfinder
{
    public:
        // The class uses a lot of memory, and is safe to reuse as long as none of the provided
        // functors to find_path make use of it.
        explicit RealityBubblePathfinder( RealityBubblePathfindingCache *cache ) : cache_( cache ) {}

        template <typename PositionCostFn, typename MoveCostFn, typename HeuristicFn>
        std::vector<tripoint_bub_ms> find_path( const RealityBubblePathfindingSettings &settings,
                                                const tripoint_bub_ms &from, const tripoint_bub_ms &to,
                                                PositionCostFn &&p_cost_fn, MoveCostFn &&m_cost_fn, HeuristicFn &&heuristic_fn );

    private:
        // Minimum implementation of std::unordered_set interface that is used in the pathfinder.
        class FastTripointSet
        {
            public:
                // Empty dummy type to return in place of an iterator. It isn't possible to give a nice
                // iterator implementation, and the pathfinder doesn't use it anyways.
                struct NotIterator {};

                std::pair<NotIterator, bool> emplace( const tripoint_bub_ms &p );

                void clear();

                std::size_t count( const tripoint_bub_ms &p ) const {
                    return set_[p.z() + OVERMAP_DEPTH].test( p.y() * MAPSIZE_X + p.x() );
                }

            private:
                std::array<bool, OVERMAP_LAYERS> dirty_ = {};
                std::array<std::bitset<MAPSIZE_X *MAPSIZE_Y>, OVERMAP_LAYERS> set_ = {};
        };

        // Minimum implementation of std::unordered_map interface that is used in the pathfinder.
        class FastBestPathMap
        {
            public:
                std::pair<std::pair<int, tripoint_bub_ms>*, bool> try_emplace( const tripoint_bub_ms &child,
                        int cost, const tripoint_bub_ms &parent );

                std::size_t count( const tripoint_bub_ms &p ) const {
                    return in_.count( p );
                }

                void clear() {
                    in_.clear();
                }

                const std::pair<int, tripoint_bub_ms> &at( const tripoint_bub_ms &child ) const {
                    return best_states_[child.z() + OVERMAP_DEPTH][child.y()][child.x()];
                }

                std::pair<int, tripoint_bub_ms> &operator[]( const tripoint_bub_ms &child ) {
                    return *try_emplace( child, 0, child ).first;
                }

            private:
                FastTripointSet in_;
                RealityBubbleArray<std::pair<int, tripoint_bub_ms>> best_states_;
        };

        template <typename AStar, typename PositionCostFn, typename MoveCostFn, typename HeuristicFn>
        std::vector<tripoint_bub_ms> find_path_impl( AStar &impl,
                const RealityBubblePathfindingSettings &settings,
                const tripoint_bub_ms &from, const tripoint_bub_ms &to,
                PositionCostFn &&p_cost_fn, MoveCostFn &&m_cost_fn, HeuristicFn &&heuristic_fn );

        RealityBubblePathfindingCache *cache_;
        AStarPathfinder<tripoint_bub_ms, int, FastTripointSet, FastBestPathMap> astar_;
        BidirectionalAStarPathfinder<tripoint_bub_ms, int, FastTripointSet, FastBestPathMap>
        bidirectional_astar_;
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

        bool avoid_falling() const {
            return !rb_settings_.allow_falling();
        }
        void set_avoid_falling( bool v = true ) {
            set_avoid_air( v );
            rb_settings_.set_allow_falling( !v );
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

        bool avoid_bashing() const {
            return avoid_bashing_;
        }
        void set_avoid_bashing( bool v = true ) {
            avoid_bashing_ = v;
            maybe_set_avoid_obstacle();
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
            set( PathfindingFlag::Obstacle, !is_digging() && ( climb_cost() <= 0 || avoid_climbing() ) &&
                 avoid_opening_doors() && ( avoid_bashing() || bash_strength() <= 0 ) );
        }

        int max_distance_ = 0;

        bool avoid_bashing_ = false;
        int bash_strength_ = 0;

        // Expected terrain cost (2 is flat ground) of climbing a wire fence, 0 means no climbing
        bool avoid_climbing_ = false;
        int climb_cost_ = 0;

        bool is_digging_ = false;
        RealityBubblePathfindingSettings rb_settings_;

        bool avoid_opening_doors_ = false;
        bool avoid_unlocking_doors_ = false;
        std::function<bool( const field_type_id & )> maybe_avoid_dangerous_fields_fn_;
        std::function<bool( const tripoint_bub_ms & )> maybe_avoid_fn_;
};

// Implementation Details

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
void AStarState<Node, Cost, VisitedSet, BestPathMap>::reset( const Node &start, Cost cost )
{
    visited_.clear();
    best_paths_.clear();

    // No clear method...
    frontier_ = FrontierQueue();

    best_paths_.try_emplace( start, 0, start );
    frontier_.emplace( cost, start );
}

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
bool AStarState<Node, Cost, VisitedSet, BestPathMap>::has_path_to( const Node &end ) const
{
    return best_paths_.count( end );
}

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
Cost AStarState<Node, Cost, VisitedSet, BestPathMap>::path_cost( const Node &end ) const
{
    return best_paths_.at( end ).first;
}

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
std::vector<Node> AStarState<Node, Cost, VisitedSet, BestPathMap>::path_to( const Node &end ) const
{
    std::vector<Node> result;
    if( has_path_to( end ) ) {
        Node current = end;
        while( best_paths_.at( current ).first != 0 ) {
            result.push_back( current );
            current = best_paths_.at( current ).second;
        }
        std::reverse( result.begin(), result.end() );
    }
    return result;
}

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
std::optional<Node> AStarState<Node, Cost, VisitedSet, BestPathMap>::get_next( Cost max )
{
    while( !frontier_.empty() ) {
        auto [cost, current] = frontier_.top();
        frontier_.pop();

        if( cost >= max ) {
            return std::nullopt;
        }

        if( const auto [_, inserted] = visited_.emplace( current ); !inserted ) {
            continue;
        }
        return current;
    }
    return std::nullopt;
}

template <typename Node, typename Cost, typename VisitedSet, typename BestPathMap>
template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
void AStarState<Node, Cost, VisitedSet, BestPathMap>::generate_neighbors(
    const Node &current, StateCostFn &&s_cost_fn, TransitionCostFn &&t_cost_fn,
    HeuristicFn &&heuristic_fn,
    NeighborsFn &&neighbors_fn )
{
    // Can't use structured bindings here due to a defect in Clang 10.
    const std::pair<Cost, Node> &best_path = best_paths_[current];
    const Cost current_cost = best_path.first;
    const Node &current_parent = best_path.second;
    neighbors_fn( current_parent, current, [this, &s_cost_fn, &t_cost_fn, &heuristic_fn, &current,
          current_cost]( const Node & neighbour ) {
        if( visited_.count( neighbour ) ) {
            return;
        }
        if( const std::optional<Cost> s_cost = s_cost_fn( neighbour ) ) {
            if( const std::optional<Cost> t_cost = t_cost_fn( current, neighbour ) ) {
                const auto& [iter, _] = best_paths_.try_emplace( neighbour, std::numeric_limits<Cost>::max(),
                                        Node() );
                auto& [best_cost, parent] = *iter;
                const Cost new_cost = current_cost + *s_cost + *t_cost;
                if( new_cost < best_cost ) {
                    best_cost = new_cost;
                    parent = current;
                    const Cost estimated_cost = new_cost + heuristic_fn( neighbour );
                    frontier_.emplace( estimated_cost, neighbour );
                }
            }
        } else {
            visited_.emplace( neighbour );
        }
    } );
}

template <typename Node, typename Cost, typename VisitedSet, typename BestStateMap>
template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
std::vector<Node> AStarPathfinder<Node, Cost, VisitedSet, BestStateMap>::find_path(
    Cost max_cost, const Node &from, const Node &to, StateCostFn &&s_cost_fn,
    TransitionCostFn &&t_cost_fn,
    HeuristicFn &&heuristic_fn, NeighborsFn &&neighbors_fn )
{
    if( !s_cost_fn( from ) || !s_cost_fn( to ) ) {
        return {};
    }

    state_.reset( from, heuristic_fn( from, to ) );
    while( const std::optional<Node> current = state_.get_next( max_cost ) ) {
        if( *current == to ) {
            return state_.path_to( to );
        }
        state_.generate_neighbors( *current, s_cost_fn, t_cost_fn, [&heuristic_fn,
        to]( const Node & from ) {
            return heuristic_fn( from, to );
        }, neighbors_fn );
    }
    return {};
}


template <typename Node, typename Cost, typename VisitedSet, typename BestStateMap>
template <typename StateCostFn, typename TransitionCostFn, typename HeuristicFn, typename NeighborsFn>
std::vector<Node> BidirectionalAStarPathfinder<Node, Cost, VisitedSet, BestStateMap>::find_path(
    Cost max_cost, const Node &from, const Node &to, StateCostFn &&s_cost_fn,
    TransitionCostFn &&t_cost_fn,
    HeuristicFn &&heuristic_fn, NeighborsFn &&neighbors_fn )
{
    if( !s_cost_fn( from ) || !s_cost_fn( to ) ) {
        return {};
    }

    // The full cost is not used since that would result in paths that are up to 2x longer than
    // intended. Half the cost cannot be used, since there is no guarantee that both searches
    // proceed at the same pace. 2/3rds the cost is a fine balance between the two, and has the
    // effect of the worst case still visiting less states than normal A*.
    const Cost partial_max_cost = 2 * max_cost / 3;
    forward_.reset( from, heuristic_fn( from, to ) );
    backward_.reset( to, heuristic_fn( to, from ) );
    for( ;; ) {
        const std::optional<Node> f_current_state = forward_.get_next( partial_max_cost );
        if( !f_current_state ) {
            break;
        }
        const std::optional<Node> b_current_state = backward_.get_next( partial_max_cost );
        if( !b_current_state ) {
            break;
        }

        bool f_links = backward_.has_path_to( *f_current_state );
        bool b_links = forward_.has_path_to( *b_current_state );

        if( f_links && b_links ) {
            const Cost f_cost = forward_.path_cost( *f_current_state ) + backward_.path_cost(
                                    *f_current_state );
            const Cost b_cost = forward_.path_cost( *b_current_state ) + backward_.path_cost(
                                    *b_current_state );
            if( b_cost < f_cost ) {
                f_links = false;
            } else {
                b_links = false;
            }
        }

        if( f_links || b_links ) {
            const Node &midpoint = f_links ? *f_current_state : *b_current_state;
            std::vector<Node> forward_path = forward_.path_to( midpoint );
            std::vector<Node> backward_path = backward_.path_to( midpoint );
            if( backward_path.empty() ) {
                return forward_path;
            }
            backward_path.pop_back();
            std::for_each( backward_path.rbegin(), backward_path.rend(), [&forward_path]( const Node & node ) {
                forward_path.push_back( node );
            } );
            forward_path.push_back( to );
            return forward_path;
        }

        forward_.generate_neighbors( *f_current_state,  s_cost_fn, t_cost_fn, [&heuristic_fn,
        &to]( const Node & from ) {
            return heuristic_fn( from, to );
        }, neighbors_fn );
        backward_.generate_neighbors( *b_current_state,  s_cost_fn, [&t_cost_fn]( const Node & from,
        const Node & to ) {
            return t_cost_fn( to, from );
        }, [&heuristic_fn, &from]( const Node & to ) {
            return heuristic_fn( to, from );
        }, neighbors_fn );
    }
    return {};
}

template <typename PositionCostFn, typename MoveCostFn, typename HeuristicFn>
std::vector<tripoint_bub_ms> RealityBubblePathfinder::find_path( const
        RealityBubblePathfindingSettings &settings, const tripoint_bub_ms &from,
        const tripoint_bub_ms &to, PositionCostFn &&p_cost_fn, MoveCostFn &&m_cost_fn,
        HeuristicFn &&heuristic_fn )
{
    // The bidirectional pathfinder doesn't handle paths generated by falling, so we have to fall
    // back to normal A* in this case.
    const bool might_fall = !settings.allow_flying() && settings.allow_falling();
    if( might_fall ) {
        return find_path_impl( astar_, settings, from, to, std::forward<PositionCostFn>( p_cost_fn ),
                               std::forward<MoveCostFn>( m_cost_fn ), std::forward<HeuristicFn>( heuristic_fn ) );
    } else {
        return find_path_impl( bidirectional_astar_, settings, from, to,
                               std::forward<PositionCostFn>( p_cost_fn ), std::forward<MoveCostFn>( m_cost_fn ),
                               std::forward<HeuristicFn>( heuristic_fn ) );
    }
}

template <typename AStar, typename PositionCostFn, typename MoveCostFn, typename HeuristicFn>
std::vector<tripoint_bub_ms> RealityBubblePathfinder::find_path_impl( AStar &impl, const
        RealityBubblePathfindingSettings &settings, const tripoint_bub_ms &from,
        const tripoint_bub_ms &to, PositionCostFn &&p_cost_fn, MoveCostFn &&m_cost_fn,
        HeuristicFn &&heuristic_fn )
{
    const tripoint_bub_ms min = coord_max( coord_min( from, to ) - settings.padding(),
                                           tripoint_bub_ms( 0, 0, -OVERMAP_DEPTH ) );
    const tripoint_bub_ms max = coord_min( coord_max( from, to ) + settings.padding(),
                                           tripoint_bub_ms( MAPSIZE_X - 1, MAPSIZE_Y - 1, OVERMAP_HEIGHT ) );
    inclusive_cuboid<tripoint_bub_ms> bounds( min, max );

    return impl.find_path( settings.max_cost(), from, to, std::forward<PositionCostFn>( p_cost_fn ),
                           std::forward<MoveCostFn>( m_cost_fn ), std::forward<HeuristicFn>( heuristic_fn ), [this, &settings,
          bounds]( tripoint_bub_ms p, tripoint_bub_ms c, auto &&emit_fn ) {
        const tripoint_rel_ms offset( 1, 1, 1 );
        const tripoint_bub_ms min = clamp( c - offset, bounds );
        const tripoint_bub_ms max = clamp( c + offset, bounds );
        if( settings.allow_flying() ) {
            for( int z = min.z(); z <= max.z(); ++z ) {
                for( int y = min.y(); y <= max.y(); ++y ) {
                    for( int x = min.x(); x <= max.x(); ++x ) {
                        if( x == c.x() && y == c.y() && z == c.z() ) {
                            continue;
                        }
                        const tripoint_bub_ms next( x, y, z );
                        emit_fn( next );
                    }
                }
            }
            return;
        }

        const PathfindingFlags flags = cache_->flags( c );

        // If we're falling, we can only continue falling.
        if( c.z() > -OVERMAP_DEPTH && flags.is_set( PathfindingFlag::Air ) ) {
            const tripoint_bub_ms down( c.x(), c.y(), c.z() - 1 );
            emit_fn( down );
            return;
        }

        const int dx = ( c.x() > p.x() ) - ( c.x() < p.x() );
        const int dy = ( c.y() > p.y() ) - ( c.y() < p.y() );
        const int xd = c.x() + dx;
        const int yd = c.y() + dy;

        if( dx != 0 && dy != 0 ) {
            // Diagonal movement. Visit the following states:
            //
            //  * * *
            //  . c *
            //  p . *
            //
            // where * is a state to be visited
            // p is the parent state (not visited)
            // c is the current state (not visited)
            // . is not visited
            if( min.x() <= xd && xd <= max.x() ) {
                for( int y = min.y(); y <= max.y(); ++y ) {
                    const tripoint_bub_ms next( xd, y, c.z() );
                    emit_fn( next );
                }
            }
            if( min.y() <= yd && yd <= max.y() ) {
                for( int x = min.x(); x <= max.x(); ++x ) {
                    if( x == xd ) {
                        continue;
                    }
                    const tripoint_bub_ms next( x, yd, c.z() );
                    emit_fn( next );
                }
            }
        } else if( dx != 0 ) {
            // Horizontal movement. Visit the following states:
            //
            //  . . *
            //  p c *
            //  . . *
            //
            // where * is a state to be visited
            // p is the parent state (not visited)
            // c is the current state (not visited)
            // . is not visited
            if( min.x() <= xd && xd <= max.x() ) {
                for( int y = min.y(); y <= max.y(); ++y ) {
                    const tripoint_bub_ms next( xd, y, c.z() );
                    emit_fn( next );
                }
            }
        } else if( dy != 0 ) {
            // Vertical movement. Visit the following states:
            //
            //  * * *
            //  . c .
            //  . p .
            //
            // where * is a state to be visited
            // p is the parent state (not visited)
            // c is the current state (not visited)
            // . is not visited
            if( min.y() <= yd && yd <= max.y() ) {
                for( int x = min.x(); x <= max.x(); ++x ) {
                    const tripoint_bub_ms next( x, yd, c.z() );
                    emit_fn( next );
                }
            }
        } else {
            // We arrived in this state with same x/y, which means
            // we got here by traversing a staircase or similar. Need to
            // visit all directions.
            for( int y = min.y(); y <= max.y(); ++y ) {
                for( int x = min.x(); x <= max.x(); ++x ) {
                    if( x == c.x() && y == c.y() ) {
                        continue;
                    }
                    const tripoint_bub_ms next( x, y, c.z() );
                    emit_fn( next );
                }
            }
        }

        if( settings.allow_stairways() ) {
            if( flags.is_set( PathfindingFlag::GoesDown ) ) {
                emit_fn( cache_->down_destination( c ) );
            }
            if( flags.is_set( PathfindingFlag::GoesUp ) ) {
                emit_fn( cache_->up_destination( c ) );
            }
        }
        if( flags.is_set( PathfindingFlag::RampUp ) ) {
            for( int y = min.y(); y <= max.y(); ++y ) {
                for( int x = min.x(); x <= max.x(); ++x ) {
                    if( x == c.x() && y == c.y() ) {
                        continue;
                    }
                    const tripoint_bub_ms above( x, y, c.z() + 1 );
                    emit_fn( above );
                }
            }
        }
        if( flags.is_set( PathfindingFlag::RampDown ) ) {
            for( int y = min.y(); y <= max.y(); ++y ) {
                for( int x = min.x(); x <= max.x(); ++x ) {
                    if( x == c.x() && y == c.y() ) {
                        continue;
                    }
                    const tripoint_bub_ms below( x, y, c.z() - 1 );
                    emit_fn( below );
                }
            }
        }
    } );
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
