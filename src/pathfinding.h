#pragma once
#ifndef CATA_SRC_PATHFINDING_H
#define CATA_SRC_PATHFINDING_H

#include "coordinates.h"
#include "game_constants.h"
#include "mdarray.h"


template <typename State, typename Cost = int, typename VisitedSet = std::unordered_set<State>, typename ParentMap = std::unordered_map<State, State>>
class AStarPathfinder
{
    public:
        AStarPathfinder( VisitedSet visited = VisitedSet{}, ParentMap parents = ParentMap{} ) : visited_
            ( visited ), parents_( parents ) {}

        template <typename NeighborsFn, typename CostFn, typename HeuristicFn>
        std::vector<State> find_path( const State &from, const State &to, NeighborsFn neighbors_fn,
                                      CostFn cost_fn, HeuristicFn heuristic_fn );

    private:
        VisitedSet visited_;
        ParentMap parents_;
};

class RealityBubblePathfinder
{
    public:
        // The class uses a lot of memory, and is safe to reuse as long as none of the provided
        // functors to find_path make use of it.
        static RealityBubblePathfinder *global();

        template <typename CostFn, typename HeuristicFn>
        std::vector<tripoint_bub_ms> find_path( const tripoint_bub_ms &from, const tripoint_bub_ms &to,
                                                CostFn cost_fn, HeuristicFn heuristic_fn );

    private:
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
            return tripoint_bub_ms( index / layer_size - OVERMAP_DEPTH, (index % layer_size) / MAPSIZE_X, index % MAPSIZE_X );
        }

        AStarPathfinder<int, int, IndexVisitedSet, IndexParentsMap> astar_;
};

enum class PathfindingFlag : uint32_t {
    Normal = 1 << 0,       // Plain boring tile (grass, dirt, floor etc.)
    ShallowWater = 1 << 1,         // Shallow water.
    DeepWater = 1 << 2,         // Deep water.
    Air = 1 << 3,         // Empty air
    Wall = 1 << 4,         // Unpassable ter/furn/vehicle
    Vehicle = 1 << 5,      // Any vehicle tile (passable or not)
    DangerousField = 1 << 6,        // Dangerous field
    DangerousTrap = 1 << 7,         // Dangerous trap (i.e. not flagged benign)
    Vertical = 1 << 8,       // Stairs, ramp etc.
    Climmable = 1 << 9,    // 0 move cost but can be climbed on examine
    Sharp = 1 << 10,        // sharp items (barbed wire, etc)
    Door = 1 << 11,        // A door (any kind)
    InsideDoor = 1 << 12, // A door that can be opened from the inside only
    LockedDoor = 1 << 13,        // A locked door
    Outside = 1 << 14,       // Tile is not indoors.
    Pit = 1 << 15, // A pit you can fall into / climb out of.
};

using PathfindingFlags = uint32_t;

class RealityBubblePathfindingCache {
public:
    PathfindingFlags get_flags(const tripoint_bub_ms& p) const;

    int move_cost(const tripoint_bub_ms& p) const;

    const std::pair<int, int>& bash_range(const tripoint_bub_ms& p) const;

private:
    bool dirty = false;
    cata::mdarray<PathfindingFlags, point_bub_ms> special;
};

struct CreaturePathfindingSettings {
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

    CreaturePathfindingSettings() = default;
    CreaturePathfindingSettings(const CreaturePathfindingSettings&) = default;

    CreaturePathfindingSettings(int bs, int md, int ml, int cc, bool aod, bool aud, bool at, bool acs,
        bool art, bool as)
        : bash_strength(bs), max_dist(md), max_length(ml), climb_cost(cc),
        allow_open_doors(aod), allow_unlock_doors(aud), avoid_traps(at), allow_climb_stairs(acs),
        avoid_rough_terrain(art), avoid_sharp(as) {}

    CreaturePathfindingSettings& operator=(const CreaturePathfindingSettings&) = default;
};

class CreaturePathfinder {
public:
    // Check if it is possible to move between two adjacent points.
    bool can_move(const tripoint_bub_ms& from, const tripoint_bub_ms& to) const;

    // Find the cost of movement between two adjacent points.
    // Returns std::nullopt if it isn't possible to move.
    std::optional<int> move_cost(const tripoint_bub_ms& from, const tripoint_bub_ms& to) const;

    std::vector<tripoint_bub_ms> find_path(const tripoint_bub_ms& from, const tripoint_bub_ms& to) const;

private:
    const CreaturePathfindingSettings* settings_;
    RealityBubblePathfindingCache* cache_;
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
            while( current_state != from ) {
                result.push_back( current_state );
                current_state = parents_[current_state];
            }
            std::reverse( result.begin(), result.end() );
            break;
        }

        neighbors_fn( current_state, [&frontier, &cost_fn, &heuristic_fn, &current_state,
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
}

template <typename CostFn, typename HeuristicFn>
std::vector<tripoint_bub_ms> RealityBubblePathfinder::find_path( const tripoint_bub_ms &from,
        const tripoint_bub_ms &to, CostFn cost_fn, HeuristicFn heuristic_fn )
{
    const std::vector<int> index_path = astar_.find_path( get_index(from),
                                        get_index(to), []( int index,
    auto &&emit_fn ) {
        const tripoint_bub_ms current = get_tripoint( index );
        for( int x = std::max( current.x() - 1, 0 ); x < std::min( current.x() + 1, MAPSIZE_X ); ++x ) {
            for( int y = std::max( current.y() - 1, 0 ); y < std::min( current.y() + 1, MAPSIZE_Y ); ++y ) {
                if( x == current.x() && y == current.y() ) {
                    continue;
                }
                const tripoint_bub_ms next = tripoint_bub_ms( x, y, current.z() );
                emit_fn( get_index( next ) );
            }
        }

        const tripoint_bub_ms up = tripoint_bub_ms( current.xy(), current.z() + 1 );
        emit_fn( get_index( up ) );

        const tripoint_bub_ms down = tripoint_bub_ms( current.xy(), current.z() - 1 );
        emit_fn( get_index( down ) );
    }, [cost_fn]( int from, int to ) {
        return cost_fn( get_tripoint( from ), get_tripoint( to ) );
    }, [heuristic_fn]( int index ) {
        return heuristic_fn( get_tripoint( index ) );
    } );

    std::vector<tripoint_bub_ms> path;
    for( int index : index_path ) {
        path.push_back( get_tripoint( index ) );
    }
    return path;
}

enum pf_special : int {
    PF_NORMAL = 0x00,    // Plain boring tile (grass, dirt, floor etc.)
    PF_SLOW = 0x01,      // Tile with move cost >2
    PF_WALL = 0x02,      // Unpassable ter/furn/vehicle
    PF_VEHICLE = 0x04,   // Any vehicle tile (passable or not)
    PF_FIELD = 0x08,     // Dangerous field
    PF_TRAP = 0x10,      // Dangerous trap
    PF_UPDOWN = 0x20,    // Stairs, ramp etc.
    PF_CLIMBABLE = 0x40, // 0 move cost but can be climbed on examine
    PF_SHARP = 0x80,     // sharp items (barbed wire, etc)
};

constexpr pf_special operator | ( pf_special lhs, pf_special rhs )
{
    return static_cast<pf_special>( static_cast< int >( lhs ) | static_cast< int >( rhs ) );
}

constexpr pf_special operator & ( pf_special lhs, pf_special rhs )
{
    return static_cast<pf_special>( static_cast< int >( lhs ) & static_cast< int >( rhs ) );
}

inline pf_special &operator |= ( pf_special &lhs, pf_special rhs )
{
    lhs = static_cast<pf_special>( static_cast< int >( lhs ) | static_cast< int >( rhs ) );
    return lhs;
}

inline pf_special &operator &= ( pf_special &lhs, pf_special rhs )
{
    lhs = static_cast<pf_special>( static_cast< int >( lhs ) & static_cast< int >( rhs ) );
    return lhs;
}

struct pathfinding_cache {
    pathfinding_cache();

    bool dirty = false;

    cata::mdarray<pf_special, point_bub_ms> special;
};

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
};

#endif // CATA_SRC_PATHFINDING_H
