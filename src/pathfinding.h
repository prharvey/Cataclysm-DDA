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

        template <typename Neighbors, typename Heuristic>
        std::vector<State> find_path( const State &start, const State &end, Neighbors neighbors_fn,
                                      Heuristic heuristic_fn );

    private:
        VisitedSet visited_;
        ParentMap parents_;
};

struct FirstElementGreaterThan {
    template <typename T, typename... Ts>
    bool operator()( const std::tuple<T, Ts...> &lhs, const std::tuple<T, Ts...> &rhs ) const {
        return std::get<0>( lhs ) > std::get<0>( rhs );
    }
};

template <typename State, typename Cost, typename VisitedSet, typename ParentMap>
template <typename Neighbors, typename Heuristic>
std::vector<State> AStarPathfinder<State, Cost, VisitedSet, ParentMap>::find_path(
    const State &start, const State &end, Neighbors neighbors_fn, Heuristic heuristic_fn )
{
    using Node = std::tuple<Cost, Cost, State, State>;
    std::priority_queue< Node, std::vector< Node>, FirstElementGreaterThan> frontier;
    std::vector<State> result;

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

        if( current_state == end ) {
            while( current_state != start ) {
                result.push_back( current_state );
                current_state = parents_[current_state];
            }
            std::reverse( result.begin(), result.end() );
            break;
        }

        neighbors_fn( current_state, [&frontier, &heuristic_fn, &current_state, current_cost]( State &&
        neighbour, Cost cost ) {
            if( visited_.count( neighbour ) == 0 ) {
                const Cost new_cost = current_cost + cost;
                const Cost estimated_cost = heuristic_fn( neighbour ) + new_cost;
                frontier.emplace( estimated_cost, new_cost, std::move( neighbour ), current_state );
            }
        } );

    } while( !frontier.empty() );
    visited_.clear();
    return result;
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
