#include "pathfinding.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "cata_utility.h"
#include "coordinates.h"
#include "debug.h"
#include "game.h"
#include "gates.h"
#include "line.h"
#include "map.h"
#include "mapdata.h"
#include "point.h"
#include "submap.h"
#include "trap.h"
#include "veh_type.h"
#include "vehicle.h"
#include "vpart_position.h"


RealityBubblePathfindingCache *RealityBubblePathfindingCache::global()
{
    static RealityBubblePathfindingCache *global = new RealityBubblePathfindingCache();
    return global;
}

// Modifies `t` to point to a tile with `flag` in a 1-submap radius of `t`'s original value, searching
// nearest points first (starting with `t` itself).
// Returns false if it could not find a suitable point
bool RealityBubblePathfindingCache::vertical_move_destination( ter_furn_flag flag,
        tripoint &t ) const
{
    const int z = t.z;
    if( const std::optional<point> p = find_point_closest_first( t.xy(), 0, SEEX, [this, flag,
          z]( const point & p ) {
    if( p.x >= 0 && p.x < MAPSIZE_X && p.y >= 0 && p.y < MAPSIZE_Y ) {
            const tripoint t2( p, z );
            return get_map().has_flag( flag, t2 );
        }
        return false;
    } ) ) {
        t = tripoint( *p, z );
        return true;
    }
    return false;
}

void RealityBubblePathfindingCache::invalidate_dependants( const tripoint_bub_ms &p )
{
    const auto iter = dependants_by_position_.find( p );
    if( iter != dependants_by_position_.end() ) {
        // This is guaranteed to have at least one element.
        std::vector<tripoint_bub_ms> &dependants = iter->second;
        const tripoint_bub_ms dependant = dependants.back();
        dependants.pop_back();
        if( dependants.empty() ) {
            // This could be recursive and bounce back here. Need to remove
            // it from the dependant map entirely to stop infinite recursion.
            dependants_by_position_.erase( iter );
        }
        invalidate( dependant );
    }
}

void RealityBubblePathfindingCache::update()
{
    for( const int z : dirty_z_levels_ ) {
        for( int y = 0; y < MAPSIZE_Y; ++y ) {
            for( int x = 0; x < MAPSIZE_X; ++x ) {
                const tripoint_bub_ms p( x, y, z );
                invalidate_dependants( p );
                update( p );
            }
        }
    }
    for( const int z : dirty_z_levels_ ) {
        dirty_positions_.erase( z );
    }
    dirty_z_levels_.clear();

    for( const auto& [z, dirty_points] : dirty_positions_ ) {
        for( const point_bub_ms &p : dirty_points ) {
            update( tripoint_bub_ms( p, z ) );
        }
    }
    dirty_positions_.clear();
}

RealityBubblePathfindingCache::RealityBubblePathfindingCache()
{
    for( int z = -OVERMAP_DEPTH; z <= OVERMAP_HEIGHT; ++z ) {
        dirty_z_levels_.emplace( z );
    }
}

void RealityBubblePathfindingCache::update( const tripoint_bub_ms &p )
{
    PathfindingFlags flags;

    map &here = get_map();
    const const_maptile &tile = here.maptile_at( p );
    const ter_t &terrain = tile.get_ter_t();
    const ter_id terrain_id = tile.get_ter();
    const furn_t &furniture = tile.get_furn_t();
    const optional_vpart_position veh = here.veh_at( p );

    const int cost = here.move_cost( p );
    move_cost_ref( p ) = cost;

    if( cost > 2 ) {
        flags |= PathfindingFlag::Slow;
    }

    if( !terrain.has_flag( ter_furn_flag::TFLAG_BURROWABLE ) ||
        !terrain.has_flag( ter_furn_flag::TFLAG_DIGGABLE ) ) {
        flags |= PathfindingFlag::HardGround;
    }

    if( cost == 0 ) {
        flags |= PathfindingFlag::Obstacle;

        bool impassable = flags.is_set( PathfindingFlag::HardGround );
        if( const auto bash_range = here.bash_range( p.raw() ) ) {
            impassable = false;
            bash_range_ref( p ) = *bash_range;
        } else {
            bash_range_ref( p ) = { std::numeric_limits<int>::max() - 1, std::numeric_limits<int>::max() };
        }

        if( terrain.open || furniture.open ) {
            impassable = false;
            flags |= PathfindingFlag::Door;

            if( terrain.has_flag( ter_furn_flag::TFLAG_OPENCLOSE_INSIDE ) ||
                furniture.has_flag( ter_furn_flag::TFLAG_OPENCLOSE_INSIDE ) ) {
                flags |= PathfindingFlag::InsideDoor;
            }
        }

        if( terrain.has_flag( ter_furn_flag::TFLAG_CLIMBABLE ) ) {
            impassable = false;
            flags |= PathfindingFlag::Climmable;
        }

        if( impassable ) {
            flags |= PathfindingFlag::Impassable;
        }
    }

    if( terrain.has_flag( ter_furn_flag::TFLAG_NO_FLOOR ) ) {
        flags |= PathfindingFlag::Air;
    } else if( terrain.has_flag( ter_furn_flag::TFLAG_SWIMMABLE ) ) {
        flags |= PathfindingFlag::Swimmable;
    } else {
        flags |= PathfindingFlag::Ground;
    }

    const bool has_vehicle_floor = here.has_vehicle_floor( p );
    if( !has_vehicle_floor ) {
        if( terrain_id == t_pit || terrain_id == t_pit_spiked || terrain_id == t_pit_glass ) {
            flags |= PathfindingFlag::Pit;
        }

        if( terrain_id == t_lava ) {
            flags |= PathfindingFlag::Lava;
        }

        if( terrain.has_flag( ter_furn_flag::TFLAG_DEEP_WATER ) ) {
            flags |= PathfindingFlag::DeepWater;
        }

        if( !tile.get_trap_t().is_benign() ) {
            flags |= PathfindingFlag::DangerousTrap;
        }

        if( terrain.has_flag( ter_furn_flag::TFLAG_SHARP ) ) {
            flags |= PathfindingFlag::Sharp;
        }
    }

    if( terrain.has_flag( ter_furn_flag::TFLAG_BURROWABLE ) ) {
        flags |= PathfindingFlag::Burrowable;
    }

    if( terrain.has_flag( ter_furn_flag::TFLAG_SMALL_PASSAGE ) ) {
        flags |= PathfindingFlag::SmallPassage;
    }

    if( !g->is_sheltered( p.raw() ) ) {
        flags |= PathfindingFlag::Unsheltered;
    }

    if( veh ) {
        flags |= PathfindingFlag::Vehicle;

        if( const auto vpobst = veh->obstacle_at_part() ) {
            const int vpobst_i = vpobst->part_index();
            const vehicle &v = veh->vehicle();
            const int open_inside = v.next_part_to_open( vpobst_i, false );
            if( open_inside != -1 ) {
                flags |= PathfindingFlag::Door;

                const int open_outside = v.next_part_to_open( vpobst_i, true );
                if( open_inside != open_outside ) {
                    flags |= PathfindingFlag::InsideDoor;
                }
                const int lock = v.next_part_to_unlock( vpobst_i, false );
                if( lock != -1 ) {
                    flags |= PathfindingFlag::LockedDoor;
                }
            }
        }
    }

    for( const auto &fld : tile.get_field() ) {
        const field_entry &cur = fld.second;
        if( cur.is_dangerous() ) {
            flags |= PathfindingFlag::DangerousField;
            break;
        }
    }

    if( p.z() < OVERMAP_HEIGHT ) {
        up_stair_destinations_.erase( p );
        const tripoint_bub_ms up( p.xy(), p.z() + 1 );
        if( terrain.has_flag( ter_furn_flag::TFLAG_GOES_UP ) ) {
            bool rope_ladder = false;
            if( std::optional<tripoint> dest = g->find_or_make_stairs( get_map(),
                                               p.z() - 1, rope_ladder, false, p.raw() ) ) {
                if( vertical_move_destination( ter_furn_flag::TFLAG_GOES_DOWN, *dest ) ) {
                    tripoint_bub_ms d( *dest );
                    flags |= PathfindingFlag::GoesUp;
                    up_stair_destinations_.emplace( p, d );
                    dependants_by_position_[d].push_back( p );
                }
            } else {
                dependants_by_position_[up].push_back( p );
            }
        }

        if( terrain.has_flag( ter_furn_flag::TFLAG_RAMP ) ||
            terrain.has_flag( ter_furn_flag::TFLAG_RAMP_UP ) ) {
            dependants_by_position_[up].push_back( p );
            if( ( terrain.has_flag( ter_furn_flag::TFLAG_RAMP ) &&
                  get_map().valid_move( p.raw(), up.raw(), false, true, true ) ) ||
                ( terrain.has_flag( ter_furn_flag::TFLAG_RAMP_UP ) &&
                  get_map().valid_move( p.raw(), up.raw(), false, true ) ) ) {
                flags |= PathfindingFlag::RampUp;
            }
        }
    }

    if( p.z() > -OVERMAP_DEPTH ) {
        down_stair_destinations_.erase( p );
        const tripoint_bub_ms down( p.xy(), p.z() - 1 );
        if( terrain.has_flag( ter_furn_flag::TFLAG_GOES_DOWN ) ) {
            bool rope_ladder = false;
            if( std::optional<tripoint> dest = g->find_or_make_stairs( get_map(),
                                               p.z(), rope_ladder, false, p.raw() ) ) {
                if( vertical_move_destination( ter_furn_flag::TFLAG_GOES_UP, *dest ) ) {
                    tripoint_bub_ms d( *dest );
                    flags |= PathfindingFlag::GoesDown;
                    down_stair_destinations_.emplace( p, d );
                    dependants_by_position_[d].push_back( p );
                }
            } else {
                dependants_by_position_[down].push_back( p );
            }
        }

        if( terrain.has_flag( ter_furn_flag::TFLAG_RAMP_DOWN ) ) {
            dependants_by_position_[down].push_back( p );
            if( get_map().valid_move( p.raw(), down.raw(), false, true, true ) ) {
                flags |= PathfindingFlag::RampDown;
            }
        }
    }

    flags_ref( p ) = flags;
}

RealityBubblePathfinder *RealityBubblePathfinder::global()
{
    static RealityBubblePathfinder *global = new RealityBubblePathfinder();
    return global;
}

int CreaturePathfindingSettings::bash_rating_from_range( int min, int max ) const
{
    // TODO: Move all the bash stuff to map so this logic isn't duplicated.
    ///\EFFECT_STR increases smashing damage
    if( bash_strength_ < min ) {
        return 0;
    } else if( bash_strength_ >= max ) {
        return 10;
    }
    const double ret = ( 10.0 * ( bash_strength_ - min ) ) / ( max - min );
    // Round up to 1, so that desperate NPCs can try to bash down walls
    return std::max( ret, 1.0 );
}

namespace
{

std::optional<int> move_cost_internal( const CreaturePathfindingSettings &settings,
                                       const RealityBubblePathfindingCache &cache, const tripoint_bub_ms &from, const tripoint_bub_ms &to )
{
    const PathfindingFlags flags = cache.flags( to );
    if( flags & settings.avoid_mask() ) {
        return std::nullopt;
    }

    const bool is_vertical_movement = from.z() != to.z();
    if( is_vertical_movement ) {
        const tripoint_bub_ms &upper = from.z() > to.z() ? from : to;
        const tripoint_bub_ms &lower = from.z() < to.z() ? from : to;
        if( cache.flags( lower ).is_set( PathfindingFlag::GoesUp ) &&
            cache.flags( upper ).is_set( PathfindingFlag::GoesDown ) ) {
            if( settings.avoid_climb_stairway() ) {
                return std::nullopt;
            }
        } else if( settings.is_flying() ) {
            const tripoint_bub_ms below_upper( upper.xy(), upper.z() - 1 );
            const tripoint_bub_ms above_lower( lower.xy(), lower.z() + 1 );
            if( !( cache.flags( below_upper ).is_set( PathfindingFlag::Air ) ||
                   cache.flags( above_lower ).is_set( PathfindingFlag::Air ) ) ) {
                return std::nullopt;
            }
        } else {
            const PathfindingFlags from_flags = cache.flags( from );
            if( !( from.z() < to.z() && from_flags.is_set( PathfindingFlag::RampUp ) ) ||
                !( from.z() > to.z() && from_flags.is_set( PathfindingFlag::RampDown ) ) ) {
                return std::nullopt;
            }
        }
    }

    constexpr int turn_cost = 2;
    int extra_cost = 0;
    if( flags.is_set( PathfindingFlag::Obstacle ) ) {
        if( settings.is_digging() ) {
            if( !flags.is_set( PathfindingFlag::Burrowable ) ) {
                return std::nullopt;
            }
        } else if( flags.is_set( PathfindingFlag::Climmable ) && !settings.avoid_climbing() &&
                   settings.climb_cost() > 0 ) {
            // Climbing fences
            extra_cost += settings.climb_cost();
        } else if( flags.is_set( PathfindingFlag::Door ) && !settings.avoid_opening_doors() &&
                   ( !flags.is_set( PathfindingFlag::LockedDoor ) || !settings.avoid_unlocking_doors() ) ) {
            const bool is_inside_door = flags.is_set( PathfindingFlag::InsideDoor );
            if( is_inside_door ) {
                const map &here = get_map();
                int dummy;
                const bool is_vehicle = flags.is_set( PathfindingFlag::Vehicle );
                const bool is_outside = is_vehicle ? here.veh_at_internal( from.raw(),
                                        dummy ) != here.veh_at_internal( to.raw(), dummy ) : here.is_outside( from.raw() );
                if( is_outside ) {
                    return std::nullopt;
                }
            }
            const bool is_locked_door = flags.is_set( PathfindingFlag::LockedDoor );
            if( is_locked_door ) {
                // An extra turn to unlock.
                extra_cost += turn_cost;
            }
            // One turn to open, one turn to move to the tile.
            extra_cost += 2 * turn_cost;
        } else {
            const auto [bash_min, bash_max] = cache.bash_range( to );
            const int bash_rating = settings.bash_rating_from_range( bash_min, bash_max );
            if( bash_rating >= 1 ) {
                // Expected number of turns to bash it down, 1 turn to move there
                extra_cost += ( 20 / bash_rating ) + turn_cost;
            } else {
                // Unbashable and unopenable from here
                return std::nullopt;
            }
        }
    }

    const auto &maybe_avoid_dangerous_fields_fn = settings.maybe_avoid_dangerous_fields_fn();
    if( flags.is_set( PathfindingFlag::DangerousField ) && maybe_avoid_dangerous_fields_fn ) {
        const field &target_field = get_map().field_at( to );
        for( const auto &dfield : target_field ) {
            if( dfield.second.is_dangerous() && maybe_avoid_dangerous_fields_fn( dfield.first ) ) {
                return std::nullopt;
            }
        }
    }

    const auto &maybe_avoid_fn = settings.maybe_avoid_fn();
    if( maybe_avoid_fn && maybe_avoid_fn( to ) ) {
        return std::nullopt;
    }

    // TODO: Move the move cost cache into map so this logic isn't duplicated.
    static constexpr std::array<int, 4> mults = { 0, 50, 71, 100 };
    // Multiply cost depending on the number of differing axes
    // 0 if all axes are equal, 100% if only 1 differs, 141% for 2, 200% for 3
    const std::size_t match = trigdist ? ( from.x() != to.x() ) + ( from.y() != to.y() ) +
                              is_vertical_movement : 1;
    const int cost = cache.move_cost( from ) + cache.move_cost( to );
    return cost * mults[match] / 2 + extra_cost * 50;
}

}  // namespace

bool CreaturePathfinder::can_move( const CreaturePathfindingSettings &settings,
                                   const tripoint_bub_ms &from, const tripoint_bub_ms &to ) const
{
    if (from == to) {
        return true;
    }
    cache_->update();
    return move_cost_internal( settings, *cache_, from, to ).has_value();
}

std::optional<int> CreaturePathfinder::move_cost( const CreaturePathfindingSettings &settings,
        const tripoint_bub_ms &from, const tripoint_bub_ms &to ) const
{
    if (from == to) {
        return 0;
    }
    cache_->update();
    return move_cost_internal( settings, *cache_, from, to );
}

std::vector<tripoint_bub_ms> CreaturePathfinder::find_path( const CreaturePathfindingSettings
        &settings, const tripoint_bub_ms &from, const tripoint_bub_ms &to ) const
{
    if (from == to) {
        return {};
    }
    // Pathfinder will update cache.
    return pathfinder_->find_path( settings.rb_settings(), from, to,
    [this, &settings]( const tripoint_bub_ms & from, const tripoint_bub_ms & to ) {
        return move_cost_internal( settings, *cache_, from, to );
    },
    [to]( const tripoint_bub_ms & from ) {
        return 100 * rl_dist( from, to );
    } );
}

CreaturePathfindingSettings pathfinding_settings::to_creature_pathfinding_settings() const
{
    CreaturePathfindingSettings settings;
    settings.set_bash_strength( bash_strength );
    settings.set_max_distance( max_dist );
    settings.set_max_cost( max_length * 50 );
    settings.set_climb_cost( climb_cost );
    settings.set_avoid_opening_doors( !allow_open_doors );
    settings.set_avoid_unlocking_doors( !allow_unlock_doors );
    settings.set_avoid_dangerous_traps( avoid_traps );
    if( avoid_rough_terrain ) {
        settings.set_avoid_rough_terrain( true );
    }
    settings.set_avoid_sharp( avoid_sharp );
    return settings;
}

std::vector<tripoint> map::route( const tripoint &f, const tripoint &t,
                                  const pathfinding_settings &settings,
                                  const std::set<tripoint> &pre_closed ) const
{
    /* TODO: If the origin or destination is out of bound, figure out the closest
     * in-bounds point and go to that, then to the real origin/destination.
     */
    std::vector<tripoint> ret;

    if( f == t || !inbounds( f ) ) {
        return ret;
    }

    if( !inbounds( t ) ) {
        tripoint clipped = t;
        clip_to_bounds( clipped );
        return route( f, clipped, settings, pre_closed );
    }

    CreaturePathfinder pathfinder;
    CreaturePathfindingSettings new_settings = settings.to_creature_pathfinding_settings();
    new_settings.set_maybe_avoid_fn( [&pre_closed]( const tripoint_bub_ms & p ) {
        return pre_closed.count( p.raw() ) == 1;
    } );
    for( const tripoint_bub_ms &p : pathfinder.find_path( new_settings, tripoint_bub_ms( f ),
            tripoint_bub_ms( t ) ) ) {
        ret.push_back( p.raw() );
    }

    return ret;
}

std::vector<tripoint_bub_ms> map::route( const tripoint_bub_ms &f, const tripoint_bub_ms &t,
        const pathfinding_settings &settings,
        const std::set<tripoint> &pre_closed ) const
{
    std::vector<tripoint> raw_result = route( f.raw(), t.raw(), settings, pre_closed );
    std::vector<tripoint_bub_ms> result;
    std::transform( raw_result.begin(), raw_result.end(), std::back_inserter( result ),
    []( const tripoint & p ) {
        return tripoint_bub_ms( p );
    } );
    return result;
}
