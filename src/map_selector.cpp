#include "map_selector.h"

#include <functional>
#include <memory>
#include <new>
#include <optional>
#include <vector>

#include "game_constants.h"
#include "map.h"
#include "map_iterator.h"
#include "rng.h"

class game;
// NOLINTNEXTLINE(cata-static-declarations)
extern std::unique_ptr<game> g;

map_selector::map_selector( const tripoint_bub_ms_ib &pos, int radius, bool accessible )
{
    map& here = get_map();
    for( const tripoint_bub_ms &e : closest_points_first( pos, radius ) ) {
        if( !accessible ) {
            data.emplace_back( e );
        }
        if (const std::optional< tripoint_bub_ms_ib> ib = here.make_inbounds(e);
            ib&& here.clear_path(pos, *ib, radius, 1, 100)) {
            data.emplace_back(e);
        }
    }
}

tripoint_range<tripoint> points_in_range( const map &m )
{
    return tripoint_range<tripoint>(
               tripoint( 0, 0, -OVERMAP_DEPTH ),
               tripoint( SEEX * m.getmapsize() - 1, SEEY * m.getmapsize() - 1, OVERMAP_HEIGHT ) );
}

std::optional<tripoint> random_point( const map &m,
                                      const std::function<bool( const tripoint & )> &predicate )
{
    return random_point( points_in_range( m ), predicate );
}

std::optional<tripoint> random_point( const tripoint_range<tripoint> &range,
                                      const std::function<bool( const tripoint & )> &predicate )
{
    // Optimist approach: just assume there are plenty of suitable places and a randomly
    // chosen point will have a good chance to hit one of them.
    // If there are only few suitable places, we have to find them all, otherwise this loop may never finish.
    for( int tries = 0; tries < 10; ++tries ) {
        const tripoint p( rng( range.min().x, range.max().x ), rng( range.min().y, range.max().y ),
                          rng( range.min().z, range.max().z ) );
        if( predicate( p ) ) {
            return p;
        }
    }
    std::vector<tripoint> suitable;
    for( const tripoint &p : range ) {
        if( predicate( p ) ) {
            suitable.push_back( p );
        }
    }
    if( suitable.empty() ) {
        return {};
    }
    return random_entry( suitable );
}
