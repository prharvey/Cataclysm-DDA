#include "submap.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>
#include <utility>

#include "basecamp.h"
#include "mapdata.h"
#include "tileray.h"
#include "trap.h"
#include "units.h"
#include "vehicle.h"

static const furn_str_id furn_f_console( "f_console" );

void maptile_soa::swap_soa_tile( const point_sm_ms_ib&p1, const point_sm_ms_ib&p2 )
{
    std::swap( ter[p1], ter[p2] );
    std::swap( frn[p1], frn[p2] );
    std::swap( lum[p1], lum[p2] );
    std::swap( itm[p1], itm[p2] );
    std::swap( fld[p1], fld[p2] );
    std::swap( trp[p1], trp[p2] );
    std::swap( rad[p1], rad[p2] );
}

submap::submap( submap && ) noexcept( map_is_noexcept ) = default;
submap::~submap() = default;

submap &submap::operator=( submap && ) noexcept = default;

void submap::clear_fields( const point_sm_ms_ib&p )
{
    field &f = get_field( p );
    field_count -= f.field_count();
    f.clear();
}

static const std::string COSMETICS_GRAFFITI( "GRAFFITI" );
static const std::string COSMETICS_SIGNAGE( "SIGNAGE" );
// Handle GCC warning: 'warning: returning reference to temporary'
static const std::string STRING_EMPTY;

struct cosmetic_find_result {
    bool result = false;
    int ndx = 0;
};
static cosmetic_find_result make_result( bool b, int ndx )
{
    cosmetic_find_result result;
    result.result = b;
    result.ndx = ndx;
    return result;
}
static cosmetic_find_result find_cosmetic(
    const std::vector<submap::cosmetic_t> &cosmetics, const point_sm_ms_ib&p, const std::string &type )
{
    for( size_t i = 0; i < cosmetics.size(); ++i ) {
        if( cosmetics[i].pos == p && cosmetics[i].type == type ) {
            return make_result( true, i );
        }
    }
    return make_result( false, -1 );
}

bool submap::has_graffiti( const point_sm_ms_ib&p ) const
{
    return find_cosmetic( cosmetics, p, COSMETICS_GRAFFITI ).result;
}

const std::string &submap::get_graffiti( const point_sm_ms_ib&p ) const
{
    const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_GRAFFITI );
    if( fresult.result ) {
        return cosmetics[ fresult.ndx ].str;
    }
    return STRING_EMPTY;
}

void submap::set_graffiti( const point_sm_ms_ib&p, const std::string &new_graffiti )
{
    ensure_nonuniform();
    // Find signage at p if available
    const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_GRAFFITI );
    if( fresult.result ) {
        cosmetics[ fresult.ndx ].str = new_graffiti;
    } else {
        insert_cosmetic( p, COSMETICS_GRAFFITI, new_graffiti );
    }
}

void submap::delete_graffiti( const point_sm_ms_ib&p )
{
    const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_GRAFFITI );
    if( fresult.result ) {
        ensure_nonuniform();
        cosmetics[ fresult.ndx ] = cosmetics.back();
        cosmetics.pop_back();
    }
}
bool submap::has_signage( const point_sm_ms_ib&p ) const
{
    if( !is_uniform() && m->frn[p]->has_flag( ter_furn_flag::TFLAG_SIGN ) ) {
        return find_cosmetic( cosmetics, p, COSMETICS_SIGNAGE ).result;
    }

    return false;
}
std::string submap::get_signage( const point_sm_ms_ib&p ) const
{
    if( !is_uniform() && m->frn[p]->has_flag( ter_furn_flag::TFLAG_SIGN ) ) {
        const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_SIGNAGE );
        if( fresult.result ) {
            return cosmetics[ fresult.ndx ].str;
        }
    }

    return STRING_EMPTY;
}
void submap::set_signage( const point_sm_ms_ib&p, const std::string &s )
{
    ensure_nonuniform();
    // Find signage at p if available
    const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_SIGNAGE );
    if( fresult.result ) {
        cosmetics[ fresult.ndx ].str = s;
    } else {
        insert_cosmetic( p, COSMETICS_SIGNAGE, s );
    }
}
void submap::delete_signage( const point_sm_ms_ib&p )
{
    const cosmetic_find_result fresult = find_cosmetic( cosmetics, p, COSMETICS_SIGNAGE );
    if( fresult.result ) {
        ensure_nonuniform();
        cosmetics[ fresult.ndx ] = cosmetics.back();
        cosmetics.pop_back();
    }
}

void submap::update_legacy_computer()
{
    if( !is_uniform() && legacy_computer ) {
        for( int x = 0; x < SEEX; ++x ) {
            for( int y = 0; y < SEEY; ++y ) {
                if( m->frn[x][y] == furn_f_console ) {
                    computers.emplace(point_sm_ms_ib::make_unchecked( x, y ), *legacy_computer );
                }
            }
        }
        legacy_computer.reset();
    }
}

bool submap::has_computer( const point_sm_ms_ib&p ) const
{
    return !is_uniform() && ( computers.find( p ) != computers.end() ||
                              ( legacy_computer && m->frn[p] == furn_f_console ) );
}

const computer *submap::get_computer( const point_sm_ms_ib&p ) const
{
    // the returned object will not get modified (should not, at least), so we
    // don't yet need to update to std::map
    const auto it = computers.find( p );
    if( it != computers.end() ) {
        return &it->second;
    }
    if( !is_uniform() && legacy_computer && m->frn[p] == furn_f_console ) {
        return legacy_computer.get();
    }
    return nullptr;
}

computer *submap::get_computer( const point_sm_ms_ib&p )
{
    // need to update to std::map first so modifications to the returned object
    // only affects the exact point p
    //update_legacy_computer();
    const auto it = computers.find( p );
    if( it != computers.end() ) {
        return &it->second;
    }
    return nullptr;
}

void submap::set_computer( const point_sm_ms_ib&p, const computer &c )
{
    //update_legacy_computer();
    const auto it = computers.find( p );
    if( it != computers.end() ) {
        it->second = c;
    } else {
        computers.emplace( p, c );
    }
}

void submap::delete_computer( const point_sm_ms_ib&p )
{
    update_legacy_computer();
    computers.erase( p );
}

bool submap::contains_vehicle( vehicle *veh )
{
    const auto match = std::find_if(
                           begin( vehicles ), end( vehicles ),
    [veh]( const std::unique_ptr<vehicle> &v ) {
        return v.get() == veh;
    } );
    return match != vehicles.end();
}

bool submap::is_open_air( const point_sm_ms_ib&p ) const
{
    ter_id t = get_ter( p );
    return t->trap == tr_ledge;
}

void submap::rotate( int turns )
{
    if( is_uniform() ) {
        return;
    }
    turns = turns % 4;

    if( turns == 0 ) {
        return;
    }

    const auto rotate_point = [turns]( const point_sm_ms_ib& p ) {
        return point_sm_ms_ib::make_unchecked(p.raw().rotate(turns, { SEEX, SEEY }));
    };
    const auto rotate_point_ccw = [turns]( const point_sm_ms_ib& p ) {
        return point_sm_ms_ib::make_unchecked(p.raw().rotate(4 - turns, { SEEX, SEEY }));
    };

    if( turns == 2 ) {
        // Swap horizontal stripes.
        for( int j = 0, je = SEEY / 2; j < je; ++j ) {
            for( int i = j, ie = SEEX - j; i < ie; ++i ) {
                const point_sm_ms_ib p = point_sm_ms_ib::make_unchecked(i, j);
                m->swap_soa_tile(p, rotate_point( p ) );
            }
        }
        // Swap vertical stripes so that they don't overlap with
        // the already swapped horizontals.
        for( int i = 0, ie = SEEX / 2; i < ie; ++i ) {
            for( int j = i + 1, je = SEEY - i - 1; j < je; ++j ) {
                const point_sm_ms_ib p = point_sm_ms_ib::make_unchecked(i, j);
                m->swap_soa_tile(p, rotate_point( p ) );
            }
        }
    } else {
        for( int j = 0, je = SEEY / 2; j < je; ++j ) {
            for( int i = j, ie = SEEX - j - 1; i < ie; ++i ) {
                point_sm_ms_ib p = point_sm_ms_ib::make_unchecked( i, j );
                point_sm_ms_ib pp = p;
                // three swaps are enough to perform the circular shift of four elements:
                // 0123 -> 3120 -> 3102 -> 3012
                for( int k = 0; k < 3; ++k ) {
                    p = pp;
                    pp = rotate_point_ccw( pp );
                    m->swap_soa_tile( p, pp );
                }
            }
        }
    }

    active_items.rotate_locations( turns, { SEEX, SEEY } );

    for( submap::cosmetic_t &elem : cosmetics ) {
        elem.pos = rotate_point( elem.pos );
    }

    for( spawn_point &elem : spawns ) {
        elem.pos = rotate_point( elem.pos );
    }

    const auto rotate_point_ob = [turns](const point_sm_ms& p) {
        return point_sm_ms::make_unchecked(p.raw().rotate(turns, { SEEX, SEEY }));
        };
    for( auto &elem : vehicles ) {
        const point_sm_ms new_pos = rotate_point_ob( elem->pos );

        elem->pos = new_pos;
        // turn the steering wheel, vehicle::turn does not actually
        // move the vehicle.
        elem->turn( turns * 90_degrees );
        // The facing direction and recalculate the positions of the parts
        elem->face = tileray( elem->turn_dir );
        elem->precalc_mounts( 0, elem->turn_dir, elem->pivot_anchor[0] );
    }

    std::map<point_sm_ms_ib, computer> rot_comp;
    for( auto &elem : computers ) {
        rot_comp.emplace( rotate_point( elem.first ), elem.second );
    }
    computers = rot_comp;
}

void submap::mirror( bool horizontally )
{
    if( is_uniform() ) {
        return;
    }
    std::map<point_sm_ms_ib, computer> mirror_comp;

    if( horizontally ) {
        for( int i = 0, ie = SEEX / 2; i < ie; i++ ) {
            for( int k = 0; k < SEEY; k++ ) {
                m->swap_soa_tile(point_sm_ms_ib::make_unchecked( i, k ), 
                    point_sm_ms_ib::make_unchecked(SEEX - 1 - i, k ) );
            }
        }

        for( submap::cosmetic_t &elem : cosmetics ) {
            elem.pos = point_sm_ms_ib::make_unchecked(-elem.pos.x() + SEEX - 1, elem.pos.y());
        }

        active_items.mirror( { SEEX, SEEY }, true );

        for( auto &elem : computers ) {
            mirror_comp.emplace(point_sm_ms_ib::make_unchecked(-elem.first.x() + SEEX - 1, elem.first.y()), elem.second );
        }
        computers = mirror_comp;
    } else {
        for( int k = 0, ke = SEEY / 2; k < ke; k++ ) {
            for( int i = 0; i < SEEX; i++ ) {
                m->swap_soa_tile(point_sm_ms_ib::make_unchecked( i, k ), point_sm_ms_ib::make_unchecked( i, SEEY - 1 - k ) );
            }
        }

        for( submap::cosmetic_t &elem : cosmetics ) {
            elem.pos = point_sm_ms_ib::make_unchecked(elem.pos.x(), -elem.pos.y() + SEEY - 1);
        }

        active_items.mirror( { SEEX, SEEY }, false );

        for( auto &elem : computers ) {
            mirror_comp.emplace(point_sm_ms_ib::make_unchecked(elem.first.x(), -elem.first.y() + SEEY - 1), elem.second );
        }
        computers = mirror_comp;
    }
}

void submap::revert_submap( submap &sr )
{
    reverted = true;
    if( sr.is_uniform() ) {
        m.reset();
        set_all_ter( sr.get_ter(point_sm_ms_ib()), true );
        return;
    }

    ensure_nonuniform();
    for( int x = 0; x < SEEX; x++ ) {
        for( int y = 0; y < SEEY; y++ ) {
            point_sm_ms_ib pt = point_sm_ms_ib::make_unchecked( x, y );
            m->frn[x][y] = sr.get_furn( pt );
            m->ter[x][y] = sr.get_ter( pt );
            m->trp[x][y] = sr.get_trap( pt );
            m->itm[x][y] = sr.get_items( pt );
            for( item &itm : m->itm[x][y] ) {
                if( itm.is_emissive() ) {
                    this->update_lum_add( pt, itm );
                }
                active_items.add( itm, pt.raw() );
            }
        }
    }
}

submap submap::get_revert_submap() const
{
    submap ret;
    ret.uniform_ter = uniform_ter;
    if( !is_uniform() ) {
        ret.m = std::make_unique<maptile_soa>( *m );
    }

    return ret;
}

void submap::update_lum_rem( const point_sm_ms_ib&p, const item &i )
{
    ensure_nonuniform();
    if( !i.is_emissive() ) {
        return;
    } else if( m->lum[p] && m->lum[p] < 255 ) {
        m->lum[p]--;
        return;
    }

    // Have to scan through all items to be sure removing i will actually lower
    // the count below 255.
    int count = 0;
    for( const item &it : m->itm[p] ) {
        if( it.is_emissive() ) {
            count++;
        }
    }

    if( count <= 256 ) {
        m->lum[p] = static_cast<uint8_t>( count - 1 );
    }
}

submap null_submap;
