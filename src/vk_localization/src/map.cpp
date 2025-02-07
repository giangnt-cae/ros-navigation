#include "vk_localization/map.hpp"

map_t *map_alloc() {
    map_t *map;
    map = new map_t;
    map->origin_x = 0.0;
    map->origin_y = 0.0;

    map->size_x = 0;
    map->size_y = 0;
    map->scale = 0;

    map->cells = (map_cell_t*) NULL;
    return map;
}

void map_delete(map_t *map) {
    delete[] map->cells;
    map->cells = NULL;
    delete[] map;
    map = NULL;
}

map_cell_t *map_get_cell(map_t *map, const Eigen::Vector2d& point) {
    int i, j;
    map_cell_t *cell;
    i = MAP_GXWX(map, point[0]);
    j = MAP_GYWY(map, point[1]);
    if(!MAP_VALID(map, i, j)) return NULL;

    cell = map->cells + MAP_INDEX(map, i, j);
    return cell;
}

void map_update_cspace(map_t *map, double max_occ_dist) {
    int i, j;
    int ni, nj;
    int s;
    double d;
    map_cell_t *cell, *ncell;
    map->max_occ_dist = max_occ_dist;
    s = (int) ceil(map->max_occ_dist / map->scale);
    
    // Reset the distance values;
    for(j = 0; j < map->size_y; j++) {
        for(i = 0; i < map->size_x; i++) {
            cell = map->cells + MAP_INDEX(map, i, j);
            cell->occ_dist = map->max_occ_dist;
        }
    }

    // Find all the occupied cells and update their neighbours
    for(j = 0; j < map->size_y; j++) {
        for(i = 0; i < map->size_x; i++) {
            cell = map->cells + MAP_INDEX(map, i, j);
            if(cell->occ_state != 1) continue;
            cell->occ_dist = 0;
            cell->nearest_cell = {MAP_WXGX(map, i), MAP_WYGY(map, j)};
            for(nj = -s; nj <= +s; nj++) {
                for(ni = -s; ni <= +s; ni++) {
                    if(!MAP_VALID(map, i + ni, j + nj)) continue;
                    ncell = map->cells + MAP_INDEX(map, i + ni, j + nj);
                    d = map->scale * sqrt(ni*ni + nj*nj);
                    if(d < ncell->occ_dist) {
                        ncell->occ_dist = d;
                        ncell->nearest_cell = {MAP_WXGX(map, i), MAP_WYGY(map, j)};
                    }
                }
            }
        }
    }
}
