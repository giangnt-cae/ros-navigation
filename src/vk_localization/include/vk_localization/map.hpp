#pragma once

#include "header.hpp"

struct map_cell_t {
    int occ_state;              // Occupancy state (occupied = 1, free = 0, unknown = -1)
    double occ_dist;            // Distance to nearest occupied cell
    Eigen::Vector2d nearest_cell;
};

struct map_t {
    double origin_x, origin_y;
    double scale;               // Map scale m/cell
    int size_x, size_y;         // Map dimensions - number of cell
    map_cell_t* cells;          // Map data
    double max_occ_dist;
};

map_t *map_alloc();             // Create a new empty map

void map_delete(map_t *map);

map_cell_t *map_get_cell(map_t *map, const Eigen::Vector2d& point); // Get the cell at the given point

int map_load_occ(map_t *map, const char *filename, double scale, int negate);   // Load an occupancy map

void map_update_cspace(map_t *map, double max_occ_dist);    // Update the cspace distances

/* Convert from map index to world coordinates */
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

/* Convert from world coordinates to map index */
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

#define MAP_VALID(map, i, j) ((i >= 0) && (i <= map->size_x) && (j >= 0) && (j <= map->size_y))

#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)