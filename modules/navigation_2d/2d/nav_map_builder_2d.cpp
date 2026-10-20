/**************************************************************************/
/*  nav_map_builder_2d.cpp                                                */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "nav_map_builder_2d.h"

#include "../nav_link_2d.h"
#include "../triangle2.h"
#include "nav_map_iteration_2d.h"
#include "nav_region_iteration_2d.h"

#include "core/config/project_settings.h"

using namespace Nav2D;

PointKey NavMapBuilder2D::get_point_key(const Vector2 &p_pos, const Vector2 &p_cell_size) {
	const int x = static_cast<int>(Math::floor(p_pos.x / p_cell_size.x));
	const int y = static_cast<int>(Math::floor(p_pos.y / p_cell_size.y));

	PointKey p;
	p.key = 0;
	p.x = x;
	p.y = y;
	return p;
}

void NavMapBuilder2D::build_navmap_iteration(NavMapIterationBuild2D &r_build) {
	PerformanceData &performance_data = r_build.performance_data;

	performance_data.pm_polygon_count = 0;
	performance_data.pm_edge_count = 0;
	performance_data.pm_edge_merge_count = 0;
	performance_data.pm_edge_connection_count = 0;
	performance_data.pm_edge_free_count = 0;

	_build_step_gather_region_polygons(r_build);

	_build_step_find_edge_connection_pairs(r_build);

	_build_step_merge_edge_connection_pairs(r_build);

	_build_step_edge_connection_margin_connections(r_build);

	_build_step_navlink_connections(r_build);

	_build_update_map_iteration(r_build);
}

void NavMapBuilder2D::_build_step_gather_region_polygons(NavMapIterationBuild2D &r_build) {
	PerformanceData &performance_data = r_build.performance_data;
	NavMapIteration2D *map_iteration = r_build.map_iteration;

	const LocalVector<Ref<NavRegionIteration2D>> &regions = map_iteration->region_iterations;
	HashMap<const NavBaseIteration2D *, LocalVector<Connection>> &region_external_connections = map_iteration->external_region_connections;

	map_iteration->navbases_polygons_external_connections.clear();

	// Remove regions connections.
	region_external_connections.clear();

	// Copy all region polygons in the map.
	int polygon_count = 0;
	for (const Ref<NavRegionIteration2D> &region : regions) {
		const uint32_t polygons_size = region->navmesh_polygons.size();
		polygon_count += polygons_size;

		region_external_connections[region.ptr()] = LocalVector<Connection>();
		map_iteration->navbases_polygons_external_connections[region.ptr()] = LocalVector<LocalVector<Connection>>();
		map_iteration->navbases_polygons_external_connections[region.ptr()].resize(polygons_size);
	}

	performance_data.pm_polygon_count = polygon_count;
	r_build.polygon_count = polygon_count;
}

void NavMapBuilder2D::_build_step_find_edge_connection_pairs(NavMapIterationBuild2D &r_build) {
	PerformanceData &performance_data = r_build.performance_data;
	NavMapIteration2D *map_iteration = r_build.map_iteration;
	int polygon_count = r_build.polygon_count;

	HashMap<EdgeKey, EdgeConnectionPair, EdgeKey> &connection_pairs_map = r_build.iter_connection_pairs_map;

	// Group all edges per key.
	connection_pairs_map.clear();
	connection_pairs_map.reserve(polygon_count);
	int free_edges_count = 0; // How many ConnectionPairs have only one Connection.
	int edge_merge_error_count = 0;

	for (const Ref<NavRegionIteration2D> &region : map_iteration->region_iterations) {
		for (const ConnectableEdge &connectable_edge : region->get_external_edges()) {
			const EdgeKey &ek = connectable_edge.ek;

			HashMap<EdgeKey, EdgeConnectionPair, EdgeKey>::Iterator pair_it = connection_pairs_map.find(ek);
			if (!pair_it) {
				pair_it = connection_pairs_map.insert(ek, EdgeConnectionPair());
				performance_data.pm_edge_count += 1;
				++free_edges_count;
			}
			EdgeConnectionPair &pair = pair_it->value;
			if (pair.size < 2) {
				// Add the polygon/edge tuple to this key.
				Connection new_connection;
				new_connection.polygon = &region->navmesh_polygons[connectable_edge.polygon_index];
				new_connection.edge = connectable_edge.edge;
				new_connection.pathway_start = connectable_edge.pathway_start;
				new_connection.pathway_end = connectable_edge.pathway_end;

				pair.connections[pair.size] = new_connection;
				++pair.size;
				if (pair.size == 2) {
					--free_edges_count;
				}

			} else {
				// The edge is already connected with another edge, skip.
				edge_merge_error_count++;
			}
		}
	}

	if (edge_merge_error_count > 0 && GLOBAL_GET_CACHED(bool, "navigation/2d/warnings/navmesh_edge_merge_errors")) {
		WARN_PRINT("Navigation map synchronization had " + itos(edge_merge_error_count) + " edge error(s).\nMore than 2 edges tried to occupy the same map rasterization space.\nThis causes a logical error in the navigation mesh geometry and is commonly caused by overlap or too densely placed edges.\nConsider baking with a higher 'cell_size', greater geometry margin, and less detailed bake objects to cause fewer edges.\nConsider lowering the 'navigation/2d/merge_rasterizer_cell_scale' in the project settings.\nThis warning can be toggled under 'navigation/2d/warnings/navmesh_edge_merge_errors' in the project settings.");
	}

	r_build.free_edge_count = free_edges_count;
}

void NavMapBuilder2D::_build_step_merge_edge_connection_pairs(NavMapIterationBuild2D &r_build) {
	PerformanceData &performance_data = r_build.performance_data;

	HashMap<EdgeKey, EdgeConnectionPair, EdgeKey> &connection_pairs_map = r_build.iter_connection_pairs_map;
	LocalVector<Connection> &free_edges = r_build.iter_free_edges;
	int free_edges_count = r_build.free_edge_count;
	bool use_edge_connections = r_build.use_edge_connections;

	free_edges.clear();
	free_edges.reserve(free_edges_count);

	NavMapIteration2D *map_iteration = r_build.map_iteration;

	HashMap<const NavBaseIteration2D *, LocalVector<LocalVector<Nav2D::Connection>>> &navbases_polygons_external_connections = map_iteration->navbases_polygons_external_connections;

	for (const KeyValue<EdgeKey, EdgeConnectionPair> &pair_it : connection_pairs_map) {
		const EdgeConnectionPair &pair = pair_it.value;
		if (pair.size == 2) {
			// Connect edge that are shared in different polygons.
			const Connection &c1 = pair.connections[0];
			const Connection &c2 = pair.connections[1];

			navbases_polygons_external_connections[c1.polygon->owner][c1.polygon->id].push_back(c2);
			navbases_polygons_external_connections[c2.polygon->owner][c2.polygon->id].push_back(c1);
			performance_data.pm_edge_connection_count += 1;

		} else {
			CRASH_COND_MSG(pair.size != 1, vformat("Number of connection != 1. Found: %d", pair.size));
			if (use_edge_connections && pair.connections[0].polygon->owner->get_use_edge_connections()) {
				free_edges.push_back(pair.connections[0]);
			}
		}
	}
}

// Reject any gap whose bridge quad the triangle fan and the funnel cannot represent.
//
// One same-sign test over the four turns rejects every bad case at once: bow-ties from
// crossed edges or reversed endpoint correspondence, concave quads the fan from vertex 0
// would misrepresent, and collinear zero-area quads. The epsilon scales with the quad
// extent so a pixel-sized bridge is held to the same relative tolerance as a tile-sized one.
static bool _validate_edge_bridge_quad(const Vector2 &p_a1, const Vector2 &p_a2, const Vector2 &p_b1, const Vector2 &p_b2, real_t &r_surface_area) {
	if (!p_a1.is_finite() || !p_a2.is_finite() || !p_b1.is_finite() || !p_b2.is_finite()) {
		return false;
	}

	const Vector2 quad[4] = { p_a1, p_a2, p_b2, p_b1 };

	real_t longest_side_squared = 0.0;
	for (int i = 0; i < 4; i++) {
		longest_side_squared = MAX(longest_side_squared, (quad[(i + 1) % 4] - quad[i]).length_squared());
	}
	if (longest_side_squared <= (real_t)CMP_EPSILON * (real_t)CMP_EPSILON) {
		return false;
	}
	const real_t epsilon = Math::sqrt(longest_side_squared) * (real_t)CMP_EPSILON;
	const real_t area_epsilon = epsilon * epsilon;

	// Both portals have to keep a usable width or the funnel receives a point gate.
	if ((p_a2 - p_a1).length() <= epsilon || (p_b2 - p_b1).length() <= epsilon) {
		return false;
	}

	int winding = 0;
	for (int i = 0; i < 4; i++) {
		const Vector2 side = quad[(i + 1) % 4] - quad[i];
		const Vector2 next_side = quad[(i + 2) % 4] - quad[(i + 1) % 4];
		const real_t turn = side.cross(next_side);
		if (Math::abs(turn) <= area_epsilon) {
			return false;
		}
		const int turn_winding = turn > 0.0 ? 1 : -1;
		if (winding == 0) {
			winding = turn_winding;
		} else if (turn_winding != winding) {
			return false;
		}
	}

	real_t double_area = 0.0;
	for (int i = 0; i < 4; i++) {
		const Vector2 &current = quad[i];
		const Vector2 &next = quad[(i + 1) % 4];
		double_area += current.x * next.y - next.x * current.y;
	}
	r_surface_area = Math::abs(double_area) * 0.5;

	return r_surface_area > area_epsilon;
}

static void _add_direct_edge_connection(const Connection &p_source_edge, const Connection &p_destination_edge, const Vector2 &p_pathway_start, const Vector2 &p_pathway_end, NavMapIteration2D *p_map_iteration, PerformanceData &r_performance_data) {
	Connection new_connection = p_destination_edge;
	new_connection.pathway_start = p_pathway_start;
	new_connection.pathway_end = p_pathway_end;

	p_map_iteration->external_region_connections[p_source_edge.polygon->owner].push_back(new_connection);
	p_map_iteration->navbases_polygons_external_connections[p_source_edge.polygon->owner][p_source_edge.polygon->id].push_back(new_connection);
	r_performance_data.pm_edge_connection_count += 1;
}

void NavMapBuilder2D::_build_step_edge_connection_margin_connections(NavMapIterationBuild2D &r_build) {
	PerformanceData &performance_data = r_build.performance_data;
	NavMapIteration2D *map_iteration = r_build.map_iteration;

	real_t edge_connection_margin = r_build.edge_connection_margin;
	const bool use_edge_connection_bridges = r_build.use_edge_connection_bridges;

	LocalVector<Connection> &free_edges = r_build.iter_free_edges;
	HashMap<const NavBaseIteration2D *, LocalVector<Connection>> &region_external_connections = map_iteration->external_region_connections;

	HashMap<const NavBaseIteration2D *, LocalVector<LocalVector<Nav2D::Connection>>> &navbases_polygons_external_connections = map_iteration->navbases_polygons_external_connections;

	// Find the compatible near edges.
	//
	// Note:
	// Without bridge polygons the pathway is placed halfway across the gap, where it is
	// an edge of neither polygon. That gives the funnel one constraint where the gap has
	// two, so a path can leave a region outside the stitched span.
	performance_data.pm_edge_free_count = free_edges.size();

	const real_t edge_connection_margin_squared = edge_connection_margin * edge_connection_margin;

	LocalVector<EdgeBridgeDescriptor> &edge_bridges = r_build.iter_edge_bridges;

	for (uint32_t i = 0; i < free_edges.size(); i++) {
		const Connection &free_edge = free_edges[i];
		const Vector2 &edge_p1 = free_edge.pathway_start;
		const Vector2 &edge_p2 = free_edge.pathway_end;

		// Bridges derive both directions from one canonical quad, so each unordered
		// pair is only visited once.
		const uint32_t first_other_edge = use_edge_connection_bridges ? i + 1 : 0;

		for (uint32_t j = first_other_edge; j < free_edges.size(); j++) {
			const Connection &other_edge = free_edges[j];
			if (i == j || free_edge.polygon->owner == other_edge.polygon->owner) {
				continue;
			}

			const Vector2 &other_edge_p1 = other_edge.pathway_start;
			const Vector2 &other_edge_p2 = other_edge.pathway_end;

			// Compute the projection of the opposite edge on the current one
			Vector2 edge_vector = edge_p2 - edge_p1;
			const real_t edge_length_squared = edge_vector.length_squared();
			if (use_edge_connection_bridges && edge_length_squared <= (real_t)CMP_EPSILON * (real_t)CMP_EPSILON) {
				continue;
			}
			real_t projected_p1_ratio = edge_vector.dot(other_edge_p1 - edge_p1) / edge_length_squared;
			real_t projected_p2_ratio = edge_vector.dot(other_edge_p2 - edge_p1) / edge_length_squared;
			if ((projected_p1_ratio < 0.0 && projected_p2_ratio < 0.0) || (projected_p1_ratio > 1.0 && projected_p2_ratio > 1.0)) {
				continue;
			}
			const real_t projected_ratio_span = projected_p2_ratio - projected_p1_ratio;
			const bool needs_interpolation = projected_p1_ratio < 0.0 || projected_p1_ratio > 1.0 || projected_p2_ratio < 0.0 || projected_p2_ratio > 1.0;
			if (use_edge_connection_bridges && needs_interpolation && Math::is_zero_approx(projected_ratio_span)) {
				continue;
			}

			// Check if the two edges are close to each other enough and compute a pathway between the two regions.
			Vector2 self1 = edge_vector * CLAMP(projected_p1_ratio, 0.0, 1.0) + edge_p1;
			Vector2 other1;
			if (projected_p1_ratio >= 0.0 && projected_p1_ratio <= 1.0) {
				other1 = other_edge_p1;
			} else {
				other1 = other_edge_p1.lerp(other_edge_p2, (1.0 - projected_p1_ratio) / projected_ratio_span);
			}
			if (other1.distance_squared_to(self1) > edge_connection_margin_squared) {
				continue;
			}

			Vector2 self2 = edge_vector * CLAMP(projected_p2_ratio, 0.0, 1.0) + edge_p1;
			Vector2 other2;
			if (projected_p2_ratio >= 0.0 && projected_p2_ratio <= 1.0) {
				other2 = other_edge_p2;
			} else {
				other2 = other_edge_p1.lerp(other_edge_p2, (0.0 - projected_p1_ratio) / projected_ratio_span);
			}
			if (other2.distance_squared_to(self2) > edge_connection_margin_squared) {
				continue;
			}

			if (!use_edge_connection_bridges) {
				// The edges can now be connected.
				Connection new_connection = other_edge;
				new_connection.pathway_start = (self1 + other1) / 2.0;
				new_connection.pathway_end = (self2 + other2) / 2.0;

				// Add the connection to the region_connection map.
				region_external_connections[free_edge.polygon->owner].push_back(new_connection);
				navbases_polygons_external_connections[free_edge.polygon->owner][free_edge.polygon->id].push_back(new_connection);
				performance_data.pm_edge_connection_count += 1;
				continue;
			}

			// Edges that already coincide leave no gap to bridge, so connect them directly
			// in both directions rather than building a degenerate polygon.
			if (self1.is_equal_approx(other1) && self2.is_equal_approx(other2)) {
				_add_direct_edge_connection(free_edge, other_edge, self1, self2, map_iteration, performance_data);
				_add_direct_edge_connection(other_edge, free_edge, other1, other2, map_iteration, performance_data);
				continue;
			}

			EdgeBridgeDescriptor descriptor;
			if (!_validate_edge_bridge_quad(self1, self2, other1, other2, descriptor.surface_area)) {
				continue;
			}
			descriptor.polygon_a = free_edge.polygon;
			descriptor.polygon_b = other_edge.polygon;
			descriptor.a1 = self1;
			descriptor.a2 = self2;
			descriptor.b1 = other1;
			descriptor.b2 = other2;
			edge_bridges.push_back(descriptor);
		}
	}

	if (use_edge_connection_bridges) {
		_build_edge_bridge_polygons(r_build);
	}
}

void NavMapBuilder2D::_build_edge_bridge_polygons(NavMapIterationBuild2D &r_build) {
	const LocalVector<EdgeBridgeDescriptor> &edge_bridges = r_build.iter_edge_bridges;
	if (edge_bridges.is_empty()) {
		return;
	}

	NavMapIteration2D *map_iteration = r_build.map_iteration;
	PerformanceData &performance_data = r_build.performance_data;

	HashMap<const NavBaseIteration2D *, LocalVector<LocalVector<Nav2D::Connection>>> &navbases_polygons_external_connections = map_iteration->navbases_polygons_external_connections;

	// Every gap yields one directed bridge per side, grouped under the region it departs
	// from. Counting first matters: Connection stores a raw Polygon pointer, so the
	// polygon storage has to reach its final size before any pointer into it is taken.
	HashMap<const NavBaseIteration2D *, uint32_t> bridge_counts;
	for (const EdgeBridgeDescriptor &descriptor : edge_bridges) {
		bridge_counts[descriptor.polygon_a->owner] += 1;
		bridge_counts[descriptor.polygon_b->owner] += 1;
	}

	HashMap<const NavBaseIteration2D *, Ref<NavEdgeBridgeIteration2D>> bridge_owners;
	for (const KeyValue<const NavBaseIteration2D *, uint32_t> &bridge_count : bridge_counts) {
		Ref<NavEdgeBridgeIteration2D> bridge_owner;
		bridge_owner.instantiate();
		bridge_owner->copy_semantics_from(*bridge_count.key);
		bridge_owner->navmesh_polygons.resize(bridge_count.value);
		bridge_owner->internal_connections.resize(bridge_count.value);

		// The query reads this map through a const reference, which crashes on a missing
		// key, so every owner that can be expanded needs an entry sized to its polygons.
		navbases_polygons_external_connections[bridge_owner.ptr()].resize(bridge_count.value);

		map_iteration->edge_bridge_iterations.push_back(bridge_owner);
		bridge_owners[bridge_count.key] = bridge_owner;
	}

	HashMap<const NavBaseIteration2D *, uint32_t> next_bridge_index;
	for (const EdgeBridgeDescriptor &descriptor : edge_bridges) {
		for (int direction = 0; direction < 2; direction++) {
			const bool reversed = direction == 1;

			Polygon *source_polygon = reversed ? descriptor.polygon_b : descriptor.polygon_a;
			Polygon *destination_polygon = reversed ? descriptor.polygon_a : descriptor.polygon_b;

			const Vector2 &source_pathway_start = reversed ? descriptor.b1 : descriptor.a1;
			const Vector2 &source_pathway_end = reversed ? descriptor.b2 : descriptor.a2;
			const Vector2 &destination_pathway_start = reversed ? descriptor.a1 : descriptor.b1;
			const Vector2 &destination_pathway_end = reversed ? descriptor.a2 : descriptor.b2;

			const NavBaseIteration2D *source_owner = source_polygon->owner;
			Ref<NavEdgeBridgeIteration2D> &bridge_owner = bridge_owners[source_owner];
			const uint32_t bridge_index = next_bridge_index[source_owner]++;

			Polygon &bridge_polygon = bridge_owner->navmesh_polygons[bridge_index];
			bridge_polygon.id = bridge_index;
			bridge_polygon.owner = bridge_owner.ptr();
			bridge_polygon.surface_area = descriptor.surface_area;
			bridge_polygon.vertices.resize(4);
			bridge_polygon.vertices[0] = descriptor.a1;
			bridge_polygon.vertices[1] = descriptor.a2;
			bridge_polygon.vertices[2] = descriptor.b2;
			bridge_polygon.vertices[3] = descriptor.b1;

			Connection entry_connection;
			entry_connection.polygon = &bridge_polygon;
			entry_connection.edge = -1;
			entry_connection.pathway_start = source_pathway_start;
			entry_connection.pathway_end = source_pathway_end;

			map_iteration->external_region_connections[source_owner].push_back(entry_connection);
			navbases_polygons_external_connections[source_owner][source_polygon->id].push_back(entry_connection);

			Connection exit_connection;
			exit_connection.polygon = destination_polygon;
			exit_connection.edge = -1;
			exit_connection.pathway_start = destination_pathway_start;
			exit_connection.pathway_end = destination_pathway_end;

			navbases_polygons_external_connections[bridge_owner.ptr()][bridge_index].push_back(exit_connection);

			performance_data.pm_edge_connection_count += 1;
		}
	}

	r_build.polygon_count += int(edge_bridges.size()) * 2;
}

void NavMapBuilder2D::_build_step_navlink_connections(NavMapIterationBuild2D &r_build) {
	NavMapIteration2D *map_iteration = r_build.map_iteration;

	real_t link_connection_radius = r_build.link_connection_radius;

	const LocalVector<Ref<NavLinkIteration2D>> &links = map_iteration->link_iterations;

	int polygon_count = r_build.polygon_count;

	real_t link_connection_radius_sqr = link_connection_radius * link_connection_radius;

	HashMap<const NavBaseIteration2D *, LocalVector<LocalVector<Nav2D::Connection>>> &navbases_polygons_external_connections = map_iteration->navbases_polygons_external_connections;
	LocalVector<Nav2D::Polygon> &navlink_polygons = map_iteration->navlink_polygons;
	navlink_polygons.clear();
	navlink_polygons.resize(links.size());
	uint32_t navlink_index = 0;

	// Search for polygons within range of a nav link.
	for (const Ref<NavLinkIteration2D> &link : links) {
		polygon_count++;
		Polygon &new_polygon = navlink_polygons[navlink_index++];

		new_polygon.id = 0;
		new_polygon.owner = link.ptr();

		const Vector2 link_start_pos = link->get_start_position();
		const Vector2 link_end_pos = link->get_end_position();

		Polygon *closest_start_polygon = nullptr;
		real_t closest_start_sqr_dist = link_connection_radius_sqr;
		Vector2 closest_start_point;

		Polygon *closest_end_polygon = nullptr;
		real_t closest_end_sqr_dist = link_connection_radius_sqr;
		Vector2 closest_end_point;

		for (const Ref<NavRegionIteration2D> &region : map_iteration->region_iterations) {
			Rect2 region_bounds = region->get_bounds().grow(link_connection_radius);
			if (!region_bounds.has_point(link_start_pos) && !region_bounds.has_point(link_end_pos)) {
				continue;
			}

			for (Polygon &polyon : region->navmesh_polygons) {
				for (uint32_t point_id = 2; point_id < polyon.vertices.size(); point_id += 1) {
					const Triangle2 triangle(polyon.vertices[0], polyon.vertices[point_id - 1], polyon.vertices[point_id]);

					{
						const Vector2 start_point = triangle.get_closest_point_to(link_start_pos);
						const real_t sqr_dist = start_point.distance_squared_to(link_start_pos);

						// Pick the polygon that is within our radius and is closer than anything we've seen yet.
						if (sqr_dist < closest_start_sqr_dist) {
							closest_start_sqr_dist = sqr_dist;
							closest_start_point = start_point;
							closest_start_polygon = &polyon;
						}
					}

					{
						const Vector2 end_point = triangle.get_closest_point_to(link_end_pos);
						const real_t sqr_dist = end_point.distance_squared_to(link_end_pos);

						// Pick the polygon that is within our radius and is closer than anything we've seen yet.
						if (sqr_dist < closest_end_sqr_dist) {
							closest_end_sqr_dist = sqr_dist;
							closest_end_point = end_point;
							closest_end_polygon = &polyon;
						}
					}
				}
			}
		}

		// If we have both a start and end point, then create a synthetic polygon to route through.
		if (closest_start_polygon && closest_end_polygon) {
			new_polygon.vertices.resize(4);

			// Build a set of vertices that create a thin polygon going from the start to the end point.
			new_polygon.vertices[0] = closest_start_point;
			new_polygon.vertices[1] = closest_start_point;
			new_polygon.vertices[2] = closest_end_point;
			new_polygon.vertices[3] = closest_end_point;

			// Setup connections to go forward in the link.
			{
				Connection entry_connection;
				entry_connection.polygon = &new_polygon;
				entry_connection.edge = -1;
				entry_connection.pathway_start = new_polygon.vertices[0];
				entry_connection.pathway_end = new_polygon.vertices[1];
				navbases_polygons_external_connections[closest_start_polygon->owner][closest_start_polygon->id].push_back(entry_connection);

				Connection exit_connection;
				exit_connection.polygon = closest_end_polygon;
				exit_connection.edge = -1;
				exit_connection.pathway_start = new_polygon.vertices[2];
				exit_connection.pathway_end = new_polygon.vertices[3];
				navbases_polygons_external_connections[link.ptr()].push_back(LocalVector<Nav2D::Connection>());
				navbases_polygons_external_connections[link.ptr()][new_polygon.id].push_back(exit_connection);
			}

			// If the link is bi-directional, create connections from the end to the start.
			if (link->is_bidirectional()) {
				Connection entry_connection;
				entry_connection.polygon = &new_polygon;
				entry_connection.edge = -1;
				entry_connection.pathway_start = new_polygon.vertices[2];
				entry_connection.pathway_end = new_polygon.vertices[3];
				navbases_polygons_external_connections[closest_end_polygon->owner][closest_end_polygon->id].push_back(entry_connection);

				Connection exit_connection;
				exit_connection.polygon = closest_start_polygon;
				exit_connection.edge = -1;
				exit_connection.pathway_start = new_polygon.vertices[0];
				exit_connection.pathway_end = new_polygon.vertices[1];
				navbases_polygons_external_connections[link.ptr()].push_back(LocalVector<Nav2D::Connection>());
				navbases_polygons_external_connections[link.ptr()][new_polygon.id].push_back(exit_connection);
			}
		}
	}

	r_build.polygon_count = polygon_count;
}

void NavMapBuilder2D::_build_update_map_iteration(NavMapIterationBuild2D &r_build) {
	NavMapIteration2D *map_iteration = r_build.map_iteration;

	map_iteration->navmesh_polygon_count = r_build.polygon_count;

	uint32_t navmesh_polygon_count = r_build.polygon_count;
	uint32_t total_polygon_count = navmesh_polygon_count;

	map_iteration->path_query_slots_mutex.lock();
	for (NavMeshQueries2D::PathQuerySlot &p_path_query_slot : map_iteration->path_query_slots) {
		p_path_query_slot.traversable_polys.clear();
		p_path_query_slot.traversable_polys.reserve(navmesh_polygon_count * 0.25);
		p_path_query_slot.path_corridor.clear();

		p_path_query_slot.path_corridor.resize(total_polygon_count);

		p_path_query_slot.poly_to_id.clear();
		p_path_query_slot.poly_to_id.reserve(total_polygon_count);

		int polygon_id = 0;
		for (Ref<NavRegionIteration2D> &region : map_iteration->region_iterations) {
			for (const Polygon &polygon : region->navmesh_polygons) {
				p_path_query_slot.poly_to_id[&polygon] = polygon_id;
				polygon_id++;
			}
		}

		for (const Ref<NavEdgeBridgeIteration2D> &edge_bridge : map_iteration->edge_bridge_iterations) {
			for (const Polygon &polygon : edge_bridge->navmesh_polygons) {
				p_path_query_slot.poly_to_id[&polygon] = polygon_id;
				polygon_id++;
			}
		}

		for (const Polygon &polygon : map_iteration->navlink_polygons) {
			p_path_query_slot.poly_to_id[&polygon] = polygon_id;
			polygon_id++;
		}

		DEV_ASSERT(p_path_query_slot.path_corridor.size() == p_path_query_slot.poly_to_id.size());
	}

	map_iteration->path_query_slots_mutex.unlock();
}
