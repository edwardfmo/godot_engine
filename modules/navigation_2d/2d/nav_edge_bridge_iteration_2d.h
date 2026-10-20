/**************************************************************************/
/*  nav_edge_bridge_iteration_2d.h                                        */
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

#pragma once

#include "nav_base_iteration_2d.h"

// Synthetic owner for the bridge polygons that span an edge connection margin gap.
//
// A bridge polygon cannot be owned by the source region directly: polygon ids are
// owner-local and are used to index `internal_connections` and
// `navbases_polygons_external_connections`, both of which are sized to the region's
// real navmesh polygons. An id past that end reads out of bounds, and recycling an
// existing id would make the bridge inherit that polygon's adjacency.
//
// So each source region gets a private proxy that carries the region's *semantics*
// while owning its own polygon and connection storage. Copying `owner_rid` matters
// beyond identity: `_query_task_build_path_corridor()` charges `enter_cost` only when
// `get_self()` differs between two consecutive polygon owners, so a matching RID keeps
// `source -> bridge` free and makes `bridge -> destination` charge exactly the source
// region's enter cost, reproducing the direct `source -> destination` cost.
class NavEdgeBridgeIteration2D : public NavBaseIteration2D {
	GDCLASS(NavEdgeBridgeIteration2D, NavBaseIteration2D);

public:
	// Adopt the traversal semantics of the region this bridge departs from.
	void copy_semantics_from(const NavBaseIteration2D &p_source_region) {
		enabled = p_source_region.enabled;
		navigation_layers = p_source_region.navigation_layers;
		enter_cost = p_source_region.enter_cost;
		travel_cost = p_source_region.travel_cost;
		owner_type = p_source_region.owner_type;
		owner_object_id = p_source_region.owner_object_id;
		owner_rid = p_source_region.owner_rid;
		owner_use_edge_connections = p_source_region.owner_use_edge_connections;
	}

	virtual ~NavEdgeBridgeIteration2D() override {
		navmesh_polygons.clear();
		internal_connections.clear();
	}
};
