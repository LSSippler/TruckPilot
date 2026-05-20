// crates/ui/src/types/map.ts
//
// Shared types for the RouteMap component + everything that publishes/subscribes
// map.* and route.* keys on the blackboard.

/** Top-down world coord (meters). ETS2 world Y axis is ignored for the map. */
export interface WorldPoint {
  x: number;
  z: number;
}

export type RoadType = "highway" | "road" | "prefab" | "bezier";

export interface MapNode {
  uid: string;
  position: WorldPoint;
  is_junction?: boolean;
}

export interface MapEdge {
  from_uid: string;
  to_uid: string;
  from: WorldPoint;
  to: WorldPoint;
  road_type?: RoadType;
}

export interface RouteWaypoint extends WorldPoint {
  uid: string;
}

/** Axis-aligned bbox in world coords. */
export interface WorldBBox {
  x1: number;
  z1: number;
  x2: number;
  z2: number;
}
