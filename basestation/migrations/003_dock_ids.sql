-- The name a charger's dock has in the robots' Nav2 docking server (dock_station.dock_id, DockGoal.dock_id).
-- The sim reads it from here (scripts/make_charger_scene.py -> state.json chargers[].dock_id) and names its docks
-- the same, so a charging task's dock_id works in the sim and on the real robots.
ALTER TABLE public.charging_stations ADD COLUMN IF NOT EXISTS dock_id text;
CREATE UNIQUE INDEX IF NOT EXISTS charging_stations_dock_id_key ON public.charging_stations (dock_id);
COMMENT ON COLUMN public.charging_stations.dock_id IS 'dock_station.dock_id of this charger: the dock name in the robots'' Nav2 docking server (sim: state.json chargers[].dock_id)';
-- by AprilTag: tag 1 is the Jackal's TR-302, tag 8 the Wiferion (a300 / a200); only fills empty values
UPDATE public.charging_stations SET dock_id = 'jackal_charger'   WHERE dock_id IS NULL AND apriltag_id = 1;
UPDATE public.charging_stations SET dock_id = 'wiferion_charger' WHERE dock_id IS NULL AND apriltag_id = 8;
-- dock_station 'jackal_charger' pointed at graph node 974, which doesn't exist: route to the Staging node 909
-- (both chargers are 6-7 m west of it; the docking server drives the rest with navigate_to_staging_pose)
UPDATE public.dock_station d SET node_id = 909
  WHERE NOT EXISTS (SELECT 1 FROM public.graph_node n WHERE n.id = d.node_id)
    AND EXISTS (SELECT 1 FROM public.graph_node n WHERE n.id = 909);
