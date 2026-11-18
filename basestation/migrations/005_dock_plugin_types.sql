-- dock_station.dock_type is the opennav docking *plugin* the robot undocks with (UndockRobot.dock_type, one of
-- nav2.yaml's docking_server.dock_plugins): 'simple_charging_dock' on every platform. The charger itself is dock_id.
-- 'wiferion_charging_dock' names no plugin: the undock after a charge at wiferion_charger failed with
-- "No dock information to undock from!" (2026-10-13).
UPDATE public.dock_station SET dock_type = 'simple_charging_dock' WHERE dock_type = 'wiferion_charging_dock';
