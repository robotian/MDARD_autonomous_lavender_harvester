-- Harvesting operation state, read and written by status_server (the fleet's task manager) and shown on the web
-- UI's /operation page. farm_harvesting_job stays the per-task log (progress_status 0 idle, 1 in progress,
-- 2 paused, 3 done, 4 failed, 5 released/cancelled); the state of each PLANT lives here.
CREATE TABLE IF NOT EXISTS public.harvest_state (
    object_id         integer PRIMARY KEY REFERENCES public.object_data(object_id) ON DELETE CASCADE,
    row_id            integer NOT NULL,
    status            text NOT NULL DEFAULT 'pending'
                      CHECK (status IN ('pending', 'assigned', 'harvested', 'failed')),
    lane_id           integer,                 -- lane the robot works from: row_id - 1 (north side) or row_id (south)
    pickup_node_id    integer,                 -- graph_node.id the robot was sent to
    assigned_robot_id integer REFERENCES public.robot_info(id) ON DELETE SET NULL,
    job_id            integer,                 -- farm_harvesting_job.id of the current / last attempt
    attempts          integer NOT NULL DEFAULT 0,
    last_error        text,
    assigned_at       timestamptz,
    harvested_at      timestamptz,
    updated_at        timestamptz NOT NULL DEFAULT now()
);
INSERT INTO public.harvest_state (object_id, row_id)
    SELECT object_id, row_id FROM public.object_data ON CONFLICT (object_id) DO NOTHING;
CREATE INDEX IF NOT EXISTS harvest_state_status_idx ON public.harvest_state (status, row_id);

-- the one operation (singleton row, id = 1)
CREATE TABLE IF NOT EXISTS public.operation (
    id                  integer PRIMARY KEY CHECK (id = 1),
    state               text NOT NULL DEFAULT 'stopped'
                        CHECK (state IN ('stopped', 'running', 'paused', 'complete')),
    coordinator_enabled boolean NOT NULL DEFAULT false,   -- the base station runs status_server while true
    target_rows         integer[],                        -- NULL = every row
    max_attempts        integer NOT NULL DEFAULT 2,       -- failed attempts before a plant is marked failed
    offline_grace_s     integer NOT NULL DEFAULT 180,     -- an offline robot's plant is released after this
    started_at          timestamptz,
    updated_at          timestamptz NOT NULL DEFAULT now(),
    note                text
);
INSERT INTO public.operation (id) VALUES (1) ON CONFLICT (id) DO NOTHING;

-- event log shown on the /operation page
CREATE TABLE IF NOT EXISTS public.operation_event (
    id        bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    ts        timestamptz NOT NULL DEFAULT now(),
    source    text NOT NULL,            -- task_manager | status_server | web_ui | supervisor
    robot     text,                     -- serial number; NULL for fleet-wide events
    kind      text NOT NULL,            -- start stop pause resume reset target assign charge harvested attempt_failed plant_failed preempt released offline online complete coordinator
    object_id integer,
    job_id    integer,
    message   text NOT NULL
);
CREATE INDEX IF NOT EXISTS operation_event_id_idx ON public.operation_event (id DESC);

-- wall-clock "last seen": robot_live.time follows the robot's ROS stamp, which is sim time in the simulator
ALTER TABLE public.robot_live ADD COLUMN IF NOT EXISTS updated_at timestamptz NOT NULL DEFAULT now();

COMMENT ON COLUMN public.farm_harvesting_job.progress_status IS '0 idle, 1 in progress, 2 paused, 3 done, 4 failed, 5 released/cancelled';
