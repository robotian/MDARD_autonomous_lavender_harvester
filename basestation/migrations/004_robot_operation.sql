-- Per-robot operator controls on the web UI's /operation page (Robots card), read by status_server every tick:
-- enrolled (Add / Remove: whether the coordinator assigns it anything), state (Pause / Resume / Stop / Start:
-- paused = finish the current task, then nothing new; stopped = the current task is cancelled, nothing new),
-- request (one-shot: 'charge' = send it to its charger now, 'stop' = cancel its task now; cleared once acted on).
CREATE TABLE IF NOT EXISTS public.robot_operation (
    robot_id   integer PRIMARY KEY REFERENCES public.robot_info(id) ON DELETE CASCADE,
    enrolled   boolean NOT NULL DEFAULT true,
    state      text NOT NULL DEFAULT 'active' CHECK (state IN ('active', 'paused', 'stopped')),
    request    text CHECK (request IN ('charge', 'stop')),
    updated_at timestamptz NOT NULL DEFAULT now()
);
INSERT INTO public.robot_operation (robot_id) SELECT id FROM public.robot_info ON CONFLICT (robot_id) DO NOTHING;
