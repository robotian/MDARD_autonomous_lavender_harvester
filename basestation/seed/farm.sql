--
-- PostgreSQL database dump
--

\restrict bgpgcU4auX3AiwgtRceasaj5UqknaPfKH1qrLX26KqkRaI2LZXt8N7YHoocuq9R

-- Dumped from database version 18.6 (Ubuntu 18.6-1.pgdg24.04+2)
-- Dumped by pg_dump version 18.6 (Ubuntu 18.6-1.pgdg24.04+2)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET transaction_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: xyz_rpw; Type: TYPE; Schema: public; Owner: -
--

CREATE TYPE public.xyz_rpw AS (
	x double precision,
	y double precision,
	z double precision,
	roll double precision,
	pitch double precision,
	yaw double precision
);


--
-- Name: move_robot_live_data(); Type: FUNCTION; Schema: public; Owner: -
--

CREATE FUNCTION public.move_robot_live_data() RETURNS trigger
    LANGUAGE plpgsql
    AS $$
BEGIN
    -- Insert the old row into table_2
    INSERT INTO public.robot_history (
        robot_live_id, time,ros_time, robot_type, online_flag, status, gps_location,
        task, topo_map_pos, current_node_id, target_node_id, battery_level,
        operation_hours_after_charging, load_status, crop_type, moved_at
    )
    VALUES (
        OLD.id, OLD.time, OLD.ros_time, OLD.robot_type, OLD.online_flag, OLD.status, OLD.gps_location,
        OLD.task, OLD.topo_map_pos, OLD.current_node_id, OLD.target_node_id, OLD.battery_level,
        OLD.operation_hours_after_charging, OLD.load_status, OLD.crop_type, now()
    );

    -- Allow the update to proceed
    RETURN NEW;
END;
$$;


--
-- Name: update_task_history(); Type: FUNCTION; Schema: public; Owner: -
--

CREATE FUNCTION public.update_task_history() RETURNS trigger
    LANGUAGE plpgsql
    AS $$
BEGIN
	INSERT INTO public.task_history (
		task_id,
		assigned_robot_id,
		target_node_id,
		description,
		crop_type,
		sub_tasks_id,
		sub_tasks_description,
		sub_tasks_data,
		status,
		completed_at
	)
	VALUES (
		NEW.task_id,
		NEW.assigned_robot_id,
		NEW.target_node_id,
		NEW.description,
		NEW.crop_type,
		NEW.sub_tasks_id,
		NEW.sub_tasks_description,
		NEW.sub_tasks_data,
		NEW.status,
		CURRENT_TIMESTAMP
	);
    RETURN NEW;
END;
$$;


SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: charging_stations; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.charging_stations (
    id integer NOT NULL,
    charger_model text,
    x_coord double precision,
    y_coord double precision,
    yaw_coord_deg double precision,
    apriltag_id integer,
    apriltag_sz_mm integer,
    frame text,
    dock_id text
);


--
-- Name: COLUMN charging_stations.charger_model; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.charger_model IS 'Model, e.g. TR-302 (WiBotic TR-302 Edge); sim asset per model in sim/scripts/add_chargers.py';


--
-- Name: COLUMN charging_stations.x_coord; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.x_coord IS 'Footprint centre of the charger on the ground, m, in frame';


--
-- Name: COLUMN charging_stations.y_coord; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.y_coord IS 'Footprint centre of the charger on the ground, m, in frame';


--
-- Name: COLUMN charging_stations.yaw_coord_deg; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.yaw_coord_deg IS 'Direction the charger''s front (AprilTag side) faces, degrees, CCW from the +X axis of frame';


--
-- Name: COLUMN charging_stations.apriltag_sz_mm; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.apriltag_sz_mm IS 'Edge of the tag''s black square, mm (apriltag_ros size)';


--
-- Name: COLUMN charging_stations.frame; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.frame IS 'Frame of x_coord / y_coord / yaw_coord_deg, e.g. map';


--
-- Name: COLUMN charging_stations.dock_id; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.charging_stations.dock_id IS 'dock_station.dock_id of this charger: the dock name in the robots'' Nav2 docking server (sim: state.json chargers[].dock_id)';


--
-- Name: dock_station; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.dock_station (
    id integer NOT NULL,
    dock_id text NOT NULL,
    type integer NOT NULL,
    status integer DEFAULT 0 NOT NULL,
    robot_id integer,
    node_id integer NOT NULL,
    dock_type text
);


--
-- Name: farm_asset_on_map; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.farm_asset_on_map (
    id integer NOT NULL,
    name character varying(255) NOT NULL,
    x_map double precision NOT NULL,
    y_map double precision NOT NULL,
    conn_nodes integer[]
);


--
-- Name: farm_edge; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.farm_edge (
    id text NOT NULL,
    zone_prefix text,
    from_node integer,
    to_node integer,
    distance real,
    weight integer
);


--
-- Name: farm_harvesting_job; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.farm_harvesting_job (
    id integer NOT NULL,
    node_id integer NOT NULL,
    job_schedule character varying(25) NOT NULL,
    crop_type character varying(50) NOT NULL,
    harvest_ready integer NOT NULL,
    progress_status integer NOT NULL,
    assigned_robot_id integer NOT NULL,
    job_location point NOT NULL,
    task_type integer DEFAULT 1 NOT NULL
);


--
-- Name: COLUMN farm_harvesting_job.progress_status; Type: COMMENT; Schema: public; Owner: -
--

COMMENT ON COLUMN public.farm_harvesting_job.progress_status IS '0 idle, 1 in progress, 2 paused, 3 done, 4 failed, 5 released/cancelled';


--
-- Name: farm_harvesting_job_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

ALTER TABLE public.farm_harvesting_job ALTER COLUMN id ADD GENERATED BY DEFAULT AS IDENTITY (
    SEQUENCE NAME public.farm_harvesting_job_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);


--
-- Name: farm_node; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.farm_node (
    id integer NOT NULL,
    zone_prefix character varying NOT NULL,
    x double precision NOT NULL,
    y double precision NOT NULL,
    type integer NOT NULL,
    theta double precision
);


--
-- Name: graph_edge; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.graph_edge (
    source_id integer,
    target_id integer,
    weight double precision,
    edge_type character varying(50)
);


--
-- Name: graph_node; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.graph_node (
    id integer NOT NULL,
    node_type character varying(50),
    x double precision,
    y double precision,
    theta double precision,
    obj_id integer
);


--
-- Name: harvest_state; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.harvest_state (
    object_id integer NOT NULL,
    row_id integer NOT NULL,
    status text DEFAULT 'pending'::text NOT NULL,
    lane_id integer,
    pickup_node_id integer,
    assigned_robot_id integer,
    job_id integer,
    attempts integer DEFAULT 0 NOT NULL,
    last_error text,
    assigned_at timestamp with time zone,
    harvested_at timestamp with time zone,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    CONSTRAINT harvest_state_status_check CHECK ((status = ANY (ARRAY['pending'::text, 'assigned'::text, 'harvested'::text, 'failed'::text])))
);


--
-- Name: object_data; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.object_data (
    object_id integer NOT NULL,
    row_id integer,
    x_coord double precision,
    y_coord double precision,
    orientation_deg double precision
);


--
-- Name: operation; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.operation (
    id integer NOT NULL,
    state text DEFAULT 'stopped'::text NOT NULL,
    coordinator_enabled boolean DEFAULT false NOT NULL,
    target_rows integer[],
    max_attempts integer DEFAULT 2 NOT NULL,
    offline_grace_s integer DEFAULT 180 NOT NULL,
    started_at timestamp with time zone,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    note text,
    CONSTRAINT operation_id_check CHECK ((id = 1)),
    CONSTRAINT operation_state_check CHECK ((state = ANY (ARRAY['stopped'::text, 'running'::text, 'paused'::text, 'complete'::text])))
);


--
-- Name: operation_event; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.operation_event (
    id bigint NOT NULL,
    ts timestamp with time zone DEFAULT now() NOT NULL,
    source text NOT NULL,
    robot text,
    kind text NOT NULL,
    object_id integer,
    job_id integer,
    message text NOT NULL
);


--
-- Name: operation_event_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

ALTER TABLE public.operation_event ALTER COLUMN id ADD GENERATED ALWAYS AS IDENTITY (
    SEQUENCE NAME public.operation_event_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);


--
-- Name: robot_history; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.robot_history (
    id integer NOT NULL,
    robot_live_id integer NOT NULL,
    "time" timestamp with time zone NOT NULL,
    ros_time text NOT NULL,
    robot_type text NOT NULL,
    online_flag text,
    status text,
    gps_location point,
    task text,
    topo_map_pos public.xyz_rpw,
    current_node_id integer,
    target_node_id integer,
    battery_level real,
    operation_hours_after_charging text,
    load_status real,
    crop_type text,
    moved_at timestamp with time zone DEFAULT now()
);


--
-- Name: robot_history_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

CREATE SEQUENCE public.robot_history_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


--
-- Name: robot_history_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: -
--

ALTER SEQUENCE public.robot_history_id_seq OWNED BY public.robot_history.id;


--
-- Name: robot_info; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.robot_info (
    id integer NOT NULL,
    robot_name text NOT NULL,
    robot_type text NOT NULL,
    robot_description text,
    manufacturer text,
    serial_number text NOT NULL
);


--
-- Name: robot_live; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.robot_live (
    id integer NOT NULL,
    "time" timestamp with time zone NOT NULL,
    ros_time text NOT NULL,
    robot_type text NOT NULL,
    online_flag integer,
    status text,
    gps_location point,
    task text,
    topo_map_pos public.xyz_rpw,
    current_node_id integer,
    target_node_id integer,
    battery_level real,
    operation_hours_after_charging text,
    load_status real,
    crop_type text,
    updated_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: robot_live_id_seq; Type: SEQUENCE; Schema: public; Owner: -
--

ALTER TABLE public.robot_live ALTER COLUMN id ADD GENERATED BY DEFAULT AS IDENTITY (
    SEQUENCE NAME public.robot_live_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1
);


--
-- Name: robot_operation; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.robot_operation (
    robot_id integer NOT NULL,
    enrolled boolean DEFAULT true NOT NULL,
    state text DEFAULT 'active'::text NOT NULL,
    request text,
    updated_at timestamp with time zone DEFAULT now() NOT NULL,
    CONSTRAINT robot_operation_request_check CHECK ((request = ANY (ARRAY['charge'::text, 'stop'::text]))),
    CONSTRAINT robot_operation_state_check CHECK ((state = ANY (ARRAY['active'::text, 'paused'::text, 'stopped'::text])))
);


--
-- Name: schema_migrations; Type: TABLE; Schema: public; Owner: -
--

CREATE TABLE public.schema_migrations (
    version text NOT NULL,
    applied_at timestamp with time zone DEFAULT now() NOT NULL
);


--
-- Name: robot_history id; Type: DEFAULT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_history ALTER COLUMN id SET DEFAULT nextval('public.robot_history_id_seq'::regclass);


--
-- Data for Name: charging_stations; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.charging_stations (id, charger_model, x_coord, y_coord, yaw_coord_deg, apriltag_id, apriltag_sz_mm, frame, dock_id) FROM stdin;
1	TR-302	-9.654	-1.972	0	1	80	map	jackal_charger
2	Wiferion	-9.654	0.5	0	8	160	map	wiferion_charger
3	Wiferion	-9.654	3.5	0	9	160	map	a200_charger
\.


--
-- Data for Name: dock_station; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.dock_station (id, dock_id, type, status, robot_id, node_id, dock_type) FROM stdin;
2	unloading_station	1	0	\N	909	simple_charging_dock
3	jackal_charger	0	0	\N	909	simple_charging_dock
1	wiferion_charger	0	0	\N	909	simple_charging_dock
4	a200_charger	0	0	\N	909	simple_charging_dock
\.


--
-- Data for Name: farm_asset_on_map; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.farm_asset_on_map (id, name, x_map, y_map, conn_nodes) FROM stdin;
1	b1_1	-10.32	4.83	{2,9}
2	b1_2	-11.32	4.83	{3,8}
3	b1_3	-12.2	4.83	{4,7}
\.


--
-- Data for Name: farm_edge; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.farm_edge (id, zone_prefix, from_node, to_node, distance, weight) FROM stdin;
1_2	b	1	2	1.34413	1
1_10	b	1	10	1.599	1
1_11	b	1	11	2.15128	1
1_12	b	1	12	2.15128	1
2_1	b	2	1	1.34413	1
2_3	b	2	3	0.99502	1
3_2	b	3	2	0.99502	1
3_4	b	3	4	0.87725	1
4_3	b	4	3	0.87725	1
4_5	b	4	5	1.342	1
5_4	b	5	4	1.342	1
5_6	b	5	6	1.491	1
5_11	b	5	11	5.98345	1
6_5	b	6	5	1.491	1
6_7	b	6	7	1.728	1
7_6	b	7	6	1.728	1
7_8	b	7	8	0.88147	1
8_7	b	8	7	0.88147	1
8_9	b	8	9	1.17668	1
9_8	b	9	8	1.17668	1
9_10	b	9	10	0.774	1
10_1	b	10	1	1.599	1
10_9	b	10	9	0.774	1
10_11	b	10	11	3.61689	1
10_12	b	10	12	3.61689	1
11_1	b	11	1	2.15128	1
11_5	b	11	5	5.98345	1
11_10	b	11	10	3.61689	1
12_1	b	12	1	2.15128	1
12_10	b	12	10	3.61689	1
\.


--
-- Data for Name: farm_harvesting_job; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.farm_harvesting_job (id, node_id, job_schedule, crop_type, harvest_ready, progress_status, assigned_robot_id, job_location, task_type) FROM stdin;
\.


--
-- Data for Name: farm_node; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.farm_node (id, zone_prefix, x, y, type, theta) FROM stdin;
1	b	-9.243	4.015	4	\N
2	b	-10.586	4.07	4	3.14
3	b	-11.581	4.076	4	3.14
4	b	-12.458	4.097	4	3.14
5	b	-13.8	4.097	4	\N
6	b	-13.8	5.588	4	\N
7	b	-12.072	5.588	4	0
8	b	-11.193	5.654	4	0
9	b	-10.017	5.614	4	0
10	b	-9.243	5.614	4	\N
11	charging	-8.135	2.171	5	0
12	unloading	-8.135	2.171	6	0
13	map	0	0	1	0
\.


--
-- Data for Name: graph_edge; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.graph_edge (source_id, target_id, weight, edge_type) FROM stdin;
1	2	1.332003378374094	Pickup Edge
2	3	1.332003378374094	Pickup Edge
3	4	1.3320033783740945	Pickup Edge
4	5	1.332003378374094	Pickup Edge
5	6	1.332003378374094	Pickup Edge
6	7	1.332003378374094	Pickup Edge
7	8	1.332003378374094	Pickup Edge
8	9	1.3320033783740954	Pickup Edge
9	10	1.332003378374094	Pickup Edge
10	11	1.332003378374094	Pickup Edge
11	12	1.332003378374094	Pickup Edge
12	13	1.3319999999999999	Pickup Edge
13	14	1.332003378374094	Pickup Edge
14	15	1.3350033707822622	Pickup Edge
15	16	1.332003378374094	Pickup Edge
16	17	1.332003378374094	Pickup Edge
17	18	1.3320033783740914	Pickup Edge
18	19	1.3320033783740968	Pickup Edge
19	20	1.3320033783740914	Pickup Edge
20	21	1.3320033783740968	Pickup Edge
21	22	1.3320033783740914	Pickup Edge
22	23	1.3320033783740968	Pickup Edge
23	24	1.3320033783740914	Pickup Edge
24	25	1.3320033783740968	Pickup Edge
25	26	1.3320033783740914	Pickup Edge
26	27	1.3320033783740968	Pickup Edge
27	28	1.3320033783740914	Pickup Edge
28	29	1.3320033783740968	Pickup Edge
29	30	1.3320033783740914	Pickup Edge
30	31	1.3350033707822622	Pickup Edge
31	32	1.3320033783740968	Pickup Edge
32	33	1.3320033783740914	Pickup Edge
33	34	1.3320033783740968	Pickup Edge
34	35	1.3320033783740914	Pickup Edge
35	36	1.332003378374102	Pickup Edge
36	37	1.3320033783740914	Pickup Edge
37	38	1.3320033783740914	Pickup Edge
38	40	4.799999999999994	Exiting Edge
39	1	4.8	Entering Edge
40	855	1.4678283785332529	Transit Edge
40	857	4.757808933891698	Transit Edge
40	856	1.4678283785332529	Transit Edge
40	858	4.757808933891698	Transit Edge
40	854	4.971534463728368	Transit Edge
40	860	10.467506137455349	Transit Edge
41	42	1.3320033783740914	Pickup Edge
42	43	1.3320033783740968	Pickup Edge
43	44	1.3320033783740968	Pickup Edge
44	45	1.3320033783740914	Pickup Edge
45	46	1.3320033783740968	Pickup Edge
46	47	1.3320033783740914	Pickup Edge
47	48	1.3320033783740968	Pickup Edge
48	49	1.3350033707822622	Pickup Edge
49	50	1.3320033783740914	Pickup Edge
50	51	1.3320033783740968	Pickup Edge
51	52	1.3320033783740914	Pickup Edge
52	53	1.3320033783740968	Pickup Edge
53	54	1.3320033783740914	Pickup Edge
54	55	1.3320033783740968	Pickup Edge
55	56	1.3320033783740914	Pickup Edge
56	57	1.3320033783740968	Pickup Edge
57	58	1.3320033783740914	Pickup Edge
58	59	1.3320033783740968	Pickup Edge
59	60	1.3320033783740914	Pickup Edge
60	61	1.3320033783740994	Pickup Edge
61	62	1.332003378374094	Pickup Edge
62	63	1.332003378374094	Pickup Edge
63	64	1.332003378374094	Pickup Edge
64	65	1.3350033707822622	Pickup Edge
65	66	1.332003378374094	Pickup Edge
66	67	1.3319999999999999	Pickup Edge
67	68	1.332003378374094	Pickup Edge
68	69	1.332003378374094	Pickup Edge
69	70	1.3320033783740928	Pickup Edge
70	71	1.332003378374094	Pickup Edge
71	72	1.332003378374094	Pickup Edge
72	73	1.332003378374094	Pickup Edge
73	74	1.332003378374094	Pickup Edge
74	75	1.332003378374094	Pickup Edge
75	76	1.3320033783740945	Pickup Edge
76	77	1.332003378374094	Pickup Edge
77	78	1.3320033783740943	Pickup Edge
78	80	4.8	Exiting Edge
79	41	4.799999999999994	Entering Edge
80	887	3.871925412787825	Transit Edge
80	885	3.9858411999710937	Transit Edge
80	888	3.871925412787825	Transit Edge
80	886	3.9858411999710937	Transit Edge
80	890	9.03746908002976	Transit Edge
80	884	9.184836328431688	Transit Edge
81	82	1.362029735358226	Pickup Edge
82	83	1.3650296700072126	Pickup Edge
83	84	1.3620297353582262	Pickup Edge
84	85	1.365029670007213	Pickup Edge
85	86	1.362029735358227	Pickup Edge
86	87	1.3650131867494908	Pickup Edge
87	88	1.3620297353582256	Pickup Edge
88	89	1.365029670007213	Pickup Edge
89	90	1.362029735358227	Pickup Edge
90	91	1.3620297353582256	Pickup Edge
91	92	1.365029670007213	Pickup Edge
92	93	1.3620297353582254	Pickup Edge
93	94	1.3650131867494908	Pickup Edge
94	95	1.3620297353582282	Pickup Edge
95	96	1.365029670007213	Pickup Edge
96	97	1.3620297353582256	Pickup Edge
97	98	1.365029670007213	Pickup Edge
98	99	1.3620297353582282	Pickup Edge
99	100	1.362029735358223	Pickup Edge
100	101	1.3650131867494908	Pickup Edge
101	102	1.3620297353582282	Pickup Edge
102	103	1.365029670007213	Pickup Edge
103	104	1.362029735358223	Pickup Edge
104	105	1.365029670007213	Pickup Edge
105	106	1.3620297353582282	Pickup Edge
106	107	1.3620132157949147	Pickup Edge
107	108	1.365029670007213	Pickup Edge
108	109	1.3620297353582227	Pickup Edge
109	110	1.365029670007213	Pickup Edge
110	111	1.3620297353582282	Pickup Edge
111	112	1.365029670007213	Pickup Edge
112	113	1.362029735358223	Pickup Edge
113	114	1.3650131867494908	Pickup Edge
114	115	1.3620297353582282	Pickup Edge
115	116	1.3620297353582282	Pickup Edge
116	117	1.3650296700072078	Pickup Edge
117	118	1.3620297353582282	Pickup Edge
118	120	4.799999999999996	Exiting Edge
119	81	4.8	Entering Edge
120	853	2.8891920517834766	Transit Edge
120	851	3.4654277624275167	Transit Edge
120	854	2.8891920517834766	Transit Edge
120	852	3.4654277624275167	Transit Edge
120	856	8.500872689734319	Transit Edge
120	850	9.124149992224732	Transit Edge
121	122	1.3620297353582282	Pickup Edge
122	123	1.3650296700072078	Pickup Edge
123	124	1.3620297353582282	Pickup Edge
124	125	1.3620297353582282	Pickup Edge
125	126	1.3650131867494908	Pickup Edge
126	127	1.362029735358223	Pickup Edge
127	128	1.365029670007213	Pickup Edge
128	129	1.3620297353582282	Pickup Edge
129	130	1.365029670007213	Pickup Edge
130	131	1.362029735358223	Pickup Edge
131	132	1.365029670007213	Pickup Edge
132	133	1.3620132157949147	Pickup Edge
133	134	1.362029735358228	Pickup Edge
134	135	1.365029670007213	Pickup Edge
135	136	1.362029735358223	Pickup Edge
136	137	1.365029670007213	Pickup Edge
137	138	1.3620297353582282	Pickup Edge
138	139	1.3650131867494908	Pickup Edge
139	140	1.362029735358223	Pickup Edge
140	141	1.3620297353582282	Pickup Edge
141	142	1.365029670007213	Pickup Edge
142	143	1.3620297353582256	Pickup Edge
143	144	1.3650296700072129	Pickup Edge
144	145	1.3620297353582282	Pickup Edge
145	146	1.3650131867494908	Pickup Edge
146	147	1.3620297353582256	Pickup Edge
147	148	1.365029670007213	Pickup Edge
148	149	1.3620297353582242	Pickup Edge
149	150	1.362029735358227	Pickup Edge
150	151	1.365029670007213	Pickup Edge
151	152	1.3620297353582256	Pickup Edge
152	153	1.3650131867494908	Pickup Edge
153	154	1.3620297353582262	Pickup Edge
154	155	1.365029670007213	Pickup Edge
155	156	1.3620297353582262	Pickup Edge
156	157	1.365029670007213	Pickup Edge
157	158	1.3620297353582262	Pickup Edge
158	160	4.8	Exiting Edge
159	121	4.799999999999996	Entering Edge
160	889	1.7757272680898935	Transit Edge
160	887	4.378151027798268	Transit Edge
160	890	1.7757272680898935	Transit Edge
160	888	4.378151027798268	Transit Edge
160	892	7.397119228359213	Transit Edge
160	886	10.137177622656152	Transit Edge
161	162	1.3710032822717821	Pickup Edge
162	163	1.371	Pickup Edge
163	164	1.3710032822717824	Pickup Edge
164	165	1.3710032822717817	Pickup Edge
165	166	1.3709999999999996	Pickup Edge
166	167	1.371003282271783	Pickup Edge
167	168	1.3709999999999996	Pickup Edge
168	169	1.3710032822717817	Pickup Edge
169	170	1.3710000000000009	Pickup Edge
170	171	1.3710032822717817	Pickup Edge
171	172	1.3710032822717844	Pickup Edge
172	173	1.3709999999999996	Pickup Edge
173	174	1.3710032822717817	Pickup Edge
174	175	1.3709999999999996	Pickup Edge
175	176	1.3710032822717817	Pickup Edge
176	177	1.3709999999999996	Pickup Edge
177	178	1.3710032822717844	Pickup Edge
178	179	1.371003282271779	Pickup Edge
179	180	1.3709999999999969	Pickup Edge
180	181	1.3710032822717844	Pickup Edge
181	182	1.3710000000000022	Pickup Edge
182	183	1.371003282271779	Pickup Edge
183	184	1.3680000000000039	Pickup Edge
184	185	1.371003282271779	Pickup Edge
185	186	1.3710032822717844	Pickup Edge
186	187	1.3709999999999969	Pickup Edge
187	188	1.3710032822717844	Pickup Edge
188	189	1.3709999999999969	Pickup Edge
189	190	1.3710032822717844	Pickup Edge
190	191	1.3710032822717844	Pickup Edge
191	192	1.3709999999999969	Pickup Edge
192	193	1.3710032822717844	Pickup Edge
193	194	1.3709999999999969	Pickup Edge
194	195	1.3710032822717844	Pickup Edge
195	196	1.3710000000000022	Pickup Edge
196	197	1.3710032822717844	Pickup Edge
197	198	1.3710032822717844	Pickup Edge
198	200	4.8	Exiting Edge
199	161	4.8	Entering Edge
200	849	2.2072352297838918	Transit Edge
200	851	4.01857507247103	Transit Edge
200	850	2.2072352297838918	Transit Edge
200	852	4.01857507247103	Transit Edge
200	848	7.821380552409492	Transit Edge
200	854	9.747059479969764	Transit Edge
201	202	1.3710032822717844	Pickup Edge
202	203	1.3710032822717737	Pickup Edge
203	204	1.3710000000000022	Pickup Edge
204	205	1.3710032822717844	Pickup Edge
205	206	1.3709999999999969	Pickup Edge
206	207	1.3710032822717844	Pickup Edge
207	208	1.3709999999999969	Pickup Edge
208	209	1.3710032822717844	Pickup Edge
209	210	1.3710032822717844	Pickup Edge
210	211	1.3709999999999969	Pickup Edge
211	212	1.3710032822717844	Pickup Edge
212	213	1.3709999999999969	Pickup Edge
213	214	1.3710032822717844	Pickup Edge
214	215	1.371003282271779	Pickup Edge
215	216	1.3680000000000039	Pickup Edge
216	217	1.371003282271779	Pickup Edge
217	218	1.3710000000000022	Pickup Edge
218	219	1.3710032822717844	Pickup Edge
219	220	1.3709999999999996	Pickup Edge
220	221	1.3710032822717817	Pickup Edge
221	222	1.3710032822717844	Pickup Edge
222	223	1.3709999999999996	Pickup Edge
223	224	1.3710032822717817	Pickup Edge
224	225	1.3709999999999996	Pickup Edge
225	226	1.3710032822717817	Pickup Edge
226	227	1.3709999999999996	Pickup Edge
227	228	1.3710032822717844	Pickup Edge
228	229	1.371003282271779	Pickup Edge
229	230	1.3710000000000009	Pickup Edge
230	231	1.3710032822717817	Pickup Edge
231	232	1.3709999999999996	Pickup Edge
232	233	1.371003282271783	Pickup Edge
233	234	1.3709999999999996	Pickup Edge
234	235	1.3710032822717824	Pickup Edge
235	236	1.3710032822717824	Pickup Edge
236	237	1.371	Pickup Edge
237	238	1.3710032822717821	Pickup Edge
238	240	4.8	Exiting Edge
239	201	4.8	Entering Edge
240	891	0.8788015216108912	Transit Edge
240	893	5.519541710991301	Transit Edge
240	892	0.8788015216108912	Transit Edge
240	894	5.519541710991301	Transit Edge
240	890	6.2175886136745255	Transit Edge
240	896	11.302103960053481	Transit Edge
241	242	1.3650032966993155	Pickup Edge
242	243	1.365003296699316	Pickup Edge
243	244	1.3680131578314587	Pickup Edge
244	245	1.365003296699316	Pickup Edge
245	246	1.3680032894697285	Pickup Edge
246	247	1.3650131867494908	Pickup Edge
247	248	1.365003296699316	Pickup Edge
248	249	1.3680032894697292	Pickup Edge
249	250	1.365003296699316	Pickup Edge
250	251	1.36801315783146	Pickup Edge
251	252	1.365003296699316	Pickup Edge
252	253	1.365003296699316	Pickup Edge
253	254	1.3680131578314574	Pickup Edge
254	255	1.365003296699316	Pickup Edge
255	256	1.3680032894697305	Pickup Edge
256	257	1.3650131867494908	Pickup Edge
257	258	1.365003296699316	Pickup Edge
258	259	1.3680032894697278	Pickup Edge
259	260	1.365003296699316	Pickup Edge
260	261	1.3680131578314574	Pickup Edge
261	262	1.365003296699316	Pickup Edge
262	263	1.365003296699316	Pickup Edge
263	264	1.3680131578314627	Pickup Edge
264	265	1.365003296699316	Pickup Edge
265	266	1.3680032894697278	Pickup Edge
266	267	1.3650131867494908	Pickup Edge
267	268	1.365003296699316	Pickup Edge
268	269	1.3680032894697278	Pickup Edge
269	270	1.365003296699316	Pickup Edge
270	271	1.3680131578314574	Pickup Edge
271	272	1.365003296699316	Pickup Edge
272	273	1.365003296699316	Pickup Edge
273	274	1.3680131578314574	Pickup Edge
274	275	1.365003296699316	Pickup Edge
275	276	1.3680032894697278	Pickup Edge
276	277	1.3650032966993213	Pickup Edge
277	278	1.3680131578314574	Pickup Edge
278	280	4.800000000000003	Exiting Edge
279	241	4.8	Entering Edge
280	847	1.1414027405876457	Transit Edge
280	849	5.268665999257445	Transit Edge
280	848	1.1414027405876457	Transit Edge
280	850	5.268665999257445	Transit Edge
280	846	6.513518531501017	Transit Edge
280	852	11.03603403613538	Transit Edge
281	282	1.3680131578314574	Pickup Edge
282	283	1.365003296699316	Pickup Edge
283	284	1.3680032894697332	Pickup Edge
284	285	1.365003296699316	Pickup Edge
285	286	1.3680131578314574	Pickup Edge
286	287	1.365003296699316	Pickup Edge
287	288	1.365003296699316	Pickup Edge
288	289	1.3680131578314574	Pickup Edge
289	290	1.365003296699316	Pickup Edge
290	291	1.3680032894697278	Pickup Edge
291	292	1.365003296699316	Pickup Edge
292	293	1.3650131867494908	Pickup Edge
293	294	1.3680032894697278	Pickup Edge
294	295	1.365003296699316	Pickup Edge
295	296	1.3680131578314627	Pickup Edge
296	297	1.365003296699316	Pickup Edge
297	298	1.365003296699316	Pickup Edge
298	299	1.3680131578314574	Pickup Edge
299	300	1.365003296699316	Pickup Edge
300	301	1.3680032894697278	Pickup Edge
301	302	1.365003296699316	Pickup Edge
302	303	1.3650131867494908	Pickup Edge
303	304	1.3680032894697278	Pickup Edge
304	305	1.365003296699316	Pickup Edge
305	306	1.3680131578314574	Pickup Edge
306	307	1.365003296699316	Pickup Edge
307	308	1.365003296699316	Pickup Edge
308	309	1.3680131578314614	Pickup Edge
309	310	1.3650032966993146	Pickup Edge
310	311	1.3680032894697292	Pickup Edge
311	312	1.365003296699316	Pickup Edge
312	313	1.3650131867494908	Pickup Edge
313	314	1.3680032894697285	Pickup Edge
314	315	1.365003296699316	Pickup Edge
315	316	1.3680131578314592	Pickup Edge
316	317	1.3650032966993155	Pickup Edge
317	318	1.3650032966993155	Pickup Edge
318	320	4.8	Exiting Edge
319	281	4.800000000000003	Entering Edge
320	893	1.7365599905791067	Transit Edge
320	895	4.616103131695604	Transit Edge
320	894	1.7365599905791067	Transit Edge
320	896	4.616103131695604	Transit Edge
320	892	7.232014775897993	Transit Edge
320	898	10.352793839515808	Transit Edge
321	322	1.3650131867494906	Pickup Edge
322	323	1.3680296049428173	Pickup Edge
323	324	1.3680131578314587	Pickup Edge
324	325	1.3680131578314587	Pickup Edge
325	326	1.368029604942817	Pickup Edge
326	327	1.3680131578314587	Pickup Edge
327	328	1.3680131578314587	Pickup Edge
328	329	1.3650131867494908	Pickup Edge
329	330	1.368029604942817	Pickup Edge
330	331	1.36801315783146	Pickup Edge
331	332	1.36801315783146	Pickup Edge
332	333	1.3680296049428156	Pickup Edge
333	334	1.36801315783146	Pickup Edge
334	335	1.3680131578314574	Pickup Edge
335	336	1.3650131867494908	Pickup Edge
336	337	1.3680296049428182	Pickup Edge
337	338	1.3680131578314574	Pickup Edge
338	339	1.3680131578314574	Pickup Edge
339	340	1.3680131578314574	Pickup Edge
340	341	1.3680296049428156	Pickup Edge
341	342	1.3680131578314627	Pickup Edge
342	343	1.3650131867494908	Pickup Edge
343	344	1.3680296049428156	Pickup Edge
344	345	1.3680131578314574	Pickup Edge
345	346	1.3680131578314627	Pickup Edge
346	347	1.3680131578314574	Pickup Edge
347	348	1.3680296049428156	Pickup Edge
348	349	1.3680131578314574	Pickup Edge
349	350	1.3650131867494908	Pickup Edge
350	351	1.3680296049428209	Pickup Edge
351	352	1.3680131578314574	Pickup Edge
352	353	1.3680131578314574	Pickup Edge
353	354	1.3680131578314574	Pickup Edge
354	355	1.3680296049428209	Pickup Edge
355	356	1.3650131867494908	Pickup Edge
356	357	1.368013157831452	Pickup Edge
357	358	1.368013157831468	Pickup Edge
358	360	4.799999999999994	Exiting Edge
359	321	4.8	Entering Edge
360	845	1.551722138590333	Transit Edge
360	843	4.95102965476671	Transit Edge
360	846	1.551722138590333	Transit Edge
360	844	4.95102965476671	Transit Edge
360	848	6.919693142611166	Transit Edge
360	842	10.686286180374546	Transit Edge
361	362	1.368013157831468	Pickup Edge
362	363	1.368013157831452	Pickup Edge
363	364	1.3650131867494908	Pickup Edge
364	365	1.3680296049428209	Pickup Edge
365	366	1.3680131578314574	Pickup Edge
366	367	1.3680131578314574	Pickup Edge
367	368	1.3680131578314574	Pickup Edge
368	369	1.3680296049428209	Pickup Edge
369	370	1.3650131867494908	Pickup Edge
370	371	1.3680131578314574	Pickup Edge
371	372	1.3680296049428156	Pickup Edge
372	373	1.3680131578314574	Pickup Edge
373	374	1.3680131578314627	Pickup Edge
374	375	1.3680131578314574	Pickup Edge
375	376	1.3680296049428156	Pickup Edge
376	377	1.3650131867494908	Pickup Edge
377	378	1.3680131578314627	Pickup Edge
378	379	1.3680296049428156	Pickup Edge
379	380	1.3680131578314574	Pickup Edge
380	381	1.36801315783146	Pickup Edge
381	382	1.3680131578314574	Pickup Edge
382	383	1.3680296049428182	Pickup Edge
383	384	1.3650131867494908	Pickup Edge
384	385	1.3680131578314574	Pickup Edge
385	386	1.36801315783146	Pickup Edge
386	387	1.3680296049428156	Pickup Edge
387	388	1.36801315783146	Pickup Edge
388	389	1.36801315783146	Pickup Edge
389	390	1.368029604942817	Pickup Edge
390	391	1.3650131867494908	Pickup Edge
391	392	1.3680131578314587	Pickup Edge
392	393	1.3680131578314587	Pickup Edge
393	394	1.368029604942817	Pickup Edge
394	395	1.3680131578314585	Pickup Edge
395	396	1.3680131578314592	Pickup Edge
396	397	1.3680296049428171	Pickup Edge
397	398	1.3650131867494908	Pickup Edge
398	400	4.8	Exiting Edge
399	361	4.799999999999994	Entering Edge
400	895	2.9114069317643567	Transit Edge
400	897	3.1179228490646547	Transit Edge
400	896	2.9114069317643567	Transit Edge
400	898	3.1179228490646547	Transit Edge
400	894	8.649294907183377	Transit Edge
400	900	8.86260458909287	Transit Edge
401	402	1.3620297353582262	Pickup Edge
402	403	1.3620297353582265	Pickup Edge
403	404	1.3620297353582262	Pickup Edge
404	405	1.3620528624102661	Pickup Edge
405	406	1.3620297353582276	Pickup Edge
406	407	1.3620297353582256	Pickup Edge
407	408	1.3620528624102668	Pickup Edge
408	409	1.3620297353582256	Pickup Edge
409	410	1.362029735358227	Pickup Edge
410	411	1.3620297353582256	Pickup Edge
411	412	1.3620528624102681	Pickup Edge
412	413	1.3620297353582256	Pickup Edge
413	414	1.3620297353582256	Pickup Edge
414	415	1.3620528624102655	Pickup Edge
415	416	1.3620297353582282	Pickup Edge
416	417	1.3620297353582256	Pickup Edge
417	418	1.3590298009977566	Pickup Edge
418	419	1.3620528624102655	Pickup Edge
419	420	1.3620297353582282	Pickup Edge
420	421	1.362029735358223	Pickup Edge
421	422	1.3620528624102684	Pickup Edge
422	423	1.362029735358228	Pickup Edge
423	424	1.362029735358223	Pickup Edge
424	425	1.3620297353582282	Pickup Edge
425	426	1.3620528624102684	Pickup Edge
426	427	1.3620297353582227	Pickup Edge
427	428	1.3620297353582282	Pickup Edge
428	429	1.3620297353582282	Pickup Edge
429	430	1.362052862410263	Pickup Edge
430	431	1.3620297353582282	Pickup Edge
431	432	1.3620297353582227	Pickup Edge
432	433	1.3620528624102684	Pickup Edge
433	434	1.3620297353582282	Pickup Edge
434	435	1.362029735358223	Pickup Edge
435	436	1.3620297353582282	Pickup Edge
436	437	1.3620528624102575	Pickup Edge
437	438	1.3620297353582282	Pickup Edge
438	440	4.800000000000003	Exiting Edge
439	401	4.8	Entering Edge
440	843	2.8011054836169786	Transit Edge
440	841	3.4526125119793867	Transit Edge
440	844	2.8011054836169786	Transit Edge
440	842	3.4526125119793867	Transit Edge
440	846	8.44695302119754	Transit Edge
440	840	9.14188377878983	Transit Edge
441	442	1.3620297353582282	Pickup Edge
442	443	1.3620528624102575	Pickup Edge
443	444	1.3620297353582282	Pickup Edge
444	445	1.362029735358223	Pickup Edge
445	446	1.3620297353582282	Pickup Edge
446	447	1.3620528624102684	Pickup Edge
447	448	1.3620297353582227	Pickup Edge
448	449	1.3620297353582282	Pickup Edge
449	450	1.362052862410263	Pickup Edge
450	451	1.3620297353582282	Pickup Edge
451	452	1.3620297353582282	Pickup Edge
452	453	1.3620297353582227	Pickup Edge
453	454	1.3620528624102684	Pickup Edge
454	455	1.3620297353582282	Pickup Edge
455	456	1.362029735358223	Pickup Edge
456	457	1.362029735358228	Pickup Edge
457	458	1.3620528624102684	Pickup Edge
458	459	1.362029735358223	Pickup Edge
459	460	1.3620297353582282	Pickup Edge
460	461	1.3620528624102655	Pickup Edge
461	462	1.3590298009977566	Pickup Edge
462	463	1.3620297353582256	Pickup Edge
463	464	1.3620297353582282	Pickup Edge
464	465	1.3620528624102655	Pickup Edge
465	466	1.3620297353582256	Pickup Edge
466	467	1.3620297353582256	Pickup Edge
467	468	1.3620528624102681	Pickup Edge
468	469	1.362029735358227	Pickup Edge
469	470	1.362029735358227	Pickup Edge
470	471	1.3620297353582256	Pickup Edge
471	472	1.3620528624102668	Pickup Edge
472	473	1.3620297353582256	Pickup Edge
473	474	1.362029735358227	Pickup Edge
474	475	1.3620528624102661	Pickup Edge
475	476	1.3620297353582262	Pickup Edge
476	477	1.3620297353582262	Pickup Edge
477	478	1.3620297353582262	Pickup Edge
478	480	4.8	Exiting Edge
479	441	4.800000000000003	Entering Edge
480	899	1.9675376158876	Transit Edge
480	897	4.319929890513363	Transit Edge
480	900	1.9675376158876	Transit Edge
480	898	4.319929890513363	Transit Edge
480	902	7.527269021392232	Transit Edge
480	896	10.051445114383581	Transit Edge
481	482	1.365003296699316	Pickup Edge
482	483	1.3650032966993155	Pickup Edge
483	484	1.3650032966993162	Pickup Edge
484	485	1.3650032966993153	Pickup Edge
485	486	1.3650032966993146	Pickup Edge
486	487	1.365003296699316	Pickup Edge
487	488	1.365003296699316	Pickup Edge
488	489	1.3650131867494908	Pickup Edge
489	490	1.365003296699316	Pickup Edge
490	491	1.365003296699316	Pickup Edge
491	492	1.365003296699316	Pickup Edge
492	493	1.365003296699316	Pickup Edge
493	494	1.365003296699316	Pickup Edge
494	495	1.365003296699316	Pickup Edge
495	496	1.3650032966993133	Pickup Edge
496	497	1.3650131867494908	Pickup Edge
497	498	1.365003296699316	Pickup Edge
498	499	1.365003296699316	Pickup Edge
499	500	1.365003296699316	Pickup Edge
500	501	1.365003296699316	Pickup Edge
501	502	1.365003296699316	Pickup Edge
502	503	1.365003296699316	Pickup Edge
503	504	1.365003296699316	Pickup Edge
504	505	1.365003296699316	Pickup Edge
505	506	1.3650131867494908	Pickup Edge
506	507	1.365003296699316	Pickup Edge
507	508	1.365003296699316	Pickup Edge
508	509	1.365003296699316	Pickup Edge
509	510	1.365003296699316	Pickup Edge
510	511	1.365003296699316	Pickup Edge
511	512	1.365003296699316	Pickup Edge
512	513	1.365003296699316	Pickup Edge
513	514	1.3650131867494855	Pickup Edge
514	515	1.365003296699316	Pickup Edge
515	516	1.365003296699316	Pickup Edge
516	517	1.3650032966993266	Pickup Edge
517	518	1.3650032966993106	Pickup Edge
518	520	4.800000000000004	Exiting Edge
519	481	4.8	Entering Edge
520	839	2.0313167165789823	Transit Edge
520	841	4.3089743069308994	Transit Edge
520	840	2.0313167165789823	Transit Edge
520	842	4.3089743069308994	Transit Edge
520	838	7.567324511861005	Transit Edge
520	844	10.029328800673785	Transit Edge
521	522	1.3650032966993106	Pickup Edge
522	523	1.365003296699316	Pickup Edge
523	524	1.365003296699316	Pickup Edge
524	525	1.365003296699316	Pickup Edge
525	526	1.3650131867494855	Pickup Edge
526	527	1.365003296699316	Pickup Edge
527	528	1.365003296699316	Pickup Edge
528	529	1.365003296699316	Pickup Edge
529	530	1.365003296699316	Pickup Edge
530	531	1.365003296699316	Pickup Edge
531	532	1.365003296699316	Pickup Edge
532	533	1.365003296699316	Pickup Edge
533	534	1.3650131867494908	Pickup Edge
534	535	1.365003296699316	Pickup Edge
535	536	1.365003296699316	Pickup Edge
536	537	1.365003296699316	Pickup Edge
537	538	1.365003296699316	Pickup Edge
538	539	1.365003296699316	Pickup Edge
539	540	1.365003296699316	Pickup Edge
540	541	1.365003296699316	Pickup Edge
541	542	1.365003296699316	Pickup Edge
542	543	1.3650131867494908	Pickup Edge
543	544	1.3650032966993133	Pickup Edge
544	545	1.365003296699316	Pickup Edge
545	546	1.365003296699316	Pickup Edge
546	547	1.365003296699316	Pickup Edge
547	548	1.365003296699316	Pickup Edge
548	549	1.365003296699316	Pickup Edge
549	550	1.365003296699316	Pickup Edge
550	551	1.3650131867494908	Pickup Edge
551	552	1.365003296699316	Pickup Edge
552	553	1.365003296699316	Pickup Edge
553	554	1.365003296699316	Pickup Edge
554	555	1.3650032966993146	Pickup Edge
555	556	1.3650032966993162	Pickup Edge
556	557	1.3650032966993157	Pickup Edge
557	558	1.3650032966993157	Pickup Edge
558	560	4.8	Exiting Edge
559	521	4.800000000000004	Entering Edge
560	901	1.0178793397225354	Transit Edge
560	903	5.565318825895279	Transit Edge
560	902	1.0178793397225354	Transit Edge
560	904	5.565318825895279	Transit Edge
560	900	6.219207964532156	Transit Edge
560	906	11.335282685358644	Transit Edge
561	562	1.3560530963055981	Pickup Edge
562	563	1.3560530963055983	Pickup Edge
563	564	1.3560530963055981	Pickup Edge
564	565	1.3560530963055988	Pickup Edge
565	566	1.3560530963055966	Pickup Edge
566	567	1.3560829620638994	Pickup Edge
567	568	1.3560530963055994	Pickup Edge
568	569	1.3530532140311395	Pickup Edge
569	570	1.3560530963056006	Pickup Edge
570	571	1.3560530963055981	Pickup Edge
571	572	1.3560829620638994	Pickup Edge
572	573	1.3560530963055981	Pickup Edge
573	574	1.356053096305598	Pickup Edge
574	575	1.3560530963055981	Pickup Edge
575	576	1.3560530963055981	Pickup Edge
576	577	1.356053096305598	Pickup Edge
577	578	1.3560829620638997	Pickup Edge
578	579	1.3530532140311395	Pickup Edge
579	580	1.3560530963055981	Pickup Edge
580	581	1.356053096305598	Pickup Edge
581	582	1.3560530963055981	Pickup Edge
582	583	1.3560530963055981	Pickup Edge
583	584	1.3560829620638994	Pickup Edge
584	585	1.3560530963055981	Pickup Edge
585	586	1.356053096305598	Pickup Edge
586	587	1.3560530963055981	Pickup Edge
587	588	1.3560530963055981	Pickup Edge
588	589	1.3530831460039712	Pickup Edge
589	590	1.356053096305598	Pickup Edge
590	591	1.3560530963055981	Pickup Edge
591	592	1.3560530963055981	Pickup Edge
592	593	1.356053096305598	Pickup Edge
593	594	1.3560530963055981	Pickup Edge
594	595	1.3560829620638994	Pickup Edge
595	596	1.3560530963055981	Pickup Edge
596	597	1.356053096305598	Pickup Edge
597	598	1.3560530963055928	Pickup Edge
598	600	4.800000000000006	Exiting Edge
599	561	4.799999999999999	Entering Edge
600	837	1.375470925393729	Transit Edge
600	839	5.904049409390368	Transit Edge
600	838	1.375470925393729	Transit Edge
600	840	5.904049409390368	Transit Edge
600	836	6.041953801890647	Transit Edge
600	842	11.636272737863209	Transit Edge
601	602	1.3560530963055981	Pickup Edge
602	603	1.356053096305598	Pickup Edge
603	604	1.3560530963055981	Pickup Edge
604	605	1.3560829620638994	Pickup Edge
605	606	1.3560530963055981	Pickup Edge
606	607	1.356053096305598	Pickup Edge
607	608	1.3560530963055981	Pickup Edge
608	609	1.3560530963055981	Pickup Edge
609	610	1.356053096305598	Pickup Edge
610	611	1.3530831460039712	Pickup Edge
611	612	1.3560530963055981	Pickup Edge
612	613	1.3560530963055981	Pickup Edge
613	614	1.356053096305598	Pickup Edge
614	615	1.3560530963055981	Pickup Edge
615	616	1.3560829620638994	Pickup Edge
616	617	1.3560530963055981	Pickup Edge
617	618	1.3560530963055981	Pickup Edge
618	619	1.356053096305598	Pickup Edge
619	620	1.3560530963055955	Pickup Edge
620	621	1.3530532140311422	Pickup Edge
621	622	1.3560829620638997	Pickup Edge
622	623	1.356053096305598	Pickup Edge
623	624	1.3560530963055981	Pickup Edge
624	625	1.3560530963055981	Pickup Edge
625	626	1.356053096305598	Pickup Edge
626	627	1.3560530963055981	Pickup Edge
627	628	1.3560829620638994	Pickup Edge
628	629	1.3560530963055981	Pickup Edge
629	630	1.3560530963055992	Pickup Edge
630	631	1.3530532140311395	Pickup Edge
631	632	1.3560530963055994	Pickup Edge
632	633	1.3560829620639003	Pickup Edge
633	634	1.3560530963055972	Pickup Edge
634	635	1.3560530963055988	Pickup Edge
635	636	1.356053096305598	Pickup Edge
636	637	1.356053096305598	Pickup Edge
637	638	1.3560530963055981	Pickup Edge
638	640	4.8	Exiting Edge
639	601	4.800000000000006	Entering Edge
640	903	1.8122906386545672	Transit Edge
640	905	4.565294356899564	Transit Edge
640	904	1.8122906386545672	Transit Edge
640	906	4.565294356899564	Transit Edge
640	902	7.301099012632797	Transit Edge
640	908	10.294590444054556	Transit Edge
641	642	1.3620825966144638	Pickup Edge
642	643	1.3620825966144636	Pickup Edge
643	644	1.3620825966144636	Pickup Edge
644	645	1.3620825966144643	Pickup Edge
645	646	1.3620825966144623	Pickup Edge
646	647	1.3620528624102668	Pickup Edge
647	648	1.3620825966144643	Pickup Edge
648	649	1.3620825966144627	Pickup Edge
649	650	1.3620825966144643	Pickup Edge
650	651	1.3620825966144656	Pickup Edge
651	652	1.3620825966144627	Pickup Edge
652	653	1.362082596614463	Pickup Edge
653	654	1.3620528624102681	Pickup Edge
654	655	1.362082596614463	Pickup Edge
655	656	1.3620825966144627	Pickup Edge
656	657	1.362082596614463	Pickup Edge
657	658	1.3620825966144656	Pickup Edge
658	659	1.3620825966144654	Pickup Edge
659	660	1.3620825966144603	Pickup Edge
660	661	1.3620528624102681	Pickup Edge
661	662	1.3620825966144603	Pickup Edge
662	663	1.3620825966144656	Pickup Edge
663	664	1.3620825966144654	Pickup Edge
664	665	1.3620825966144603	Pickup Edge
665	666	1.3620825966144654	Pickup Edge
666	667	1.3620825966144656	Pickup Edge
667	668	1.3620825966144603	Pickup Edge
668	669	1.3620528624102681	Pickup Edge
669	670	1.359082778935851	Pickup Edge
670	671	1.3620825966144654	Pickup Edge
671	672	1.3620825966144656	Pickup Edge
672	673	1.3620825966144603	Pickup Edge
673	674	1.3620825966144654	Pickup Edge
674	675	1.3620825966144656	Pickup Edge
675	676	1.3620528624102628	Pickup Edge
676	677	1.362082596614455	Pickup Edge
677	678	1.3620825966144654	Pickup Edge
678	680	4.800000000000002	Exiting Edge
679	641	4.8	Entering Edge
680	835	1.8761688215499974	Transit Edge
680	833	4.4990260553284305	Transit Edge
680	836	1.8761688215499974	Transit Edge
680	834	4.4990260553284305	Transit Edge
680	838	7.374140498179104	Transit Edge
680	832	10.224559945816178	Transit Edge
681	682	1.3620825966144656	Pickup Edge
682	683	1.362082596614455	Pickup Edge
683	684	1.362052862410263	Pickup Edge
684	685	1.3620825966144656	Pickup Edge
685	686	1.3620825966144656	Pickup Edge
686	687	1.3620825966144603	Pickup Edge
687	688	1.3620825966144656	Pickup Edge
688	689	1.3620825966144656	Pickup Edge
689	690	1.359082778935851	Pickup Edge
690	691	1.3620528624102684	Pickup Edge
691	692	1.3620825966144603	Pickup Edge
692	693	1.3620825966144656	Pickup Edge
693	694	1.3620825966144656	Pickup Edge
694	695	1.3620825966144603	Pickup Edge
695	696	1.3620825966144656	Pickup Edge
696	697	1.3620825966144656	Pickup Edge
697	698	1.3620825966144603	Pickup Edge
698	699	1.3620528624102684	Pickup Edge
699	700	1.3620825966144603	Pickup Edge
700	701	1.3620825966144656	Pickup Edge
701	702	1.3620825966144656	Pickup Edge
702	703	1.362082596614463	Pickup Edge
703	704	1.362082596614463	Pickup Edge
704	705	1.362082596614463	Pickup Edge
705	706	1.3620528624102684	Pickup Edge
706	707	1.362082596614463	Pickup Edge
707	708	1.362082596614463	Pickup Edge
708	709	1.3620825966144643	Pickup Edge
709	710	1.3620825966144643	Pickup Edge
710	711	1.362082596614463	Pickup Edge
711	712	1.3620825966144643	Pickup Edge
712	713	1.3620528624102668	Pickup Edge
713	714	1.362082596614463	Pickup Edge
714	715	1.3620825966144643	Pickup Edge
715	716	1.3620825966144636	Pickup Edge
716	717	1.362082596614464	Pickup Edge
717	718	1.3620825966144636	Pickup Edge
718	720	4.8	Exiting Edge
719	681	4.800000000000002	Entering Edge
720	905	3.0034471899519826	Transit Edge
720	907	3.1746643929032308	Transit Edge
720	906	3.0034471899519826	Transit Edge
720	908	3.1746643929032308	Transit Edge
720	904	8.691493581519223	Transit Edge
720	802	8.872173219217565	Transit Edge
721	722	1.3680296049428173	Pickup Edge
722	723	1.368029604942817	Pickup Edge
723	724	1.368029604942817	Pickup Edge
724	725	1.368029604942817	Pickup Edge
725	726	1.3680296049428176	Pickup Edge
726	727	1.368029604942817	Pickup Edge
727	728	1.368029604942817	Pickup Edge
728	729	1.3710295401631587	Pickup Edge
729	730	1.368029604942817	Pickup Edge
730	731	1.3680296049428156	Pickup Edge
731	732	1.3680296049428182	Pickup Edge
732	733	1.3680296049428156	Pickup Edge
733	734	1.3680296049428182	Pickup Edge
734	735	1.3680296049428182	Pickup Edge
735	736	1.3680296049428156	Pickup Edge
736	737	1.3680296049428182	Pickup Edge
737	738	1.3710295401631574	Pickup Edge
738	739	1.3680296049428156	Pickup Edge
739	740	1.3680296049428156	Pickup Edge
740	741	1.3680296049428153	Pickup Edge
741	742	1.3680296049428209	Pickup Edge
742	743	1.3680296049428156	Pickup Edge
743	744	1.3680296049428156	Pickup Edge
744	745	1.3680296049428156	Pickup Edge
745	746	1.37102954016316	Pickup Edge
746	747	1.3680296049428156	Pickup Edge
747	748	1.3680296049428156	Pickup Edge
748	749	1.3680296049428209	Pickup Edge
749	750	1.3680296049428156	Pickup Edge
750	751	1.3680296049428156	Pickup Edge
751	752	1.3680296049428209	Pickup Edge
752	753	1.3680296049428156	Pickup Edge
753	754	1.3680296049428156	Pickup Edge
754	755	1.37102954016316	Pickup Edge
755	756	1.3680296049428209	Pickup Edge
756	757	1.3680296049428156	Pickup Edge
757	758	1.3680296049428156	Pickup Edge
758	760	4.800000000000006	Exiting Edge
759	721	4.8	Entering Edge
760	833	2.947281151857034	Transit Edge
760	831	3.081894242317058	Transit Edge
760	834	2.947281151857034	Transit Edge
760	832	3.081894242317058	Transit Edge
760	836	8.68642762560381	Transit Edge
760	830	8.825465579393475	Transit Edge
761	762	1.3680296049428156	Pickup Edge
762	763	1.3680296049428102	Pickup Edge
763	764	1.3680296049428156	Pickup Edge
764	765	1.37102954016316	Pickup Edge
765	766	1.3680296049428156	Pickup Edge
766	767	1.3680296049428156	Pickup Edge
767	768	1.3680296049428209	Pickup Edge
768	769	1.3680296049428156	Pickup Edge
769	770	1.3680296049428156	Pickup Edge
770	771	1.3680296049428209	Pickup Edge
771	772	1.3680296049428156	Pickup Edge
772	773	1.3680296049428156	Pickup Edge
773	774	1.37102954016316	Pickup Edge
774	775	1.3680296049428156	Pickup Edge
775	776	1.3680296049428156	Pickup Edge
776	777	1.3680296049428156	Pickup Edge
777	778	1.3680296049428209	Pickup Edge
778	779	1.3680296049428153	Pickup Edge
779	780	1.3680296049428156	Pickup Edge
780	781	1.3680296049428209	Pickup Edge
781	782	1.3710295401631547	Pickup Edge
782	783	1.3680296049428209	Pickup Edge
783	784	1.3680296049428156	Pickup Edge
784	785	1.3680296049428156	Pickup Edge
785	786	1.3680296049428209	Pickup Edge
786	787	1.3680296049428156	Pickup Edge
787	788	1.3680296049428156	Pickup Edge
788	789	1.368029604942817	Pickup Edge
789	790	1.368029604942817	Pickup Edge
790	791	1.3710295401631587	Pickup Edge
791	792	1.368029604942817	Pickup Edge
792	793	1.368029604942817	Pickup Edge
793	794	1.368029604942817	Pickup Edge
794	795	1.368029604942817	Pickup Edge
795	796	1.3680296049428173	Pickup Edge
796	797	1.368029604942817	Pickup Edge
797	798	1.368029604942817	Pickup Edge
798	800	4.800000000000001	Exiting Edge
799	761	4.800000000000006	Entering Edge
800	801	1.7281132059394764	Transit Edge
800	907	4.644707530660292	Transit Edge
800	802	1.7281132059394764	Transit Edge
800	908	4.644707530660292	Transit Edge
800	804	4.8152916971496635	Transit Edge
800	906	10.379759413285974	Transit Edge
801	802	0	Via Swap
801	803	1.441025168756317	Via Edge CCW
801	909	46.93468578458423	Staging
802	801	0	Via Swap
802	908	1.4532201811784304	Via Edge CW
802	909	46.93468578458423	Staging
802	759	9.707462539477165	Transit Edge
803	804	0	Via Swap
803	805	1.441025168756317	Via Edge CCW
804	803	0	Via Swap
804	802	1.441025168756317	Via Edge CW
805	806	0	Via Swap
805	807	1.4410251687563167	Via Edge CCW
806	805	0	Via Swap
806	804	1.441025168756317	Via Edge CW
807	808	0	Via Swap
807	809	1.4410251687563171	Via Edge CCW
808	807	0	Via Swap
808	806	1.4410251687563167	Via Edge CW
809	810	0	Via Swap
809	811	1.4410251687563171	Via Edge CCW
810	809	0	Via Swap
810	808	1.4410251687563171	Via Edge CW
811	812	0	Via Swap
811	813	1.4410251687563163	Via Edge CCW
812	811	0	Via Swap
812	810	1.4410251687563171	Via Edge CW
813	814	0	Via Swap
813	815	1.441025168756318	Via Edge CCW
814	813	0	Via Swap
814	812	1.4410251687563163	Via Edge CW
815	816	0	Via Swap
815	817	1.4410251687563154	Via Edge CCW
816	815	0	Via Swap
816	814	1.441025168756318	Via Edge CW
817	818	0	Via Swap
817	819	1.441025168756319	Via Edge CCW
818	817	0	Via Swap
818	816	1.4410251687563154	Via Edge CW
819	820	0	Via Swap
819	821	1.4410251687563154	Via Edge CCW
820	819	0	Via Swap
820	818	1.441025168756319	Via Edge CW
821	822	0	Via Swap
821	823	1.441025168756319	Via Edge CCW
822	821	0	Via Swap
822	820	1.4410251687563154	Via Edge CW
823	824	0	Via Swap
823	825	1.4410251687563154	Via Edge CCW
824	823	0	Via Swap
824	822	1.441025168756319	Via Edge CW
825	826	0	Via Swap
825	827	1.441025168756319	Via Edge CCW
826	825	0	Via Swap
826	824	1.4410251687563154	Via Edge CW
826	799	9.099583834382225	Transit Edge
827	828	0	Via Swap
827	829	1.4410251687563154	Via Edge CCW
827	799	3.3958268741060715	Transit Edge
828	827	0	Via Swap
828	826	1.441025168756319	Via Edge CW
828	719	8.49824198302834	Transit Edge
828	799	3.3958268741060715	Transit Edge
829	830	0	Via Swap
829	831	1.4532201811784304	Via Edge CCW
829	799	2.590476315758424	Transit Edge
830	829	0	Via Swap
830	828	1.4410251687563154	Via Edge CW
830	719	8.48602215264102	Transit Edge
830	799	2.590476315758424	Transit Edge
831	832	0	Via Swap
831	833	1.4532201811784304	Via Edge CCW
831	719	3.5964374475315206	Transit Edge
832	831	0	Via Swap
832	830	1.4532201811784304	Via Edge CW
832	639	10.003730264168865	Transit Edge
832	719	3.5964374475315206	Transit Edge
832	799	5.5856549036538	Transit Edge
833	834	0	Via Swap
833	835	1.4532201811784304	Via Edge CCW
833	639	4.809551813504618	Transit Edge
833	719	4.62982907809744	Transit Edge
834	833	0	Via Swap
834	832	1.4532201811784304	Via Edge CW
834	559	11.453176693605755	Transit Edge
834	639	4.809551813504618	Transit Edge
834	719	4.62982907809744	Transit Edge
835	836	0	Via Swap
835	837	1.4532201811784304	Via Edge CCW
835	559	5.98654567860199	Transit Edge
835	639	3.710542107943218	Transit Edge
836	835	0	Via Swap
836	834	1.4532201811784304	Via Edge CW
836	559	5.98654567860199	Transit Edge
836	639	3.710542107943218	Transit Edge
837	838	0	Via Swap
837	839	1.4532201811784304	Via Edge CCW
837	559	2.8427744526287864	Transit Edge
838	837	0	Via Swap
838	836	1.4532201811784304	Via Edge CW
838	479	7.446551411890082	Transit Edge
838	559	2.8427744526287864	Transit Edge
838	639	8.484316204759844	Transit Edge
839	840	0	Via Swap
839	841	1.4532201811784287	Via Edge CCW
839	479	3.0146409680505735	Transit Edge
840	839	0	Via Swap
840	838	1.4532201811784304	Via Edge CW
840	399	8.904688136925415	Transit Edge
840	479	3.0146409680505735	Transit Edge
840	559	6.921211501937091	Transit Edge
841	842	0	Via Swap
841	843	1.4532201811784304	Via Edge CCW
841	399	3.8652687356757762	Transit Edge
841	479	5.504921165836274	Transit Edge
842	841	0	Via Swap
842	840	1.4532201811784287	Via Edge CW
842	319	10.40020817798696	Transit Edge
842	399	3.8652687356757762	Transit Edge
842	479	5.504921165836274	Transit Edge
843	844	0	Via Swap
843	845	1.4532201811784304	Via Edge CCW
843	319	4.983594102852451	Transit Edge
843	399	4.262194108857729	Transit Edge
844	843	0	Via Swap
844	842	1.4532201811784304	Via Edge CW
844	239	11.680128451390976	Transit Edge
844	319	4.983594102852451	Transit Edge
844	399	4.262194108857729	Transit Edge
844	479	10.913268005386477	Transit Edge
845	846	0	Via Swap
845	847	1.4532201811784304	Via Edge CCW
845	239	6.1639083697739645	Transit Edge
845	319	3.014507398896419	Transit Edge
846	845	0	Via Swap
846	844	1.4532201811784304	Via Edge CW
846	239	6.1639083697739645	Transit Edge
846	319	3.014507398896419	Transit Edge
846	399	9.432468366428116	Transit Edge
847	848	0	Via Swap
847	849	1.4532201811784304	Via Edge CCW
847	239	2.67232048610243	Transit Edge
848	847	0	Via Swap
848	846	1.4532201811784304	Via Edge CW
848	159	7.460510148283065	Transit Edge
848	239	2.67232048610243	Transit Edge
848	319	7.80496407253188	Transit Edge
849	850	0	Via Swap
849	851	1.4532201811784304	Via Edge CCW
849	159	3.1185916332649075	Transit Edge
850	849	0	Via Swap
850	848	1.4532201811784304	Via Edge CW
850	79	8.906644808145543	Transit Edge
850	159	3.1185916332649075	Transit Edge
850	239	6.623291614402357	Transit Edge
851	852	0	Via Swap
851	853	1.4532201811784287	Via Edge CCW
851	79	3.8172790247975312	Transit Edge
851	159	5.600998171487098	Transit Edge
852	851	0	Via Swap
852	850	1.4532201811784304	Via Edge CW
852	79	3.8172790247975312	Transit Edge
852	159	5.600998171487098	Transit Edge
853	854	0	Via Swap
853	855	1.4532201811784295	Via Edge CCW
853	79	4.170621201613618	Transit Edge
854	853	0	Via Swap
854	852	1.4532201811784287	Via Edge CW
854	79	4.170621201613618	Transit Edge
854	159	10.981616998349116	Transit Edge
855	856	0	Via Swap
855	857	1.4410251687563154	Via Edge CCW
855	909	44.41437111792457	Staging
856	855	0	Via Swap
856	854	1.4532201811784295	Via Edge CW
856	909	44.41437111792457	Staging
856	79	9.369936387245934	Transit Edge
857	858	0	Via Swap
857	859	1.441025168756319	Via Edge CCW
857	909	41.596875698202616	Staging
858	857	0	Via Swap
858	856	1.4410251687563154	Via Edge CW
858	909	41.596875698202616	Staging
859	860	0	Via Swap
859	861	1.4410251687563154	Via Edge CCW
859	909	38.78886698781778	Staging
860	859	0	Via Swap
860	858	1.441025168756319	Via Edge CW
860	909	38.78886698781778	Staging
861	862	0	Via Swap
861	863	1.441025168756319	Via Edge CCW
861	909	35.992565410671055	Staging
862	861	0	Via Swap
862	860	1.4410251687563154	Via Edge CW
862	909	35.992565410671055	Staging
863	864	0	Via Swap
863	865	1.4410251687563154	Via Edge CCW
863	909	33.210928258333304	Staging
864	863	0	Via Swap
864	862	1.441025168756319	Via Edge CW
864	909	33.210928258333304	Staging
865	866	0	Via Swap
865	867	1.4410251687563171	Via Edge CCW
865	909	30.447974891851224	Staging
866	865	0	Via Swap
866	864	1.4410251687563154	Via Edge CW
866	909	30.447974891851224	Staging
867	868	0	Via Swap
867	869	1.4410251687563171	Via Edge CCW
867	909	27.709294876365167	Staging
868	867	0	Via Swap
868	866	1.4410251687563171	Via Edge CW
868	909	27.709294876365167	Staging
869	870	0	Via Swap
869	871	1.4410251687563171	Via Edge CCW
869	909	25.002865803179933	Staging
870	869	0	Via Swap
870	868	1.4410251687563171	Via Edge CW
870	909	25.002865803179933	Staging
871	872	0	Via Swap
871	873	1.4410251687563171	Via Edge CCW
871	909	22.340411869395012	Staging
872	871	0	Via Swap
872	870	1.4410251687563171	Via Edge CW
872	909	22.340411869395012	Staging
873	874	0	Via Swap
873	875	1.4410251687563171	Via Edge CCW
873	909	19.739734925081944	Staging
874	873	0	Via Swap
874	872	1.4410251687563171	Via Edge CW
874	909	19.739734925081944	Staging
875	876	0	Via Swap
875	877	1.4410251687563171	Via Edge CCW
875	909	17.228833263651595	Staging
876	875	0	Via Swap
876	874	1.4410251687563171	Via Edge CW
876	909	17.228833263651595	Staging
877	878	0	Via Swap
877	879	1.4410251687563154	Via Edge CCW
877	909	14.853305512135295	Staging
878	877	0	Via Swap
878	876	1.4410251687563171	Via Edge CW
878	909	14.853305512135295	Staging
879	880	0	Via Swap
879	881	1.441025168756319	Via Edge CCW
879	909	12.689409046242591	Staging
880	879	0	Via Swap
880	878	1.4410251687563154	Via Edge CW
880	909	12.689409046242591	Staging
880	39	7.300078971028319	Transit Edge
881	882	0	Via Swap
881	883	1.4410251687563163	Via Edge CCW
881	909	10.864342941237226	Staging
881	39	1.6929548346383347	Transit Edge
882	881	0	Via Swap
882	880	1.441025168756319	Via Edge CW
882	909	10.864342941237226	Staging
882	39	1.6929548346383347	Transit Edge
882	119	8.542025354288013	Transit Edge
883	884	0	Via Swap
883	885	1.4532201811784304	Via Edge CCW
883	909	9.573882255525831	Staging
883	39	4.346349312314224	Transit Edge
884	883	0	Via Swap
884	882	1.4410251687563163	Via Edge CW
884	909	9.573882255525831	Staging
884	39	4.346349312314224	Transit Edge
884	119	8.324790171986201	Transit Edge
885	886	0	Via Swap
885	887	1.4532201811784295	Via Edge CCW
885	909	12.356815859894857	Staging
885	119	3.3189295387180024	Transit Edge
886	885	0	Via Swap
886	884	1.4532201811784304	Via Edge CW
886	909	12.356815859894857	Staging
886	39	6.586328681812098	Transit Edge
886	119	3.3189295387180024	Transit Edge
886	199	10.080427807663325	Transit Edge
887	888	0	Via Swap
887	889	1.4532201811784304	Via Edge CCW
887	909	15.186091205702008	Staging
887	119	4.506397651988597	Transit Edge
887	199	4.647513406864942	Transit Edge
888	887	0	Via Swap
888	886	1.4532201811784295	Via Edge CW
888	909	15.186091205702008	Staging
888	119	4.506397651988597	Transit Edge
888	199	4.647513406864942	Transit Edge
888	279	11.120567238213903	Transit Edge
889	890	0	Via Swap
889	891	1.4532201811784304	Via Edge CCW
889	909	18.039917549155398	Staging
889	199	3.027028507953517	Transit Edge
889	279	5.672355953412552	Transit Edge
890	889	0	Via Swap
890	888	1.4532201811784304	Via Edge CW
890	909	18.039917549155398	Staging
890	199	3.027028507953517	Transit Edge
890	279	5.672355953412552	Transit Edge
891	892	0	Via Swap
891	893	1.4532201811784304	Via Edge CCW
891	909	20.90824419726879	Staging
891	279	2.874611811060739	Transit Edge
892	891	0	Via Swap
892	890	1.4532201811784304	Via Edge CW
892	909	20.90824419726879	Staging
892	199	8.019076439867241	Transit Edge
892	279	2.874611811060739	Transit Edge
892	359	6.884550600633138	Transit Edge
893	894	0	Via Swap
893	895	1.4532201811784304	Via Edge CCW
893	909	23.785825964317432	Staging
893	359	2.5286111154450563	Transit Edge
894	893	0	Via Swap
894	892	1.4532201811784304	Via Edge CW
894	909	23.785825964317432	Staging
894	279	7.20627009664928	Transit Edge
894	359	2.5286111154450563	Transit Edge
894	439	8.290396513666598	Transit Edge
895	896	0	Via Swap
895	897	1.4532201811784304	Via Edge CCW
895	909	26.669667214962484	Staging
895	359	5.741940091390987	Transit Edge
895	439	3.4134085352695904	Transit Edge
896	895	0	Via Swap
896	894	1.4532201811784304	Via Edge CW
896	909	26.669667214962484	Staging
896	359	5.741940091390987	Transit Edge
896	439	3.4134085352695904	Transit Edge
896	519	10.112073356704958	Transit Edge
897	898	0	Via Swap
897	899	1.4532201811784304	Via Edge CCW
897	909	29.557935872938867	Staging
897	439	4.706506766486578	Transit Edge
897	519	4.7342564006725905	Transit Edge
898	897	0	Via Swap
898	896	1.4532201811784304	Via Edge CW
898	909	29.557935872938867	Staging
898	359	11.274974168156245	Transit Edge
898	439	4.706506766486578	Transit Edge
898	519	4.7342564006725905	Transit Edge
898	599	11.21567906590249	Transit Edge
899	900	0	Via Swap
899	901	1.4532201811784304	Via Edge CCW
899	909	32.44944973247716	Staging
899	519	3.1861425584447978	Transit Edge
899	599	5.782755606294064	Transit Edge
900	899	0	Via Swap
900	898	1.4532201811784304	Via Edge CW
900	909	32.44944973247716	Staging
900	439	10.011504317374749	Transit Edge
900	519	3.1861425584447978	Transit Edge
900	599	5.782755606294064	Transit Edge
901	902	0	Via Swap
901	903	1.4532201811784304	Via Edge CCW
901	909	35.34341231364385	Staging
901	599	2.9441862261793394	Transit Edge
902	901	0	Via Swap
902	900	1.4532201811784304	Via Edge CW
902	909	35.34341231364385	Staging
902	519	8.091290983375494	Transit Edge
902	599	2.9441862261793394	Transit Edge
902	679	7.016693092703612	Transit Edge
903	904	0	Via Swap
903	905	1.4532201811784304	Via Edge CCW
903	909	38.239267659877804	Staging
903	679	2.816877221391186	Transit Edge
904	903	0	Via Swap
904	902	1.4532201811784304	Via Edge CW
904	909	38.239267659877804	Staging
904	599	7.174633600834227	Transit Edge
904	679	2.816877221391186	Transit Edge
904	759	8.607780785720388	Transit Edge
905	906	0	Via Swap
905	907	1.4532201811784269	Via Edge CCW
905	909	41.13661604355899	Staging
905	679	5.849339898064321	Transit Edge
905	759	3.637082857955306	Transit Edge
906	905	0	Via Swap
906	904	1.4532201811784304	Via Edge CW
906	909	41.13661604355899	Staging
906	679	5.849339898064321	Transit Edge
906	759	3.637082857955306	Transit Edge
907	908	0	Via Swap
907	801	1.4532201811784304	Via Edge CCW
907	909	44.03516275689788	Staging
907	759	4.465648666850551	Transit Edge
908	907	0	Via Swap
908	906	1.4532201811784269	Via Edge CW
908	909	44.03516275689788	Staging
908	679	11.31697494232792	Transit Edge
908	759	4.465648666850551	Transit Edge
909	801	46.93468578458423	Staging
909	855	44.41437111792457	Staging
909	857	41.596875698202616	Staging
909	859	38.78886698781778	Staging
909	861	35.992565410671055	Staging
909	863	33.210928258333304	Staging
909	865	30.447974891851224	Staging
909	867	27.709294876365167	Staging
909	869	25.002865803179933	Staging
909	871	22.340411869395012	Staging
909	873	19.739734925081944	Staging
909	875	17.228833263651595	Staging
909	877	14.853305512135295	Staging
909	879	12.689409046242591	Staging
909	881	10.864342941237226	Staging
909	883	9.573882255525831	Staging
909	885	12.356815859894857	Staging
909	887	15.186091205702008	Staging
909	889	18.039917549155398	Staging
909	891	20.90824419726879	Staging
909	893	23.785825964317432	Staging
909	895	26.669667214962484	Staging
909	897	29.557935872938867	Staging
909	899	32.44944973247716	Staging
909	901	35.34341231364385	Staging
909	903	38.239267659877804	Staging
909	905	41.13661604355899	Staging
909	907	44.03516275689788	Staging
909	802	46.93468578458423	Staging
909	856	44.41437111792457	Staging
909	858	41.596875698202616	Staging
909	860	38.78886698781778	Staging
909	862	35.992565410671055	Staging
909	864	33.210928258333304	Staging
909	866	30.447974891851224	Staging
909	868	27.709294876365167	Staging
909	870	25.002865803179933	Staging
909	872	22.340411869395012	Staging
909	874	19.739734925081944	Staging
909	876	17.228833263651595	Staging
909	878	14.853305512135295	Staging
909	880	12.689409046242591	Staging
909	882	10.864342941237226	Staging
909	884	9.573882255525831	Staging
909	886	12.356815859894857	Staging
909	888	15.186091205702008	Staging
909	890	18.039917549155398	Staging
909	892	20.90824419726879	Staging
909	894	23.785825964317432	Staging
909	896	26.669667214962484	Staging
909	898	29.557935872938867	Staging
909	900	32.44944973247716	Staging
909	902	35.34341231364385	Staging
909	904	38.239267659877804	Staging
909	906	41.13661604355899	Staging
909	908	44.03516275689788	Staging
\.


--
-- Data for Name: graph_node; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.graph_node (id, node_type, x, y, theta, obj_id) FROM stdin;
1	Pickup	0.491622160326074	-3.023460508831056	-0.0022689280275926286	1
2	Pickup	0.9356221603260739	-3.0244605088310563	-0.0022689280275926286	2
3	Pickup	1.3796221603260739	-3.0254605088310567	-0.0022689280275926286	3
4	Pickup	1.823622160326074	-3.026460508831056	-0.0022689280275926286	4
5	Pickup	2.267622160326074	-3.0274605088310564	-0.0022689280275926286	5
6	Pickup	2.711622160326074	-3.028460508831056	-0.0022689280275926286	6
7	Pickup	3.155622160326074	-3.029460508831056	-0.0022689280275926286	7
8	Pickup	3.599622160326074	-3.0304605088310566	-0.0022689280275926286	8
9	Pickup	4.043622160326074	-3.031460508831056	-0.0022689280275926286	9
10	Pickup	4.487622160326074	-3.0324605088310563	-0.0022689280275926286	10
11	Pickup	4.931622160326074	-3.0334605088310567	-0.0022689280275926286	11
12	Pickup	5.375622160326074	-3.034460508831056	-0.0022689280275926286	12
13	Pickup	5.819622160326074	-3.034460508831056	-0.0022689280275926286	13
14	Pickup	6.263622160326074	-3.0354605088310564	-0.0022689280275926286	14
15	Pickup	6.708622160326074	-3.036460508831056	-0.0022689280275926286	15
16	Pickup	7.152622160326074	-3.0374605088310562	-0.0022689280275926286	16
17	Pickup	7.596622160326074	-3.0384605088310566	-0.0022689280275926286	17
18	Pickup	8.040622160326073	-3.039460508831056	-0.0022689280275926286	18
19	Pickup	8.484622160326074	-3.0404605088310563	-0.0022689280275926286	19
20	Pickup	8.928622160326073	-3.0414605088310567	-0.0022689280275926286	20
21	Pickup	9.372622160326074	-3.042460508831056	-0.0022689280275926286	21
22	Pickup	9.816622160326073	-3.0434605088310565	-0.0022689280275926286	22
23	Pickup	10.260622160326074	-3.044460508831056	-0.0022689280275926286	23
24	Pickup	10.704622160326073	-3.0454605088310562	-0.0022689280275926286	24
25	Pickup	11.148622160326074	-3.0464605088310566	-0.0022689280275926286	25
26	Pickup	11.592622160326073	-3.047460508831056	-0.0022689280275926286	26
27	Pickup	12.036622160326074	-3.0484605088310563	-0.0022689280275926286	27
28	Pickup	12.480622160326073	-3.0494605088310567	-0.0022689280275926286	28
29	Pickup	12.924622160326074	-3.050460508831056	-0.0022689280275926286	29
30	Pickup	13.368622160326073	-3.0514605088310565	-0.0022689280275926286	30
31	Pickup	13.813622160326073	-3.052460508831056	-0.0022689280275926286	31
32	Pickup	14.257622160326074	-3.0534605088310562	-0.0022689280275926286	32
33	Pickup	14.701622160326073	-3.0544605088310566	-0.0022689280275926286	33
34	Pickup	15.145622160326074	-3.055460508831056	-0.0022689280275926286	34
35	Pickup	15.589622160326073	-3.0564605088310564	-0.0022689280275926286	35
36	Pickup	16.033622160326075	-3.0574605088310567	-0.0022689280275926286	36
37	Pickup	16.477622160326074	-3.058460508831056	-0.0022689280275926286	37
38	Pickup	16.921622160326073	-3.0594605088310565	-0.0022689280275926286	38
39	Entering	-0.7083747508546145	-3.020737797534048	-0.0022689280275926286	\N
40	Exiting	18.12161907150676	-3.0621832201280643	-0.0022689280275926286	\N
41	Pickup	16.514377839673926	-5.058539491168943	3.1393237255622006	38
42	Pickup	16.070377839673927	-5.057539491168943	3.1393237255622006	37
43	Pickup	15.626377839673927	-5.056539491168944	3.1393237255622006	36
44	Pickup	15.182377839673926	-5.055539491168943	3.1393237255622006	35
45	Pickup	14.738377839673927	-5.054539491168943	3.1393237255622006	34
46	Pickup	14.294377839673926	-5.0535394911689435	3.1393237255622006	33
47	Pickup	13.850377839673927	-5.052539491168943	3.1393237255622006	32
48	Pickup	13.406377839673926	-5.051539491168943	3.1393237255622006	31
49	Pickup	12.961377839673926	-5.050539491168943	3.1393237255622006	30
50	Pickup	12.517377839673927	-5.049539491168943	3.1393237255622006	29
51	Pickup	12.073377839673926	-5.048539491168944	3.1393237255622006	28
52	Pickup	11.629377839673927	-5.047539491168943	3.1393237255622006	27
53	Pickup	11.185377839673926	-5.046539491168943	3.1393237255622006	26
54	Pickup	10.741377839673927	-5.0455394911689435	3.1393237255622006	25
55	Pickup	10.297377839673926	-5.044539491168943	3.1393237255622006	24
56	Pickup	9.853377839673927	-5.043539491168943	3.1393237255622006	23
57	Pickup	9.409377839673926	-5.042539491168943	3.1393237255622006	22
58	Pickup	8.965377839673927	-5.041539491168943	3.1393237255622006	21
59	Pickup	8.521377839673926	-5.040539491168944	3.1393237255622006	20
60	Pickup	8.077377839673927	-5.039539491168943	3.1393237255622006	19
61	Pickup	7.633377839673925	-5.038539491168943	3.1393237255622006	18
62	Pickup	7.189377839673925	-5.0375394911689435	3.1393237255622006	17
63	Pickup	6.7453778396739255	-5.036539491168943	3.1393237255622006	16
64	Pickup	6.3013778396739255	-5.035539491168943	3.1393237255622006	15
65	Pickup	5.856377839673925	-5.034539491168943	3.1393237255622006	14
66	Pickup	5.412377839673925	-5.033539491168943	3.1393237255622006	13
67	Pickup	4.968377839673925	-5.033539491168943	3.1393237255622006	12
68	Pickup	4.524377839673925	-5.032539491168944	3.1393237255622006	11
69	Pickup	4.080377839673925	-5.031539491168943	3.1393237255622006	10
70	Pickup	3.636377839673926	-5.030539491168943	3.1393237255622006	9
71	Pickup	3.192377839673926	-5.0295394911689435	3.1393237255622006	8
72	Pickup	2.748377839673926	-5.028539491168943	3.1393237255622006	7
73	Pickup	2.304377839673926	-5.027539491168943	3.1393237255622006	6
74	Pickup	1.8603778396739261	-5.026539491168943	3.1393237255622006	5
75	Pickup	1.4163778396739262	-5.025539491168943	3.1393237255622006	4
76	Pickup	0.972377839673926	-5.024539491168944	3.1393237255622006	3
77	Pickup	0.528377839673926	-5.023539491168943	3.1393237255622006	2
78	Pickup	0.08437783967392601	-5.022539491168943	3.1393237255622006	1
79	Entering	17.714374750854613	-5.061262202465951	3.1393237255622006	\N
80	Exiting	-1.1156190715067624	-5.0198167798719355	3.1393237255622006	\N
81	Pickup	0.06463291762040821	-4.808285951784715	-0.006283185307179586	39
82	Pickup	0.5186329176204082	-4.811285951784713	-0.006283185307179586	40
83	Pickup	0.9736329176204082	-4.814285951784713	-0.006283185307179586	41
84	Pickup	1.4276329176204081	-4.817285951784713	-0.006283185307179586	42
85	Pickup	1.8826329176204082	-4.820285951784713	-0.006283185307179586	43
86	Pickup	2.3366329176204084	-4.8232859517847135	-0.006283185307179586	44
87	Pickup	2.7916329176204084	-4.825285951784714	-0.006283185307179586	45
88	Pickup	3.245632917620408	-4.828285951784714	-0.006283185307179586	46
89	Pickup	3.7006329176204082	-4.831285951784714	-0.006283185307179586	47
90	Pickup	4.154632917620408	-4.8342859517847145	-0.006283185307179586	48
91	Pickup	4.608632917620408	-4.837285951784715	-0.006283185307179586	49
92	Pickup	5.063632917620408	-4.840285951784715	-0.006283185307179586	50
93	Pickup	5.517632917620408	-4.843285951784713	-0.006283185307179586	51
94	Pickup	5.972632917620408	-4.845285951784714	-0.006283185307179586	52
95	Pickup	6.426632917620409	-4.848285951784714	-0.006283185307179586	53
96	Pickup	6.881632917620409	-4.851285951784714	-0.006283185307179586	54
97	Pickup	7.3356329176204085	-4.854285951784714	-0.006283185307179586	55
98	Pickup	7.7906329176204085	-4.857285951784714	-0.006283185307179586	56
99	Pickup	8.24463291762041	-4.860285951784714	-0.006283185307179586	57
100	Pickup	8.698632917620408	-4.8632859517847145	-0.006283185307179586	58
101	Pickup	9.153632917620408	-4.865285951784713	-0.006283185307179586	59
102	Pickup	9.607632917620409	-4.8682859517847135	-0.006283185307179586	60
103	Pickup	10.062632917620409	-4.871285951784714	-0.006283185307179586	61
104	Pickup	10.516632917620408	-4.874285951784714	-0.006283185307179586	62
105	Pickup	10.971632917620408	-4.877285951784714	-0.006283185307179586	63
106	Pickup	11.425632917620408	-4.880285951784714	-0.006283185307179586	64
107	Pickup	11.879632917620409	-4.882285951784715	-0.006283185307179586	65
869	Via_CCW	8.310778948681428	-2.820737797534048	3.141592653589793	\N
108	Pickup	12.334632917620409	-4.885285951784715	-0.006283185307179586	66
109	Pickup	12.788632917620408	-4.888285951784713	-0.006283185307179586	67
110	Pickup	13.243632917620408	-4.891285951784713	-0.006283185307179586	68
111	Pickup	13.697632917620409	-4.894285951784713	-0.006283185307179586	69
112	Pickup	14.152632917620409	-4.897285951784713	-0.006283185307179586	70
113	Pickup	14.606632917620407	-4.9002859517847135	-0.006283185307179586	71
114	Pickup	15.061632917620408	-4.902285951784714	-0.006283185307179586	72
115	Pickup	15.515632917620408	-4.905285951784714	-0.006283185307179586	73
116	Pickup	15.969632917620409	-4.908285951784714	-0.006283185307179586	74
117	Pickup	16.424632917620407	-4.9112859517847145	-0.006283185307179586	75
118	Pickup	16.878632917620408	-4.914285951784715	-0.006283185307179586	76
119	Entering	-1.1353433954069563	-4.800746179026044	-0.006283185307179586	\N
120	Exiting	18.07860923064777	-4.921825724543385	-0.006283185307179586	\N
121	Pickup	16.46336708237959	-6.911714048215286	3.135309468282614	76
122	Pickup	16.00936708237959	-6.908714048215286	3.135309468282614	75
123	Pickup	15.554367082379592	-6.905714048215286	3.135309468282614	74
124	Pickup	15.100367082379591	-6.902714048215286	3.135309468282614	73
125	Pickup	14.64636708237959	-6.8997140482152854	3.135309468282614	72
126	Pickup	14.19136708237959	-6.8977140482152866	3.135309468282614	71
127	Pickup	13.737367082379592	-6.894714048215286	3.135309468282614	70
128	Pickup	13.282367082379592	-6.891714048215286	3.135309468282614	69
129	Pickup	12.828367082379591	-6.888714048215286	3.135309468282614	68
130	Pickup	12.373367082379591	-6.885714048215286	3.135309468282614	67
131	Pickup	11.919367082379592	-6.882714048215286	3.135309468282614	66
132	Pickup	11.464367082379592	-6.879714048215286	3.135309468282614	65
133	Pickup	11.010367082379592	-6.877714048215285	3.135309468282614	64
134	Pickup	10.556367082379591	-6.874714048215287	3.135309468282614	63
135	Pickup	10.101367082379591	-6.871714048215287	3.135309468282614	62
136	Pickup	9.647367082379592	-6.868714048215287	3.135309468282614	61
137	Pickup	9.192367082379592	-6.8657140482152865	3.135309468282614	60
138	Pickup	8.738367082379591	-6.862714048215286	3.135309468282614	59
139	Pickup	8.283367082379591	-6.860714048215286	3.135309468282614	58
140	Pickup	7.8293670823795924	-6.857714048215286	3.135309468282614	57
141	Pickup	7.375367082379592	-6.8547140482152855	3.135309468282614	56
142	Pickup	6.920367082379592	-6.851714048215285	3.135309468282614	55
143	Pickup	6.466367082379592	-6.848714048215285	3.135309468282614	54
144	Pickup	6.011367082379592	-6.845714048215287	3.135309468282614	53
145	Pickup	5.557367082379591	-6.842714048215287	3.135309468282614	52
146	Pickup	5.102367082379591	-6.840714048215286	3.135309468282614	51
147	Pickup	4.6483670823795915	-6.837714048215286	3.135309468282614	50
148	Pickup	4.193367082379591	-6.834714048215286	3.135309468282614	49
149	Pickup	3.739367082379592	-6.831714048215286	3.135309468282614	48
150	Pickup	3.285367082379592	-6.828714048215286	3.135309468282614	47
151	Pickup	2.830367082379592	-6.825714048215286	3.135309468282614	46
152	Pickup	2.376367082379592	-6.8227140482152855	3.135309468282614	45
153	Pickup	1.921367082379592	-6.820714048215287	3.135309468282614	44
154	Pickup	1.4673670823795921	-6.8177140482152865	3.135309468282614	43
155	Pickup	1.012367082379592	-6.814714048215286	3.135309468282614	42
156	Pickup	0.5583670823795921	-6.811714048215286	3.135309468282614	41
157	Pickup	0.10336708237959205	-6.808714048215286	3.135309468282614	40
158	Pickup	-0.35063291762040794	-6.805714048215286	3.135309468282614	39
159	Entering	17.663343395406955	-6.919253820973957	3.135309468282614	\N
160	Exiting	-1.5506092306477723	-6.798174275456615	3.135309468282614	\N
161	Pickup	0.02513187585077098	-6.719755824385575	0.0012217304763960308	77
162	Pickup	0.48213187585077094	-6.718755824385576	0.0012217304763960308	78
163	Pickup	0.939131875850771	-6.718755824385576	0.0012217304763960308	79
164	Pickup	1.396131875850771	-6.717755824385575	0.0012217304763960308	80
165	Pickup	1.8531318758507709	-6.716755824385575	0.0012217304763960308	81
166	Pickup	2.3101318758507707	-6.716755824385575	0.0012217304763960308	82
167	Pickup	2.767131875850771	-6.7157558243855755	0.0012217304763960308	83
168	Pickup	3.224131875850771	-6.7157558243855755	0.0012217304763960308	84
169	Pickup	3.6811318758507707	-6.714755824385575	0.0012217304763960308	85
170	Pickup	4.138131875850771	-6.714755824385575	0.0012217304763960308	86
171	Pickup	4.595131875850771	-6.713755824385576	0.0012217304763960308	87
172	Pickup	5.052131875850772	-6.712755824385575	0.0012217304763960308	88
173	Pickup	5.5091318758507715	-6.712755824385575	0.0012217304763960308	89
174	Pickup	5.966131875850771	-6.711755824385575	0.0012217304763960308	90
175	Pickup	6.423131875850771	-6.711755824385575	0.0012217304763960308	91
176	Pickup	6.880131875850771	-6.710755824385576	0.0012217304763960308	92
177	Pickup	7.337131875850771	-6.710755824385576	0.0012217304763960308	93
178	Pickup	7.794131875850772	-6.709755824385575	0.0012217304763960308	94
179	Pickup	8.25113187585077	-6.708755824385575	0.0012217304763960308	95
180	Pickup	8.70813187585077	-6.708755824385575	0.0012217304763960308	96
181	Pickup	9.16513187585077	-6.7077558243855755	0.0012217304763960308	97
182	Pickup	9.622131875850771	-6.7077558243855755	0.0012217304763960308	98
183	Pickup	10.07913187585077	-6.706755824385575	0.0012217304763960308	99
184	Pickup	10.535131875850771	-6.706755824385575	0.0012217304763960308	100
185	Pickup	10.99213187585077	-6.705755824385576	0.0012217304763960308	101
186	Pickup	11.449131875850771	-6.704755824385575	0.0012217304763960308	102
187	Pickup	11.90613187585077	-6.704755824385575	0.0012217304763960308	103
188	Pickup	12.36313187585077	-6.703755824385575	0.0012217304763960308	104
189	Pickup	12.82013187585077	-6.703755824385575	0.0012217304763960308	105
190	Pickup	13.27713187585077	-6.702755824385576	0.0012217304763960308	106
191	Pickup	13.734131875850771	-6.701755824385575	0.0012217304763960308	107
192	Pickup	14.19113187585077	-6.701755824385575	0.0012217304763960308	108
193	Pickup	14.64813187585077	-6.700755824385575	0.0012217304763960308	109
194	Pickup	15.10513187585077	-6.700755824385575	0.0012217304763960308	110
195	Pickup	15.56213187585077	-6.6997558243855755	0.0012217304763960308	111
196	Pickup	16.01913187585077	-6.6997558243855755	0.0012217304763960308	112
197	Pickup	16.476131875850772	-6.698755824385575	0.0012217304763960308	113
198	Pickup	16.933131875850773	-6.697755824385576	0.0012217304763960308	114
199	Entering	-1.1748672285741262	-6.721221900592533	0.0012217304763960308	\N
200	Exiting	18.13313098027567	-6.696289748178618	0.0012217304763960308	\N
201	Pickup	16.53286812414923	-8.698244175614425	-3.140370923113397	114
202	Pickup	16.075868124149228	-8.699244175614425	-3.140370923113397	113
203	Pickup	15.61886812414923	-8.700244175614424	-3.140370923113397	112
204	Pickup	15.16186812414923	-8.700244175614424	-3.140370923113397	111
205	Pickup	14.704868124149229	-8.701244175614423	-3.140370923113397	110
206	Pickup	14.24786812414923	-8.701244175614423	-3.140370923113397	109
207	Pickup	13.79086812414923	-8.702244175614425	-3.140370923113397	108
208	Pickup	13.33386812414923	-8.702244175614425	-3.140370923113397	107
209	Pickup	12.87686812414923	-8.703244175614424	-3.140370923113397	106
210	Pickup	12.419868124149229	-8.704244175614424	-3.140370923113397	105
211	Pickup	11.96286812414923	-8.704244175614424	-3.140370923113397	104
212	Pickup	11.505868124149229	-8.705244175614425	-3.140370923113397	103
213	Pickup	11.04886812414923	-8.705244175614425	-3.140370923113397	102
214	Pickup	10.59186812414923	-8.706244175614424	-3.140370923113397	101
215	Pickup	10.13486812414923	-8.707244175614424	-3.140370923113397	100
216	Pickup	9.67886812414923	-8.707244175614424	-3.140370923113397	99
217	Pickup	9.22186812414923	-8.708244175614425	-3.140370923113397	98
218	Pickup	8.76486812414923	-8.708244175614425	-3.140370923113397	97
219	Pickup	8.307868124149229	-8.709244175614424	-3.140370923113397	96
220	Pickup	7.850868124149229	-8.709244175614424	-3.140370923113397	95
221	Pickup	7.393868124149229	-8.710244175614424	-3.140370923113397	94
222	Pickup	6.936868124149228	-8.711244175614425	-3.140370923113397	93
223	Pickup	6.479868124149228	-8.711244175614425	-3.140370923113397	92
224	Pickup	6.022868124149229	-8.712244175614424	-3.140370923113397	91
225	Pickup	5.565868124149229	-8.712244175614424	-3.140370923113397	90
226	Pickup	5.108868124149229	-8.713244175614424	-3.140370923113397	89
227	Pickup	4.651868124149229	-8.713244175614424	-3.140370923113397	88
228	Pickup	4.194868124149228	-8.714244175614425	-3.140370923113397	87
229	Pickup	3.7378681241492293	-8.715244175614425	-3.140370923113397	86
230	Pickup	3.280868124149229	-8.715244175614425	-3.140370923113397	85
231	Pickup	2.823868124149229	-8.716244175614424	-3.140370923113397	84
232	Pickup	2.3668681241492293	-8.716244175614424	-3.140370923113397	83
233	Pickup	1.909868124149229	-8.717244175614423	-3.140370923113397	82
234	Pickup	1.4528681241492292	-8.717244175614423	-3.140370923113397	81
235	Pickup	0.9958681241492291	-8.718244175614425	-3.140370923113397	80
236	Pickup	0.538868124149229	-8.719244175614424	-3.140370923113397	79
237	Pickup	0.08186812414922906	-8.719244175614424	-3.140370923113397	78
238	Pickup	-0.3751318758507709	-8.720244175614424	-3.140370923113397	77
239	Entering	17.732867228574126	-8.696778099407467	-3.140370923113397	\N
240	Exiting	-1.5751309802756681	-8.721710251821381	-3.140370923113397	\N
241	Pickup	0.1101454900438136	-8.424567262154726	-0.0027925268031909274	115
242	Pickup	0.5651454900438135	-8.425567262154727	-0.0027925268031909274	116
243	Pickup	1.0201454900438136	-8.426567262154727	-0.0027925268031909274	117
244	Pickup	1.4761454900438136	-8.428567262154727	-0.0027925268031909274	118
245	Pickup	1.9311454900438136	-8.429567262154727	-0.0027925268031909274	119
246	Pickup	2.3871454900438134	-8.430567262154726	-0.0027925268031909274	120
247	Pickup	2.8421454900438134	-8.432567262154727	-0.0027925268031909274	121
248	Pickup	3.2971454900438135	-8.433567262154726	-0.0027925268031909274	122
249	Pickup	3.7531454900438135	-8.434567262154726	-0.0027925268031909274	123
250	Pickup	4.2081454900438136	-8.435567262154727	-0.0027925268031909274	124
251	Pickup	4.664145490043814	-8.437567262154726	-0.0027925268031909274	125
252	Pickup	5.119145490043814	-8.438567262154727	-0.0027925268031909274	126
253	Pickup	5.574145490043814	-8.439567262154727	-0.0027925268031909274	127
254	Pickup	6.030145490043814	-8.441567262154727	-0.0027925268031909274	128
255	Pickup	6.485145490043814	-8.442567262154727	-0.0027925268031909274	129
256	Pickup	6.941145490043814	-8.443567262154726	-0.0027925268031909274	130
257	Pickup	7.396145490043814	-8.445567262154727	-0.0027925268031909274	131
258	Pickup	7.851145490043814	-8.446567262154726	-0.0027925268031909274	132
259	Pickup	8.307145490043814	-8.447567262154726	-0.0027925268031909274	133
260	Pickup	8.762145490043814	-8.448567262154727	-0.0027925268031909274	134
261	Pickup	9.218145490043813	-8.450567262154726	-0.0027925268031909274	135
262	Pickup	9.673145490043813	-8.451567262154727	-0.0027925268031909274	136
263	Pickup	10.128145490043813	-8.452567262154727	-0.0027925268031909274	137
264	Pickup	10.584145490043815	-8.454567262154727	-0.0027925268031909274	138
265	Pickup	11.039145490043815	-8.455567262154727	-0.0027925268031909274	139
266	Pickup	11.495145490043814	-8.456567262154726	-0.0027925268031909274	140
267	Pickup	11.950145490043814	-8.458567262154727	-0.0027925268031909274	141
268	Pickup	12.405145490043815	-8.459567262154726	-0.0027925268031909274	142
269	Pickup	12.861145490043814	-8.460567262154727	-0.0027925268031909274	143
270	Pickup	13.316145490043814	-8.461567262154727	-0.0027925268031909274	144
271	Pickup	13.772145490043814	-8.463567262154726	-0.0027925268031909274	145
272	Pickup	14.227145490043814	-8.464567262154727	-0.0027925268031909274	146
273	Pickup	14.682145490043814	-8.465567262154726	-0.0027925268031909274	147
274	Pickup	15.138145490043813	-8.467567262154727	-0.0027925268031909274	148
275	Pickup	15.593145490043813	-8.468567262154727	-0.0027925268031909274	149
276	Pickup	16.049145490043813	-8.469567262154726	-0.0027925268031909274	150
277	Pickup	16.504145490043815	-8.470567262154727	-0.0027925268031909274	151
278	Pickup	16.960145490043814	-8.472567262154726	-0.0027925268031909274	152
279	Entering	-1.089849831035659	-8.421216234346234	-0.0027925268031909274	\N
280	Exiting	18.160140811123288	-8.475918289963218	-0.0027925268031909274	\N
281	Pickup	16.551854509956186	-10.471432737845273	3.1388001267866024	152
282	Pickup	16.095854509956187	-10.469432737845274	3.1388001267866024	151
283	Pickup	15.640854509956187	-10.468432737845273	3.1388001267866024	150
284	Pickup	15.184854509956185	-10.467432737845273	3.1388001267866024	149
285	Pickup	14.729854509956185	-10.466432737845274	3.1388001267866024	148
286	Pickup	14.273854509956186	-10.464432737845273	3.1388001267866024	147
287	Pickup	13.818854509956186	-10.463432737845274	3.1388001267866024	146
288	Pickup	13.363854509956186	-10.462432737845273	3.1388001267866024	145
289	Pickup	12.907854509956186	-10.460432737845274	3.1388001267866024	144
290	Pickup	12.452854509956186	-10.459432737845274	3.1388001267866024	143
291	Pickup	11.996854509956187	-10.458432737845273	3.1388001267866024	142
292	Pickup	11.541854509956186	-10.457432737845274	3.1388001267866024	141
293	Pickup	11.086854509956186	-10.455432737845273	3.1388001267866024	140
294	Pickup	10.630854509956187	-10.454432737845273	3.1388001267866024	139
295	Pickup	10.175854509956187	-10.453432737845274	3.1388001267866024	138
296	Pickup	9.719854509956185	-10.451432737845273	3.1388001267866024	137
297	Pickup	9.264854509956185	-10.450432737845274	3.1388001267866024	136
298	Pickup	8.809854509956185	-10.449432737845273	3.1388001267866024	135
299	Pickup	8.353854509956186	-10.447432737845274	3.1388001267866024	134
300	Pickup	7.898854509956186	-10.446432737845273	3.1388001267866024	133
301	Pickup	7.442854509956186	-10.445432737845273	3.1388001267866024	132
302	Pickup	6.987854509956186	-10.444432737845274	3.1388001267866024	131
303	Pickup	6.532854509956186	-10.442432737845273	3.1388001267866024	130
304	Pickup	6.076854509956187	-10.441432737845274	3.1388001267866024	129
305	Pickup	5.6218545099561865	-10.440432737845274	3.1388001267866024	128
306	Pickup	5.165854509956187	-10.438432737845273	3.1388001267866024	127
307	Pickup	4.710854509956187	-10.437432737845274	3.1388001267866024	126
308	Pickup	4.255854509956187	-10.436432737845273	3.1388001267866024	125
309	Pickup	3.799854509956186	-10.434432737845274	3.1388001267866024	124
310	Pickup	3.3448545099561864	-10.433432737845273	3.1388001267866024	123
311	Pickup	2.8888545099561864	-10.432432737845273	3.1388001267866024	122
312	Pickup	2.4338545099561864	-10.431432737845274	3.1388001267866024	121
313	Pickup	1.9788545099561863	-10.429432737845273	3.1388001267866024	120
314	Pickup	1.5228545099561865	-10.428432737845274	3.1388001267866024	119
315	Pickup	1.0678545099561865	-10.427432737845274	3.1388001267866024	118
316	Pickup	0.6118545099561864	-10.425432737845274	3.1388001267866024	117
317	Pickup	0.15685450995618644	-10.424432737845274	3.1388001267866024	116
318	Pickup	-0.2981454900438135	-10.423432737845273	3.1388001267866024	115
319	Entering	17.75184983103566	-10.474783765653763	3.1388001267866024	\N
320	Exiting	-1.4981408111232861	-10.420081710036783	3.1388001267866024	\N
321	Pickup	0.03641260357497922	-10.248033025737325	-0.005061454830783556	153
322	Pickup	0.4914126035749792	-10.250033025737324	-0.005061454830783556	154
323	Pickup	0.9474126035749793	-10.253033025737324	-0.005061454830783556	155
324	Pickup	1.4034126035749792	-10.255033025737323	-0.005061454830783556	156
325	Pickup	1.8594126035749792	-10.257033025737325	-0.005061454830783556	157
326	Pickup	2.315412603574979	-10.260033025737325	-0.005061454830783556	158
327	Pickup	2.771412603574979	-10.262033025737324	-0.005061454830783556	159
328	Pickup	3.227412603574979	-10.264033025737323	-0.005061454830783556	160
329	Pickup	3.682412603574979	-10.266033025737325	-0.005061454830783556	161
330	Pickup	4.138412603574979	-10.269033025737325	-0.005061454830783556	162
331	Pickup	4.5944126035749795	-10.271033025737324	-0.005061454830783556	163
332	Pickup	5.05041260357498	-10.273033025737323	-0.005061454830783556	164
333	Pickup	5.506412603574979	-10.276033025737323	-0.005061454830783556	165
334	Pickup	5.96241260357498	-10.278033025737322	-0.005061454830783556	166
335	Pickup	6.418412603574979	-10.280033025737325	-0.005061454830783556	167
336	Pickup	6.873412603574979	-10.282033025737324	-0.005061454830783556	168
337	Pickup	7.32941260357498	-10.285033025737324	-0.005061454830783556	169
338	Pickup	7.785412603574979	-10.287033025737323	-0.005061454830783556	170
339	Pickup	8.241412603574979	-10.289033025737325	-0.005061454830783556	171
340	Pickup	8.697412603574978	-10.291033025737324	-0.005061454830783556	172
341	Pickup	9.153412603574978	-10.294033025737324	-0.005061454830783556	173
342	Pickup	9.60941260357498	-10.296033025737323	-0.005061454830783556	174
343	Pickup	10.06441260357498	-10.298033025737325	-0.005061454830783556	175
344	Pickup	10.520412603574979	-10.301033025737325	-0.005061454830783556	176
345	Pickup	10.976412603574978	-10.303033025737324	-0.005061454830783556	177
346	Pickup	11.43241260357498	-10.305033025737323	-0.005061454830783556	178
347	Pickup	11.888412603574979	-10.307033025737322	-0.005061454830783556	179
348	Pickup	12.344412603574979	-10.310033025737322	-0.005061454830783556	180
349	Pickup	12.800412603574978	-10.312033025737325	-0.005061454830783556	181
350	Pickup	13.255412603574978	-10.314033025737324	-0.005061454830783556	182
351	Pickup	13.71141260357498	-10.317033025737324	-0.005061454830783556	183
352	Pickup	14.167412603574979	-10.319033025737323	-0.005061454830783556	184
353	Pickup	14.623412603574979	-10.321033025737325	-0.005061454830783556	185
354	Pickup	15.079412603574978	-10.323033025737324	-0.005061454830783556	186
355	Pickup	15.53541260357498	-10.326033025737324	-0.005061454830783556	187
356	Pickup	15.99041260357498	-10.328033025737323	-0.005061454830783556	188
357	Pickup	16.446412603574977	-10.330033025737325	-0.005061454830783556	189
358	Pickup	16.90241260357498	-10.332033025737324	-0.005061454830783556	190
359	Entering	-1.1635720254628332	-10.24195930587355	-0.005061454830783556	\N
360	Exiting	18.10239723261279	-10.338106745601099	-0.005061454830783556	\N
361	Pickup	16.489587396425023	-12.329966974262677	3.13653119875901	190
362	Pickup	16.03358739642502	-12.327966974262678	3.13653119875901	189
363	Pickup	15.577587396425022	-12.325966974262677	3.13653119875901	188
364	Pickup	15.122587396425022	-12.323966974262676	3.13653119875901	187
365	Pickup	14.66658739642502	-12.320966974262676	3.13653119875901	186
366	Pickup	14.210587396425021	-12.318966974262677	3.13653119875901	185
367	Pickup	13.754587396425022	-12.316966974262677	3.13653119875901	184
368	Pickup	13.298587396425022	-12.314966974262678	3.13653119875901	183
369	Pickup	12.84258739642502	-12.311966974262678	3.13653119875901	182
370	Pickup	12.38758739642502	-12.309966974262677	3.13653119875901	181
371	Pickup	11.931587396425021	-12.307966974262676	3.13653119875901	180
372	Pickup	11.475587396425022	-12.304966974262676	3.13653119875901	179
373	Pickup	11.019587396425022	-12.302966974262677	3.13653119875901	178
374	Pickup	10.56358739642502	-12.300966974262677	3.13653119875901	177
375	Pickup	10.107587396425021	-12.298966974262678	3.13653119875901	176
376	Pickup	9.651587396425022	-12.295966974262678	3.13653119875901	175
377	Pickup	9.196587396425022	-12.293966974262677	3.13653119875901	174
378	Pickup	8.74058739642502	-12.291966974262676	3.13653119875901	173
379	Pickup	8.284587396425021	-12.288966974262676	3.13653119875901	172
380	Pickup	7.828587396425021	-12.286966974262677	3.13653119875901	171
381	Pickup	7.372587396425021	-12.284966974262677	3.13653119875901	170
382	Pickup	6.9165873964250215	-12.282966974262678	3.13653119875901	169
383	Pickup	6.460587396425021	-12.279966974262678	3.13653119875901	168
384	Pickup	6.005587396425021	-12.277966974262677	3.13653119875901	167
385	Pickup	5.5495873964250215	-12.275966974262676	3.13653119875901	166
386	Pickup	5.093587396425021	-12.273966974262677	3.13653119875901	165
387	Pickup	4.637587396425022	-12.270966974262677	3.13653119875901	164
388	Pickup	4.181587396425021	-12.268966974262677	3.13653119875901	163
389	Pickup	3.725587396425021	-12.266966974262678	3.13653119875901	162
390	Pickup	3.269587396425021	-12.263966974262678	3.13653119875901	161
391	Pickup	2.8145873964250208	-12.261966974262677	3.13653119875901	160
392	Pickup	2.358587396425021	-12.259966974262676	3.13653119875901	159
393	Pickup	1.9025873964250208	-12.257966974262677	3.13653119875901	158
394	Pickup	1.4465873964250209	-12.254966974262677	3.13653119875901	157
395	Pickup	0.990587396425021	-12.252966974262677	3.13653119875901	156
396	Pickup	0.534587396425021	-12.250966974262678	3.13653119875901	155
397	Pickup	0.07858739642502094	-12.247966974262678	3.13653119875901	154
398	Pickup	-0.3764126035749791	-12.245966974262677	3.13653119875901	153
399	Entering	17.689572025462834	-12.336040694126451	3.13653119875901	\N
400	Exiting	-1.5763972326127915	-12.239893254398902	3.13653119875901	\N
401	Pickup	0.11450438087040304	-12.049467525946213	-0.007155849933176751	191
402	Pickup	0.568504380870403	-12.052467525946213	-0.007155849933176751	192
403	Pickup	1.022504380870403	-12.055467525946213	-0.007155849933176751	193
404	Pickup	1.476504380870403	-12.058467525946213	-0.007155849933176751	194
405	Pickup	1.930504380870403	-12.062467525946213	-0.007155849933176751	195
406	Pickup	2.3845043808704034	-12.065467525946213	-0.007155849933176751	196
407	Pickup	2.838504380870403	-12.068467525946213	-0.007155849933176751	197
408	Pickup	3.2925043808704033	-12.072467525946212	-0.007155849933176751	198
409	Pickup	3.746504380870403	-12.075467525946213	-0.007155849933176751	199
410	Pickup	4.200504380870403	-12.078467525946213	-0.007155849933176751	200
411	Pickup	4.654504380870403	-12.081467525946213	-0.007155849933176751	201
412	Pickup	5.108504380870404	-12.085467525946212	-0.007155849933176751	202
413	Pickup	5.562504380870403	-12.088467525946212	-0.007155849933176751	203
414	Pickup	6.016504380870403	-12.091467525946213	-0.007155849933176751	204
415	Pickup	6.470504380870403	-12.095467525946212	-0.007155849933176751	205
416	Pickup	6.924504380870403	-12.098467525946212	-0.007155849933176751	206
417	Pickup	7.378504380870403	-12.101467525946212	-0.007155849933176751	207
418	Pickup	7.8315043808704035	-12.104467525946212	-0.007155849933176751	208
419	Pickup	8.285504380870403	-12.108467525946212	-0.007155849933176751	209
420	Pickup	8.739504380870404	-12.111467525946212	-0.007155849933176751	210
421	Pickup	9.193504380870403	-12.114467525946212	-0.007155849933176751	211
422	Pickup	9.647504380870403	-12.118467525946214	-0.007155849933176751	212
423	Pickup	10.101504380870404	-12.121467525946212	-0.007155849933176751	213
424	Pickup	10.555504380870403	-12.124467525946212	-0.007155849933176751	214
425	Pickup	11.009504380870403	-12.127467525946212	-0.007155849933176751	215
426	Pickup	11.463504380870404	-12.131467525946213	-0.007155849933176751	216
427	Pickup	11.917504380870403	-12.134467525946212	-0.007155849933176751	217
428	Pickup	12.371504380870403	-12.137467525946212	-0.007155849933176751	218
429	Pickup	12.825504380870404	-12.140467525946212	-0.007155849933176751	219
430	Pickup	13.279504380870403	-12.144467525946213	-0.007155849933176751	220
870	Via_CW	8.310778948681428	-2.820737797534048	0	\N
431	Pickup	13.733504380870404	-12.147467525946213	-0.007155849933176751	221
432	Pickup	14.187504380870402	-12.150467525946212	-0.007155849933176751	222
433	Pickup	14.641504380870403	-12.154467525946213	-0.007155849933176751	223
434	Pickup	15.095504380870404	-12.157467525946213	-0.007155849933176751	224
435	Pickup	15.549504380870403	-12.160467525946213	-0.007155849933176751	225
436	Pickup	16.003504380870403	-12.163467525946213	-0.007155849933176751	226
437	Pickup	16.4575043808704	-12.167467525946213	-0.007155849933176751	227
438	Pickup	16.9115043808704	-12.170467525946213	-0.007155849933176751	228
439	Entering	-1.0854648955477408	-12.040880579310972	-0.007155849933176751	\N
440	Exiting	18.111473657288546	-12.179054472581454	-0.007155849933176751	\N
441	Pickup	16.494495619129594	-14.167532474053788	3.134436803656616	228
442	Pickup	16.040495619129594	-14.164532474053788	3.134436803656616	227
443	Pickup	15.586495619129597	-14.160532474053788	3.134436803656616	226
444	Pickup	15.132495619129596	-14.157532474053788	3.134436803656616	225
445	Pickup	14.678495619129597	-14.154532474053788	3.134436803656616	224
446	Pickup	14.224495619129597	-14.151532474053788	3.134436803656616	223
447	Pickup	13.770495619129596	-14.147532474053786	3.134436803656616	222
448	Pickup	13.316495619129597	-14.144532474053788	3.134436803656616	221
449	Pickup	12.862495619129596	-14.141532474053788	3.134436803656616	220
450	Pickup	12.408495619129598	-14.137532474053787	3.134436803656616	219
451	Pickup	11.954495619129597	-14.134532474053787	3.134436803656616	218
452	Pickup	11.500495619129596	-14.131532474053786	3.134436803656616	217
453	Pickup	11.046495619129598	-14.128532474053788	3.134436803656616	216
454	Pickup	10.592495619129597	-14.124532474053787	3.134436803656616	215
455	Pickup	10.138495619129596	-14.121532474053787	3.134436803656616	214
456	Pickup	9.684495619129597	-14.118532474053787	3.134436803656616	213
457	Pickup	9.230495619129597	-14.115532474053788	3.134436803656616	212
458	Pickup	8.776495619129596	-14.111532474053787	3.134436803656616	211
459	Pickup	8.322495619129597	-14.108532474053787	3.134436803656616	210
460	Pickup	7.868495619129597	-14.105532474053787	3.134436803656616	209
461	Pickup	7.414495619129597	-14.101532474053787	3.134436803656616	208
462	Pickup	6.961495619129597	-14.098532474053787	3.134436803656616	207
463	Pickup	6.507495619129597	-14.095532474053787	3.134436803656616	206
464	Pickup	6.053495619129596	-14.092532474053787	3.134436803656616	205
465	Pickup	5.599495619129597	-14.088532474053787	3.134436803656616	204
466	Pickup	5.145495619129597	-14.085532474053787	3.134436803656616	203
467	Pickup	4.691495619129597	-14.082532474053787	3.134436803656616	202
468	Pickup	4.2374956191295965	-14.078532474053787	3.134436803656616	201
469	Pickup	3.7834956191295963	-14.075532474053787	3.134436803656616	200
470	Pickup	3.329495619129596	-14.072532474053787	3.134436803656616	199
471	Pickup	2.8754956191295964	-14.069532474053787	3.134436803656616	198
472	Pickup	2.421495619129596	-14.065532474053787	3.134436803656616	197
473	Pickup	1.9674956191295965	-14.062532474053787	3.134436803656616	196
474	Pickup	1.5134956191295963	-14.059532474053787	3.134436803656616	195
475	Pickup	1.0594956191295963	-14.055532474053788	3.134436803656616	194
476	Pickup	0.6054956191295964	-14.052532474053788	3.134436803656616	193
477	Pickup	0.1514956191295964	-14.049532474053787	3.134436803656616	192
478	Pickup	-0.3025043808704036	-14.046532474053787	3.134436803656616	191
479	Entering	17.69446489554774	-14.176119420689028	3.134436803656616	\N
480	Exiting	-1.5024736572885473	-14.037945527418547	3.134436803656616	\N
481	Pickup	0.07179660977509153	-13.98649606282601	-0.0024434609527920616	229
482	Pickup	0.5267966097750916	-13.98749606282601	-0.0024434609527920616	230
483	Pickup	0.9817966097750915	-13.988496062826009	-0.0024434609527920616	231
484	Pickup	1.4367966097750917	-13.98949606282601	-0.0024434609527920616	232
485	Pickup	1.8917966097750916	-13.99049606282601	-0.0024434609527920616	233
486	Pickup	2.346796609775091	-13.991496062826009	-0.0024434609527920616	234
487	Pickup	2.8017966097750913	-13.99249606282601	-0.0024434609527920616	235
488	Pickup	3.2567966097750913	-13.99349606282601	-0.0024434609527920616	236
489	Pickup	3.7117966097750914	-13.995496062826009	-0.0024434609527920616	237
490	Pickup	4.1667966097750915	-13.99649606282601	-0.0024434609527920616	238
491	Pickup	4.6217966097750915	-13.99749606282601	-0.0024434609527920616	239
492	Pickup	5.076796609775092	-13.998496062826009	-0.0024434609527920616	240
493	Pickup	5.531796609775092	-13.99949606282601	-0.0024434609527920616	241
494	Pickup	5.986796609775092	-14.00049606282601	-0.0024434609527920616	242
495	Pickup	6.441796609775092	-14.001496062826009	-0.0024434609527920616	243
496	Pickup	6.896796609775091	-14.00249606282601	-0.0024434609527920616	244
497	Pickup	7.351796609775091	-14.004496062826009	-0.0024434609527920616	245
498	Pickup	7.806796609775091	-14.00549606282601	-0.0024434609527920616	246
499	Pickup	8.261796609775091	-14.00649606282601	-0.0024434609527920616	247
500	Pickup	8.716796609775091	-14.007496062826009	-0.0024434609527920616	248
501	Pickup	9.171796609775091	-14.008496062826008	-0.0024434609527920616	249
502	Pickup	9.626796609775091	-14.00949606282601	-0.0024434609527920616	250
503	Pickup	10.081796609775092	-14.01049606282601	-0.0024434609527920616	251
504	Pickup	10.536796609775092	-14.011496062826009	-0.0024434609527920616	252
505	Pickup	10.991796609775092	-14.01249606282601	-0.0024434609527920616	253
506	Pickup	11.446796609775092	-14.014496062826009	-0.0024434609527920616	254
507	Pickup	11.901796609775092	-14.01549606282601	-0.0024434609527920616	255
508	Pickup	12.356796609775092	-14.01649606282601	-0.0024434609527920616	256
509	Pickup	12.811796609775092	-14.017496062826009	-0.0024434609527920616	257
510	Pickup	13.266796609775092	-14.01849606282601	-0.0024434609527920616	258
511	Pickup	13.721796609775092	-14.01949606282601	-0.0024434609527920616	259
512	Pickup	14.176796609775092	-14.020496062826009	-0.0024434609527920616	260
513	Pickup	14.631796609775092	-14.02149606282601	-0.0024434609527920616	261
514	Pickup	15.08679660977509	-14.023496062826009	-0.0024434609527920616	262
515	Pickup	15.54179660977509	-14.024496062826008	-0.0024434609527920616	263
516	Pickup	15.99679660977509	-14.02549606282601	-0.0024434609527920616	264
517	Pickup	16.451796609775094	-14.02649606282601	-0.0024434609527920616	265
518	Pickup	16.906796609775093	-14.027496062826009	-0.0024434609527920616	266
519	Entering	-1.1281998079258342	-13.983563912600395	-0.0024434609527920616	\N
520	Exiting	18.10679302747602	-14.030428213051623	-0.0024434609527920616	\N
521	Pickup	16.499203390224906	-16.026503937173988	3.139149192637001	266
522	Pickup	16.044203390224908	-16.02550393717399	3.139149192637001	265
523	Pickup	15.589203390224908	-16.02450393717399	3.139149192637001	264
524	Pickup	15.134203390224908	-16.023503937173988	3.139149192637001	263
525	Pickup	14.679203390224908	-16.02250393717399	3.139149192637001	262
526	Pickup	14.22420339022491	-16.02050393717399	3.139149192637001	261
527	Pickup	13.76920339022491	-16.01950393717399	3.139149192637001	260
528	Pickup	13.31420339022491	-16.01850393717399	3.139149192637001	259
529	Pickup	12.85920339022491	-16.01750393717399	3.139149192637001	258
530	Pickup	12.40420339022491	-16.01650393717399	3.139149192637001	257
531	Pickup	11.949203390224909	-16.01550393717399	3.139149192637001	256
532	Pickup	11.494203390224909	-16.01450393717399	3.139149192637001	255
533	Pickup	11.039203390224909	-16.01350393717399	3.139149192637001	254
534	Pickup	10.584203390224909	-16.01150393717399	3.139149192637001	253
535	Pickup	10.129203390224909	-16.01050393717399	3.139149192637001	252
536	Pickup	9.674203390224909	-16.00950393717399	3.139149192637001	251
537	Pickup	9.219203390224909	-16.00850393717399	3.139149192637001	250
538	Pickup	8.764203390224909	-16.00750393717399	3.139149192637001	249
539	Pickup	8.309203390224909	-16.00650393717399	3.139149192637001	248
540	Pickup	7.854203390224908	-16.00550393717399	3.139149192637001	247
541	Pickup	7.399203390224908	-16.00450393717399	3.139149192637001	246
542	Pickup	6.944203390224908	-16.00350393717399	3.139149192637001	245
543	Pickup	6.489203390224908	-16.00150393717399	3.139149192637001	244
544	Pickup	6.034203390224909	-16.00050393717399	3.139149192637001	243
545	Pickup	5.579203390224909	-15.99950393717399	3.139149192637001	242
546	Pickup	5.124203390224909	-15.998503937173991	3.139149192637001	241
547	Pickup	4.669203390224909	-15.99750393717399	3.139149192637001	240
548	Pickup	4.214203390224909	-15.99650393717399	3.139149192637001	239
549	Pickup	3.7592033902249087	-15.995503937173991	3.139149192637001	238
550	Pickup	3.3042033902249086	-15.99450393717399	3.139149192637001	237
551	Pickup	2.8492033902249085	-15.992503937173991	3.139149192637001	236
552	Pickup	2.3942033902249085	-15.991503937173992	3.139149192637001	235
553	Pickup	1.9392033902249084	-15.99050393717399	3.139149192637001	234
554	Pickup	1.4842033902249083	-15.98950393717399	3.139149192637001	233
555	Pickup	1.0292033902249087	-15.988503937173991	3.139149192637001	232
556	Pickup	0.5742033902249085	-15.98750393717399	3.139149192637001	231
557	Pickup	0.1192033902249085	-15.98650393717399	3.139149192637001	230
558	Pickup	-0.3357966097750915	-15.985503937173991	3.139149192637001	229
559	Entering	17.699199807925833	-16.029436087399603	3.139149192637001	\N
560	Exiting	-1.5357930274760172	-15.982571786948377	3.139149192637001	\N
561	Pickup	0.1335952435636033	-15.716906406207055	-0.009250245035569947	267
562	Pickup	0.5855952435636033	-15.720906406207057	-0.009250245035569947	268
563	Pickup	1.0375952435636033	-15.724906406207054	-0.009250245035569947	269
564	Pickup	1.4895952435636033	-15.728906406207056	-0.009250245035569947	270
565	Pickup	1.9415952435636035	-15.732906406207057	-0.009250245035569947	271
566	Pickup	2.393595243563603	-15.736906406207055	-0.009250245035569947	272
567	Pickup	2.845595243563603	-15.741906406207054	-0.009250245035569947	273
568	Pickup	3.2975952435636033	-15.745906406207055	-0.009250245035569947	274
569	Pickup	3.748595243563603	-15.749906406207057	-0.009250245035569947	275
570	Pickup	4.200595243563604	-15.753906406207054	-0.009250245035569947	276
571	Pickup	4.652595243563604	-15.757906406207056	-0.009250245035569947	277
572	Pickup	5.104595243563604	-15.762906406207055	-0.009250245035569947	278
573	Pickup	5.556595243563604	-15.766906406207056	-0.009250245035569947	279
574	Pickup	6.008595243563604	-15.770906406207054	-0.009250245035569947	280
575	Pickup	6.460595243563604	-15.774906406207055	-0.009250245035569947	281
576	Pickup	6.9125952435636036	-15.778906406207057	-0.009250245035569947	282
577	Pickup	7.3645952435636035	-15.782906406207054	-0.009250245035569947	283
578	Pickup	7.8165952435636035	-15.787906406207057	-0.009250245035569947	284
579	Pickup	8.267595243563603	-15.791906406207055	-0.009250245035569947	285
580	Pickup	8.719595243563603	-15.795906406207056	-0.009250245035569947	286
581	Pickup	9.171595243563603	-15.799906406207054	-0.009250245035569947	287
582	Pickup	9.623595243563603	-15.803906406207055	-0.009250245035569947	288
583	Pickup	10.075595243563603	-15.807906406207056	-0.009250245035569947	289
584	Pickup	10.527595243563603	-15.812906406207055	-0.009250245035569947	290
585	Pickup	10.979595243563603	-15.816906406207057	-0.009250245035569947	291
586	Pickup	11.431595243563603	-15.820906406207055	-0.009250245035569947	292
587	Pickup	11.883595243563603	-15.824906406207056	-0.009250245035569947	293
588	Pickup	12.335595243563603	-15.828906406207057	-0.009250245035569947	294
589	Pickup	12.786595243563603	-15.833906406207056	-0.009250245035569947	295
590	Pickup	13.238595243563603	-15.837906406207054	-0.009250245035569947	296
591	Pickup	13.690595243563603	-15.841906406207055	-0.009250245035569947	297
592	Pickup	14.142595243563603	-15.845906406207057	-0.009250245035569947	298
593	Pickup	14.594595243563603	-15.849906406207054	-0.009250245035569947	299
594	Pickup	15.046595243563603	-15.853906406207056	-0.009250245035569947	300
595	Pickup	15.498595243563603	-15.858906406207055	-0.009250245035569947	301
596	Pickup	15.950595243563603	-15.862906406207056	-0.009250245035569947	302
597	Pickup	16.402595243563603	-15.866906406207054	-0.009250245035569947	303
598	Pickup	16.8545952435636	-15.870906406207055	-0.009250245035569947	304
599	Entering	-1.0663534165825506	-15.7058062704669	-0.009250245035569947	\N
600	Exiting	18.054543903709757	-15.882006541947211	-0.009250245035569947	\N
601	Pickup	16.433404756436396	-17.867093593792944	3.132342408554223	304
602	Pickup	15.981404756436396	-17.863093593792943	3.132342408554223	303
603	Pickup	15.529404756436396	-17.859093593792945	3.132342408554223	302
604	Pickup	15.077404756436396	-17.855093593792944	3.132342408554223	301
605	Pickup	14.625404756436396	-17.850093593792945	3.132342408554223	300
606	Pickup	14.173404756436396	-17.846093593792943	3.132342408554223	299
607	Pickup	13.721404756436396	-17.842093593792946	3.132342408554223	298
608	Pickup	13.269404756436396	-17.838093593792944	3.132342408554223	297
609	Pickup	12.817404756436396	-17.834093593792943	3.132342408554223	296
610	Pickup	12.365404756436396	-17.830093593792945	3.132342408554223	295
611	Pickup	11.914404756436396	-17.825093593792946	3.132342408554223	294
612	Pickup	11.462404756436396	-17.821093593792945	3.132342408554223	293
613	Pickup	11.010404756436396	-17.817093593792944	3.132342408554223	292
614	Pickup	10.558404756436396	-17.813093593792946	3.132342408554223	291
615	Pickup	10.106404756436396	-17.809093593792944	3.132342408554223	290
616	Pickup	9.654404756436396	-17.804093593792945	3.132342408554223	289
617	Pickup	9.202404756436396	-17.800093593792944	3.132342408554223	288
618	Pickup	8.750404756436396	-17.796093593792943	3.132342408554223	287
619	Pickup	8.298404756436396	-17.792093593792945	3.132342408554223	286
620	Pickup	7.846404756436397	-17.788093593792944	3.132342408554223	285
621	Pickup	7.395404756436396	-17.784093593792946	3.132342408554223	284
622	Pickup	6.943404756436396	-17.779093593792943	3.132342408554223	283
623	Pickup	6.491404756436396	-17.775093593792946	3.132342408554223	282
624	Pickup	6.039404756436396	-17.771093593792944	3.132342408554223	281
625	Pickup	5.5874047564363964	-17.767093593792943	3.132342408554223	280
626	Pickup	5.1354047564363965	-17.763093593792945	3.132342408554223	279
627	Pickup	4.6834047564363965	-17.759093593792944	3.132342408554223	278
628	Pickup	4.231404756436397	-17.754093593792945	3.132342408554223	277
629	Pickup	3.7794047564363966	-17.750093593792943	3.132342408554223	276
630	Pickup	3.327404756436396	-17.746093593792946	3.132342408554223	275
631	Pickup	2.8764047564363966	-17.742093593792944	3.132342408554223	274
632	Pickup	2.424404756436396	-17.738093593792943	3.132342408554223	273
633	Pickup	1.972404756436396	-17.733093593792944	3.132342408554223	272
634	Pickup	1.5204047564363963	-17.729093593792946	3.132342408554223	271
635	Pickup	1.068404756436396	-17.725093593792945	3.132342408554223	270
636	Pickup	0.6164047564363963	-17.721093593792943	3.132342408554223	269
637	Pickup	0.16440475643639627	-17.717093593792946	3.132342408554223	268
638	Pickup	-0.2875952435636037	-17.713093593792944	3.132342408554223	267
639	Entering	17.63335341658255	-17.878193729533102	3.132342408554223	\N
640	Exiting	-1.4875439037097578	-17.701993458052787	3.132342408554223	\N
641	Pickup	0.10898863938406526	-17.527201425459914	-0.01064650843716541	305
642	Pickup	0.5629886393840653	-17.532201425459917	-0.01064650843716541	306
643	Pickup	1.0169886393840653	-17.53720142545992	-0.01064650843716541	307
644	Pickup	1.4709886393840652	-17.542201425459915	-0.01064650843716541	308
645	Pickup	1.9249886393840654	-17.547201425459917	-0.01064650843716541	309
646	Pickup	2.378988639384065	-17.55220142545992	-0.01064650843716541	310
647	Pickup	2.832988639384065	-17.556201425459918	-0.01064650843716541	311
648	Pickup	3.2869886393840653	-17.56120142545992	-0.01064650843716541	312
649	Pickup	3.740988639384065	-17.566201425459916	-0.01064650843716541	313
650	Pickup	4.194988639384065	-17.57120142545992	-0.01064650843716541	314
651	Pickup	4.648988639384066	-17.57620142545992	-0.01064650843716541	315
652	Pickup	5.1029886393840656	-17.581201425459916	-0.01064650843716541	316
653	Pickup	5.556988639384065	-17.58620142545992	-0.01064650843716541	317
654	Pickup	6.010988639384066	-17.590201425459917	-0.01064650843716541	318
655	Pickup	6.464988639384066	-17.59520142545992	-0.01064650843716541	319
656	Pickup	6.918988639384065	-17.600201425459915	-0.01064650843716541	320
657	Pickup	7.372988639384065	-17.605201425459917	-0.01064650843716541	321
658	Pickup	7.826988639384066	-17.61020142545992	-0.01064650843716541	322
659	Pickup	8.280988639384066	-17.615201425459915	-0.01064650843716541	323
660	Pickup	8.734988639384065	-17.620201425459918	-0.01064650843716541	324
661	Pickup	9.188988639384066	-17.624201425459916	-0.01064650843716541	325
662	Pickup	9.642988639384065	-17.629201425459918	-0.01064650843716541	326
663	Pickup	10.096988639384065	-17.63420142545992	-0.01064650843716541	327
664	Pickup	10.550988639384066	-17.639201425459916	-0.01064650843716541	328
665	Pickup	11.004988639384065	-17.64420142545992	-0.01064650843716541	329
666	Pickup	11.458988639384065	-17.649201425459914	-0.01064650843716541	330
667	Pickup	11.912988639384066	-17.654201425459917	-0.01064650843716541	331
668	Pickup	12.366988639384065	-17.65920142545992	-0.01064650843716541	332
669	Pickup	12.820988639384066	-17.663201425459917	-0.01064650843716541	333
670	Pickup	13.273988639384065	-17.66820142545992	-0.01064650843716541	334
671	Pickup	13.727988639384066	-17.673201425459915	-0.01064650843716541	335
672	Pickup	14.181988639384066	-17.678201425459918	-0.01064650843716541	336
673	Pickup	14.635988639384065	-17.68320142545992	-0.01064650843716541	337
674	Pickup	15.089988639384066	-17.688201425459916	-0.01064650843716541	338
675	Pickup	15.543988639384066	-17.693201425459918	-0.01064650843716541	339
676	Pickup	15.997988639384065	-17.697201425459916	-0.01064650843716541	340
677	Pickup	16.451988639384062	-17.70220142545992	-0.01064650843716541	341
678	Pickup	16.905988639384063	-17.707201425459914	-0.01064650843716541	342
679	Entering	-1.0909433523731806	-17.514425856686337	-0.01064650843716541	\N
680	Exiting	18.10592063114131	-17.71997699423349	-0.01064650843716541	\N
681	Pickup	16.48201136061593	-19.70279857454008	3.1309461451526275	342
682	Pickup	16.02801136061593	-19.69779857454008	3.1309461451526275	341
683	Pickup	15.574011360615934	-19.69279857454008	3.1309461451526275	340
684	Pickup	15.120011360615935	-19.68879857454008	3.1309461451526275	339
685	Pickup	14.666011360615935	-19.68379857454008	3.1309461451526275	338
686	Pickup	14.212011360615934	-19.67879857454008	3.1309461451526275	337
687	Pickup	13.758011360615935	-19.67379857454008	3.1309461451526275	336
688	Pickup	13.304011360615934	-19.66879857454008	3.1309461451526275	335
689	Pickup	12.850011360615934	-19.66379857454008	3.1309461451526275	334
690	Pickup	12.397011360615934	-19.658798574540082	3.1309461451526275	333
691	Pickup	11.943011360615934	-19.65479857454008	3.1309461451526275	332
692	Pickup	11.489011360615935	-19.649798574540082	3.1309461451526275	331
693	Pickup	11.035011360615934	-19.64479857454008	3.1309461451526275	330
694	Pickup	10.581011360615934	-19.63979857454008	3.1309461451526275	329
695	Pickup	10.127011360615935	-19.63479857454008	3.1309461451526275	328
696	Pickup	9.673011360615934	-19.629798574540082	3.1309461451526275	327
697	Pickup	9.219011360615934	-19.62479857454008	3.1309461451526275	326
698	Pickup	8.765011360615935	-19.61979857454008	3.1309461451526275	325
699	Pickup	8.311011360615934	-19.61579857454008	3.1309461451526275	324
700	Pickup	7.857011360615935	-19.61079857454008	3.1309461451526275	323
701	Pickup	7.403011360615935	-19.60579857454008	3.1309461451526275	322
702	Pickup	6.949011360615934	-19.600798574540082	3.1309461451526275	321
703	Pickup	6.495011360615934	-19.59579857454008	3.1309461451526275	320
704	Pickup	6.041011360615935	-19.59079857454008	3.1309461451526275	319
705	Pickup	5.587011360615935	-19.58579857454008	3.1309461451526275	318
706	Pickup	5.133011360615934	-19.58179857454008	3.1309461451526275	317
707	Pickup	4.6790113606159345	-19.57679857454008	3.1309461451526275	316
708	Pickup	4.225011360615935	-19.571798574540082	3.1309461451526275	315
709	Pickup	3.7710113606159346	-19.56679857454008	3.1309461451526275	314
710	Pickup	3.3170113606159344	-19.56179857454008	3.1309461451526275	313
711	Pickup	2.8630113606159346	-19.556798574540082	3.1309461451526275	312
712	Pickup	2.4090113606159345	-19.55179857454008	3.1309461451526275	311
713	Pickup	1.9550113606159343	-19.54779857454008	3.1309461451526275	310
714	Pickup	1.5010113606159345	-19.542798574540083	3.1309461451526275	309
715	Pickup	1.0470113606159344	-19.53779857454008	3.1309461451526275	308
716	Pickup	0.5930113606159344	-19.53279857454008	3.1309461451526275	307
717	Pickup	0.13901136061593436	-19.527798574540082	3.1309461451526275	306
718	Pickup	-0.3149886393840656	-19.52279857454008	3.1309461451526275	305
719	Entering	17.681943352373178	-19.715574143313656	3.1309461451526275	\N
720	Exiting	-1.5149206311413117	-19.510023005766502	3.1309461451526275	\N
721	Pickup	0.12198152193215353	-19.39635849018934	-0.0066322511575784525	343
722	Pickup	0.5779815219321536	-19.39935849018934	-0.0066322511575784525	344
723	Pickup	1.0339815219321535	-19.40235849018934	-0.0066322511575784525	345
724	Pickup	1.4899815219321535	-19.40535849018934	-0.0066322511575784525	346
725	Pickup	1.9459815219321535	-19.40835849018934	-0.0066322511575784525	347
726	Pickup	2.4019815219321536	-19.41135849018934	-0.0066322511575784525	348
727	Pickup	2.8579815219321536	-19.41435849018934	-0.0066322511575784525	349
728	Pickup	3.3139815219321536	-19.41735849018934	-0.0066322511575784525	350
729	Pickup	3.770981521932154	-19.42035849018934	-0.0066322511575784525	351
730	Pickup	4.226981521932154	-19.42335849018934	-0.0066322511575784525	352
731	Pickup	4.682981521932153	-19.42635849018934	-0.0066322511575784525	353
732	Pickup	5.138981521932154	-19.42935849018934	-0.0066322511575784525	354
733	Pickup	5.594981521932153	-19.43235849018934	-0.0066322511575784525	355
734	Pickup	6.050981521932154	-19.43535849018934	-0.0066322511575784525	356
735	Pickup	6.506981521932154	-19.43835849018934	-0.0066322511575784525	357
736	Pickup	6.962981521932154	-19.44135849018934	-0.0066322511575784525	358
737	Pickup	7.418981521932154	-19.44435849018934	-0.0066322511575784525	359
738	Pickup	7.875981521932154	-19.44735849018934	-0.0066322511575784525	360
739	Pickup	8.331981521932153	-19.45035849018934	-0.0066322511575784525	361
740	Pickup	8.787981521932153	-19.45335849018934	-0.0066322511575784525	362
741	Pickup	9.243981521932152	-19.456358490189338	-0.0066322511575784525	363
742	Pickup	9.699981521932154	-19.459358490189338	-0.0066322511575784525	364
743	Pickup	10.155981521932153	-19.462358490189338	-0.0066322511575784525	365
744	Pickup	10.611981521932153	-19.465358490189338	-0.0066322511575784525	366
745	Pickup	11.067981521932152	-19.46835849018934	-0.0066322511575784525	367
746	Pickup	11.524981521932153	-19.47135849018934	-0.0066322511575784525	368
747	Pickup	11.980981521932152	-19.47435849018934	-0.0066322511575784525	369
748	Pickup	12.436981521932152	-19.47735849018934	-0.0066322511575784525	370
749	Pickup	12.892981521932153	-19.48035849018934	-0.0066322511575784525	371
750	Pickup	13.348981521932153	-19.48335849018934	-0.0066322511575784525	372
751	Pickup	13.804981521932152	-19.48635849018934	-0.0066322511575784525	373
752	Pickup	14.260981521932154	-19.48935849018934	-0.0066322511575784525	374
753	Pickup	14.716981521932153	-19.49235849018934	-0.0066322511575784525	375
871	Via_CCW	6.8697537799251105	-2.820737797534048	3.141592653589793	\N
754	Pickup	15.172981521932153	-19.49535849018934	-0.0066322511575784525	376
755	Pickup	15.629981521932153	-19.49835849018934	-0.0066322511575784525	377
756	Pickup	16.085981521932155	-19.50135849018934	-0.0066322511575784525	378
757	Pickup	16.541981521932154	-19.50435849018934	-0.0066322511575784525	379
758	Pickup	16.997981521932154	-19.50735849018934	-0.0066322511575784525	380
759	Entering	-1.0779920861113377	-19.38839984714636	-0.0066322511575784525	\N
760	Exiting	18.197955129975647	-19.51531713323232	-0.0066322511575784525	\N
761	Pickup	16.582018478067845	-21.50464150981066	3.134960402432215	380
762	Pickup	16.126018478067845	-21.50164150981066	3.134960402432215	379
763	Pickup	15.670018478067847	-21.49864150981066	3.134960402432215	378
764	Pickup	15.214018478067848	-21.49564150981066	3.134960402432215	377
765	Pickup	14.757018478067847	-21.49264150981066	3.134960402432215	376
766	Pickup	14.301018478067848	-21.48964150981066	3.134960402432215	375
767	Pickup	13.845018478067848	-21.48664150981066	3.134960402432215	374
768	Pickup	13.389018478067847	-21.48364150981066	3.134960402432215	373
769	Pickup	12.933018478067847	-21.48064150981066	3.134960402432215	372
770	Pickup	12.477018478067848	-21.47764150981066	3.134960402432215	371
771	Pickup	12.021018478067846	-21.47464150981066	3.134960402432215	370
772	Pickup	11.565018478067847	-21.47164150981066	3.134960402432215	369
773	Pickup	11.109018478067847	-21.46864150981066	3.134960402432215	368
774	Pickup	10.652018478067847	-21.46564150981066	3.134960402432215	367
775	Pickup	10.196018478067847	-21.46264150981066	3.134960402432215	366
776	Pickup	9.740018478067848	-21.45964150981066	3.134960402432215	365
777	Pickup	9.284018478067848	-21.45664150981066	3.134960402432215	364
778	Pickup	8.828018478067847	-21.45364150981066	3.134960402432215	363
779	Pickup	8.372018478067847	-21.450641509810662	3.134960402432215	362
780	Pickup	7.916018478067848	-21.447641509810662	3.134960402432215	361
781	Pickup	7.4600184780678465	-21.444641509810662	3.134960402432215	360
782	Pickup	7.0030184780678475	-21.441641509810662	3.134960402432215	359
783	Pickup	6.547018478067846	-21.43864150981066	3.134960402432215	358
784	Pickup	6.091018478067847	-21.43564150981066	3.134960402432215	357
785	Pickup	5.635018478067847	-21.43264150981066	3.134960402432215	356
786	Pickup	5.179018478067846	-21.42964150981066	3.134960402432215	355
787	Pickup	4.723018478067846	-21.42664150981066	3.134960402432215	354
788	Pickup	4.267018478067847	-21.42364150981066	3.134960402432215	353
789	Pickup	3.811018478067847	-21.42064150981066	3.134960402432215	352
790	Pickup	3.355018478067847	-21.41764150981066	3.134960402432215	351
791	Pickup	2.8980184780678466	-21.41464150981066	3.134960402432215	350
792	Pickup	2.4420184780678467	-21.41164150981066	3.134960402432215	349
793	Pickup	1.9860184780678467	-21.40864150981066	3.134960402432215	348
794	Pickup	1.5300184780678467	-21.40564150981066	3.134960402432215	347
795	Pickup	1.0740184780678468	-21.40264150981066	3.134960402432215	346
796	Pickup	0.6180184780678467	-21.39964150981066	3.134960402432215	345
797	Pickup	0.16201847806784675	-21.39664150981066	3.134960402432215	344
798	Pickup	-0.2939815219321532	-21.39364150981066	3.134960402432215	343
799	Entering	17.781992086111337	-21.51260015285364	3.134960402432215	\N
800	Exiting	-1.4939551299756446	-21.38568286676768	3.134960402432215	\N
801	Via_CCW	-1.7763972326127915	-21.71260015285364	0	\N
802	Via_CW	-1.7763972326127915	-21.71260015285364	-3.141592653589793	\N
803	Via_CCW	-0.33537206385647456	-21.71260015285364	0	\N
804	Via_CW	-0.33537206385647456	-21.71260015285364	-3.141592653589793	\N
805	Via_CCW	1.1056531048998424	-21.71260015285364	0	\N
806	Via_CW	1.1056531048998424	-21.71260015285364	-3.141592653589793	\N
807	Via_CCW	2.546678273656159	-21.71260015285364	0	\N
808	Via_CW	2.546678273656159	-21.71260015285364	-3.141592653589793	\N
809	Via_CCW	3.987703442412476	-21.71260015285364	0	\N
810	Via_CW	3.987703442412476	-21.71260015285364	-3.141592653589793	\N
811	Via_CCW	5.428728611168793	-21.71260015285364	0	\N
812	Via_CW	5.428728611168793	-21.71260015285364	-3.141592653589793	\N
813	Via_CCW	6.86975377992511	-21.71260015285364	0	\N
814	Via_CW	6.86975377992511	-21.71260015285364	-3.141592653589793	\N
815	Via_CCW	8.310778948681428	-21.71260015285364	0	\N
816	Via_CW	8.310778948681428	-21.71260015285364	-3.141592653589793	\N
817	Via_CCW	9.751804117437743	-21.71260015285364	0	\N
818	Via_CW	9.751804117437743	-21.71260015285364	-3.141592653589793	\N
819	Via_CCW	11.192829286194062	-21.71260015285364	0	\N
820	Via_CW	11.192829286194062	-21.71260015285364	-3.141592653589793	\N
821	Via_CCW	12.633854454950377	-21.71260015285364	0	\N
822	Via_CW	12.633854454950377	-21.71260015285364	-3.141592653589793	\N
823	Via_CCW	14.074879623706696	-21.71260015285364	0	\N
824	Via_CW	14.074879623706696	-21.71260015285364	-3.141592653589793	\N
825	Via_CCW	15.515904792463012	-21.71260015285364	0	\N
826	Via_CW	15.515904792463012	-21.71260015285364	-3.141592653589793	\N
827	Via_CCW	16.95692996121933	-21.71260015285364	0	\N
828	Via_CW	16.95692996121933	-21.71260015285364	-3.141592653589793	\N
829	Via_CCW	18.397955129975646	-21.71260015285364	1.5707963267948966	\N
830	Via_CW	18.397955129975646	-21.71260015285364	-1.5707963267948966	\N
831	Via_CCW	18.397955129975646	-20.25937997167521	1.5707963267948966	\N
832	Via_CW	18.397955129975646	-20.25937997167521	-1.5707963267948966	\N
833	Via_CCW	18.397955129975646	-18.80615979049678	1.5707963267948966	\N
834	Via_CW	18.397955129975646	-18.80615979049678	-1.5707963267948966	\N
835	Via_CCW	18.397955129975646	-17.352939609318348	1.5707963267948966	\N
836	Via_CW	18.397955129975646	-17.352939609318348	-1.5707963267948966	\N
837	Via_CCW	18.397955129975646	-15.899719428139917	1.5707963267948966	\N
838	Via_CW	18.397955129975646	-15.899719428139917	-1.5707963267948966	\N
839	Via_CCW	18.397955129975646	-14.446499246961487	1.5707963267948966	\N
840	Via_CW	18.397955129975646	-14.446499246961487	-1.5707963267948966	\N
841	Via_CCW	18.397955129975646	-12.993279065783058	1.5707963267948966	\N
842	Via_CW	18.397955129975646	-12.993279065783058	-1.5707963267948966	\N
843	Via_CCW	18.397955129975646	-11.540058884604628	1.5707963267948966	\N
844	Via_CW	18.397955129975646	-11.540058884604628	-1.5707963267948966	\N
845	Via_CCW	18.397955129975646	-10.086838703426197	1.5707963267948966	\N
846	Via_CW	18.397955129975646	-10.086838703426197	-1.5707963267948966	\N
847	Via_CCW	18.397955129975646	-8.633618522247767	1.5707963267948966	\N
848	Via_CW	18.397955129975646	-8.633618522247767	-1.5707963267948966	\N
849	Via_CCW	18.397955129975646	-7.180398341069337	1.5707963267948966	\N
850	Via_CW	18.397955129975646	-7.180398341069337	-1.5707963267948966	\N
851	Via_CCW	18.397955129975646	-5.727178159890906	1.5707963267948966	\N
852	Via_CW	18.397955129975646	-5.727178159890906	-1.5707963267948966	\N
853	Via_CCW	18.397955129975646	-4.2739579787124775	1.5707963267948966	\N
854	Via_CW	18.397955129975646	-4.2739579787124775	-1.5707963267948966	\N
855	Via_CCW	18.397955129975646	-2.820737797534048	3.141592653589793	\N
856	Via_CW	18.397955129975646	-2.820737797534048	0	\N
857	Via_CCW	16.95692996121933	-2.820737797534048	3.141592653589793	\N
858	Via_CW	16.95692996121933	-2.820737797534048	0	\N
859	Via_CCW	15.515904792463012	-2.820737797534048	3.141592653589793	\N
860	Via_CW	15.515904792463012	-2.820737797534048	0	\N
861	Via_CCW	14.074879623706696	-2.820737797534048	3.141592653589793	\N
862	Via_CW	14.074879623706696	-2.820737797534048	0	\N
863	Via_CCW	12.633854454950377	-2.820737797534048	3.141592653589793	\N
864	Via_CW	12.633854454950377	-2.820737797534048	0	\N
865	Via_CCW	11.192829286194062	-2.820737797534048	3.141592653589793	\N
866	Via_CW	11.192829286194062	-2.820737797534048	0	\N
867	Via_CCW	9.751804117437745	-2.820737797534048	3.141592653589793	\N
868	Via_CW	9.751804117437745	-2.820737797534048	0	\N
872	Via_CW	6.8697537799251105	-2.820737797534048	0	\N
873	Via_CCW	5.428728611168793	-2.820737797534048	3.141592653589793	\N
874	Via_CW	5.428728611168793	-2.820737797534048	0	\N
875	Via_CCW	3.987703442412476	-2.820737797534048	3.141592653589793	\N
876	Via_CW	3.987703442412476	-2.820737797534048	0	\N
877	Via_CCW	2.546678273656159	-2.820737797534048	3.141592653589793	\N
878	Via_CW	2.546678273656159	-2.820737797534048	0	\N
879	Via_CCW	1.1056531048998437	-2.820737797534048	3.141592653589793	\N
880	Via_CW	1.1056531048998437	-2.820737797534048	0	\N
881	Via_CCW	-0.3353720638564752	-2.820737797534048	3.141592653589793	\N
882	Via_CW	-0.3353720638564752	-2.820737797534048	0	\N
883	Via_CCW	-1.7763972326127915	-2.820737797534048	-1.5707963267948966	\N
884	Via_CW	-1.7763972326127915	-2.820737797534048	1.5707963267948966	\N
885	Via_CCW	-1.7763972326127915	-4.273957978712478	-1.5707963267948966	\N
886	Via_CW	-1.7763972326127915	-4.273957978712478	1.5707963267948966	\N
887	Via_CCW	-1.7763972326127915	-5.727178159890908	-1.5707963267948966	\N
888	Via_CW	-1.7763972326127915	-5.727178159890908	1.5707963267948966	\N
889	Via_CCW	-1.7763972326127915	-7.180398341069338	-1.5707963267948966	\N
890	Via_CW	-1.7763972326127915	-7.180398341069338	1.5707963267948966	\N
891	Via_CCW	-1.7763972326127915	-8.633618522247769	-1.5707963267948966	\N
892	Via_CW	-1.7763972326127915	-8.633618522247769	1.5707963267948966	\N
893	Via_CCW	-1.7763972326127915	-10.0868387034262	-1.5707963267948966	\N
894	Via_CW	-1.7763972326127915	-10.0868387034262	1.5707963267948966	\N
895	Via_CCW	-1.7763972326127915	-11.54005888460463	-1.5707963267948966	\N
896	Via_CW	-1.7763972326127915	-11.54005888460463	1.5707963267948966	\N
897	Via_CCW	-1.7763972326127915	-12.99327906578306	-1.5707963267948966	\N
898	Via_CW	-1.7763972326127915	-12.99327906578306	1.5707963267948966	\N
899	Via_CCW	-1.7763972326127915	-14.44649924696149	-1.5707963267948966	\N
900	Via_CW	-1.7763972326127915	-14.44649924696149	1.5707963267948966	\N
901	Via_CCW	-1.7763972326127915	-15.899719428139921	-1.5707963267948966	\N
902	Via_CW	-1.7763972326127915	-15.899719428139921	1.5707963267948966	\N
903	Via_CCW	-1.7763972326127915	-17.35293960931835	-1.5707963267948966	\N
904	Via_CW	-1.7763972326127915	-17.35293960931835	1.5707963267948966	\N
905	Via_CCW	-1.7763972326127915	-18.806159790496782	-1.5707963267948966	\N
906	Via_CW	-1.7763972326127915	-18.806159790496782	1.5707963267948966	\N
907	Via_CCW	-1.7763972326127915	-20.25937997167521	-1.5707963267948966	\N
908	Via_CW	-1.7763972326127915	-20.25937997167521	1.5707963267948966	\N
909	Staging	-3.3437286319817705	1.702344961516484	-0.057203673205103354	\N
\.


--
-- Data for Name: harvest_state; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.harvest_state (object_id, row_id, status, lane_id, pickup_node_id, assigned_robot_id, job_id, attempts, last_error, assigned_at, harvested_at, updated_at) FROM stdin;
29	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
30	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
32	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
33	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
34	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
72	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
35	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
36	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
37	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
38	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
73	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
74	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
75	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
76	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
77	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
78	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
79	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
80	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
81	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
82	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
83	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
84	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
85	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
86	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
87	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
88	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
89	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
90	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
91	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
92	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
93	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
94	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
95	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
96	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
97	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
98	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
99	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
100	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
101	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
102	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
103	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
104	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
105	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
106	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
107	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
108	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
109	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
110	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
111	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
112	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
113	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
114	3	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
115	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
116	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
117	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
118	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
119	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
120	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
41	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
2	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
42	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
44	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
4	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
5	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
46	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
47	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
48	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
8	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
50	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
51	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
52	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
11	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
54	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
55	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
13	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
14	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
58	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
15	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
16	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
17	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
61	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
62	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
63	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
64	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
65	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
66	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
22	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
23	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
69	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
25	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
70	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
27	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
121	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
122	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
123	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
124	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
125	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
126	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
127	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
128	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
129	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
130	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
131	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
132	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
133	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
134	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
135	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
136	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
137	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
138	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
139	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
140	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
141	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
142	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
143	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
144	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
145	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
146	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
147	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
148	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
149	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
150	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
151	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
152	4	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
153	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
154	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
155	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
156	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
157	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
158	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
159	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
160	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
161	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
162	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
163	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
164	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
165	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
166	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
167	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
168	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
169	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
170	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
171	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
172	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
173	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
174	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
175	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
176	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
177	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
178	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
179	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
180	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
181	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
182	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
183	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
184	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
185	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
186	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
187	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
188	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
189	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
190	5	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
191	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
192	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
193	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
194	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
195	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
196	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
197	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
198	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
199	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
200	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
201	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
202	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
203	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
204	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
205	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
206	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
207	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
208	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
209	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
210	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
211	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
212	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
213	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
214	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
215	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
216	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
217	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
218	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
219	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
220	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
221	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
222	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
223	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
224	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
225	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
226	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
227	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
228	6	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
229	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
230	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
231	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
232	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
233	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
234	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
235	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
236	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
237	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
238	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
239	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
240	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
241	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
242	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
243	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
244	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
245	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
246	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
247	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
248	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
249	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
250	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
251	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
252	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
253	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
254	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
255	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
256	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
257	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
258	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
259	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
260	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
261	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
262	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
263	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
264	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
265	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
266	7	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
267	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
268	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
269	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
270	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
271	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
272	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
273	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
274	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
275	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
276	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
277	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
278	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
279	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
280	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
281	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
282	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
283	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
284	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
285	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
286	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
287	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
288	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
289	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
290	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
291	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
292	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
293	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
294	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
295	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
296	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
297	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
298	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
299	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
300	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
301	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
302	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
303	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
304	8	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
305	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
306	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
307	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
308	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
309	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
310	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
311	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
312	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
313	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
314	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
315	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
316	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
317	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
318	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
319	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
320	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
321	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
322	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
323	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
324	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
325	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
326	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
327	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
328	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
329	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
330	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
331	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
332	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
333	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
334	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
335	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
336	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
337	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
338	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
339	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
340	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
341	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
342	9	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
343	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
344	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
345	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
346	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
347	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
348	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
349	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
350	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
351	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
352	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
353	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
354	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
355	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
356	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
357	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
358	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
359	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
360	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
361	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
362	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
363	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
364	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
365	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
366	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
367	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
368	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
369	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
370	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
371	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
372	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
373	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
374	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
375	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
376	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
377	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
378	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
379	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
380	10	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
28	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
31	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
39	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
1	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
40	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
3	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
43	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
45	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
6	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
7	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
49	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
9	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
10	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
53	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
12	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
56	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
57	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
59	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
60	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
18	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
19	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
20	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
21	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
67	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
71	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
68	2	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
24	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
26	1	pending	\N	\N	\N	\N	0	\N	\N	\N	2026-10-26 16:54:33.416154+00
\.


--
-- Data for Name: object_data; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.object_data (object_id, row_id, x_coord, y_coord, orientation_deg) FROM stdin;
1	1	0.288	-4.023	-0.13
2	1	0.732	-4.024	-0.13
3	1	1.176	-4.025	-0.13
4	1	1.62	-4.026	-0.13
5	1	2.064	-4.027	-0.13
6	1	2.508	-4.028	-0.13
7	1	2.952	-4.029	-0.13
8	1	3.396	-4.03	-0.13
9	1	3.84	-4.031	-0.13
10	1	4.284	-4.032	-0.13
11	1	4.728	-4.033	-0.13
12	1	5.172	-4.034	-0.13
13	1	5.616	-4.034	-0.13
14	1	6.06	-4.035	-0.13
15	1	6.505	-4.036	-0.13
16	1	6.949	-4.037	-0.13
17	1	7.393	-4.038	-0.13
18	1	7.837	-4.039	-0.13
19	1	8.281	-4.04	-0.13
20	1	8.725	-4.041	-0.13
21	1	9.169	-4.042	-0.13
22	1	9.613	-4.043	-0.13
23	1	10.057	-4.044	-0.13
24	1	10.501	-4.045	-0.13
25	1	10.945	-4.046	-0.13
26	1	11.389	-4.047	-0.13
27	1	11.833	-4.048	-0.13
28	1	12.277	-4.049	-0.13
29	1	12.721	-4.05	-0.13
30	1	13.165	-4.051	-0.13
31	1	13.61	-4.052	-0.13
32	1	14.054	-4.053	-0.13
33	1	14.498	-4.054	-0.13
34	1	14.942	-4.055	-0.13
35	1	15.386	-4.056	-0.13
36	1	15.83	-4.057	-0.13
37	1	16.274	-4.058	-0.13
38	1	16.718	-4.059	-0.13
39	2	-0.143	-5.807	-0.36
40	2	0.311	-5.81	-0.36
41	2	0.766	-5.813	-0.36
42	2	1.22	-5.816	-0.36
43	2	1.675	-5.819	-0.36
44	2	2.129	-5.822	-0.36
45	2	2.584	-5.824	-0.36
46	2	3.038	-5.827	-0.36
47	2	3.493	-5.83	-0.36
48	2	3.947	-5.833	-0.36
49	2	4.401	-5.836	-0.36
50	2	4.856	-5.839	-0.36
51	2	5.31	-5.842	-0.36
52	2	5.765	-5.844	-0.36
53	2	6.219	-5.847	-0.36
54	2	6.674	-5.85	-0.36
55	2	7.128	-5.853	-0.36
56	2	7.583	-5.856	-0.36
57	2	8.037	-5.859	-0.36
58	2	8.491	-5.862	-0.36
59	2	8.946	-5.864	-0.36
60	2	9.4	-5.867	-0.36
61	2	9.855	-5.87	-0.36
62	2	10.309	-5.873	-0.36
63	2	10.764	-5.876	-0.36
64	2	11.218	-5.879	-0.36
65	2	11.672	-5.881	-0.36
66	2	12.127	-5.884	-0.36
67	2	12.581	-5.887	-0.36
68	2	13.036	-5.89	-0.36
69	2	13.49	-5.893	-0.36
70	2	13.945	-5.896	-0.36
71	2	14.399	-5.899	-0.36
72	2	14.854	-5.901	-0.36
73	2	15.308	-5.904	-0.36
74	2	15.762	-5.907	-0.36
75	2	16.217	-5.91	-0.36
76	2	16.671	-5.913	-0.36
77	3	-0.175	-7.72	0.07
78	3	0.282	-7.719	0.07
79	3	0.739	-7.719	0.07
80	3	1.196	-7.718	0.07
81	3	1.653	-7.717	0.07
82	3	2.11	-7.717	0.07
83	3	2.567	-7.716	0.07
84	3	3.024	-7.716	0.07
85	3	3.481	-7.715	0.07
86	3	3.938	-7.715	0.07
87	3	4.395	-7.714	0.07
88	3	4.852	-7.713	0.07
89	3	5.309	-7.713	0.07
90	3	5.766	-7.712	0.07
91	3	6.223	-7.712	0.07
92	3	6.68	-7.711	0.07
93	3	7.137	-7.711	0.07
94	3	7.594	-7.71	0.07
95	3	8.051	-7.709	0.07
96	3	8.508	-7.709	0.07
97	3	8.965	-7.708	0.07
98	3	9.422	-7.708	0.07
99	3	9.879	-7.707	0.07
100	3	10.335	-7.707	0.07
101	3	10.792	-7.706	0.07
102	3	11.249	-7.705	0.07
103	3	11.706	-7.705	0.07
104	3	12.163	-7.704	0.07
105	3	12.62	-7.704	0.07
106	3	13.077	-7.703	0.07
107	3	13.534	-7.702	0.07
108	3	13.991	-7.702	0.07
109	3	14.448	-7.701	0.07
110	3	14.905	-7.701	0.07
111	3	15.362	-7.7	0.07
112	3	15.819	-7.7	0.07
113	3	16.276	-7.699	0.07
114	3	16.733	-7.698	0.07
115	4	-0.094	-9.424	-0.16
116	4	0.361	-9.425	-0.16
117	4	0.816	-9.426	-0.16
118	4	1.272	-9.428	-0.16
119	4	1.727	-9.429	-0.16
120	4	2.183	-9.43	-0.16
121	4	2.638	-9.432	-0.16
122	4	3.093	-9.433	-0.16
123	4	3.549	-9.434	-0.16
124	4	4.004	-9.435	-0.16
125	4	4.46	-9.437	-0.16
126	4	4.915	-9.438	-0.16
127	4	5.37	-9.439	-0.16
128	4	5.826	-9.441	-0.16
129	4	6.281	-9.442	-0.16
130	4	6.737	-9.443	-0.16
131	4	7.192	-9.445	-0.16
132	4	7.647	-9.446	-0.16
133	4	8.103	-9.447	-0.16
134	4	8.558	-9.448	-0.16
135	4	9.014	-9.45	-0.16
136	4	9.469	-9.451	-0.16
137	4	9.924	-9.452	-0.16
138	4	10.38	-9.454	-0.16
139	4	10.835	-9.455	-0.16
140	4	11.291	-9.456	-0.16
141	4	11.746	-9.458	-0.16
142	4	12.201	-9.459	-0.16
143	4	12.657	-9.46	-0.16
144	4	13.112	-9.461	-0.16
145	4	13.568	-9.463	-0.16
146	4	14.023	-9.464	-0.16
147	4	14.478	-9.465	-0.16
148	4	14.934	-9.467	-0.16
149	4	15.389	-9.468	-0.16
150	4	15.845	-9.469	-0.16
151	4	16.3	-9.47	-0.16
152	4	16.756	-9.472	-0.16
153	5	-0.17	-11.247	-0.29
154	5	0.285	-11.249	-0.29
155	5	0.741	-11.252	-0.29
156	5	1.197	-11.254	-0.29
157	5	1.653	-11.256	-0.29
158	5	2.109	-11.259	-0.29
159	5	2.565	-11.261	-0.29
160	5	3.021	-11.263	-0.29
161	5	3.476	-11.265	-0.29
162	5	3.932	-11.268	-0.29
163	5	4.388	-11.27	-0.29
164	5	4.844	-11.272	-0.29
165	5	5.3	-11.275	-0.29
166	5	5.756	-11.277	-0.29
167	5	6.212	-11.279	-0.29
168	5	6.667	-11.281	-0.29
169	5	7.123	-11.284	-0.29
170	5	7.579	-11.286	-0.29
171	5	8.035	-11.288	-0.29
172	5	8.491	-11.29	-0.29
173	5	8.947	-11.293	-0.29
174	5	9.403	-11.295	-0.29
175	5	9.858	-11.297	-0.29
176	5	10.314	-11.3	-0.29
177	5	10.77	-11.302	-0.29
178	5	11.226	-11.304	-0.29
179	5	11.682	-11.306	-0.29
180	5	12.138	-11.309	-0.29
181	5	12.594	-11.311	-0.29
182	5	13.049	-11.313	-0.29
183	5	13.505	-11.316	-0.29
184	5	13.961	-11.318	-0.29
185	5	14.417	-11.32	-0.29
186	5	14.873	-11.322	-0.29
187	5	15.329	-11.325	-0.29
188	5	15.784	-11.327	-0.29
189	5	16.24	-11.329	-0.29
190	5	16.696	-11.331	-0.29
191	6	-0.094	-13.048	-0.41
192	6	0.36	-13.051	-0.41
193	6	0.814	-13.054	-0.41
194	6	1.268	-13.057	-0.41
195	6	1.722	-13.061	-0.41
196	6	2.176	-13.064	-0.41
197	6	2.63	-13.067	-0.41
198	6	3.084	-13.071	-0.41
199	6	3.538	-13.074	-0.41
200	6	3.992	-13.077	-0.41
201	6	4.446	-13.08	-0.41
202	6	4.9	-13.084	-0.41
203	6	5.354	-13.087	-0.41
204	6	5.808	-13.09	-0.41
205	6	6.262	-13.094	-0.41
206	6	6.716	-13.097	-0.41
207	6	7.17	-13.1	-0.41
208	6	7.623	-13.103	-0.41
209	6	8.077	-13.107	-0.41
210	6	8.531	-13.11	-0.41
211	6	8.985	-13.113	-0.41
212	6	9.439	-13.117	-0.41
213	6	9.893	-13.12	-0.41
214	6	10.347	-13.123	-0.41
215	6	10.801	-13.126	-0.41
216	6	11.255	-13.13	-0.41
217	6	11.709	-13.133	-0.41
218	6	12.163	-13.136	-0.41
219	6	12.617	-13.139	-0.41
220	6	13.071	-13.143	-0.41
221	6	13.525	-13.146	-0.41
222	6	13.979	-13.149	-0.41
223	6	14.433	-13.153	-0.41
224	6	14.887	-13.156	-0.41
225	6	15.341	-13.159	-0.41
226	6	15.795	-13.162	-0.41
227	6	16.249	-13.166	-0.41
228	6	16.703	-13.169	-0.41
229	7	-0.132	-14.986	-0.14
230	7	0.323	-14.987	-0.14
231	7	0.778	-14.988	-0.14
232	7	1.233	-14.989	-0.14
233	7	1.688	-14.99	-0.14
234	7	2.143	-14.991	-0.14
235	7	2.598	-14.992	-0.14
236	7	3.053	-14.993	-0.14
237	7	3.508	-14.995	-0.14
238	7	3.963	-14.996	-0.14
239	7	4.418	-14.997	-0.14
240	7	4.873	-14.998	-0.14
241	7	5.328	-14.999	-0.14
242	7	5.783	-15	-0.14
243	7	6.238	-15.001	-0.14
244	7	6.693	-15.002	-0.14
245	7	7.148	-15.004	-0.14
246	7	7.603	-15.005	-0.14
247	7	8.058	-15.006	-0.14
248	7	8.513	-15.007	-0.14
249	7	8.968	-15.008	-0.14
250	7	9.423	-15.009	-0.14
251	7	9.878	-15.01	-0.14
252	7	10.333	-15.011	-0.14
253	7	10.788	-15.012	-0.14
254	7	11.243	-15.014	-0.14
255	7	11.698	-15.015	-0.14
256	7	12.153	-15.016	-0.14
257	7	12.608	-15.017	-0.14
258	7	13.063	-15.018	-0.14
259	7	13.518	-15.019	-0.14
260	7	13.973	-15.02	-0.14
261	7	14.428	-15.021	-0.14
262	7	14.883	-15.023	-0.14
263	7	15.338	-15.024	-0.14
264	7	15.793	-15.025	-0.14
265	7	16.248	-15.026	-0.14
266	7	16.703	-15.027	-0.14
267	8	-0.077	-16.715	-0.53
268	8	0.375	-16.719	-0.53
269	8	0.827	-16.723	-0.53
270	8	1.279	-16.727	-0.53
271	8	1.731	-16.731	-0.53
272	8	2.183	-16.735	-0.53
273	8	2.635	-16.74	-0.53
274	8	3.087	-16.744	-0.53
275	8	3.538	-16.748	-0.53
276	8	3.99	-16.752	-0.53
277	8	4.442	-16.756	-0.53
278	8	4.894	-16.761	-0.53
279	8	5.346	-16.765	-0.53
280	8	5.798	-16.769	-0.53
281	8	6.25	-16.773	-0.53
282	8	6.702	-16.777	-0.53
283	8	7.154	-16.781	-0.53
284	8	7.606	-16.786	-0.53
285	8	8.057	-16.79	-0.53
286	8	8.509	-16.794	-0.53
287	8	8.961	-16.798	-0.53
288	8	9.413	-16.802	-0.53
289	8	9.865	-16.806	-0.53
290	8	10.317	-16.811	-0.53
291	8	10.769	-16.815	-0.53
292	8	11.221	-16.819	-0.53
293	8	11.673	-16.823	-0.53
294	8	12.125	-16.827	-0.53
295	8	12.576	-16.832	-0.53
296	8	13.028	-16.836	-0.53
297	8	13.48	-16.84	-0.53
298	8	13.932	-16.844	-0.53
299	8	14.384	-16.848	-0.53
300	8	14.836	-16.852	-0.53
301	8	15.288	-16.857	-0.53
302	8	15.74	-16.861	-0.53
303	8	16.192	-16.865	-0.53
304	8	16.644	-16.869	-0.53
305	9	-0.103	-18.525	-0.61
306	9	0.351	-18.53	-0.61
307	9	0.805	-18.535	-0.61
308	9	1.259	-18.54	-0.61
309	9	1.713	-18.545	-0.61
310	9	2.167	-18.55	-0.61
311	9	2.621	-18.554	-0.61
312	9	3.075	-18.559	-0.61
313	9	3.529	-18.564	-0.61
314	9	3.983	-18.569	-0.61
315	9	4.437	-18.574	-0.61
316	9	4.891	-18.579	-0.61
317	9	5.345	-18.584	-0.61
318	9	5.799	-18.588	-0.61
319	9	6.253	-18.593	-0.61
320	9	6.707	-18.598	-0.61
321	9	7.161	-18.603	-0.61
322	9	7.615	-18.608	-0.61
323	9	8.069	-18.613	-0.61
324	9	8.523	-18.618	-0.61
325	9	8.977	-18.622	-0.61
326	9	9.431	-18.627	-0.61
327	9	9.885	-18.632	-0.61
328	9	10.339	-18.637	-0.61
329	9	10.793	-18.642	-0.61
330	9	11.247	-18.647	-0.61
331	9	11.701	-18.652	-0.61
332	9	12.155	-18.657	-0.61
333	9	12.609	-18.661	-0.61
334	9	13.062	-18.666	-0.61
335	9	13.516	-18.671	-0.61
336	9	13.97	-18.676	-0.61
337	9	14.424	-18.681	-0.61
338	9	14.878	-18.686	-0.61
339	9	15.332	-18.691	-0.61
340	9	15.786	-18.695	-0.61
341	9	16.24	-18.7	-0.61
342	9	16.694	-18.705	-0.61
343	10	-0.086	-20.395	-0.38
344	10	0.37	-20.398	-0.38
345	10	0.826	-20.401	-0.38
346	10	1.282	-20.404	-0.38
347	10	1.738	-20.407	-0.38
348	10	2.194	-20.41	-0.38
349	10	2.65	-20.413	-0.38
350	10	3.106	-20.416	-0.38
351	10	3.563	-20.419	-0.38
352	10	4.019	-20.422	-0.38
353	10	4.475	-20.425	-0.38
354	10	4.931	-20.428	-0.38
355	10	5.387	-20.431	-0.38
356	10	5.843	-20.434	-0.38
357	10	6.299	-20.437	-0.38
358	10	6.755	-20.44	-0.38
359	10	7.211	-20.443	-0.38
360	10	7.668	-20.446	-0.38
361	10	8.124	-20.449	-0.38
362	10	8.58	-20.452	-0.38
363	10	9.036	-20.455	-0.38
364	10	9.492	-20.458	-0.38
365	10	9.948	-20.461	-0.38
366	10	10.404	-20.464	-0.38
367	10	10.86	-20.467	-0.38
368	10	11.317	-20.47	-0.38
369	10	11.773	-20.473	-0.38
370	10	12.229	-20.476	-0.38
371	10	12.685	-20.479	-0.38
372	10	13.141	-20.482	-0.38
373	10	13.597	-20.485	-0.38
374	10	14.053	-20.488	-0.38
375	10	14.509	-20.491	-0.38
376	10	14.965	-20.494	-0.38
377	10	15.422	-20.497	-0.38
378	10	15.878	-20.5	-0.38
379	10	16.334	-20.503	-0.38
380	10	16.79	-20.506	-0.38
\.


--
-- Data for Name: operation; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.operation (id, state, coordinator_enabled, target_rows, max_attempts, offline_grace_s, started_at, updated_at, note) FROM stdin;
1	stopped	f	{1,2}	3	180	\N	2026-10-26 16:54:33.416154+00	\N
\.


--
-- Data for Name: operation_event; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.operation_event (id, ts, source, robot, kind, object_id, job_id, message) FROM stdin;
\.


--
-- Data for Name: robot_history; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.robot_history (id, robot_live_id, "time", ros_time, robot_type, online_flag, status, gps_location, task, topo_map_pos, current_node_id, target_node_id, battery_level, operation_hours_after_charging, load_status, crop_type, moved_at) FROM stdin;
\.


--
-- Data for Name: robot_info; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.robot_info (id, robot_name, robot_type, robot_description, manufacturer, serial_number) FROM stdin;
1	Husky1	Husky	Clearpath husky with zedcamera and 3D lidar	Clearpath	husky1
2	Husky2	Husky	Clearpath husky with zedcamera and 2D lidar	Clearpath	a200_0285
3	Jackal1	Jackal	Clearpath jackal	Clearpath	j100_0921
4	HuskyA300_1	Husky A300	Clearpath Husky A300	Clearpath	a300_00036
5	HuskyA200_0284	Husky A200	Clearpath Husky A200 with Kinova Gen3 ARM	Clearpath	a200_0284
\.


--
-- Data for Name: robot_live; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.robot_live (id, "time", ros_time, robot_type, online_flag, status, gps_location, task, topo_map_pos, current_node_id, target_node_id, battery_level, operation_hours_after_charging, load_status, crop_type, updated_at) FROM stdin;
\.


--
-- Data for Name: robot_operation; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.robot_operation (robot_id, enrolled, state, request, updated_at) FROM stdin;
1	f	active	\N	2026-10-26 16:54:33.416154+00
2	f	active	\N	2026-10-26 16:54:33.416154+00
4	f	active	\N	2026-10-26 16:54:33.416154+00
3	t	active	\N	2026-10-26 16:54:33.416154+00
5	f	active	\N	2026-10-26 16:54:33.416154+00
\.


--
-- Data for Name: schema_migrations; Type: TABLE DATA; Schema: public; Owner: -
--

COPY public.schema_migrations (version, applied_at) FROM stdin;
001_charging_stations	2026-10-09 13:44:19.755526+00
002_harvest_operation	2026-10-13 03:03:09.146796+00
003_dock_ids	2026-10-13 03:03:09.213399+00
004_robot_operation	2026-10-13 10:20:47.047673+00
005_dock_plugin_types	2026-10-13 11:34:13.425577+00
\.


--
-- Name: farm_harvesting_job_id_seq; Type: SEQUENCE SET; Schema: public; Owner: -
--

SELECT pg_catalog.setval('public.farm_harvesting_job_id_seq', 133, true);


--
-- Name: operation_event_id_seq; Type: SEQUENCE SET; Schema: public; Owner: -
--

SELECT pg_catalog.setval('public.operation_event_id_seq', 221, true);


--
-- Name: robot_history_id_seq; Type: SEQUENCE SET; Schema: public; Owner: -
--

SELECT pg_catalog.setval('public.robot_history_id_seq', 22669, true);


--
-- Name: robot_live_id_seq; Type: SEQUENCE SET; Schema: public; Owner: -
--

SELECT pg_catalog.setval('public.robot_live_id_seq', 1, false);


--
-- Name: charging_stations charging_stations_apriltag_id_key; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.charging_stations
    ADD CONSTRAINT charging_stations_apriltag_id_key UNIQUE (apriltag_id);


--
-- Name: charging_stations charging_stations_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.charging_stations
    ADD CONSTRAINT charging_stations_pkey PRIMARY KEY (id);


--
-- Name: dock_station dock_station_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.dock_station
    ADD CONSTRAINT dock_station_pkey PRIMARY KEY (id);


--
-- Name: farm_asset_on_map farm_asset_on_map_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.farm_asset_on_map
    ADD CONSTRAINT farm_asset_on_map_pkey PRIMARY KEY (id);


--
-- Name: farm_edge farm_edge_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.farm_edge
    ADD CONSTRAINT farm_edge_pkey PRIMARY KEY (id);


--
-- Name: farm_harvesting_job farm_harvesting_job_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.farm_harvesting_job
    ADD CONSTRAINT farm_harvesting_job_pkey PRIMARY KEY (id);


--
-- Name: farm_node farm_node_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.farm_node
    ADD CONSTRAINT farm_node_pkey PRIMARY KEY (id, zone_prefix);


--
-- Name: graph_node graph_node_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.graph_node
    ADD CONSTRAINT graph_node_pkey PRIMARY KEY (id);


--
-- Name: harvest_state harvest_state_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.harvest_state
    ADD CONSTRAINT harvest_state_pkey PRIMARY KEY (object_id);


--
-- Name: object_data object_data_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.object_data
    ADD CONSTRAINT object_data_pkey PRIMARY KEY (object_id);


--
-- Name: operation_event operation_event_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.operation_event
    ADD CONSTRAINT operation_event_pkey PRIMARY KEY (id);


--
-- Name: operation operation_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.operation
    ADD CONSTRAINT operation_pkey PRIMARY KEY (id);


--
-- Name: robot_history robot_history_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_history
    ADD CONSTRAINT robot_history_pkey PRIMARY KEY (id);


--
-- Name: robot_info robot_info_id_unique; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_info
    ADD CONSTRAINT robot_info_id_unique UNIQUE (id);


--
-- Name: robot_info robot_info_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_info
    ADD CONSTRAINT robot_info_pkey PRIMARY KEY (id, robot_name, serial_number);


--
-- Name: robot_live robot_live_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_live
    ADD CONSTRAINT robot_live_pkey PRIMARY KEY (id);


--
-- Name: robot_operation robot_operation_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_operation
    ADD CONSTRAINT robot_operation_pkey PRIMARY KEY (robot_id);


--
-- Name: schema_migrations schema_migrations_pkey; Type: CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.schema_migrations
    ADD CONSTRAINT schema_migrations_pkey PRIMARY KEY (version);


--
-- Name: charging_stations_dock_id_key; Type: INDEX; Schema: public; Owner: -
--

CREATE UNIQUE INDEX charging_stations_dock_id_key ON public.charging_stations USING btree (dock_id);


--
-- Name: harvest_state_status_idx; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX harvest_state_status_idx ON public.harvest_state USING btree (status, row_id);


--
-- Name: operation_event_id_idx; Type: INDEX; Schema: public; Owner: -
--

CREATE INDEX operation_event_id_idx ON public.operation_event USING btree (id DESC);


--
-- Name: robot_live before_update_move_old_data; Type: TRIGGER; Schema: public; Owner: -
--

CREATE TRIGGER before_update_move_old_data BEFORE UPDATE ON public.robot_live FOR EACH ROW EXECUTE FUNCTION public.move_robot_live_data();


--
-- Name: dock_station dock_station_robot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.dock_station
    ADD CONSTRAINT dock_station_robot_id_fkey FOREIGN KEY (robot_id) REFERENCES public.robot_info(id) ON DELETE SET NULL;


--
-- Name: harvest_state harvest_state_assigned_robot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.harvest_state
    ADD CONSTRAINT harvest_state_assigned_robot_id_fkey FOREIGN KEY (assigned_robot_id) REFERENCES public.robot_info(id) ON DELETE SET NULL;


--
-- Name: harvest_state harvest_state_object_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.harvest_state
    ADD CONSTRAINT harvest_state_object_id_fkey FOREIGN KEY (object_id) REFERENCES public.object_data(object_id) ON DELETE CASCADE;


--
-- Name: robot_operation robot_operation_robot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: -
--

ALTER TABLE ONLY public.robot_operation
    ADD CONSTRAINT robot_operation_robot_id_fkey FOREIGN KEY (robot_id) REFERENCES public.robot_info(id) ON DELETE CASCADE;


--
-- PostgreSQL database dump complete
--

\unrestrict bgpgcU4auX3AiwgtRceasaj5UqknaPfKH1qrLX26KqkRaI2LZXt8N7YHoocuq9R

