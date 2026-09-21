#!/usr/bin/env python3
"""Publish map->odom by fusing GPS + LIO position, with an absolute-yaw rotation.

Rationale
---------
LIO gives accurate short-term motion but its yaw is relative to an *arbitrary
startup orientation*, not true north. GPS gives absolute position (and, via
navsat, ENU/north). We fuse position in a small KF, and rotate the whole
map->odom by the absolute-heading correction (yaw_offset) so the map frame is
ENU/north — otherwise the LIO cloud stays rotated by yaw_offset and never lines
up with the geo-referenced base map.

yaw_offset source
-----------------
gps_heading_align estimates yaw_offset = wrap(gps_course_enu - lio_yaw) and,
once locked, publishes it on /heading_align/yaw_offset. Before any offset is
received (or in setups without heading align, e.g. the sim EKF path where this
node is not even started) yaw_offset stays 0 and map->odom is identity-rotation,
i.e. the previous behavior — so nothing else changes.

Filter
------
State: robot position in the map(ENU) frame  p = [px, py]  (two scalar KFs)
    Predict: p += R(yaw_offset) * (LIO position delta)   # rotate odom delta -> map/ENU
             P  += q * dt, capped at max_variance        # process noise (LIO drift)
    Update : z = GPS position (ENU, from navsat), R = GPS covariance, inflated
             and floored according to solution quality (see "Trust policy")
             standard KF update, with a Mahalanobis gate to reject GPS spikes

Output: map->odom rotation = R(yaw_offset);
        map->odom translation = p_published - R(yaw_offset) * LIO_position,
        where p_published tracks LIO motion exactly but chases the KF estimate
        at no more than max_correction_speed (see "Why the rate limit").

Trust policy (why the receiver's covariance alone is not enough)
---------------------------------------------------------------
unicore_gnss fills position_covariance from the receiver's reported lat/lon
std. That std is honest about the *solution mode* but systematically optimistic
under multipath: an RTK-fixed solution keeps reporting sigma ~ 2 cm while the
position can be metres off in an urban canyon. Feeding r = 4e-4 m^2 straight
into the KF drives the gain to k ~ 0.93, i.e. the filter is a pass-through of
GPS and inherits every multipath excursion. So we
  * inflate the reported variance by gps_variance_scale (optimism factor),
  * scale it again by solution status (SINGLE/SBAS get a bigger penalty than
    RTK, because the optimism grows as the mode degrades),
  * floor it at gps_variance_floor so no fix is ever trusted at cm level,
  * and drop it entirely above max_gps_variance.
When the receiver itself admits the fix is bad (status below min_fix_status, or
horizontal variance above max_h_var) for bad_gps_hold_sec, we stop fusing
altogether and dead-reckon on LIO — continuity beats absolute accuracy here.

Why the rate limit (this is what actually stops the "teleport")
--------------------------------------------------------------
The published translation t = p - R*lio *is* the accumulated GPS correction:
the predict step advances p by exactly R*delta_lio, which leaves t unchanged.
So every visible jump of map->odom is a jump of t, and bounding dt/dt bounds
the jump — without delaying real robot motion, because p_published advances by
R*delta_lio too and only the *correction* is rate-limited.

This matters because a Mahalanobis gate on its own turns a sustained GPS offset
into a *delayed* teleport rather than preventing it: while the gate rejects, var
grows by q*dt, so the gate widens and the gain approaches 1; the frame the gate
finally opens, the whole accumulated offset lands in a single cycle. Measured
with the previous defaults (r from a 2 cm RTK sigma, q=0.05, gate=5 sigma), a
3 m GPS offset was rejected for 7.2 s and then applied as one 2.997 m step.
Raising r alone does not fix this (still 2.26 m in one step); the rate limit
does (0.015 m per cycle at 0.3 m/s and 20 Hz).

False fixes (why none of the checks above can see them)
------------------------------------------------------
An RTK fixed solution works by resolving the carrier-phase integer ambiguity
N. Under multipath the solver can resolve it to the *wrong* integer: position
is then off by a geometric combination of wavelengths (19 cm on L1), so
decimetres to metres, while the receiver keeps reporting a 2 cm sigma because
its error model assumes N is correct. No amount of trust policy catches this
-- a false fix is not bad data, it is a self-consistent wrong answer, and it
passes min_fix_status, max_h_var and the variance floor untouched.

What does give it away is that a wrong N stays *constant* while the fix
holds, so it cancels out of position differences. A false fix is a square
wave: one step in when the solver latches the wrong N, a plateau where the
absolute position is off but increments are still correct, one step back out
when it re-resolves. Only the two edges are observable at all.

Hence step detection compares the GPS increment against the LIO increment
over one measurement interval. LIO drift accumulates slowly (0.01-0.1 m/s of
mismatch); a solution step dumps metres into a single 0.1 s epoch, i.e. tens
of m/s. Two to three orders of magnitude apart, which is why the threshold
is uncritical -- it only has to sit between them.

A step on its own must not condemn a fix, though. The identical edge appears
when a long PSRDIFF stretch finally fixes: the metres of LIO drift that piled
up meanwhile get corrected at once, and that correction is both real and
wanted. The two cases differ in whether the *reported sigma* jumped with the
position -- a genuine recovery arrives with the solution mode improving by
orders of magnitude, a false fix goes on claiming the same centimetres it
claimed before the step. That ratio is the discriminator (step_sigma_ratio),
and it must be computed on the receiver's raw variance, not on the scaled one
the filter acts on, because gps_variance_floor deliberately compresses the
very range being measured.

Once a step is condemned, position fusing stops for step_distrust_sec and the
filter dead-reckons on LIO. Rejecting outright rather than leaning on the
gate is deliberate: the gate cannot see an offset smaller than itself and
gps_variance_floor pins it near 2.5 m, so every sub-gate false fix used to be
fused whole. Dropping the plateau costs nothing -- its increments are good
but LIO already supplies those, and its absolute position is the one thing
known to be wrong.

The hold ends when the receiver re-resolves, recognised as a step cancelling
the one being held against. That comparison is made against the held offset
rather than against the estimate, because by then the estimate has
dead-reckoned away from where the hold began: judged against it, the trailing
edge reads backwards as soon as LIO has drifted past half the offset, which
condemns the recovery and strands the filter on a stale position. If no
cancelling step arrives the hold just expires and the sustained-offset escape
takes over as before, so step_distrust_sec bounds how long a plateau can be
ridden out and one that outlasts it degrades to the old behaviour rather than
to anything worse.

Inputs:
    /odometry/gps (nav_msgs/Odometry)      -- robot position in map (from navsat)
    /gps/fix (sensor_msgs/NavSatFix)       -- solution quality always; position
                                              too while /odometry/gps is silent
    /heading_align/yaw_offset (std_msgs/Float64) -- ENU heading correction [rad]
    TF odom->base_link                     -- from FAST-LIO chain

Outputs:
    TF map->odom
    /map_odom_from_gps/status (std_msgs/String, JSON) -- filter diagnostics
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
import tf2_ros


class MapOdomFromGps(Node):
    def __init__(self):
        super().__init__('map_odom_from_gps')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('gps_topic', '/odometry/gps')
        # Absolute-heading correction (rad) from gps_heading_align. map->odom is
        # rotated by this so odom(LIO) is expressed in ENU/north. 0 => identity
        # rotation (previous behavior / no heading-align running).
        self.declare_parameter('yaw_offset_topic', '/heading_align/yaw_offset')
        self.declare_parameter('publish_rate', 20.0)
        # Process noise: how fast LIO position drifts (m^2 per second, per axis).
        self.declare_parameter('process_noise', 0.05)
        # Fallback GPS variance (m^2) when the message covariance is 0/unknown.
        self.declare_parameter('gps_variance_default', 1.0)
        # Use the covariance carried in the GPS Odometry message for R.
        self.declare_parameter('use_msg_covariance', True)
        # Reject a GPS update if innovation exceeds this many sigmas (0 disables).
        self.declare_parameter('gps_gate_sigma', 5.0)
        # Initial state covariance (large => first GPS snaps the estimate in).
        self.declare_parameter('initial_variance', 10.0)
        self.declare_parameter('publish_identity_until_gps', True)
        # Fixed campus datum: same values used by gps_heading_align and campus_nav.
        # map_odom_from_gps subscribes to /gps/fix and converts lat/lon -> ENU
        # directly so GPS position is available immediately, before navsat can
        # produce /odometry/gps (which requires heading lock first).
        self.declare_parameter('datum_lat', 22.574357630323675)
        self.declare_parameter('datum_lon', 113.93507078035577)
        self.declare_parameter('raw_gps_topic', '/gps/fix')

        # --- Rate limit on the published correction (see module docstring) ----
        # Max speed (m/s) at which map->odom translation may chase the KF
        # estimate. 0 disables (previous behavior: corrections apply instantly).
        # Bounds the per-cycle jump at max_correction_speed / publish_rate.
        self.declare_parameter('max_correction_speed', 0.3)

        # --- Trust policy: how much of the reported GPS covariance to believe --
        # Multiplier on the reported variance (receiver sigma is systematically
        # optimistic under multipath). 4.0 == sigma x2.
        self.declare_parameter('gps_variance_scale', 4.0)
        # Never trust a fix better than this (m^2). 0.09 == sigma 0.3 m; keeps
        # the gain around 0.2 instead of 0.93 for a 2 cm RTK sigma.
        self.declare_parameter('gps_variance_floor', 0.09)
        # Above this (m^2) a fix carries no usable information; skip the update.
        # 400.0 == sigma 20 m.
        self.declare_parameter('max_gps_variance', 400.0)
        # Extra variance penalty by NavSatStatus. The receiver's optimism grows
        # as the solution mode degrades, so penalise the weaker modes on top of
        # their (already larger) reported sigma.
        #   status 2 = STATUS_GBAS_FIX -> RTK. NOTE bestnava._GBAS_TYPES merges
        #     NARROW_FLOAT and NARROW_INT into 2, so float and fixed are
        #     indistinguishable here; their reported sigma separates them.
        self.declare_parameter('var_scale_status_gbas', 1.0)   # status 2
        self.declare_parameter('var_scale_status_sbas', 4.0)   # status 1
        self.declare_parameter('var_scale_status_fix', 9.0)    # status 0, SINGLE
        # Below this NavSatStatus the fix is unusable (-1 = STATUS_NO_FIX).
        # Same name/meaning as gps_heading_align's min_fix_status.
        self.declare_parameter('min_fix_status', 0)
        # Horizontal variance (m^2) above which the receiver itself is telling us
        # the fix is bad. Same name/meaning as gps_heading_align's max_h_var.
        self.declare_parameter('max_h_var', 25.0)
        # Cap on the state variance. Without it the gate widens without bound
        # while GPS is rejected and the gain creeps to 1, so the first accepted
        # fix lands as a step. 4.0 == sigma 2 m.
        self.declare_parameter('max_variance', 4.0)
        # A gate rejection that persists this long with a consistent innovation
        # sign is a real offset, not a spike: accept it and let the rate limit
        # ease it in. 0 disables (gate rejects forever until variance grows).
        # Unreachable during a false-fix hold, which rejects further upstream:
        # a false fix produces exactly this signature and would be ramped in.
        self.declare_parameter('sustained_offset_sec', 3.0)
        # --- False-fix detection (see module docstring) ----------------------
        # Mismatch speed between the GPS and LIO increments above which the
        # position is taken to have stepped rather than moved. LIO drift sits
        # near 0.1 m/s and a metre-class step at 10 Hz is tens of m/s, so this
        # only has to clear the jitter from GPS/timer phase misalignment (up to
        # ~1 m/s at speed). 0 disables the detector entirely.
        self.declare_parameter('step_detect_speed', 3.0)
        # A step is excused when the receiver's raw sigma improved by at least
        # this factor, i.e. the mode genuinely recovered (PSRDIFF 1.5 m -> INT
        # 0.03 m is 50x) rather than jumping at constant claimed accuracy
        # (false fix, 2 cm -> 2 cm is 1x).
        self.declare_parameter('step_sigma_ratio', 5.0)
        # How long an unexcused step suspends position fusing. Ends early when
        # the receiver re-resolves. Must outlast the typical false-fix plateau,
        # or it expires mid-plateau and the sustained-offset escape ramps the
        # wrong position in after all -- but not by more than needed, since a
        # misjudged step costs this long on dead reckoning. 0 disables the
        # detector, as does step_detect_speed.
        self.declare_parameter('step_distrust_sec', 25.0)
        # How long to keep trying to re-seed the estimate once the heading lock
        # arrives, before giving up and easing the offset in the old way.
        # Until gps_heading_align locks, yaw_offset is 0 while the truth can be
        # any angle, so the estimate built during that window was integrated
        # with the wrong rotation and is not worth keeping. See _maybe_reseed.
        # 0 disables re-seeding entirely.
        self.declare_parameter('reseed_on_heading_lock_sec', 30.0)
        # Stop fusing entirely after this long of continuously bad quality and
        # dead-reckon on LIO. 0 disables the hold.
        self.declare_parameter('bad_gps_hold_sec', 3.0)
        # /odometry/gps older than this counts as silent, and the raw /gps/fix
        # path takes over as the measurement source.
        self.declare_parameter('gps_odom_fresh_sec', 1.0)
        self.declare_parameter('status_topic', '/map_odom_from_gps/status')
        self.declare_parameter('status_rate', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        gps_topic = self.get_parameter('gps_topic').value
        yaw_offset_topic = self.get_parameter('yaw_offset_topic').value
        rate = float(self.get_parameter('publish_rate').value)
        self.q = float(self.get_parameter('process_noise').value)
        self.r_default = float(self.get_parameter('gps_variance_default').value)
        self.use_msg_cov = bool(self.get_parameter('use_msg_covariance').value)
        self.gate = float(self.get_parameter('gps_gate_sigma').value)
        self.p0 = float(self.get_parameter('initial_variance').value)
        self.publish_identity_until_gps = bool(
            self.get_parameter('publish_identity_until_gps').value)

        self.max_corr_speed = float(
            self.get_parameter('max_correction_speed').value)
        self.r_scale = float(self.get_parameter('gps_variance_scale').value)
        self.r_floor = float(self.get_parameter('gps_variance_floor').value)
        self.r_max = float(self.get_parameter('max_gps_variance').value)
        self.r_scale_gbas = float(self.get_parameter('var_scale_status_gbas').value)
        self.r_scale_sbas = float(self.get_parameter('var_scale_status_sbas').value)
        self.r_scale_fix = float(self.get_parameter('var_scale_status_fix').value)
        self.min_fix_status = int(self.get_parameter('min_fix_status').value)
        self.max_h_var = float(self.get_parameter('max_h_var').value)
        self.max_var = float(self.get_parameter('max_variance').value)
        self.sustained_sec = float(self.get_parameter('sustained_offset_sec').value)
        self.step_speed = float(self.get_parameter('step_detect_speed').value)
        self.step_sigma_ratio = float(
            self.get_parameter('step_sigma_ratio').value)
        self.step_distrust_sec = float(
            self.get_parameter('step_distrust_sec').value)
        self.reseed_window = float(
            self.get_parameter('reseed_on_heading_lock_sec').value)
        self.bad_hold_sec = float(self.get_parameter('bad_gps_hold_sec').value)
        self.gps_odom_fresh = float(self.get_parameter('gps_odom_fresh_sec').value)

        # ENU conversion for the raw /gps/fix path. The projection must match
        # navsat_transform's UTM metric, because /odometry/gps (the other
        # measurement source) comes out of navsat: any mismatch shows up as a
        # position step the moment the filter switches sources. A tangent plane
        # linearized at the datum left 0.61 m of it at 4 km (the Daxuecheng
        # site), so this goes through the same UTM series instead and the step is
        # gone at any distance.
        _datum_lat_deg = self.get_parameter('datum_lat').value
        _datum_lon_deg = self.get_parameter('datum_lon').value
        self._datum_lat_deg = _datum_lat_deg
        self._datum_lon_deg = _datum_lon_deg
        self._utm_zone = int((_datum_lon_deg + 180.0) // 6) + 1
        self._datum_e, self._datum_n = self._lonlat_to_utm(
            _datum_lon_deg, _datum_lat_deg, self._utm_zone)
        _gamma = self._grid_convergence(
            _datum_lat_deg, _datum_lon_deg, self._utm_zone)
        self._cos_gamma = math.cos(_gamma)
        self._sin_gamma = math.sin(_gamma)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # Absolute-heading correction: map->odom yaw. 0 until gps_heading_align
        # locks and starts publishing => identity rotation (previous behavior).
        self.yaw_offset = 0.0
        self._c = 1.0   # cos(yaw_offset)
        self._s = 0.0   # sin(yaw_offset)
        self.have_offset = False
        # Timestamp of the first yaw_offset, while a re-seed is still pending.
        self.reseed_since = None

        # KF state (map-frame position) + per-axis variance
        self.px = self.py = 0.0
        self.var_x = self.var_y = self.p0
        self.initialized = False

        # Published (rate-limited) map-frame position. Advances with LIO exactly;
        # chases (px, py) at max_corr_speed.
        self.pub_x = self.pub_y = 0.0

        # Latest GPS measurement (consumed once)
        self.gps = None          # (x, y, rx, ry)
        self.gps_new = False
        self.gps_src = 'none'    # 'odom' | 'raw'

        # Solution quality, cached from /gps/fix and shared by both measurement
        # paths (/odometry/gps carries no status).
        self.fix_status = None
        self.fix_h_var = None
        self.bad_since = None
        self.holding = False
        self.last_gps_odom_t = None

        # Per-axis gate rejection tracking, for the sustained-offset escape.
        self.rej = {'x': {'since': None, 'sign': 0, 'count': 0},
                    'y': {'since': None, 'sign': 0, 'count': 0}}

        # Solution-step detection. _lio_accum holds the map-frame LIO
        # displacement since the last fused measurement; the GPS increment is
        # compared against it. _last_raw_h_var is the receiver's own variance,
        # deliberately not the scaled one (see module docstring).
        self._lio_accum = [0.0, 0.0]
        self._last_z = None
        self._last_z_t = None
        self._last_raw_h_var = None
        self.step_since = None
        self.step_vec = (0.0, 0.0)
        self.step_count = 0

        self.lio_prev = None     # last LIO position used for prediction
        self.last_time = None

        # Diagnostics
        self.state = 'no_gps'
        self.last_r = 0.0
        self.last_k = 0.0
        self.last_innov = 0.0
        self._warned_backlog = False

        self.create_subscription(Odometry, gps_topic, self.gps_cb, 10)
        self.create_subscription(Float64, yaw_offset_topic, self.offset_cb, 10)
        raw_gps_topic = self.get_parameter('raw_gps_topic').value
        # BEST_EFFORT is mandatory: unicore_gnss publishes /gps/fix with
        # qos_profile_sensor_data (BEST_EFFORT), and a RELIABLE subscriber is
        # incompatible with it — rclpy only logs an "incompatible QoS" warning
        # and delivers nothing. Same reasoning as nav_node.py and
        # gps_heading_align.py, both of which already subscribe BEST_EFFORT.
        self.create_subscription(
            NavSatFix, raw_gps_topic, self._raw_gps_cb, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        status_rate = float(self.get_parameter('status_rate').value)
        if status_rate > 0.0:
            self.create_timer(1.0 / status_rate, self.publish_status)
        self.get_logger().info(
            f'map_odom_from_gps: 2D position KF (x,y) + ENU yaw from '
            f'{yaw_offset_topic} (identity until first offset). '
            f'q={self.q}, gate={self.gate} sigma, '
            f'R: x{self.r_scale} floor {self.r_floor} cap {self.r_max}, '
            f'var cap {self.max_var}, correction limit '
            f'{self.max_corr_speed} m/s, bad-GPS hold {self.bad_hold_sec}s. '
            f'Pre-lock GPS position from {raw_gps_topic} (datum '
            f'{_datum_lat_deg:.6f}, {_datum_lon_deg:.6f})')

    # ------------------------------------------------------------------ inputs
    def offset_cb(self, msg: Float64):
        self.yaw_offset = float(msg.data)
        self._c = math.cos(self.yaw_offset)
        self._s = math.sin(self.yaw_offset)
        if not self.have_offset:
            self.have_offset = True
            # Only meaningful if the filter already ran with the wrong rotation.
            # If it has not initialized yet, the initial seed below will use this
            # offset from the start and there is nothing to discard.
            if self.reseed_window > 0.0 and self.initialized:
                self.reseed_since = self._now()
            self.get_logger().info(
                f'yaw_offset locked: {math.degrees(self.yaw_offset):.1f} deg; '
                f'map->odom now rotates odom(LIO) into ENU/north')

    # WGS84 + UTM constants. Kept inline rather than imported: this is a
    # standalone script, and campus_nav is only an exec_depend. Same trade-off as
    # NavNode's copy.
    _WGS84_A = 6378137.0                          # semi-major axis (m)
    _WGS84_F = 1.0 / 298.257223563                # flattening
    _WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)       # first eccentricity squared
    _WGS84_EP2 = _WGS84_E2 / (1.0 - _WGS84_E2)    # second eccentricity squared
    _UTM_K0 = 0.9996                              # UTM central scale factor
    _UTM_FALSE_EASTING = 500000.0

    @classmethod
    def _lonlat_to_utm(cls, lon, lat, zone):
        """WGS84 lon/lat (deg) -> UTM easting, northing (m). Northern hemisphere.

        Transverse Mercator series (Snyder / USGS PP-1395), sub-millimetre within
        a zone. MUST stay consistent with campus_nav/route_graph.py's
        lonlat_to_utm, NaviSkill/nav_node.py's copy and PointMarker/geo.py.
        """
        p = math.radians(lat)
        dlon = math.radians(lon - (6 * zone - 183))
        e2, ep2 = cls._WGS84_E2, cls._WGS84_EP2

        n = cls._WGS84_A / math.sqrt(1.0 - e2 * math.sin(p) ** 2)
        t = math.tan(p) ** 2
        c = ep2 * math.cos(p) ** 2
        a = dlon * math.cos(p)
        m = cls._WGS84_A * (
            (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * p
            - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * p)
            + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * p)
            - (35 * e2**3 / 3072) * math.sin(6 * p)
        )

        east = cls._UTM_K0 * n * (
            a
            + (1 - t + c) * a**3 / 6
            + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a**5 / 120
        ) + cls._UTM_FALSE_EASTING
        north = cls._UTM_K0 * (
            m
            + n * math.tan(p) * (
                a**2 / 2
                + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
                + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720
            )
        )
        return east, north

    @classmethod
    def _grid_convergence(cls, lat, lon, zone):
        """Angle from UTM grid north to true north (rad), positive east of the
        central meridian. 1.128 deg at the Zhicheng datum."""
        return math.atan(
            math.tan(math.radians(lon - (6 * zone - 183)))
            * math.sin(math.radians(lat)))

    def _latlon_to_enu(self, lat, lon):
        """ENU relative to the campus datum, reproducing navsat_transform's map
        frame: UTM, then the one grid-convergence de-rotation navsat establishes
        at the datum. Same formula as campus_nav.route_graph's ENU — kept inline
        to avoid cross-package imports."""
        east, north = self._lonlat_to_utm(lon, lat, self._utm_zone)
        de, dn = east - self._datum_e, north - self._datum_n
        return (de * self._cos_gamma + dn * self._sin_gamma,
                -de * self._sin_gamma + dn * self._cos_gamma)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _gps_odom_fresh(self) -> bool:
        if self.last_gps_odom_t is None:
            return False
        if self.gps_odom_fresh <= 0.0:
            return True
        return (self._now() - self.last_gps_odom_t) <= self.gps_odom_fresh

    def _raw_gps_cb(self, msg: NavSatFix):
        """Cache solution quality (always) and, while /odometry/gps is silent,
        also use the fix as the position measurement.

        The quality cache is the only place status reaches the filter:
        /odometry/gps is a plain Odometry and carries no NavSatStatus, so the
        post-lock path depends on this callback for its trust decisions.

        The measurement role is gated on /odometry/gps freshness rather than on
        have_offset: gps_heading_align publishes a coarse yaw_offset during a
        SLAM->outdoor handoff, well before it locks and before navsat can emit
        /odometry/gps. Keying off have_offset would switch this path off while
        nothing had replaced it, leaving the filter uninitialised and map->odom
        stuck at identity for the whole coarse phase.
        """
        finite = math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
        self.fix_status = int(msg.status.status)
        cov = msg.position_covariance
        h_var = 0.5 * (float(cov[0]) + float(cov[4]))
        if not math.isfinite(h_var) or h_var <= 0.0:
            # Unknown variance: allow but do not treat as high quality; use a
            # conservative stand-in so max_h_var still applies if set low.
            # Matches gps_heading_align._fix_quality_ok.
            h_var = self.max_h_var
        self.fix_h_var = h_var

        if not finite or msg.status.status < 0:  # STATUS_NO_FIX
            return
        if self._gps_odom_fresh():
            return
        east, north = self._latlon_to_enu(msg.latitude, msg.longitude)
        # NavSatFix position_covariance is [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        # (east variance = [0], north variance = [4]).
        rx = ry = self.r_default
        if (self.use_msg_cov
                and msg.position_covariance_type >= 2  # DIAGONAL_KNOWN or KNOWN
                and math.isfinite(cov[0]) and cov[0] > 0.0
                and math.isfinite(cov[4]) and cov[4] > 0.0):
            rx = float(cov[0])
            ry = float(cov[4])
        self.gps = (east, north, self._scale_r(rx), self._scale_r(ry))
        self.gps_new = True
        self.gps_src = 'raw'

    def gps_cb(self, msg: Odometry):
        self.last_gps_odom_t = self._now()
        rx = ry = self.r_default
        if self.use_msg_cov:
            cxx = msg.pose.covariance[0]
            cyy = msg.pose.covariance[7]
            # navsat_transform rotates the NavSatFix covariance into the map
            # frame, so a COVARIANCE_TYPE_UNKNOWN fix (-1 diagonals) can come
            # out as positive garbage. Require finite and sane before using it.
            if math.isfinite(cxx) and cxx > 0.0:
                rx = float(cxx)
            if math.isfinite(cyy) and cyy > 0.0:
                ry = float(cyy)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self.gps = (x, y, self._scale_r(rx), self._scale_r(ry))
        self.gps_new = True
        self.gps_src = 'odom'

    # ----------------------------------------------------------- trust policy
    def _scale_r(self, r: float) -> float:
        """Reported variance -> variance the filter is willing to act on."""
        r = max(r, 0.0) * self.r_scale
        st = self.fix_status
        if st is None or st <= 0:
            # No status yet, or STATUS_FIX (SINGLE): the most optimistic mode.
            r *= self.r_scale_fix
        elif st == 1:
            r *= self.r_scale_sbas
        else:
            r *= self.r_scale_gbas
        return max(r, self.r_floor)

    def _quality_bad(self) -> bool:
        """True when the receiver itself says the fix is unusable."""
        if self.fix_status is None:
            return False  # no /gps/fix seen yet; don't block on missing info
        if self.fix_status < self.min_fix_status:
            return True
        if self.fix_h_var is not None and self.fix_h_var > self.max_h_var:
            return True
        return False

    def _update_hold(self, now: float) -> None:
        """Latch dead-reckoning after bad_gps_hold_sec of continuously bad GPS."""
        if self._quality_bad():
            if self.bad_since is None:
                self.bad_since = now
            hold = (self.bad_hold_sec > 0.0
                    and (now - self.bad_since) >= self.bad_hold_sec)
        else:
            self.bad_since = None
            hold = False
        if hold and not self.holding:
            self.get_logger().warn(
                f'GPS quality bad for {self.bad_hold_sec}s '
                f'(status={self.fix_status}, h_var={self.fix_h_var}); '
                f'dead-reckoning on LIO until it recovers')
        elif self.holding and not hold:
            self.get_logger().info(
                'GPS quality recovered; resuming fusion (correction eased in at '
                f'{self.max_corr_speed} m/s)')
        self.holding = hold

    # ------------------------------------------------------------------- cycle
    def _lio_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_link_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _maybe_reseed(self, now_t: float, lio) -> bool:
        """Seed the estimate from GPS on the first heading lock.

        Returns True when this cycle was consumed by the re-seed.

        Before the lock, yaw_offset is 0 while the truth can be any angle, so
        every LIO delta integrated until then entered the estimate rotated by
        the wrong amount. On 2026-08-26 the truth was 166.9 deg and the 5 m
        alignment baseline alone put the estimate 9.9 m out; the filter then saw
        that as a sustained GPS offset, accepted it, and handed the rate limiter
        a 12.6 m backlog to grind through at 0.3 m/s. map->odom drifts for those
        42 s, and a drifting map->odom is what pushes the costmaps' sensor origin
        out of bounds and stalls planning. None of that history is worth keeping,
        so seed from GPS exactly like the initial fix does.

        Waits for a fix the trust policy accepts, then gives up after
        reseed_window so a late GPS recovery cannot step the pose after the robot
        has already driven a long way on LIO alone.
        """
        if self.gps is None or self._quality_bad():
            if now_t - self.reseed_since > self.reseed_window:
                self.reseed_since = None
                self.get_logger().warn(
                    'heading locked but no fix good enough to re-seed on within '
                    f'{self.reseed_window:.0f}s; keeping the pre-lock estimate '
                    'and easing any offset in at '
                    f'{self.max_corr_speed} m/s instead')
            return False

        self.reseed_since = None
        dropped = math.hypot(self.gps[0] - self.px, self.gps[1] - self.py)
        self.px, self.py = self.gps[0], self.gps[1]
        # Same reason the initial seed bypasses it: the discarded estimate can be
        # tens of metres out, and easing that in is the problem being fixed.
        self.pub_x, self.pub_y = self.px, self.py
        self.var_x = self.var_y = self.p0
        # Must move with the seed, or the next cycle's delta would re-apply the
        # whole pre-lock stretch.
        self.lio_prev = lio
        self.last_time = now_t
        self.gps_new = False
        self.state = 'reseed'
        self.get_logger().warn(
            f'heading lock: re-seeded map position from GPS, discarding a '
            f'{dropped:.2f} m pre-lock error that was integrated with '
            f'yaw_offset=0')
        self._broadcast_from_state(lio)
        return True

    def on_timer(self):
        now_t = self._now()
        lio = self._lio_xy()

        # Not enough to run the filter yet
        if lio is None or (not self.initialized and self.gps is None):
            self.state = 'no_gps'
            if self.publish_identity_until_gps and not self.initialized:
                self.broadcast(0.0, 0.0, 0.0)
            return

        # Initialize the estimate from the first GPS fix
        if not self.initialized:
            self.px, self.py = self.gps[0], self.gps[1]
            # The rate limit must NOT apply here: the robot can be kilometres
            # from the datum, and easing that in at max_corr_speed would take
            # hours. Seed the published state on top of the estimate instead.
            self.pub_x, self.pub_y = self.px, self.py
            self.var_x = self.var_y = self.p0
            self.lio_prev = lio
            self.last_time = now_t
            self.initialized = True
            self.gps_new = False
            self.state = 'init'
            self._broadcast_from_state(lio)
            return

        # Drop the estimate built before the heading was known (see _maybe_reseed)
        if self.reseed_since is not None and self._maybe_reseed(now_t, lio):
            return

        # --- Predict: advance by LIO delta (rotated odom->map), grow covariance ---
        dt = now_t - self.last_time
        self.last_time = now_t
        if dt < 0.0:
            dt = 0.0
        dx = lio[0] - self.lio_prev[0]
        dy = lio[1] - self.lio_prev[1]
        mdx = self._c * dx - self._s * dy
        mdy = self._s * dx + self._c * dy
        self.px += mdx
        self.py += mdy
        # The published state follows real motion with zero lag; only the
        # correction below is rate-limited.
        self.pub_x += mdx
        self.pub_y += mdy
        # Same delta the step detector weighs the next GPS increment against.
        self._lio_accum[0] += mdx
        self._lio_accum[1] += mdy
        self.lio_prev = lio
        self.var_x = min(self.var_x + self.q * dt, self.max_var)
        self.var_y = min(self.var_y + self.q * dt, self.max_var)

        # --- Update: fuse a fresh GPS fix (quality-weighted, outlier-gated) ---
        self._update_hold(now_t)
        if self.holding:
            self.state = 'dead_reckon'
            self.gps_new = False
        elif self.gps_new:
            self.gps_new = False
            zx, zy, rx, ry = self.gps
            if max(rx, ry) > self.r_max:
                self.state = 'gated'
                self.get_logger().warn(
                    f'GPS variance {max(rx, ry):.1f} > max_gps_variance '
                    f'{self.r_max}; skipping update', throttle_duration_sec=5.0)
            else:
                self._detect_solution_step(zx, zy, now_t)
                self.last_r = 0.5 * (rx + ry)
                ok_x = self._kf_update_axis('x', zx, rx, now_t)
                ok_y = self._kf_update_axis('y', zy, ry, now_t)
                # 'gated' as soon as *either* axis is being rejected: a
                # multipath excursion is usually along one axis only, and that
                # is exactly the case an operator needs to see. reject_x /
                # reject_y in the status message say which.
                self.state = 'fusing' if (ok_x and ok_y) else 'gated'

        self._apply_rate_limit(dt)
        self._broadcast_from_state(lio)

    def _detect_solution_step(self, zx, zy, now_t) -> None:
        """Flag a GPS position step that no gain in solution quality explains.

        Runs once per fused measurement, ahead of the KF update, so a hold it
        raises or releases takes effect on that same measurement.
        """
        prev_z, prev_t = self._last_z, self._last_z_t
        prev_h_var = self._last_raw_h_var
        lio_dx, lio_dy = self._lio_accum
        self._last_z = (zx, zy)
        self._last_z_t = now_t
        self._last_raw_h_var = self.fix_h_var
        self._lio_accum = [0.0, 0.0]

        if (self.step_speed <= 0.0 or self.step_distrust_sec <= 0.0
                or prev_z is None or prev_t is None):
            return
        dt = now_t - prev_t
        # On a gap (dead-reckon hold, dropped fixes) _lio_accum spans motion the
        # GPS increment cannot be checked against, so skip rather than read the
        # accumulated mismatch as a step.
        if dt <= 0.0 or dt > 1.0:
            return

        step_dx = zx - prev_z[0] - lio_dx
        step_dy = zy - prev_z[1] - lio_dy
        step = math.hypot(step_dx, step_dy)
        if step / dt <= self.step_speed:
            return

        if self._step_distrusted(now_t):
            # Already holding, so the reference for "is the fix coming back" is
            # the offset being held against -- not the estimate, which has been
            # dead-reckoning away from where the hold began and would read the
            # trailing edge exactly backwards once LIO has drifted far enough.
            held = math.hypot(*self.step_vec)
            if math.hypot(step_dx + self.step_vec[0],
                          step_dy + self.step_vec[1]) < 0.5 * held:
                self._accept_step(f'stepped {step:.2f} m, undoing the '
                                  f'{held:.2f} m it was held for')
                return
            # Stepped somewhere else again. Extend the hold and accumulate, so
            # the trailing edge still cancels whenever it comes.
            self.step_since = now_t
            self.step_vec = (self.step_vec[0] + step_dx,
                             self.step_vec[1] + step_dy)
            self.step_count += 1
            self.get_logger().warn(
                f'GPS stepped another {step:.2f} m while held; offset now '
                f'{math.hypot(*self.step_vec):.2f} m')
            return

        # Not holding. A step that leaves the fix *closer* to the estimate is
        # the receiver arriving at a position we already had right, so it is
        # the old solution that was wrong, not this one.
        d_now = math.hypot(zx - self.px, zy - self.py)
        d_no_step = math.hypot(zx - step_dx - self.px, zy - step_dy - self.py)
        if d_now < d_no_step:
            self._accept_step(f'stepped {step:.2f} m toward the estimate')
            return

        # It stepped away. Did the receiver's own accuracy claim improve enough
        # to account for it? sqrt because what is stored is a variance.
        ratio = 0.0
        if (prev_h_var is not None and self.fix_h_var is not None
                and prev_h_var > 0.0 and self.fix_h_var > 0.0):
            ratio = math.sqrt(prev_h_var / self.fix_h_var)
        if ratio >= self.step_sigma_ratio:
            self._accept_step(
                f'stepped {step:.2f} m as sigma improved {ratio:.0f}x')
            return

        self.step_since = now_t
        self.step_vec = (step_dx, step_dy)
        self.step_count += 1
        self.get_logger().warn(
            f'GPS stepped {step:.2f} m ({step / dt:.0f} m/s vs LIO) at '
            f'unchanged accuracy (sigma {ratio:.1f}x): suspected false fix, '
            f'suspending position fusing for {self.step_distrust_sec:.0f}s')

    def _accept_step(self, what: str) -> None:
        """Take a step as legitimate, releasing any hold it interrupts."""
        held = self.step_since is not None
        self.step_since = None
        self.step_vec = (0.0, 0.0)
        self.get_logger().info(
            f'GPS {what}: accepting, '
            + ('false-fix hold released' if held
               else f'easing in at {self.max_corr_speed} m/s'))

    def _step_distrusted(self, now_t) -> bool:
        """True while a suspected false fix should keep fusing suspended."""
        return (self.step_since is not None
                and self.step_distrust_sec > 0.0
                and (now_t - self.step_since) < self.step_distrust_sec)

    def _kf_update_axis(self, axis, z, r, now_t) -> bool:
        """Standard scalar KF update. Returns True if the fix was accepted."""
        if axis == 'x':
            p, var = self.px, self.var_x
        else:
            p, var = self.py, self.var_y
        innov = z - p
        s = var + r
        self.last_innov = innov
        rj = self.rej[axis]

        if self._step_distrusted(now_t):
            # Hold: the fix carries a known wrong offset, so its absolute
            # position is worthless until the receiver re-resolves. Its
            # increments are still good but LIO already supplies those, so
            # there is nothing left here worth fusing -- dead-reckon instead.
            # This has to come before the gate, which cannot see an offset
            # smaller than itself (gps_variance_floor holds it near 2.5 m).
            self.get_logger().warn(
                f'GPS {axis} held off (innov={innov:.2f}): suspected false fix '
                f'{now_t - self.step_since:.0f}s ago',
                throttle_duration_sec=5.0)
            return False

        if self.gate > 0.0 and s > 0.0 and (innov * innov) > (self.gate * self.gate) * s:
            sign = 1 if innov > 0.0 else -1
            if rj['since'] is None or rj['sign'] != sign:
                rj['since'], rj['sign'], rj['count'] = now_t, sign, 1
            else:
                rj['count'] += 1
            sustained = (self.sustained_sec > 0.0
                         and (now_t - rj['since']) >= self.sustained_sec)
            if sustained and self._step_distrusted(now_t):
                # Persistence alone cannot tell a false fix from LIO drift --
                # both hold a steady offset. The step that opened this offset
                # can, and it said false fix, so go on rejecting.
                self.get_logger().warn(
                    f'GPS {axis} offset {innov:.2f} m not accepted: arrived as '
                    f'a solution step {now_t - self.step_since:.0f}s ago',
                    throttle_duration_sec=5.0)
                return False
            if not sustained:
                self.get_logger().warn(
                    f'GPS {axis} rejected (innov={innov:.2f}, '
                    f'sqrt(S)={math.sqrt(s):.2f})',
                    throttle_duration_sec=2.0)
                return False
            # A consistent offset this persistent is real, not a spike. Accept
            # it; the rate limit turns it into a ramp instead of a step.
            self.get_logger().warn(
                f'GPS {axis} offset {innov:.2f} m persisted '
                f'{now_t - rj["since"]:.1f}s over {rj["count"]} frames; '
                f'accepting and easing in at {self.max_corr_speed} m/s')
        rj['since'], rj['sign'], rj['count'] = None, 0, 0

        k = var / s if s > 0.0 else 0.0
        self.last_k = k
        p = p + k * innov
        var = (1.0 - k) * var
        if axis == 'x':
            self.px, self.var_x = p, var
        else:
            self.py, self.var_y = p, var
        return True

    def _apply_rate_limit(self, dt: float) -> None:
        """Move the published position toward the KF estimate, bounded in speed.

        Bounds the per-cycle jump of map->odom at max_correction_speed * dt.
        The bound only applies once the heading is known -- see below for why
        applying it earlier is actively harmful.
        """
        ex = self.px - self.pub_x
        ey = self.py - self.pub_y
        if self.max_corr_speed <= 0.0 or not self.have_offset:
            # Before the heading lock the limit is not just useless, it is the
            # thing that makes the published pose diverge. yaw_offset is still 0
            # while the truth can be any angle, so the prediction injects error
            # at 2*v*sin(dyaw/2) -- 0.79 m/s for the 166.9 deg / 0.4 m/s case on
            # 2026-08-26 -- and it reaches pub_x unlimited (see on_timer), while
            # the GPS correction that would cancel it is capped here at 0.3 m/s.
            # The estimate px stays near truth (innov points at it); only the
            # published pose runs away, taking global_costmap's map-frame
            # projection with it. Track the estimate directly until the rotation
            # is known; _maybe_reseed then hands over with a clean state.
            self.pub_x, self.pub_y = self.px, self.py
            return
        # Limit the 2D correction as a vector so the ramp direction is preserved
        # (limiting each axis independently would bend the path).
        d = math.hypot(ex, ey)
        lim = self.max_corr_speed * max(dt, 0.0)
        if d <= lim or d <= 0.0:
            self.pub_x, self.pub_y = self.px, self.py
        else:
            self.pub_x += ex * (lim / d)
            self.pub_y += ey * (lim / d)
        self._check_backlog(d)

    def _check_backlog(self, d: float) -> None:
        """A correction backlog that never drains means GPS and LIO disagree
        systematically (uncalibrated GPS antenna lever arm, wrong datum, bad
        yaw_offset) — the rate limit hides the jump but not the disagreement."""
        if d > 5.0 and not self._warned_backlog:
            self._warned_backlog = True
            self.get_logger().warn(
                f'correction backlog {d:.1f} m: GPS and LIO disagree by more '
                f'than the rate limit can absorb quickly '
                f'({d / max(self.max_corr_speed, 1e-6):.0f}s to drain)')
        elif d < 1.0:
            self._warned_backlog = False

    def _broadcast_from_state(self, lio):
        # map<-odom rotates by yaw_offset, so p_map = R * p_odom + t.
        # For base_link: p_gps(map,ENU) = R * lio(odom) + t  =>  t = p - R*lio.
        # p here is the rate-limited published position, not the raw estimate.
        rx = self._c * lio[0] - self._s * lio[1]
        ry = self._s * lio[0] + self._c * lio[1]
        self.broadcast(self.pub_x - rx, self.pub_y - ry, self.yaw_offset)

    def broadcast(self, x, y, yaw):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = 0.0
        # Yaw-only correction (odom(LIO) -> ENU); roll/pitch stay with LIO.
        tf.transform.rotation.z = math.sin(yaw * 0.5)
        tf.transform.rotation.w = math.cos(yaw * 0.5)
        self.br.sendTransform(tf)

    # -------------------------------------------------------------- diagnostics
    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'gps_src': self.gps_src,
            'fix_status': self.fix_status,
            'fix_h_var': (round(self.fix_h_var, 4)
                          if self.fix_h_var is not None else None),
            'r_used': round(self.last_r, 4),
            'k': round(self.last_k, 5),
            'innov': round(self.last_innov, 3),
            'var_x': round(self.var_x, 4),
            'var_y': round(self.var_y, 4),
            'reject_x': self.rej['x']['count'],
            'reject_y': self.rej['y']['count'],
            'steps': self.step_count,
            'step_hold': self._step_distrusted(self._now()),
            'correction_backlog': round(
                math.hypot(self.px - self.pub_x, self.py - self.pub_y), 3),
            'yaw_offset_deg': round(math.degrees(self.yaw_offset), 2),
            'holding': self.holding,
        })
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = MapOdomFromGps()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM from launch/tmux teardown arrives as ExternalShutdownException;
        # catching it keeps the shutdown log clean (same as fake_gps_publisher).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
