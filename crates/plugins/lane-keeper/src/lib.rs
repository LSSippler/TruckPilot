//! Lane-Keeper plugin â€” dual-mode: route-following (Catmull-Rom) or vision-based.
//!
//! ## Mode selection
//! Set `plugin.lane_keeper.mode` on the Blackboard to `"vision"` or `"route_following"`.
//! Default (if key absent): `RouteFollowing` â€” preserves all existing behaviour.
//!
//! ## Vision mode â€” 5-Level Fallback Cascade (DS1 spec)
//! Level 0: Normal vision PID.  Level 1: Single-lane extrapolation.
//! Level 2: Confidence-drop (EMA).  Level 3: Heading-hold.  Level 4: Disengage.
//!
//! ## Route-following mode
//! Reads waypoints from `router.waypoints`.  Speed-adaptive look-ahead.
//! `look_ahead_m = BASE_LOOK_AHEAD + speed_kmh * SPEED_FACTOR`

mod extrapolation;
mod fallback;
mod heading_hold;

use extrapolation::{extrapolate_center, LaneWidthState};
use fallback::FallbackState;
use heading_hold::{wrap_angle, HeadingHoldState};
use std::collections::HashMap;
use std::sync::Arc;
use truckpilot_map_parser::{
    arc_length::{
        arc_length, build_all_luts, build_forward_adjacency, build_lut, lookahead, t_at_arc_length,
        ArcLengthLUT,
    },
    spline::{evaluate, evaluate_tangent, HermiteSegment, Vec3},
    spline_index::{HeadingFilteredHit, NearestHit},
    SplineIndex,
};
use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

// â”€â”€ Priorities â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const PRIORITY_NORMAL: i32 = 50;
const PRIORITY_LEVEL4: i32 = 200;

// â”€â”€ Route-following constants â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const BASE_LOOK_AHEAD: f64 = 5.0;
const SPEED_FACTOR: f64 = 0.5;
const WAYPOINT_REACH_M: f64 = 5.0;
/// Right-lane offset (Rechtsfahrgebot). Mirrors lane-follower LANE_OFFSET_RIGHT_M.
const LANE_OFFSET_RIGHT_M: f64 = 1.875;

/// Phase 2d: Max Hops die der Spline-Lookahead entlang der Route walkt, bevor er
/// am Segmentende clampt (Zyklus-/Nicht-VorrÃ¼ck-Sicherung, analog Lane-Follower).
const SPLINE_LOOKAHEAD_MAX_HOPS: usize = 64;
/// Phase 2d: Max laterale Distanz Truckâ†”Hop-Segment (Centerline), bevor der
/// Spline-Pfad auf Catmull zurÃ¼ckfÃ¤llt (Schutz gegen verirrte Projektionen,
/// hÃ¤rtet Finding A zusÃ¤tzlich ab). GroÃŸzÃ¼gig: korrekt in der rechten Spur
/// liegt der Truck bis ~17m (5-spurig) von der Centerline.
///
/// Phase 2h-Befund3-Fix: 40 â†’ 50 m. Diagnose an Kreuzung 1051105/1051103 zeigte:
/// der korrekte On-Route-Hop 1051103 liegt im Eintrittsfenster (t=6.3â€”9.8)
/// durchgehend bei 40.15â€”41.77 m â€” KNAPP Ã¼ber 40 m. Der dist-Filter verwarf ihn
/// â†’ Fallback auf globalen nearest â†’ Abbieger 1051105 â†’ off_route â†’ Catmull. 50 m
/// deckt die gemessenen 42 m mit Marge ab, bleibt aber moderat genug um legitime
/// nahe Off-Route-Segmente (echter Spurwechsel) nicht zu Ã¼berstimmen.
/// 50 â†’ 65 m: Live-Diag zeigte On-Route-Segment bei 51.12 m (1.12 m Ã¼ber Gate).
const MAX_HOP_PROJECTION_DIST_M: f32 = 65.0;
/// Dual-CW-Guard: Route-Treffer ablehnen wenn der physisch nÃ¤chste Treffer
/// < DUAL_CW_REJECT_RATIO * route_dist. Verhindert Vollausschlag-Lenkung auf
/// Gegenfahrbahn/Parallelfahrspur die ~40m entfernt ist (Dual Carriageway).
const DUAL_CW_REJECT_RATIO: f32 = 0.30;
/// Dual-CW-Guard: nur aktiv wenn physischer Treffer â‰¤ diesem Wert. Stellt sicher,
/// dass der Guard nicht bei echten >12m-Abweichungen unkontrolliert feuert.
const PHYSICAL_CLOSE_DIST_M: f32 = 12.0;
/// Phase 2h-Wurzelfix: max. Richtungsknick (Grad) an einem Walk-Hop-Ãœbergang,
/// bevor der Lookahead-Walk STOPPT statt Ã¼ber den Knick auf ein abknickendes
/// Segment zu zielen. Gemessen als |Î”heading| zwischen auslaufender Tangente
/// (cur, t=1.0) und einlaufender Tangente (Kandidat, t=0.0). Normale Kurven
/// haben an der Naht <~20Â° (kontinuierliche Hermite-Tangenten), Abzweig-Knicke
/// >40Â° (Diag4: 66Â°). Per Blackboard `plugin.lane_keeper.kink_stop_deg` justierbar.
const KINK_STOP_DEG: f32 = 35.0;
/// Phase 2h-Wurzelfix: hÃ¤lt der Kink-Stop auf DEMSELBEN Hop lÃ¤nger als dieses
/// Fenster an (Truck kommt nicht am Knick vorbei), fÃ¤llt der Lane-Keeper auf den
/// Catmull-Pfad zurÃ¼ck (geglÃ¤ttete Route-Waypoints runden die Kreuzung) statt
/// dauerhaft kurz vor dem Knick zu kleben. Dead-Lock-Schutz.
const KINK_STUCK_FALLBACK_S: f64 = 4.0;
/// Phase 2h-Wurzelfix v2 (Richtung B): interne KrÃ¼mmung (Grad, t=0â†’final_t) eines
/// PREFAB-Landesegments, ab der der Lane-Keeper auf Catmull Ã¼bergeht statt heading-only
/// auf einen fernen Kurvenpunkt zu zielen (Diag5: 60â€“100Â° â†’ herr-Spike â†’ AutoReplan).
/// Catmull rundet die Kreuzung (empirisch herr <0.5). Per Blackboard
/// `plugin.lane_keeper.prefab_curve_fallback_deg` justierbar.
const PREFAB_CURVE_FALLBACK_DEG: f32 = 40.0;
/// Austritts-Hysterese: erst zurÃ¼ck auf Spline wenn die interne KrÃ¼mmung wieder
/// unter (Schwelle âˆ’ Margin) liegt. Verhindert Flackern an der Schwelle.
const PREFAB_CURVE_EXIT_MARGIN_DEG: f32 = 10.0;
/// Phase 2h-Befund4-Fix: PlausibilitÃ¤ts-Obergrenze (Grad) fÃ¼r den prefab_curve_latch
/// (Latch-EINTRITT). Ãœber diesem Wert gilt die gemessene interne KrÃ¼mmung als
/// mis-orientiertes Junction-Node-Tangenten-Artefakt (m0 = From-Node-Quaternion-
/// Forward zeigt an Kreuzungen bis ~131Â° vom echten Connector-Verlauf weg). In
/// diesem Regime feuert der Latch NICHT â€” das route-aware-Spline-Tracking hat das
/// korrekte On-Route-Segment ohnehin und ist besser als Catmull (das heading-only
/// in die Kreuzungsbeule zielt).
///
/// Default 100Â° (Reviewer-Befund4): legitime enge Connector (Kreisverkehr-Segmente,
/// Autobahnrampen, scharfe Stadtabbieger) kÃ¶nnen intern 95â€“105Â° drehen und sollen
/// noch per Catmull gerundet werden; das beobachtete Artefakt liegt bei 131Â° (>>100).
/// Per Blackboard `plugin.lane_keeper.intk_plausible_max_deg`.
const INTK_PLAUSIBLE_MAX_DEG: f32 = 100.0;
/// Phase 2h-Befund4-Fix: Hysterese-Margin am Cap (Grad). Eintritt nur bis
/// `INTK_PLAUSIBLE_MAX_DEG` (100Â°); ein bereits aktiver Latch wird erst Ã¼ber
/// `INTK_PLAUSIBLE_MAX_DEG + diese Margin` (110Â°) beendet. Das Totband [100,110]Â°
/// verhindert Frame-zu-Frame-Flackern (Latchâ†”Spline) wenn intK durch
/// Quaternion-Rauschen (Â±2â€“5Â°) um den Cap oszilliert.
const INTK_PLAUSIBLE_MAX_EXIT_MARGIN_DEG: f32 = 10.0;
/// Phase 2h-Befund2-Fix: absolute Untergrenze fÃ¼r den Catmull-Kurven-Lookahead (m).
/// Inverse-Square-Skalierung: factor = (PREFAB_CURVE_FALLBACK_DEG / internal_kink_deg)^2
/// bei 60Â°: (40/60)^2=0.44 â†’ 12.5m â†’ ~5.5m; bei 80Â°: (40/80)^2=0.25 â†’ ~3.1m.
/// BB-Override: `plugin.lane_keeper.catmull_min_look_ahead_m`.
const CATMULL_CURVE_MIN_LOOK_AHEAD: f64 = 3.0;
/// Phase 2h-Befund2-Fix (Route-Hop-Limit): maximale Anzahl Route-Hops die der
/// Catmull-Lookahead-Walk vorausschauen darf. Verhindert, dass der Walk um
/// Kreuzungskurven herum auf Wegpunkte jenseits der Kreuzungsmitte zielt.
/// BB-Override: `plugin.lane_keeper.catmull_max_route_hops`.
const CATMULL_MAX_ROUTE_HOPS: usize = 2;
/// Phase 2h-Befund3-Fix: Anzahl R-tree-kNN-Kandidaten, die der route-aware
/// nearest-Query durchsucht, um den geometrisch nÃ¤chsten ON-ROUTE-Treffer zu
/// finden. HÃ¶her als die globale Query (8), weil der On-Route-Ast an Kreuzungen
/// etwas weiter weg liegen kann als der geometrisch nÃ¤chste Abbieger. R-tree-kNN
/// (`take(N)`), KEIN Linearscan Ã¼ber alle Segmente.
const ROUTE_NEAREST_CANDIDATES: usize = 256;
/// NavCurve-Prioritaets-Margin: NavCurve gewinnt ueber Road-Segment wenn
/// dist_navcurve <= dist_road + MARGIN. Topologie schlaegt Proximity-Wettbewerb
/// (ETS2LA-Ansatz). 50 m = ~2 s bei 90 km/h Autobahn.
const NAVCURVE_PRIORITY_MARGIN_M: f32 = 50.0;
/// Laufender Reanchor: maximal so viele Hops rÃ¼ckwÃ¤rts erlaubt (Jitter-Toleranz).
const REANCHOR_BACKWARD_WINDOW: usize = 2;
/// Laufender Reanchor: maximal so viele Hops vorwÃ¤rts erlaubt.
const REANCHOR_FORWARD_WINDOW: usize = 32;

// â”€â”€ PID defaults â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const DEFAULT_KP: f64 = 0.8;
const DEFAULT_KI: f64 = 0.1;
const DEFAULT_KD: f64 = 0.3;

/// Block-2: max heading error (radians) before lane-keeper suspends steering.
/// ~80Â°: covers normal curves/lane-changes (â‰¤45Â°) but blocks clear mismatch cases.
pub(crate) const HEADING_MISMATCH_THRESHOLD_RAD: f64 = 1.4;

/// Phase 2h-Safety: geschwindigkeits-gerampte Bremsung wenn die LenkautoritÃ¤t
/// verloren ist (Heading-Stage AutoReplan/Disengaging). brake = speed_ms * PER_MS,
/// geklemmt â€” sanft bei langsamer Fahrt, stÃ¤rker bei hÃ¶herem Tempo. Wird der Truck
/// unter der Bremsung langsamer, sinkt die Bremse â†’ sauberes Ausrollen.
const SAFETY_BRAKE_PER_MS: f64 = 0.072; // ~0.30 bei 15 km/h, ~0.60 bei 30 km/h
const SAFETY_BRAKE_MIN: f64 = 0.15; // Halte-Bremse bis Stillstand
const SAFETY_BRAKE_MAX: f64 = 0.80; // keine Vollbremsung (Auffahrschutz)
/// Phase 2h-Safety: AutoReplan ist erholbar â€” erst nach diesem Zeitfenster ohne
/// Erholung sauber disengagen. Disengaging (Latch) eskaliert SOFORT (kein Timeout).
const SAFETY_AUTOREPLAN_DISENGAGE_S: f64 = 15.0;

// â”€â”€ Schritt 2: Junction-Failsafe â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Verliert lane_keeper an einer ERKANNTEN Junction (lane_follower.junction_detected)
// die LenkautoritÃ¤t (heading_stage â†’ Disengaging), NICHT sofort in Off fallen â€”
// sonst rollt der Truck ungebremst geradeaus in die kurvige Junction (Leitplanke).
// Stattdessen kurzes Grace-Fenster mit harter Bremse: nimmt Tempo raus und gibt
// lane_keeper Zeit, route_hit Ã¼ber die On-Route-NavCurve (Fix C) wiederzufinden.
// Greift NUR an erkannten Junctions; ohne Junction-Signal bleibt der sofortige
// Disengage (User-/echtes-Off-Route) unverÃ¤ndert.
/// Dauer des Junction-Grace-Fensters, bevor doch disengaged wird.
const JUNCTION_FAILSAFE_GRACE_S: f64 = 1.5;
/// Mindest-Bremswert im Grace-Fenster (entschlossenes Tempo-Raus statt freiem Rollen).
const JUNCTION_FAILSAFE_BRAKE: f64 = 0.60;

/// Block-2: max steering change per tick.
const STEERING_MAX_DELTA_PER_TICK: f64 = 0.1;

/// Ticks before Level-4 brake is lifted.
const L4_BRAKE_TICKS: u64 = 50;

// â”€â”€ NearestSpline-mode constants (routerless lane-following, Weg B) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
/// Einheitliche Heading-Toleranz (Grad) fÃ¼r Engage UND Segment-Auswahl im
/// NearestSpline-Modus. Bewusst identisch zum SplineIndex-Heading-Dot (cos 60Â° =
/// 0.5) und zum lookahead()-Junction-Pick â†’ keine Zwei-Stufen-Logik, kein stilles
/// â€žengaged aber lenkt nicht". Ãœber 60Â° â†’ kein Segment / None-Steering.
const NEAREST_HEADING_TOL_DEG: f32 = 60.0;
/// Chain-Weiterschaltung: am Segmentende werden forward_adj-Nachfolger nur akzeptiert,
/// wenn ihr Entry-Heading < dieser Toleranz (Grad) vom Truck-Heading abweicht. Bewusst
/// WEITER als die Engage-/Spatial-Toleranz (60Â°), damit echte Kurven-Nachfolger (60â€“90Â°)
/// der Chain folgen kÃ¶nnen; Gegenrichtung (â‰ˆ180Â°) und Querverkehr (â‰¥90Â°) bleiben strukturell
/// drauÃŸen. Greift kein Nachfolger â†’ Spatial-Re-Acquisition (60Â°-Filter) statt Sprung.
const CHAIN_SUCCESSOR_TOL_DEG: f32 = 90.0;
/// VorwÃ¤rts-Fortschritt: ab diesem t auf dem aktuellen Segment Ã¼ber forward_adj auf den
/// Nachfolger weiterschalten (Chain-Advance statt per-Frame-Spatial-Suche).
const NEAREST_FWD_PROGRESS_T: f32 = 0.85;
/// Prefab-Bias: NavCurves bekommen in der Auswahl diesen Distanz-Bonus (m) â€” die
/// echte Fahrkurve gewinnt gegen den geraden Road-Chord in der Junction-Zone.
const NEAREST_PREFAB_BIAS_M: f32 = 10.0;
/// Engage-Gate: der Truck muss innerhalb dieser Distanz (m) zu einem heading-
/// kompatiblen Segment stehen, damit `lane_keeper.engage_allowed=true`.
/// Phase â€žVon-Ã¼berall-Engage": von 20 m auf 45 m geÃ¶ffnet â€” Capture-Modus +
/// Low-Speed-Steer-Cap + Stuck-Watchdog machen das Einfangen aus der Ferne sanft.
/// Bewusst 5 m unter `NEAREST_QUERY_RADIUS_M` (50 m): ein Treffer an der Radius-
/// Kante flackert im R-Tree-Query â€” die Marge hÃ¤lt engage_allowed stabil.
const NEAREST_ENGAGE_DIST_M: f32 = 45.0;
/// Engage-Gate: maximale laterale Ablage (m) zur Soll-Linie. Additiv zum Heading-Gate â€”
/// beide mÃ¼ssen erfÃ¼llt sein. Live-tunbar via `[plugins.lane-keeper] engage_max_lateral_m`
/// in truckpilot.toml (BB-Key: `lane_keeper.engage_max_lateral_m`).
const DEFAULT_ENGAGE_MAX_LATERAL_M: f32 = 3.0;
/// Such-Radius (m) fÃ¼r den heading-gefilterten Nearest-Query.
const NEAREST_QUERY_RADIUS_M: f32 = 50.0;
/// Chain-Advance-Distanz (m): erst weiterschalten, wenn die RESTBOGENLÃ„NGE des aktuellen
/// Segments unter diesem Wert liegt â€” statt bei fixer t-Fraktion. Auf langen Segmenten feuerte
/// `t â‰¥ NEAREST_FWD_PROGRESS_T` (0.85) Dutzende Meter vor dem Knoten (z.B. 52 m bei 347 m
/// Segment), der kurze Nachfolger lag dann noch auÃŸerhalb `NEAREST_QUERY_RADIUS_M` (50 m) â†’
/// Truck projizierte sofort â€žoff" â†’ `chain_broken` â†’ Re-Acquire â†’ 2-Zyklus-Flackern.
/// MUSS `< NEAREST_QUERY_RADIUS_M` sein, damit der Nachfolger beim Schalten in Reichweite ist
/// (35 m â†’ ~15 m Marge gegen KrÃ¼mmung/Tick-GranularitÃ¤t). Wirkt NUR bei Segmenten lÃ¤nger als
/// `NEAREST_ADVANCE_DIST_M / (1 âˆ’ NEAREST_FWD_PROGRESS_T)` â‰ˆ 233 m; kÃ¼rzere behalten exakt das
/// alte `t â‰¥ 0.85`-Verhalten (das t-Gate bindet dort, nicht das Distanz-Gate).
const NEAREST_ADVANCE_DIST_M: f32 = 35.0;
/// Lookahead-Untergrenze (m) NUR im NearestSpline-Pfad. Der geschwindigkeits-
/// abhÃ¤ngige Lookahead `BASE_LOOK_AHEAD + v*SPEED_FACTOR` (5 + v*0.5) fÃ¤llt bei
/// Kriechtempo auf 5 m â†’ kurzer Hebel â†’ hohe effektive LenkverstÃ¤rkung â†’ Oszillation
/// (Pendeln 0â€“6 km/h, Vollanschlag). Dieser Floor hÃ¤lt den Lookahead im Stand/Kriech-
/// tempo bei â‰¥12 m; oberhalb ~14 km/h ist `v*0.5+5 > 12` und der Floor greift nicht
/// mehr (transparent bei hÃ¶herer Geschwindigkeit). RouteFollowing nutzt diesen Floor
/// NICHT â€” dort bleibt `BASE_LOOK_AHEAD` (5 m) unverÃ¤ndert.
const NEAREST_MIN_LOOK_AHEAD: f64 = 12.0;
/// Cross-Track-Term v3 â€” Default-Gain `K_CT`. Zur Laufzeit per
/// `set-gain nearest_xtrack_k` (BB `plugin.lane_keeper.nearest_xtrack_k`)
/// justierbar; sinnvoller Tuning-Bereich 0.3â€“2.0. `K_CT=0` â†’ reine Heading-Regelung.
const NEAREST_XTRACK_K_DEFAULT: f64 = 0.5;
/// Harte Obergrenze (rad) fÃ¼r den Cross-Track-Anteil (P + I, gemeinsam). ZusÃ¤tzliche
/// Sicherung neben der strukturellen Lookahead-Nenner-Bindung â€” `|xtrack|` kann nie
/// Vollausschlag erzwingen.
const XTRACK_MAX_RAD: f64 = 1.0;
/// Cross-Track v3 â€” Integral-Gain `K_CT_I` (Option 3: getrennte Cross-Track-/Heading-Regler).
/// Per `set-gain nearest_xtrack_ki` (BB `plugin.lane_keeper.nearest_xtrack_ki`) justierbar;
/// sinnvoller Bereich ~0.03â€“0.15. SchlieÃŸt den v3-Regel-Restfehler: der reine atan2-P-Anteil
/// findet ein Gleichgewicht bei e_lat â‰ˆ look/k (â‰ˆ2.7 m gemessen, skaliert mit look/k), erst der
/// eigene Integralpfad zieht e_lat stationÃ¤r â†’ 0. `K_CT_I=0` â†’ reines v3-P-Verhalten (Regression).
const NEAREST_XTRACK_KI_DEFAULT: f64 = 0.08;
/// Anti-Windup-Gate (m): das Cross-Track-Integral integriert NUR, solange `|e_lat|` unter
/// dieser Schwelle liegt; darÃ¼ber wird der Speicher auf 0 gehalten. Verhindert Windup bei
/// groÃŸer Anfangsabweichung (Stillstand-Engage-Artefakt, Re-Acquisition-Sprung).
const CT_INTEG_GATE_M: f64 = 3.0;
/// Clamp (Betrag) des Cross-Track-Integralspeichers [mÂ·s]. Begrenzt den maximalen I-Beitrag
/// (= `CT_INTEG_MAX Â· K_CT_I` rad) gegen Windup; der finale Cross-Track-Wert bleibt zusÃ¤tzlich
/// durch `XTRACK_MAX_RAD` gedeckelt.
const CT_INTEG_MAX: f64 = 5.0;

// â”€â”€ Capture-Modus + Anti-Stall (Einfang-Diagnose 2026-06-10, NUR NearestSpline) â”€â”€
/// Low-Speed-Steer-Cap: minimaler erlaubter |steer| bei v=0. Verhindert das
/// Volleinschlag-Verkeilen im Stand (selbsthaltender SchrÃ¤glage-Stillstand:
/// bei vâ‰ˆ0 sind alle Regler-EingÃ¤nge eingefroren, das volle Lenkrad verhindert
/// das Wieder-Anfahren). Nachfolger der entfernten SLOW_SPEED_GUARD_MS-Infra,
/// jetzt als AKTIVER Pfad.
const NEAREST_STEER_CAP_MIN: f64 = 0.3;
/// Geschwindigkeit (m/s) ab der der Low-Speed-Cap voll geÃ¶ffnet ist (Cap = 1.0).
/// Linear interpoliert: cap = 0.3 + (v/3)Â·0.7. Oberhalb 3 m/s transparent.
const NEAREST_STEER_CAP_FULL_SPEED_MS: f64 = 3.0;
/// Stuck-Watchdog: Geschwindigkeit (m/s) unterhalb derer â€žsteht" gilt.
const STUCK_SPEED_MS: f64 = 0.5;
/// Stuck-Watchdog: Lenk-ABSICHT (|heading_P + xtrack|, VOR den Caps) oberhalb derer
/// der Stillstand als verkeilt zÃ¤hlt. Bewusst die ungecappte Absicht statt des
/// Outputs: der Low-Speed-Cap drÃ¼ckt den Output unter 0.3 â€” der Watchdog soll
/// trotzdem erkennen, dass der Regler im Stand â€žvoll ziehen will".
const STUCK_STEER_MIN: f64 = 0.4;
/// Stuck-Watchdog: Ticks bis zur AuslÃ¶sung (~1 s bei 50 Hz).
const STUCK_TICKS: u32 = 50;
/// Stuck-Watchdog: Ticks im Recovery-Zustand ohne Fortschritt bis zum Disengage.
/// Nach ~1 s Recovery noch ~3 s ohne Bewegung â†’ autopilot.disengage_requested.
const STUCK_DISENGAGE_TICKS: u32 = 150;
/// Stuck-Recovery: harter Steer-Cap solange der Watchdog aktiv ist â€” nahe geradeaus,
/// damit der Truck anfahren kann statt gegen den Einschlag zu drÃ¼cken.
const STUCK_RELAX_CAP: f64 = 0.15;
/// Stuck-Watchdog: Geschwindigkeit (m/s) ab der ZÃ¤hler + Recovery zurÃ¼ckgesetzt
/// werden (Truck rollt wieder â†’ normale Regelung).
const STUCK_RESET_SPEED_MS: f64 = 1.0;
/// Capture-Modus: |e_lat| (m), oberhalb dessen Capture aktiv ist/bleibt.
const CAPTURE_EXIT_ELAT_M: f64 = 1.5;
/// Capture-Modus: |heading_err| (Grad), oberhalb dessen Capture aktiv ist/bleibt.
const CAPTURE_EXIT_HEADING_DEG: f64 = 10.0;
/// Capture-Modus: Steer-Cap wÃ¤hrend des Einfangens (sanftes Eindrehen, kein
/// Quer-Einlenken neben die Bahn).
const CAPTURE_STEER_CAP: f64 = 0.5;
/// Capture-Modus: Tempoziel (km/h), publiziert als `lane_keeper.capture_speed_target_kmh`
/// â†’ zusÃ¤tzlicher min-Eingang in `compute_target_speed` des Speed-Controllers.
const CAPTURE_SPEED_TARGET_KMH: f64 = 20.0;
/// Capture-Exit-Hysterese: so viele Ticks mÃ¼ssen BEIDE Exit-Bedingungen
/// (|e_lat| â‰¤ 1.5 m UND |heading_err| â‰¤ 10Â°) in Folge erfÃ¼llt sein, bevor Capture
/// endet â€” kein Flackern an der Schwelle.
const CAPTURE_EXIT_STABLE_TICKS: u32 = 25;

/// Low-Speed-Steer-Cap (Task 1): lineare Ã–ffnung von `NEAREST_STEER_CAP_MIN` (v=0)
/// auf 1.0 (v â‰¥ `NEAREST_STEER_CAP_FULL_SPEED_MS`). Pure Funktion (testbar).
fn low_speed_steer_cap(speed_ms: f64) -> f64 {
    (NEAREST_STEER_CAP_MIN
        + (speed_ms.max(0.0) / NEAREST_STEER_CAP_FULL_SPEED_MS) * (1.0 - NEAREST_STEER_CAP_MIN))
        .clamp(NEAREST_STEER_CAP_MIN, 1.0)
}

// â”€â”€ Mode enum â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum LaneKeeperMode {
    #[default]
    RouteFollowing,
    Vision,
    /// Routerless lane-following: folgt der lokalen Spline-Geometrie unter dem Truck
    /// ohne Router-Route (lane_only-Engage). Koexistiert mit RouteFollowing.
    NearestSpline,
    Off,
}

// â”€â”€ Plugin struct â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

pub struct LaneKeeperPlugin {
    pid: Pid,

    // â”€â”€ Route-following fields â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    waypoints: Vec<[f64; 2]>,
    progress_idx: usize,
    subdivisions: usize,
    last_gains: (f64, f64, f64),
    last_waypoints_hash: u64,
    previous_steering_out: f64,
    heading_stage: Option<String>,
    previous_heading_stage: Option<String>,

    // â”€â”€ Vision-mode fields â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    mode: LaneKeeperMode,
    fallback: FallbackState,
    extrapolator: LaneWidthState,
    heading_hold: HeadingHoldState,
    /// Heading captured at Active-session start (for Block-2 guard in vision mode).
    engagement_heading: Option<f64>,
    /// Tick at which Level-4 was entered (None = not in L4).
    level_4_entered_at_tick: Option<u64>,
    /// Monotonic counter across all tick_request calls.
    tick_count: u64,
    /// Whether the last vision tick was in the Active state (for transition detection).
    was_active: bool,

    // â”€â”€ Phase 2c/2d: SplineIndex Route-Geometrie â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    index: Option<Arc<SplineIndex>>,
    router_graph: Option<Arc<RouterGraph>>,
    seg_by_from_to: HashMap<(u64, u64), usize>,
    /// Fix C: NavCurve-Segmente (Index >= `road_seg_count`) adressiert nach
    /// `(from_uid, to_uid)`. Anders als Road (1:1) kann ein Knotenpaar mehrere
    /// NavCurves tragen (z.B. parallele Junction-Durchfahrten / Lanes) â†’ `Vec`.
    /// In `on_load` neben `seg_by_from_to` befÃ¼llt; treibt die On-Route-NavCurve-
    /// Aufnahme in `cached_route_seg_set`, damit lane_keeper an Junctions einen
    /// gÃ¼ltigen `route_hit` bekommt statt in heading_stage/Disengage zu fallen.
    navcurve_by_from_to: HashMap<(u64, u64), Vec<usize>>,
    /// Fix C: Anzahl Road-Segmente vorne im Index; NavCurves beginnen hier.
    /// Aus `ctx.spline_index_road_seg_count`. FÃ¼r `chosen_segment_is_navcurve`.
    road_seg_count: usize,
    /// Fix C: Anzahl On-Route-NavCurves in `cached_route_seg_set` (Diagnostik).
    cached_route_navcurve_count: usize,
    // Diagnose-Felder fuer NavCurve-Force-Scan (jeden Tick ueberschrieben).
    dbg_force_candidates: u32,
    dbg_force_proj_ok:    u32,
    dbg_force_t_filtered: u32,
    dbg_force_best_dist:  f32,
    cached_route_node_ids: Vec<u64>,
    cached_route_hash: u64,
    /// Phase 2h-Befund3-Fix: Menge der Segment-Indizes, die auf der aktuellen
    /// Route-Hop-Sequenz liegen. Aus `seg_by_from_to` + `cached_route_node_ids`
    /// gebaut, nur bei Routenwechsel neu (kein Per-Tick-Rebuild). Treibt den
    /// route-aware nearest-Query: an Kreuzungen wird der On-Route-Ast bevorzugt
    /// statt des geometrisch nÃ¤chsten Abbiegers.
    cached_route_seg_set: std::collections::HashSet<usize>,
    /// Phase 2h-Befund3-Fix: Route-Hash, fÃ¼r den `cached_route_seg_set` gebaut
    /// wurde. Rebuild genau dann, wenn != `cached_route_hash` (ein Rebuild pro
    /// Routenwechsel, kein Per-Tick-Rebuild â€” auch nicht bei legitim leerem Set,
    /// z.B. Ferry-Route). Bleibt stale, falls `seg_by_from_to` beim Wechsel noch
    /// leer war (Index nicht geladen) â†’ Rebuild greift, sobald die Quelle bereit ist.
    cached_route_seg_hash: u64,
    node_progress_idx: usize,
    /// true solange der Spline-Pfad im letzten Tick aktiv war (fÃ¼r sauberen
    /// Re-Sync des Catmull-progress_idx beim Ãœbergang Splineâ†’Catmull).
    was_spline_active: bool,
    /// Phase 2c/2d-Diagnose (read-only): einmaliges Flag, damit der
    /// route_miss-Sample-Log (erste 3 Hops + in_map) nur EINMAL feuert.
    route_miss_sample_logged: bool,
    /// Phase 2f-B-Diagnose (read-only): zÃ¤hlt JEDEN Eintritt in
    /// `try_spline_heading_error` (vor jeder Bedingung). WÃ¤chst er NICHT mit den
    /// Ticks â†’ Funktion wird gar nicht betreten (H1).
    reanchor_called_count: u64,
    /// Phase 2f-B-Diagnose (read-only): zÃ¤hlt, wie oft der Code bis zum
    /// tatsÃ¤chlichen `node_progress_idx`-Schreiben (re-anchor-Scan-Write) kommt.
    /// called wÃ¤chst aber reached NICHT â†’ eine Bedingung VOR dem Scan bricht ab.
    reanchor_reached_write: u64,
    /// Phase 2h-Diag (read-only): lane_offset_applied_m des letzten Ticks,
    /// um den Offset-Sprung (delta) an road/prefab/Catmull-Grenzen zu messen.
    prev_lane_offset_m: f32,
    /// Phase 2h-Diag (read-only): lateral_source des letzten Ticks,
    /// um Source-Wechsel (spline_roadâ†”prefabâ†”catmull) pro Tick zu detektieren.
    prev_lateral_source: String,

    /// Phase 2h-Safety: Sekunden (dt-akkumuliert) seit Eintritt in AutoReplan ohne
    /// Erholung. Tick-Rate-robust statt Frame-ZÃ¤hler. Reset bei Stage-Recovery.
    safety_autoreplan_secs: f64,
    /// Heading-Fehler (rad), gegen den der heading_mismatch-Safety-Gate prueft.
    /// WICHTIG: das ist NICHT der Steuer-Fehler (der zielt auf den fernen Lookahead-
    /// Punkt und spiked in Kurven). Hier steht der LOKALE Fehler (Truck-Heading vs
    /// Tangente des nearest-Segments) im Spline-Pfad bzw. der Lookahead-Fehler im
    /// Catmull-Fallback. So bremst der Truck nur bei echter Fehlausrichtung, nicht
    /// bloss weil der 15m-Lookahead um eine Kurve greift (ETS2LA-Ansatz: Heading
    /// gegen die nahe Centerline messen, nicht gegen den fernen Zielpunkt).
    heading_mismatch_herr_rad: f64,
    /// Schritt 2: akkumulierte Zeit im Junction-Failsafe-Grace (hart bremsen statt
    /// sofort disengage). Reset bei Recovery / Off / Re-Engage.
    junction_failsafe_secs: f64,

    /// Phase 2h-Wurzelfix: akkumulierte Sekunden anhaltenden Kink-Stops auf demselben
    /// Hop (dt-basiert). Reset sobald der Walk NICHT am Knick stoppt oder der Hop wechselt.
    kink_stuck_secs: f64,
    /// Der Hop (from,to) auf dem der Kink-Stop gerade akkumuliert.
    kink_stuck_hop: (u64, u64),

    /// Phase 2h-Wurzelfix v2: true solange der Catmull-Fallback wegen interner
    /// Prefab-KrÃ¼mmung aktiv ist (margin-basierte Hysterese). Latch Ã¼ber Ticks.
    prefab_curve_latched: bool,
    /// Phase 2h-Befund2-Fix: interne KrÃ¼mmung (Grad) zum Zeitpunkt des letzten
    /// prefab_curve_latched=true. Struct-Feld statt BB-Read (kein Stale-Risiko).
    prefab_curve_kink_deg: f64,

    /// H2-Catmull-Offset-Fix: der nearest-Segment-Index (in `index.segments`), der
    /// in `try_spline_heading_error` DIESEN Tick bestimmt wurde. `Some` nur wenn die
    /// nearest-Query diesen Tick lief (Reset auf `None` am Funktionsanfang). Der
    /// Catmull-Fallback liest daraus den per-Segment-`lane_offset_right_m` (statt des
    /// festen 1.875 m). `None` (kein frisches nearest, z.B. index_none/route_miss vor
    /// der Query) â†’ Default-Offset 1.875 m, NICHT 0 (sonst mittig auf 1-spurig).
    last_nearest_seg: Option<usize>,

    // â”€â”€ NearestSpline-Modus (routerless) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    /// Aktuell verfolgtes Segment im NearestSpline-Modus (fÃ¼r die Hysterese Ã¼ber
    /// Ticks). `None` = noch kein Segment gewÃ¤hlt / Tracking zurÃ¼ckgesetzt.
    nearest_seg_idx: Option<usize>,
    /// Offset-Vererbung (Fix 1): zuletzt gesehener ECHTER Road-`lane_offset_right_m`
    /// (gesetzt bei `Some(m)` ohne `is_prefab`). Metadatenlose `direction="prefab"`-
    /// LÃ¼ckensegmente (meta=None) erben diesen Wert statt auf `LANE_OFFSET_RIGHT_M`
    /// (1.875 m) zu fallen â†’ Soll-Linie bleibt Ã¼ber die kurze KreuzungslÃ¼cke kontinuierlich.
    /// `None` = noch kein echtes Road-Segment gesehen (frischer Engage / nach Re-Acquisition);
    /// dann greift der 1.875-m-Default als Fallback des Fallbacks. Reset zusammen mit
    /// `nearest_seg_idx` (Engage / Modus-Wechsel / Chain-Abriss).
    last_road_lane_offset_m: Option<f32>,
    /// Arc-Length-LUTs Ã¼ber ALLE Index-Segmente (Road + NavCurve). Lazy gebaut beim
    /// ersten NearestSpline-Tick (Route-Following zahlt nichts).
    nearest_luts: Vec<ArcLengthLUT>,
    /// Forward-Adjacency `from_uid â†’ Vec<seg_idx>` Ã¼ber alle Index-Segmente. Lazy.
    nearest_forward_adj: HashMap<u64, Vec<usize>>,
    /// true sobald `nearest_luts`/`nearest_forward_adj` gebaut wurden.
    nearest_index_built: bool,
    /// Cross-Track-Integralspeicher [mÂ·s] fÃ¼r den NearestSpline-PI-Regler (Option 3).
    /// Akkumuliert `e_latÂ·dt` nur innerhalb des Anti-Windup-Gates (`CT_INTEG_GATE_M`) und
    /// wird bei Disengage / Re-Acquisition / chain_broken / Modus-Wechsel auf 0 zurÃ¼ckgesetzt
    /// (sauberer Neustart, kein Alt-Integral). Nur vom NearestSpline-Pfad benutzt.
    xtrack_integ: f64,
    /// Laterales Engage-Gate (m). Truck muss â‰¤ diesem Wert von der nÃ¤chsten
    /// heading-kompatiblen Soll-Linie entfernt sein. Geladen aus truckpilot.toml.
    engage_max_lateral_m: f32,
    /// Stuck-Watchdog (Task 2): Ticks in Folge mit v < `STUCK_SPEED_MS` und
    /// Lenk-Absicht > `STUCK_STEER_MIN`. Reset bei v > `STUCK_RESET_SPEED_MS`,
    /// Disengage und Re-Acquisition.
    stuck_ticks: u32,
    /// true solange die Stuck-Recovery aktiv ist (Steer hart auf Â±`STUCK_RELAX_CAP`
    /// gecapt). Latch bis der Truck wieder rollt (v > `STUCK_RESET_SPEED_MS`).
    stuck_recovery: bool,
    /// Ticks seit Eintritt in stuck_recovery ohne Fortschritt. Nach
    /// `STUCK_DISENGAGE_TICKS` â†’ autopilot.disengage_requested.
    stuck_disengage_ticks: u32,
    /// Capture-Modus (Task 3): true solange der Truck die Soll-Linie noch einfÃ¤ngt
    /// (|e_lat| > 1.5 m oder |heading_err| > 10Â°). Steuert Capture-Steer-Cap und
    /// Capture-Tempoziel.
    capture_active: bool,
    /// Ticks in Folge, in denen BEIDE Capture-Exit-Bedingungen erfÃ¼llt sind
    /// (Hysterese gegen Flackern an der Schwelle).
    capture_exit_ticks: u32,
}

impl Default for LaneKeeperPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 2.0, 1.0),
            waypoints: Vec::new(),
            progress_idx: 0,
            subdivisions: 4,
            last_gains: (DEFAULT_KP, DEFAULT_KI, DEFAULT_KD),
            last_waypoints_hash: 0,
            previous_steering_out: 0.0,
            heading_stage: None,
            previous_heading_stage: None,
            mode: LaneKeeperMode::default(),
            fallback: FallbackState::new(),
            extrapolator: LaneWidthState::new(),
            heading_hold: HeadingHoldState::new(),
            engagement_heading: None,
            level_4_entered_at_tick: None,
            tick_count: 0,
            was_active: false,
            index: None,
            router_graph: None,
            seg_by_from_to: HashMap::new(),
            navcurve_by_from_to: HashMap::new(),
            road_seg_count: 0,
            cached_route_navcurve_count: 0,
            dbg_force_candidates: 0,
            dbg_force_proj_ok:    0,
            dbg_force_t_filtered: 0,
            dbg_force_best_dist:  -1.0,
            cached_route_node_ids: Vec::new(),
            cached_route_hash: 0,
            cached_route_seg_set: std::collections::HashSet::new(),
            cached_route_seg_hash: 0,
            node_progress_idx: 0,
            was_spline_active: false,
            route_miss_sample_logged: false,
            reanchor_called_count: 0,
            reanchor_reached_write: 0,
            prev_lane_offset_m: 0.0,
            prev_lateral_source: String::new(),
            safety_autoreplan_secs: 0.0,
            heading_mismatch_herr_rad: 0.0,
            junction_failsafe_secs: 0.0,
            kink_stuck_secs: 0.0,
            kink_stuck_hop: (0, 0),
            prefab_curve_latched: false,
            prefab_curve_kink_deg: 0.0,
            last_nearest_seg: None,
            nearest_seg_idx: None,
            last_road_lane_offset_m: None,
            nearest_luts: Vec::new(),
            nearest_forward_adj: HashMap::new(),
            nearest_index_built: false,
            xtrack_integ: 0.0,
            stuck_ticks: 0,
            stuck_recovery: false,
            stuck_disengage_ticks: 0,
            capture_active: false,
            capture_exit_ticks: 0,
            engage_max_lateral_m: DEFAULT_ENGAGE_MAX_LATERAL_M,
        }
    }
}

// â”€â”€ Shared helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

impl LaneKeeperPlugin {
    fn apply_gain_overrides(&mut self, ctx: &PluginContext) {
        let kp = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kp")
            .unwrap_or(DEFAULT_KP);
        let ki = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.ki")
            .unwrap_or(DEFAULT_KI);
        let kd = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kd")
            .unwrap_or(DEFAULT_KD);
        let next = (kp, ki, kd);
        if next != self.last_gains {
            self.pid.set_kp(kp);
            self.pid.set_ki(ki);
            self.pid.set_kd(kd);
            self.last_gains = next;
            tracing::info!("[lane-keeper] gains updated kp={kp} ki={ki} kd={kd}");
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kp", kp.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.ki", ki.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kd", kd.to_string());
        }
    }

    fn update_mode_from_blackboard(&mut self, ctx: &PluginContext) {
        let raw = ctx.blackboard.get("plugin.lane_keeper.mode");
        let next = match raw.as_deref() {
            Some("vision") => LaneKeeperMode::Vision,
            Some("nearest_spline") => LaneKeeperMode::NearestSpline,
            Some("off") => LaneKeeperMode::Off,
            _ => LaneKeeperMode::RouteFollowing,
        };
        if next != self.mode {
            tracing::info!("[lane-keeper] mode switch {:?} â†’ {:?}", self.mode, next);
            self.mode = next;
            self.pid.reset();
            self.fallback.reset();
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            self.previous_steering_out = 0.0;
            // NearestSpline-Tracking neu beginnen lassen.
            self.nearest_seg_idx = None;
            self.last_road_lane_offset_m = None;
            self.xtrack_integ = 0.0;
            // Capture/Stuck-Zustand rÃ¤umen (Reviewer-K1): der Disengage setzt den Mode
            // SOFORT auf route_following zurÃ¼ck â€” der Pre-Engage-Cleanup im NearestSpline-
            // Pfad lÃ¤uft dann nie wieder. Ohne dieses RÃ¤umen bliebe
            // lane_keeper.capture_speed_target_kmh=20 hÃ¤ngen und wÃ¼rde den nÃ¤chsten
            // Route-Engage dauerhaft auf 20 km/h capen (der Speed-Controller liest den
            // Key mode-agnostisch).
            self.stuck_ticks = 0;
            self.stuck_recovery = false;
            self.stuck_disengage_ticks = 0;
            self.capture_active = false;
            self.capture_exit_ticks = 0;
            ctx.blackboard.set("lane_keeper.stuck_recovery", "false");
            ctx.blackboard.set("lane_keeper.capture_active", "false");
            ctx.blackboard
                .set("lane_keeper.capture_speed_target_kmh", "-1.0");
        }
    }

    /// Phase 2h-Safety: einheitliche VerzÃ¶gerungs-Request bei Verlust der
    /// LenkautoritÃ¤t. `cause` = Diagnose-Label (z.B. "autoreplan", "heading_mismatch",
    /// "disengaging"). `immediate_disengage`=true â†’ sofort disengage anfordern (Latch-
    /// Stage). Sonst dt-Akkumulation + Disengage nach SAFETY_AUTOREPLAN_DISENGAGE_S.
    #[allow(clippy::too_many_arguments)]
    fn safety_brake_request(
        &mut self,
        cause: &str,
        safety_state: &str,
        immediate_disengage: bool,
        speed_ms: f64,
        dt: f64,
        err: f64,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let mut brake = (speed_ms * SAFETY_BRAKE_PER_MS).clamp(SAFETY_BRAKE_MIN, SAFETY_BRAKE_MAX);
        self.previous_steering_out = 0.0;
        self.pid.reset();
        self.xtrack_integ = 0.0;

        let mut junction_failsafe = false;
        if immediate_disengage {
            // Schritt 2: Junction-Failsafe. An einer ERKANNTEN Junction NICHT sofort
            // disengagen (sonst rollt der Truck ungebremst geradeaus in die Leitplanke).
            // Stattdessen kurzes Grace-Fenster mit harter Bremse; erst danach disengage.
            // Ohne Junction-Signal bleibt der sofortige Disengage unverÃ¤ndert.
            let at_junction = ctx
                .blackboard
                .get("lane_follower.junction_detected")
                .as_deref()
                == Some("true");
            if at_junction && self.junction_failsafe_secs < JUNCTION_FAILSAFE_GRACE_S {
                self.junction_failsafe_secs += dt;
                junction_failsafe = true;
                brake = brake.max(JUNCTION_FAILSAFE_BRAKE);
                tracing::warn!(
                    "[lane-keeper] {cause}: junction failsafe grace {:.2}s â€” hard brake, no disengage yet",
                    self.junction_failsafe_secs
                );
            } else {
                ctx.blackboard.set("autopilot.disengage_requested", "true");
                tracing::warn!("[lane-keeper] {cause}: lane authority lost â†’ immediate disengage");
            }
        } else {
            self.safety_autoreplan_secs += dt;
            if self.safety_autoreplan_secs > SAFETY_AUTOREPLAN_DISENGAGE_S {
                ctx.blackboard.set("autopilot.disengage_requested", "true");
                tracing::warn!(
                    "[lane-keeper] {cause} hung {:.1}s â†’ disengage",
                    self.safety_autoreplan_secs
                );
            }
        }
        ctx.blackboard.set(
            "lane_keeper.junction_failsafe_active",
            junction_failsafe.to_string(),
        );

        ctx.blackboard.set("lane_keeper.returned_none", "false");
        ctx.blackboard
            .set("lane_keeper.null_steer_cause", "safety_brake_active");
        ctx.blackboard
            .set("lane_keeper.skip_reason", "heading_stage");
        ctx.blackboard.set("lane_keeper.safety_state", safety_state);
        ctx.blackboard
            .set("lane_keeper.safety_brake", format!("{brake:.4}"));
        ctx.blackboard.set(
            "lane_keeper.safety_autoreplan_secs",
            format!("{:.2}", self.safety_autoreplan_secs),
        );
        ctx.blackboard
            .set("lane_keeper.steering_suppressed", "true");
        ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
        ctx.blackboard
            .set("lane_keeper.error_rad", format!("{err:.6}"));
        ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.steering_rate_limited", "false");
        ctx.blackboard
            .set("lane_keeper.steering_delta_clamped", "0.0000");
        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        self.publish_stage_steering_diag(ctx, stage, cause, false);

        Some(ControlRequest {
            steering: None,
            throttle: Some(0.0),
            brake: Some(brake),
            priority: PRIORITY_LEVEL4,
        })
    }

    /// Apply the rate-limiter and return the clamped steering value.
    fn rate_limit(&mut self, target: f64, ctx: &PluginContext) -> f64 {
        let delta = target - self.previous_steering_out;
        let clamped = delta.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let output = self.previous_steering_out + clamped;
        let was_limited = (clamped - delta).abs() > 1e-9;
        ctx.blackboard
            .set("lane_keeper.steering_rate_limited", was_limited.to_string());
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta - clamped),
        );
        self.previous_steering_out = output;
        output
    }
}

// â”€â”€ Route-following implementation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

impl LaneKeeperPlugin {
    fn load_waypoints_from_blackboard(&mut self, ctx: &PluginContext) {
        if let Some(json) = ctx.blackboard.get("router.waypoints") {
            if let Ok(pts) = serde_json::from_str::<Vec<[f64; 2]>>(&json) {
                let new_hash = hash_str(&json);
                self.waypoints = smooth_catmull_rom(&pts, self.subdivisions);
                self.progress_idx = 0;
                self.last_waypoints_hash = new_hash;
                if !self.waypoints.is_empty() {
                    tracing::info!(
                        "[lane-keeper] first 3 spline pts: [{:.1},{:.1}] [{:.1},{:.1}] [{:.1},{:.1}]",
                        self.waypoints[0][0], self.waypoints[0][1],
                        self.waypoints.get(1).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(1).map(|p| p[1]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[1]).unwrap_or(0.0),
                    );
                }
                tracing::info!(
                    "[lane-keeper] loaded {} waypoints (smoothed, hash={:x})",
                    self.waypoints.len(),
                    new_hash
                );
                self.previous_steering_out = 0.0;
                self.pid.reset();
            }
        }
    }

    /// R3 steering-stage diagnose (read-only): why a tick did or did not emit
    /// `ControlRequest.steering`.
    fn publish_stage_steering_diag(
        &self,
        ctx: &PluginContext,
        stage: &str,
        block_reason: &str,
        steering_emitted: bool,
    ) {
        ctx.blackboard.set("lane_keeper.stage", stage);
        ctx.blackboard
            .set("lane_keeper.stage_block_reason", block_reason);
        ctx.blackboard.set(
            "lane_keeper.steering_request_emitted",
            steering_emitted.to_string(),
        );
    }

    /// R3 Task-1: per-tick Route-/Snap-Diagnose (read-only, kein Verhaltens-Fix).
    fn publish_r3_lane_diag_keys(&self, ctx: &PluginContext) {
        let route_json = ctx.blackboard.get("router.route_node_ids");
        let present = route_json.is_some();
        let len = route_json
            .as_ref()
            .and_then(|j| serde_json::from_str::<Vec<u64>>(j).ok())
            .map(|v| v.len())
            .unwrap_or(0);
        ctx.blackboard
            .set("lane_keeper.route_node_ids_present", present.to_string());
        ctx.blackboard
            .set("lane_keeper.route_node_ids_len", len.to_string());

        let rejected = ctx
            .blackboard
            .get("router.last_snap_rejected_by_heading")
            .and_then(|s| s.parse::<u32>().ok())
            .unwrap_or(0);
        ctx.blackboard
            .set("lane_keeper.rejected_by_heading", rejected.to_string());
    }

    fn publish_lane_match_sentinels(&self, ctx: &PluginContext) {
        ctx.blackboard.set("lane_keeper.chosen_segment_index", "-1");
        ctx.blackboard.set("lane_keeper.chosen_edge_id", "none");
        ctx.blackboard.set("lane_keeper.heading_error_deg", "-1.0");
        ctx.blackboard.set("lane_keeper.signed_cte_m", "0.000");
    }

    fn try_spline_heading_error(
        &mut self,
        tx: f64,
        tz: f64,
        heading: f64,
        speed_ms: f64,
        ctx: &PluginContext,
    ) -> Option<f64> {
        // Phase 2f-B-Diagnose (read-only): Funktions-Eintritt zÃ¤hlen, GANZ AM ANFANG,
        // vor JEDER Bedingung/early-return. WÃ¤chst dieser ZÃ¤hler nicht mit den Ticks,
        // wird try_spline_heading_error (und damit der re-anchor-Scan) gar nicht
        // betreten (H1: hinter dem dist_gate / im Spline-Zweig Ã¼bersprungen).
        self.reanchor_called_count += 1;
        ctx.blackboard.set(
            "lane_keeper.reanchor_called_count",
            self.reanchor_called_count.to_string(),
        );
        // H2-Catmull-Offset-Fix: Frische-Reset. Wird nur dann wieder Some, wenn die
        // nearest-Query diesen Tick erreicht wird (siehe `self.last_nearest_seg = Some(cur_seg)`).
        // Bricht try_spline VOR der Query ab (index_none/route_miss), bleibt es None â†’
        // der Catmull-Fallback nutzt den 1.875-m-Default statt eines stale Segments.
        self.last_nearest_seg = None;

        self.publish_r3_lane_diag_keys(ctx);
        self.publish_lane_match_sentinels(ctx);

        // Phase 2c/2d-Diagnose (read-only): jeder Dispatch-Pfad schreibt GENAU EINEN
        // `lane_keeper.fallback_reason` (6-Wert-Vertrag) plus eine feinere
        // `lane_keeper.fallback_detail`. Reihenfolge = Dispatch-Flow â†’ die ERSTE
        // greifende Bedingung gewinnt. KEINE VerhaltensÃ¤nderung: nur Keys + Logs.
        let Some(index) = self.index.as_ref() else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "index_none");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "index_none");
            return None;
        };
        let Some(rg) = self.router_graph.as_ref() else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "index_none");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "graph_none");
            return None;
        };

        // Route cachen (hash-reload); progress reset bei Routenwechsel.
        let Some(route_json) = ctx.blackboard.get("router.route_node_ids") else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "no_route_node_ids");
            return None;
        };
        let route_hash = hash_str(&route_json);
        let route_changed = route_hash != self.cached_route_hash;
        if route_changed {
            let Ok(nodes) = serde_json::from_str::<Vec<u64>>(&route_json) else {
                ctx.blackboard
                    .set("lane_keeper.fallback_reason", "route_miss");
                ctx.blackboard
                    .set("lane_keeper.fallback_detail", "route_parse_err");
                return None;
            };
            self.cached_route_node_ids = nodes;
            self.cached_route_hash = route_hash;
            self.node_progress_idx = 0;
            self.route_miss_sample_logged = false; // neue Route â†’ Sample erneut erlauben
        }
        // Phase 2h-Befund3-Fix: On-Route-Segmentmenge cachen. Rebuild genau einmal
        // pro Routenwechsel (Hash-Tracking, kein Per-Tick-Rebuild â€” auch nicht bei
        // legitim leerem Set). Jeder konsekutive Hop (route[j],route[j+1]) â†’
        // segment_idx. Wenn seg_by_from_to noch leer ist (Index nicht geladen),
        // bleibt der seg-Hash stale â†’ Rebuild greift, sobald die Quelle bereit ist.
        //
        // Fix C: ZusÃ¤tzlich zu den Road-Segmenten auch die On-Route-NavCurves
        // aufnehmen â€” die NavCurve(s), deren (from_uid,to_uid) GENAU auf einem
        // konsekutiven Route-Paar (route[j],route[j+1]) liegen. Off-Route-NavCurves
        // (anderes Knotenpaar, z.B. der Abbieger) bleiben ausgeschlossen. Auf reinen
        // Road-Strecken gibt es fÃ¼r die Road-Paare keinen NavCurve-Eintrag â†’ das Set
        // bleibt dort road-only, die Geraden-Selektion Ã¤ndert sich NICHT.
        if self.cached_route_seg_hash != self.cached_route_hash {
            let mut set = std::collections::HashSet::new();
            let mut navcurve_count = 0usize;
            for w in self.cached_route_node_ids.windows(2) {
                if let Some(&seg) = self.seg_by_from_to.get(&(w[0], w[1])) {
                    set.insert(seg);
                }
                if let Some(navs) = self.navcurve_by_from_to.get(&(w[0], w[1])) {
                    for &nseg in navs {
                        if set.insert(nseg) {
                            navcurve_count += 1;
                        }
                    }
                }
            }
            self.cached_route_seg_set = set;
            self.cached_route_navcurve_count = navcurve_count;
            // Latch erst, wenn mindestens eine Quelle bereit ist (Index geladen).
            if !self.seg_by_from_to.is_empty() || !self.navcurve_by_from_to.is_empty() {
                self.cached_route_seg_hash = self.cached_route_hash;
            }
        }
        let route = &self.cached_route_node_ids;
        if route.len() < 2 {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "route_too_short");
            return None;
        }

        // Phase 2g (Variante B): Globale R-tree-nearest-Query (wie Lane-Follower) statt
        // route-only VorwÃ¤rts-Fenster-Scan. Der route-only-Scan misst die Distanz zur
        // VorgÃ¤nger-/Snap-Kante falsch (Truck sitzt auf to_uid == route[0], KEIN
        // VorwÃ¤rts-Hop â†’ 69m statt 3.4m â†’ dist_gate feuert permanent). Die globale
        // nearest-Query findet die geometrisch nÃ¤chste, heading-kompatible Kante (~3.4m,
        // inkl. Gegenfahrbahn-Schutz). AnschlieÃŸend Route-Relevanz-PrÃ¼fung gegen die
        // Route (kein Abspringen auf ParallelstraÃŸen).
        let truck_heading_deg = ((-heading) * 360.0).rem_euclid(360.0) as f32; // heading = t.heading [0..1]
        let query = Vec3::new(tx as f32, 0.0, tz as f32); // Y=0 ok: R-tree ist XZ-only

        // Phase 2h-Befund3-Fix: ROUTE-AWARE nearest. Bevorzuge den geometrisch
        // nÃ¤chsten ON-ROUTE-Treffer (innerhalb dist_gate). Nur wenn kein
        // Route-Segment in Reichweite ist, fÃ¤llt der Code auf die alte globale
        // heading-gefilterte Query zurÃ¼ck. Damit schnappt der Spline an Kreuzungen
        // nicht mehr auf den geometrisch nÃ¤heren Abbieger (off-route â†’ None â†’
        // Catmull-Querziehen), sondern bleibt auf dem Route-Ast.
        let global_hit = index.nearest_with_heading_filter(query, truck_heading_deg, 8);
        // Info des globalen Treffers VOR dem evtl. Move sichern (fÃ¼r Diag).
        let global_seg_info = global_hit.as_ref().map(|gh| {
            let f = index.segments[gh.segment_idx].from_uid;
            let t = index.segments[gh.segment_idx].to_uid;
            (gh.segment_idx, f, t)
        });
        let route_seg_set_len = self.cached_route_seg_set.len();
        // Phase 2h-Befund3-Diag (temporÃ¤r): rohen On-Route-Treffer VOR dem Gate
        // festhalten. So zeigen die Diag-Keys eindeutig, WARUM der route-aware-Query
        // ggf. leer bleibt: (a) Set leer / kein On-Route-Kandidat unter den N
        // nÃ¤chsten â†’ query_hits=0; (b) dist > Gate; (c) Heading-Gate verwirft.
        let raw_route_hit = if self.cached_route_seg_set.is_empty() {
            None // keine Route â†’ altes Verhalten (globaler nearest)
        } else {
            let route_seg_set = &self.cached_route_seg_set;
            index.nearest_with_projection_filtered(query, ROUTE_NEAREST_CANDIDATES, |idx, _| {
                route_seg_set.contains(&idx)
            })
        };
        // Junction-NavCurve-Force (ETS2LA-Ansatz): IMMER topologisch nach NavCurves
        // fuer die naechsten Hops im Scan-Fenster suchen — kein Proximity-Wettbewerb.
        // NavCurve gewinnt gegenueber Road-Segment wenn dist_nav <= dist_road + MARGIN
        // (Topologie schlaegt Proximity, analog ETS2LA accepted_lanes). Greift auch
        // wenn raw_route_hit schon ein Road-Segment gefunden hat (das war der Fehler:
        // das Road-Segment 51m entfernt gewann, NavCurve wurde ignoriert).
        // Scan die GESAMTE Route nach NavCurves — nicht nur cursor±window.
        // Der cursor-basierte Scan versagt wenn der Cursor bei Hop 3 steht aber
        // die Junction-NavCurve bei Hop 50 liegt (ausserhalb 0..32-Fenster).
        // Der Distance-Check (NAVCURVE_PRIORITY_MARGIN_M) verhindert Fruehzugriff
        // auf weit entfernte Junctions. Guard t>0.95: keine bereits passierten
        // NavCurves (Projektion am Ende = Truck hat sie ueberholt).
        let junction_navcurve_best = {
            let mut best: Option<NearestHit> = None;
            let mut diag_candidates = 0u32;   // NavCurves via .get() gefunden
            let mut diag_proj_ok = 0u32;      // project_on_segment lieferte Some
            let mut diag_t_filtered = 0u32;   // durch t>0.95 gefiltert
            let mut diag_best_dist = f32::MAX; // beste Distanz vor t-Filter
            for k in 0..route.len().saturating_sub(1) {
                let Some(navs) = self.navcurve_by_from_to.get(&(route[k], route[k + 1])) else {
                    continue;
                };
                diag_candidates += navs.len() as u32;
                for &nav_idx in navs {
                    let Some((t, dist_m)) = index.project_on_segment(nav_idx, query) else {
                        continue;
                    };
                    diag_proj_ok += 1;
                    if dist_m < diag_best_dist {
                        diag_best_dist = dist_m;
                    }
                    // t > 0.95: Truck hat diese NavCurve bereits passiert → ueberspringen.
                    if t > 0.95 {
                        diag_t_filtered += 1;
                        continue;
                    }
                    // Heading-Filter: NavCurve ablehnen wenn > 60° vom Truck-Heading abweicht.
                    // Verhindert, dass ein geometrisch naehes aber perpendiculares NavCurve
                    // (z.B. Querspange, falscher Abfahrtsast) als "bestes" gewaehlt wird.
                    let seg = &index.segments[nav_idx];
                    let tan = evaluate_tangent(seg, t);
                    let heading_deg =
                        f32::atan2(tan.x, -tan.z).to_degrees().rem_euclid(360.0);
                    let mut hd = (heading_deg - truck_heading_deg).rem_euclid(360.0);
                    if hd > 180.0 { hd -= 360.0; }
                    if hd.abs() > 60.0 {
                        continue;
                    }
                    if best.as_ref().map_or(true, |b| dist_m < b.dist_m) {
                        let point_on_curve = evaluate(seg, t);
                        best = Some(NearestHit {
                            segment_idx: nav_idx,
                            t,
                            point_on_curve,
                            dist_m,
                            heading_deg,
                            heading_filter_applied: true,
                        });
                    }
                }
            }
            self.dbg_force_candidates = diag_candidates;
            self.dbg_force_proj_ok    = diag_proj_ok;
            self.dbg_force_t_filtered = diag_t_filtered;
            self.dbg_force_best_dist  = if diag_best_dist < f32::MAX { diag_best_dist } else { -1.0 };
            best
        };
        let (raw_route_hit, junction_navcurve_forced) =
            match (raw_route_hit, junction_navcurve_best) {
                // NavCurve vorhanden UND nah genug — Topologie gewinnt.
                (Some(road), Some(nav))
                    if nav.dist_m <= road.dist_m + NAVCURVE_PRIORITY_MARGIN_M =>
                {
                    (Some(nav), true)
                }
                // Kein Road-Treffer, aber NavCurve gefunden.
                (None, Some(nav)) => (Some(nav), true),
                // Road-Treffer klar besser oder keine NavCurve — unveraendert.
                (road, _) => (road, false),
            };
        let route_query_hits = u8::from(raw_route_hit.is_some());
        let (route_best_dist, route_best_hop, route_heading_diff) = match &raw_route_hit {
            Some(h) => {
                let f = index.segments[h.segment_idx].from_uid;
                let t = index.segments[h.segment_idx].to_uid;
                let mut d = (h.heading_deg - truck_heading_deg).rem_euclid(360.0);
                if d > 180.0 {
                    d -= 360.0;
                }
                (h.dist_m, format!("{f}->{t}"), d.abs())
            }
            None => (-1.0f32, "none".to_string(), -1.0f32),
        };
        // Dual-CW-Guard: prÃ¼fen BEVOR route_hit-Filter, da raw_route_hit danach consumed wird.
        // Wenn global_hit (physisch nÃ¤chstes Segment) deutlich nÃ¤her als der Route-Treffer,
        // fÃ¤hrt der Truck auf einer Parallelfahrbahn â†’ Route-Treffer ablehnen.
        // Ausnahme: NavCurve-Segmente (Index >= road_seg_count) sind NIEMALS Parallel-
        // fahrbahnen — sie sind Junction-Pfade. Der Guard darf sie nicht verwerfen, auch
        // wenn die geometrisch nÃ¤here Geradeaus-Road im selben Abschnitt liegt.
        let dual_cw_guard = match (&raw_route_hit, &global_hit) {
            (Some(rh), Some(gh)) => {
                let route_is_navcurve = rh.segment_idx >= self.road_seg_count;
                !route_is_navcurve
                    && gh.dist_m < PHYSICAL_CLOSE_DIST_M
                    && gh.dist_m < rh.dist_m * DUAL_CW_REJECT_RATIO
            }
            _ => false,
        };

        // Gate anwenden (dist â‰¤ MAX_HOP_PROJECTION_DIST_M UND heading â‰¤90Â°) â€” Logik
        // GeÃ¤ndert: Heading-Gate 60Â° â†’ 90Â° fÃ¼r On-Route-Segmente. Der globale
        // Query (global_hit) nutzt weiterhin 60Â° (cos0.5). FÃ¼r Route-Segmente
        // kÃ¶nnen wir vertrauen, dass es die richtige Kante ist â€” Truck nÃ¤hert sich
        // einer Kurve oder Ausfahrt von einem Winkel an (64Â° live beobachtet).
        // GegenlÃ¤ufige Segmente (>90Â°) bleiben weiterhin gesperrt.
        let route_hit = raw_route_hit.filter(|h| {
            if dual_cw_guard {
                return false; // Parallel-Fahrbahn â†’ kein Route-Lock
            }
            // NavCurves (junction_navcurve_forced) umgehen das Dist-Gate:
            // Ihr Abstand ist per NAVCURVE_PRIORITY_MARGIN_M bereits selektiert;
            // das harte 65m-Gate wuerde sie sonst bei 52m+ trotzdem verwerfen.
            let is_navcurve = h.segment_idx >= self.road_seg_count;
            if !is_navcurve && h.dist_m > MAX_HOP_PROJECTION_DIST_M {
                return false;
            }
            let mut d = (h.heading_deg - truck_heading_deg).rem_euclid(360.0);
            if d > 180.0 {
                d -= 360.0;
            }
            d.abs() <= 90.0
        });
        let route_gate_rejected = u8::from(route_query_hits == 1 && route_hit.is_none());
        // Diag-Keys IMMER setzen (vor jedem Branch/early-return), damit das
        // t=16-23-Fenster vollstÃ¤ndig belegt ist.
        ctx.blackboard.set(
            "lane_keeper.nearest_route_seg_set_size",
            route_seg_set_len.to_string(),
        );
        // Fix C: wie viele On-Route-NavCurves im aktuellen route_seg_set stecken.
        ctx.blackboard.set(
            "lane_keeper.total_route_navcurve_count",
            self.cached_route_navcurve_count.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.nearest_route_query_hits",
            route_query_hits.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.nearest_route_gate_rejected",
            route_gate_rejected.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.dual_cw_guard_active",
            u8::from(dual_cw_guard).to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.junction_navcurve_forced",
            u8::from(junction_navcurve_forced).to_string(),
        );
        // Diagnose NavCurve-Force-Scan: was hat der Scan intern gesehen?
        ctx.blackboard.set(
            "lk.force_candidates",
            self.dbg_force_candidates.to_string(),
        );
        ctx.blackboard.set(
            "lk.force_proj_ok",
            self.dbg_force_proj_ok.to_string(),
        );
        ctx.blackboard.set(
            "lk.force_t_filtered",
            self.dbg_force_t_filtered.to_string(),
        );
        ctx.blackboard.set(
            "lk.force_best_dist_m",
            format!("{:.2}", self.dbg_force_best_dist),
        );
        ctx.blackboard.set(
            "lane_keeper.nearest_route_best_dist_m",
            format!("{route_best_dist:.2}"),
        );
        ctx.blackboard
            .set("lane_keeper.nearest_route_best_hop", route_best_hop);
        ctx.blackboard.set(
            "lane_keeper.nearest_route_heading_diff_deg",
            format!("{route_heading_diff:.1}"),
        );
        let (hit, nearest_route_filtered) = if let Some(rh) = route_hit {
            (rh, true)
        } else if let Some(gh) = global_hit {
            (gh, false)
        } else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "no_nearest");
            ctx.blackboard
                .set("lane_keeper.nearest_route_filtered", "false");
            ctx.blackboard.set(
                "lane_keeper.nearest_route_seg_set_size",
                route_seg_set_len.to_string(),
            );
            return None;
        };

        // Task 3 Diag: belegen, dass der Route-Ast gewÃ¤hlt wurde, und welcher
        // Off-route-Kandidat (globaler nearest) dabei verworfen wurde.
        ctx.blackboard.set(
            "lane_keeper.nearest_route_filtered",
            nearest_route_filtered.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.nearest_route_seg_set_size",
            route_seg_set_len.to_string(),
        );
        match global_seg_info {
            Some((gseg, gf, gt)) if nearest_route_filtered && gseg != hit.segment_idx => {
                ctx.blackboard.set(
                    "lane_keeper.nearest_discarded_offroute_seg",
                    gseg.to_string(),
                );
                ctx.blackboard.set(
                    "lane_keeper.nearest_discarded_offroute_hop",
                    format!("{gf}->{gt}"),
                );
            }
            _ => {
                ctx.blackboard
                    .set("lane_keeper.nearest_discarded_offroute_seg", "none");
                ctx.blackboard
                    .set("lane_keeper.nearest_discarded_offroute_hop", "none");
            }
        }

        let cur_seg = hit.segment_idx;
        let t_cur = hit.t;
        // H2-Catmull-Offset-Fix: frisches nearest-Segment dieses Ticks festhalten, damit
        // der Catmull-Fallback (falls try_spline spÃ¤ter None liefert: off_route/dist_gate/
        // latch/degenerate) den per-Segment-lane_offset_right_m nutzen kann.
        self.last_nearest_seg = Some(cur_seg);
        let seg_f = index.segments[cur_seg].from_uid;
        let seg_t = index.segments[cur_seg].to_uid;
        // Phase 2h-Diag3 (read-only, Task 2): which segment the global nearest-query
        // picked when the source flips catmullâ†’spline. A jump here (vs the prior
        // tick) is the candidate cause of the herr spike.
        ctx.blackboard
            .set("lane_keeper.nearest_segment_id", cur_seg.to_string());
        // Fix C: ist das gewÃ¤hlte nearest-Segment eine NavCurve (Index >= road_n)?
        // An der Junction wird hier kurz "true" erwartet (On-Route-NavCurve gewÃ¤hlt).
        ctx.blackboard.set(
            "lane_keeper.chosen_segment_is_navcurve",
            (cur_seg >= self.road_seg_count).to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.nearest_segment_t", format!("{t_cur:.3}"));
        ctx.blackboard
            .set("lane_keeper.nearest_seg_hop", format!("{seg_f}->{seg_t}"));
        ctx.blackboard.set(
            "lane_keeper.nearest_heading_filter_applied",
            hit.heading_filter_applied.to_string(),
        );
        let mut seg_heading_diff = (hit.heading_deg - truck_heading_deg).rem_euclid(360.0);
        if seg_heading_diff > 180.0 {
            seg_heading_diff -= 360.0;
        }
        ctx.blackboard
            .set("lane_keeper.chosen_segment_index", cur_seg.to_string());
        ctx.blackboard
            .set("lane_keeper.chosen_edge_id", format!("{seg_f}->{seg_t}"));
        ctx.blackboard.set(
            "lane_keeper.heading_error_deg",
            format!("{:.1}", seg_heading_diff.abs()),
        );
        // ETS2LA-Fix: lokalen Heading-Fehler (Truck vs Tangente des nearest-Segments)
        // als Safety-Gate-Referenz festhalten. Spiked NICHT in Kurven, anders als der
        // ferne Lookahead-`err` weiter unten. Der heading_mismatch-Gate prueft gegen
        // DIESEN Wert, damit der Truck nur bei echter Fehlausrichtung bremst statt nur
        // weil der 15m-Lookahead um eine Kurve greift. Spline-Some-returns tragen ihn;
        // None-returns werden vom Catmull-Fallback ueberschrieben.
        self.heading_mismatch_herr_rad = (seg_heading_diff as f64).to_radians();

        // Route-Relevanz-PrÃ¼fung (Route-Constraint), windowed:
        //   Erst-Anchor (route_changed=true): globaler Scan 0..route.len()
        //   Laufend (route_changed=false):    Fenster [progressâˆ’BACKWARD, progress+FORWARD)
        //
        //   Pass 1 â€“ on-route:   âˆƒ j: route[j]==F && route[j+1]==T â†’ progress=j, end=j+1
        //   Pass 2 â€“ feeds-into: âˆƒ k: route[k]==T                  â†’ progress=k, end=k
        //   off-route:           kein Treffer im Fenster â†’ Catmull-Fallback
        //
        //   Pass-Trennung garantiert: on_route schlÃ¤gt feeds_into unabhÃ¤ngig vom Index.
        //   Monotonie: scan_lo = max(0, progressâˆ’BACKWARD) verhindert Regression nach k=0.
        let (scan_lo, scan_hi) = if route_changed {
            (0, route.len())
        } else {
            let lo = self.node_progress_idx.saturating_sub(REANCHOR_BACKWARD_WINDOW);
            let hi = (self.node_progress_idx + REANCHOR_FORWARD_WINDOW).min(route.len());
            (lo, hi)
        };

        // Pass 1: on-route (exakter direktionaler (from,to)-Match â€” hÃ¶chste PrioritÃ¤t).
        let mut on_route_idx: Option<usize> = None;
        for k in scan_lo..scan_hi {
            if k + 1 < route.len() && route[k] == seg_f && route[k + 1] == seg_t {
                on_route_idx = Some(k);
                break;
            }
        }

        // Pass 2: feeds-into (nur wenn kein on_route im Fenster).
        let feeds_into_idx: Option<usize> = if on_route_idx.is_none() {
            route[scan_lo..scan_hi]
                .iter()
                .position(|&n| n == seg_t)
                .map(|i| scan_lo + i)
        } else {
            None
        };

        let (node_progress_idx, end_route_idx) = if let Some(j) = on_route_idx {
            // on-route: immer sicher, unabhÃ¤ngig von Heading-Flag (W1) und Forward-Hop-PrÃ¼fung (W2).
            (j, j + 1)
        } else if let Some(k) = feeds_into_idx {
            // feeds-into (VorgÃ¤nger-Kante): nur akzeptieren wenn
            //   W1: der nearest-Treffer den Heading-Filter bestanden hat
            //       (sonst evtl. flach einmÃ¼ndende QuerstraÃŸe), UND
            //   W2: der erste VorwÃ¤rts-Walk-Hop route[k]â†’route[k+1] existiert
            //       (sonst RÃ¼ckwÃ¤rts-/U-turn-Route ohne Forward-Segment).
            let accept_feeds_into = hit.heading_filter_applied
                && k + 1 < route.len()
                && (self.seg_by_from_to.contains_key(&(route[k], route[k + 1]))
                    || self.navcurve_by_from_to.contains_key(&(route[k], route[k + 1])));
            if !accept_feeds_into {
                let detail = if !hit.heading_filter_applied {
                    "feeds_into_no_heading"
                } else {
                    "feeds_into_no_forward_hop"
                };
                ctx.blackboard
                    .set("lane_keeper.fallback_reason", "off_route");
                ctx.blackboard.set("lane_keeper.fallback_detail", detail);
                return None;
            }
            (k, k)
        } else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "off_route");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "nearest_not_on_route");
            return None;
        };
        self.node_progress_idx = node_progress_idx;

        // Phase 2f-B-Diagnose (read-only): Re-anchor-Instrumentierung weiter befÃ¼llen.
        // Gate-Distanz ist 2D (XZ) zum projizierten Punkt auf der nearest-Kante.
        let pc = evaluate(&index.segments[cur_seg], t_cur);
        let dist = (((tx - pc.x as f64).powi(2) + (tz - pc.z as f64).powi(2)).sqrt()) as f32;
        // Phase 2g-Diag2 (read-only): IST-Lateralversatz des Trucks gegen die Spur-MITTE,
        // gemessen am Truck-Projektionspunkt `pc` auf cur_seg. +rechts / -links (Right-Normal
        // n=(-tan.z,tan.x)). Entscheidende Metrik: fÃ¤hrt der Truck mittig (truck_latâ‰ˆ0) trotz
        // gelogtem Offset 5.625 â†’ die Kette Offsetâ†’Position greift nicht.
        let tan_cur = evaluate_tangent(&index.segments[cur_seg], t_cur);
        let tcl = (tan_cur.x * tan_cur.x + tan_cur.z * tan_cur.z).sqrt();
        let truck_lat_vs_centerline = if tcl > 1e-6 {
            let rn_cx = (-tan_cur.z / tcl) as f64;
            let rn_cz = (tan_cur.x / tcl) as f64;
            (tx - pc.x as f64) * rn_cx + (tz - pc.z as f64) * rn_cz
        } else {
            0.0
        };
        ctx.blackboard.set(
            "lane_keeper.reanchor_scan_window",
            format!("{scan_lo}..{scan_hi}"),
        );
        ctx.blackboard.set(
            "lane_keeper.reanchor_best_idx",
            node_progress_idx.to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.reanchor_best_dist_m", format!("{dist:.2}"));
        self.reanchor_reached_write += 1;
        ctx.blackboard.set(
            "lane_keeper.reanchor_reached_write",
            self.reanchor_reached_write.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.reanchor_idx_written",
            node_progress_idx.to_string(),
        );

        // seg_idx0 = cur_seg; i0/a0/b0/t_truck auf die nearest-Kante umgestellt.
        let i0 = node_progress_idx;
        ctx.blackboard
            .set("lane_keeper.seg_idx0_source_value", i0.to_string());
        let seg_idx0 = cur_seg;
        let (a0, b0) = (seg_f, seg_t);
        let t_truck = t_cur;
        ctx.blackboard
            .set("lane_keeper.hop_projection_dist_m", format!("{dist:.2}"));

        // â”€â”€ Phase 2f-Diagnose (read-only): warum sitzt dist strukturell >40m? â”€â”€
        // WICHTIG: `dist` (= hop_projection_dist_m) ist die Distanz Truckâ†’NÃ„CHSTER
        // PUNKT auf seg_idx0 (project_on_segment), NICHT ein Vorausschau-Abstand.
        // `look_ahead` (Soll-Voraus) wird erst NACH dem Gate berechnet und geht NICHT
        // ins Gate ein. truck_to_segment_dist_m == projected_point_dist_m == dist.
        //   â†’ H1 (Gate zu eng auf legitimem Lookahead) ist damit strukturell NICHT
        //     der Mechanismus; das Gate misst Truckâ†”Segment.
        // Diskriminator H2-Varianten:
        //   projection_t â‰ˆ 1.0 (oder 0.0) + dist groÃŸ  â†’ Truck am Segment-ENDE
        //     vorbei (longitudinaler Overshoot): node_progress hÃ¤ngt â†’ seg_idx0 ist
        //     ein bereits passierter Hop. Fix = node-advance, nicht Gate-Anheben.
        //   projection_t mittig (0.3..0.7) + dist groÃŸ  â†’ echter LATERALER Miss
        //     (falsches/zu weit entferntes Segment). Fix = Segment-Auswahl/Projektion.
        //   dist_to_next_node_m strukturell > WAYPOINT_REACH_M (5m)  â†’ der
        //     node-advance feuert nie (Truck fÃ¤hrt rechte Spur ~5.6m neben den
        //     Median-Nodes) â†’ progress hÃ¤ngt. Das ist die wahrscheinlichste Wurzel.
        let look_ahead_target = (BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR) as f32;
        let seg_len = index.segments[seg_idx0].length_m;
        let seg_is_prefab = index
            .metadata
            .get(seg_idx0)
            .and_then(|m| m.as_ref())
            .map(|m| m.is_prefab)
            .unwrap_or(false);
        let dist_to_next_node = match rg.positions.get(&b0) {
            Some(&(nx, nz)) => ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt(),
            None => -1.0,
        };
        ctx.blackboard.set(
            "lane_keeper.lookahead_target_dist_m",
            format!("{look_ahead_target:.2}"),
        );
        ctx.blackboard
            .set("lane_keeper.truck_to_segment_dist_m", format!("{dist:.2}"));
        ctx.blackboard
            .set("lane_keeper.projected_point_dist_m", format!("{dist:.2}"));
        ctx.blackboard
            .set("lane_keeper.projection_t", format!("{t_truck:.3}"));
        ctx.blackboard
            .set("lane_keeper.current_seg_length_m", format!("{seg_len:.2}"));
        ctx.blackboard.set(
            "lane_keeper.current_seg_is_prefab",
            seg_is_prefab.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.dist_to_next_node_m",
            format!("{dist_to_next_node:.2}"),
        );
        ctx.blackboard
            .set("lane_keeper.current_hop", format!("{a0}->{b0}"));
        ctx.blackboard.set(
            "lane_keeper.node_progress_idx",
            self.node_progress_idx.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.dist_gate_threshold_m",
            format!("{MAX_HOP_PROJECTION_DIST_M:.2}"),
        );

        // â”€â”€ Phase 2g-Diagnose (read-only): WO kommen die ~58m her? â”€â”€
        // Vergleicht fÃ¼r den AKTUELLEN Hop (a0->b0) drei Koordinaten-Quellen am selben Tick:
        //   1. Truck-Weltposition (tx,tz)
        //   2. Router-Graph-Node-Positionen von a0 (= route[i0]) und b0
        //   3. SplineIndex-Segment-Endpunkte p0/p1 des Segments seg_by_from_to[(a0,b0)]
        // Beweisrichtung:
        //   node(a0) â‰ˆ seg.p0 UND node(b0) â‰ˆ seg.p1  â†’ identische Geometrie â†’ NICHT H-A/H-B
        //     (beide stammen aus map_graph.nodes[uid]; build_router_graph & build_splines_ex teilen die Quelle).
        //   dist(truck, node(a0)) â‰ˆ truck_to_segment_dist_m (~60m) bei projection_tâ‰ˆ0
        //     â†’ route[0] (= direction-aligned Snap-Endpunkt) ist selbst weit weg, nicht die Geometrie â†’ H-C.
        let seg0 = &index.segments[seg_idx0];
        let (sp0x, sp0z) = (seg0.p0.x as f64, seg0.p0.z as f64);
        let (sp1x, sp1z) = (seg0.p1.x as f64, seg0.p1.z as f64);
        let (na0x, na0z, na0_present) = match rg.positions.get(&a0) {
            Some(&(x, z)) => (x, z, true),
            None => (0.0, 0.0, false),
        };
        let (nb0x, nb0z, nb0_present) = match rg.positions.get(&b0) {
            Some(&(x, z)) => (x, z, true),
            None => (0.0, 0.0, false),
        };
        let dxz =
            |ax: f64, az: f64, bx: f64, bz: f64| ((ax - bx).powi(2) + (az - bz).powi(2)).sqrt();
        let node_a0_vs_segp0 = if na0_present {
            dxz(na0x, na0z, sp0x, sp0z)
        } else {
            -1.0
        };
        let node_b0_vs_segp1 = if nb0_present {
            dxz(nb0x, nb0z, sp1x, sp1z)
        } else {
            -1.0
        };
        let truck_to_node_a0 = if na0_present {
            dxz(tx, tz, na0x, na0z)
        } else {
            -1.0
        };
        let truck_to_segp0 = dxz(tx, tz, sp0x, sp0z);
        ctx.blackboard
            .set("lane_keeper.diag_truck_xz", format!("{tx:.2},{tz:.2}"));
        ctx.blackboard.set(
            "lane_keeper.diag_node_a0_xz",
            format!("{na0x:.2},{na0z:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_node_b0_xz",
            format!("{nb0x:.2},{nb0z:.2}"),
        );
        ctx.blackboard
            .set("lane_keeper.diag_seg_p0_xz", format!("{sp0x:.2},{sp0z:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_seg_p1_xz", format!("{sp1x:.2},{sp1z:.2}"));
        ctx.blackboard.set(
            "lane_keeper.diag_node_a0_vs_segp0_m",
            format!("{node_a0_vs_segp0:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_node_b0_vs_segp1_m",
            format!("{node_b0_vs_segp1:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_truck_to_node_a0_m",
            format!("{truck_to_node_a0:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_truck_to_segp0_m",
            format!("{truck_to_segp0:.2}"),
        );

        // Task 2: Offset Ã¼ber mehrere Hops â€” globaler Frame-Versatz (H-B) oder ein einzelnes
        // falsches Mapping (H-A)? Pro Hop: dist(node(route[j]), seg.p0). Konstant ~58m â†’ H-B;
        // nur dieser Hop â†’ H-A; alle ~0 â†’ Geometrie stimmt Ã¼berall â†’ NICHT H-A/H-B.
        let mut max_diff = 0.0f64;
        let mut sum_diff = 0.0f64;
        let mut n_diff = 0usize;
        let mut per_hop = String::new();
        let hop_end = route.len().saturating_sub(1).min(10);
        for j in 0..hop_end {
            if let Some(&seg) = self.seg_by_from_to.get(&(route[j], route[j + 1])) {
                if let (Some(&(nx, nz)), Some(s)) =
                    (rg.positions.get(&route[j]), index.segments.get(seg))
                {
                    let d = dxz(nx, nz, s.p0.x as f64, s.p0.z as f64);
                    max_diff = max_diff.max(d);
                    sum_diff += d;
                    n_diff += 1;
                    if per_hop.len() < 200 {
                        per_hop.push_str(&format!("h{j}={d:.1} "));
                    }
                }
            }
        }
        let mean_diff = if n_diff > 0 {
            sum_diff / n_diff as f64
        } else {
            -1.0
        };
        ctx.blackboard.set(
            "lane_keeper.diag_node_vs_seg_maxdiff_m",
            format!("{max_diff:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_node_vs_seg_meandiff_m",
            format!("{mean_diff:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.diag_node_vs_seg_per_hop",
            per_hop.trim().to_string(),
        );

        if dist > MAX_HOP_PROJECTION_DIST_M {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "dist_gate");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "dist_gate");
            return None; // Truck nicht wirklich auf diesem Hop â†’ Catmull-Fallback
        }

        let look_ahead = (BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR) as f32;

        // Phase 2h-Wurzelfix: Knick-Schwelle aus Blackboard (justierbar), Diag-Keys initialisieren.
        let kink_threshold_deg = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kink_stop_deg")
            .map(|v| v as f32)
            .unwrap_or(KINK_STOP_DEG);
        ctx.blackboard.set(
            "lane_keeper.kink_threshold_deg",
            format!("{kink_threshold_deg:.1}"),
        );
        ctx.blackboard
            .set("lane_keeper.walk_stopped_at_kink", "false");

        // Multi-Hop Arc-Length-Walk entlang forward Road-Hops. Startet bei cur_seg/t_cur
        // und hÃ¤ngt ab end_route_idx VorwÃ¤rts-Route-Hops an. Vereinheitlicht on-route &
        // VorgÃ¤nger-Kante (feeds-into):
        //   on-route Fall (cur_seg = route[j]â†’route[j+1], end_route_idx=j+1):
        //     erster Advance = route[j+1]â†’route[j+2].
        //   VorgÃ¤nger Fall (cur_seg = Snap-Kante endet an route[k], end_route_idx=k):
        //     erster Advance = route[k]â†’route[k+1].
        let mut cur = cur_seg;
        let mut cur_lut = build_lut(&index.segments[cur]);
        let mut arc_at = arc_length(&cur_lut, t_cur);
        let mut remaining = look_ahead;
        let mut next_route_idx = end_route_idx; // route-Index des END-Knotens von cur
        let mut hop_count = 0usize;
        // Phase 2h-Wurzelfix: true wenn der Loop per Kink-Stop verlassen wurde â€” dann
        // den Stuck-ZÃ¤hler NICHT nullen (er muss tick-Ã¼bergreifend akkumulieren).
        let mut kink_stopped = false;
        // Phase 2h-Diag5 (read-only): warum misst der Naht-Check 0Â°, obwohl fin_seg auf ein
        // abknickendes Segment springt? ZÃ¤hle geprÃ¼fte Hops + degenerierte Tangenten (Guard-
        // Skips) + den grÃ¶ÃŸten gemessenen NAHT-Knick. Hypothese: der Knick liegt INNERHALB
        // des Landesegments, nicht an der Naht â†’ Naht-Check sieht ~0Â°.
        let mut walk_hops_checked = 0u32;
        let mut walk_kink_degenerate = 0u32;
        let mut walk_max_kink_deg = 0.0f32;
        let mut walk_max_kink_hop = String::new();

        let (final_seg, final_t) = loop {
            let arc_remaining = (cur_lut.total_length_m - arc_at).max(0.0);
            if remaining <= arc_remaining {
                let target_arc = arc_at + remaining;
                break (
                    cur,
                    t_at_arc_length(&cur_lut, &index.segments[cur], target_arc),
                );
            }
            remaining -= arc_remaining;
            hop_count += 1;
            if hop_count >= SPLINE_LOOKAHEAD_MAX_HOPS {
                break (cur, 1.0);
            }
            if next_route_idx + 1 >= route.len() {
                break (cur, 1.0);
            }
            let (na, nb) = (route[next_route_idx], route[next_route_idx + 1]);
            match self.seg_by_from_to.get(&(na, nb)) {
                Some(&ni) => {
                    // â”€â”€ Phase 2h-Wurzelfix: Knick-Erkennung am Hop-Ãœbergang curâ†’ni â”€â”€
                    let tan_cur_end = evaluate_tangent(&index.segments[cur], 1.0);
                    let tan_ni_start = evaluate_tangent(&index.segments[ni], 0.0);
                    let lc = (tan_cur_end.x * tan_cur_end.x + tan_cur_end.z * tan_cur_end.z).sqrt();
                    let ln =
                        (tan_ni_start.x * tan_ni_start.x + tan_ni_start.z * tan_ni_start.z).sqrt();
                    walk_hops_checked += 1;
                    if lc <= 1e-6 || ln <= 1e-6 {
                        walk_kink_degenerate += 1;
                    }
                    if lc > 1e-6 && ln > 1e-6 {
                        // atan2(x, -z): CW-Heading von Nord (konsistent mit spline.rs evaluate_heading_deg)
                        let h_cur = tan_cur_end.x.atan2(-tan_cur_end.z);
                        let h_ni = tan_ni_start.x.atan2(-tan_ni_start.z);
                        let mut d = h_ni - h_cur;
                        while d > std::f32::consts::PI {
                            d -= std::f32::consts::TAU;
                        }
                        while d < -std::f32::consts::PI {
                            d += std::f32::consts::TAU;
                        }
                        let kink_deg = d.abs().to_degrees();
                        // Diag: max gemessener Knick + Hop, auch wenn kein Stop.
                        if kink_deg > walk_max_kink_deg {
                            walk_max_kink_deg = kink_deg;
                            walk_max_kink_hop = format!("{na}->{nb}");
                        }
                        ctx.blackboard
                            .set("lane_keeper.walk_kink_deg", format!("{kink_deg:.4}"));
                        ctx.blackboard
                            .set("lane_keeper.walk_kink_hop", format!("{na}->{nb}"));
                        if kink_deg > kink_threshold_deg {
                            ctx.blackboard
                                .set("lane_keeper.walk_stopped_at_kink", "true");
                            kink_stopped = true;
                            // Dead-Lock-Schutz: zÃ¤hlt anhaltenden Stop auf DEMSELBEN Hop.
                            if self.kink_stuck_hop == (na, nb) {
                                self.kink_stuck_secs += ctx.dt_s.min(0.1);
                            } else {
                                self.kink_stuck_hop = (na, nb);
                                self.kink_stuck_secs = ctx.dt_s.min(0.1);
                            }
                            ctx.blackboard.set(
                                "lane_keeper.kink_stuck_secs",
                                format!("{:.2}", self.kink_stuck_secs),
                            );
                            if self.kink_stuck_secs > KINK_STUCK_FALLBACK_S {
                                // Truck kommt am Knick nicht vorbei â†’ Catmull Ã¼bernimmt (rundet die Ecke).
                                ctx.blackboard
                                    .set("lane_keeper.fallback_reason", "kink_stuck");
                                ctx.blackboard
                                    .set("lane_keeper.fallback_detail", "kink_stuck_catmull");
                                return None;
                            }
                            break (cur, 1.0); // Ziel am Segmentende VOR dem Knick
                        }
                    }
                    cur = ni;
                    cur_lut = build_lut(&index.segments[cur]);
                    arc_at = 0.0;
                    next_route_idx += 1;
                }
                None => break (cur, 1.0), // nÃ¤chster Hop reversed/prefab/miss â†’ konservativ clampen (2e-Grenze)
            }
        };

        // Phase 2h-Wurzelfix: NUR wenn der Walk OHNE Kink-Stop durchlief, den Stuck-ZÃ¤hler
        // nullen. Bei Kink-Stop muss er tick-Ã¼bergreifend akkumulieren (sonst greift der
        // KINK_STUCK_FALLBACK_S-Dead-Lock-Schutz nie). Sobald der Truck am Knick vorbei
        // ist, lÃ¤uft der nÃ¤chste Tick sauber bis hierher und der ZÃ¤hler wird zurÃ¼ckgesetzt.
        if !kink_stopped {
            self.kink_stuck_secs = 0.0;
            self.kink_stuck_hop = (0, 0);
        }

        // â”€â”€ Phase 2h-Diag5 (read-only): Naht-Check-Statistik + INTERNER Knick des
        // Landesegments. Diskriminator:
        //   walk_max_kink_deg â‰ˆ 0 (Naht stetig) ABER final_internal_kink_deg groÃŸ
        //     â†’ der Knick liegt INNERHALB von final_seg (Prefab/NavCurve dreht von
        //       t=0 bis final_t) â†’ Naht-only-Check ist blind dafÃ¼r â†’ Fix muss die
        //       Segment-INTERNE KrÃ¼mmung bis final_t prÃ¼fen, nicht nur die Naht.
        //   walk_kink_degenerate > 0 â†’ Tangenten degeneriert â†’ Guard Ã¼bersprang Stop.
        ctx.blackboard.set(
            "lane_keeper.walk_hops_checked",
            walk_hops_checked.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.walk_kink_degenerate",
            walk_kink_degenerate.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.walk_max_kink_deg",
            format!("{walk_max_kink_deg:.4}"),
        );
        ctx.blackboard
            .set("lane_keeper.walk_max_kink_hop", walk_max_kink_hop.clone());
        // â”€â”€ Alternative A: interner Knick ab TRUCK-Position statt ab t=0 â”€â”€
        // Befund: evaluate_tangent(final_seg, 0.0) == m0 (From-Node-Quaternion-Forward)
        // ist an Junction-Knoten ~131â€“136Â° mis-orientiert. Die "Beule" sitzt damit am
        // SEGMENT-ANFANG (t=0); der Truck erfÃ¤hrt sie ab seiner realen Position kaum.
        // Wir messen die interne KrÃ¼mmung daher Ã¼ber [t_start_intk, final_t]:
        //   final_seg == cur_seg â†’ Truck ist auf dem Landesegment â†’ t_start = t_cur
        //     (echte Projektion). Die m0-Anomalie bei t=0 fÃ¤llt aus der Latch-
        //     Entscheidung heraus; der Latch reagiert nur noch auf KrÃ¼mmung, die der
        //     Truck ab seiner Position tatsÃ¤chlich fÃ¤hrt.
        //   final_seg != cur_seg â†’ Truck fÃ¤hrt final_seg NOCH NICHT (Lookahead reicht
        //     Ã¼ber den aktuellen Hop hinaus). intK := 0, KEIN Latch (Task-1.b/Option 1):
        //     die m0-Anomalie eines Lookahead-End-Segments darf die Latch-Entscheidung
        //     nicht verfÃ¤lschen. Sobald der Truck auf final_seg projiziert (final_seg
        //     wird cur_seg), greift die Messung pro Tick neu.
        let f_tt = evaluate_tangent(&index.segments[final_seg], final_t);
        let f_lt = (f_tt.x * f_tt.x + f_tt.z * f_tt.z).sqrt();
        let intk_on_truck_seg = final_seg == cur_seg;
        let t_start_intk = if intk_on_truck_seg { t_cur } else { 0.0 };
        let final_internal_kink_deg = if !intk_on_truck_seg {
            0.0 // Truck fÃ¤hrt final_seg noch nicht â†’ keine intK-getriebene Latch-Entscheidung
        } else {
            let f_t0 = evaluate_tangent(&index.segments[final_seg], t_start_intk);
            let f_l0 = (f_t0.x * f_t0.x + f_t0.z * f_t0.z).sqrt();
            // LÃ¤ngen-Guards (>1e-6) + atan2((x,-z))-Normierung [0,180] beibehalten.
            // Leeres Intervall (t_start_intk == final_t, Truck am Segmentende): beide
            // Tangenten ~gleich â†’ dâ‰ˆ0 â†’ intKâ‰ˆ0 â†’ kein Latch (panik-sicher durch Guards).
            if f_l0 > 1e-6 && f_lt > 1e-6 {
                let h0 = f_t0.x.atan2(-f_t0.z);
                let ht = f_tt.x.atan2(-f_tt.z);
                let mut d = ht - h0;
                while d > std::f32::consts::PI {
                    d -= std::f32::consts::TAU;
                }
                while d < -std::f32::consts::PI {
                    d += std::f32::consts::TAU;
                }
                d.abs().to_degrees()
            } else {
                -1.0 // degeneriert
            }
        };
        // Heading der Tangente AM Zielpunkt (final_t) â€” gegen Truck-Heading vergleichbar.
        let final_tangent_heading_deg = if f_lt > 1e-6 {
            f_tt.x.atan2(-f_tt.z).to_degrees().rem_euclid(360.0)
        } else {
            -1.0
        };
        let final_is_prefab = index
            .metadata
            .get(final_seg)
            .and_then(|m| m.as_ref())
            .map(|m| m.is_prefab)
            .unwrap_or(false);
        ctx.blackboard.set(
            "lane_keeper.final_internal_kink_deg",
            format!("{final_internal_kink_deg:.4}"),
        );
        // Alternative-A-Diag (read-only): ab welcher t und ob auf dem Truck-Segment
        // gemessen wurde. final_seg != cur_seg â†’ intk_on_truck_seg=false, intK=0.
        ctx.blackboard
            .set("lane_keeper.intk_t_start", format!("{t_start_intk:.3}"));
        ctx.blackboard.set(
            "lane_keeper.intk_on_truck_seg",
            intk_on_truck_seg.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.final_tangent_heading_deg",
            format!("{final_tangent_heading_deg:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.final_seg_is_prefab",
            final_is_prefab.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.final_seg_length_m",
            format!("{:.2}", index.segments[final_seg].length_m),
        );

        // â”€â”€ Phase 2h-Wurzelfix v2 (Richtung B): interner KrÃ¼mmungs-Knick â†’ Catmull â”€â”€
        // Diag5/Diag6-Befund: die hohe interne KrÃ¼mmung tritt NICHT nur auf NavCurve-Prefabs
        // auf, sondern auch auf ROAD-Segmenten (Edge spannt Ã¼ber eine Kurve/Kreuzung; die
        // Quaternion-Tangenten von From-/To-Node laufen ~60â€“100Â° auseinander). Im SplineIndex
        // liegen Road-Segmente vor den angehÃ¤ngten NavCurves â†’ das Spike-Segment 1051105 ist
        // ein Road-Edge mit is_prefab=false. Der AuslÃ¶ser ist daher die interne KrÃ¼mmung ALLEIN,
        // unabhÃ¤ngig vom is_prefab-Flag. `final_is_prefab` wird nur noch zur Diagnose geloggt.
        let prefab_curve_threshold_deg = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.prefab_curve_fallback_deg")
            .map(|v| v as f32)
            .unwrap_or(PREFAB_CURVE_FALLBACK_DEG);
        // Phase 2h-Befund4-Fix: PlausibilitÃ¤ts-Obergrenze. Ãœber diesem Wert gilt der
        // intK als mis-orientiertes Junction-Tangenten-Artefakt (m0 = From-Node-
        // Quaternion-Forward), KEIN Latch â†’ route-aware-Spline trackt weiter.
        let intk_plausible_max_deg = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.intk_plausible_max_deg")
            .map(|v| v as f32)
            .unwrap_or(INTK_PLAUSIBLE_MAX_DEG);
        // Hysterese-Totband am Cap (Reviewer-Befund4): EINTRITT nur bis
        // intk_plausible_max_deg (100Â°); ein bereits aktiver Latch wird erst Ã¼ber
        // (cap + Margin) (110Â°) beendet. Verhindert Frame-zu-Frame-Flackern, wenn
        // intK durch Quaternion-Rauschen um den Cap oszilliert.
        let over_cap_entry = final_internal_kink_deg > intk_plausible_max_deg;
        let over_cap_exit =
            final_internal_kink_deg > intk_plausible_max_deg + INTK_PLAUSIBLE_MAX_EXIT_MARGIN_DEG;
        // Eintritt nur im plausiblen Band (threshold, cap]. Austritt unter
        // (Schwelle âˆ’ Margin), degeneriert, oder Ã¼ber (cap + Margin).
        let curve_over = final_internal_kink_deg >= 0.0
            && final_internal_kink_deg > prefab_curve_threshold_deg
            && !over_cap_entry;
        if self.prefab_curve_latched {
            // Austritt bei: degeneriert, ODER Ãœberschreiten von (cap + Margin),
            // ODER unter (Schwelle âˆ’ Margin). Der over_cap_exit-Austritt ist bewusst
            // (Task-1.3-Entscheidung): steigt intK aus dem legitimen Band (z.B. 85Â°)
            // Ã¼ber (cap + Margin) (131Â° > 110Â°), verlÃ¤sst der Latch das Catmull-Regime
            // SOFORT, statt im Artefakt hÃ¤ngen zu bleiben â€” der Spline ist dort die
            // bessere Mechanik, und der route-aware-nearest hat das richtige Segment.
            // Im Totband [100,110]Â° HÃ„LT ein aktiver Latch (kein Flackern).
            let exit_ok = final_internal_kink_deg < 0.0 // degeneriert â†’ raus
                || over_cap_exit // Artefakt-Regime (> cap + Margin) â†’ raus
                || final_internal_kink_deg
                    < (prefab_curve_threshold_deg - PREFAB_CURVE_EXIT_MARGIN_DEG);
            if exit_ok {
                self.prefab_curve_latched = false;
                self.prefab_curve_kink_deg = 0.0;
            }
        } else if curve_over {
            self.prefab_curve_latched = true;
            self.prefab_curve_kink_deg = final_internal_kink_deg as f64;
        }
        ctx.blackboard.set(
            "lane_keeper.internal_kink_over_threshold",
            curve_over.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.prefab_curve_threshold_deg",
            format!("{prefab_curve_threshold_deg:.1}"),
        );
        // Task 2 Diag: true wenn intK Ã¼ber der PlausibilitÃ¤ts-Obergrenze (Cap) liegt,
        // also Eintritt blockiert / als Junction-Tangenten-Artefakt behandelt wird.
        ctx.blackboard.set(
            "lane_keeper.final_internal_kink_capped",
            over_cap_entry.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.intk_plausible_max_deg",
            format!("{intk_plausible_max_deg:.1}"),
        );
        ctx.blackboard.set(
            "lane_keeper.prefab_curve_fallback",
            self.prefab_curve_latched.to_string(),
        );
        if self.prefab_curve_latched {
            // Catmull rundet die Kurve (bewÃ¤hrt). compute_heading_error fÃ¤llt bei None auf Catmull.
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "prefab_curve");
            ctx.blackboard.set(
                "lane_keeper.fallback_detail",
                format!("internal_curve_{final_internal_kink_deg:.0}deg"),
            );
            return None;
        }

        // Lane-Offset aus Metadaten des Lande-Segments.
        // Phase 2h: empirische Kalibrier-Konstante additiv NUR auf road-Segmente,
        // nicht auf prefab (NavCurves liegen bereits auf der Spur-Mitte).
        let cal = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.lane_offset_cal_m")
            .unwrap_or(0.0) as f32;
        let seg = &index.segments[final_seg];
        // Phase 2h-Diag3 (read-only, Task 2): the segment the lookahead-walk
        // landed on (where the steer target is taken). Differs from
        // nearest_segment_id once the arc-walk hops forward.
        ctx.blackboard
            .set("lane_keeper.lookahead_final_seg_id", final_seg.to_string());
        ctx.blackboard
            .set("lane_keeper.lookahead_final_t", format!("{final_t:.3}"));
        let look_point = evaluate(seg, final_t);
        let (lane_offset, source, lanes_in_dir): (f32, &str, u8) = match index.metadata[final_seg] {
            Some(m) if m.is_prefab => (0.0, "spline_prefab", 0),
            Some(m) => (m.lane_offset_right_m + cal, "spline_road", m.lanes_in_direction),
            None => (LANE_OFFSET_RIGHT_M as f32 + cal, "spline_road", 2),
        };
        ctx.blackboard.set(
            "lane_keeper.spline_seg_lanes_in_direction",
            lanes_in_dir.to_string(),
        );

        // Right-Normal an der lokalen Tangente (Fahrtrichtung = p0â†’p1, nur forward-Hops): n=(-tz,tx)/|t|.
        let tan = evaluate_tangent(seg, final_t);
        let len_xz = (tan.x * tan.x + tan.z * tan.z).sqrt();
        if len_xz < 1e-6 {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "degenerate_tangent");
            return None;
        }
        let n_x = (-tan.z / len_xz) as f64;
        let n_z = (tan.x / len_xz) as f64;
        let look_x = look_point.x as f64 + n_x * lane_offset as f64;
        let look_z = look_point.z as f64 + n_z * lane_offset as f64;

        // â”€â”€ Phase 2g-Diag2 (read-only): bricht die Kette Offset â†’ Lenk-Zielpunkt? â”€â”€
        // Task 1: Soll-Linie sichtbar machen. steer_target = Lookahead-Punkt MIT Offset (geht in
        // den heading_error/Lenkung), centerline = derselbe Lookahead OHNE Offset.
        //   |steer_target - centerline| â‰ˆ 0   â†’ Offset NICHT im Zielpunkt (H1)
        //   |steer_target - centerline| â‰ˆ 5.6 â†’ Offset IST im Zielpunkt (weiter zu H2 / Control-Law)
        let steer_dx = look_x - look_point.x as f64;
        let steer_dz = look_z - look_point.z as f64;
        let steer_target_minus_centerline = (steer_dx * steer_dx + steer_dz * steer_dz).sqrt();
        ctx.blackboard.set(
            "lane_keeper.steer_target_xz",
            format!("{look_x:.2},{look_z:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.spline_centerline_xz",
            format!("{:.2},{:.2}", look_point.x, look_point.z),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_target_minus_centerline_m",
            format!("{steer_target_minus_centerline:.3}"),
        );
        // Task 2: Right-Normal + Richtung relativ zur TRUCK-Fahrtrichtung (nicht nur zur Segment-
        // Tangente). dir_dot>0 â†’ Offset nach Truck-RECHTS (+1), <0 â†’ links (-1, H2-Vorzeichenfehler).
        let h_cw = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let truck_fx = h_cw.sin();
        let truck_fz = -h_cw.cos();
        let truck_right_x = -truck_fz; // right of travel = (-fz, fx)
        let truck_right_z = truck_fx;
        let dir_dot = n_x * truck_right_x + n_z * truck_right_z;
        let offset_direction_check = if (lane_offset as f64).abs() < 1e-6 {
            0.0
        } else if dir_dot >= 0.0 {
            1.0
        } else {
            -1.0
        };
        ctx.blackboard
            .set("lane_keeper.right_normal_xz", format!("{n_x:.3},{n_z:.3}"));
        ctx.blackboard.set(
            "lane_keeper.offset_direction_check",
            format!("{offset_direction_check:.0}"),
        );
        // Task 3: Truck-IST-Versatz gegen Centerline und gegen die Soll-Offset-Linie.
        //   truck_lat_vs_centerline â‰ˆ 0    â†’ Truck fÃ¤hrt MITTIG (Kette greift nicht)
        //   truck_lat_vs_centerline â‰ˆ +5.6 â†’ Truck auf der Soll-Spur (visuell evtl. fehlinterpretiert)
        //   truck_lat_vs_offsetline â‰ˆ 0    â†’ Truck IST auf der Offset-Linie
        ctx.blackboard.set(
            "lane_keeper.truck_lat_vs_centerline_m",
            format!("{truck_lat_vs_centerline:.3}"),
        );
        ctx.blackboard.set(
            "lane_keeper.signed_cte_m",
            format!("{truck_lat_vs_centerline:.3}"),
        );
        let e_lat = truck_lat_vs_centerline - lane_offset as f64;
        ctx.blackboard.set(
            "lane_keeper.truck_lat_vs_offsetline_m",
            format!("{:.3}", e_lat),
        );
        ctx.blackboard.set(
            "lane_keeper.lat_error_reference",
            "heading_to_offset_lookahead_crosstrack",
        );

        // Diagnostik
        ctx.blackboard.set("lane_keeper.lateral_source", source);
        ctx.blackboard.set(
            "lane_keeper.lane_offset_applied_m",
            format!("{lane_offset:.3}"),
        );
        // Phase 2h-Diag: Offset-Sprung an road/prefab/Catmull-Grenzen sichtbar machen.
        let offset_delta = lane_offset - self.prev_lane_offset_m;
        let source_changed = if source != self.prev_lateral_source.as_str() {
            1u8
        } else {
            0u8
        };
        ctx.blackboard
            .set("lane_keeper.offset_delta_m", format!("{offset_delta:.3}"));
        ctx.blackboard
            .set("lane_keeper.source_changed", source_changed.to_string());
        self.prev_lane_offset_m = lane_offset;
        self.prev_lateral_source = source.to_string();
        ctx.blackboard
            .set("lane_keeper.lookahead_hop_count", hop_count.to_string());
        ctx.blackboard
            .set("lane_keeper.current_hop", format!("{a0}->{b0}"));
        ctx.blackboard.set(
            "lane_keeper.node_progress_idx",
            self.node_progress_idx.to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{look_x:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{look_z:.2}"));

        // Spline-Pfad hat gegriffen â†’ Fallback-Grund = "none" (kein Fallback).
        ctx.blackboard.set("lane_keeper.fallback_reason", "none");
        ctx.blackboard.set("lane_keeper.fallback_detail", "none");

        // Shared Tail â€” IDENTISCH zum Catmull-Pfad; 2b-Heading-Konvert UNVERÃ„NDERT.
        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return Some(0.0);
        }
        let target = dx.atan2(-dz);
        let heading_rad = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let mut err = target - heading_rad;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }

        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{target:.6}"));
        Some(err)
    }

    fn resync_progress_idx(&mut self, tx: f64, tz: f64) {
        if self.waypoints.len() < 2 {
            return;
        }
        let mut best_i = 0usize;
        let mut best_d = f64::MAX;
        for (i, &[wx, wz]) in self.waypoints.iter().enumerate() {
            let d = (tx - wx).powi(2) + (tz - wz).powi(2);
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        self.progress_idx = best_i.min(self.waypoints.len().saturating_sub(2));
    }

    fn compute_heading_error(
        &mut self,
        tx: f64,
        tz: f64,
        heading: f64,
        speed_ms: f64,
        ctx: &PluginContext,
    ) -> f64 {
        if let Some(err) = self.try_spline_heading_error(tx, tz, heading, speed_ms, ctx) {
            self.was_spline_active = true;
            return err;
        }
        if self.was_spline_active {
            self.was_spline_active = false;
            self.resync_progress_idx(tx, tz); // progress_idx auf nÃ¤chsten Smoothed-Waypoint setzen
        }
        ctx.blackboard
            .set("lane_keeper.lateral_source", "catmullrom_fallback");
        // H2-Catmull-Offset-Fix: statt des festen LANE_OFFSET_RIGHT_M (1.875 m, egal ob
        // 1-/2-/3-spurig) den per-Segment-`lane_offset_right_m` des nearest-Segments
        // verwenden â€” konsistent zum Spline-Pfad (lib.rs:1297-1318). Quelle:
        // `self.last_nearest_seg` (diesen Tick in try_spline gesetzt) â†’ `index.metadata[idx]`.
        // Der Kalibrierwert `cal` wird â€” wie im Spline-Pfad â€” auf road/Default addiert,
        // NICHT auf prefab (NavCurves liegen bereits auf der Spur-Mitte).
        //   Some(Some(m)) is_prefab=true â†’ 0.0  (konsistent zum Spline-prefab-Arm lib.rs:1298)
        //   Some(Some(m))                â†’ m.lane_offset_right_m + cal (lane-count-abhÃ¤ngig)
        //   _ (None / kein frisches nearest / idx out of range / metadata None)
        //                                â†’ 1.875 + cal Default, NICHT 0 (sonst mittig auf 1-spurig).
        let cal = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.lane_offset_cal_m")
            .unwrap_or(0.0) as f32;
        let (catmull_offset, catmull_lanes) = match self.last_nearest_seg.and_then(|idx| {
            self.index
                .as_ref()
                .and_then(|ix| ix.metadata.get(idx).copied())
        }) {
            Some(Some(m)) if m.is_prefab => (0.0_f32, 0u8),
            Some(Some(m)) => (m.lane_offset_right_m + cal, m.lanes_in_direction),
            _ => (LANE_OFFSET_RIGHT_M as f32 + cal, 2u8),
        };
        ctx.blackboard.set(
            "lane_keeper.lane_offset_applied_m",
            format!("{catmull_offset:.3}"),
        );
        ctx.blackboard.set(
            "lane_keeper.catmull_seg_lanes_in_direction",
            catmull_lanes.to_string(),
        );
        let offset_delta = catmull_offset - self.prev_lane_offset_m;
        let source_changed = if "catmullrom_fallback" != self.prev_lateral_source.as_str() {
            1u8
        } else {
            0u8
        };
        ctx.blackboard
            .set("lane_keeper.offset_delta_m", format!("{offset_delta:.3}"));
        ctx.blackboard
            .set("lane_keeper.source_changed", source_changed.to_string());
        self.prev_lane_offset_m = catmull_offset;
        self.prev_lateral_source = "catmullrom_fallback".to_string();

        if self.waypoints.len() < 2 {
            return 0.0;
        }

        while self.progress_idx + 1 < self.waypoints.len() {
            let [wx, wz] = self.waypoints[self.progress_idx + 1];
            let dist = ((tx - wx).powi(2) + (tz - wz).powi(2)).sqrt();
            if dist < WAYPOINT_REACH_M {
                self.progress_idx += 1;
            } else {
                break;
            }
        }

        if self.progress_idx + 1 >= self.waypoints.len() {
            ctx.blackboard.set("lane_keeper.skip_reason", "route_end");
            ctx.blackboard.set("lane_keeper.error_rad", "0.0");
            ctx.blackboard.set(
                "lane_keeper.progress_idx_after_advance",
                self.progress_idx.to_string(),
            );
            ctx.blackboard.set("lane_keeper.waypoints_remaining", "0");
            ctx.blackboard.set("lane_keeper.advance_check_dist", "0.00");
            ctx.blackboard.set("lane_keeper.walk_iterations", "0");
            ctx.blackboard.set("lane_keeper.walk_accumulated_m", "0.00");
            return 0.0;
        }

        let [nx, nz] = self.waypoints[self.progress_idx + 1];
        let advance_check_dist = ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt();

        let base_look_ahead = BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR;
        // Phase 2h-Befund2-Fix: bei Catmull-Fallback und hoher interner KrÃ¼mmung
        // Lookahead inverse-square skalieren. Nutzt Struct-Felder (self.prefab_curve_latched,
        // self.prefab_curve_kink_deg) statt BB-Reads, um Stale-Werte zu vermeiden.
        let curve_factor = if self.prefab_curve_latched && self.prefab_curve_kink_deg >= 1.0 {
            let threshold = ctx
                .blackboard
                .get_f64("plugin.lane_keeper.prefab_curve_fallback_deg")
                .unwrap_or(PREFAB_CURVE_FALLBACK_DEG as f64);
            if self.prefab_curve_kink_deg > threshold {
                (threshold / self.prefab_curve_kink_deg).powi(2).min(1.0)
            } else {
                1.0f64
            }
        } else {
            1.0f64
        };
        let look_ahead = if self.prefab_curve_latched {
            let min_la = ctx
                .blackboard
                .get_f64("plugin.lane_keeper.catmull_min_look_ahead_m")
                .unwrap_or(CATMULL_CURVE_MIN_LOOK_AHEAD);
            (base_look_ahead * curve_factor).max(min_la)
        } else {
            base_look_ahead
        };

        // Phase 2h-Befund2-Fix (Route-Hop-Limit): Walk auf die nÃ¤chsten N Route-Hops
        // begrenzen (Option 3A, progress_idx-RÃ¼ckrechnung). Verhindert dass der Walk
        // um Kreuzungskurven auf Waypoints jenseits der Kreuzungsmitte zielt.
        // Fallback auf unbegrenzt wenn cached_route_node_ids < 2 (kein Routing aktiv).
        let max_hops = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.catmull_max_route_hops")
            .map(|v| v as usize)
            .unwrap_or(CATMULL_MAX_ROUTE_HOPS);
        let route_len = self.cached_route_node_ids.len();
        let walk_end = if route_len >= 2 {
            let current_route_idx = (self.progress_idx + self.subdivisions / 2) / self.subdivisions;
            let max_route_idx = (current_route_idx + max_hops).min(route_len - 1);
            // walk_end ist exklusive Obergrenze (wie waypoints.len() im Fallback-Ast).
            // max_route_idx * subdivisions ist der INKLUSIVE Waypoint-Index des Ziel-Knotens;
            // +1 macht daraus die exklusive Grenze fÃ¼r den Slice [walk_start..walk_end].
            (max_route_idx * self.subdivisions + 1).min(self.waypoints.len())
        } else {
            // Kein Routing â†’ altes Verhalten (unbegrenzt).
            self.waypoints.len()
        };
        let walk_start = self.progress_idx + 1;

        let mut look_x = tx;
        let mut look_z = tz;
        let mut accumulated = 0.0;
        let mut walk_iterations: usize = 0;

        if walk_start < walk_end {
            for &[px, pz] in &self.waypoints[walk_start..walk_end] {
                let seg = ((px - look_x).powi(2) + (pz - look_z).powi(2)).sqrt();
                accumulated += seg;
                walk_iterations += 1;
                look_x = px;
                look_z = pz;
                if accumulated >= look_ahead {
                    break;
                }
            }
        }

        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard.set(
            "lane_keeper.catmull_effective_look_ahead_m",
            format!("{look_ahead:.2}"),
        );
        if self.prefab_curve_latched {
            ctx.blackboard.set(
                "lane_keeper.catmull_curve_factor",
                format!("{curve_factor:.3}"),
            );
        }
        // Phase 2h-Befund2 Diag-Keys (Route-Hop-Limit, read-only).
        ctx.blackboard
            .set("lane_keeper.catmull_max_hops", max_hops.to_string());
        ctx.blackboard.set(
            "lane_keeper.catmull_current_route_idx",
            if route_len >= 2 {
                let i = (self.progress_idx + self.subdivisions / 2) / self.subdivisions;
                i.to_string()
            } else {
                "n/a".to_string()
            },
        );
        // catmull_max_waypoint_idx: inklusiver letzter erreichbarer Waypoint-Index
        // (= walk_end - 1 wenn Route aktiv). catmull_walk_end ist die exklusive Grenze.
        ctx.blackboard.set(
            "lane_keeper.catmull_max_waypoint_idx",
            if route_len >= 2 {
                walk_end.saturating_sub(1).to_string()
            } else {
                "n/a".to_string()
            },
        );
        ctx.blackboard
            .set("lane_keeper.catmull_walk_end", walk_end.to_string());
        ctx.blackboard.set(
            "lane_keeper.catmull_total_waypoints",
            self.waypoints.len().to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{look_x:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{look_z:.2}"));
        ctx.blackboard
            .set("lane_keeper.dx", format!("{:.2}", look_x - tx));
        ctx.blackboard
            .set("lane_keeper.dz", format!("{:.2}", look_z - tz));
        ctx.blackboard
            .set("lane_keeper.walk_iterations", walk_iterations.to_string());
        ctx.blackboard.set(
            "lane_keeper.walk_accumulated_m",
            format!("{accumulated:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.advance_check_dist",
            format!("{advance_check_dist:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.progress_idx_after_advance",
            self.progress_idx.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.waypoints_remaining",
            self.waypoints
                .len()
                .saturating_sub(self.progress_idx + 1)
                .to_string(),
        );

        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return 0.0;
        }

        // Phase 2h-Diag6 (read-only): rohe Waypoint-Centerline VOR dem Offset festhalten.
        let catmull_cl_x = look_x;
        let catmull_cl_z = look_z;

        // Shift lookahead right by `catmull_offset` (Rechtsfahrgebot, H2-Fix: lane-count-
        // abhÃ¤ngig statt fix 1.875 m). Right-normal in ETS2 XZ (x=East, z=South): (-dz, dx)/|d|.
        // Mirrors lane-follower/src/lib.rs:913-918. Division safe: 1e-12 guard above.
        let len_xz = (dx * dx + dz * dz).sqrt();
        let off = catmull_offset as f64;
        let look_x = look_x + (-dz / len_xz) * off;
        let look_z = look_z + (dx / len_xz) * off;
        let dx = look_x - tx;
        let dz = look_z - tz;

        // â”€â”€ Phase 2h-Diag6 (read-only): Catmull-Pfad-Zielgeometrie â”€â”€
        // H2-Fix: der Catmull-Fallback wendet jetzt den lane-count-abhÃ¤ngigen `catmull_offset`
        // an (Quelle: nearest-Segment-Metadaten), nicht mehr fix 1.875 m. catmull_centerline =
        // roher Waypoint-Lookahead (vor Offset); catmull_steer_target = nach Offset (geht in den
        // heading_error); catmull_truck = Truck-Weltpos.
        ctx.blackboard.set(
            "lane_keeper.catmull_steer_target_xz",
            format!("{look_x:.2},{look_z:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.catmull_centerline_xz",
            format!("{catmull_cl_x:.2},{catmull_cl_z:.2}"),
        );
        ctx.blackboard
            .set("lane_keeper.catmull_truck_xz", format!("{tx:.2},{tz:.2}"));
        ctx.blackboard.set(
            "lane_keeper.catmull_offset_applied_m",
            format!("{catmull_offset:.3}"),
        );
        // Vektor Truckâ†’Ziel relativ zur Truck-Fahrtrichtung: LÃ¤ngs-/Quer-Anteil. GroÃŸer |quer|
        // bei kleinem lÃ¤ngs â‡’ Ziel liegt seitlich â‡’ heading-only zieht quer (Befund 2).
        let h_cw = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let fwd_x = h_cw.sin();
        let fwd_z = -h_cw.cos();
        let along = dx * fwd_x + dz * fwd_z;
        let lateral = dx * (-fwd_z) + dz * fwd_x; // +rechts / -links
        ctx.blackboard
            .set("lane_keeper.catmull_target_along_m", format!("{along:.2}"));
        ctx.blackboard.set(
            "lane_keeper.catmull_target_lateral_m",
            format!("{lateral:.2}"),
        );

        let target = dx.atan2(-dz);
        // t.heading (Telemetry) is ETS2 SDK format: [0..1] CCW from North.
        // Convert to CW radians (0=N, Ï€/2=E) to match target's convention.
        // Formula mirrors lane-follower: (-raw * 2Ï€).rem_euclid(2Ï€).
        let heading_rad = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let mut err = target - heading_rad;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }

        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{target:.6}"));
        err
    }

    fn tick_request_route_following(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        // heading_stage is set by tick() before tick_request(). In tests that call
        // tick_request() directly, the field is pre-set via struct literal.
        // Do NOT re-read from blackboard here â€” that would overwrite the pre-set value.

        if !ctx.is_active() {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            if !self.waypoints.is_empty() {
                self.waypoints.clear();
                self.last_waypoints_hash = 0;
                self.progress_idx = 0;
                tracing::info!("[lane-keeper] state=Off, cleared waypoint cache");
            }
            // Phase 2c/2d: Spline-Route-Zustand beim Disengage zurÃ¼cksetzen â€”
            // unconditional, NICHT im waypoints-Sub-Block (der wird Ã¼bersprungen
            // wenn waypoints schon leer). Sonst startet Re-Engage auf identischer
            // Route (gleicher route_node_ids-Hash â†’ kein Reset in try_spline) mit
            // stale node_progress_idx mitten in der alten Route (Reviewer-Finding A).
            self.node_progress_idx = 0;
            self.was_spline_active = false;
            self.cached_route_hash = 0;
            self.cached_route_node_ids.clear();
            // Phase 2h-Safety: Re-Engage sauber starten.
            self.safety_autoreplan_secs = 0.0;
            // Schritt 2: Junction-Failsafe-Grace bei Off/Re-Engage zurÃ¼cksetzen.
            self.junction_failsafe_secs = 0.0;
            // Phase 2h-Wurzelfix: Kink-Stuck-ZÃ¤hler bei Off/Disengage zurÃ¼cksetzen.
            self.kink_stuck_secs = 0.0;
            self.kink_stuck_hop = (0, 0);
            // Phase 2h-Wurzelfix v2: Prefab-Curve-Latch bei Off/Disengage zurÃ¼cksetzen.
            self.prefab_curve_latched = false;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "state_not_active");
            // Diag-only (Engage-Debug): Engage-Gate-Vorschau auch VOR dem lane_only-
            // Engage. Im Off-Zustand steht der Mode noch auf route_following (der
            // Auto-Switch auf nearest_spline passiert erst beim Engage-Event) â€” ohne
            // diese Vorschau wÃ¤ren die engage_*-Diag-Keys vor dem Engage unsichtbar.
            // Schreibt NUR die Diag-Keys, NICHT lane_keeper.engage_allowed.
            if let (Some(index), Some(t)) = (self.index.clone(), telemetry) {
                let query = Vec3::new(
                    t.position[0] as f32,
                    t.position[1] as f32,
                    t.position[2] as f32,
                );
                let truck_heading_deg = ((-t.heading) * 360.0).rem_euclid(360.0) as f32;
                let cands = index.within_radius_filtered_heading(
                    query,
                    NEAREST_QUERY_RADIUS_M,
                    truck_heading_deg.to_radians(),
                    NEAREST_HEADING_TOL_DEG.to_radians(),
                    |_idx, _meta| true,
                );
                let fresh_raw = cands.first();
                let would_allow = fresh_raw.is_some_and(|h| {
                    h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= self.engage_max_lateral_m
                });
                self.publish_engage_gate_diag(
                    &index,
                    query,
                    truck_heading_deg,
                    fresh_raw,
                    would_allow,
                    ctx,
                );
            } else {
                // Sentinels statt stale Werte vom letzten Tick (Reviewer-Finding).
                ctx.blackboard.set("lane_keeper.engage_seg_found", "false");
                ctx.blackboard.set("lane_keeper.engage_dist_m", "-1.00");
                ctx.blackboard
                    .set("lane_keeper.engage_heading_diff_deg", "-1.0");
                ctx.blackboard.set(
                    "lane_keeper.engage_block_reason",
                    if telemetry.is_none() {
                        "no_telemetry"
                    } else {
                        "no_spline_index"
                    },
                );
            }
            return None;
        }

        self.apply_gain_overrides(ctx);

        let t = telemetry?;

        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.safety_autoreplan_secs = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            ctx.blackboard.set("lane_keeper.safety_state", "normal");
            return None;
        }

        if self.waypoints.is_empty() {
            self.safety_autoreplan_secs = 0.0;
            ctx.blackboard
                .set("lane_keeper.skip_reason", "no_waypoints");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.safety_state", "normal");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        self.publish_r3_lane_diag_keys(ctx);

        let err =
            self.compute_heading_error(t.position[0], t.position[2], t.heading, t.speed_ms, ctx);

        // ETS2LA-Fix: der heading_mismatch-Safety-Gate prueft gegen den LOKALEN Heading-
        // Fehler (Truck vs nahe Segment-Tangente), NICHT gegen den fernen Lookahead-`err`
        // (der spiked in Kurven -> Truck bremste dort grundlos hart und fror ein, weil
        // gestoppt das Heading nie korrigiert). Spline-Pfad: try_spline hat
        // heading_mismatch_herr_rad bereits auf den lokalen Fehler gesetzt. Catmull-
        // Fallback / Early-Return (kein Spline aktiv): Lookahead-`err` wie bisher.
        if !self.was_spline_active {
            self.heading_mismatch_herr_rad = err;
        }
        let mismatch_herr = self.heading_mismatch_herr_rad;
        ctx.blackboard.set(
            "lane_keeper.mismatch_herr_deg",
            format!("{:.1}", mismatch_herr.to_degrees()),
        );

        if mismatch_herr.abs() > HEADING_MISMATCH_THRESHOLD_RAD {
            // Phase 2h-Safety: heading_mismatch (>1.4 rad / >80Â°) is the most
            // dangerous situation â€” must brake, not coast silently. Recoverable,
            // so accumulate time like AutoReplan.
            return self.safety_brake_request(
                "heading_mismatch",
                "decelerating_heading_mismatch",
                false,
                t.speed_ms,
                dt,
                mismatch_herr,
                ctx,
            );
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        // Phase 2h-Diag3 (read-only): which heading-stage the lane-keeper acted on
        // this tick. AutoReplan/Disengaging â‡’ steering nulled below.
        ctx.blackboard.set("lane_keeper.stage_seen", stage);

        // Copy flags before `&mut self` borrow in safety_brake_request.
        let is_autoreplan = stage == "AutoReplan";
        let is_disengaging = stage == "Disengaging";
        let stage_owned = stage.to_owned();
        if is_autoreplan || is_disengaging {
            let immediate = is_disengaging;
            let state = if immediate {
                "disengaging_lane_authority_lost"
            } else {
                "decelerating_lane_authority_lost"
            };
            return self.safety_brake_request(
                &stage_owned,
                state,
                immediate,
                t.speed_ms,
                dt,
                err,
                ctx,
            );
        }

        let stage_changed = self.previous_heading_stage != self.heading_stage;
        let transitional = stage_changed
            && (self.heading_stage.as_deref() == Some("SoftLaneKeep")
                || self.previous_heading_stage.as_deref() == Some("SoftLaneKeep"));
        if transitional {
            self.pid.reset();
        }
        self.previous_heading_stage = self.heading_stage.clone();

        let effective_err = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            err * 0.3
        } else {
            err
        };

        let raw = self.pid.update(effective_err, dt).clamp(-1.0, 1.0);

        // â”€â”€ Phase 2h-Diag2 (read-only): Steer-Herkunft aufschlÃ¼sseln â”€â”€
        // Frage A: ist 0.8281 = geklemmter Max-Wert (output_clamp_active=true,
        // unclamped > 1.0) oder echter Regler-Output (steer_p+steer_i+steer_d
        // â‰ˆ 0.828, unclamped < 1.0)? Bei kp=0.8, ki=0.1, integral_limit=2.0 ergibt
        // ein konstanter heading_errorâ‰ˆ0.785 rad: pâ‰ˆ0.628, i_satâ‰ˆ0.2, dâ‰ˆ0 â†’ 0.828
        // OHNE Clamp â†’ dann ist der ferne Lookahead (heading-only ohne Cross-Track)
        // die Wurzel, nicht ein Clamp.
        ctx.blackboard
            .set("lane_keeper.heading_error_rad", format!("{err:.6}"));
        ctx.blackboard.set(
            "lane_keeper.effective_err_rad",
            format!("{effective_err:.6}"),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_p_term",
            format!("{:.6}", self.pid.last_p()),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_i_term",
            format!("{:.6}", self.pid.last_i()),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_d_term",
            format!("{:.6}", self.pid.last_d()),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_unclamped",
            format!("{:.6}", self.pid.last_unclamped()),
        );
        ctx.blackboard
            .set("lane_keeper.steer_raw", format!("{raw:.6}"));
        ctx.blackboard.set(
            "lane_keeper.steer_output_clamp_active",
            self.pid.last_output_clamped().to_string(),
        );

        let delta_raw = raw - self.previous_steering_out;
        let delta_clamped =
            delta_raw.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let steering = self.previous_steering_out + delta_clamped;
        self.previous_steering_out = steering;

        let was_rate_limited = (delta_clamped - delta_raw).abs() > 1e-9;
        ctx.blackboard.set(
            "lane_keeper.steering_rate_limited",
            was_rate_limited.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta_raw - delta_clamped),
        );

        // Phase 2h-Diag3 (read-only): this tick DID emit a steering opinion
        // (Some) â†’ distinguishes a real drive tick from a frozen/stale one.
        ctx.blackboard.set("lane_keeper.returned_none", "false");
        ctx.blackboard.set("lane_keeper.null_steer_cause", "none");
        // Phase 2h-Safety: Stage recovery â†’ reset accumulator and clear safety keys.
        self.safety_autoreplan_secs = 0.0;
        // Schritt 2: Recovery â†’ Junction-Failsafe-Grace zurÃ¼cksetzen.
        self.junction_failsafe_secs = 0.0;
        ctx.blackboard
            .set("lane_keeper.junction_failsafe_active", "false");
        ctx.blackboard.set("lane_keeper.safety_state", "normal");
        ctx.blackboard
            .set("lane_keeper.steering_suppressed", "false");
        ctx.blackboard.set("lane_keeper.safety_brake", "0.0000");
        ctx.blackboard
            .set("lane_keeper.safety_autoreplan_secs", "0.00");
        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.error_rad", format!("{err:.6}"));
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        ctx.blackboard.set(
            "lane_keeper.waypoints_loaded",
            self.waypoints.len().to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.progress_idx", self.progress_idx.to_string());
        ctx.blackboard.set("lane_keeper.dt_s", format!("{dt:.6}"));
        ctx.blackboard
            .set("lane_keeper.truck_x", format!("{:.2}", t.position[0]));
        ctx.blackboard
            .set("lane_keeper.truck_z", format!("{:.2}", t.position[2]));
        ctx.blackboard
            .set("lane_keeper.truck_heading", format!("{:.6}", t.heading));
        ctx.blackboard
            .set("lane_keeper.truck_speed_ms", format!("{:.2}", t.speed_ms));

        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        let block_reason = if stage == "SoftLaneKeep" {
            "none_soft_scaled"
        } else {
            "none"
        };
        self.publish_stage_steering_diag(ctx, stage, block_reason, true);

        Some(ControlRequest {
            steering: Some(steering),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }
}

// â”€â”€ NearestSpline-mode implementation (routerless lane-following, Weg B) â”€â”€â”€â”€â”€â”€â”€
//
// Folgt der lokalen Spline-Geometrie unter dem Truck OHNE Router-Route. Die
// Lenk-Mathematik (Look-Ahead-Punkt + v3-Cross-Track + PID + Rate-Limit) bleibt
// unberÃ¼hrt; nur die Segment-AUSWAHL lÃ¤uft als TOPOLOGIE-CHAIN statt per-Frame-Spatial:
// der Truck klebt am current_segment und schaltet nur am Segmentende Ã¼ber die gerichtete
// `forward_adj` weiter (Gegenspur ist KEIN VorwÃ¤rts-Nachfolger â†’ strukturell unwÃ¤hlbar â†’
// behebt den 180Â°-Sprung). Eine Spatial-Query (60Â°-Filter) lÃ¤uft NUR beim Einstieg
// (Engage) und bei Chain-Abriss/Drift als Re-Acquisition. RouteFollowing bleibt vollstÃ¤ndig
// unberÃ¼hrt.

/// Ergebnis der Chain-Segment-Auswahl.
enum NearestSelection {
    /// Ein verfolgbares Segment wurde gewÃ¤hlt â†’ Lenken.
    Steer {
        seg_idx: usize,
        t: f32,
        heading_diff_deg: f32,
        /// Legacy-Vokabular fÃ¼r `nearest_seg_switch_reason`
        /// (initial | fwd_progress | junction_pick | sticky).
        switch_reason: &'static str,
        /// Chain-Vokabular fÃ¼r `chain_advance_reason`
        /// (sticky | segment_end | off_segment | reacquire).
        chain_reason: &'static str,
        /// Anzahl der forward_adj-Nachfolger, die am Ãœbergang NACH dem Reverse-Skip den
        /// Richtungs-Filter passierten (0 auÃŸerhalb eines Ãœbergangs).
        successor_count: usize,
        /// Anzahl der am Ãœbergang Ã¼bersprungenen exakten Reverse-Geschwister (from/to
        /// vertauscht). >0 â‡’ ein 2-Zyklus-Kandidat wurde strukturell ausgeschlossen.
        reverse_skipped: usize,
        /// true, wenn die Topologie-Chain nicht weiterlief und auf die Spatial-Query
        /// zurÃ¼ckgefallen werden musste (forward_adj leer / alle gefiltert / Drift).
        chain_broken: bool,
    },
    /// Kein gÃ¼ltiges Segment â†’ KEIN Steering (Lenkrad zentriert, kein Brake).
    /// cause: no_segment | heading_filter_exceeded
    Null { cause: &'static str },
}

/// Minimaler Winkelabstand zweier Headings (Grad), Ergebnis in [0, 180].
fn angular_diff_deg(a: f32, b: f32) -> f32 {
    let d = (a - b).abs() % 360.0;
    if d > 180.0 {
        360.0 - d
    } else {
        d
    }
}

/// Heading-Differenz (Grad) zwischen Truck und Segment-Tangente am Parameter `t`.
/// ETS2-Konvention: `heading_deg = atan2(tan.x, -tan.z)` (0=N, 90=O), CW von Nord.
fn seg_heading_diff_deg(seg: &HermiteSegment, t: f32, truck_heading_deg: f32) -> f32 {
    let tan = evaluate_tangent(seg, t);
    let len = (tan.x * tan.x + tan.z * tan.z).sqrt();
    if len < 1e-6 {
        return 180.0; // degeneriert â†’ als max behandeln
    }
    let seg_heading_deg = f32::atan2(tan.x, -tan.z).to_degrees().rem_euclid(360.0);
    angular_diff_deg(truck_heading_deg, seg_heading_deg)
}

/// Prefab-Bias-Distanz: NavCurves (is_prefab) bekommen +`NEAREST_PREFAB_BIAS_M`
/// Bonus (â†’ kleinere effektive Distanz) gegenÃ¼ber Road-Chords. NavCurves existieren
/// nur in Prefabs/Junctions, daher wirkt der Bias faktisch nur in der Junction-Zone.
fn effective_select_dist(h: &HeadingFilteredHit) -> f32 {
    let is_prefab = h.meta.is_some_and(|m| m.is_prefab);
    h.dist_m
        - if is_prefab {
            NEAREST_PREFAB_BIAS_M
        } else {
            0.0
        }
}

/// Effektiver Lookahead (m) im NearestSpline-Pfad: geschwindigkeitsabhÃ¤ngig, aber
/// mit hartem Floor `NEAREST_MIN_LOOK_AHEAD` gegen die Kriechtempo-Oszillation.
/// RouteFollowing nutzt diese Funktion NICHT (eigener Lookahead, `BASE_LOOK_AHEAD`
/// unverÃ¤ndert).
fn nearest_look_ahead_m(speed_kmh: f64) -> f32 {
    (BASE_LOOK_AHEAD + speed_kmh * SPEED_FACTOR).max(NEAREST_MIN_LOOK_AHEAD) as f32
}

#[cfg(test)]
mod lookahead_floor_tests {
    use super::*;

    #[test]
    fn lookahead_floor_at_low_speed() {
        // Stillstand: ohne Floor wÃ¤ren es BASE_LOOK_AHEAD (5 m). Mit Floor >= 12 m.
        assert!((nearest_look_ahead_m(0.0) - NEAREST_MIN_LOOK_AHEAD as f32).abs() < 1e-4);
        assert!(nearest_look_ahead_m(0.0) >= NEAREST_MIN_LOOK_AHEAD as f32);
        assert!(
            nearest_look_ahead_m(0.0) > (BASE_LOOK_AHEAD as f32),
            "Floor muss bei 0 km/h deutlich Ã¼ber den alten 5 m liegen"
        );
        // 6 km/h: 5 + 3 = 8 m < Floor â†’ auf 12 m angehoben.
        assert!((nearest_look_ahead_m(6.0) - 12.0).abs() < 1e-4);
    }

    #[test]
    fn lookahead_unchanged_at_high_speed() {
        // 45 km/h: 5 + 22.5 = 27.5 m, deutlich Ã¼ber Floor â†’ Floor wirkungslos.
        let expected = (BASE_LOOK_AHEAD + 45.0 * SPEED_FACTOR) as f32; // 27.5
        assert!((nearest_look_ahead_m(45.0) - expected).abs() < 1e-4);
        assert!(nearest_look_ahead_m(45.0) > NEAREST_MIN_LOOK_AHEAD as f32);
    }

    #[test]
    fn lookahead_floor_transition_point() {
        // Floor greift bis ~14 km/h (5 + 0.5*14 = 12). DarÃ¼ber transparent.
        assert!((nearest_look_ahead_m(14.0) - 12.0).abs() < 1e-4);
        assert!(nearest_look_ahead_m(20.0) > 12.0); // 5 + 10 = 15
    }

    #[test]
    fn route_following_lookahead_unchanged() {
        // RouteFollowing nutzt KEINEN Floor: BASE_LOOK_AHEAD bleibt 5 m, und die
        // route-seitige Formel (ohne .max(FLOOR)) liefert bei Kriechtempo weiterhin
        // < 12 m. Guard gegen versehentliches Anheben von BASE_LOOK_AHEAD (Option a).
        assert!((BASE_LOOK_AHEAD - 5.0).abs() < 1e-9);
        let route_low = (BASE_LOOK_AHEAD + 0.0 * SPEED_FACTOR) as f32; // 5 m, kein Floor
        assert!((route_low - 5.0).abs() < 1e-4);
        // Der Floor betrifft ausschlieÃŸlich den NearestSpline-Pfad.
        assert!(nearest_look_ahead_m(0.0) > route_low);
    }
}

impl LaneKeeperPlugin {
    /// Baut Arc-Length-LUTs + forward_adj Ã¼ber ALLE Index-Segmente â€” lazy, beim
    /// ersten NearestSpline-Tick. RouteFollowing-Nutzer zahlen nichts.
    fn ensure_nearest_index_built(&mut self, index: &SplineIndex) {
        if self.nearest_index_built {
            return;
        }
        self.nearest_luts = build_all_luts(&index.segments);
        self.nearest_forward_adj = build_forward_adjacency(&index.segments);
        self.nearest_index_built = true;
        tracing::info!(
            "[lane-keeper] NearestSpline index built: {} LUTs, {} adjacency keys",
            self.nearest_luts.len(),
            self.nearest_forward_adj.len()
        );
    }

    /// Spatial-Akquise/-Re-Acquisition: den frischen 60Â°-Spatial-Kandidaten (falls vorhanden)
    /// zum neuen Chain-Anker machen. `chain_broken=true` markiert einen Abriss-Fallback
    /// (forward_adj leer/gefiltert oder Drift), `false` die erwartete Initial-Akquise beim
    /// Einstieg. Kein 60Â°-Kandidat im Radius â†’ Null(`fallback_cause`), KEIN Steering.
    fn spatial_anchor(
        fresh: Option<&HeadingFilteredHit>,
        switch_reason: &'static str,
        chain_reason: &'static str,
        chain_broken: bool,
        reverse_skipped: usize,
        fallback_cause: &'static str,
    ) -> NearestSelection {
        match fresh {
            Some(h) => NearestSelection::Steer {
                seg_idx: h.idx,
                t: h.t,
                heading_diff_deg: h.heading_diff_rad.to_degrees(),
                switch_reason,
                chain_reason,
                successor_count: 0,
                reverse_skipped,
                chain_broken,
            },
            None => NearestSelection::Null {
                cause: fallback_cause,
            },
        }
    }

    /// Chain-Segment-Auswahl (nur im Active-Fall aufgerufen). Statt per-Frame-Spatial-Suche
    /// eine Topologie-Chain: am current_segment kleben, am Ende Ã¼ber `forward_adj`
    /// weiterschalten, Spatial nur bei Einstieg/Abriss/Drift.
    ///
    /// `fresh`: bester heading-gefilterter (â‰¤60Â°, prefab-biased) Spatial-Kandidat dieses
    /// Ticks â€” Eingang NUR fÃ¼r Initial-Akquise und Re-Acquisition. Im stationÃ¤ren Chain-Lauf
    /// (sticky / segment_end) wird er bewusst NICHT verwendet (keine rÃ¤umlichen SprÃ¼nge).
    fn select_nearest_segment(
        &self,
        index: &SplineIndex,
        query: Vec3,
        truck_heading_deg: f32,
        fresh: Option<&HeadingFilteredHit>,
    ) -> NearestSelection {
        // Aktuelles Chain-Segment unter dem Truck projizieren.
        let cur = self.nearest_seg_idx.and_then(|i| {
            index.segments.get(i).and_then(|seg| {
                index.project_on_segment(i, query).map(|(t, d)| {
                    let hd = seg_heading_diff_deg(seg, t, truck_heading_deg);
                    (i, t, d, hd)
                })
            })
        });

        let Some((ci, ct, cd, chd)) = cur else {
            // Kein aktuelles Segment â†’ INITIALE Spatial-Akquise (Engage / nach Null-Reset).
            // Erwarteter Einstieg, KEIN Chain-Abriss.
            return Self::spatial_anchor(fresh, "initial", "reacquire", false, 0, "no_segment");
        };

        // â”€â”€ Distanz-basierter Advance-Trigger (Fix 2): erst weiterschalten, wenn die
        //    RESTBOGENLÃ„NGE des aktuellen Segments unter NEAREST_ADVANCE_DIST_M liegt â€” statt
        //    bei fixer t-Fraktion. Auf langen Segmenten feuerte `t â‰¥ 0.85` Dutzende Meter vor
        //    dem Knoten; der kurze Nachfolger lag dann auÃŸerhalb NEAREST_QUERY_RADIUS_M (50 m)
        //    â†’ off_segment â†’ chain_broken â†’ Re-Acquire â†’ 2-Zyklus-Flackern. Die zusÃ¤tzliche
        //    `ct â‰¥ NEAREST_FWD_PROGRESS_T`-Bedingung hÃ¤lt das alte Verhalten auf kurzen
        //    Segmenten (das t-Gate bindet dort, nicht das Distanz-Gate) und schaltet damit nie
        //    frÃ¼her als bisher â€” nur Segmente lÃ¤nger als ~233 m schalten jetzt SPÃ„TER (nÃ¤her
        //    am Knoten), sodass der Nachfolger beim Schalten garantiert in Reichweite liegt.
        //    Chord-RestlÃ¤nge `length_mÂ·(1âˆ’t)` (length_m = Chord) genÃ¼gt; die ~15-m-Marge zum
        //    50-m-Radius deckt die KrÃ¼mmungs-UnterschÃ¤tzung ab.
        let remaining_arc = (index.segments[ci].length_m * (1.0 - ct)).max(0.0);
        let want_advance = ct >= NEAREST_FWD_PROGRESS_T && remaining_arc < NEAREST_ADVANCE_DIST_M;

        // â”€â”€ (1) Noch nicht am Segmentende? â†’ sticky bleiben, KEINE Spatial-Suche.
        //        Genau das killt den 180Â°-Sprung: mitten im Segment wird NIE neu rÃ¤umlich
        //        gesucht, egal wie nah die Gegenspur liegt.
        if !want_advance && cd <= NEAREST_QUERY_RADIUS_M && chd <= NEAREST_HEADING_TOL_DEG {
            return NearestSelection::Steer {
                seg_idx: ci,
                t: ct,
                heading_diff_deg: chd,
                switch_reason: "sticky",
                chain_reason: "sticky",
                successor_count: 0,
                reverse_skipped: 0,
                chain_broken: false,
            };
        }

        // â”€â”€ (2a) Noch nicht am Ende, aber abgewichen (Drift > 50 m ODER Heading > 60Â°)
        //         â†’ Chain gebrochen â†’ Spatial-Re-Acquisition (60Â°-Filter, nie Gegenspur).
        if !want_advance {
            let fallback_cause = if cd > NEAREST_QUERY_RADIUS_M {
                "no_segment"
            } else {
                "heading_filter_exceeded"
            };
            return Self::spatial_anchor(fresh, "initial", "off_segment", true, 0, fallback_cause);
        }

        // â”€â”€ (2b) Nahe am Segmentende (Restbogen < NEAREST_ADVANCE_DIST_M) â†’ forward_adj.
        self.advance_forward_adj(index, ci, query, fresh, &self.cached_route_seg_set)
    }

    /// Segmentende (Restbogen < `NEAREST_ADVANCE_DIST_M`): Ã¼ber `forward_adj[current.to_uid]`
    /// weiterschalten.
    ///
    /// Zwei WÃ¤chter gegen den VorwÃ¤rts/RÃ¼ckwÃ¤rts-2-Zyklus:
    /// - **(Task 1) UID-Reverse-Skip:** das exakte Reverse-Geschwister (from/to vertauscht)
    ///   wird rein topologisch Ã¼bersprungen â€” immun gegen fehlerhafte Reverse-Tangenten
    ///   (die map-parser-seitig m0/m1 vertauscht statt negiert ablegt â†’ t=0-Tangente zeigt
    ///   vorwÃ¤rts und tÃ¤uscht JEDEN tangentenbasierten Filter; nur der UID-Vergleich greift).
    /// - **(Task 2) Exit-Tangenten-Filter:** statt gegen das fragile Truck-Heading wird der
    ///   Richtungs-Knick gegen die VorgÃ¤nger-Exit-Tangente geprÃ¼ft (dieselbe Wahrheitsquelle
    ///   wie `lookahead()`): `kink = acos(dot(exit@t=1, entry@t=0))`, behalten wenn
    ///   `kink < 90Â°`. Truck-heading-unabhÃ¤ngig (behebt Low-Speed-/Crash-FragilitÃ¤t).
    ///
    /// Mehrere gÃ¼ltige Nachfolger â†’ geradeaus-ster (grÃ¶ÃŸter Dot = kleinster Knick). Kein
    /// gÃ¼ltiger Nachfolger â†’ Chain-Abriss â†’ Spatial-Re-Acquisition (60Â°-Filter).
    fn advance_forward_adj(
        &self,
        index: &SplineIndex,
        ci: usize,
        query: Vec3,
        fresh: Option<&HeadingFilteredHit>,
        route_seg_set: &std::collections::HashSet<usize>,
    ) -> NearestSelection {
        let cur_seg = &index.segments[ci];
        let cur_from = cur_seg.from_uid;
        let cur_to = cur_seg.to_uid;
        // VorgÃ¤nger-Exit-Tangente (t=1) als Richtungs-Referenz (wie lookahead()).
        let exit_tan = evaluate_tangent(cur_seg, 1.0).normalize();

        let mut best: Option<(usize, f32)> = None; // (seg_idx, kink_deg)
        let mut count = 0usize;
        let mut reverse_skipped = 0usize;
        if let Some(list) = self.nearest_forward_adj.get(&cur_to) {
            for &si in list {
                if si == ci {
                    continue; // Selbst-Referenz
                }
                let Some(seg) = index.segments.get(si) else {
                    continue;
                };
                // (Task 1) Exaktes Reverse-Geschwister (from/to vertauscht) hart Ã¼berspringen.
                if seg.from_uid == cur_to && seg.to_uid == cur_from {
                    reverse_skipped += 1;
                    continue;
                }
                // (Task 2) Richtungs-Knick gegen die VorgÃ¤nger-Exit-Tangente (statt Truck-Heading).
                let entry_tan = evaluate_tangent(seg, 0.0).normalize();
                let dot =
                    exit_tan.x * entry_tan.x + exit_tan.y * entry_tan.y + exit_tan.z * entry_tan.z;
                let kink_deg = dot.clamp(-1.0, 1.0).acos().to_degrees();
                if kink_deg >= CHAIN_SUCCESSOR_TOL_DEG {
                    continue; // â‰¥ 90Â° Knick (inkl. RÃ¼ckwÃ¤rts â‰ˆ180Â°) â†’ kein VorwÃ¤rts-Nachfolger
                }
                // Route-Guard: bei aktiver Route nur Route-Nachfolger zulassen.
                // Leer = keine Route â†’ Guard inaktiv, altes Kink-Verhalten bleibt.
                if !route_seg_set.is_empty() && !route_seg_set.contains(&si) {
                    continue;
                }
                count += 1;
                // Geradeaus-ster = kleinster Knick.
                if best.is_none_or(|(_, bkink)| kink_deg < bkink) {
                    best = Some((si, kink_deg));
                }
            }
        }

        match best {
            Some((si, kink_deg)) => {
                let (t, _d) = index.project_on_segment(si, query).unwrap_or((0.0, 0.0));
                let switch_reason = if count > 1 {
                    "junction_pick"
                } else {
                    "fwd_progress"
                };
                NearestSelection::Steer {
                    seg_idx: si,
                    t,
                    heading_diff_deg: kink_deg,
                    switch_reason,
                    chain_reason: "segment_end",
                    successor_count: count,
                    reverse_skipped,
                    chain_broken: false,
                }
            }
            // Chain-Abriss: kein gÃ¼ltiger VorwÃ¤rts-Nachfolger â†’ Spatial-Re-Acquisition (60Â°).
            // Findet die Query den geraden Pfad wieder (Dead-End: meist das aktuelle Segment),
            // rollt der Truck sauber aus; sonst Null(no_segment) â†’ None-Steering.
            None => Self::spatial_anchor(
                fresh,
                "initial",
                "reacquire",
                true,
                reverse_skipped,
                "no_segment",
            ),
        }
    }

    /// Diag-only (Engage-Debug): publiziert die Einzelbedingungen des NearestSpline-
    /// Engage-Gates ins Blackboard, damit der Blockier-Grund live sichtbar ist:
    ///
    /// * `lane_keeper.engage_seg_found`        â€” Spatial-Query (50 m, â‰¤60Â°) hat getroffen
    /// * `lane_keeper.engage_dist_m`           â€” Distanz zum nÃ¤chsten Segment (âˆ’1 = keins)
    /// * `lane_keeper.engage_heading_diff_deg` â€” Heading-Diff Truck vs Segment-Tangente (âˆ’1 = keins)
    /// * `lane_keeper.engage_block_reason`     â€” "ok" | "too_far" | "heading" | "no_segment"
    ///
    /// Der 60Â°-Filter in `within_radius_filtered_heading` verwirft HART â€” ein leeres
    /// Ergebnis ist von auÃŸen nicht von "kein Segment im Radius" unterscheidbar. Darum
    /// schaut der Fallback ungefiltert auf das nÃ¤chste Segment: liegt eines im Radius,
    /// war der Heading-Filter der Blocker ("heading"), sonst "no_segment".
    /// Reine Anzeige â€” die `engage_allowed`-Logik bleibt unverÃ¤ndert.
    fn publish_engage_gate_diag(
        &self,
        index: &SplineIndex,
        query: Vec3,
        truck_heading_deg: f32,
        fresh_raw: Option<&HeadingFilteredHit>,
        engage_allowed: bool,
        ctx: &PluginContext,
    ) {
        ctx.blackboard.set(
            "lane_keeper.engage_seg_found",
            if fresh_raw.is_some() { "true" } else { "false" },
        );
        match fresh_raw {
            Some(h) => {
                ctx.blackboard
                    .set("lane_keeper.engage_dist_m", format!("{:.2}", h.dist_m));
                ctx.blackboard.set(
                    "lane_keeper.engage_heading_diff_deg",
                    format!("{:.1}", h.heading_diff_rad.to_degrees()),
                );
                // Reihenfolge der Bedingungen ist entscheidend:
                // too_far (>45m) hat Vorrang vor lateral_too_far (3-45m),
                // damit lateral_too_far nicht durch too_far maskiert wird.
                ctx.blackboard.set(
                    "lane_keeper.engage_block_reason",
                    if engage_allowed {
                        "ok"
                    } else if h.dist_m >= NEAREST_ENGAGE_DIST_M {
                        "too_far"
                    } else {
                        "lateral_too_far"
                    },
                );
            }
            None => match index.nearest_with_projection(query, ROUTE_NEAREST_CANDIDATES) {
                Some(hit) if hit.dist_m <= NEAREST_QUERY_RADIUS_M => {
                    let d = (truck_heading_deg - hit.heading_deg).rem_euclid(360.0);
                    let diff = if d > 180.0 { 360.0 - d } else { d };
                    ctx.blackboard
                        .set("lane_keeper.engage_dist_m", format!("{:.2}", hit.dist_m));
                    ctx.blackboard.set(
                        "lane_keeper.engage_heading_diff_deg",
                        format!("{:.1}", diff),
                    );
                    ctx.blackboard
                        .set("lane_keeper.engage_block_reason", "heading");
                }
                _ => {
                    ctx.blackboard.set("lane_keeper.engage_dist_m", "-1.00");
                    ctx.blackboard
                        .set("lane_keeper.engage_heading_diff_deg", "-1.0");
                    ctx.blackboard
                        .set("lane_keeper.engage_block_reason", "no_segment");
                }
            },
        }
    }

    fn tick_request_nearest_spline(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        // Status-Key (Task 4): in welchem Modus der Lane-Keeper tatsÃ¤chlich lÃ¤uft.
        ctx.blackboard.set("lane_keeper.mode", "nearest_spline");
        self.apply_gain_overrides(ctx);
        let is_active = ctx.is_active();

        // Index vorhanden? Ohne Index kein routerloses Folgen mÃ¶glich.
        let Some(index) = self.index.clone() else {
            ctx.blackboard.set("lane_keeper.engage_allowed", "false");
            ctx.blackboard
                .set("lane_keeper.null_steer_cause", "no_segment");
            ctx.blackboard
                .set("lane_keeper.nearest_kreuzung_unresolved", "false");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "no_spline_index");
            // Diag-Keys konsistent halten: ohne Index kann kein Segment gefunden werden.
            ctx.blackboard.set("lane_keeper.engage_seg_found", "false");
            ctx.blackboard.set("lane_keeper.engage_dist_m", "-1.00");
            ctx.blackboard
                .set("lane_keeper.engage_heading_diff_deg", "-1.0");
            ctx.blackboard
                .set("lane_keeper.engage_block_reason", "no_spline_index");
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.xtrack_integ = 0.0;
            return None;
        };
        self.ensure_nearest_index_built(&index);

        let Some(t) = telemetry else {
            ctx.blackboard.set("lane_keeper.engage_allowed", "false");
            ctx.blackboard
                .set("lane_keeper.null_steer_cause", "no_segment");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "no_telemetry");
            // Diag-Keys konsistent halten: ohne Telemetrie keine Position â†’ kein Query.
            ctx.blackboard.set("lane_keeper.engage_seg_found", "false");
            ctx.blackboard.set("lane_keeper.engage_dist_m", "-1.00");
            ctx.blackboard
                .set("lane_keeper.engage_heading_diff_deg", "-1.0");
            ctx.blackboard
                .set("lane_keeper.engage_block_reason", "no_telemetry");
            return None;
        };

        let tx = t.position[0];
        let tz = t.position[2];
        let query = Vec3::new(tx as f32, t.position[1] as f32, tz as f32);
        // ETS2-Heading 0..1 CCW von Nord â†’ CW-Grad (0=N, 90=O), wie SplineIndex.
        let truck_heading_deg = ((-t.heading) * 360.0).rem_euclid(360.0) as f32;
        let truck_heading_rad = truck_heading_deg.to_radians();

        // â”€â”€ Frischer heading-gefilterter Nearest-Query (â‰¤60Â°) â€” Basis fÃ¼r engage_allowed
        //    UND Stufe 1/2 der Auswahl. within_radius_filtered_heading verwirft >60Â°
        //    HART (kein Fallback auf Gegenrichtung).
        let cands = index.within_radius_filtered_heading(
            query,
            NEAREST_QUERY_RADIUS_M,
            truck_heading_rad,
            NEAREST_HEADING_TOL_DEG.to_radians(),
            |_idx, _meta| true,
        );
        // engage_allowed: nÃ¤chstes (RAW-Distanz) heading-kompatibles Segment
        // < NEAREST_ENGAGE_DIST_M (45 m) UND laterale Ablage <= engage_max_lateral_m (3 m default).
        // Beide Gates mÃ¼ssen erfÃ¼llt sein (additiv).
        let fresh_raw = cands.first();
        let engage_allowed = fresh_raw.is_some_and(|h| {
            h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= self.engage_max_lateral_m
        });
        ctx.blackboard.set(
            "lane_keeper.engage_allowed",
            if engage_allowed { "true" } else { "false" },
        );
        // Diag-only: Einzelbedingungen des Engage-Gates sichtbar machen (auch Pre-Engage,
        // da dieser Block VOR dem is_active-Early-Return lÃ¤uft).
        self.publish_engage_gate_diag(
            &index,
            query,
            truck_heading_deg,
            fresh_raw,
            engage_allowed,
            ctx,
        );
        if let Some(h) = fresh_raw {
            ctx.blackboard.set(
                "lane_keeper.nearest_seg_heading_diff_deg",
                format!("{:.1}", h.heading_diff_rad.to_degrees()),
            );
            ctx.blackboard
                .set("lane_keeper.nearest_seg_dist_m", format!("{:.2}", h.dist_m));
        } else {
            ctx.blackboard
                .set("lane_keeper.nearest_seg_heading_diff_deg", "180.0");
            ctx.blackboard
                .set("lane_keeper.nearest_seg_dist_m", "-1.00");
        }
        // Prefab-biased bester Kandidat (Auswahl-Eingang).
        let fresh_biased = cands.iter().min_by(|a, b| {
            effective_select_dist(a)
                .partial_cmp(&effective_select_dist(b))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // â”€â”€ Pre-Engage (Off/Engaging): nur engage_allowed publizieren, NICHT lenken.
        if !is_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.nearest_seg_idx = None;
            self.last_road_lane_offset_m = None;
            self.xtrack_integ = 0.0;
            // Disengage: Capture/Stuck-Zustand zurÃ¼cksetzen + Capture-Tempoziel rÃ¤umen
            // (sonst hielte der Speed-Controller das 20-km/h-Limit nach dem Disengage).
            self.stuck_ticks = 0;
            self.stuck_recovery = false;
            self.stuck_disengage_ticks = 0;
            self.capture_active = false;
            self.capture_exit_ticks = 0;
            ctx.blackboard.set("lane_keeper.stuck_recovery", "false");
            ctx.blackboard.set("lane_keeper.capture_active", "false");
            ctx.blackboard
                .set("lane_keeper.capture_speed_target_kmh", "-1.0");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "state_not_active");
            ctx.blackboard.set("lane_keeper.null_steer_cause", "none");
            ctx.blackboard
                .set("lane_keeper.nearest_kreuzung_unresolved", "false");
            ctx.blackboard
                .set("lane_keeper.nearest_seg_switch_reason", "none");
            // Chain im Pre-Engage zurÃ¼cksetzen (kein laufender Chain-Lauf).
            ctx.blackboard.set("lane_keeper.chain_segment_idx", "-1");
            ctx.blackboard
                .set("lane_keeper.chain_advance_reason", "none");
            ctx.blackboard.set("lane_keeper.chain_successor_count", "0");
            ctx.blackboard.set("lane_keeper.chain_reverse_skipped", "0");
            ctx.blackboard.set("lane_keeper.chain_broken", "false");
            // Junction-Snap-Diag: im Pre-Engage-Zustand kein Nearest-Lauf â†’ Reset-Werte,
            // route_set_size aber aktuell (Route-Set wird unabhaengig vom State gepflegt).
            ctx.blackboard.set("lane_keeper.sel_path", "none");
            ctx.blackboard.set(
                "lane_keeper.route_set_size",
                self.cached_route_seg_set.len().to_string(),
            );
            ctx.blackboard.set("lane_keeper.route_set_has_nearest", "false");
            ctx.blackboard.set("lane_keeper.route_set_has_lookahead", "false");
            return None;
        }

        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.nearest_seg_idx = None;
            self.last_road_lane_offset_m = None;
            self.xtrack_integ = 0.0;
            self.stuck_ticks = 0;
            self.stuck_recovery = false;
            self.stuck_disengage_ticks = 0;
            self.capture_active = false;
            self.capture_exit_ticks = 0;
            ctx.blackboard.set("lane_keeper.stuck_recovery", "false");
            ctx.blackboard.set("lane_keeper.capture_active", "false");
            ctx.blackboard
                .set("lane_keeper.capture_speed_target_kmh", "-1.0");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            ctx.blackboard.set("lane_keeper.null_steer_cause", "none");
            ctx.blackboard.set("lane_keeper.sel_path", "none");
            ctx.blackboard.set(
                "lane_keeper.route_set_size",
                self.cached_route_seg_set.len().to_string(),
            );
            ctx.blackboard.set("lane_keeper.route_set_has_nearest", "false");
            ctx.blackboard.set("lane_keeper.route_set_has_lookahead", "false");
            return None;
        }

        // â”€â”€ Active: 5-stufige Auswahl â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        let sel = self.select_nearest_segment(&index, query, truck_heading_deg, fresh_biased);

        let (
            seg_idx,
            t_cur,
            heading_diff_deg,
            switch_reason,
            chain_reason,
            successor_count,
            reverse_skipped,
            chain_broken,
        ) = match sel {
            NearestSelection::Null { cause } => {
                // KEIN Steering-Request â†’ Arbitration nutzt legacy.steering = 0.0
                // (Lenkrad zentriert, Truck rollt geradeaus). KEIN Brake. Auch die Spatial-
                // Re-Acquisition fand nichts â†’ Chain gebrochen â†’ Tracking zurÃ¼cksetzen, der
                // nÃ¤chste Tick startet die Chain neu per Initial-Spatial-Query.
                self.pid.reset();
                self.previous_steering_out = 0.0;
                self.nearest_seg_idx = None;
                self.last_road_lane_offset_m = None;
                self.xtrack_integ = 0.0;
                // Re-Acquisition: Capture/Stuck-Zustand mit zurÃ¼cksetzen (Task 2/3) â€”
                // ohne Steering-Request ist das Lenkrad zentriert, kein Verkeil-Risiko;
                // Capture re-evaluiert sich beim nÃ¤chsten Steering-Tick aus e_lat/heading.
                self.stuck_ticks = 0;
                self.stuck_recovery = false;
                self.stuck_disengage_ticks = 0;
                self.capture_active = false;
                self.capture_exit_ticks = 0;
                ctx.blackboard.set("lane_keeper.stuck_recovery", "false");
                ctx.blackboard.set("lane_keeper.capture_active", "false");
                ctx.blackboard
                    .set("lane_keeper.capture_speed_target_kmh", "-1.0");
                ctx.blackboard.set("lane_keeper.active", "true");
                ctx.blackboard.set("lane_keeper.returned_none", "true");
                ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
                ctx.blackboard.set("lane_keeper.null_steer_cause", cause);
                ctx.blackboard
                    .set("lane_keeper.nearest_seg_switch_reason", "none");
                ctx.blackboard
                    .set("lane_keeper.nearest_kreuzung_unresolved", "false");
                ctx.blackboard.set("lane_keeper.chain_segment_idx", "-1");
                ctx.blackboard
                    .set("lane_keeper.chain_advance_reason", "reacquire");
                ctx.blackboard.set("lane_keeper.chain_successor_count", "0");
                ctx.blackboard.set("lane_keeper.chain_reverse_skipped", "0");
                ctx.blackboard.set("lane_keeper.chain_broken", "true");
                ctx.blackboard.set("lane_keeper.sel_path", "null_steer");
                ctx.blackboard.set(
                    "lane_keeper.route_set_size",
                    self.cached_route_seg_set.len().to_string(),
                );
                ctx.blackboard.set("lane_keeper.route_set_has_nearest", "false");
                ctx.blackboard.set("lane_keeper.route_set_has_lookahead", "false");
                return None;
            }
            NearestSelection::Steer {
                seg_idx,
                t,
                heading_diff_deg,
                switch_reason,
                chain_reason,
                successor_count,
                reverse_skipped,
                chain_broken,
            } => (
                seg_idx,
                t,
                heading_diff_deg,
                switch_reason,
                chain_reason,
                successor_count,
                reverse_skipped,
                chain_broken,
            ),
        };

        self.nearest_seg_idx = Some(seg_idx);
        ctx.blackboard
            .set("lane_keeper.nearest_seg_idx", seg_idx.to_string());
        ctx.blackboard.set(
            "lane_keeper.nearest_seg_heading_diff_deg",
            format!("{heading_diff_deg:.1}"),
        );
        ctx.blackboard
            .set("lane_keeper.nearest_seg_switch_reason", switch_reason);
        // nearest_kreuzung_unresolved (Legacy): true sobald die Topologie-Chain nicht
        // weiterlief und auf die Spatial-Query zurÃ¼ckgefallen werden musste.
        ctx.blackboard.set(
            "lane_keeper.nearest_kreuzung_unresolved",
            chain_broken.to_string(),
        );
        // â”€â”€ Hypothesen-Diag (Junction-Snap-Debug) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        // Welcher Selektionspfad hat nearest_seg_idx gesetzt?
        let sel_path = match (chain_reason, chain_broken) {
            ("sticky", _) => "sticky",
            ("segment_end", _) => "advance_forward_adj",
            ("off_segment", _) => "spatial_off_segment",
            ("reacquire", false) => "spatial_initial",
            ("reacquire", true) => "spatial_chain_broken",
            _ => "unknown",
        };
        ctx.blackboard.set("lane_keeper.sel_path", sel_path);
        ctx.blackboard.set(
            "lane_keeper.route_set_size",
            self.cached_route_seg_set.len().to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.route_set_has_nearest",
            self.cached_route_seg_set.contains(&seg_idx).to_string(),
        );
        // â”€â”€ Chain-Diagnose (Task 2/3) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        ctx.blackboard
            .set("lane_keeper.chain_segment_idx", seg_idx.to_string());
        ctx.blackboard
            .set("lane_keeper.chain_advance_reason", chain_reason);
        // chain_successor_count = gÃ¼ltige VorwÃ¤rts-Nachfolger NACH dem Reverse-Skip.
        ctx.blackboard.set(
            "lane_keeper.chain_successor_count",
            successor_count.to_string(),
        );
        // chain_reverse_skipped = am Ãœbergang Ã¼bersprungene exakte Reverse-Geschwister.
        ctx.blackboard.set(
            "lane_keeper.chain_reverse_skipped",
            reverse_skipped.to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.chain_broken", chain_broken.to_string());
        // â”€â”€ Distanz-Diagnose (Fix-2-Sichtbarkeit): t auf dem aktuellen Chain-Segment und
        //    verbleibende (Chord-)BogenlÃ¤nge bis zum Endknoten. Macht den distanz-basierten
        //    Advance sichtbar (bei welcher RestlÃ¤nge geschaltet wird) und ergÃ¤nzt die
        //    bestehenden chain_*/xtrack_*-Keys. lane_offset_applied_m (unten) zeigt zusÃ¤tzlich
        //    den tatsÃ¤chlich verwendeten Offset inkl. Vererbung.
        let chain_remaining_m = (index.segments[seg_idx].length_m * (1.0 - t_cur)).max(0.0);
        ctx.blackboard
            .set("lane_keeper.chain_seg_t", format!("{t_cur:.3}"));
        ctx.blackboard.set(
            "lane_keeper.chain_remaining_m",
            format!("{chain_remaining_m:.2}"),
        );

        // â”€â”€ Look-Ahead-Punkt: Cross-Segment-Walk (forward_adj, Junction-Heading-Pick).
        let dt = ctx.dt_s.min(0.1);
        let speed_kmh = t.speed_ms * 3.6;
        // Lookahead-Floor (nur NearestSpline): bei Kriechtempo nicht unter
        // NEAREST_MIN_LOOK_AHEAD fallen, sonst oszilliert die Lenkung. Oberhalb
        // ~14 km/h ist die geschwindigkeits-Komponente eh grÃ¶ÃŸer â†’ kein Effekt.
        let look_ahead = nearest_look_ahead_m(speed_kmh);
        let la = lookahead(
            seg_idx,
            t_cur,
            look_ahead,
            &self.nearest_forward_adj,
            index.segments.as_slice(),
            &self.nearest_luts,
        );
        let Some(la) = la else {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.xtrack_integ = 0.0;
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard.set("lane_keeper.returned_none", "true");
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard
                .set("lane_keeper.null_steer_cause", "no_segment");
            return None;
        };

        // â”€â”€ v3 Cross-Track-Steuerung (Stanley, Lookahead im Nenner; NUR NearestSpline) â”€â”€
        // heading_err = reine Heading-Ausrichtung (Truck vs. Pfad-Tangente am Look-Ahead).
        // Der Lane-Offset steckt NICHT mehr im Lenk-Zielpunkt (das erzeugte die
        // atan(offset/look)-SÃ¤ttigung â†’ Stall-Spirale), sondern wird vom GEBUNDENEN
        // Cross-Track-Term getragen. Vorzeichen verifiziert (Task 1): Right-Normal
        // n=(-tan.z,tan.x) â†’ e_lat>0 = Truck RECHTS der Soll-Linie (wie lib.rs:817/1435);
        // positive Lenkung = rechts (lib.rs:3155/3768) â†’ âˆ’xtrack lenkt LINKS zurÃ¼ck.
        let cal = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.lane_offset_cal_m")
            .unwrap_or(0.0) as f32;

        // (a) Heading-Term: Pfad-Tangente am Look-Ahead-Punkt (Preview) gegen Truck-Heading.
        let final_seg = &index.segments[la.seg_idx];
        let tan_la = evaluate_tangent(final_seg, la.t);
        let len_la = (tan_la.x * tan_la.x + tan_la.z * tan_la.z).sqrt();
        if len_la < 1e-6 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.xtrack_integ = 0.0;
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard.set("lane_keeper.returned_none", "true");
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard
                .set("lane_keeper.null_steer_cause", "no_segment");
            return None;
        }
        let path_heading = f64::atan2(tan_la.x as f64, -(tan_la.z as f64));
        let truck_heading_rad =
            (-t.heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let mut heading_err = path_heading - truck_heading_rad;
        while heading_err > std::f64::consts::PI {
            heading_err -= 2.0 * std::f64::consts::PI;
        }
        while heading_err < -std::f64::consts::PI {
            heading_err += 2.0 * std::f64::consts::PI;
        }

        // (b) Cross-Track: lateraler Abstand Truck â†’ Soll-Linie (Centerline + lane_offset_right),
        //     gemessen am Truck-Projektionspunkt auf dem AKTUELLEN Segment (seg_idx, t_cur).
        let seg_cur = &index.segments[seg_idx];
        let meta_cur = index.metadata[seg_idx];
        let is_prefab = meta_cur.is_some_and(|m| m.is_prefab);
        // â”€â”€ Offset-Vererbung (Fix 1): metadatenlose `direction="prefab"`-LÃ¼ckensegmente
        //    (Kreuzungs-interne Road-Geometrie, meta=None) fielen bisher auf
        //    LANE_OFFSET_RIGHT_M (1.875 m = 1-spurige Spurmitte) zurÃ¼ck. Zwischen 2-/3-spurigen
        //    Roads (5.625/9.375 m) sprang die Soll-Linie dadurch bis 7.5 m â†’ e_lat-Sprung beim
        //    Ãœbergang. Jetzt erbt eine meta=None-LÃ¼cke den zuletzt gÃ¼ltigen ECHTEN Road-Offset,
        //    sodass die Soll-Linie Ã¼ber die kurze LÃ¼cke kontinuierlich bleibt.
        //    WICHTIG: ECHTE Road-Segmente (`Some(m)`, auch mit ANDERER Spuranzahl) behalten
        //    IMMER ihren echten `lane_offset_right_m` â€” dort SOLL er sich Ã¤ndern. NavCurves
        //    (`Some(m)` mit `is_prefab`) bleiben bei 0.0 (Spline liegt bereits auf Spurmitte).
        //    Nur der `None`-Fall erbt; vor dem ersten Road-Segment greift der 1.875-m-Default.
        let raw_lane_offset: f32 = match meta_cur {
            Some(m) if m.is_prefab => 0.0,
            Some(m) => {
                self.last_road_lane_offset_m = Some(m.lane_offset_right_m);
                m.lane_offset_right_m
            }
            None => self
                .last_road_lane_offset_m
                .unwrap_or(LANE_OFFSET_RIGHT_M as f32),
        };
        let offset_inherited = meta_cur.is_none() && self.last_road_lane_offset_m.is_some();
        let lane_offset = raw_lane_offset + cal;
        let source = if is_prefab {
            "nearest_prefab"
        } else if offset_inherited {
            "nearest_inherited"
        } else {
            "nearest_road"
        };
        let pc = evaluate(seg_cur, t_cur);
        let tan_cur = evaluate_tangent(seg_cur, t_cur);
        let len_cur = (tan_cur.x * tan_cur.x + tan_cur.z * tan_cur.z).sqrt();
        let truck_lat_vs_centerline = if len_cur > 1e-6 {
            let rn_x = (-tan_cur.z / len_cur) as f64;
            let rn_z = (tan_cur.x / len_cur) as f64;
            (tx - pc.x as f64) * rn_x + (tz - pc.z as f64) * rn_z
        } else {
            0.0
        };
        let e_lat = truck_lat_vs_centerline - lane_offset as f64;

        // (c) Cross-Track-Pfad (PI, Option 3): EIGENER Integralpfad auf e_lat. Der v3-Modus
        //     summierte heading_err + xtrack in EINEN PID; dessen Integrator trieb die SUMME â†’ 0,
        //     also xtrack â†’ âˆ’heading_err und e_lat pinnte im Gleichgewicht bei â‰ˆlook/k (â‰ˆ2.7 m
        //     gemessen). Jetzt schlieÃŸt ein dedizierter e_lat-Integralpfad den Restfehler (e_lat â†’ 0),
        //     wÃ¤hrend der P-Anteil = v3 atan2(K_CTÂ·e_lat, max(look, BASE)) UNVERÃ„NDERT bleibt
        //     (KEIN v im Nenner â†’ SingularitÃ¤t strukturell eliminiert, kein Offset im Zielpunkt â†’
        //     keine atan(offset/look)-SÃ¤ttigung â†’ Stall-Sicherheit erhalten).
        let k_ct = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.nearest_xtrack_k")
            .unwrap_or(NEAREST_XTRACK_K_DEFAULT);
        let k_ct_i = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.nearest_xtrack_ki")
            .unwrap_or(NEAREST_XTRACK_KI_DEFAULT);
        let denom = (look_ahead as f64).max(BASE_LOOK_AHEAD);
        let xtrack_p = f64::atan2(k_ct * e_lat, denom); // rad â€” v3-P-Anteil, unverÃ¤ndert
                                                        // Anti-Windup: NUR innerhalb des Gates integrieren, sonst Speicher hart auf 0. So erzeugt
                                                        // eine groÃŸe Abweichung (Stillstand-Engage-Artefakt, Re-Acquisition-Sprung) keinen Windup.
        if e_lat.abs() < CT_INTEG_GATE_M {
            self.xtrack_integ = (self.xtrack_integ + e_lat * dt).clamp(-CT_INTEG_MAX, CT_INTEG_MAX);
        } else {
            self.xtrack_integ = 0.0;
        }
        let xtrack_i = k_ct_i * self.xtrack_integ; // rad â€” Integralbeitrag
                                                   // Gemeinsamer Clamp Ã¼ber P+I; Vorzeichen wie v3: e_lat>0 (rechts) â†’ negativ â†’ LINKS zurÃ¼ck.
        let xtrack_contribution = (-(xtrack_p + xtrack_i)).clamp(-XTRACK_MAX_RAD, XTRACK_MAX_RAD);

        // (d) Heading-Pfad: reines P, KEIN Integral (ein Integral hier wÃ¼rde e_lat Ã¼ber die Summe
        //     wieder pinnen). Gain = live `plugin.lane_keeper.kp` (RouteFollowing-getunt, Default
        //     0.8); der Rate-Limiter (0.1/Tick) dÃ¤mpft Spikes, daher ist kein D-Term nÃ¶tig.
        //     Hinweis Live-Tuning: heading_contribution (Lenk-Einheiten, kpÂ·rad) und
        //     xtrack_contribution (rad) werden direkt addiert â€” `kp` und `nearest_xtrack_k` regeln
        //     gemeinsam die Headingâ†”Cross-Track-Balance. Im NearestSpline-Modus primÃ¤r
        //     `nearest_xtrack_k`/`nearest_xtrack_ki` zum Tunen nutzen, `kp` mÃ¶glichst stehen lassen.
        let kp_h = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kp")
            .unwrap_or(DEFAULT_KP);
        let heading_contribution = kp_h * heading_err; // Lenk-Einheiten

        // Finaler Lenkwert = Heading-P + Cross-Track-(P+I). Die beiden AUSGÃ„NGE addieren; es gibt
        // KEINEN gemeinsamen Integrator mehr (self.pid integriert die Summe NICHT â€” der e_lat-Bug
        // ist damit strukturell weg). Bestehender Rate-Limiter (0.1/Tick) wie gehabt.
        let steer_wish = heading_contribution + xtrack_contribution;

        // â”€â”€ Capture-Modus (Task 3): solange der Truck die Soll-Linie noch nicht sauber
        // erreicht hat, sanft eindrehen (Steer-Cap 0.5) + langsam anfahren (20 km/h als
        // min-Eingang des Speed-Controllers, s.u.).
        // EINTRITT nur Ã¼ber die Lateral-Abweichung (Reviewer-K2): heading_err ist der
        // PREVIEW-Fehler gegen die Lookahead-Tangente und liegt in stationÃ¤rer Kurven-
        // fahrt auch AUF der Linie strukturell bei â‰ˆ look/R (>10Â° bei R â‰² 90 m) â€” ein
        // Heading-ODER-Eintritt wÃ¼rde in jeder Kurve Capture zÃ¼nden (20-km/h-Ziel â†’
        // Bergab-Override â†’ Vollbremsung; 0.5-Cap wÃ¼rde enge Junction-Kurven wÃ¼rgen).
        // EXIT verlangt BEIDES stabil (|e_lat| â‰¤ 1.5 m UND |heading_err| â‰¤ 10Â°) Ã¼ber
        // CAPTURE_EXIT_STABLE_TICKS â€” kein Exit solange der Truck noch schief steht,
        // kein Flackern an der Schwelle.
        if e_lat.abs() > CAPTURE_EXIT_ELAT_M {
            self.capture_active = true;
            self.capture_exit_ticks = 0;
        } else if self.capture_active {
            if heading_err.abs() <= CAPTURE_EXIT_HEADING_DEG.to_radians() {
                self.capture_exit_ticks += 1;
                if self.capture_exit_ticks >= CAPTURE_EXIT_STABLE_TICKS {
                    self.capture_active = false;
                    self.capture_exit_ticks = 0;
                }
            } else {
                self.capture_exit_ticks = 0;
            }
        }

        // â”€â”€ Stuck-Watchdog (Fix 2/3): vâ‰ˆ0 + groÃŸe Lenk-ABSICHT Ã¼ber ~1 s = verkeilt.
        // Recovery capt Steer auf Â±0.15 (Anfahren mÃ¶glich). Nach weiteren ~3 s ohne
        // Fortschritt â†’ Disengage.
        // throttle_cmd-Gate entfernt (Fix 3): nach einem Crash-Stop ist throttle=0,
        // der alte Gate verhinderte das ZÃ¤hlen genau im Crash-Fall (beobachtetes Symptom).
        if t.speed_ms > STUCK_RESET_SPEED_MS {
            self.stuck_ticks = 0;
            self.stuck_recovery = false;
            self.stuck_disengage_ticks = 0;
        } else if t.speed_ms < STUCK_SPEED_MS && steer_wish.abs() > STUCK_STEER_MIN {
            self.stuck_ticks = self.stuck_ticks.saturating_add(1);
            if self.stuck_ticks > STUCK_TICKS {
                self.stuck_recovery = true;
                self.stuck_disengage_ticks = self.stuck_disengage_ticks.saturating_add(1);
                if self.stuck_disengage_ticks > STUCK_DISENGAGE_TICKS {
                    ctx.blackboard.set("autopilot.disengage_requested", "true");
                    tracing::warn!(
                        "[lane-keeper] SAFETY DISENGAGE â€” stuck_watchdog ({} ticks)",
                        self.stuck_disengage_ticks
                    );
                }
            }
        }

        // â”€â”€ Steer-Caps (Task 1/2/3): engster Cap gewinnt. Low-Speed-Cap gegen das
        // Volleinschlag-Verkeilen (ab 3 m/s transparent = 1.0, normales Fahren
        // unberÃ¼hrt), Capture-Cap fÃ¼rs sanfte Eindrehen, Stuck-Cap zum LÃ¶sen.
        let mut steer_cap = low_speed_steer_cap(t.speed_ms);
        if self.capture_active {
            steer_cap = steer_cap.min(CAPTURE_STEER_CAP);
        }
        if self.stuck_recovery {
            steer_cap = steer_cap.min(STUCK_RELAX_CAP);
        }
        let steering_target = steer_wish.clamp(-steer_cap, steer_cap);
        let steering = self.rate_limit(steering_target, ctx);

        // Winkelfehler (rad) fÃ¼r die Heading-Stage-Statemachine â€” deren error_rad-Schwellen sind in
        // rad. Wie zuvor: Heading-Fehler + Cross-Track-Korrektur, NICHT der dimensionslose Lenkwert.
        let angular_err =
            (heading_err + xtrack_contribution).clamp(-std::f64::consts::PI, std::f64::consts::PI);

        // Diagnostik (Heading- vs. Cross-Track-Anteil getrennt sichtbar).
        let la_point = evaluate(final_seg, la.t);
        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard.set("lane_keeper.returned_none", "false");
        ctx.blackboard.set("lane_keeper.null_steer_cause", "none");
        ctx.blackboard.set("lane_keeper.lateral_source", source);
        ctx.blackboard
            .set("lane_keeper.heading_error_rad", format!("{angular_err:.6}"));
        ctx.blackboard
            .set("lane_keeper.error_rad", format!("{angular_err:.6}"));
        ctx.blackboard.set(
            "lane_keeper.heading_contribution_rad",
            format!("{heading_err:.6}"),
        );
        ctx.blackboard.set(
            "lane_keeper.xtrack_contribution_rad",
            format!("{xtrack_contribution:.6}"),
        );
        ctx.blackboard
            .set("lane_keeper.xtrack_e_lat_m", format!("{e_lat:.3}"));
        ctx.blackboard
            .set("lane_keeper.xtrack_k", format!("{k_ct:.3}"));
        // Cross-Track-PI-AufschlÃ¼sselung (Windup-Kontrolle + Live-Tuning). xtrack_p_rad und
        // xtrack_i_rad tragen bereits das Korrektur-Vorzeichen (âˆ’), summieren sich also (vor dem
        // gemeinsamen Clamp) zu xtrack_contribution_rad.
        ctx.blackboard.set(
            "lane_keeper.xtrack_integ",
            format!("{:.4}", self.xtrack_integ),
        );
        ctx.blackboard
            .set("lane_keeper.xtrack_p_rad", format!("{:.6}", -xtrack_p));
        ctx.blackboard
            .set("lane_keeper.xtrack_i_rad", format!("{:.6}", -xtrack_i));
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        self.publish_stage_steering_diag(ctx, "nearest_spline", "none", true);
        // Capture/Anti-Stall-Diagnose (Task 5).
        ctx.blackboard
            .set("lane_keeper.steer_cap_applied", format!("{steer_cap:.3}"));
        ctx.blackboard.set(
            "lane_keeper.stuck_recovery",
            if self.stuck_recovery { "true" } else { "false" },
        );
        ctx.blackboard.set(
            "lane_keeper.capture_active",
            if self.capture_active { "true" } else { "false" },
        );
        // Capture-Tempoziel: > 0 nur wÃ¤hrend Capture; -1.0 = inaktiv (Speed-Controller
        // ignoriert Werte <= 0 in compute_target_speed).
        ctx.blackboard.set(
            "lane_keeper.capture_speed_target_kmh",
            if self.capture_active {
                format!("{CAPTURE_SPEED_TARGET_KMH:.1}")
            } else {
                "-1.0".to_string()
            },
        );
        ctx.blackboard.set(
            "lane_keeper.lane_offset_applied_m",
            format!("{lane_offset:.3}"),
        );
        ctx.blackboard.set(
            "lane_keeper.nearest_seg_lanes_in_direction",
            match meta_cur {
                Some(m) if !m.is_prefab => m.lanes_in_direction.to_string(),
                _ => "0".to_string(),
            },
        );
        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard
            .set("lane_keeper.lookahead_final_seg_id", la.seg_idx.to_string());
        ctx.blackboard.set(
            "lane_keeper.route_set_has_lookahead",
            self.cached_route_seg_set.contains(&la.seg_idx).to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{path_heading:.6}"));
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{:.2}", la_point.x));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{:.2}", la_point.z));
        ctx.blackboard.set("lane_keeper.fallback_reason", "none");

        Some(ControlRequest {
            steering: Some(steering),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }
}

// â”€â”€ Vision-mode implementation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

impl LaneKeeperPlugin {
    fn tick_request_vision(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let is_active = ctx.is_active();
        let was_active = self.was_active;
        self.was_active = is_active;

        // Activeâ†’Off: reset steering state but keep fallback accumulating so
        // engage_allowed can be re-evaluated while disengaged.
        if !is_active && was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            ctx.blackboard.set("lane_keeper.active", "false");
        }

        // Offâ†’Active: fresh PID start; engagement_heading captured below on first tick.
        if is_active && !was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
        }

        self.apply_gain_overrides(ctx);

        // Engine gate â€” applies in both Active and Off states.
        let t = telemetry?;
        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.engage_allowed", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        // Read lane perception (no telemetry dependency â€” blackboard only).
        let center_offset = ctx.blackboard.get_f64("lane.center_offset").unwrap_or(0.0);
        let confidence = ctx.blackboard.get_f64("lane.confidence").unwrap_or(0.0);
        let left_vis = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
        let right_vis = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
        // NaN â†’ None (absent lane)
        let left_x = ctx
            .blackboard
            .get_f64("lane.left_x")
            .filter(|x| x.is_finite());
        let right_x = ctx
            .blackboard
            .get_f64("lane.right_x")
            .filter(|x| x.is_finite());

        // Update fallback cascade â€” runs always so engage_allowed reflects real
        // lane quality even while the autopilot is still in Off state.
        self.fallback.push_confidence(confidence);
        self.fallback.push_offset(center_offset);
        self.extrapolator.advance_tick();
        let avg_conf = self.fallback.rolling_avg_confidence();
        let level = self.fallback.update(avg_conf, left_vis, right_vis);

        // Publish engage_allowed always â€” breaks the Off-state chicken-and-egg.
        let engage_allowed = level <= 1;
        ctx.blackboard.set(
            "lane_keeper.engage_allowed",
            if engage_allowed { "true" } else { "false" },
        );
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());

        if !is_active {
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "computing_engage_allowed");
            return None;
        }

        // â”€â”€ Active-only path â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

        // Capture engagement heading on the first tick of each Active session.
        if self.engagement_heading.is_none() {
            self.engagement_heading = Some(t.heading);
            tracing::info!("[lane-keeper] vision engage heading={:.4}", t.heading);
        }

        // Block-2 guard: heading drift since engagement.
        let eng_heading = self.engagement_heading.unwrap_or(t.heading);
        let heading_drift = wrap_angle(t.heading - eng_heading).abs();
        if heading_drift > HEADING_MISMATCH_THRESHOLD_RAD {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_mismatch");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        // Heading stage gate.
        self.heading_stage = ctx.blackboard.get("state.heading_stage");
        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        if matches!(stage, "AutoReplan" | "Disengaging") {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_stage");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            let block = if stage == "Disengaging" {
                "disengaging"
            } else {
                "autoreplan"
            };
            self.publish_stage_steering_diag(ctx, stage, block, false);
            return None;
        }

        // Heading-hold transitions.
        if level == 3 && !self.heading_hold.active {
            self.heading_hold
                .enter(t.heading, self.tick_count, self.previous_steering_out);
            ctx.blackboard.set(
                "lane_keeper.level_3_entered_at_tick",
                self.fallback.level_entered_at_tick.to_string(),
            );
            tracing::warn!("[lane-keeper] entering L3 heading-hold h={:.4}", t.heading);
        } else if level != 3 && self.heading_hold.active {
            self.heading_hold.exit();
        }

        // Level-4 disengage.
        if level == 4 {
            let l4_start = *self.level_4_entered_at_tick.get_or_insert(self.tick_count);
            let ticks_since = self.tick_count.saturating_sub(l4_start);

            if ticks_since == 0 {
                let event = format!(
                    r#"{{"tick":{},"reason":"{}","confidence":{:.4},"blind_ticks":{}}}"#,
                    self.tick_count,
                    self.fallback.fallback_reason,
                    confidence,
                    self.fallback.blind_tick_count,
                );
                ctx.blackboard.set("lane_keeper.level_4_event_json", event);
                tracing::error!("[lane-keeper] LEVEL-4 DISENGAGE tick={}", self.tick_count);
            }

            let brake = if ticks_since < L4_BRAKE_TICKS {
                Some(0.30)
            } else {
                None
            };
            ctx.blackboard.set("lane_keeper.fallback_level", "4");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.steering_source", "none_l4");
            self.publish_vision_diagnostics(ctx, level, avg_conf, 0.0);

            return Some(ControlRequest {
                steering: None,
                brake,
                priority: PRIORITY_LEVEL4,
                ..Default::default()
            });
        }
        self.level_4_entered_at_tick = None;

        // Speed-adaptive gain.
        let speed_kmh = t.speed_ms * 3.6;
        let speed_gain = if speed_kmh < 60.0 {
            1.2
        } else if speed_kmh <= 100.0 {
            1.0
        } else {
            0.8
        };

        // Per-level steering computation.
        let (vision_error, gain_factor, steering_source) = match level {
            0 => {
                // Normal vision: error = -center_offset
                (-center_offset, 1.0 * speed_gain, "vision_l0")
            }
            1 => {
                // Single-lane extrapolation
                let (err, _) =
                    extrapolate_center(left_x, right_x, avg_conf, &mut self.extrapolator);
                (err, 0.6 * speed_gain, "extrapolation_l1")
            }
            2 => {
                // Confidence-drop: EMA-weighted, same error source as L0
                (-center_offset, 0.35 * speed_gain, "vision_l2")
            }
            3 => {
                // Heading-hold: separate PID path, return early
                let pid_ref = &mut self.pid;
                let steering_l3 = self.heading_hold.compute_steering(
                    t.heading,
                    self.tick_count,
                    &mut |err, dt_val| pid_ref.update(err, dt_val),
                    dt,
                );

                // SoftLaneKeep scaling
                let scaled = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
                    steering_l3 * 0.3
                } else {
                    steering_l3
                };

                let output = self.rate_limit(scaled, ctx);
                ctx.blackboard.set("lane_keeper.active", "true");
                ctx.blackboard.set("lane_keeper.fallback_level", "3");
                ctx.blackboard
                    .set("lane_keeper.heading_hold_active", "true");
                ctx.blackboard.set(
                    "lane_keeper.hold_heading_rad",
                    format!("{:.6}", self.heading_hold.hold_heading),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_drift_rad",
                    format!("{:.6}", self.heading_hold.heading_drift(t.heading)),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_hold_ticks",
                    self.heading_hold.ticks_active(self.tick_count).to_string(),
                );
                ctx.blackboard
                    .set("lane_keeper.steering_source", "heading_hold_l3");
                self.publish_vision_diagnostics(ctx, level, avg_conf, output);

                return Some(ControlRequest {
                    steering: Some(output),
                    priority: PRIORITY_NORMAL,
                    ..Default::default()
                });
            }
            _ => (-center_offset, 0.0, "unknown"),
        };

        // PID update (levels 0, 1, 2).
        let raw_pid = self.pid.update(vision_error, dt).clamp(-1.0, 1.0);
        let scaled = (raw_pid * gain_factor).clamp(-1.0, 1.0);

        // SoftLaneKeep scaling (levels 0, 1, 2).
        let effective = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            scaled * 0.3
        } else {
            scaled
        };

        // Rate limiter (Block-2).
        let output = self.rate_limit(effective, ctx);

        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());
        ctx.blackboard
            .set("lane_keeper.heading_hold_active", "false");
        ctx.blackboard
            .set("lane_keeper.steering_source", steering_source);
        self.publish_vision_diagnostics(ctx, level, avg_conf, output);

        Some(ControlRequest {
            steering: Some(output),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }

    fn publish_vision_diagnostics(
        &self,
        ctx: &PluginContext,
        level: u8,
        avg_conf: f64,
        steering: f64,
    ) {
        ctx.blackboard.set(
            "lane_keeper.fallback_reason",
            self.fallback.fallback_reason.clone(),
        );
        ctx.blackboard.set(
            "lane_keeper.confidence_trend",
            format!("{:.4}", self.fallback.compute_confidence_trend()),
        );
        ctx.blackboard.set(
            "lane_keeper.detection_stability",
            format!("{:.4}", self.fallback.compute_detection_stability()),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_ticks",
            self.fallback.blind_tick_count.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_duration_ms",
            format!("{:.0}", self.fallback.blind_tick_count as f64 * 20.0),
        );
        ctx.blackboard.set("lane_keeper.single_lane_side", {
            let lv = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
            let rv = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
            match (lv, rv) {
                (true, false) => "left_only",
                (false, true) => "right_only",
                (true, true) => "both",
                (false, false) => "none",
            }
        });
        ctx.blackboard.set(
            "lane_keeper.lane_width_estimate_px",
            self.extrapolator
                .lane_width_estimate()
                .map(|w| format!("{w:.4}"))
                .unwrap_or_else(|| "NaN".to_string()),
        );
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        let _ = (level, avg_conf); // included in other keys already
    }
}

// â”€â”€ Plugin trait impl â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

impl Plugin for LaneKeeperPlugin {
    fn name(&self) -> &str {
        "lane-keeper"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"},"subdivisions":{"type":"integer","minimum":1,"maximum":20}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        self.load_waypoints_from_blackboard(ctx);
        // Pre-populate engage_allowed=false so the state machine never sees an absent key.
        ctx.blackboard.set("lane_keeper.engage_allowed", "false");
        tracing::info!("[lane-keeper] loaded, engage_allowed=false (pre-populated)");

        self.engage_max_lateral_m = ctx
            .blackboard
            .get("lane_keeper.engage_max_lateral_m")
            .and_then(|v| v.parse::<f32>().ok())
            .unwrap_or(DEFAULT_ENGAGE_MAX_LATERAL_M);
        tracing::info!(
            "[lane-keeper] engage_max_lateral_m={:.1}m",
            self.engage_max_lateral_m
        );

        if let Some(shared) = &ctx.spline_index {
            self.index = Some(Arc::clone(shared));
            let road_n = ctx.spline_index_road_seg_count.min(shared.segments.len());
            self.road_seg_count = road_n;
            let mut map = HashMap::with_capacity(road_n);
            for i in 0..road_n {
                let s = &shared.segments[i];
                map.insert((s.from_uid, s.to_uid), i);
            }
            self.seg_by_from_to = map;
            // Fix C: NavCurve-Segmente (Index >= road_n) nach (from_uid,to_uid).
            // Vec, weil ein Knotenpaar mehrere NavCurves tragen kann (Lanes/
            // parallele Durchfahrten). Damit kann die On-Route-NavCurve an einer
            // Junction in cached_route_seg_set aufgenommen werden.
            let mut nc_map: HashMap<(u64, u64), Vec<usize>> = HashMap::new();
            for i in road_n..shared.segments.len() {
                let s = &shared.segments[i];
                nc_map.entry((s.from_uid, s.to_uid)).or_default().push(i);
            }
            let nc_pairs = nc_map.len();
            let nc_segs: usize = nc_map.values().map(|v| v.len()).sum();
            self.navcurve_by_from_to = nc_map;
            if let Some(rg) = &ctx.graph {
                self.router_graph = Some(Arc::clone(rg));
            }
            tracing::info!(
                "[lane-keeper] shared SplineIndex: {} road segs mapped, {} NavCurve segs in {} node-pairs (graph={})",
                self.seg_by_from_to.len(),
                nc_segs,
                nc_pairs,
                self.router_graph.is_some()
            );
            // Phase 2c/2d-Diagnose (read-only): Index-Status auch ohne stdout per
            // blackboard-query lesbar machen.
            ctx.blackboard
                .set("lane_keeper.spline_index_present", "true");
            ctx.blackboard.set(
                "lane_keeper.seg_by_from_to_count",
                self.seg_by_from_to.len().to_string(),
            );
            ctx.blackboard.set(
                "lane_keeper.navcurve_by_from_to_count",
                nc_segs.to_string(),
            );
            ctx.blackboard.set(
                "lane_keeper.router_graph_present",
                self.router_graph.is_some().to_string(),
            );
        } else {
            tracing::warn!(
                "[lane-keeper] no shared SplineIndex; route-following uses Catmull-Rom fallback"
            );
            // Phase 2c/2d-Diagnose (read-only): expliziter Negativ-Status.
            ctx.blackboard
                .set("lane_keeper.spline_index_present", "false");
            ctx.blackboard.set("lane_keeper.seg_by_from_to_count", "0");
            ctx.blackboard
                .set("lane_keeper.router_graph_present", "false");
        }
    }

    fn on_unload(&mut self) {
        tracing::info!("[lane-keeper] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        self.heading_stage = ctx.blackboard.get("state.heading_stage");

        if self.mode == LaneKeeperMode::RouteFollowing
            && ctx.blackboard.get("router.active").as_deref() == Some("true")
        {
            let current_hash = ctx
                .blackboard
                .get("router.waypoints")
                .map(|j| hash_str(&j))
                .unwrap_or(0);
            if self.waypoints.is_empty() || current_hash != self.last_waypoints_hash {
                self.load_waypoints_from_blackboard(ctx);
            }
        }
    }

    fn tick_request(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        self.tick_count += 1;
        let tick_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros())
            .unwrap_or(0);
        ctx.blackboard
            .set("lane_keeper.tick_seq", self.tick_count.to_string());
        ctx.blackboard
            .set("lane_keeper.last_tick_us", tick_us.to_string());
        ctx.blackboard
            .set("lane_keeper.tick_ctx_active", ctx.is_active().to_string());

        // Re-check mode every tick (cheap: one Blackboard read; `update_mode_from_blackboard`
        // only does work on an actual change). Per-tick statt alle 50 Ticks, damit der
        // automatische lane_only â†’ nearest_spline-Wechsel der State-Machine ohne ~50-Tick-
        // Latenz greift (sonst hÃ¤ngt Engaging am stale engage_allowed). RouteFollowing-/
        // Vision-Verhalten bleibt unverÃ¤ndert (Mode wechselt nicht â†’ kein Reset).
        self.update_mode_from_blackboard(ctx);

        match self.mode {
            LaneKeeperMode::RouteFollowing => self.tick_request_route_following(telemetry, ctx),
            LaneKeeperMode::Vision => self.tick_request_vision(telemetry, ctx),
            LaneKeeperMode::NearestSpline => self.tick_request_nearest_spline(telemetry, ctx),
            LaneKeeperMode::Off => None,
        }
    }
}

// â”€â”€ Route-following utilities â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

fn smooth_catmull_rom(pts: &[[f64; 2]], subdivisions: usize) -> Vec<[f64; 2]> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let n = pts.len();
    let mut result = Vec::with_capacity((n - 1) * (subdivisions + 1));

    for i in 1..n {
        let p0 = if i >= 2 { pts[i - 2] } else { pts[0] };
        let p1 = pts[i - 1];
        let p2 = pts[i];
        let p3 = if i + 1 < n { pts[i + 1] } else { pts[n - 1] };

        for j in 0..=subdivisions {
            if j == 0 && i > 1 {
                continue;
            }
            let t = j as f64 / subdivisions as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let x = 0.5
                * ((2.0 * p1[0])
                    + (-p0[0] + p2[0]) * t
                    + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                    + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3);
            let z = 0.5
                * ((2.0 * p1[1])
                    + (-p0[1] + p2[1]) * t
                    + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                    + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3);
            result.push([x, z]);
        }
    }
    result
}

fn hash_str(s: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    s.hash(&mut h);
    h.finish()
}

truckpilot_plugin_api::export_plugin!(LaneKeeperPlugin);

// â”€â”€ Tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    fn make_telemetry(speed_ms: f64, heading: f64) -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_gear: 0,
            engine_rpm: 1200.0,
            cruise_control_kmh: 80.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
        }
    }

    fn ctx_with_state(state: &str) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        PluginContext::new("lane-keeper", bb)
    }

    fn fresh_ctx() -> PluginContext {
        let bb = SharedBlackboard::new();
        PluginContext::new("lane-keeper", bb)
    }

    fn active_plugin_with_straight_path() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        }
    }

    // â”€â”€ Vision-mode helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    fn vision_bb(
        state: &str,
        center_offset: f64,
        confidence: f64,
        left: bool,
        right: bool,
    ) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane.center_offset", center_offset.to_string());
        bb.set("lane.confidence", confidence.to_string());
        bb.set("lane.left_visible", left.to_string());
        bb.set("lane.right_visible", right.to_string());
        bb.set("lane.left_x", "NaN");
        bb.set("lane.right_x", "NaN");
        PluginContext::new("lane-keeper", bb)
    }

    fn make_vision_plugin() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            mode: LaneKeeperMode::Vision,
            ..Default::default()
        }
    }

    // â”€â”€ Pre-existing route-following geometry tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    #[test]
    fn look_ahead_increases_with_speed() {
        let slow = BASE_LOOK_AHEAD + 0.0 * SPEED_FACTOR;
        let fast = BASE_LOOK_AHEAD + 80.0 * SPEED_FACTOR;
        assert!(fast > slow);
        assert!((slow - 5.0).abs() < 0.01);
        assert!((fast - 45.0).abs() < 0.01);
    }

    #[test]
    fn straight_north_zero_error() {
        let mut lk = active_plugin_with_straight_path();
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        // After offset: lookahead shifts East â†’ target slightly right of North â†’ err > 0.
        assert!(
            err > 0.0,
            "truck on centerline, target right â†’ positive error, got {err}"
        );
    }

    #[test]
    fn turn_right_positive_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        assert!(err > 0.0, "expected positive (right turn), got {err}");
    }

    #[test]
    fn catmull_rom_more_points_than_input() {
        let pts = vec![[0.0, 0.0], [50.0, 10.0], [100.0, 0.0]];
        let smoothed = smooth_catmull_rom(&pts, 4);
        assert!(smoothed.len() > pts.len());
    }

    #[test]
    fn test_state_gate_returns_none_when_off() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_state_gate_returns_none_when_engaging() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Engaging");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_active_steering_with_waypoints() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk
            .tick_request(Some(&t), &ctx)
            .expect("active must request");
        let s = req.steering.expect("active must request steering");
        assert!(s > 0.0, "expected positive steering, got {s}");
        assert_eq!(req.priority, PRIORITY_NORMAL);
    }

    #[test]
    fn test_straight_line_near_zero_steering() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        // After offset: truck on centerline â†’ small positive steering toward right lane.
        assert!(
            s > 0.0 && s < 0.2,
            "positive steering toward right lane expected, got {s}"
        );
    }

    #[test]
    fn heading_convention_north_is_zero() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(0.0, 0.0, 0.0, 13.88, &ctx);
        // After offset: lookahead shifts East â†’ err > 0 (turn right toward right lane).
        assert!(
            err > 0.0,
            "truck on centerline, target right â†’ positive error, got {err}"
        );
    }

    #[test]
    fn heading_convention_east_is_half_pi() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // ETS2 East = 0.75 (0.75 CCW turns from North = 270Â° CCW = 90Â° CW = East)
        let err = plugin.compute_heading_error(0.0, 0.0, 0.75, 13.88, &ctx);
        // After offset: truck heads East, right lane is South (+z) â†’ target shifts South
        // â†’ target angle > Ï€/2, heading_rad = Ï€/2 â†’ err > 0.
        assert!(
            err > 0.0,
            "truck on centerline heading East, target shifted South â†’ err > 0, got {err}"
        );
    }

    #[test]
    fn heading_convention_punkt_vor_rechts_kleiner_positiver_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [144.89, -156.22]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck heading ~20Â° CW from North in ETS2 [0..1] CCW format:
        // ets2 = 1 - 20Â°/360Â° = 0.9444; converts to 0.349 rad â‰ˆ 20Â°.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.9444, 13.88, &ctx);
        assert!(err > 0.0 && err < 0.6, "expected ~0.4 positive, got {err}");
    }

    #[test]
    fn ets2_raw_heading_gives_small_error_on_aligned_road() {
        // Regression guard: before fix, heading=0.9796 (ETS2 [0..1] for ~7.33Â° CW) was
        // subtracted directly from a radian target, producing a phantom âˆ’49Â° error
        // that immediately triggered AutoReplan (threshold 60Â°) and suppressed all steering.
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]], // North road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // ETS2 heading 0.9796 â‰ˆ 7.33Â° CW from North (truck nearly aligned with road).
        let err = plugin.compute_heading_error(0.0, 0.0, 0.9796, 0.0, &ctx);
        // Must be near-zero (â‰¤15Â° = 0.26 rad), NOT âˆ’49Â° (âˆ’0.852 rad).
        assert!(
            err.abs() < 0.26,
            "ETS2 heading 0.9796 should give ~7Â° error, got {:.4} rad ({:.1}Â°)",
            err,
            err.to_degrees(),
        );
    }

    #[test]
    fn test_heading_wraparound() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [-0.1, 100.0]],
            ..Default::default()
        };
        // ETS2 [0..1] for ~179.4Â° CW (nearly South) â‰ˆ 0.5016.
        // Converts to Ï€-0.01 rad after rem_euclid, matching original test intent.
        let heading = 0.5016_f64;
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, heading, 10.0, &ctx);
        assert!(err.abs() < 0.5, "wraparound produced {err}");
    }

    #[test]
    fn test_pid_reset_on_state_exit() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..10 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        let off = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &off).is_none());

        lk.waypoints = vec![[0.0, 0.0], [20.0, -100.0]];

        let mut fresh = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let active2 = ctx_with_state("Active");
        let fresh_req = fresh.tick_request(Some(&t), &active2).unwrap();
        let resumed_req = lk.tick_request(Some(&t), &active2).unwrap();
        assert!(
            (fresh_req.steering.unwrap() - resumed_req.steering.unwrap()).abs() < 1e-6,
            "post-reset response must match a fresh PID"
        );
    }

    #[test]
    fn waypoints_reload_on_route_change() {
        let mut plugin = LaneKeeperPlugin::default();

        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.active", "true");
        bb.set("router.waypoints", "[[0.0,0.0],[10.0,0.0],[20.0,0.0]]");
        let ctx = PluginContext::new("lane-keeper", bb.clone());

        plugin.tick(None, &mut ControlOutput::default(), &ctx);
        let first_len = plugin.waypoints.len();
        let first_hash = plugin.last_waypoints_hash;
        assert!(first_len > 0);
        assert!(first_hash != 0);

        bb.set(
            "router.waypoints",
            "[[100.0,100.0],[110.0,100.0],[120.0,100.0]]",
        );
        plugin.tick(None, &mut ControlOutput::default(), &ctx);

        assert_ne!(plugin.last_waypoints_hash, first_hash);
        assert!((plugin.waypoints[0][0] - 100.0).abs() < 1e-6);
        assert_eq!(plugin.progress_idx, 0);
    }

    #[test]
    fn test_dt_clamp_at_0_1() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(10.0);
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!((-1.0..=1.0).contains(&s), "output out of range: {s}");
    }

    #[test]
    fn route_end_returns_zero_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0]],
            progress_idx: 1,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(9.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(err, 0.0, "Route-End-Guard must return 0.0, got {err}");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("route_end"),
        );
    }

    #[test]
    fn progress_advances_when_truck_near_next_waypoint() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(6.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(plugin.progress_idx, 1);
    }

    #[test]
    fn lookahead_starts_from_truck_position() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, 0.0], [40.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("20.00")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_z").as_deref(),
            Some("0.00")
        );
    }

    #[test]
    fn lookahead_walks_through_multiple_waypoints() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [3.0, 0.0], [6.0, 0.0], [9.0, 0.0], [12.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.walk_iterations").as_deref(),
            Some("1")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("6.00")
        );
    }

    #[test]
    fn heading_mismatch_above_threshold_returns_brake() {
        // Phase 2h-Safety: err > 1.4 rad must now emit a safety brake request,
        // not return None silently (Blocker 1 fix).
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(
            req.is_some(),
            "heading_mismatch > 1.4 rad must emit a brake ControlRequest"
        );
        let req = req.unwrap();
        assert!(
            req.steering.is_none(),
            "steering must be None during heading_mismatch brake"
        );
        assert!(
            req.brake.is_some(),
            "brake must be Some during heading_mismatch brake"
        );
        assert!(
            req.brake.unwrap() >= SAFETY_BRAKE_MIN,
            "brake >= SAFETY_BRAKE_MIN"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("decelerating_heading_mismatch")
        );
    }

    #[test]
    fn heading_mismatch_at_threshold_is_allowed() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn steering_rate_limit_clamps_large_delta() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9);
    }

    #[test]
    fn steering_rate_limit_passes_small_delta() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() < STEERING_MAX_DELTA_PER_TICK);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.steering_rate_limited")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn previous_steering_resets_on_disengage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        let off = ctx_with_state("Off");
        lk.tick_request(Some(&t), &off);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn previous_steering_resets_on_heading_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        lk.waypoints = vec![[0.0, 0.0], [0.0, 100.0]];
        let _ = lk.tick_request(Some(&t), &active);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn heading_convention_north_regression_unaffected_by_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(req.is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn auto_replan_stage_returns_none() {
        // Phase 2h-Safety: AutoReplan now emits a brake request (Some) instead of
        // None. Verify it returns Some with steering=None, brake>0, and the
        // correct blackboard keys.
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("AutoReplan".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(
            req.is_some(),
            "AutoReplan should emit a brake ControlRequest"
        );
        let req = req.unwrap();
        assert!(
            req.steering.is_none(),
            "steering must remain None during AutoReplan"
        );
        assert!(req.brake.is_some(), "brake must be Some during AutoReplan");
        assert!(
            req.brake.unwrap() >= SAFETY_BRAKE_MIN,
            "brake >= SAFETY_BRAKE_MIN"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("decelerating_lane_authority_lost")
        );
    }

    #[test]
    fn disengaging_stage_returns_none() {
        // Phase 2h-Safety: Disengaging now emits a brake request (Some) and sets
        // autopilot.disengage_requested, instead of returning None silently.
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(
            req.is_some(),
            "Disengaging should emit a brake ControlRequest"
        );
        let req = req.unwrap();
        assert!(
            req.steering.is_none(),
            "steering must remain None during Disengaging"
        );
        assert!(req.brake.is_some(), "brake must be Some during Disengaging");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("disengaging_lane_authority_lost")
        );
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true")
        );
    }

    #[test]
    fn stage_change_soft_to_normal_resets_pid() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("SoftLaneKeep".to_string()),
            previous_heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(lk.previous_heading_stage, Some("SoftLaneKeep".to_string()));
    }

    #[test]
    fn normal_stage_produces_steering() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(result.is_some());
        assert!(result.unwrap().steering.is_some());
    }

    #[test]
    fn phase_6_5p_guard_takes_priority_over_heading_stage() {
        // Phase 2h-Safety: heading_mismatch gate now returns a brake request (not None).
        // The gate still fires BEFORE the stage gate â€” safety_state = decelerating_heading_mismatch.
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(
            result.is_some(),
            "heading_mismatch gate must emit brake request"
        );
        let result = result.unwrap();
        assert!(result.steering.is_none());
        assert!(result.brake.is_some());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("decelerating_heading_mismatch")
        );
    }

    #[test]
    fn route_following_no_waypoints_yields() {
        let mut lk = LaneKeeperPlugin::default();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("no_waypoints")
        );
    }

    // â”€â”€ Vision-mode tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    #[test]
    fn vision_level0_produces_steering() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.1, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L0 must produce steering");
        assert!(req.steering.is_some());
        assert_eq!(req.priority, PRIORITY_NORMAL);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
    }

    #[test]
    fn vision_state_gate_returns_none_when_off() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_engine_off_returns_none() {
        let mut plugin = make_vision_plugin();
        let mut t = make_telemetry(20.0, 0.0);
        t.engine_rpm = 0.0;
        let ctx = vision_bb("Active", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("engine_off")
        );
    }

    #[test]
    fn vision_engage_allowed_true_at_level0() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump 5 ticks to build rolling avg > 0.70
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn vision_engage_allowed_false_at_level2() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Low confidence â†’ level 2
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.20, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.20, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_produces_high_priority_request() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // 105 blind ticks â†’ L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L4 must produce ControlRequest");
        assert_eq!(req.priority, PRIORITY_LEVEL4);
        assert!(req.steering.is_none(), "L4 must not steer");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_brake_first_50_ticks() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Get to L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        // First L4 tick â†’ brake = 0.30
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.brake, Some(0.30), "first L4 tick must brake at 0.30");
    }

    #[test]
    fn vision_center_offset_steers_toward_lane_center() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Positive center_offset = truck to the right â†’ steer left (negative)
        // vision_error = -center_offset = -0.3 â†’ negative PID â†’ negative steering
        // Pump a few ticks so rolling avg stabilises before the final assertion.
        for _ in 0..5 {
            let c = vision_bb("Active", 0.3, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &c);
        }
        let ctx = vision_bb("Active", 0.3, 0.85, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s < 0.0, "positive offset â†’ steer left (negative), got {s}");
    }

    #[test]
    fn vision_rate_limiter_clamps_first_tick() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Large offset â†’ PID would want large output, rate-limiter clamps it
        let ctx = vision_bb("Active", 1.0, 0.95, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        if let Some(s) = req.steering {
            assert!(
                s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9,
                "rate limiter must clamp first-tick output, got {s}"
            );
        }
    }

    #[test]
    fn vision_reset_on_off_clears_engagement_heading() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert!(plugin.engagement_heading.is_some());

        let ctx_off = vision_bb("Off", 0.0, 0.0, false, false);
        let _ = plugin.tick_request(Some(&t), &ctx_off);
        assert!(plugin.engagement_heading.is_none());
    }

    #[test]
    fn vision_on_load_publishes_engage_allowed_false() {
        let mut plugin = make_vision_plugin();
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Off");
        let ctx = PluginContext::new("lane-keeper", bb);
        plugin.on_load(&ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    // â”€â”€ Off-state engage_allowed tests (chicken-and-egg fix) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    #[test]
    fn vision_off_state_publishes_engage_allowed_with_good_lane_detection() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump ticks in Off state with valid lane data â€” engage_allowed must become true.
        for _ in 0..5 {
            let ctx = vision_bb("Off", 0.0, 0.85, true, true);
            let result = plugin.tick_request(Some(&t), &ctx);
            assert!(result.is_none(), "Off state must produce no steering");
        }
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        let result = plugin.tick_request(Some(&t), &ctx);
        assert!(result.is_none(), "Off state must produce no steering");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_off_state_produces_no_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn vision_active_state_produces_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Warm up fallback in Active state so rolling avg is stable.
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("Active must produce ControlRequest");
        assert!(req.steering.is_some(), "Active must produce steering");
    }

    // â”€â”€ Lane-offset (Rechtsfahrgebot) tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    #[test]
    fn lane_offset_north_road_shifts_target_right() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]], // North road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck on centerline heading North â†’ offset shifts target East (+x).
        // err > 0: must steer right toward right lane.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        assert!(
            err > 0.0 && err < 0.1,
            "north road: expected small positive error (target right of center), got {err:.4}",
        );
    }

    #[test]
    fn lane_offset_east_road_shifts_target_south() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]], // East road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck on centerline heading East (ETS2=0.75) â†’ offset shifts target South (+z).
        // target angle > Ï€/2, heading_rad = Ï€/2 â†’ err > 0.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.75, 10.0, &ctx);
        assert!(
            err > 0.0 && err < 0.1,
            "east road: expected small positive error (target shifted South), got {err:.4}",
        );
    }

    // â”€â”€ Phase 2c/2d: SplineIndex route-geometry tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // These exercise `try_spline_heading_error` (the spline path) and its
    // Catmull-Rom fallback dispatch in `compute_heading_error`.

    use truckpilot_map_parser::build_index_with_metadata;
    use truckpilot_map_parser::spline::{HermiteSegment, SegmentMetadata, Vec3};

    /// Straight Hermite segment with chord tangents (m0=m1=p1-p0).
    fn seg(p0: (f32, f32), p1: (f32, f32), from: u64, to: u64) -> HermiteSegment {
        let a = Vec3::new(p0.0, 0.0, p0.1);
        let b = Vec3::new(p1.0, 0.0, p1.1);
        let m = b - a;
        HermiteSegment {
            p0: a,
            p1: b,
            m0: m,
            m1: m,
            length_m: m.length(),
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    /// Road metadata: lane_offset_right_m from compute_lane_offset_right_m for non-prefab.
    fn road_meta(lanes: u8, w: f32, prefab: bool) -> SegmentMetadata {
        SegmentMetadata {
            lanes_in_direction: lanes,
            lanes_opposite: lanes,
            lanes_total: lanes * 2,
            lane_width_m: w,
            lane_offset_right_m: if prefab {
                0.0
            } else {
                truckpilot_map_parser::compute_lane_offset_right_m(lanes, w, 0.0)
            },
            road_offset_m: 0.0,
            road_look_token: 0,
            is_prefab: prefab,
        }
    }

    /// Wire a plugin via on_load with the given segments/metadata, router graph and
    /// route_node_ids JSON. `road_seg_count` is how many segments are road-derived.
    fn wired_plugin(
        segs: Vec<HermiteSegment>,
        metas: Vec<Option<SegmentMetadata>>,
        nodes: Vec<(u64, f64, f64)>,
        edges: Vec<(u64, u64, f64)>,
        route_json: &str,
        road_seg_count: usize,
    ) -> (LaneKeeperPlugin, PluginContext) {
        let idx = Arc::new(build_index_with_metadata(segs, metas));
        let rg = Arc::new(RouterGraph::new(nodes, edges));
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.route_node_ids", route_json);
        let mut ctx = PluginContext::new("lane-keeper", bb)
            .with_spline_index(Arc::clone(&idx), road_seg_count);
        ctx.graph = Some(Arc::clone(&rg));
        let mut lk = LaneKeeperPlugin::default();
        lk.on_load(&ctx);
        (lk, ctx)
    }

    /// Test 1: 3-lane north road â†’ lane_offset_right_m = 3.75 m,
    /// lateral_source = "spline_road".
    #[test]
    fn spline_road_3lane_offset_9375() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        // Truck on centerline at origin, heading North (0.0), speed 20 m/s.
        let _err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "must take the spline-road path"
        );
        let applied = ctx
            .blackboard
            .get("lane_keeper.lane_offset_applied_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("lane_offset_applied_m must parse");
        assert!(
            (applied - 3.75).abs() < 0.01,
            "expected offset â‰ˆ 3.75, got {applied}"
        );
        assert!(
            applied > 0.0,
            "spline offset must be positive for 3-lane road, got {applied}"
        );
    }

    /// Test 2: sign check â€” north travel, offset shifts lookahead East (+x) â†’
    /// look_x > 0 and err > 0 (steer right).
    #[test]
    fn spline_offset_right_of_travel_direction() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        let look_x = ctx
            .blackboard
            .get("lane_keeper.look_x")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("look_x must parse");
        assert!(
            look_x > 0.0,
            "right of North-travel = East (+x), got {look_x}"
        );
        assert!(
            err > 0.0,
            "target right of heading â†’ positive error, got {err}"
        );
    }

    /// Test 3: route hop is reversed vs. the indexed segment direction â†’
    /// get((20,10)) misses â†’ catmullrom_fallback.
    #[test]
    fn reversed_hop_falls_back_to_catmull() {
        // Map segment direction is 10â†’20, but the route walks 20â†’10.
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(20u64, 10u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,10]", 1);
        // Provide Catmull waypoints so the fallback path has geometry to chew on.
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "reversed hop has no forward segment â†’ must fall back"
        );
    }

    /// Test 4: route nodes not present in the segment map â†’ catmullrom_fallback.
    #[test]
    fn route_miss_falls_back_to_catmull() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        // Graph has positions for 99/98 so node-advance can run, but no segment maps them.
        let nodes = vec![(99u64, 0.0, 0.0), (98u64, 0.0, -200.0)];
        let edges = vec![(99u64, 98u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[99,98]", 1);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "unknown route nodes â†’ must fall back"
        );
    }

    /// Test 5: no SplineIndex (default plugin, no on_load wiring) â†’ existing
    /// Catmull-Rom behaviour, still produces a sensible error.
    #[test]
    fn no_index_falls_back_to_catmull() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        };
        // ctx carries a route but no spline index â†’ try_spline returns None early.
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.route_node_ids", "[10,20]");
        let ctx = PluginContext::new("lane-keeper", bb);

        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "index=None â†’ must use catmull fallback"
        );
        // North road, truck on centerline â†’ small positive error toward the right lane.
        assert!(
            err > 0.0 && err < 0.2,
            "fallback must still yield a sensible small positive error, got {err}"
        );
    }

    // â”€â”€ H2-Catmull-Offset-Fix: per-Segment lane_offset_right_m statt fix 1.875 â”€â”€â”€â”€â”€â”€
    //
    // Aufbau (wie reversed_hop_falls_back_to_catmull): reversed route â†’ der globale
    // nearest findet das Nord-Segment (heading passt), setzt cur_seg/last_nearest_seg,
    // dann off_route â†’ None â†’ Catmull-Fallback. Der Catmull-Offset stammt jetzt aus
    // index.metadata[last_nearest_seg]. Helfer road_meta(lanes,w,false) =>
    // lane_offset_right_m via compute_lane_offset_right_m: 1-lane=0, 2-lane=1.875, prefab=0.

    /// H2-1: 2-spuriges nearest-Segment â†’ Catmull-Offset 1.875 m (statt fix 1.875).
    #[test]
    fn catmull_offset_uses_segment_lane_offset() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, false))]; // 2-lane â†’ 1.875
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(20u64, 10u64, 200.0)]; // reversed â†’ off_route â†’ Catmull
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,10]", 1);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "reversed hop â†’ Catmull-Fallback"
        );
        assert_eq!(
            lk.last_nearest_seg,
            Some(0),
            "nearest-Segment dieses Ticks muss festgehalten sein"
        );
        let off = bb_f32(&ctx, "lane_keeper.lane_offset_applied_m");
        assert!(
            (off - 1.875).abs() < 0.01,
            "Catmull-Offset muss der 2-lane-lane_offset_right_m (1.875) sein, got {off:.3}"
        );
    }

    /// H2-2: nearest-Segment ist Prefab (NavCurve) â†’ Catmull-Offset 0 (konsistent zum Spline).
    #[test]
    fn catmull_offset_zero_on_prefab() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, true))]; // is_prefab=true â†’ 0
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(20u64, 10u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,10]", 1);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
        );
        let off = bb_f32(&ctx, "lane_keeper.lane_offset_applied_m");
        assert!(
            off.abs() < 0.01,
            "Prefab-nearest â†’ Catmull-Offset 0, got {off:.3}"
        );
    }

    /// H2-3: kein frisches nearest (index=None â†’ try_spline bricht vor der Query ab) â†’
    /// Default-Offset 1.875 m, NICHT 0 (sonst mittig auf 1-spurig).
    #[test]
    fn catmull_offset_default_without_segment() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.route_node_ids", "[10,20]");
        let ctx = PluginContext::new("lane-keeper", bb);

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "index=None â†’ Catmull-Fallback"
        );
        assert_eq!(
            lk.last_nearest_seg, None,
            "index=None: nearest-Query lief nie â†’ last_nearest_seg bleibt None"
        );
        let off = bb_f32(&ctx, "lane_keeper.lane_offset_applied_m");
        assert!(
            (off - 1.875).abs() < 0.01,
            "ohne frisches nearest â†’ Default 1.875 (NICHT 0), got {off:.3}"
        );
    }

    /// Test 6: prefab hop â†’ lateral_source = "spline_prefab", offset â‰ˆ 0.0.
    #[test]
    fn prefab_hop_zero_offset_spline_prefab() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, true))]; // is_prefab=true
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_prefab"),
            "prefab segment â†’ spline_prefab source"
        );
        let applied = ctx
            .blackboard
            .get("lane_keeper.lane_offset_applied_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("lane_offset_applied_m must parse");
        assert!(
            applied.abs() < 0.001,
            "prefab offset must be â‰ˆ 0, got {applied}"
        );
    }

    /// Test 7: 2 forward segments, lookahead crosses the segment boundary â†’
    /// lookahead_hop_count = 1, lateral_source = "spline_road".
    #[test]
    fn multi_hop_walk_crosses_boundary() {
        // Seg A: (0,0)â†’(0,-30) [10â†’20], Seg B: (0,-30)â†’(0,-200) [20â†’30].
        let segs = vec![
            seg((0.0, 0.0), (0.0, -30.0), 10, 20),
            seg((0.0, -30.0), (0.0, -200.0), 20, 30),
        ];
        let metas = vec![
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
        ];
        // Node 20 sits at (0,-30): far enough (>5m) from the truck at origin that
        // node_progress_idx does NOT advance, so the current hop stays 10â†’20.
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -30.0), (30u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 30.0), (20u64, 30u64, 170.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // speed 20 m/s â†’ look_ahead = 5 + 20*3.6*0.5 = 41m > 30m seg-A length â†’ lands on seg B.
        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "multi-hop walk stays on the spline path"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.lookahead_hop_count")
                .as_deref(),
            Some("1"),
            "41m lookahead must walk exactly one hop past the 30m boundary"
        );
    }

    /// Test 8: truck 100m laterally off the segment (> MAX_HOP_PROJECTION_DIST_M=50)
    /// â†’ dist gate trips â†’ catmullrom_fallback.
    #[test]
    fn dist_gate_far_truck_falls_back() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        // Graph node positions far from the truck so node-advance does not fire,
        // but the route does resolve to a forward hop; the per-segment projection
        // distance is what must trip the gate.
        let nodes = vec![(10u64, 100.0, 0.0), (20u64, 100.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);
        lk.waypoints = vec![[100.0, 0.0], [100.0, -100.0], [100.0, -200.0]];

        // Truck at x=100, segment centerline at x=0 â†’ projection distance â‰ˆ 100m > 40m.
        lk.compute_heading_error(100.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "projection distance > 50m must trip the dist gate â†’ fallback"
        );
    }

    /// Test 9 (Finding-A regression): disengage resets node_progress_idx,
    /// was_spline_active and the cached route, so re-engage on the identical
    /// route does not resume mid-route with a stale index.
    #[test]
    fn disengage_resets_node_progress() {
        // 4-node route so node_progress can advance a couple of steps.
        let segs = vec![
            seg((0.0, 0.0), (0.0, -30.0), 10, 20),
            seg((0.0, -30.0), (0.0, -60.0), 20, 30),
            seg((0.0, -60.0), (0.0, -200.0), 30, 40),
        ];
        let metas = vec![
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -30.0),
            (30u64, 0.0, -60.0),
            (40u64, 0.0, -200.0),
        ];
        let edges = vec![
            (10u64, 20u64, 30.0),
            (20u64, 30u64, 30.0),
            (30u64, 40u64, 140.0),
        ];
        let route = "[10,20,30,40]";
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes.clone(), edges, route, 3);
        // Waypoints must be present, otherwise tick_request short-circuits at
        // "no_waypoints" before compute_heading_error advances node_progress_idx.
        // (In production the router writes both router.waypoints and
        // router.route_node_ids each tick.)
        lk.waypoints = vec![[0.0, 0.0], [0.0, -60.0], [0.0, -200.0]];

        // Drive a few Active ticks near node 20 (0,-30) then node 30 (0,-60) so
        // node_progress_idx advances past 0.
        let mut t = make_telemetry(20.0, 0.0);
        // Position the truck within reach (<5m) of node 20 â†’ advance to idx 1.
        t.position = [0.0, 0.0, -28.0];
        let _ = lk.tick_request(Some(&t), &ctx);
        // Now within reach of node 30 â†’ advance to idx 2.
        t.position = [0.0, 0.0, -58.0];
        let _ = lk.tick_request(Some(&t), &ctx);
        assert!(
            lk.node_progress_idx > 0,
            "precondition: node_progress_idx must have advanced (got {})",
            lk.node_progress_idx
        );

        // Disengage: state=Off tick.
        ctx.blackboard.set("autopilot.state", "Off");
        let off = lk.tick_request(Some(&t), &ctx);
        assert!(off.is_none(), "Off state yields no control request");

        // All spline route state must be reset.
        assert_eq!(
            lk.node_progress_idx, 0,
            "node_progress_idx must reset to 0 on disengage"
        );
        assert!(
            !lk.was_spline_active,
            "was_spline_active must reset to false on disengage"
        );
        assert_eq!(
            lk.cached_route_hash, 0,
            "cached_route_hash must reset to 0 on disengage"
        );
        assert!(
            lk.cached_route_node_ids.is_empty(),
            "cached_route_node_ids must be cleared on disengage"
        );

        // Re-engage on the IDENTICAL route from the start position â†’ fresh start.
        ctx.blackboard.set("autopilot.state", "Active");
        let mut t_start = make_telemetry(20.0, 0.0);
        t_start.position = [0.0, 0.0, 0.0];
        let _ = lk.tick_request(Some(&t_start), &ctx);
        // At the route start, node 20 (0,-30) is 30m away (>5m) â†’ no advance â†’
        // node_progress_idx stays 0 (no stale resume into the middle of the route).
        assert_eq!(
            lk.node_progress_idx, 0,
            "re-engage at start must keep node_progress_idx at 0, not resume stale"
        );
    }

    // â”€â”€ Phase 2f-B: nearest-hop re-anchor tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // Geometry convention: straight Hermite segments in ETS2 XZ (x=East, z=South-negative
    // for northward travel). All segments are 100 m, so the second segment spans z=-100..-200.
    //
    //  Node 10 at (0,    0)
    //  Seg 0 : 10â†’20  (0,0) â†’ (0,-100)   length 100 m
    //  Node 20 at (0, -100)
    //  Seg 1 : 20â†’30  (0,-100) â†’ (0,-200) length 100 m
    //  Node 30 at (0, -200)

    /// Helper: 3-node north-road route `[10,20,30]` with two 100 m straight segments.
    /// Returns `(plugin, ctx)` wired via `on_load` (same pattern as existing spline tests).
    fn three_node_route() -> (LaneKeeperPlugin, PluginContext) {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),
            seg((0.0, -100.0), (0.0, -200.0), 20, 30),
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
        ];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2)
    }

    /// Test R1: Re-anchor advances `node_progress_idx` when truck is longitudinally
    /// deep on the second segment (near node 30).
    ///
    /// The old 5m-euclidean advance would never fire here because the truck never gets
    /// within 5 m of the median-node positions. The re-anchor scans all hops on the
    /// first (route_changed) tick and picks the hop whose segment is closest: that is
    /// hop 1 (20â†’30) because the truck is at z=-160, far from seg 0 (z=0..-100).
    #[test]
    fn reanchor_advances_on_longitudinal_progress() {
        let (mut lk, ctx) = three_node_route();
        // Truck sits 60 m into the second segment (z = -160, well past node 20 at z=-100).
        // On the first tick route_changed=true â†’ full scan â†’ seg 1 (20â†’30) wins.
        lk.compute_heading_error(0.0, -160.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "re-anchor must select hop 1 (20â†’30) when truck is 60 m into the second segment; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("20->30"),
            "current_hop must reflect the advanced hop"
        );
    }

    /// Test R2: Re-anchor is not blocked by lateral offset (the core fix).
    ///
    /// Truck is on the second segment (hop 1, 20â†’30) but displaced 5.6 m laterally â€”
    /// the typical right-lane offset in ETS2. The old euclidean node-advance checked
    /// distance to the MEDIAN node (0, -100) and found >5 m â†’ never advanced. The
    /// re-anchor uses `project_on_segment`, which measures perpendicular distance, so
    /// the lateral offset costs only ~5.6 m (< MAX_HOP_PROJECTION_DIST_M = 50 m) and
    /// the second segment still wins against the first (which is ~100 m away along z).
    #[test]
    fn reanchor_lateral_offset_does_not_block_advance() {
        let (mut lk, ctx) = three_node_route();
        // Truck is longitudinally mid-second-segment (z=-150) and 5.6 m east (x=5.6).
        // project_on_segment for seg 0 (z=0..-100): truck is ~50 m past the end â†’ clamped at t=1,
        // distance â‰ˆ sqrt(5.6Â²+50Â²) â‰ˆ 50 m.
        // project_on_segment for seg 1 (z=-100..-200): t â‰ˆ 0.5, distance â‰ˆ 5.6 m â†’ wins.
        lk.compute_heading_error(5.6, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "lateral offset of 5.6 m must not prevent re-anchor to hop 1 (20â†’30); got {}",
            lk.node_progress_idx
        );
        // Confirm the spline path actually engaged (dist < 40 m gate).
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage despite lateral offset"
        );
    }

    /// Test R3: Re-anchor does not jump backward on a normal (non-replan) tick.
    ///
    /// `node_progress_idx` is pre-set to 1 (hop 20â†’30). On a non-replan tick the
    /// scan window is `[1, 1+16)` â€” forward-only. The truck is placed on hop 1,
    /// so hop 1 wins the scan. The index must not drop back to 0.
    #[test]
    fn reanchor_does_not_jump_backward_on_normal_tick() {
        let (mut lk, ctx) = three_node_route();

        // Prime the route cache so the SECOND call is a non-replan tick.
        // First call: route_changed=true, truck at beginning (hop 0 wins â†’ idx stays 0).
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);

        // Manually advance idx to 1, then call again WITHOUT changing the route.
        // The second call sees route_changed=false â†’ window [1, 17) â†’ forward-only.
        lk.node_progress_idx = 1;
        // Truck still on the second segment.
        lk.compute_heading_error(0.0, -150.0, 0.0, 0.0, &ctx);

        assert!(
            lk.node_progress_idx >= 1,
            "non-replan re-anchor must not jump backward; idx dropped to {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("20->30"),
            "current_hop must still be 20->30 after forward-only scan"
        );
    }

    /// Test R4: Re-anchor re-establishes position after a route change (replan).
    ///
    /// Simulates a replan: same geometry, but the route JSON string changes so
    /// `cached_route_hash` differs and `route_changed=true`. The truck is positioned
    /// deep in the route (hop 1). The full-route scan (lo=0, hi=route.len()-1) must
    /// pick hop 1, not remain stuck at 0.
    #[test]
    fn reanchor_picks_truck_segment_after_route_change() {
        let (mut lk, ctx) = three_node_route();

        // First tick: establish cache with truck on hop 0.
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);
        assert_eq!(lk.node_progress_idx, 0, "precondition: truck on hop 0");

        // Simulate a replan by writing a new route_node_ids JSON (same nodes, but
        // whitespace changes the hash so cached_route_hash â‰  new hash).
        ctx.blackboard
            .set("router.route_node_ids", "[ 10 , 20 , 30 ]");

        // Second tick: route_changed=true â†’ full scan â†’ truck now deep on hop 1.
        lk.compute_heading_error(0.0, -160.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "after replan, full-route scan must re-anchor to hop 1 (truck at z=-160); got {}",
            lk.node_progress_idx
        );
    }

    // â”€â”€ Phase 2g: 2D-nearest segment selection + Y-ignoring gate â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // These guard the Phase-2g fix in `try_spline_heading_error`:
    //   (a) the forward scan picks the geometrically nearest route segment by 2D
    //       (XZ) distance, NOT route[node_progress_idx]'s far segment;
    //   (b) the gate / reported distance is 2D â€” node-height (Y) is ignored.

    /// Straight Hermite segment with a fixed Y on both endpoints (height baked in).
    /// Tangents are chord = p1-p0 in XZ only (Y delta 0), matching `seg`'s flat tangent.
    fn seg_y(p0: (f32, f32), p1: (f32, f32), y: f32, from: u64, to: u64) -> HermiteSegment {
        let a = Vec3::new(p0.0, y, p0.1);
        let b = Vec3::new(p1.0, y, p1.1);
        let m = Vec3::new(p1.0 - p0.0, 0.0, p1.1 - p0.1);
        HermiteSegment {
            p0: a,
            p1: b,
            m0: m,
            m1: m,
            length_m: (b - a).length(),
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    /// Test 2g-1 (H-C core case): the lookahead must pick the route segment the
    /// truck actually sits on, NOT route[0]'s far segment.
    ///
    /// Route `[10,20,30,40]`:
    ///   hop 0 (10â†’20) is a segment FAR from the truck (centerline at x=200, â‰ˆ200m off)
    ///   hop 2 (30â†’40) is the segment the truck sits on (â‰ˆ2m off in XZ)
    /// The old "blind route[node_progress_idx]" lookup would anchor on hop 0 and trip
    /// the dist gate (>40m). The 2g scan must pick hop 2 â†’ `node_progress_idx=2`,
    /// `truck_to_segment_dist_m` < 10m, `fallback_reason="none"`, `lateral_source="spline_road"`.
    #[test]
    fn reanchor_picks_truck_segment_not_route0_far_segment() {
        // hop 0/1 live way out east (x=200); hop 2 (30â†’40) runs north under the truck.
        let segs = vec![
            seg((200.0, 0.0), (200.0, -100.0), 10, 20),  // far
            seg((200.0, -100.0), (0.0, -100.0), 20, 30), // connector
            seg((0.0, -100.0), (0.0, -300.0), 30, 40),   // under the truck
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 200.0, 0.0),
            (20u64, 200.0, -100.0),
            (30u64, 0.0, -100.0),
            (40u64, 0.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 200.0),
            (30u64, 40u64, 200.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30,40]", 3);

        // Truck on hop 2's centerline, ~2m west of it (x=2), mid-segment (z=-200), heading North.
        lk.compute_heading_error(2.0, -200.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 2,
            "scan must anchor on hop 2 (30â†’40, under the truck), not route[0]'s far hop; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("30->40"),
            "current_hop must be the nearby segment"
        );
        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            dist < 10.0,
            "truck-to-segment distance must be the NEAR segment's (~2m), got {dist}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "near segment is well within the gate â†’ no fallback"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the near segment"
        );
    }

    /// Test 2g-2 (Y-2D proof): node height must be ignored when gating.
    ///
    /// A single segment sits ~2m (XZ) from the truck but is baked at y=37 (typical
    /// Berlin node height). The truck is queried at y=0. The OLD 3D
    /// `project_on_segment` distance would be â‰ˆ sqrt(2Â² + 37Â²) â‰ˆ 37m and â€” with a
    /// taller height â€” would exceed the 40m gate. The 2g fix measures XZ only, so the
    /// reported `truck_to_segment_dist_m` must be ~2m and the gate must NOT trip.
    #[test]
    fn gate_uses_2d_distance_ignoring_node_height() {
        let segs = vec![seg_y((0.0, 0.0), (0.0, -200.0), 37.0, 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        // Truck 2m east of the centerline at y=0 (implicit), mid-segment.
        lk.compute_heading_error(2.0, -100.0, 0.0, 0.0, &ctx);

        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            (dist - 2.0).abs() < 0.5,
            "reported distance must be 2D (~2m), not 3D (~37m incl. node height), got {dist}"
        );
        // Direct proof the Y-fix matters: the 3D distance would already exceed the gate
        // for a taller node, and even at 37m it is far from the 2m we expect.
        assert!(
            dist < MAX_HOP_PROJECTION_DIST_M as f64,
            "2D distance is below the 40m gate; height must not inflate it (got {dist})"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "Y height ignored â†’ gate does not trip â†’ spline path engages"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage despite the 37m node height"
        );
    }

    /// Test 2g-3 (heading filter, U-turn): two route segments equally near the truck,
    /// one in the truck's heading direction and one doubled back (Î”â‰ˆ180Â°). The
    /// heading-compatible candidate (same direction) must be chosen.
    ///
    /// Route `[10,20,30]`:
    ///   hop 0 (10â†’20): runs NORTH (z: 0 â†’ -100). Truck heads North â†’ compatible.
    ///   hop 1 (20â†’30): runs back SOUTH (z: -100 â†’ 0), i.e. it folds back over hop 0.
    /// The truck sits at z=-50 â€” equidistant (in 2D) from both overlapping segments â€”
    /// but heading North. The heading filter (dot â‰¥ 0.5) must keep hop 0 and reject the
    /// reversed hop 1, so `node_progress_idx=0` and `current_hop="10->20"`.
    #[test]
    fn heading_filter_prefers_aligned_segment_over_doubled_back() {
        // Both segments occupy the SAME XZ corridor (x=0, z in [0,-100]) but opposite
        // direction, so 2D distance alone cannot disambiguate â€” only heading can.
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // North
            seg((0.0, -100.0), (0.0, 0.0), 20, 30), // South (doubled back)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -100.0), (30u64, 0.0, 0.0)];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // Truck mid-corridor (z=-50), heading North (0.0) â†’ only hop 0 is heading-compatible.
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 0,
            "heading filter must pick the North-aligned hop 0, not the doubled-back hop 1; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "current_hop must be the heading-compatible segment"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the aligned segment"
        );
    }

    // â”€â”€ Phase 2g (Variante B): global-nearest + route-relevance + W1/W2 â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // These guard the Variante-B reroute of `try_spline_heading_error`:
    //   - the GLOBAL R-tree nearest-query (`nearest_with_heading_filter`) finds the
    //     geometrically closest, heading-compatible segment (not a route-only scan),
    //   - the route-relevance gate accepts it as ON-ROUTE (âˆƒj: route[j]==F, route[j+1]==T)
    //     or FEEDS-INTO (âˆƒk: route[k]==T = predecessor/snap edge), else falls back,
    //   - W1: feeds-into only if `hit.heading_filter_applied`,
    //   - W2: feeds-into only if `(route[k],route[k+1])` is a forward hop in seg_by_from_to.

    /// Test VB-1 (the real H-C case Variante b could NOT do): the truck SITS on the
    /// predecessor/snap edge Pâ†’A whose head is route[0]=A. The first on-route hop
    /// Aâ†’B is the far snap-endpoint hop (>40m east). The global-nearest query finds
    /// Pâ†’A (~2m), feeds-into A=route[0], heading-compatible (W1) and Aâ†’B is a forward
    /// hop (W2) â†’ accepted. The route-only forward scan would have measured Aâ†’B at >40m
    /// and tripped the dist gate.
    ///
    /// Route `[20,30,40]` (A=20, B=30, C=40):
    ///   predecessor Pâ†’A = 10â†’20  : (0,0)â†’(0,-100), runs NORTH under the truck (~2m)
    ///   hop Aâ†’B        = 20â†’30  : (0,-100)â†’(200,-100), runs EAST, FAR from the truck
    ///   hop Bâ†’C        = 30â†’40  : (200,-100)â†’(200,-300)
    #[test]
    fn feeds_into_predecessor_edge_picks_snap_segment() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // predecessor Pâ†’A, under truck
            seg((0.0, -100.0), (200.0, -100.0), 20, 30), // hop Aâ†’B, far (east)
            seg((200.0, -100.0), (200.0, -300.0), 30, 40), // hop Bâ†’C
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 200.0, -100.0),
            (40u64, 200.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 200.0),
            (30u64, 40u64, 200.0),
        ];
        // Route does NOT contain P (=10); it starts at A (=20).
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,30,40]", 3);

        // Truck on the predecessor edge Pâ†’A, 2m east of its centerline, mid-segment,
        // heading North â†’ heading-compatible with the North-running Pâ†’A.
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 0,
            "feeds-into must anchor on route[0]=A (the snap endpoint); got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "current_hop must be the predecessor/snap edge P->A"
        );
        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            dist < 10.0,
            "distance must be to the NEAR predecessor edge (~2m), not the far A->B hop, got {dist}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "feeds-into accepted (W1+W2) â†’ no fallback"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the predecessor edge"
        );
    }

    /// Test VB-2 (off-route protection): the global-nearest segment is a PARALLEL road
    /// Xâ†’Y that is neither on-route nor feeds into a route node. The route hops are far
    /// away. The route-relevance gate must reject the nearest hit â†’ off-route â†’ Catmull.
    ///
    /// Route `[10,20,30]` (Aâ†’Bâ†’C), all hops far east (x=200).
    /// Parallel road Xâ†’Y = 90â†’91 : (0,0)â†’(0,-200) directly under the truck; neither 90
    /// nor 91 is a route node, and 91 is not the `to` of any route hop â†’ off-route.
    #[test]
    fn off_route_nearest_segment_falls_back_to_catmull() {
        let segs = vec![
            seg((200.0, 0.0), (200.0, -100.0), 10, 20), // route hop A->B (far east)
            seg((200.0, -100.0), (200.0, -200.0), 20, 30), // route hop B->C (far east)
            seg((0.0, 0.0), (0.0, -200.0), 90, 91),     // parallel road X->Y, under truck
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 200.0, 0.0),
            (20u64, 200.0, -100.0),
            (30u64, 200.0, -200.0),
            (90u64, 0.0, 0.0),
            (91u64, 0.0, -200.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (90u64, 91u64, 200.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 3);
        // Catmull fallback needs geometry to chew on.
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        // Truck on the parallel road (x=2 east of X->Y centerline), heading North.
        // Nearest segment globally = X->Y (~2m), but it is off-route.
        lk.compute_heading_error(2.0, -100.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("off_route"),
            "nearest segment is a parallel road (not on-route, not feeds-into) â†’ off_route"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "off-route nearest must NOT hijack the spline path â†’ Catmull fallback"
        );
    }

    // â”€â”€ Fix C: On-Route-NavCurve in route_seg_set (Junction-Durchfahrt) â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // An Junctions sitzt der Truck auf einer NavCurve (Prefab-Segment, Index >=
    // road_seg_count). Vor Fix C war seg_by_from_to road-only â†’ die On-Route-NavCurve
    // lag NIE in route_seg_set â†’ route_hit=None â†’ heading_stage/Disengage â†’ niemand
    // lenkt â†’ geradeaus in die Leitplanke. Fix C nimmt NavCurves, deren (from,to) auf
    // einem konsekutiven Route-Paar liegen, in route_seg_set mit auf.

    /// Fix-C-1: Eine On-Route-NavCurve (20â†’30) unter dem Truck wird als route_hit
    /// gewÃ¤hlt; chosen_segment_is_navcurve=true, kein off_route/heading_stage-Fallback.
    #[test]
    fn fixc_navcurve_on_route_is_selected() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // road 10->20  (index 0)
            seg((0.0, -100.0), (0.0, -200.0), 20, 30), // NavCurve 20->30 (index 1)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(1, 3.75, true)), // is_prefab â†’ NavCurve
        ];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -100.0), (30u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        // road_seg_count=1 â†’ index 1 is a NavCurve.
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 1);

        // Truck mid-NavCurve (z=-150, 2 m east), heading North â†’ on the 20->30 curve.
        lk.compute_heading_error(2.0, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chosen_segment_is_navcurve")
                .as_deref(),
            Some("true"),
            "the On-Route NavCurve under the truck must be the chosen segment"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_hop").as_deref(),
            Some("20->30"),
            "chosen hop must be the on-route NavCurve pair"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_filtered")
                .as_deref(),
            Some("true"),
            "route-aware query must accept the NavCurve as route_hit"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.total_route_navcurve_count")
                .as_deref(),
            Some("1"),
            "exactly one on-route NavCurve in route_seg_set"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "on-route NavCurve â†’ no heading_stage/off_route fallback"
        );
    }

    /// Fix-C-2 (Diskriminierung + Tie-Break, User-Punkt 2): am Junction-Knoten 20
    /// hÃ¤ngen ZWEI NavCurves â€” die On-Route (20â†’30, geradeaus) UND eine Off-Route
    /// (20â†’99, Abbieger), die GEOMETRISCH NÃ„HER am Truck liegt. Nur die On-Route
    /// (20,30) ist in route_seg_set; der route-gefilterte nearest-Query wÃ¤hlt sie,
    /// NICHT die nÃ¤here Off-Route-NavCurve. Das ist exakt der Crash-Fall.
    #[test]
    fn fixc_off_route_navcurve_excluded_even_when_closer() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // road 10->20            (index 0)
            seg((0.0, -100.0), (0.0, -200.0), 20, 30), // on-route NavCurve 20->30 (index 1)
            seg((0.0, -100.0), (30.0, -110.0), 20, 99), // off-route NavCurve 20->99 (index 2, Abbieger)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(1, 3.75, true)),
            Some(road_meta(1, 3.75, true)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
            (99u64, 30.0, -110.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (20u64, 99u64, 32.0),
        ];
        // road_seg_count=1 â†’ indices 1,2 are NavCurves.
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 1);

        // Truck just on the on-route curve, heading North. The off-route Abbieger
        // shares node 20 but its pair (20,99) is NOT on the route.
        lk.compute_heading_error(2.0, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_hop").as_deref(),
            Some("20->30"),
            "must pick the ON-ROUTE NavCurve, never the off-route Abbieger 20->99"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chosen_segment_is_navcurve")
                .as_deref(),
            Some("true")
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.total_route_navcurve_count")
                .as_deref(),
            Some("1"),
            "off-route NavCurve (20,99) must NOT enter route_seg_set"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none")
        );
    }

    /// Fix-C-3 (Vec, mehrere NavCurves am selben Knotenpaar): zwei NavCurves teilen
    /// (20,30) â€” beide On-Route â†’ BEIDE landen in route_seg_set (navcurve_count=2).
    #[test]
    fn fixc_multiple_navcurves_same_pair_both_enter_set() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // road 10->20            (index 0)
            seg((0.0, -100.0), (0.0, -200.0), 20, 30), // NavCurve 20->30 lane A  (index 1)
            seg((3.0, -100.0), (3.0, -200.0), 20, 30), // NavCurve 20->30 lane B  (index 2)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(1, 3.75, true)),
            Some(road_meta(1, 3.75, true)),
        ];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -100.0), (30u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 1);

        // Truck on lane A (xâ‰ˆ1), heading North.
        lk.compute_heading_error(1.0, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.total_route_navcurve_count")
                .as_deref(),
            Some("2"),
            "both NavCurves sharing the on-route pair (20,30) must enter route_seg_set"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chosen_segment_is_navcurve")
                .as_deref(),
            Some("true"),
            "nearest tie-break picks one of the on-route NavCurves"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none")
        );
    }

    /// Fix-C-4 (Regression â€” Gerade bleibt road-only): reine Road-Route mit einer
    /// NavCurve an einem NICHT-Route-Paar â†’ navcurve_count=0, Auswahl unverÃ¤ndert Road.
    #[test]
    fn fixc_straight_road_route_seg_set_stays_road_only() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // road 10->20 (index 0)
            seg((0.0, -100.0), (0.0, -200.0), 20, 30), // road 20->30 (index 1)
            seg((100.0, 0.0), (100.0, -100.0), 50, 51), // NavCurve off-route (index 2)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(1, 3.75, true)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
            (50u64, 100.0, 0.0),
            (51u64, 100.0, -100.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (50u64, 51u64, 100.0),
        ];
        // road_seg_count=2 â†’ only index 2 is a NavCurve, but it is off-route.
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // Truck on the straight road 10->20 (z=-50), heading North.
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.total_route_navcurve_count")
                .as_deref(),
            Some("0"),
            "no NavCurve on a route pair â†’ route_seg_set stays road-only on the straight"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chosen_segment_is_navcurve")
                .as_deref(),
            Some("false"),
            "straight road selection must remain a road segment"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_hop").as_deref(),
            Some("10->20"),
            "straight selection unchanged from pre-Fix-C"
        );
    }

    /// Test VB-3 (W2 explicit): feeds-into is rejected when the first forward walk-hop
    /// route[0]â†’route[1] does NOT exist as a forward segment.
    ///
    /// Route `[20,30]` (A=20, B=30). The truck sits on predecessor edge Pâ†’A = 10â†’20
    /// (head = A = route[0] â†’ feeds-into candidate, heading-compatible â†’ W1 passes).
    /// BUT the only indexed segment for the (A,B) corridor runs Bâ†’A (30â†’20), so
    /// `seg_by_from_to[(20,30)]` is MISSING â†’ W2 fails â†’ off_route â†’ Catmull.
    ///
    /// This differs from `reversed_hop_falls_back_to_catmull` (no feeds-into edge there;
    /// the truck sits ON the reversed route corridor). Here the feeds-into branch is
    /// entered and only W2 stops it, so the W2 guard itself is exercised.
    #[test]
    fn feeds_into_rejected_when_no_forward_hop() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // predecessor P->A, under truck
            seg((0.0, -200.0), (0.0, -100.0), 30, 20), // B->A only (no forward A->B)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
        ];
        // Edge 30->20 indexed (so seg_by_from_to has (30,20) but NOT (20,30)).
        let edges = vec![(10u64, 20u64, 100.0), (30u64, 20u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,30]", 2);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        // Truck on P->A (10->20), 2m east, heading North â†’ feeds-into A=route[0],
        // W1 passes (heading-compatible), but W2 fails: (20,30) is not a forward hop.
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("off_route"),
            "feeds-into without a forward route[0]->route[1] hop must be rejected (W2)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_detail").as_deref(),
            Some("feeds_into_no_forward_hop"),
            "W2 rejection detail must indicate the missing forward hop"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "W2 rejection â†’ Catmull fallback"
        );
    }

    // â”€â”€ Phase 2h-Befund3: route-aware nearest â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // These guard the route-aware nearest fix: at a junction the spline must stay
    // on the route hop instead of snapping to the geometrically nearer off-route
    // turn-off (which previously triggered off_route â†’ None â†’ Catmull cross-pull).

    /// Test RAN-1: at a junction an OFF-ROUTE turn-off is geometrically NEARER than
    /// the on-route hop. The route-aware nearest must still pick the ON-ROUTE hop
    /// and engage the spline, recording the discarded off-route candidate.
    ///
    /// Route `[10,20,30]` runs North along x=0.
    ///   on-route A->B = 10->20 : (0,0)->(0,-100)
    ///   on-route B->C = 20->30 : (0,-100)->(0,-200)
    ///   off-route T   = 20->90 : (2,-100)->(2,-300)  (parallel east at x=2, NOT on route)
    /// Truck at (1.5,-150) heading North: off-route x=2 line is 0.5m away, on-route
    /// x=0 line is 1.5m away â†’ the GLOBAL query would pick 20->90, the route-aware
    /// query must pick 20->30.
    #[test]
    fn route_aware_nearest_prefers_on_route_over_nearer_turnoff() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // on-route A->B (idx 0)
            seg((0.0, -100.0), (0.0, -200.0), 20, 30), // on-route B->C (idx 1)
            seg((2.0, -100.0), (2.0, -300.0), 20, 90), // off-route turn-off (idx 2), nearer
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
            (90u64, 2.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (20u64, 90u64, 200.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 3);

        lk.compute_heading_error(1.5, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_filtered")
                .as_deref(),
            Some("true"),
            "route-aware nearest must engage (on-route hop in range)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("20->30"),
            "must anchor on the ON-ROUTE hop 20->30, not the nearer off-route turn-off"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_discarded_offroute_hop")
                .as_deref(),
            Some("20->90"),
            "the geometrically-nearest off-route turn-off must be recorded as discarded"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline must engage on the route hop (no Catmull cross-pull)"
        );
    }

    /// Test RAN-2: when NO route segment is mappable (route hops not in the index â†’
    /// cached_route_seg_set empty), the query falls back to the global geometric
    /// nearest (nearest_route_filtered=false), preserving the pre-fix behaviour.
    ///
    /// Route `[10,20]` but the only indexed segment runs 20->10 (reversed), so
    /// seg_by_from_to has (20,10) not (10,20) â†’ route_seg_set is empty.
    #[test]
    fn route_aware_nearest_falls_back_without_route_segments() {
        let segs = vec![seg((0.0, -200.0), (0.0, 0.0), 20, 10)]; // reversed hop only
        let metas = vec![Some(road_meta(2, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(20u64, 10u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, -100.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_filtered")
                .as_deref(),
            Some("false"),
            "empty route_seg_set â†’ route-aware branch skipped, global nearest used"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_seg_set_size")
                .as_deref(),
            Some("0"),
            "no route hop maps to an indexed segment â†’ set size 0"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "reversed-hop route still degrades to Catmull (unchanged from pre-fix)"
        );
    }

    /// Test RAN-3: normal straight on-route road â€” route-aware and global agree, the
    /// spline engages on the route hop, no off-route candidate is discarded.
    /// Guards against regression on the common case.
    #[test]
    fn route_aware_nearest_normal_road_unchanged() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        lk.compute_heading_error(0.0, -100.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_filtered")
                .as_deref(),
            Some("true"),
            "single on-route road: route-aware nearest engages"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "must anchor on the only route hop"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_discarded_offroute_seg")
                .as_deref(),
            Some("none"),
            "global and route-aware agree â†’ nothing discarded"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "normal road must keep the spline path"
        );
    }

    /// Test RAN-4: heading gate. At an on-route TURN, the outgoing turn hop is
    /// geometrically NEARER than the incoming hop but ~90Â° off the truck heading.
    /// The route-aware nearest must NOT snap to the misaligned turn hop (which would
    /// force a premature turn-in); the heading gate rejects it â†’ fall back to the
    /// aligned incoming hop via the global query.
    ///
    /// Route `[10,20,30]`:
    ///   incoming A->B = 10->20 : (0,0)->(0,-100)   North
    ///   turn     B->C = 20->30 : (0,-100)->(100,-100)  East (90Â° turn at B)
    /// Truck at (3,-100) heading North: the East turn hop is 0m away, the North
    /// incoming hop 3m away â€” but the turn hop is 90Â° off heading.
    #[test]
    fn route_aware_nearest_heading_gate_rejects_misaligned_turn() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20), // incoming North (idx 0)
            seg((0.0, -100.0), (100.0, -100.0), 20, 30), // turn East (idx 1), nearer
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 100.0, -100.0),
        ];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // Truck heading North (0.0), 3m east of B, where the East turn hop is nearer.
        lk.compute_heading_error(3.0, -100.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_route_filtered")
                .as_deref(),
            Some("false"),
            "heading gate must reject the 90Â°-off turn hop â†’ fall back to global"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "must stay on the aligned incoming hop, not snap to the turn hop 20->30"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "aligned incoming hop is on-route â†’ spline engages (no Catmull)"
        );
    }

    // â”€â”€ Phase 2h-Safety: dedicated tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// Helper: Active plugin in AutoReplan stage with a straight north path.
    fn autoreplan_plugin() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("AutoReplan".to_string()),
            ..Default::default()
        }
    }

    /// Test S1: brake value is ramped with speed and clamped correctly.
    ///
    /// - slow speed (4.17 m/s â‰ˆ 15 km/h): 4.17 * 0.072 = 0.300 â†’ â‰ˆ 0.30
    /// - high speed (13.9 m/s â‰ˆ 50 km/h): 13.9 * 0.072 = 1.001 â†’ clamped to SAFETY_BRAKE_MAX (0.80)
    /// - very low speed (1.0 m/s):         1.0  * 0.072 = 0.072 â†’ clamped to SAFETY_BRAKE_MIN (0.15)
    #[test]
    fn safety_brake_ramps_with_speed_and_clamps() {
        // Slow: â‰ˆ 0.30
        let mut lk = autoreplan_plugin();
        let t_slow = make_telemetry(4.17, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk
            .tick_request(Some(&t_slow), &ctx)
            .expect("AutoReplan must emit brake");
        let brake_slow = req.brake.expect("brake must be Some");
        assert!(
            (brake_slow - 0.30).abs() < 0.02,
            "slow speed: expected brake â‰ˆ 0.30, got {brake_slow:.4}"
        );

        // High: should hit SAFETY_BRAKE_MAX
        let mut lk2 = autoreplan_plugin();
        let t_fast = make_telemetry(13.9, 0.0);
        let ctx2 = ctx_with_state("Active");
        let req2 = lk2
            .tick_request(Some(&t_fast), &ctx2)
            .expect("must emit brake");
        let brake_fast = req2.brake.expect("brake must be Some");
        assert_eq!(
            brake_fast, SAFETY_BRAKE_MAX,
            "high speed: brake must be clamped to SAFETY_BRAKE_MAX ({SAFETY_BRAKE_MAX}), got {brake_fast}"
        );

        // Very low: should hit SAFETY_BRAKE_MIN
        let mut lk3 = autoreplan_plugin();
        let t_crawl = make_telemetry(1.0, 0.0);
        let ctx3 = ctx_with_state("Active");
        let req3 = lk3
            .tick_request(Some(&t_crawl), &ctx3)
            .expect("must emit brake");
        let brake_crawl = req3.brake.expect("brake must be Some");
        assert_eq!(
            brake_crawl, SAFETY_BRAKE_MIN,
            "very low speed: brake must be clamped to SAFETY_BRAKE_MIN ({SAFETY_BRAKE_MIN}), got {brake_crawl}"
        );
    }

    /// Test S2: recovery from AutoReplan back to Normal stage produces steering
    /// (not a brake-request), resets safety_autoreplan_secs to 0 and sets
    /// safety_state = "normal".
    #[test]
    fn recovery_from_autoreplan_to_normal_produces_steering_and_resets_accumulator() {
        let mut lk = autoreplan_plugin();
        let t = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");

        // Tick 1: in AutoReplan â€” accumulates time, emits brake.
        let brake_req = lk
            .tick_request(Some(&t), &ctx)
            .expect("must emit brake in AutoReplan");
        assert!(brake_req.brake.is_some(), "AutoReplan tick must have brake");
        // Accumulator must have grown.
        assert!(
            lk.safety_autoreplan_secs > 0.0,
            "accumulator must grow during AutoReplan, got {}",
            lk.safety_autoreplan_secs
        );

        // Tick 2: stage recovers to Normal with heading-aligned waypoints.
        lk.heading_stage = Some("Normal".to_string());
        // Give a heading-aligned straight path to ensure err < 1.4 rad.
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];
        let steer_req = lk
            .tick_request(Some(&t), &ctx)
            .expect("recovery tick must emit a ControlRequest");
        assert!(
            steer_req.steering.is_some(),
            "recovery tick must produce a steering output, not brake"
        );
        assert!(
            steer_req.brake.is_none() || steer_req.brake == Some(0.0),
            "no brake expected after recovery, got {:?}",
            steer_req.brake
        );
        assert_eq!(
            lk.safety_autoreplan_secs, 0.0,
            "safety_autoreplan_secs must reset to 0 on recovery"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("normal"),
            "safety_state must be 'normal' after recovery"
        );
    }

    /// Test S3: AutoReplan accumulates time across ticks; disengage is NOT
    /// requested before 15 s, but IS requested after >15 s.
    ///
    /// dt is capped at 0.1 s inside the function, so we need >150 iterations
    /// to cross the 15 s threshold. We run 151 to guarantee one tick past the
    /// boundary. The blackboard key `autopilot.disengage_requested` must
    /// remain absent (or not "true") before tick 150 and be "true" after tick 151.
    #[test]
    fn autoreplan_timeout_escalates_to_disengage_after_15s() {
        let mut lk = autoreplan_plugin();
        let t = make_telemetry(10.0, 0.0);

        // Each tick uses dt = default 0.02 s, capped to min(0.02, 0.1) = 0.02 inside.
        // We need >15 / 0.02 = 750 ticks. Use with_dt(0.1) so each tick adds 0.1 s
        // and we cross after >150 ticks.
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(0.1);

        // Run 150 ticks â€” must NOT yet have triggered disengage.
        for _ in 0..150 {
            let _ = lk.tick_request(Some(&t), &ctx);
        }
        assert_ne!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "must NOT disengage before 15 s (got {} s accumulated)",
            lk.safety_autoreplan_secs
        );

        // Tick 151 (â‰¥ 15 s) â€” disengage must now be requested.
        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "must disengage after >15 s in AutoReplan (got {} s)",
            lk.safety_autoreplan_secs
        );
    }

    /// Test S4: a single tick in Disengaging stage immediately sets
    /// `autopilot.disengage_requested = "true"` â€” no accumulation needed.
    #[test]
    fn disengaging_escalates_immediately_on_first_tick() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");

        // A single tick must immediately set disengage_requested.
        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "Disengaging must set disengage_requested on the very first tick"
        );
        // safety_autoreplan_secs must NOT have grown (immediate path, no accumulation).
        assert_eq!(
            lk.safety_autoreplan_secs, 0.0,
            "Disengaging must not accumulate into safety_autoreplan_secs (got {})",
            lk.safety_autoreplan_secs
        );
    }

    // â”€â”€ Schritt 2: Junction-Failsafe (kein ungebremstes Geradeaus) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// Schritt-2-1: An einer erkannten Junction lÃ¶st die Disengaging-Stage NICHT
    /// sofort disengage aus, sondern bremst hart (Grace) â€” kein Steering-Vakuum mit
    /// 0.0-Geradeaus, sondern Tempo raus.
    #[test]
    fn junction_failsafe_brakes_instead_of_immediate_disengage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");
        ctx.blackboard
            .set("lane_follower.junction_detected", "true");

        let req = lk.tick_request(Some(&t), &ctx);

        assert_ne!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "junction failsafe must NOT disengage on the first tick"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.junction_failsafe_active")
                .as_deref(),
            Some("true"),
            "junction_failsafe_active must report the grace"
        );
        let req = req.expect("failsafe must still emit a (brake) ControlRequest");
        assert!(req.steering.is_none(), "no steering during failsafe");
        assert!(
            req.brake.unwrap_or(0.0) >= JUNCTION_FAILSAFE_BRAKE,
            "must brake hard during grace, got {:?}",
            req.brake
        );
    }

    /// Schritt-2-2: Nach Ablauf des Grace-Fensters wird doch disengaged (gebremst,
    /// nicht ewig hÃ¤ngend).
    #[test]
    fn junction_failsafe_disengages_after_grace_expires() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(10.0, 0.0);
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("lane_follower.junction_detected", "true");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(0.1);

        // dt=0.1 â†’ ~16 Ticks bis > 1.5 s Grace; 25 Ticks mit Reserve.
        for _ in 0..25 {
            let _ = lk.tick_request(Some(&t), &ctx);
        }
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "after the grace window the failsafe must disengage (secs={})",
            lk.junction_failsafe_secs
        );
    }

    /// Schritt-2-3 (Regression): OHNE Junction-Signal bleibt der sofortige Disengage
    /// unverÃ¤ndert (User-/echtes-Off-Route-Disengage funktioniert weiter).
    #[test]
    fn no_junction_still_disengages_immediately() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");
        // lane_follower.junction_detected NICHT gesetzt.

        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "without junction signal, Disengaging must disengage immediately as before"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.junction_failsafe_active")
                .as_deref(),
            Some("false")
        );
    }

    /// Test S6: waypoints cleared mid-AutoReplan resets the accumulator and
    /// safety_state back to "normal" (the no_waypoints early-return path).
    ///
    /// Also serves as a regression guard for the engine_off path (analogous
    /// reset): if the no_waypoints path resets, the pattern is symmetric.
    #[test]
    fn no_waypoints_resets_autoreplan_accumulator() {
        let mut lk = autoreplan_plugin();
        let t = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");

        // Accumulate some AutoReplan time.
        let _ = lk.tick_request(Some(&t), &ctx);
        assert!(
            lk.safety_autoreplan_secs > 0.0,
            "precondition: accumulator must have grown"
        );

        // Clear waypoints â€” triggers the no_waypoints early-return.
        lk.waypoints.clear();
        let _ = lk.tick_request(Some(&t), &ctx);

        assert_eq!(
            lk.safety_autoreplan_secs, 0.0,
            "no_waypoints path must reset safety_autoreplan_secs to 0"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("normal"),
            "no_waypoints path must write safety_state = 'normal'"
        );
    }

    /// Test S6b: engine_off early-return also resets safety_autoreplan_secs
    /// and writes safety_state = "normal".
    #[test]
    fn engine_off_resets_autoreplan_accumulator() {
        let mut lk = autoreplan_plugin();
        let t_running = make_telemetry(10.0, 0.0);
        let ctx = ctx_with_state("Active");

        // Accumulate some AutoReplan time.
        let _ = lk.tick_request(Some(&t_running), &ctx);
        assert!(
            lk.safety_autoreplan_secs > 0.0,
            "precondition: accumulator must have grown"
        );

        // Engine off â€” triggers the engine_off early-return.
        let mut t_off = make_telemetry(10.0, 0.0);
        t_off.engine_rpm = 0.0;
        let _ = lk.tick_request(Some(&t_off), &ctx);

        assert_eq!(
            lk.safety_autoreplan_secs, 0.0,
            "engine_off path must reset safety_autoreplan_secs to 0"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.safety_state").as_deref(),
            Some("normal"),
            "engine_off path must write safety_state = 'normal'"
        );
    }

    // â”€â”€ Phase 2h-Wurzelfix: Kink-Stop-Tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // Geometry convention: all segments run in the XZ-plane (y=0).
    // The kink is measured at the hop boundary between seg0 (cur) and seg1 (ni):
    //   kink_deg = |heading(seg0.m1) - heading(seg1.m0)| in degrees
    //
    // For a Hermite segment, evaluate_tangent(seg, 0.0) == seg.m0
    //                    and evaluate_tangent(seg, 1.0) == seg.m1.
    //
    // Setup: seg0 = short (~20 m, North); seg1 = long (200 m, some direction).
    // Truck at the near end of seg0 (z â‰ˆ -2) heading North; speed = 20 m/s so
    // look_ahead = 5 + 20*3.6*0.5 = 41 m > 20 m (seg0 length) â†’ arc-walk MUST
    // attempt the seg0â†’seg1 hop.

    /// Build a Hermite segment with explicit m0 / m1 tangent vectors (not chord).
    ///
    /// `p0`/`p1` are (x, z) in ETS2 XZ.  `m0_xz`/`m1_xz` are the entry and exit
    /// tangent directions (scaled so that length_m matches the chord, which is
    /// sufficient for `build_lut` to produce a reasonable arc-length table).
    fn seg_custom(
        p0: (f32, f32),
        p1: (f32, f32),
        m0_xz: (f32, f32),
        m1_xz: (f32, f32),
        from: u64,
        to: u64,
    ) -> HermiteSegment {
        let a = Vec3::new(p0.0, 0.0, p0.1);
        let b = Vec3::new(p1.0, 0.0, p1.1);
        let chord = b - a;
        let scale = chord.length(); // keep tangent magnitude comparable to chord
        let m0 = Vec3::new(m0_xz.0 * scale, 0.0, m0_xz.1 * scale);
        let m1 = Vec3::new(m1_xz.0 * scale, 0.0, m1_xz.1 * scale);
        HermiteSegment {
            p0: a,
            p1: b,
            m0,
            m1,
            length_m: chord.length(),
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    /// Build a wired plugin for a two-segment kink scenario.
    ///
    /// seg0: straight North (0,0)â†’(0,-20), m0=m1=(0,-1) [North unit dir].
    /// seg1: starts at (0,-20), m0 given by `ni_m0_xz` (controls kink), ends far South.
    /// Truck placed at (0, -2) heading North; speed_ms passed to compute_heading_error.
    ///
    /// Returns `(plugin, ctx)` ready for `compute_heading_error(0.0, -2.0, 0.0, speed_ms, &ctx)`.
    fn kink_plugin(ni_m0_xz: (f32, f32), ni_p1: (f32, f32)) -> (LaneKeeperPlugin, PluginContext) {
        // seg0: North, 20 m, from 10â†’20.  m0=m1=(0,-1) unit-vector â†’ scaled by length 20.
        let seg0 = seg_custom((0.0, 0.0), (0.0, -20.0), (0.0, -1.0), (0.0, -1.0), 10, 20);
        // seg1: from (0,-20) to ni_p1, with the given entry tangent and chord exit.
        let chord1_x = ni_p1.0;
        let chord1_z = ni_p1.1 - (-20.0);
        let len1 = (chord1_x * chord1_x + chord1_z * chord1_z).sqrt();
        let m1_xz = if len1 > 1e-6 {
            (chord1_x / len1, chord1_z / len1)
        } else {
            (0.0, -1.0)
        };
        let seg1 = seg_custom((0.0, -20.0), ni_p1, ni_m0_xz, m1_xz, 20, 30);

        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -20.0),
            (30u64, ni_p1.0 as f64, ni_p1.1 as f64),
        ];
        let edges = vec![(10u64, 20u64, 20.0), (20u64, 30u64, len1 as f64)];
        wired_plugin(vec![seg0, seg1], metas, nodes, edges, "[10,20,30]", 2)
    }

    /// Helper: read `walk_stopped_at_kink` blackboard key.
    fn walk_stopped(ctx: &PluginContext) -> bool {
        ctx.blackboard
            .get("lane_keeper.walk_stopped_at_kink")
            .as_deref()
            == Some("true")
    }

    /// Helper: read `lookahead_final_seg_id` blackboard key (0-based index).
    fn final_seg_id(ctx: &PluginContext) -> Option<usize> {
        ctx.blackboard
            .get("lane_keeper.lookahead_final_seg_id")
            .and_then(|s| s.parse().ok())
    }

    /// Test K1: two collinear (0Â°-kink) hops â†’ walk runs through, no kink stop.
    ///
    /// seg0 exits North; seg1 enters North â†’ Î”heading = 0Â° < 35Â° â†’ walk hops to seg1.
    /// Expected: walk_stopped_at_kink=false, final_seg=1 (the second segment),
    ///           heading error small.
    #[test]
    fn kink_walk_straight_no_stop() {
        // Both segments head North: m1 of seg0 = (0,-1), m0 of seg1 = (0,-1).
        let (mut lk, ctx) = kink_plugin((0.0, -1.0), (0.0, -220.0));

        // speed=20 m/s â†’ look_ahead=41 m > 20 m seg0 â†’ must hop.
        let err = lk.compute_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            !walk_stopped(&ctx),
            "0Â°-kink: walk must NOT stop at kink (walk_stopped_at_kink=false)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must have engaged"
        );
        // The lookahead landed on seg1 (index 1), not seg0 (index 0).
        let fseg = final_seg_id(&ctx).expect("lookahead_final_seg_id must be set");
        assert_eq!(
            fseg, 1,
            "0Â°-kink: lookahead must land on seg1 (index 1), not seg0 (index 0); got {fseg}"
        );
        // Heading error: cross-track adds ~0.14 rad (truck on centerline, 2-lane offset=5.625m).
        // Primary intent: walk hops through (no false kink-stop), not error magnitude.
        assert!(
            err.abs() < 0.4,
            "0Â°-kink: heading error must be modest (no extreme overshoot), got {err:.4} rad"
        );
    }

    /// Test K2: mild 20Â°-kink (< 35Â° default threshold) â†’ walk still hops through.
    ///
    /// seg1 enters at 20Â° CW from North; Î”heading â‰ˆ 20Â° < 35Â° â†’ no stop.
    #[test]
    fn kink_walk_mild_curve_no_stop() {
        // 20Â° CW from North: x=sin(20Â°), z=-cos(20Â°)
        let kink_rad = 20.0f32.to_radians();
        let ni_m0 = (kink_rad.sin(), -kink_rad.cos());
        // seg1 heads ~20Â° SE
        let ni_p1 = (200.0 * kink_rad.sin(), -20.0 - 200.0 * kink_rad.cos());
        let (mut lk, ctx) = kink_plugin(ni_m0, ni_p1);

        lk.compute_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            !walk_stopped(&ctx),
            "20Â°-kink (<35Â° threshold): walk must NOT stop at kink"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must remain active"
        );
        let fseg = final_seg_id(&ctx).expect("lookahead_final_seg_id must be set");
        assert_eq!(
            fseg, 1,
            "20Â°-kink: lookahead must hop to seg1 (index 1); got {fseg}"
        );
    }

    /// Test K3: sharp 66Â°-kink (> 35Â° default threshold) â†’ walk stops before the hop.
    ///
    /// seg1 enters at 66Â° CW from North. The kink check fires â†’ break (cur=seg0, t=1.0).
    /// Expected: walk_stopped_at_kink=true, final_seg=0 (seg0, before the kink),
    ///           heading error small (target at end of seg0 = straight North).
    #[test]
    fn kink_walk_sharp_turn_stops() {
        // 66Â° CW from North: x=sin(66Â°), z=-cos(66Â°)
        let kink_rad = 66.0f32.to_radians();
        let ni_m0 = (kink_rad.sin(), -kink_rad.cos());
        let ni_p1 = (200.0 * kink_rad.sin(), -20.0 - 200.0 * kink_rad.cos());
        let (mut lk, ctx) = kink_plugin(ni_m0, ni_p1);

        let err = lk.compute_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            walk_stopped(&ctx),
            "66Â°-kink (>35Â° threshold): walk must stop at kink (walk_stopped_at_kink=true)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "kink-stop still returns a spline result (not Catmull fallback)"
        );
        // final_seg must be seg0 (index 0) â€” the segment BEFORE the kink.
        let fseg = final_seg_id(&ctx).expect("lookahead_final_seg_id must be set");
        assert_eq!(
            fseg, 0,
            "66Â°-kink: kink-stop must target the end of seg0 (index 0), not hop to seg1; got {fseg}"
        );
        // The walk broke at t=1.0 on seg0 (= North end of seg0); truck heads North.
        // Target is the lane-offset-shifted end of seg0 â†’ heading error < ~0.5 rad.
        // (The lane offset shifts the target ~3.75 m east of the North end; at ~18 m
        // distance the atan gives â‰ˆ 0.20â€“0.30 rad, well within the 0.5 rad margin.)
        assert!(
            err.abs() < 0.5,
            "66Â°-kink: target at end of seg0 is ahead â†’ heading error < 0.5 rad, got {err:.4} rad"
        );
    }

    /// Test K4: threshold override via Blackboard key `plugin.lane_keeper.kink_stop_deg`.
    ///
    /// Set threshold to 20Â°; Î”heading â‰ˆ 25Â° (would NOT stop with default 35Â°) â†’ now stops.
    #[test]
    fn kink_walk_threshold_from_blackboard() {
        // 25Â° kink: just above the custom 20Â° threshold but below the default 35Â°.
        let kink_rad = 25.0f32.to_radians();
        let ni_m0 = (kink_rad.sin(), -kink_rad.cos());
        let ni_p1 = (200.0 * kink_rad.sin(), -20.0 - 200.0 * kink_rad.cos());
        let (mut lk, ctx) = kink_plugin(ni_m0, ni_p1);

        // Override: lower threshold to 20Â° so 25Â° now triggers a kink stop.
        ctx.blackboard
            .set("plugin.lane_keeper.kink_stop_deg", "20.0");

        lk.compute_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        // With default 35Â°, this would NOT stop; with 20Â° override it MUST stop.
        assert!(
            walk_stopped(&ctx),
            "25Â°-kink with override threshold=20Â°: walk must stop (kink_stop_deg override not applied?)"
        );
        let fseg = final_seg_id(&ctx).expect("lookahead_final_seg_id must be set");
        assert_eq!(
            fseg, 0,
            "override-threshold kink: lookahead must land on seg0 (before kink); got {fseg}"
        );
        // Verify the threshold was actually read from blackboard (Diag key).
        let thr = ctx
            .blackboard
            .get("lane_keeper.kink_threshold_deg")
            .and_then(|s| s.parse::<f32>().ok())
            .expect("kink_threshold_deg must be set");
        assert!(
            (thr - 20.0).abs() < 0.5,
            "kink_threshold_deg must reflect the blackboard override (20Â°), got {thr}"
        );
    }

    /// Test K5: a persistent kink-stop on the SAME hop accumulates `kink_stuck_secs`
    /// across ticks (the post-loop reset is guarded â€” it only fires on a clean walk,
    /// not after a kink-stop break). Once the accumulator exceeds `KINK_STUCK_FALLBACK_S`
    /// (4.0 s), `try_spline_heading_error` returns `None` with `fallback_reason="kink_stuck"`,
    /// handing navigation to the Catmull path (which rounds the corner). Dead-lock guard.
    ///
    ///   - dt = 0.1 s â†’ tick n accumulates â‰ˆ nÂ·0.1 s.
    ///   - ticks 1..=39 (â‰¤ 3.9 s): kink-stop returns Some (target on cur), accumulating.
    ///   - tick 40: forty IEEE754 0.1-adds sum to 4.0000000000000036 > 4.0 â†’ None,
    ///     fallback_reason="kink_stuck". (FP overshoot is deterministic across platforms.)
    #[test]
    fn kink_walk_stuck_falls_back_to_catmull() {
        // 66Â°-kink: reliably above the 35Â° threshold.
        let kink_rad = 66.0f32.to_radians();
        let ni_m0 = (kink_rad.sin(), -kink_rad.cos());
        let ni_p1 = (200.0 * kink_rad.sin(), -20.0 - 200.0 * kink_rad.cos());
        let (mut lk, ctx) = kink_plugin(ni_m0, ni_p1);

        let bb = ctx.blackboard.clone();
        let idx = ctx
            .spline_index
            .clone()
            .expect("spline_index must be wired");
        let rg = ctx.graph.clone().expect("router_graph must be wired");
        let mut new_ctx = PluginContext::new("lane-keeper", bb)
            .with_spline_index(idx, 2)
            .with_dt(0.1);
        new_ctx.graph = Some(rg);

        // ticks 1..=39 (â‰¤ 3.9 s): kink-stop returns Some, accumulator builds.
        for i in 1..=39 {
            let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &new_ctx);
            assert!(
                result.is_some(),
                "tick {i}: below the 4.0 s stuck window the walk still kink-stops (Some)"
            );
            let expected = i as f64 * 0.1;
            assert!(
                (lk.kink_stuck_secs - expected).abs() < 1e-6,
                "tick {i}: kink_stuck_secs must accumulate to {expected:.2}, got {:.2}",
                lk.kink_stuck_secs
            );
            assert_ne!(
                new_ctx
                    .blackboard
                    .get("lane_keeper.fallback_reason")
                    .as_deref(),
                Some("kink_stuck"),
                "tick {i}: kink_stuck fallback must NOT fire before 4.0 s"
            );
        }

        // tick 40: accumulator crosses 4.0 s â†’ Catmull fallback (None + reason).
        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &new_ctx);
        assert!(
            result.is_none(),
            "tick 40: stuck > 4.0 s must hand off to Catmull (None)"
        );
        assert_eq!(
            new_ctx
                .blackboard
                .get("lane_keeper.fallback_reason")
                .as_deref(),
            Some("kink_stuck"),
            "tick 40: fallback_reason must be kink_stuck"
        );
    }

    /// Test K6: the post-loop reset (`kink_stuck_secs = 0.0`) fires both on clean walks
    /// AND on kink-stop breaks. This test confirms the clean-walk case specifically.
    ///
    /// If `kink_stuck_secs` and `kink_stuck_hop` are non-zero entering `try_spline_heading_error`
    /// and the walk completes without a kink-stop (clean walk), both fields must be 0/(0,0)
    /// after the call â€” the reset at line 804 fires.
    #[test]
    fn kink_stuck_resets_on_clean_walk() {
        // Build a straight-scenario plugin (0Â°-kink, walk always succeeds without stop).
        let (mut lk_straight, ctx_straight) = kink_plugin((0.0, -1.0), (0.0, -220.0));

        // Pre-load non-zero kink_stuck state as if a previous kink-stop had set it
        // (e.g. on a different route before transitioning to this straight road).
        lk_straight.kink_stuck_secs = 2.5;
        lk_straight.kink_stuck_hop = (20, 30);

        let bb = ctx_straight.blackboard.clone();
        let idx = ctx_straight.spline_index.clone().unwrap();
        let rg = ctx_straight.graph.clone().unwrap();
        let mut ctx = PluginContext::new("lane-keeper", bb)
            .with_spline_index(idx, 2)
            .with_dt(0.1);
        ctx.graph = Some(rg);

        // One clean-walk tick (0Â°-kink, walk hops to seg1 without any kink-stop break).
        let result = lk_straight.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);
        assert!(result.is_some(), "clean walk must return Some");
        assert!(
            !walk_stopped(&ctx),
            "clean walk must NOT set walk_stopped_at_kink"
        );

        // Post-loop reset must have fired: both fields cleared.
        assert_eq!(
            lk_straight.kink_stuck_secs, 0.0,
            "clean walk must reset kink_stuck_secs to 0.0 (post-loop reset); got {:.2}",
            lk_straight.kink_stuck_secs
        );
        assert_eq!(
            lk_straight.kink_stuck_hop,
            (0, 0),
            "clean walk must reset kink_stuck_hop to (0,0)"
        );
    }

    // â”€â”€ Alternative A: Prefab-/Curve-Fallback-Tests (intK ab Truck-Position) â”€â”€â”€â”€â”€â”€
    //
    // Geometry: EIN Kurvensegment, auf dessen ANFANG der Truck sitzt.
    //
    //   seg0: Kurve (oder Road, je nach Test), 20 m, uid 10â†’20.
    //         p0 = (0,-2) == Truck-Position â†’ Projektion t_cur â‰ˆ 0.
    //         m0 = North (0,-1) [echte Einfahrt, KEINE m0-Anomalie].
    //         m1 = exit_deg_cwÂ° CW von North â†’ interner Knick â‰ˆ exit_deg_cw.
    //
    // Truck at p0=(0,-2) heading North, speed = 20 (bzw. 40) m/s.
    //   look_ahead = 5 + v*3.6*0.5 â‰¥ 41 m > seg-LÃ¤nge (20 m), und es gibt KEINEN
    //   weiteren Forward-Hop (route=[10,20]) â†’ der Walk klemmt bei (seg0, 1.0).
    //   â†’ final_seg == cur_seg == seg0  (intk_on_truck_seg = true).
    //   â†’ t_start_intk = t_cur â‰ˆ 0 â†’ final_internal_kink_deg == exit_deg_cw EXAKT
    //     (Hermite: P'(0)=m0=North, P'(1)=m1=exit).
    //
    // Wichtig (Alternative A): weil final_seg == cur_seg, lÃ¤uft die intK-Messung Ã¼ber das
    // Truck-Segment â€” genau der Pfad, den der Fix einfÃ¼hrt. Die Latch-State-Machine
    // (Cap/Hysterese/Schwelle) wird so prÃ¤zise getestet; die eigentliche t_cur>0-Anomalie-
    // UnterdrÃ¼ckung prÃ¼fen die intk_from_truck_*-Tests separat.

    /// Build a curve test fixture (single curve segment, truck at its start).
    ///
    /// `exit_deg_cw` â€” internal kink of the curve: exit tangent rotated this many Â° CW from North.
    /// `seg_is_prefab` â€” whether the curve segment carries `is_prefab=true` metadata.
    /// `custom_threshold` â€” if Some, set `plugin.lane_keeper.prefab_curve_fallback_deg` on the BB.
    ///
    /// Returns `(plugin, ctx)` ready for `try_spline_heading_error(0.0, -2.0, 0.0, v, &ctx)`.
    fn prefab_curve_plugin(
        exit_deg_cw: f32,
        seg_is_prefab: bool,
        custom_threshold: Option<f64>,
    ) -> (LaneKeeperPlugin, PluginContext) {
        // Alternative A: EIN Kurvensegment, der Truck sitzt an seinem ANFANG (p0).
        // m0 = North (echte Einfahrt, KEINE m0-Anomalie), m1 = exit_deg_cwÂ° CW.
        let exit_rad = exit_deg_cw.to_radians();
        let m1_x = exit_rad.sin();
        let m1_z = -exit_rad.cos();
        let p0 = (0.0f32, -2.0f32); // == Truck-Position der Aufrufer â†’ t_cur â‰ˆ 0
        let p1 = (p0.0 + 20.0 * m1_x, p0.1 + 20.0 * m1_z); // p1 folgt m1 fÃ¼r 20 m
        let seg0 = seg_custom(p0, p1, (0.0, -1.0), (m1_x, m1_z), 10, 20);

        let metas = vec![Some(road_meta(2, 3.75, seg_is_prefab))];
        let nodes = vec![
            (10u64, p0.0 as f64, p0.1 as f64),
            (20u64, p1.0 as f64, p1.1 as f64),
        ];
        let edges = vec![(10u64, 20u64, 20.0)];

        let (mut lk, ctx) = wired_plugin(vec![seg0], metas, nodes, edges, "[10,20]", 1);

        if let Some(thr) = custom_threshold {
            ctx.blackboard.set(
                "plugin.lane_keeper.prefab_curve_fallback_deg",
                thr.to_string(),
            );
        }

        // Catmull-StÃ¼tzpunkte fÃ¼r den Fallback-Pfad (falls None zurÃ¼ckkommt).
        lk.waypoints = vec![
            [p0.0 as f64, p0.1 as f64],
            [((p0.0 + p1.0) * 0.5) as f64, ((p0.1 + p1.1) * 0.5) as f64],
            [p1.0 as f64, p1.1 as f64],
        ];

        (lk, ctx)
    }

    /// Helper: read `prefab_curve_fallback` blackboard key (the latch bool).
    fn prefab_curve_latched_key(ctx: &PluginContext) -> bool {
        ctx.blackboard
            .get("lane_keeper.prefab_curve_fallback")
            .as_deref()
            == Some("true")
    }

    /// Helper: read `internal_kink_over_threshold` blackboard key.
    fn kink_over_threshold(ctx: &PluginContext) -> bool {
        ctx.blackboard
            .get("lane_keeper.internal_kink_over_threshold")
            .as_deref()
            == Some("true")
    }

    /// Helper: read `prefab_curve_threshold_deg` blackboard key.
    fn prefab_threshold_deg(ctx: &PluginContext) -> Option<f32> {
        ctx.blackboard
            .get("lane_keeper.prefab_curve_threshold_deg")
            .and_then(|s| s.parse().ok())
    }

    /// Test PC1: Prefab segment with internal kink ~66Â° (> 40Â° default threshold).
    ///
    /// Expected: `try_spline_heading_error` returns `None`, `fallback_reason="prefab_curve"`,
    /// `prefab_curve_latched=true`, `prefab_curve_fallback="true"`,
    /// `internal_kink_over_threshold="true"`.
    ///
    /// This is the Diag5 scenario: walk lands on a prefab curve segment whose heading
    /// rotates 66Â° internally, causing herr-spike â†’ lane-keeper now correctly falls back
    /// to Catmull-Rom.
    #[test]
    fn prefab_curve_over_threshold_falls_back() {
        let (mut lk, ctx) = prefab_curve_plugin(66.0, true, None);

        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            result.is_none(),
            "Prefab seg with 66Â° internal kink must return None (Catmull fallback)"
        );
        assert!(
            lk.prefab_curve_latched,
            "prefab_curve_latched field must be true after 66Â° internal kink on prefab"
        );
        assert!(
            prefab_curve_latched_key(&ctx),
            "prefab_curve_fallback BB key must be 'true'"
        );
        assert!(
            kink_over_threshold(&ctx),
            "internal_kink_over_threshold must be 'true' (66Â° > 40Â°)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "fallback_reason must be 'prefab_curve'"
        );
    }

    /// Test PC2 (THE GAP â€” critical): Prefab segment with internal kink ~35Â° (< 40Â° threshold).
    ///
    /// Expected: NO fallback â€” `try_spline_heading_error` returns `Some`, `prefab_curve_latched=false`,
    /// `prefab_curve_fallback="false"`.
    ///
    /// This test prevents fahrbare (drivable) Prefab segments from being incorrectly
    /// sent to Catmull-Rom. The threshold must only block genuinely curved segments
    /// (â‰¥ 40Â°), not mildly curved ones (< 40Â°).
    #[test]
    fn prefab_curve_under_threshold_no_fallback() {
        let (mut lk, ctx) = prefab_curve_plugin(35.0, true, None);

        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            result.is_some(),
            "Prefab seg with 35Â° internal kink (< 40Â° threshold) must return Some (no Catmull fallback)"
        );
        assert!(
            !lk.prefab_curve_latched,
            "prefab_curve_latched must be false for 35Â° internal kink (below threshold)"
        );
        assert!(
            !prefab_curve_latched_key(&ctx),
            "prefab_curve_fallback BB key must be 'false'"
        );
        assert!(
            !kink_over_threshold(&ctx),
            "internal_kink_over_threshold must be 'false' (35Â° < 40Â°)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "fallback_reason must be 'none' (spline path active)"
        );
    }

    /// Test PC3: Road segment (is_prefab=false) with internal kink > 40Â°.
    ///
    /// Expected: NO prefab_curve fallback â€” the prefab-curve guard is prefab-only.
    /// The spline path remains active (returns Some). Confirms that road segments
    /// with gentle internal curves (e.g. sweeping motorway arcs) are not incorrectly
    /// forced to Catmull-Rom.
    #[test]
    fn road_segment_internal_curve_also_falls_back() {
        // Phase 2h v2-Fix: der AuslÃ¶ser ist die interne KrÃ¼mmung ALLEIN, unabhÃ¤ngig vom
        // is_prefab-Flag. Ein ROAD-Segment (is_prefab=false) mit 66Â° interner KrÃ¼mmung muss
        // jetzt EBENFALLS auf Catmull fallbacken (das Spike-Segment 1051105 ist ein Road-Edge).
        let (mut lk, ctx) = prefab_curve_plugin(66.0, false, None);

        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            result.is_none(),
            "Road seg with 66Â° internal kink must trigger the curve fallback (returns None)"
        );
        assert!(
            lk.prefab_curve_latched,
            "curve fallback must latch on a road segment too (is_prefab no longer gates)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "road segment with high internal curvature: fallback_reason must be 'prefab_curve'"
        );
        assert!(
            kink_over_threshold(&ctx),
            "internal_kink_over_threshold must be 'true' for 66Â° regardless of is_prefab"
        );
    }

    /// Test PC4: Hysteresis â€” latch activates at ~50Â°, remains latched at ~35Â° (between
    /// exit threshold of 30Â° and entry threshold of 40Â°), only releases below 30Â°.
    ///
    /// Sequence:
    ///   tick 1 â€” 50Â° prefab (> 40Â°)    â†’ latch ON  (curve_over=true)
    ///   tick 2 â€” 35Â° prefab (30Â°â€“40Â°)  â†’ latch STAYS ON (exit requires < 30Â°)
    ///   tick 3 â€” 25Â° prefab (< 30Â°)    â†’ latch OFF  (exit threshold crossed)
    ///
    /// This tests the `PREFAB_CURVE_EXIT_MARGIN_DEG = 10.0` hysteresis window.
    /// Note: each tick uses a different plugin fixture with the appropriate internal kink.
    /// The latch field is transferred manually between ticks to simulate the tick-over-tick
    /// persistence.
    #[test]
    fn prefab_curve_hysteresis() {
        // Tick 1: 50Â°, prefab â†’ latch ON.
        let (mut lk_50, ctx_50) = prefab_curve_plugin(50.0, true, None);
        let r1 = lk_50.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx_50);
        assert!(r1.is_none(), "tick1 50Â°: must fall back (latch ON)");
        assert!(lk_50.prefab_curve_latched, "tick1: latch must be ON");

        // Tick 2: 35Â° prefab â€” between 30Â° and 40Â° â†’ still within hysteresis band â†’ stays latched.
        // Transfer latch state to a new 35Â°-fixture.
        let (mut lk_35, ctx_35) = prefab_curve_plugin(35.0, true, None);
        lk_35.prefab_curve_latched = true; // carry latch from tick 1
        let r2 = lk_35.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx_35);
        assert!(
            r2.is_none(),
            "tick2 35Â° (still latched): hysteresis band 30Â°â€“40Â° must keep latch ON (returns None)"
        );
        assert!(
            lk_35.prefab_curve_latched,
            "tick2 35Â°: latch must remain true inside hysteresis window"
        );

        // Tick 3: 25Â° prefab â€” below exit threshold (40Â° âˆ’ 10Â° = 30Â°) â†’ latch OFF.
        let (mut lk_25, ctx_25) = prefab_curve_plugin(25.0, true, None);
        lk_25.prefab_curve_latched = true; // carry latch from tick 2
        let r3 = lk_25.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx_25);
        assert!(
            r3.is_some(),
            "tick3 25Â° (< 30Â° exit threshold): latch must release â†’ returns Some"
        );
        assert!(
            !lk_25.prefab_curve_latched,
            "tick3 25Â°: latch must be OFF after exit threshold crossed"
        );
    }

    /// Test PC5: Custom threshold via Blackboard.
    ///
    /// `plugin.lane_keeper.prefab_curve_fallback_deg = 20.0` is set.
    /// Prefab segment internal kink ~25Â° â†’ would NOT fall back with default 40Â°, but
    /// MUST fall back with the custom 20Â° threshold.
    /// Also verifies `prefab_curve_threshold_deg` BB key reflects the override.
    #[test]
    fn prefab_curve_blackboard_threshold() {
        // 25Â° internal kink: above custom 20Â°, below default 40Â°.
        let (mut lk, ctx) = prefab_curve_plugin(25.0, true, Some(20.0));

        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        assert!(
            result.is_none(),
            "25Â° prefab with custom threshold 20Â°: must fall back (None)"
        );
        assert!(
            lk.prefab_curve_latched,
            "prefab_curve_latched must be true (25Â° > custom 20Â° threshold)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "fallback_reason must be 'prefab_curve' with custom threshold"
        );

        let thr = prefab_threshold_deg(&ctx).expect("prefab_curve_threshold_deg must be set");
        assert!(
            (thr - 20.0).abs() < 0.5,
            "prefab_curve_threshold_deg must reflect the BB override (20Â°), got {thr:.1}"
        );
    }

    /// Test PC6: Degenerate tangent â†’ `final_internal_kink_deg = -1.0` â†’ NO latch.
    ///
    /// A segment whose m0 tangent has near-zero magnitude produces a degenerate
    /// internal kink calculation (returns -1.0). The guard `final_internal_kink_deg >= 0.0`
    /// in `curve_over` prevents the latch from activating. Degeneracy must NOT cause
    /// a spurious Catmull fallback.
    ///
    /// Construction (Alternative A): single segment whose m0 â‰ˆ (0,0,0). The truck sits at
    /// p0=(0,-2) â†’ t_curâ‰ˆ0 â†’ evaluate_tangent at t_startâ‰ˆ0 == m0 â‰ˆ 0 â†’ f_l0 < 1e-6 â†’
    /// final_internal_kink_deg = -1.0 â†’ guard blocks the latch.
    #[test]
    fn prefab_curve_degenerate_no_fallback() {
        // Single prefab curve at the truck's start with degenerate m0 (zero tangent at t=0).
        let seg0 = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, -2.0),
            p1: Vec3::new(0.0, 0.0, -22.0),
            m0: Vec3::new(0.0, 0.0, 0.0), // degenerate: zero tangent at t=0
            m1: Vec3::new(0.0, 0.0, -20.0),
            length_m: 20.0,
            from_uid: 10,
            to_uid: 20,
            edge_uid: 1020,
        };
        let metas = vec![Some(road_meta(2, 3.75, true))]; // prefab
        let nodes = vec![(10u64, 0.0, -2.0), (20u64, 0.0, -22.0)];
        let edges = vec![(10u64, 20u64, 20.0)];
        let (mut lk, ctx) = wired_plugin(vec![seg0], metas, nodes, edges, "[10,20]", 1);

        let _result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 20.0, &ctx);

        // The degenerate tangent should produce final_internal_kink_deg = -1.0,
        // which the guard `final_internal_kink_deg >= 0.0` blocks â†’ no latch.
        assert!(
            !lk.prefab_curve_latched,
            "degenerate tangent (intK=-1.0) must NOT set prefab_curve_latched"
        );
        assert!(
            !kink_over_threshold(&ctx),
            "internal_kink_over_threshold must be false when tangent is degenerate"
        );
        // The function may return Some or None (degenerate tangent could also trip the
        // `degenerate_tangent` fallback at the lookahead-evaluation step if final_t
        // evaluation returns a zero tangent there â€” that is also correct behaviour).
        // What must NOT happen: fallback_reason = "prefab_curve".
        assert_ne!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "degenerate tangent must NOT produce a prefab_curve fallback"
        );
        // Verify the BB key is set (even if result is None for another reason).
        let intk: f32 = ctx
            .blackboard
            .get("lane_keeper.final_internal_kink_deg")
            .and_then(|s| s.parse().ok())
            .expect("final_internal_kink_deg must be set");
        assert!(
            intk < 0.0,
            "degenerate tangent must produce final_internal_kink_deg < 0, got {intk:.4}"
        );
    }

    // â”€â”€ Alternative A: intK ab Truck-Position statt ab t=0 â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // Kern des Fixes: final_internal_kink_deg wird Ã¼ber [t_truck, final_t] gemessen
    // (Start-Tangens = evaluate_tangent(final_seg, t_cur)) statt Ã¼ber [0, final_t]
    // (== m0, dem mis-orientierten Junction-Quaternion-Forward am Segment-ANFANG).

    /// Read a float blackboard key (helper for the Alternative-A tests).
    fn bb_f32(ctx: &PluginContext, key: &str) -> f32 {
        ctx.blackboard
            .get(key)
            .and_then(|s| s.parse().ok())
            .unwrap_or_else(|| panic!("BB key '{key}' must be set & parse"))
    }

    /// AA1: m0-Anomalie am Segment-ANFANG, Truck weiter hinten (t_cur > 0).
    /// Das Segment ist geometrisch gerade (p0â†’p1 North, m1 = North), aber m0 ist ~135Â°
    /// fehlorientiert (Junction-Quaternion-Artefakt). Gemessen ab t=0 ergÃ¤be das ~135Â°;
    /// gemessen ab der realen Truck-Position (tâ‰ˆ0.4) ist die RestkrÃ¼mmung klein â†’ KEIN
    /// Latch, der route-aware-Spline trackt weiter.
    #[test]
    fn intk_from_truck_ignores_start_anomaly() {
        // p0â†’p1 gerade nach North; m0 zeigt ~135Â° CW (fehlorientiert), m1 = North.
        let m0_anom = (135f32.to_radians().sin(), -135f32.to_radians().cos()); // (0.707, 0.707)
        let seg0 = seg_custom((0.0, 0.0), (0.0, -40.0), m0_anom, (0.0, -1.0), 10, 20);
        // Truck an die reale Position bei tâ‰ˆ0.4 setzen (weit hinter der m0-Beule).
        let tp = evaluate(&seg0, 0.4);

        let metas = vec![Some(road_meta(2, 3.75, true))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -40.0)];
        let edges = vec![(10u64, 20u64, 40.0)];
        let (mut lk, ctx) = wired_plugin(vec![seg0], metas, nodes, edges, "[10,20]", 1);

        let _ = lk.try_spline_heading_error(tp.x as f64, tp.z as f64, 0.0, 20.0, &ctx);

        // Messung lief Ã¼ber das Truck-Segment, ab einer echten Position > Segment-Anfang.
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.intk_on_truck_seg")
                .as_deref(),
            Some("true"),
            "final_seg == cur_seg â†’ Messung Ã¼ber das Truck-Segment"
        );
        let t_start = bb_f32(&ctx, "lane_keeper.intk_t_start");
        assert!(
            t_start > 0.1,
            "Start-t muss die reale Truck-Projektion sein (>0.1), got {t_start:.3}"
        );
        let intk = bb_f32(&ctx, "lane_keeper.final_internal_kink_deg");
        assert!(
            (0.0..40.0).contains(&intk),
            "ab Truck-Position fÃ¤llt die m0-Anomalie raus â†’ kleine RestkrÃ¼mmung (<40Â°), got {intk:.2}"
        );
        assert!(
            !lk.prefab_curve_latched,
            "m0-Anomalie am Anfang darf KEINEN Latch auslÃ¶sen"
        );
        assert_ne!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "kein prefab_curve-Fallback bei reiner Start-Anomalie"
        );
    }

    /// AA2: echte Kurve ab Truck-Position. Chord = North, m0 = North (saubere Einfahrt),
    /// m1 = 60Â° CW â€” die KrÃ¼mmung liegt verteilt/spÃ¤t, sodass auch ab tâ‰ˆ0.3 die
    /// RestkrÃ¼mmung bis final_t groÃŸ bleibt (> 40Â°-Eintrittsschwelle) â†’ Latch feuert â†’
    /// Catmull rundet. Belegt, dass die Ab-Truck-Messung echte enge Prefabs NICHT
    /// unterschÃ¤tzt.
    #[test]
    fn intk_from_truck_detects_real_curve() {
        let exit = 60f32.to_radians();
        let (m1x, m1z) = (exit.sin(), -exit.cos());
        // Chord nach North (0,-40); nur die EXIT-Tangente dreht auf 60Â° â†’ die KrÃ¼mmung
        // sitzt nicht am Anfang, sondern wird bis final_t gefahren.
        let seg0 = seg_custom((0.0, 0.0), (0.0, -40.0), (0.0, -1.0), (m1x, m1z), 10, 20);
        let tp = evaluate(&seg0, 0.3);

        let metas = vec![Some(road_meta(2, 3.75, true))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -40.0)];
        let edges = vec![(10u64, 20u64, 40.0)];
        let (mut lk, ctx) = wired_plugin(vec![seg0], metas, nodes, edges, "[10,20]", 1);

        let result = lk.try_spline_heading_error(tp.x as f64, tp.z as f64, 0.0, 20.0, &ctx);

        let t_start = bb_f32(&ctx, "lane_keeper.intk_t_start");
        assert!(
            t_start > 0.1,
            "Truck sitzt mitten auf der Kurve (t>0.1), got {t_start:.3}"
        );
        let intk = bb_f32(&ctx, "lane_keeper.final_internal_kink_deg");
        assert!(
            intk > 40.0,
            "echte Kurve ab Truck-Position muss > 40Â°-Schwelle liefern, got {intk:.2}"
        );
        assert!(
            result.is_none() && lk.prefab_curve_latched,
            "echte Kurve â†’ Latch feuert â†’ Catmull (None)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "echte Kurve erzeugt prefab_curve-Fallback"
        );
    }

    /// AA3: leeres Intervall [t_truck, final_t] (Truck am Segment-ENDE, t_cur == final_t).
    /// Muss intK â‰ˆ 0 liefern (kein Latch) und darf NICHT paniken.
    #[test]
    fn intk_empty_interval_safe() {
        // Gerades Segment; Truck exakt am Endpunkt p1 â†’ t_cur â‰ˆ 1.0, final_t = 1.0.
        let seg0 = seg((0.0, 0.0), (0.0, -20.0), 10, 20);
        let metas = vec![Some(road_meta(2, 3.75, true))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -20.0)];
        let edges = vec![(10u64, 20u64, 20.0)];
        let (mut lk, ctx) = wired_plugin(vec![seg0], metas, nodes, edges, "[10,20]", 1);

        // Truck am Segment-Ende (0,-20), heading North. Darf nicht paniken.
        let _ = lk.try_spline_heading_error(0.0, -20.0, 0.0, 20.0, &ctx);

        let intk = bb_f32(&ctx, "lane_keeper.final_internal_kink_deg");
        assert!(
            intk.abs() < 1.0,
            "leeres/degeneriertes Intervall â†’ intK â‰ˆ 0, got {intk:.4}"
        );
        assert!(
            !lk.prefab_curve_latched,
            "leeres Intervall darf keinen Latch setzen"
        );
    }

    // â”€â”€ Phase 2h-Befund4: PlausibilitÃ¤ts-Cap fÃ¼r den prefab_curve_latch â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // Alle nutzen speed=40 m/s â†’ look_ahead = 5 + 40*3.6*0.5 = 77 m. Mit dem einen
    // Kurvensegment (route=[10,20]) erreicht der Lookahead-Walk das Routenende â†’ klemmt
    // bei (seg0, 1.0). Truck sitzt bei p0 â†’ t_curâ‰ˆ0 â†’ final_internal_kink_deg ==
    // exit-Winkel EXAKT (intk_on_truck_seg = true, Messung Ã¼ber das Truck-Segment).

    /// Test PC8 (Befund4): Latch feuert fÃ¼r intK im plausiblen Band (â‰¤ Cap).
    /// 55Â° ist eine legitime scharfe Kurve (40 < 55 â‰¤ 90) â†’ Latch â†’ Catmull.
    #[test]
    fn latch_fires_in_plausible_range() {
        let (mut lk, ctx) = prefab_curve_plugin(55.0, false, None);
        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 40.0, &ctx);
        assert!(
            result.is_none(),
            "55Â° (plausibles Band) muss latchen â†’ Catmull â†’ None"
        );
        assert!(lk.prefab_curve_latched, "Latch muss bei intK=55Â° feuern");
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.final_internal_kink_capped")
                .as_deref(),
            Some("false"),
            "55Â° liegt unter dem 90Â°-Cap â†’ nicht capped"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "Latch im plausiblen Band erzeugt prefab_curve-Fallback"
        );
    }

    /// Test PC9 (Befund4): Latch UNTERDRÃœCKT fÃ¼r intK Ã¼ber dem Cap (Junction-
    /// Tangenten-Artefakt). 131Â° > 90Â° â†’ kein Latch, capped=true, Spline trackt weiter.
    #[test]
    fn latch_suppressed_above_cap() {
        let (mut lk, ctx) = prefab_curve_plugin(131.0, false, None);
        let result = lk.try_spline_heading_error(0.0, -2.0, 0.0, 40.0, &ctx);
        assert!(
            !lk.prefab_curve_latched,
            "131Â° > 90Â°-Cap â†’ Latch muss unterdrÃ¼ckt werden (Junction-Artefakt)"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.final_internal_kink_capped")
                .as_deref(),
            Some("true"),
            "131Â° > Cap â†’ final_internal_kink_capped=true"
        );
        assert_ne!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("prefab_curve"),
            "capped intK darf KEINEN prefab_curve-Fallback erzeugen"
        );
        assert!(
            result.is_some(),
            "mit unterdrÃ¼cktem Latch engaged der Spline-Pfad (liefert Heading-Error)"
        );
    }

    /// Test PC10 (Befund4, Task 1.3): ein bereits im plausiblen Band (85Â°) aktiver
    /// Latch muss BEENDET werden, wenn intK Ã¼ber den Cap steigt (â†’131Â°), statt im
    /// Artefakt-Regime hÃ¤ngen zu bleiben.
    #[test]
    fn latch_exits_when_crossing_cap() {
        // 131Â°-Geometrie; simuliere den Latch, der zuvor bei 85Â° eingerastet war.
        let (mut lk, ctx) = prefab_curve_plugin(131.0, false, None);
        lk.prefab_curve_latched = true;
        lk.prefab_curve_kink_deg = 85.0;
        let _ = lk.try_spline_heading_error(0.0, -2.0, 0.0, 40.0, &ctx);
        assert!(
            !lk.prefab_curve_latched,
            "intK 85Â°â†’131Â° (Ã¼ber Cap) muss den Latch BEENDEN (Task 1.3)"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.final_internal_kink_capped")
                .as_deref(),
            Some("true"),
            "Over-Cap-Austritt muss capped=true melden"
        );
    }

    /// Test PC11 (Befund4, Hysterese-Totband): intK im Totband [cap, cap+Margin]
    /// = [100, 110]Â°. Ein bereits aktiver Latch HÃ„LT (kein Flacker-Exit); ein
    /// frischer (nicht gelatchter) Zustand tritt NICHT ein (Entry blockiert > cap).
    /// Belegt die Anti-Flacker-Hysterese am oberen Cap.
    #[test]
    fn latch_cap_deadband_holds_state() {
        // 105Â° liegt im Totband (> cap 100, < cap+Margin 110).
        // (a) Bereits gelatcht â†’ bleibt gelatcht (kein Exit).
        let (mut lk_held, ctx_held) = prefab_curve_plugin(105.0, false, None);
        lk_held.prefab_curve_latched = true;
        lk_held.prefab_curve_kink_deg = 85.0;
        let _ = lk_held.try_spline_heading_error(0.0, -2.0, 0.0, 40.0, &ctx_held);
        assert!(
            lk_held.prefab_curve_latched,
            "105Â° im Totband [100,110] â†’ aktiver Latch HÃ„LT (kein Flacker-Exit)"
        );

        // (b) Frisch / nicht gelatcht â†’ KEIN Eintritt (Entry > cap blockiert).
        let (mut lk_fresh, ctx_fresh) = prefab_curve_plugin(105.0, false, None);
        let _ = lk_fresh.try_spline_heading_error(0.0, -2.0, 0.0, 40.0, &ctx_fresh);
        assert!(
            !lk_fresh.prefab_curve_latched,
            "105Â° > cap 100 â†’ frischer Eintritt blockiert (kein neuer Latch)"
        );
    }

    /// Test PC7: Latch resets on disengage (ctx.is_active() = false).
    ///
    /// When `tick_request_route_following` is called with the autopilot in a non-Active
    /// state, `prefab_curve_latched` must be reset to false (Disengage path).
    ///
    /// Scenario: manually set `prefab_curve_latched = true`, then call
    /// `tick_request_route_following` via the public `tick_request` entry with
    /// `autopilot.state = "Idle"` â†’ the disengage branch must clear the latch.
    #[test]
    fn prefab_curve_reset_on_disengage() {
        // Build a minimal plugin with a latch already active.
        let mut lk = LaneKeeperPlugin {
            prefab_curve_latched: true,
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        assert!(
            lk.prefab_curve_latched,
            "precondition: latch must start as true"
        );

        // ctx.is_active() = false: autopilot state is NOT "Active".
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Idle");
        let ctx = PluginContext::new("lane-keeper", bb);

        let t = make_telemetry(10.0, 0.0);
        let _ = lk.tick_request(Some(&t), &ctx);

        assert!(
            !lk.prefab_curve_latched,
            "prefab_curve_latched must be false after disengage (state != Active)"
        );
    }

    /// Phase 2h-Befund2-Fix: bei hoher interner KrÃ¼mmung (intK > threshold)
    /// muss der Catmull-Lookahead kÃ¼rzer sein als der Basis-Lookahead.
    #[test]
    fn catmull_curve_factor_reduces_lookahead() {
        let threshold = PREFAB_CURVE_FALLBACK_DEG as f64; // 40Â°
        let internal_kink = 80.0f64;
        let base = BASE_LOOK_AHEAD; // 5.0 at 0 m/s
        let curve_factor = (threshold / internal_kink).powi(2);
        let scaled = base * curve_factor;
        let effective = scaled.max(CATMULL_CURVE_MIN_LOOK_AHEAD);
        assert!(
            effective < base,
            "effective lookahead {effective:.2} must be < base {base:.2} when intK=80Â°"
        );
        assert!(
            effective >= CATMULL_CURVE_MIN_LOOK_AHEAD,
            "must not go below floor {CATMULL_CURVE_MIN_LOOK_AHEAD}"
        );
        assert!(
            curve_factor < 0.5,
            "curve_factor {curve_factor:.3} should be < 0.5 at intK=80Â°"
        );
    }

    /// Keine Ã„nderung wenn kein Catmull-Fallback (intK = 0).
    #[test]
    fn catmull_curve_factor_unchanged_at_zero_kink() {
        let base = BASE_LOOK_AHEAD + 15.0 * 3.6 * SPEED_FACTOR;
        let threshold = PREFAB_CURVE_FALLBACK_DEG as f64;
        let internal_kink = 0.0f64;
        let curve_factor = if internal_kink <= threshold || internal_kink < 1.0 {
            1.0f64
        } else {
            (threshold / internal_kink).powi(2)
        };
        let scaled = base * curve_factor;
        let effective = scaled.max(CATMULL_CURVE_MIN_LOOK_AHEAD);
        assert!(
            (effective - base).abs() < 1e-9,
            "effective {effective:.2} must equal base {base:.2} when intK=0"
        );
    }

    // â”€â”€ Phase 2h-Befund2-Fix: Route-Hop-Limit Tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // These verify the walk_end calculation in compute_heading_error (Catmull path).
    // The formula (same code as in production) â€” walk_end is an EXCLUSIVE upper bound:
    //   current_route_idx = (progress_idx + subdivisions / 2) / subdivisions
    //   max_route_idx     = (current_route_idx + max_hops).min(route_len - 1)
    //   walk_end          = (max_route_idx * subdivisions + 1).min(waypoints.len())
    // with max_hops = CATMULL_MAX_ROUTE_HOPS = 2, subdivisions = 4 (Default).
    //
    // Since walk_end is a local variable inside compute_heading_error, we verify it
    // indirectly via the "lane_keeper.catmull_walk_end" blackboard key that the
    // production code writes unconditionally on the Catmull path.

    /// Test CHR-1: 5-node route, Truck at progress_idx=4 (Node 1 of route).
    ///
    /// Setup:
    ///   - 5 route nodes  â†’ cached_route_node_ids has 5 entries â†’ route_len = 5
    ///   - subdivisions   = 4 (Default)
    ///   - waypoints      = 17 entries [(5-1)*4+1 = 17]
    ///   - progress_idx   = 4
    ///   - max_hops       = 2 (CATMULL_MAX_ROUTE_HOPS)
    ///
    /// Formula:
    ///   current_route_idx = (4 + 2) / 4 = 1
    ///   max_route_idx     = min(1 + 2, 4) = 3
    ///   walk_end          = min(3 * 4 + 1, 17) = min(13, 17) = 13
    ///   walk_start        = 4 + 1 = 5
    ///
    /// Expected: walk_end = 13 < waypoints.len() (17) â€” the hop-limit caps the walk.
    #[test]
    fn catmull_route_hop_limit_caps_walk_end() {
        // Verify the formula independently (no plugin side-effects needed here).
        let progress_idx: usize = 4;
        let subdivisions: usize = 4;
        let route_len: usize = 5;
        let max_hops: usize = CATMULL_MAX_ROUTE_HOPS;
        let waypoints_len: usize = (route_len - 1) * subdivisions + 1; // = 17

        let current_route_idx = (progress_idx + subdivisions / 2) / subdivisions;
        let max_route_idx = (current_route_idx + max_hops).min(route_len - 1);
        let walk_end = (max_route_idx * subdivisions + 1).min(waypoints_len);
        let walk_start = progress_idx + 1;

        assert_eq!(current_route_idx, 1, "current_route_idx must be 1");
        assert_eq!(max_route_idx, 3, "max_route_idx must be 3");
        assert_eq!(walk_end, 13, "walk_end must be capped at 13 by hop-limit");
        assert_eq!(walk_start, 5, "walk_start must be progress_idx + 1 = 5");
        assert!(
            walk_end < waypoints_len,
            "hop-limit must cap walk_end ({walk_end}) below total waypoints ({waypoints_len})"
        );

        // Also verify via the blackboard key that the running plugin emits.
        // Waypoints spaced 50m apart in -z direction so the advance-loop does NOT
        // swallow them (truck at (0,0), dist to waypoint[5] = 250m >> WAYPOINT_REACH_M=5m).
        let wps: Vec<[f64; 2]> = (0..waypoints_len)
            .map(|i| [0.0, -(i as f64) * 50.0])
            .collect();
        let mut lk = LaneKeeperPlugin {
            waypoints: wps,
            progress_idx,
            subdivisions,
            cached_route_node_ids: vec![10u64, 11, 12, 13, 14],
            ..Default::default()
        };
        // Force catmull path: no spline index wired â†’ try_spline returns None.
        let ctx = fresh_ctx();
        lk.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);

        let bb_walk_end: usize = ctx
            .blackboard
            .get("lane_keeper.catmull_walk_end")
            .and_then(|s| s.parse().ok())
            .expect("catmull_walk_end must be set on the Catmull path");
        assert_eq!(
            bb_walk_end, 13,
            "catmull_walk_end BB key must be 13 (hop-limit), got {bb_walk_end}"
        );
        assert!(
            bb_walk_end < waypoints_len,
            "catmull_walk_end ({bb_walk_end}) must be < total waypoints ({waypoints_len})"
        );
    }

    /// Test CHR-2: No route (cached_route_node_ids empty) â†’ walk_end = waypoints.len().
    ///
    /// When no routing is active (route_len < 2), the Catmull path must fall back to
    /// the old unlimited-walk behaviour: walk_end = waypoints.len().
    ///
    /// Setup:
    ///   - cached_route_node_ids empty â†’ route_len = 0 < 2 â†’ no hop-limit
    ///   - waypoints = 20 entries
    ///   - progress_idx = 4 â†’ walk_start = 5
    ///
    /// Expected: walk_end = 20 = waypoints.len() â€” complete walk, same as before the fix.
    #[test]
    fn catmull_route_hop_limit_noop_without_route() {
        let waypoints_len: usize = 20;
        let progress_idx: usize = 4;

        // Formula guard: route_len < 2 â†’ walk_end = waypoints.len() (old behaviour).
        let route_len: usize = 0;
        let walk_end_expected = waypoints_len; // unlimited
        let walk_start = progress_idx + 1;

        assert!(route_len < 2, "precondition: no routing active");
        assert_eq!(
            walk_end_expected, waypoints_len,
            "unlimited walk must cover all waypoints"
        );
        assert_eq!(walk_start, 5, "walk_start must be 5");

        // Verify via plugin + blackboard.
        // Waypoints spaced 50m apart so the advance-loop does NOT swallow them all.
        let wps: Vec<[f64; 2]> = (0..waypoints_len)
            .map(|i| [0.0, -(i as f64) * 50.0])
            .collect();
        let mut lk = LaneKeeperPlugin {
            waypoints: wps,
            progress_idx,
            cached_route_node_ids: vec![], // empty â†’ route_len < 2
            ..Default::default()
        };
        let ctx = fresh_ctx();
        lk.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);

        let bb_walk_end: usize = ctx
            .blackboard
            .get("lane_keeper.catmull_walk_end")
            .and_then(|s| s.parse().ok())
            .expect("catmull_walk_end must be set");
        assert_eq!(
            bb_walk_end, waypoints_len,
            "no-route: walk_end must equal waypoints.len() ({waypoints_len}), got {bb_walk_end}"
        );
    }

    /// Test CHR-3: Truck at the very last waypoint â†’ walk_start >= walk_end â†’ no walk, no panic.
    ///
    /// Setup:
    ///   - 3 route nodes   â†’ route_len = 3
    ///   - subdivisions = 4 â†’ waypoints_len = (3-1)*4+1 = 9
    ///   - progress_idx = 8 (= waypoints.len()-1, last index)
    ///
    /// Formula:
    ///   current_route_idx = (8 + 2) / 4 = 2
    ///   max_route_idx     = min(2 + 2, 2) = 2
    ///   max_waypoint_idx  = min(2 * 4, 8) = 8
    ///   walk_end          = min(8, 9) = 8
    ///   walk_start        = 8 + 1 = 9
    ///
    /// walk_start (9) > walk_end (8) â†’ the walk-loop body is skipped entirely.
    /// Must not panic; heading error must return 0.0 (route_end guard fires first).
    #[test]
    fn catmull_route_hop_limit_empty_range_safe() {
        let subdivisions: usize = 4;
        let route_len: usize = 3;
        let waypoints_len: usize = (route_len - 1) * subdivisions + 1; // = 9
        let progress_idx: usize = waypoints_len - 1; // = 8 (last index)
        let max_hops: usize = CATMULL_MAX_ROUTE_HOPS;

        // Verify formula produces the empty range.
        let current_route_idx = (progress_idx + subdivisions / 2) / subdivisions;
        let max_route_idx = (current_route_idx + max_hops).min(route_len - 1);
        let max_waypoint_idx = (max_route_idx * subdivisions).min(waypoints_len - 1);
        let walk_end = max_waypoint_idx.min(waypoints_len);
        let walk_start = progress_idx + 1;

        assert_eq!(current_route_idx, 2, "current_route_idx must be 2");
        assert_eq!(
            max_route_idx, 2,
            "max_route_idx must be clamped to route_len-1=2"
        );
        assert_eq!(max_waypoint_idx, 8, "max_waypoint_idx must be 8");
        assert_eq!(walk_end, 8, "walk_end must be 8");
        assert_eq!(walk_start, 9, "walk_start must be 9 (past end)");
        assert!(
            walk_start > walk_end,
            "walk_start ({walk_start}) must exceed walk_end ({walk_end}) â†’ empty range"
        );

        // Verify the plugin does NOT panic and returns 0.0 (route_end guard).
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0]; waypoints_len],
            progress_idx,
            subdivisions,
            cached_route_node_ids: vec![10u64, 11, 12],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Must not panic.
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            err, 0.0,
            "at route end, heading error must be 0.0 (route_end guard), got {err}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("route_end"),
            "skip_reason must be 'route_end' when progress_idx is at last waypoint"
        );
    }

    // â”€â”€ NearestSpline-mode tests (routerless lane-following, Weg B) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// Telemetry at a given XZ position with a given ETS2 heading (0..1, CCW from N).
    fn tel_at(x: f64, z: f64, heading: f64, speed_ms: f64) -> Telemetry {
        Telemetry {
            position: [x, 0.0, z],
            ..make_telemetry(speed_ms, heading)
        }
    }

    /// Wire a NearestSpline-mode plugin (state=Active by default) with the given
    /// segments/metadata. No router graph, no route â€” routerless by construction.
    fn nearest_wired(
        segs: Vec<HermiteSegment>,
        metas: Vec<Option<SegmentMetadata>>,
        state: &str,
    ) -> (LaneKeeperPlugin, PluginContext) {
        let road_n = segs.len();
        let idx = Arc::new(build_index_with_metadata(segs, metas));
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        bb.set("plugin.lane_keeper.mode", "nearest_spline");
        let ctx = PluginContext::new("lane-keeper", bb).with_spline_index(Arc::clone(&idx), road_n);
        let mut lk = LaneKeeperPlugin::default();
        lk.on_load(&ctx);
        (lk, ctx)
    }

    /// A north-bound straight road (tangent â†’ -Z â†’ heading 0Â° = North).
    fn north_seg(x: f32, z0: f32, z1: f32, from: u64, to: u64) -> HermiteSegment {
        seg((x, z0), (x, z1), from, to)
    }

    #[test]
    fn nearest_spline_selects_forward_segment() {
        // North road, truck on it facing North â†’ a heading-consistent segment is selected,
        // steering is emitted (Some), and the tracked segment is recorded.
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        let tel = tel_at(0.0, -100.0, 0.0, 10.0); // mid-segment, facing North
        let req = lk.tick_request(Some(&tel), &ctx);
        assert!(
            req.is_some(),
            "must emit steering on a heading-aligned road"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.mode").as_deref(),
            Some("nearest_spline")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "must track the only (north) segment"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.null_steer_cause")
                .as_deref(),
            Some("none")
        );
    }

    #[test]
    fn oncoming_segment_never_selected() {
        // Bidirectional road: north seg (idx 0) and overlapping south seg (idx 1).
        // Truck faces North â†’ the heading filter (â‰¤60Â°) must reject the oncoming south
        // segment; only the north segment can ever be selected.
        let segs = vec![
            north_seg(0.0, 0.0, -200.0, 10, 20),    // North (heading 0Â°)
            seg((0.0, -200.0), (0.0, 0.0), 20, 10), // South (heading 180Â°)
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        for _ in 0..5 {
            let tel = tel_at(0.0, -100.0, 0.0, 10.0);
            lk.tick_request(Some(&tel), &ctx);
            assert_eq!(
                ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
                Some("0"),
                "oncoming (south) segment must never be selected"
            );
        }
        let diff: f32 = ctx
            .blackboard
            .get("lane_keeper.nearest_seg_heading_diff_deg")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            diff < 60.0,
            "selected segment heading must be â‰¤60Â° off, got {diff}"
        );
    }

    #[test]
    fn chain_stays_on_segment_no_spatial_flip() {
        // Two parallel north segments (no shared uids â†’ no adjacency). The chain must NOT
        // do a per-frame spatial search mid-segment: even when B (idx 1) becomes the clearly
        // closest segment in space, the truck stays anchored on A (no flicker). This is what
        // structurally kills the 180Â°/lane-jump bug.
        let segs = vec![
            north_seg(0.0, 0.0, -200.0, 10, 20), // A
            north_seg(1.5, 0.0, -200.0, 30, 40), // B (1.5 m to the right)
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        // Tick 1: truck at x=0.3 â†’ A clearly closest â†’ chain anchors on A.
        lk.tick_request(Some(&tel_at(0.3, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "tick 1 must anchor on A"
        );

        // Tick 2: truck at x=1.3 â†’ distA=1.3, distB=0.2 (B over 6Ã— closer in SPACE). Old
        // hysteresis would have switched; the chain must NOT â€” mid-segment never re-queries.
        lk.tick_request(Some(&tel_at(1.3, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "chain must stay on A despite B being far closer (no mid-segment spatial flip)"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("sticky"),
            "staying on the segment â†’ chain_advance_reason=sticky"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("false"),
            "sticky stay is not a chain break"
        );
    }

    #[test]
    fn fwd_progress_advances_at_segment_end() {
        // A (10â†’20) then successor B (20â†’30), both north. Truck near the end of A
        // (tâ‰¥0.85) â†’ must advance to the forward_adj successor B.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A
            north_seg(0.0, -100.0, -200.0, 20, 30), // B (B.from_uid == A.to_uid)
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        // Tick 1: truck near end of A (z=-95 â†’ tâ‰ˆ0.95) â†’ anchors A.
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "tick 1 anchors A"
        );
        // Tick 2: tâ‰¥0.85 on A â†’ forward progress to B.
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("1"),
            "must advance to successor B"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_seg_switch_reason")
                .as_deref(),
            Some("fwd_progress")
        );
    }

    #[test]
    fn junction_no_straight_successor_falls_back_spatial() {
        // A (10â†’20) north; its only successor B (20â†’30) turns East (90Â°, fails the <90Â°
        // chain-successor filter). At the end of A the chain has no valid forward successor
        // â†’ spatial re-acquisition (60Â° filter). The East segment is >60Â° off truck heading,
        // so spatial re-grabs A â€” the truck NEVER jumps onto the cross/East segment.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),         // A (North), idx 0
            seg((0.0, -100.0), (100.0, -100.0), 20, 30), // B (East, 90Â°), idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // anchor A
        let _ = lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // segment end â†’ break
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "must NOT jump onto the East/cross segment â€” spatial fallback re-grabs A"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("true"),
            "no valid forward successor â†’ chain broken â†’ spatial fallback"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("reacquire"),
            "chain break â†’ reacquire via spatial query"
        );
    }

    // â”€â”€ Offset-Vererbung + distanz-basierter Advance (Segment-Sprung-Fix) â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// f32-Blackboard-Helper fÃ¼r die Offset-/Distanz-Assertions.
    fn bb_f32_key(ctx: &PluginContext, key: &str) -> f32 {
        ctx.blackboard
            .get(key)
            .unwrap_or_else(|| panic!("{key} must be set"))
            .parse()
            .unwrap_or_else(|_| panic!("{key} must parse as f32"))
    }

    #[test]
    fn offset_inherited_on_meta_none() {
        // A = 3-spurige Road (offset 3.75), B = metadatenlose prefab-LÃ¼cke (meta=None,
        // B.from == A.to). Beim Chain-Advance Aâ†’B muss B den ECHTEN Offset von A ERBEN
        // (3.75), NICHT auf LANE_OFFSET_RIGHT_M (1.875) fallen.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A (3-lane road), idx 0
            north_seg(0.0, -100.0, -200.0, 20, 30), // B (meta=None gap), idx 1
        ];
        let metas = vec![Some(road_meta(3, 3.75, false)), None];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        // Tick 1: anchor A (real 3-lane metadata) â†’ last_road_lane_offset = 3.75.
        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "tick 1 anchors the 3-lane road A"
        );
        assert!(
            (bb_f32_key(&ctx, "lane_keeper.lane_offset_applied_m") - 3.75).abs() < 1e-3,
            "on A the applied offset is the real 3-lane offset 3.75"
        );

        // Tick 2: near end of A â†’ advance to B (meta=None). Offset must be INHERITED 3.75.
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("1"),
            "tick 2 advances onto the meta=None gap B"
        );
        assert!(
            (bb_f32_key(&ctx, "lane_keeper.lane_offset_applied_m") - 3.75).abs() < 1e-3,
            "meta=None gap must INHERIT A's 3.75 offset, NOT fall back to 1.875"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("nearest_inherited"),
            "inherited-offset segment is tagged nearest_inherited"
        );
    }

    #[test]
    fn offset_not_inherited_road_to_road() {
        // A = 3-spurig (3.75), B = 2-spurig (1.875), beide ECHTE Road-Metadaten,
        // B.from == A.to. Am Roadâ†’Road-Ãœbergang mit anderer Spuranzahl behÃ¤lt B seinen
        // EIGENEN Offset (1.875) â€” Vererbung greift hier NICHT.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A (3-lane), idx 0
            north_seg(0.0, -100.0, -200.0, 20, 30), // B (2-lane), idx 1
        ];
        let metas = vec![
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A (3.75)
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // advance to B (2-lane)
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("1"),
            "advanced onto the 2-lane road B"
        );
        assert!(
            (bb_f32_key(&ctx, "lane_keeper.lane_offset_applied_m") - 1.875).abs() < 1e-3,
            "real road keeps its OWN 2-lane offset 1.875 (no inheritance of 3.75)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("nearest_road"),
            "real road metadata â†’ nearest_road, not nearest_inherited"
        );
    }

    #[test]
    fn offset_default_when_no_prior_road() {
        // ALLERERSTES Segment nach Engage ist meta=None (kein last_road) â†’ sauberer
        // Default 1.875 (Fallback des Fallbacks), keine Panik durch ein leeres `None`.
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![None];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        let req = lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx);
        assert!(req.is_some(), "meta=None road still steers");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0")
        );
        assert!(
            (bb_f32_key(&ctx, "lane_keeper.lane_offset_applied_m") - 1.875).abs() < 1e-3,
            "no prior real road â†’ default 1.875, not a stale/zero offset"
        );
    }

    #[test]
    fn advance_distance_based_long_segment() {
        // 300 m langes A: t=0.85 liegt 45 m vor dem Knoten. Mit dem Distanz-Gate
        // (NEAREST_ADVANCE_DIST_M=35) wird bei t=0.87 (Restbogen 39 m) NOCH NICHT
        // geschaltet (alt: tâ‰¥0.85 hÃ¤tte geschaltet) â€” erst bei Restbogen <35 m, dann
        // liegt der Nachfolger sicher im 50-m-Radius (chain_broken=false).
        let segs = vec![
            north_seg(0.0, 0.0, -300.0, 10, 20),    // A (300 m), idx 0
            north_seg(0.0, -300.0, -400.0, 20, 30), // B, idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -150.0, 0.0, 10.0)), &ctx); // anchor A

        // z=-261 â†’ t=0.87, Restbogen â‰ˆ 39 m > 35 â†’ STICKY (alt hÃ¤tte advanced).
        lk.tick_request(Some(&tel_at(0.0, -261.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "Restbogen 39 m > 35 m â†’ noch NICHT schalten (kein verfrÃ¼hter Advance)"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("sticky")
        );

        // z=-270 â†’ t=0.90, Restbogen 30 m < 35 â†’ advance, Nachfolger in Reichweite.
        lk.tick_request(Some(&tel_at(0.0, -270.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("1"),
            "Restbogen 30 m < 35 m â†’ jetzt auf B schalten"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("false"),
            "Nachfolger lag bei 30 m im Radius â†’ sauberer Ãœbergang, kein Abriss"
        );
    }

    #[test]
    fn advance_distance_based_short_segment() {
        // 40 m kurzes A (< 233 m â†’ Distanz-Gate bindet NICHT): bei t=0.5 (Restbogen 20 m
        // < 35!) darf NICHT geschaltet werden (das t-Gate â‰¥0.85 schÃ¼tzt vor verfrÃ¼htem
        // Advance / Lookahead-Verschiebung); erst bei tâ‰¥0.85 (wie bisher) schalten.
        let segs = vec![
            north_seg(0.0, 0.0, -40.0, 10, 20),    // A (40 m), idx 0
            north_seg(0.0, -40.0, -140.0, 20, 30), // B, idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -10.0, 0.0, 10.0)), &ctx); // anchor A

        // z=-20 â†’ t=0.5: Restbogen 20 m < 35, ABER t<0.85 â†’ STICKY (kein FrÃ¼h-Advance).
        lk.tick_request(Some(&tel_at(0.0, -20.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("0"),
            "kurzes Segment t=0.5: t-Gate verhindert verfrÃ¼hten Advance trotz Restbogen<35"
        );

        // z=-36 â†’ t=0.9 â‰¥ 0.85 â†’ wie bisher schalten (gutmÃ¼tig auf kurzen Segmenten).
        lk.tick_request(Some(&tel_at(0.0, -36.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.nearest_seg_idx").as_deref(),
            Some("1"),
            "kurzes Segment t=0.9: zeitig schalten (unverÃ¤ndertes Verhalten)"
        );
    }

    #[test]
    fn no_fwd_bwd_flicker_at_long_segment() {
        // Der t28-Fall: 400 m A â†’ kurzes B. Alt schaltete bei t=0.85 (60 m vor dem Knoten,
        // Nachfolger auÃŸerhalb 50 m â†’ off_segment â†’ Re-Acquire â†’ 2-Zyklus). Neu: Restbogen-
        // Gate hÃ¤lt bis 35 m am Knoten; Ã¼ber die kritische Zone darf KEIN chain_broken=true
        // auftreten und der Index darf nicht 1â†’0 zurÃ¼ckspringen (kein Flackern).
        let segs = vec![
            north_seg(0.0, 0.0, -400.0, 10, 20),    // A (400 m), idx 0
            north_seg(0.0, -400.0, -500.0, 20, 30), // B (100 m), idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -200.0, 0.0, 10.0)), &ctx); // anchor A

        // Kritische Zone: 60 m â†’ 20 m vor dem Knoten (z=-400). Alt: Flackern; neu: glatt.
        let zs = [-340.0, -350.0, -360.0, -368.0, -380.0];
        let mut seq: Vec<u32> = Vec::new();
        for &z in &zs {
            lk.tick_request(Some(&tel_at(0.0, z, 0.0, 10.0)), &ctx);
            let seg: u32 = ctx
                .blackboard
                .get("lane_keeper.nearest_seg_idx")
                .unwrap()
                .parse()
                .unwrap();
            seq.push(seg);
            assert_eq!(
                ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
                Some("false"),
                "kein Chain-Abriss in der Ãœbergangszone (z={z}) â€” Nachfolger immer in Reichweite"
            );
        }
        // Index monoton steigend (0â€¦0,1â€¦1), nie 1â†’0 zurÃ¼ck â†’ kein 2-Zyklus.
        for w in seq.windows(2) {
            assert!(
                w[1] >= w[0],
                "Chain-Index darf nicht zurÃ¼ckspringen (Flacker-Signatur): {seq:?}"
            );
        }
        assert_eq!(
            *seq.last().unwrap(),
            1,
            "am Ende der Zone ist die Chain sauber auf B (idx 1)"
        );
    }

    #[test]
    fn engage_allowed_without_route() {
        // NearestSpline, state Off (pre-engage), truck within 20 m of a heading-aligned
        // segment â†’ engage_allowed=true is published WITHOUT any router route. No steering.
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Off");
        let req = lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 0.0)), &ctx);
        assert!(req.is_none(), "must not steer while not Active");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true"),
            "engage_allowed must be true near a heading-aligned lane, no route needed"
        );
        // Far-away truck â†’ engage_allowed=false.
        let req2 = lk.tick_request(Some(&tel_at(500.0, -100.0, 0.0, 0.0)), &ctx);
        assert!(req2.is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false"),
            "engage_allowed must be false when far from any lane"
        );
    }

    // â”€â”€ Capture-Modus + Anti-Stall (Low-Speed-Cap, Stuck-Watchdog, Gate 45 m) â”€â”€

    /// Standard-Szenario der Capture-Tests: 1-spurige NordstraÃŸe, Soll-Linie bei
    /// x = 0.0 (lane_offset_right). `x_off` = gewÃ¼nschtes e_lat.
    fn capture_wired() -> (LaneKeeperPlugin, PluginContext) {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        nearest_wired(segs, metas, "Active")
    }

    #[test]
    fn steer_cap_low_speed() {
        // Pure Funktion: lineare Ã–ffnung 0.3 (v=0) â†’ 1.0 (vâ‰¥3).
        assert!((low_speed_steer_cap(0.0) - 0.3).abs() < 1e-9);
        assert!((low_speed_steer_cap(1.5) - 0.65).abs() < 1e-9);
        assert!((low_speed_steer_cap(3.0) - 1.0).abs() < 1e-9);
        assert!((low_speed_steer_cap(10.0) - 1.0).abs() < 1e-9);
        // Integration: v=0 weit neben der Linie (e_lat=20 â†’ xtrack-Wunsch ~0.69)
        // â†’ Output auf 0.3 gecapt (Watchdog-Schwelle 50 Ticks hier nicht erreicht).
        let (mut lk, ctx) = capture_wired();
        for _ in 0..20 {
            lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 0.0)), &ctx);
        }
        let steer = bb_f64(&ctx, "lane_keeper.steering_out");
        assert!(
            steer.abs() <= 0.3 + 1e-6,
            "v=0 â†’ |steer| <= 0.3, ist {steer}"
        );
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!((cap - 0.3).abs() < 1e-6, "Cap bei v=0 ist 0.3, ist {cap}");
    }

    #[test]
    fn steer_cap_full_at_speed() {
        // Auf der Linie bei v=10 m/s: kein Capture, kein Low-Speed-Cap â†’ Cap 1.0.
        let (mut lk, ctx) = capture_wired();
        lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx);
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!(
            (cap - 1.0).abs() < 1e-6,
            "bei Tempo + auf der Linie kein Cap (1.0), ist {cap}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn stuck_watchdog_relaxes() {
        // vâ‰ˆ0 + groÃŸe Lenk-Absicht (e_lat=20 â†’ Wunsch ~0.69 > 0.4) Ã¼ber >50 Ticks
        // â†’ Recovery aktiv, Cap 0.15, Output relaxt unter 0.15.
        let (mut lk, ctx) = capture_wired();
        for _ in 0..60 {
            lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 0.1)), &ctx);
        }
        assert_eq!(
            ctx.blackboard.get("lane_keeper.stuck_recovery").as_deref(),
            Some("true"),
            "Watchdog muss nach >50 Ticks Stillstand+Lenkwunsch auslÃ¶sen"
        );
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!((cap - 0.15).abs() < 1e-6, "Recovery-Cap 0.15, ist {cap}");
        let steer = bb_f64(&ctx, "lane_keeper.steering_out");
        assert!(
            steer.abs() <= 0.15 + 1e-6,
            "Lenkung relaxt auf <= 0.15, ist {steer}"
        );
    }

    #[test]
    fn stuck_watchdog_resets_when_moving() {
        // Recovery aktiv â†’ Truck rollt wieder (v=2 > 1.0) â†’ ZÃ¤hler+Recovery weg.
        let (mut lk, ctx) = capture_wired();
        for _ in 0..60 {
            lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 0.1)), &ctx);
        }
        assert_eq!(
            ctx.blackboard.get("lane_keeper.stuck_recovery").as_deref(),
            Some("true")
        );
        lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 2.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.stuck_recovery").as_deref(),
            Some("false"),
            "v > 1.0 m/s muss den Watchdog zurÃ¼cksetzen"
        );
        assert_eq!(lk.stuck_ticks, 0, "stuck_ticks muss auf 0 zurÃ¼ckgehen");
        // Capture (e_lat 20) bleibt aktiv â†’ Cap = min(low_speed(2)=0.767, 0.5) = 0.5.
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!((cap - 0.5).abs() < 1e-6, "Capture-Cap 0.5, ist {cap}");
    }

    #[test]
    fn capture_active_when_far() {
        // e_lat = 3 m (> 1.5) bei Tempo â†’ Capture aktiv, Cap 0.5, Tempoziel 20 km/h.
        let (mut lk, ctx) = capture_wired();
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("true"),
            "|e_lat| > 1.5 m muss Capture aktivieren"
        );
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!((cap - 0.5).abs() < 1e-6, "Capture-Steer-Cap 0.5, ist {cap}");
        let steer = bb_f64(&ctx, "lane_keeper.steering_out");
        assert!(steer.abs() <= 0.5 + 1e-6, "Steering im Capture <= 0.5");
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("20.0")
        );
    }

    #[test]
    fn capture_exits_when_on_line() {
        // Erst fern (Capture an), dann auf der Linie: Exit erst nach der Hysterese
        // (CAPTURE_EXIT_STABLE_TICKS), kein Flackern nach einem einzelnen guten Tick.
        let (mut lk, ctx) = capture_wired();
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("true")
        );
        // Ein einzelner On-Line-Tick beendet Capture NICHT (Hysterese).
        lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("true"),
            "Hysterese: 1 guter Tick darf Capture nicht beenden"
        );
        for _ in 0..CAPTURE_EXIT_STABLE_TICKS {
            lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx);
        }
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("false"),
            "stabil auf der Linie â†’ Capture endet"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("-1.0"),
            "Tempoziel muss nach Capture-Exit gerÃ¤umt sein"
        );
        let cap = bb_f64(&ctx, "lane_keeper.steer_cap_applied");
        assert!((cap - 1.0).abs() < 1e-6, "nach Exit voller Cap (1.0)");
    }

    #[test]
    fn capture_speed_target_published() {
        // Pre-Engage/Disengage rÃ¤umt das Tempoziel (-1.0), Capture setzt es (20.0).
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Off");
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("-1.0"),
            "Pre-Engage: Tempoziel inaktiv"
        );
        ctx.blackboard.set("autopilot.state", "Active");
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("20.0"),
            "Active + Capture: Tempoziel 20 km/h"
        );
    }

    #[test]
    fn gate_engage_at_45m() {
        // Laterales Gate (3 m) + Ã¤uÃŸeres Gate (45 m):
        //  - 2 m: unter beiden Gates â†’ engage_allowed (auf der Soll-Linie)
        //  - 44 m: unter 45 m-Gate, ÃœBER 3 m-Lateral-Gate â†’ lateral_too_far
        //  - 48 m: Ã¼ber 45 m-Gate â†’ too_far
        //  - 80 m: auÃŸerhalb des 50-m-Suchradius â†’ no_segment
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Off");

        // 2 m lateral: beide Gates erfÃ¼llt â†’ engaged
        lk.tick_request(Some(&tel_at(2.0, -100.0, 0.0, 5.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true"),
            "2 m: unter 3 m-Lateral-Gate â†’ engage_allowed"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.engage_block_reason")
                .as_deref(),
            Some("ok"),
        );

        // 44 m lateral: unter 45 m-Gate, Ã¼ber 3 m-Lateral-Gate â†’ lateral_too_far
        lk.tick_request(Some(&tel_at(44.0, -100.0, 0.0, 5.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false"),
            "44 m: laterales Gate blockiert"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.engage_block_reason")
                .as_deref(),
            Some("lateral_too_far"),
            "44 m: Dist unter 45 m-Grenze, aber Ã¼ber 3 m-Lateral-Gate"
        );

        // 48 m: Ã¼ber 45 m Ã¤uÃŸeres Gate â†’ too_far
        lk.tick_request(Some(&tel_at(48.0, -100.0, 0.0, 5.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.engage_block_reason")
                .as_deref(),
            Some("too_far"),
            "48 m: im Radius, aber Ã¼ber dem 45 m-Gate"
        );

        // 80 m: auÃŸerhalb des 50-m-Suchradius â†’ no_segment
        lk.tick_request(Some(&tel_at(80.0, -100.0, 0.0, 5.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.engage_block_reason")
                .as_deref(),
            Some("no_segment"),
            "80 m: auÃŸerhalb des 50-m-Suchradius"
        );
    }

    #[test]
    fn mode_switch_clears_capture_state() {
        // Reviewer-K1: der Disengage setzt plugin.lane_keeper.mode SOFORT auf
        // route_following zurÃ¼ck â€” der nearest-Pre-Engage-Cleanup lÃ¤uft dann nie.
        // Der Mode-Wechsel selbst muss das Capture-Tempoziel rÃ¤umen, sonst cappt
        // es den nÃ¤chsten Route-Engage dauerhaft auf 20 km/h.
        let (mut lk, ctx) = capture_wired();
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("20.0")
        );
        ctx.blackboard
            .set("plugin.lane_keeper.mode", "route_following");
        ctx.blackboard.set("autopilot.state", "Off");
        lk.tick_request(Some(&tel_at(3.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(lk.mode, LaneKeeperMode::RouteFollowing);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("-1.0"),
            "Mode-Wechsel muss das Capture-Tempoziel rÃ¤umen (K1)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("false")
        );
        assert!(!lk.capture_active);
        assert_eq!(lk.stuck_ticks, 0);
    }

    #[test]
    fn capture_no_entry_from_heading_alone() {
        // Reviewer-K2: heading_err ist ein Preview-Fehler (â‰ˆ look/R in Kurven) â€”
        // auf der Linie (e_lat â‰ˆ 0) darf ein groÃŸer Heading-Fehler Capture NICHT
        // zÃ¼nden (sonst 20-km/h-Ziel + Vollbrems-Override in jeder Kurve).
        let (mut lk, ctx) = capture_wired();
        // Auf der Soll-Linie, aber 30Â° schief (heading 30Â°/360Â° CW â†’ raw -30/360).
        lk.tick_request(Some(&tel_at(0.0, -100.0, -30.0 / 360.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.capture_active").as_deref(),
            Some("false"),
            "Heading allein darf Capture nicht aktivieren"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.capture_speed_target_kmh")
                .as_deref(),
            Some("-1.0")
        );
    }

    #[test]
    fn route_following_unchanged() {
        // Control: a default (RouteFollowing) plugin, Active, no waypoints â†’ unchanged
        // behaviour (None + skip_reason=no_waypoints) and the NearestSpline status key is
        // never written. The per-tick mode re-check must not perturb RouteFollowing.
        let ctx = ctx_with_state("Active");
        let mut lk = LaneKeeperPlugin::default();
        let tel = make_telemetry(10.0, 0.0);
        let req = lk.tick_request(Some(&tel), &ctx);
        assert!(
            req.is_none(),
            "route-following with no waypoints returns None"
        );
        assert_eq!(lk.mode, LaneKeeperMode::RouteFollowing);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("no_waypoints"),
            "unchanged route-following skip path"
        );
        assert_ne!(
            ctx.blackboard.get("lane_keeper.mode").as_deref(),
            Some("nearest_spline"),
            "route-following must never enter nearest_spline"
        );
    }

    #[test]
    fn far_from_lane_returns_none() {
        // Active, but the truck is >50 m from any segment â†’ no candidate, no tracking â†’
        // clean None with null_steer_cause=no_segment (no garbage steering).
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        let req = lk.tick_request(Some(&tel_at(500.0, -100.0, 0.0, 10.0)), &ctx);
        assert!(req.is_none(), "far from any lane must not steer");
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.null_steer_cause")
                .as_deref(),
            Some("no_segment")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    // â”€â”€ Chain-Topology tests (180Â°-Fix: forward_adj statt per-Frame-Spatial) â”€â”€â”€â”€â”€â”€

    #[test]
    fn chain_advances_via_forward_adj() {
        // A (10â†’20) then forward_adj successor B (20â†’30), both north. At the end of A the
        // chain advances to B over the topology (forward_adj), reason=segment_end â€” NOT a
        // spatial re-query (chain_broken=false).
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A, idx 0
            north_seg(0.0, -100.0, -200.0, 20, 30), // B (B.from == A.to), idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A (mid)
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("0"),
            "tick 1 anchors A"
        );
        // Truck at end of A (tâ‰ˆ0.95) â†’ advance to B via forward_adj.
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "must advance to forward_adj successor B"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("segment_end"),
            "advance at segment end â†’ chain_advance_reason=segment_end"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("false"),
            "topology advance is not a chain break"
        );
    }

    #[test]
    fn chain_never_selects_oncoming() {
        // Aâ†’B forward chain, plus an oncoming (south) segment that spatially OVERLAPS A but is
        // NOT a forward_adj successor (own uids). The chain follows Aâ†’B; the oncoming segment
        // (idx 1) can never be selected â€” it is not in forward_adj[A.to_uid].
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A (North), idx 0
            seg((0.0, -100.0), (0.0, 0.0), 98, 99), // oncoming (South, 180Â°) overlapping A, idx 1
            north_seg(0.0, -100.0, -200.0, 20, 30), // B (B.from == A.to), idx 2
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        // Drive A (mid) â†’ end of A (advance to B). At no point may idx 1 (oncoming) be chosen.
        for (z, _label) in [(-40.0, "mid-A"), (-95.0, "end-Aâ†’B"), (-150.0, "mid-B")] {
            lk.tick_request(Some(&tel_at(0.0, z, 0.0, 10.0)), &ctx);
            let idx = ctx.blackboard.get("lane_keeper.chain_segment_idx").unwrap();
            assert_ne!(idx, "1", "oncoming segment (idx 1) must never be selected");
        }
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("2"),
            "after the segment end the chain is on B (idx 2)"
        );
    }

    #[test]
    fn chain_picks_straightest_at_fork() {
        // A forks into B (straight north, 0Â°) and C (45Â° NE). Both pass the 90Â° filter, so the
        // chain picks the straightest (smallest heading kink = B), deterministically.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),        // A, idx 0
            north_seg(0.0, -100.0, -200.0, 20, 30),     // B straight, idx 1
            seg((0.0, -100.0), (70.7, -170.7), 20, 40), // C 45Â° NE, idx 2
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // fork
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "at the fork the straightest successor (B) must be chosen"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_successor_count")
                .as_deref(),
            Some("2"),
            "both forks pass the 90Â° filter â†’ successor_count=2"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_seg_switch_reason")
                .as_deref(),
            Some("junction_pick"),
            "multiple successors â†’ junction_pick"
        );
    }

    #[test]
    fn chain_spatial_fallback_on_break() {
        // A north, dead-end: forward_adj[A.to_uid] is empty (no successor). At the segment end
        // the chain breaks â†’ spatial re-acquisition (60Â° filter) re-grabs A (still aligned),
        // chain_broken=true. No jump to anything off-heading.
        let segs = vec![north_seg(0.0, 0.0, -100.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // dead-end â†’ break
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("true"),
            "empty forward_adj â†’ chain broken â†’ spatial fallback"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("0"),
            "spatial fallback re-grabs the only aligned segment (A)"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("reacquire")
        );
    }

    #[test]
    fn chain_reacquires_after_drift() {
        // Truck anchors on A (x=0), then drifts >50 m sideways onto parallel D (x=100). Mid-
        // segment the chain detects off-segment (dist>50 m) â†’ spatial re-acquisition onto D.
        let segs = vec![
            north_seg(0.0, 0.0, -200.0, 10, 20),   // A, idx 0
            north_seg(100.0, 0.0, -200.0, 30, 40), // D (100 m east, parallel), idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx); // anchor A
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("0"),
            "tick 1 anchors A"
        );
        // Drift to D (100 m from A, still mid-segment tâ‰ˆ0.5 on A â†’ dist>50 m off A).
        lk.tick_request(Some(&tel_at(100.0, -100.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "after drift the chain re-acquires onto D"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_advance_reason")
                .as_deref(),
            Some("off_segment"),
            "mid-segment drift > 50 m â†’ chain_advance_reason=off_segment"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("true"),
            "drift off the chain is a break â†’ spatial fallback"
        );
    }

    // â”€â”€ Reverse-Geschwister-Fix tests (2-Zyklus-HÃ¤rtung) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// A reverse-direction sibling built the way the map-parser actually builds them
    /// (b-ii, confirmed by `seg-inspect` on real segment 412921): endpoints swapped
    /// (correct) but the Hermite tangents COPIED, not negated â€” so `evaluate_tangent(rev, 0.0)`
    /// points FORWARD and fools any tangent-based filter. Only the UID reverse-skip catches it.
    fn malformed_reverse(fwd: &HermiteSegment, from: u64, to: u64) -> HermiteSegment {
        HermiteSegment {
            p0: fwd.p1,
            p1: fwd.p0,
            m0: fwd.m1, // â† copied (swapped), NOT negated = the real bug
            m1: fwd.m0,
            length_m: fwd.length_m,
            from_uid: from,
            to_uid: to,
            edge_uid: fwd.edge_uid.wrapping_add(1),
        }
    }

    #[test]
    fn chain_skips_reverse_sibling() {
        // A(10â†’20). forward_adj[20] holds the MALFORMED reverse sibling (20â†’10, whose t=0
        // tangent points forward â†’ fools a tangent filter) AND a real forward continuation
        // B(20â†’30). The UID reverse-skip must drop the sibling and advance to B.
        let a = north_seg(0.0, 0.0, -100.0, 10, 20);
        let rev = malformed_reverse(&a, 20, 10); // exact reverse of A, malformed tangent
        let b = north_seg(0.0, -100.0, -200.0, 20, 30); // real forward continuation
        let segs = vec![a, rev, b]; // idx 0=A, 1=rev, 2=B
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // segment end
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("2"),
            "must skip the reverse sibling and advance to the real forward continuation B"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_reverse_skipped")
                .as_deref(),
            Some("1"),
            "exactly one reverse sibling skipped"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_successor_count")
                .as_deref(),
            Some("1"),
            "successor_count counts only the real forward successor (after reverse skip)"
        );
    }

    #[test]
    fn chain_keeps_real_forward_successor() {
        // A real forward continuation (same direction, different segment, NOT from/to-swapped)
        // must NOT be skipped by the reverse guard.
        let a = north_seg(0.0, 0.0, -100.0, 10, 20);
        let b = north_seg(0.0, -100.0, -200.0, 20, 30); // forward, from=20=A.to, to=30
        let segs = vec![a, b];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // advance
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "real forward successor B must remain selectable"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_reverse_skipped")
                .as_deref(),
            Some("0"),
            "a real forward successor is NOT a reverse sibling â†’ nothing skipped"
        );
    }

    #[test]
    fn chain_exit_tangent_filter_rejects_reverse() {
        // A successor that points BACKWARD (180Â°) but is NOT a UID reverse-sibling (to=30 â‰ 
        // A.from=10) â†’ the exit-tangent dot filter (not the UID skip) must reject it.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A (North), idx 0
            seg((0.0, -100.0), (0.0, 0.0), 20, 30), // S: from=20=A.to, to=30; heading 180Â°
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // segment end â†’ S rejected
        assert_ne!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "backward successor (dotâ‰ˆâˆ’1) must be rejected by the exit-tangent filter"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_reverse_skipped")
                .as_deref(),
            Some("0"),
            "S is not a UID reverse-sibling â†’ it is the dot filter, not the UID skip, that drops it"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("true"),
            "no valid forward successor â†’ chain breaks â†’ spatial fallback"
        );
    }

    #[test]
    fn chain_exit_tangent_filter_keeps_forward() {
        // A forward successor (same direction, dotâ‰ˆ+1) must pass the exit-tangent filter.
        let segs = vec![
            north_seg(0.0, 0.0, -100.0, 10, 20),    // A, idx 0
            north_seg(0.0, -100.0, -200.0, 20, 30), // S forward, idx 1
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        lk.tick_request(Some(&tel_at(0.0, -50.0, 0.0, 10.0)), &ctx); // anchor A
        lk.tick_request(Some(&tel_at(0.0, -95.0, 0.0, 10.0)), &ctx); // advance to S
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "forward successor (dotâ‰ˆ+1) must pass the exit-tangent filter"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.chain_broken").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn chain_no_flicker_fwd_bwd() {
        // Full 2-cycle reproduction: A + its malformed reverse sibling + a real continuation B.
        // Across many ticks driving Aâ†’B, the chain must NEVER land on the reverse sibling
        // (idx 1) â€” no forward/backward flicker.
        let a = north_seg(0.0, 0.0, -100.0, 10, 20);
        let rev = malformed_reverse(&a, 20, 10);
        let b = north_seg(0.0, -100.0, -200.0, 20, 30);
        let segs = vec![a, rev, b]; // idx 1 = reverse sibling
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");

        for z in [-40.0, -80.0, -95.0, -99.0, -120.0, -160.0, -190.0] {
            lk.tick_request(Some(&tel_at(0.0, z, 0.0, 10.0)), &ctx);
            assert_ne!(
                ctx.blackboard
                    .get("lane_keeper.chain_segment_idx")
                    .as_deref(),
                Some("1"),
                "reverse sibling must never be selected (no fwd/bwd flicker) at z={z}"
            );
        }
        // Ends up on the real continuation B (idx 2), heading aligned (no 180Â° flip).
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("2"),
            "chain follows Aâ†’B forward, never the reverse sibling"
        );
    }

    // â”€â”€ v3 Cross-Track-Term tests (NearestSpline only) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    fn bb_f64(ctx: &PluginContext, key: &str) -> f64 {
        ctx.blackboard
            .get(key)
            .unwrap_or_else(|| panic!("key {key} missing"))
            .parse()
            .unwrap_or_else(|_| panic!("key {key} not f64"))
    }

    #[test]
    fn xtrack_zero_when_on_line() {
        // 2-lane north road â†’ soll-line at x = lane_offset = 1.875. Truck ON it, aligned â†’
        // e_lat â‰ˆ 0 â†’ cross-track contribution â‰ˆ 0.
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, false))]; // 2-lane â†’ 1.875
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        lk.tick_request(Some(&tel_at(1.875, -100.0, 0.0, 10.0)), &ctx);
        let e_lat = bb_f64(&ctx, "lane_keeper.xtrack_e_lat_m");
        let xc = bb_f64(&ctx, "lane_keeper.xtrack_contribution_rad");
        assert!(e_lat.abs() < 0.05, "on soll-line â†’ e_latâ‰ˆ0, got {e_lat}");
        assert!(xc.abs() < 0.02, "on soll-line â†’ xtrackâ‰ˆ0, got {xc}");
    }

    #[test]
    fn xtrack_corrects_toward_line() {
        // 2-lane soll-line at x=1.875.
        // Truck RIGHT of line (x=10) â†’ e_lat>0 â†’ steer LEFT (contribution<0).
        let (mut lk_r, ctx_r) = nearest_wired(
            vec![north_seg(0.0, 0.0, -200.0, 10, 20)],
            vec![Some(road_meta(2, 3.75, false))],
            "Active",
        );
        lk_r.tick_request(Some(&tel_at(10.0, -100.0, 0.0, 10.0)), &ctx_r);
        let e_r = bb_f64(&ctx_r, "lane_keeper.xtrack_e_lat_m");
        let xc_r = bb_f64(&ctx_r, "lane_keeper.xtrack_contribution_rad");
        assert!(e_r > 0.0, "truck right of soll-line â†’ e_lat>0, got {e_r}");
        assert!(
            xc_r < 0.0,
            "truck right â†’ cross-track steers LEFT (negative), got {xc_r}"
        );

        // Truck LEFT of line (x=2) â†’ e_lat<0 â†’ steer RIGHT (contribution>0).
        let (mut lk_l, ctx_l) = nearest_wired(
            vec![north_seg(0.0, 0.0, -200.0, 10, 20)],
            vec![Some(road_meta(2, 3.75, false))],
            "Active",
        );
        lk_l.tick_request(Some(&tel_at(0.0, -100.0, 0.0, 10.0)), &ctx_l);
        let e_l = bb_f64(&ctx_l, "lane_keeper.xtrack_e_lat_m");
        let xc_l = bb_f64(&ctx_l, "lane_keeper.xtrack_contribution_rad");
        assert!(e_l < 0.0, "truck left of soll-line â†’ e_lat<0, got {e_l}");
        assert!(
            xc_l > 0.0,
            "truck left â†’ cross-track steers RIGHT (positive), got {xc_l}"
        );
    }

    #[test]
    fn xtrack_bounded() {
        // High K_CT (4.0) + large e_lat (7.6 m) â†’ |contribution| clamped to XTRACK_MAX_RAD.
        // Mit dem Kriechtempo-Lookahead-Floor ist denom bei v=0 = NEAREST_MIN_LOOK_AHEAD
        // = 12 m (statt frÃ¼her 5 m); K_CT daher hochgesetzt, damit der Clamp weiterhin
        // getestet wird: atan2(4.0Â·7.6, 12) â‰ˆ 1.20 rad > XTRACK_MAX_RAD (1.0).
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))]; // offset 0.0
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        ctx.blackboard
            .set("plugin.lane_keeper.nearest_xtrack_k", "4.0");
        // x=7.6 â†’ e_lat = 7.6 âˆ’ 0 = 7.6 (within 50 m radius); v=0 â†’ denom = 12 m (Floor).
        lk.tick_request(Some(&tel_at(7.6, -100.0, 0.0, 0.0)), &ctx);
        let xc = bb_f64(&ctx, "lane_keeper.xtrack_contribution_rad");
        assert!(
            xc.abs() <= XTRACK_MAX_RAD + 1e-9,
            "|xtrack| must be â‰¤ XTRACK_MAX_RAD, got {xc}"
        );
        assert!(
            (xc + XTRACK_MAX_RAD).abs() < 1e-6,
            "truck right + high gain â†’ clamp to âˆ’XTRACK_MAX_RAD, got {xc}"
        );
    }

    #[test]
    fn xtrack_no_spike_at_zero_speed() {
        // v=0, large e_lat â†’ first-tick steering is rate-limited (â‰¤0.1), NO spike (v1-killer).
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        let req = lk.tick_request(Some(&tel_at(7.6, -100.0, 0.0, 0.0)), &ctx);
        let steer = req.expect("must steer").steering.expect("steering set");
        assert!(
            steer.abs() <= 0.1 + 1e-9,
            "first tick must be rate-limited â‰¤0.1, got {steer}"
        );
    }

    #[test]
    fn kct_zero_falls_back_to_heading() {
        // K_CT=0 â†’ no cross-track term; steering = pure heading alignment (safe fallback).
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        ctx.blackboard
            .set("plugin.lane_keeper.nearest_xtrack_k", "0.0");
        // Truck far off the soll-line (x=15) but aligned North â†’ e_lat large, xtrack MUST be 0.
        lk.tick_request(Some(&tel_at(15.0, -100.0, 0.0, 10.0)), &ctx);
        let e_lat = bb_f64(&ctx, "lane_keeper.xtrack_e_lat_m");
        let xc = bb_f64(&ctx, "lane_keeper.xtrack_contribution_rad");
        let hc = bb_f64(&ctx, "lane_keeper.heading_contribution_rad");
        assert!(
            e_lat.abs() > 5.0,
            "truck far off-line â†’ e_lat large, got {e_lat}"
        );
        assert!(
            xc.abs() < 1e-6,
            "K_CT=0 â†’ cross-track contribution must be 0 despite e_lat, got {xc}"
        );
        assert!(
            hc.abs() < 0.05,
            "aligned truck â†’ heading contribution â‰ˆ 0 (steering pure-heading), got {hc}"
        );
    }

    // â”€â”€ Option-3 Cross-Track-PI + Heading-P split-controller tests â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //
    // A 1-lane north road â†’ soll-line at x = lane_offset = 0.0.
    // The truck rides on a north segment whose right normal is +x, so
    //     e_lat = truck_x âˆ’ 0.0.
    // All ticks use v=0 â†’ look_ahead = NEAREST_MIN_LOOK_AHEAD = 12 m (Kriechtempo-Floor)
    // â†’ denom = 12 (no v in the denominator; deterministic P and a fixed lookahead). dt_s = 0.02
    // (PluginContext default, verified), so the integral step per tick is e_latÂ·0.02.

    /// Closed-loop disturbance rejection â€” the integral's defining job. A first-order
    /// integrator plant is pushed every tick by a constant lateral "wind" disturbance D
    /// (m/tick). Under pure v3-P (ki=0) the loop can only balance D against the P term,
    /// so it PINS at a nonzero steady-state residual e_ss â‰ˆ atanâ»Â¹(D/PLANT_GAIN)Â·denom/k.
    /// The dedicated cross-track integral (ki>0) winds up to deliver exactly that steering
    /// at e_lat=0, so the line is HELD (residual driven to ~0). This is the textbook proof
    /// that the integral closes a steady-state error the v3-P term structurally cannot â€”
    /// the same mechanism that, in the live loop, frees e_lat from its â‰ˆlook/k pin.
    ///
    /// NB: a plain integrator plant with NO disturbance has zero steady-state error even
    /// under pure P (an integrator in the plant already removes step error), so the v3-P
    /// pin only becomes observable once a persistent disturbance/bias is present â€” hence D.
    #[test]
    fn xtrack_converges_to_zero() {
        // x_next = x + steeringÂ·PLANT_GAIN + DISTURB. Truck right of line â†’ steering<0 â†’
        // x sinks â†’ e_lat sinks; DISTURB pushes the other way. PLANT_GAIN, DISTURB and the
        // tick budget are chosen so: (a) the v3-P equilibrium e_ss â‰ˆ 0.5 m stays well inside
        // the 3 m integral gate, (b) the ki>0 loop has ample time to wind its integral up to
        // integ_ss = D/(PLANT_GAINÂ·k_i) â‰ˆ 0.63 (< CT_INTEG_MAX) and settle without
        // limit-cycling (k_effÂ·PLANT_GAIN well below the discrete first-order stability bound).
        const PLANT_GAIN: f64 = 0.6;
        const SOLL: f64 = 0.0;
        const DISTURB: f64 = 0.03; // constant lateral push (m/tick), keeps e_lat inside the gate

        fn run(ki: f64) -> f64 {
            let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
            let metas = vec![Some(road_meta(1, 3.75, false))];
            let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
            ctx.blackboard
                .set("plugin.lane_keeper.nearest_xtrack_ki", ki.to_string());
            let mut x = SOLL; // start ON the line; only the disturbance drives e_lat off
            for _ in 0..2000 {
                let req = lk.tick_request(Some(&tel_at(x, -100.0, 0.0, 0.0)), &ctx);
                let steer = req.expect("must steer").steering.expect("steering set");
                x += steer * PLANT_GAIN + DISTURB;
            }
            bb_f64(&ctx, "lane_keeper.xtrack_e_lat_m")
        }

        let e_ki = run(NEAREST_XTRACK_KI_DEFAULT); // ki = 0.08 (PI)
        let e_p_only = run(0.0); // pure v3-P

        // PI run: disturbance rejected, line HELD near 0 (well below the v3-P pin).
        assert!(
            e_ki.abs() < 0.3,
            "PI run must reject the disturbance (e_latâ†’~0, line held), got {e_ki}"
        );
        // P-only run pins at a clearly larger steady-state residual it cannot close.
        assert!(
            e_p_only.abs() > e_ki.abs() + 0.1,
            "ki=0 must pin at a clearly larger steady-state residual than ki>0 \
             (integral closes the gap): p_only={e_p_only}, pi={e_ki}"
        );
    }

    /// Anti-windup gate: with |e_lat| > 3 m the integral store is held hard at 0.
    #[test]
    fn integ_gated_during_large_error() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // x = 4.0 â†’ e_lat = 4.0 m > CT_INTEG_GATE_M (3.0).
        for _ in 0..5 {
            lk.tick_request(Some(&tel_at(4.0, -100.0, 0.0, 0.0)), &ctx);
        }
        let e_lat = bb_f64(&ctx, "lane_keeper.xtrack_e_lat_m");
        assert!(
            e_lat.abs() > CT_INTEG_GATE_M,
            "setup: e_lat must exceed the gate, got {e_lat}"
        );
        assert_eq!(
            lk.xtrack_integ, 0.0,
            "outside the gate the integral store must stay 0, got {}",
            lk.xtrack_integ
        );
    }

    /// Integral anti-windup clamp: held just inside the gate for many ticks, the
    /// integral would grow past CT_INTEG_MAX but is clamped.
    #[test]
    fn integ_clamped() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // x = 2.8 â†’ e_lat = 2.8 m (inside the 3.0 m gate). Truck is
        // FIXED (no plant), so the integral keeps accumulating e_latÂ·dt every tick:
        // 500Â·2.8Â·0.02 = 28 â‰« CT_INTEG_MAX (5.0) â†’ must saturate at the clamp.
        for _ in 0..500 {
            lk.tick_request(Some(&tel_at(2.8, -100.0, 0.0, 0.0)), &ctx);
        }
        assert!(
            lk.xtrack_integ.abs() <= CT_INTEG_MAX + 1e-9,
            "integral must be clamped to Â±CT_INTEG_MAX, got {}",
            lk.xtrack_integ
        );
        // Sanity: it actually drove into the clamp (not stuck near 0).
        assert!(
            (lk.xtrack_integ.abs() - CT_INTEG_MAX).abs() < 1e-6,
            "integral should sit at the clamp after 500 ticks, got {}",
            lk.xtrack_integ
        );
    }

    /// Disengage resets the integral: accumulate inside the gate (integ â‰  0), then a
    /// single tick in a non-Active state must zero the store.
    #[test]
    fn integ_resets_on_disengage() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // A few active ticks at e_lat = 2.0 m (inside gate) â†’ integral winds up.
        for _ in 0..5 {
            lk.tick_request(Some(&tel_at(2.0, -100.0, 0.0, 0.0)), &ctx);
        }
        assert!(
            lk.xtrack_integ.abs() > 1e-6,
            "setup: integral must be non-zero before disengage, got {}",
            lk.xtrack_integ
        );
        // Flip to a non-Active state (Off) â†’ the !is_active branch must reset.
        ctx.blackboard.set("autopilot.state", "Off");
        lk.tick_request(Some(&tel_at(2.0, -100.0, 0.0, 0.0)), &ctx);
        assert_eq!(
            lk.xtrack_integ, 0.0,
            "disengage (state != Active) must reset the integral, got {}",
            lk.xtrack_integ
        );
    }

    /// The heading path is pure-P (no integral): a constant heading error must NOT
    /// grow the steering output over time (no heading windup), and with e_latâ‰ˆ0 the
    /// cross-track integral stays small (no cross-track windup either).
    #[test]
    fn heading_path_no_integral() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // Truck ON the soll-line (x = 0.0 â†’ e_lat â‰ˆ 0) but yawed ~20Â° off North.
        // ETS2 heading 0..1 CCW: 20Â° CW from North = 1 âˆ’ 20/360 = 0.9444 â†’ inside the
        // 60Â° filter, so the segment stays selected and heading_err is constant.
        let tel = tel_at(0.0, -100.0, 0.9444, 0.0);
        let mut steer_late = Vec::new();
        for tick in 0..50 {
            let req = lk.tick_request(Some(&tel), &ctx);
            req.expect("must steer").steering.expect("steering set");
            if tick >= 5 {
                steer_late.push(bb_f64(&ctx, "lane_keeper.steering_out"));
            }
        }
        // Compare two late, settled ticks: a pure-P heading path is constant once the
        // rate-limiter has caught up; an integral would keep ramping.
        let s_early = steer_late[0]; // tick 5
        let s_late = *steer_late.last().unwrap(); // tick 49
        assert!(
            (s_late - s_early).abs() < 1e-3,
            "pure-P heading path: steering must not ramp (no integral), \
             tick5={s_early}, tick49={s_late}"
        );
        // e_lat â‰ˆ 0 â†’ no cross-track windup.
        assert!(
            lk.xtrack_integ.abs() < 0.1,
            "on-line truck â†’ cross-track integral stays small, got {}",
            lk.xtrack_integ
        );
    }

    /// P+I are jointly clamped: high K_CT (2.0) AND high K_CT_I (1.0) with e_lat inside
    /// the gate, accumulated over many ticks, must never push |xtrack_contribution|
    /// past XTRACK_MAX_RAD on any tick.
    #[test]
    fn total_xtrack_bounded() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        ctx.blackboard
            .set("plugin.lane_keeper.nearest_xtrack_k", "2.0");
        ctx.blackboard
            .set("plugin.lane_keeper.nearest_xtrack_ki", "1.0");
        // x = 2.5 â†’ e_lat = 2.5 m (inside the 3.0 m gate). Truck FIXED,
        // so the integral keeps climbing to its clamp; check the joint bound every tick.
        for _ in 0..400 {
            lk.tick_request(Some(&tel_at(2.5, -100.0, 0.0, 0.0)), &ctx);
            let xc = bb_f64(&ctx, "lane_keeper.xtrack_contribution_rad");
            assert!(
                xc.abs() <= XTRACK_MAX_RAD + 1e-9,
                "P+I jointly clamped: |xtrack| â‰¤ XTRACK_MAX_RAD, got {xc}"
            );
        }
    }

    /// Low-speed safety: at v=0 the denominator is max(look, BASE) = NEAREST_MIN_LOOK_AHEAD
    /// = 12 m (Kriechtempo-Floor, never v), so the cross-track P term cannot saturate from a
    /// vanishing denominator and the truck does not spiral. First-tick steering stays
    /// rate-limited and the cross-track contribution stays below the hard clamp.
    #[test]
    fn no_stall_at_low_speed() {
        let segs = vec![north_seg(0.0, 0.0, -200.0, 10, 20)];
        let metas = vec![Some(road_meta(1, 3.75, false))];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // x = 1.5 â†’ e_lat = 1.5 m (inside gate), v = 0.
        let req = lk.tick_request(Some(&tel_at(1.5, -100.0, 0.0, 0.0)), &ctx);
        let steer = req.expect("must steer").steering.expect("steering set");
        assert!(
            steer.abs() <= 0.1 + 1e-9,
            "first tick rate-limited â‰¤0.1, got {steer}"
        );
        // denom = max(look, BASE) = NEAREST_MIN_LOOK_AHEAD = 12 (Kriechtempo-Floor, no v in
        // the denominator) â†’ xtrack_p = atan2(0.5Â·1.5, 12) â‰ˆ 0.062 rad, far from XTRACK_MAX_RAD
        // â†’ no atan(offset/look) saturation, the stall-spiral stays away.
        let xc = bb_f64(&ctx, "lane_keeper.xtrack_contribution_rad");
        assert!(
            xc.abs() < XTRACK_MAX_RAD,
            "low-speed cross-track must not saturate, got {xc}"
        );
    }

    /// Regression guard: a RouteFollowing tick (mode != nearest_spline) must leave the
    /// NearestSpline cross-track integral untouched (== 0) while still steering â€”
    /// proving the new integral is isolated to the NearestSpline path. (Named
    /// `*_integral_isolated` because the bare `route_following_unchanged` already
    /// exists for the no-waypoints None path.)
    #[test]
    fn route_following_integral_isolated() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active"); // no plugin.lane_keeper.mode â†’ RouteFollowing
        let req = lk
            .tick_request(Some(&t), &ctx)
            .expect("route-following must request");
        let s = req.steering.expect("route-following must steer");
        assert!(
            s != 0.0,
            "route-following must still produce steering, got {s}"
        );
        assert_eq!(
            lk.xtrack_integ, 0.0,
            "RouteFollowing must not touch the NearestSpline cross-track integral, got {}",
            lk.xtrack_integ
        );
    }

    // â”€â”€ Laterales Engage-Gate â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// Hilfsfunktion: baut einen minimalen HeadingFilteredHit mit gegebener Distanz.
    fn make_hit(dist_m: f32) -> HeadingFilteredHit {
        HeadingFilteredHit {
            idx: 0,
            dist_m,
            t: 0.5,
            point_on_curve: Vec3::new(0.0, 0.0, 0.0),
            heading_diff_rad: 0.0,
            meta: None,
        }
    }

    /// Hilfsfunktion: erstellt PluginContext mit gesetztem engage_max_lateral_m BB-Key.
    fn ctx_with_lateral_limit(limit_m: f32) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("lane_keeper.engage_max_lateral_m", format!("{limit_m}"));
        PluginContext::new("lane-keeper", bb)
    }

    #[test]
    fn engage_max_lateral_m_loaded_from_blackboard() {
        // on_load liest engage_max_lateral_m aus dem toml-geseedten BB-Key.
        let mut plugin = LaneKeeperPlugin::default();
        assert_eq!(
            plugin.engage_max_lateral_m, DEFAULT_ENGAGE_MAX_LATERAL_M,
            "Default muss 3.0 sein"
        );
        let ctx = ctx_with_lateral_limit(7.5);
        plugin.on_load(&ctx);
        assert!(
            (plugin.engage_max_lateral_m - 7.5).abs() < 1e-5,
            "on_load muss 7.5 aus BB laden, got {}",
            plugin.engage_max_lateral_m
        );
    }

    #[test]
    fn engage_max_lateral_m_fallback_when_key_absent() {
        // Kein BB-Key â†’ Fallback auf DEFAULT_ENGAGE_MAX_LATERAL_M (3.0).
        let mut plugin = LaneKeeperPlugin::default();
        let ctx = fresh_ctx();
        plugin.on_load(&ctx);
        assert_eq!(
            plugin.engage_max_lateral_m, DEFAULT_ENGAGE_MAX_LATERAL_M,
            "Fehlender Key muss auf Default 3.0 fallen"
        );
    }

    #[test]
    fn engage_gate_lateral_too_far_sets_block_reason() {
        // dist=10m: unter 45m (altes Gate ok), Ã¼ber 3m (laterales Gate verletzt)
        // â†’ engage_block_reason="lateral_too_far"
        let plugin = LaneKeeperPlugin {
            engage_max_lateral_m: 3.0,
            ..Default::default()
        };
        // engage_allowed=false, h.dist_m=10.0 < 45.0 â†’ lateral_too_far
        let ctx = fresh_ctx();
        // Kein SplineIndex â†’ rufe publish_engage_gate_diag direkt mit fake SplineIndex auf.
        // Stattdessen prÃ¼fen wir die Gate-Logik Ã¼ber die Bool-Bedingung inline.
        let h = make_hit(10.0);
        let engage_allowed =
            h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= plugin.engage_max_lateral_m;
        assert!(
            !engage_allowed,
            "10m > 3m-Limit â†’ engage_allowed muss false sein"
        );

        let block_reason = if engage_allowed {
            "ok"
        } else if h.dist_m >= NEAREST_ENGAGE_DIST_M {
            "too_far"
        } else {
            "lateral_too_far"
        };
        assert_eq!(block_reason, "lateral_too_far");
        let _ = ctx; // silence unused warning
    }

    #[test]
    fn engage_gate_lateral_ok_allows_engage() {
        // dist=1.5m: unter 45m UND unter 3m â†’ engage_allowed=true
        let plugin = LaneKeeperPlugin {
            engage_max_lateral_m: 3.0,
            ..Default::default()
        };
        let h = make_hit(1.5);
        let engage_allowed =
            h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= plugin.engage_max_lateral_m;
        assert!(
            engage_allowed,
            "1.5m <= 3m-Limit â†’ engage_allowed muss true sein"
        );

        let block_reason = if engage_allowed {
            "ok"
        } else if h.dist_m >= NEAREST_ENGAGE_DIST_M {
            "too_far"
        } else {
            "lateral_too_far"
        };
        assert_eq!(block_reason, "ok");
    }

    #[test]
    fn engage_gate_too_far_not_masked_by_lateral() {
        // dist=50m: Ã¼ber 45m â†’ "too_far", nicht "lateral_too_far"
        // Stellt sicher dass too_far Vorrang hat (block_reason-Reihenfolge korrekt).
        let plugin = LaneKeeperPlugin {
            engage_max_lateral_m: 3.0,
            ..Default::default()
        };
        let h = make_hit(50.0);
        let engage_allowed =
            h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= plugin.engage_max_lateral_m;
        assert!(!engage_allowed);

        let block_reason = if engage_allowed {
            "ok"
        } else if h.dist_m >= NEAREST_ENGAGE_DIST_M {
            "too_far"
        } else {
            "lateral_too_far"
        };
        assert_eq!(
            block_reason, "too_far",
            "50m muss too_far liefern, nicht lateral_too_far"
        );
    }

    #[test]
    fn engage_gate_boundary_at_limit_exact() {
        // dist == engage_max_lateral_m (3.0): genau an der Grenze â†’ allowed (<=)
        let plugin = LaneKeeperPlugin {
            engage_max_lateral_m: 3.0,
            ..Default::default()
        };
        let h = make_hit(3.0);
        let engage_allowed =
            h.dist_m < NEAREST_ENGAGE_DIST_M && h.dist_m <= plugin.engage_max_lateral_m;
        assert!(
            engage_allowed,
            "dist==limit (3.0m) muss noch erlaubt sein (<=)"
        );
    }

    // â”€â”€ Junction-Fix Tests (Fix 1/2/3) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

    /// Test A â€” advance_forward_adj Route-Guard: bei aktiver Route gewinnt der
    /// Route-Nachfolger (idx 2, Kink 45Â°) Ã¼ber den geometrisch geradeaus-sten
    /// Off-Route-Kandidaten (idx 1, Kink â‰ˆ0Â°).
    #[test]
    fn advance_fwd_route_guard_picks_route_successor() {
        // A (idx 0): Nord, UID 10â†’20
        // B (idx 1): Nord (gerade, Kink â‰ˆ0Â°), UID 20â†’30 â€” NICHT auf Route
        // C (idx 2): NE 45Â°, UID 20â†’40 â€” AUF Route (knik 45Â°, geometrisch knickreicher)
        // route_seg_set = {2} â†’ C muss gewinnen trotz grÃ¶ÃŸerem Kink
        let segs = vec![
            north_seg(0.0, 0.0, -90.0, 10, 20),        // A idx 0
            north_seg(0.0, -90.0, -200.0, 20, 30),     // B gerade idx 1
            seg((0.0, -90.0), (63.6, -153.6), 20, 40), // C 45Â° NE idx 2
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        lk.cached_route_seg_set.insert(2); // nur C ist auf der Route

        lk.tick_request(Some(&tel_at(0.0, -45.0, 0.0, 10.0)), &ctx); // auf A einrasten
        lk.tick_request(Some(&tel_at(0.0, -82.0, 0.0, 10.0)), &ctx); // Segmentende â†’ advance
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("2"),
            "Route-Guard: On-Route C (idx 2, Kink 45Â°) muss Off-Route B (idx 1, Kink 0Â°) schlagen"
        );
        // Route-Guard filtert B aus â†’ nur C bleibt â†’ count=1 â†’ fwd_progress (korrekt)
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.nearest_seg_switch_reason")
                .as_deref(),
            Some("fwd_progress"),
            "nach Route-Guard ein Kandidat Ã¼brig â†’ fwd_progress"
        );
    }

    /// Test B â€” leeres route_set: altes Kink-Verhalten bleibt erhalten.
    /// Exakt selbes Setup wie Test A, aber ohne route_set â†’ B (Kink â‰ˆ0Â°) gewinnt.
    #[test]
    fn advance_fwd_empty_route_set_keeps_kink_behavior() {
        let segs = vec![
            north_seg(0.0, 0.0, -90.0, 10, 20),
            north_seg(0.0, -90.0, -200.0, 20, 30), // B gerade idx 1
            seg((0.0, -90.0), (63.6, -153.6), 20, 40), // C 45Â° idx 2
        ];
        let metas = vec![
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
            Some(road_meta(1, 3.75, false)),
        ];
        let (mut lk, ctx) = nearest_wired(segs, metas, "Active");
        // cached_route_seg_set bleibt leer (kein insert)

        lk.tick_request(Some(&tel_at(0.0, -45.0, 0.0, 10.0)), &ctx);
        lk.tick_request(Some(&tel_at(0.0, -82.0, 0.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.chain_segment_idx")
                .as_deref(),
            Some("1"),
            "Leeres route_set: geradeaus-ster Nachfolger B (idx 1) gewinnt (altes Verhalten)"
        );
    }

    /// Test C â€” Stuck-Watchdog zÃ¤hlt auch bei throttle=0.
    /// Fix 3 entfernte den throttle_cmd > 0.05-Gate. Der Watchdog muss jetzt
    /// auch nach einem Crash-Stop (throttle=0) zÃ¤hlen und Recovery auslÃ¶sen.
    #[test]
    fn stuck_watchdog_counts_without_throttle() {
        let (mut lk, ctx) = capture_wired();
        ctx.blackboard.set("speed_controller.throttle_cmd", "0.0"); // Crash-Stop: throttle=0

        for _ in 0..60 {
            lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 0.1)), &ctx);
        }
        assert_eq!(
            ctx.blackboard.get("lane_keeper.stuck_recovery").as_deref(),
            Some("true"),
            "Watchdog muss nach >50 Ticks auch bei throttle=0 auslÃ¶sen (Fix 3)"
        );
    }

    /// Test D â€” Stuck-Watchdog Disengage nach STUCK_DISENGAGE_TICKS.
    /// Nach Eintritt in Recovery ohne Fortschritt â†’ autopilot.disengage_requested.
    #[test]
    fn stuck_watchdog_disengages_after_timeout() {
        let (mut lk, ctx) = capture_wired();
        ctx.blackboard.set("speed_controller.throttle_cmd", "0.0");

        // STUCK_TICKS (50) + STUCK_DISENGAGE_TICKS (150) + 5 Puffer = 205 Ticks
        for _ in 0..205 {
            lk.tick_request(Some(&tel_at(20.0, -100.0, 0.0, 0.1)), &ctx);
        }
        assert_eq!(
            ctx.blackboard
                .get("autopilot.disengage_requested")
                .as_deref(),
            Some("true"),
            "Stuck-Watchdog muss nach STUCK_TICKS + STUCK_DISENGAGE_TICKS disengagen (Fix 2)"
        );
    }

    // â”€â”€ Phase Reanchor-Window: windowed scan (REANCHOR_BACKWARD_WINDOW / FORWARD_WINDOW) â”€â”€
    //
    // These guard the windowed reanchor fix: laufender Reanchor darf node_progress_idx
    // nicht auf k=0 zurÃ¼cksetzen, wenn der Truck bereits bei k=4 steht und das nÃ¤chste
    // Segment nur feeds_into route[0] (Jitter / off-route Segment am Ortsrand).

    /// Test RW-1 (Regression): node_progress_idx=4 vorgesetzt; nÃ¤chstes Segment (70â†’10)
    /// ist off-route und feeds_into route[0]=10 â€” alter Code wÃ¼rde auf k=0 zurÃ¼ckspringen.
    /// Der neue windowed Scan hat scan_lo=max(0,4-2)=2: k=0 liegt auÃŸerhalb des Fensters,
    /// feeds_into wird nicht gefunden â†’ off_route â†’ node_progress_idx bleibt 4.
    ///
    /// Route `[10,20,30,40,50,60]`:
    ///   Hops 0-4 laufen Nord an x=0 (Truck steht 200 m weiter Ã¶stlich â†’ dist > 50 m gate).
    ///   Segment 70â†’10 an x=200 lÃ¤uft ebenfalls Nord, direkt unter dem Truck.
    #[test]
    fn reanchor_window_blocks_feeds_into_k0_when_progress_is_4() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),         // hop 0
            seg((0.0, -100.0), (0.0, -200.0), 20, 30),      // hop 1
            seg((0.0, -200.0), (0.0, -300.0), 30, 40),      // hop 2
            seg((0.0, -300.0), (0.0, -400.0), 40, 50),      // hop 3
            seg((0.0, -400.0), (0.0, -500.0), 50, 60),      // hop 4
            seg((200.0, -390.0), (200.0, -410.0), 70, 10),  // off-route, feeds_into route[0]=10
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
            (40u64, 0.0, -300.0),
            (50u64, 0.0, -400.0),
            (60u64, 0.0, -500.0),
            (70u64, 200.0, -390.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (30u64, 40u64, 100.0),
            (40u64, 50u64, 100.0),
            (50u64, 60u64, 100.0),
            (70u64, 10u64, 20.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30,40,50,60]", 5);

        // Tick 1: Truck auf hop 4 (x=0, z=-450). route_changed=true â†’ globaler Scan.
        // Nearest = hop 4 (50â†’60), node_progress_idx sollte auf 4 gesetzt werden.
        lk.compute_heading_error(0.0, -450.0, 0.0, 0.0, &ctx);
        assert_eq!(
            lk.node_progress_idx, 4,
            "tick 1: Erst-Anchor muss hop 4 treffen; got {}",
            lk.node_progress_idx
        );

        // Tick 2: Truck 200 m Ã¶stlich (x=200, z=-400) â€” alle on-route Segmente > 50 m entfernt.
        // Global-nearest = 70â†’10 (feeds_into route[0]=10, k=0).
        // scan_lo = max(0, 4-2) = 2 â†’ k=0 liegt auÃŸerhalb â†’ off_route â†’ kein Schreiben.
        lk.compute_heading_error(200.0, -400.0, 0.0, 0.0, &ctx);
        // node_progress_idx darf nicht auf 0 fallen â€” off_route-Pfad Ã¼berschreibt es nicht.
        assert_eq!(
            lk.node_progress_idx, 4,
            "windowed scan muss k=0 feeds_into blockieren; node_progress_idx darf nicht auf 0 fallen; got {}",
            lk.node_progress_idx
        );
        // off_route-Pfad muss fallback_reason setzen (Beweis, dass das Segment verworfen wurde).
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("off_route"),
            "segment 70â†’10 muss als off_route klassifiziert werden (k=0 liegt auÃŸerhalb Fenster)"
        );
    }

    /// Test RW-2 (Backward-Toleranz): node_progress_idx=4 vorgesetzt; nearest ist on-route
    /// hop 2 (30â†’40), der innerhalb des RÃ¼ckwÃ¤rts-Fensters liegt (4-2=2 â‰¤ k=2 â‰¤ 4).
    /// Ergebnis: node_progress_idx=2 (kleiner RÃ¼cksprung toleriert), nicht 0 oder 4.
    ///
    /// Route `[10,20,30,40,50]`:
    ///   Nur hop 2 (30â†’40) liegt unter dem Truck (x=0, z=-200..-300).
    ///   Alle anderen Hops liegen weit Ã¶stlich (x=500).
    #[test]
    fn reanchor_window_backward_tolerance_bounded_at_scan_lo() {
        let segs = vec![
            seg((500.0, 0.0), (500.0, -100.0), 10, 20),     // hop 0, far east
            seg((500.0, -100.0), (500.0, -200.0), 20, 30),  // hop 1, far east
            seg((0.0, -200.0), (0.0, -300.0), 30, 40),      // hop 2, under truck
            seg((500.0, -300.0), (500.0, -400.0), 40, 50),  // hop 3, far east
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 500.0, 0.0),
            (20u64, 500.0, -100.0),
            (30u64, 0.0, -200.0),
            (40u64, 0.0, -300.0),
            (50u64, 500.0, -400.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 500.0),
            (30u64, 40u64, 100.0),
            (40u64, 50u64, 500.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30,40,50]", 4);

        // Tick 1: Truck auf hop 2 (x=0, z=-250). route_changed=true â†’ Erst-Anchor â†’ k=2.
        lk.compute_heading_error(0.0, -250.0, 0.0, 0.0, &ctx);
        assert_eq!(lk.node_progress_idx, 2, "Erst-Anchor muss hop 2 setzen");

        // Manuell auf k=4 setzen (Simulation: Truck war weiter vorne, jetzt jittert er zurÃ¼ck).
        lk.node_progress_idx = 4;

        // Tick 2: gleiche Position, scan_lo=max(0,4-2)=2. hop 2 liegt bei k=2 â†’ on_route gefunden.
        // Ergebnis: 2 (RÃ¼cksprung toleriert, aber nicht unter scan_lo=2 und nicht 0).
        lk.compute_heading_error(0.0, -250.0, 0.0, 0.0, &ctx);
        assert_eq!(
            lk.node_progress_idx, 2,
            "Backward-Toleranz: on_route bei k=2 muss gefunden werden; got {}",
            lk.node_progress_idx
        );
    }

    /// Test RW-3 (Erst-Anchor bleibt global): route_changed=true â†’ scan_lo=0, feeds_into
    /// bei k=0 wird akzeptiert. PrÃ¼ft, dass der Erst-Anchor-Sonderfall nicht vom Fenster
    /// blockiert wird.
    ///
    /// Identisch zu VB-1, aber prÃ¼ft explizit scan_window = "0..N".
    #[test]
    fn reanchor_erst_anchor_uses_global_scan() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),         // predecessor Pâ†’A, unter Truck
            seg((0.0, -100.0), (200.0, -100.0), 20, 30),    // hop Aâ†’B, weit Ã¶stlich
            seg((200.0, -100.0), (200.0, -300.0), 30, 40),  // hop Bâ†’C
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 200.0, -100.0),
            (40u64, 200.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 200.0),
            (30u64, 40u64, 200.0),
        ];
        // Route beginnt bei A=20, predecessor Pâ†’A ist nicht in der Route.
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,30,40]", 3);

        // Erster Tick: route_changed=true â†’ globaler Scan. nearest=10â†’20 (feeds_into route[0]=20).
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 0,
            "Erst-Anchor muss feeds_into bei k=0 akzeptieren (globaler Scan); got {}",
            lk.node_progress_idx
        );
        let win = ctx
            .blackboard
            .get("lane_keeper.reanchor_scan_window")
            .unwrap_or_default();
        assert!(
            win.starts_with("0.."),
            "Erst-Anchor muss '0..' (globalen Scan) verwenden; got '{win}'"
        );
    }
}

