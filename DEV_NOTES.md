# Dev notes

Handoff notes so work can continue from anywhere (including a phone, with only GitHub access).

## Workflow

- **This repo is the workspace.** On the PC it lives at `/home/opforgood/flipper-trail` and is symlinked to
  `~/Momentum-Firmware/applications_user/flipper_trail`, so `./fbt fap_flipper_trail` builds it. Keep stray files
  (`*.bak`, `*.c.old`) out of this folder: the app build globs `*.c*` and will try to link them.
- **CI:** every push builds with uFBT against the Momentum SDK (`.github/workflows/build.yml`).
  - The `.fap` is the `flipper-trail-fap` artifact of the run.
  - Pushes to `main` also refresh the pre-release **`latest`** with the `.fap` attached (easy download on a phone).
  - Every push writes `STATUS.md` on the orphan branch **`ci-status`**: result, compiler warning count, last 150
    lines of the build log. After pushing, wait ~1 min, then read it (`get_file_contents`, ref `ci-status`,
    path `STATUS.md`) to see whether the build passed and any compiler errors.
- **Editing without a local checkout:** the GitHub API has no patch call, so an edit means sending the whole
  file. That is why the source is split into modules of 9-24 KB under `src/`. Edit the one module that holds the
  change; use `push_files` for one commit that touches several files. Do not rewrite `flipper_trail.c`
  (it only holds the include list).
- **Build is strict:** warnings are errors. `snprintf` buffers must fit the worst case (a `%d` can need 12
  bytes, a `%s` of a `char[32]` needs 32) or `-Wformat-truncation` fails the build; use `strlcpy` for copies.
- CI checks the SDK API version against `expected_api` (87.1, the owner's firmware). If Momentum's release
  channel moves past it, build with **Run workflow** and an exact `sdk_url`, or update the default.
- Installing: copy `flipper_trail.fap` to `SD/apps/GPS/` (app category `mbm/GPS`).

## Module map (one translation unit; `flipper_trail.c` includes these in order, later ones use earlier ones)

| File | Contents |
|---|---|
| `01_types_and_config.inc` | includes, constants, all structs (`Trail`, `TrailSettings`, `RTile`, `TileStore`...), `SettingsPage` enum, math helpers (`dist_m`, `global_xy`, `lb_clip`), route tile file I/O (`rtile_*`) |
| `02_geometry_and_tiles.inc` | segment geometry, route simplification/merging (`dp_simplify`, `rpts_*`), the 3x3 resident tile store (`tilestore_*`), recording points (`rt_record_point`), free-move view store (`viewstore_*`) |
| `03_waypoints_and_routing.inc` | toast, saved locations (`wp_*`), routing over saved routes (`nav_*`, `nav_compute`) |
| `04_position_time_gps.inc` | last known position, time zone lookup (`tz_*`), UTC clock and trim (`utc_*`), NMEA parsing (`parse_line`, `rx_callback`), live point appending |
| `05_settings.inc` | settings defaults/load/save, `settings_change` (OK), `settings_nudge` (Up), value text, trim and destination helpers |
| `06_ui_helpers.inc` | tab names, unit formatting, page predicates, upright-screen helpers (`canvas_is_tall`, `body_bottom`, `trail_orientation`), headers and footers |
| `07_map_view.inc` | map drawing: projection (`map_xy`), clipped lines, markers, scale bar, compass, labels (`push_label`), `draw_live_map` |
| `08_route_page.inc` | route page (`draw_route_*`), dashboard/home menu/time/elevation pages, `draw_settings`, `draw_inner` (screen router), `draw`, `input_cb`, recording start, route loading |
| `09_cleanup_routes.inc` | import old CSV recordings, optimizer that merges repeated routes (`optimizer_run`) |
| `10_app_loop.inc` | area enter/leave toast, routing tick, route page edit/rename, free-move and pan, `flipper_trail_app` (main loop and orientation switching) |

## How things work

- **Settings** are saved to `/ext/GPS/settings.bin` as a magic + version byte + the `TrailSettings` struct. New
  fields go at the end (or in padding), the loader accepts older versions, so bump `SETTINGS_VERSION` and
  sanitize or migrate the new field in `settings_load`. History: v10 `map_screen`, v11 `trim_step_idx`,
  v12 zoom now 13 levels (old index 0/1/2 migrates to 4/6/8).
- **Adding a settings page:** add to the `SettingsPage` enum, then `settings_names`, `settings_short`,
  `settings_hints`, `settings_defaults`, `settings_change` (OK), `settings_value_text`, optionally
  `settings_nudge` (Up) and `settings_center_label`.
- **Upright map:** setting "Map screen" (Wide / Tall / Tall flip). `trail_orientation()` picks the viewport
  orientation from the page; the main loop applies it before `view_port_update`; `draw()` skips a frame if the
  canvas does not match yet. Map code reads sizes from the canvas (`canvas_width/height`, `body_bottom()`,
  `map_cx/cy()`); the GUI remaps the arrow keys itself, so input code is unchanged. The Route page stays wide.
- **Zoom:** 13 widths from 20 m to 1.6 km (`zoom_span_m`). Pixel scale is `124 / span`, kept in upright mode, so
  the label means the wide screen. On the zoom setting page OK = wider, Up = closer.
- **Time trim:** `trim_ms` within +-3000 ms, step from `trim_steps` (1, 5, 10, 50, 100 ms; setting "Trim step").
- **Route tiles:** ~350 x 290 m tiles in `/ext/GPS/rtiles`, only the 3x3 around you is resident
  (`TILE_CACHE 9`), so above ~1 km zoom saved routes show only near your position. Other files in `/ext/GPS/`:
  `settings.bin`, `tz.bin` (made by `gen_tz.py` on a PC), saved locations, last position.

## Owner's design decisions

- Hardware: "dr.where" GPS module; Flipper on Momentum firmware.
- The **OK button stays free** on the map for a planned adventure-game port (pause/resume was dropped for this).
- Long-press Back must not exit the app while recording.
- Settings are reached only from the menu and stats screens (not the map). Map free-move is toggled with OK and
  panned with the arrows (with an "off" option); configurable step per tap.
- Local time is shown in the app only (the Flipper's own clock is optional); time zone is automatic from GPS.
- Route page (Left on the map): rename points, pick destination, routing options, start/stop routing.
- Altitude is averaged when two route points are walked on top of each other; the map can face north, the
  heading, or the route direction.
- Saved routes load as actual route lines per tile; the optimizer filters repeated routes.

## Ideas not done yet

- Adventure game port (keep OK free).
- Upright Route page; acceleration while holding Up/OK on the trim page; larger tile cache for very wide zoom.
- `application.fam` still says `fap_author="OpenAI"`.

## Local backups on the PC

`~/flipper_trail_old_folder` (the app folder before the symlink) and `~/flipper_trail.c.pre_vertical.bak`.
