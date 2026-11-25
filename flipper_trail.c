/* Flipper Trail: GPS route recorder and mapper.
   The app is one translation unit; the modules in src/ are #included in this order
   (later modules use earlier ones; see DEV_NOTES.md for what is in each). */
#include "src/01_types_and_config.inc"
#include "src/02_geometry_and_tiles.inc"
#include "src/03_waypoints_and_routing.inc"
#include "src/04_position_time_gps.inc"
#include "src/05_settings.inc"
#include "src/06_ui_helpers.inc"
#include "src/07_map_view.inc"
#include "src/08_route_page.inc"
#include "src/09_cleanup_routes.inc"
#include "src/10_app_loop.inc"
