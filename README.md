# Flipper Trail

GPS route recorder and mapper for the Flipper Zero (Momentum firmware), using a GPS module on the GPIO header.

- Live GPS stats, route map (wide or held upright), elevation, saved locations and routing
- Routes are stored in small per-area tile files, so only the area you are in is loaded
- Settings: map view/facing/screen, zoom (20 m to 1.6 km), time zone and time trim, and more

## Build

Every push builds the app with [uFBT](https://github.com/flipperdevices/flipperzero-ufbt) against the
Momentum SDK (see `.github/workflows/build.yml`). Download the `.fap` from the run's **Artifacts**.

Locally: `pip install ufbt && ufbt update --index-url=https://up.momentum-fw.dev/firmware/directory.json --channel release && ufbt`

Copy `dist/flipper_trail.fap` to `SD/apps/GPS/` (category `mbm/GPS`).

The workflow checks the SDK API version against `expected_api` (default 87.1, matching the firmware on my Flipper).
Use **Run workflow** to pick another channel or pin an exact SDK.
