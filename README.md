# ANAFI FreeFlight 6 Planner

Android app base focused on offline photogrammetry planning for Parrot ANAFI and export to FreeFlight 6 `savedPlan.json`.

Current scope:

- configurable ANAFI camera profiles;
- grid survey metrics based on FOV and overlap;
- timing validation against observed FreeFlight photo-period limits;
- grid waypoint generation with boustrophedon ordering;
- JSON export preview for FreeFlight 6;
- simple Android shell around a fixed demo polygon.

Explicitly out of scope in this phase:

- GroundSDK/SkyController control;
- real map drawing;
- terrain-follow DEM pipeline;
- Circlegrammetry runtime UI.

Next steps:

1. replace the demo polygon with an editable map;
2. add takeoff point and terrain-follow models;
3. add Circlegrammetry and hybrid mission generators;
4. export/share `savedPlan.json` through SAF.
