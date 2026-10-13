# F1TENTH tracks

Tracks imported from official F1TENTH repositories. Sim only (no real map).
Each track keeps the license of its source in its own `LICENSE` file.

Launch with `ros2 launch arcsf1tenth_bringup simulation.launch.py track:=<folder>`.

| Folder | Track | Source | Paths | License |
|---|---|---|---|---|
| `berlin` | Berlin | [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | - | MIT |
| `levine` | Levine Hall | [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | - | MIT |
| `skirk` | Skirk | [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | - | MIT |
| `stata_basement` | Stata Basement | [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | - | MIT |
| `vegas` | Vegas | [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | - | MIT |
| `austin` | Austin (Circuit of the Americas) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `brands_hatch` | Brands Hatch | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `budapest` | Budapest (Hungaroring) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `catalunya` | Catalunya (Barcelona) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `hockenheim` | Hockenheim | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `ims` | Indianapolis Motor Speedway | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `melbourne` | Melbourne (Albert Park) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `mexico_city` | Mexico City (Hermanos Rodriguez) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `montreal` | Montreal (Gilles Villeneuve) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline | GPL-3.0 |
| `monza` | Monza | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `moscow_raceway` | Moscow Raceway | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `nuerburgring` | Nuerburgring | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `oschersleben` | Oschersleben | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `sakhir` | Sakhir (Bahrain) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `sao_paulo` | Sao Paulo (Interlagos) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `sepang` | Sepang | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `shanghai` | Shanghai | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline | GPL-3.0 |
| `silverstone` | Silverstone | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `sochi` | Sochi | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `spa` | Spa-Francorchamps | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `spielberg` | Spielberg (Red Bull Ring) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `yas_marina` | Yas Marina (Abu Dhabi) | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |
| `zandvoort` | Zandvoort | [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks) | centerline, raceline | GPL-3.0 |

The 1:10 F1 circuits have a fixed 2.2 m track width; `centerline.csv` and `raceline.csv`
are in the same frame as the map/world. Montreal and Shanghai have no raceline upstream.

To add another track: copy the map image/yaml to `<track>/maps/sim/map.png/.yaml`
(fix the `image:` field), copy any centerline/raceline CSVs, write `track.yaml`
(see an existing one), then generate the world:

```
ros2 run arcsf1tenth_gazebo map_to_world.py <track>/maps/sim/map.yaml <track>/world/world.sdf --name <track>
```

Add `--unknown-is-wall` if corridors end in grey (unexplored) space.
