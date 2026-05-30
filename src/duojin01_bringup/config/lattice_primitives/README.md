Legacy source of truth:
`ugv_omni_2cm_spec.json`

Rich preset source of truth:
`ugv_omni_2cm_rich_spec.json`

Generated artifact used by Nav2:
`ugv_omni_2cm_rich_lattice.json`

Legacy generated artifact:
`ugv_omni_2cm_lattice.json`

Do not hand-edit the expanded lattice JSON unless you are debugging a one-off issue.
Update the source spec instead, then regenerate and validate.

Rich preset intent:

- Lateral moves are available from `micro` through `long` so the planner can express stronger omni motion when it is beneficial.
- `front_left/right` and `back_left/right` templates are still kept for obstacle-aware diagonal progress and heading-coupled sidestepping.
- Keep `config/nav2.yaml` Smac lattice penalties aligned with the active rich lattice when changing motion mix or step scales.

Commands:

```bash
python3 -m ugv_bringup.lattice_generator generate \
  --spec config/lattice_primitives/ugv_omni_2cm_spec.json \
  --output config/lattice_primitives/ugv_omni_2cm_lattice.json

python3 -m ugv_bringup.lattice_generator validate \
  --lattice config/lattice_primitives/ugv_omni_2cm_lattice.json \
  --spec config/lattice_primitives/ugv_omni_2cm_spec.json

python3 -m ugv_bringup.lattice_generator bootstrap-spec \
  --lattice config/lattice_primitives/ugv_omni_2cm_lattice.json \
  --output config/lattice_primitives/ugv_omni_2cm_spec.json

python3 -m ugv_bringup.omni_lattice_preset \
  --nav2-config config/nav2.yaml \
  --lattice-config config/lattice_primitives/ugv_omni_2cm_config.json \
  --base-lattice config/lattice_primitives/ugv_omni_2cm_lattice.json \
  --output-spec config/lattice_primitives/ugv_omni_2cm_rich_spec.json \
  --output-lattice config/lattice_primitives/ugv_omni_2cm_rich_lattice.json \
  --validate
```

Template modes:

- `pose_sequence`: stores a canonical local pose sequence and expands it across heading groups.
- `segment_sequence`: stores reusable geometric segments such as `translate`, `rotate`, and `arc`.
